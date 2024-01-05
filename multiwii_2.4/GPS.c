
#ifdef STM32F10X_MD
#include "stm32f10x.h"
#include "delay.h"
#else
#include "gd32f3x0.h"
#endif
#include "config.h"
#include "def.h"
#include "types.h"
#include "GPS.h"
#include "Serial.h"
#include "Sensors.h"
#include "MultiWii.h"
#include "EEPROM.h"
#include <math.h>
#include "timer.h"
#include <ctype.h>

#if GPS
static int16_t Current_Heading = 0; // Store current bearing
extern int16_t rcData[RC_CHANS];    // stm32 add
static int32_t GPS_prev_R[2];       // 过航点记录，用于计算半径
user_mission_dorp_ user_mission_dorp;
// Function prototypes for other GPS functions
// These perhaps could go to the gps.h file, however these are local to the gps.cpp
static void GPS_bearing(int32_t *lat1, int32_t *lon1, int32_t *lat2, int32_t *lon2, int32_t *bearing);
static void GPS_distance_cm(int32_t *lat1, int32_t *lon1, int32_t *lat2, int32_t *lon2, uint32_t *dist);
static void GPS_calc_velocity(void);
static void GPS_calc_location_error(int32_t *target_lat, int32_t *target_lng, int32_t *gps_lat, int32_t *gps_lng);
static void GPS_calc_poshold(void);
static uint16_t GPS_calc_desired_speed(uint16_t max_speed, bool _slow);
static void GPS_calc_nav_rate(uint16_t max_speed);
int32_t wrap_18000(int32_t ang);
static bool check_missed_wp(void);
void GPS_calc_longitude_scaling(int32_t lat);
static void GPS_update_crosstrack(void);
int32_t wrap_36000(int32_t ang);

// 在此处添加固定翼程序
void clearNav(void);
#if defined(FIXEDWING)
float NaverrorI, AlterrorI;
int16_t NaverrorP = 0;
static int16_t lastAltDiff, lastNavDiff, SpeedBoost;
static int16_t AltHist[GPS_UPD_HZ];    // shift register
static int16_t NavDif[GPS_UPD_HZ];     // shift register
static uint16_t GPS_NAV_speed_set = 0; // 导航速度设定
// This is the angle from the copter to the "next_WP" location
// Reinseerted for FIXEDWING from V2.3
int32_t nav_bearing;
////////////////////////////
////////////////////////////
// Leadig filter - TODO: rewrite to normal C instead of C++

// Set up gps lag
#if defined(UBLOX) || defined(MTK_BINARY19)
#define GPS_LAG 0.5f // UBLOX GPS has a smaller lag than MTK and other
#else
#define GPS_LAG 1.0f // We assumes that MTK GPS has a 1 sec lag
#endif

static int32_t GPS_coord_lead[2]; // Lead filtered gps coordinates

static int32_t get_position(uint8_t w, int32_t pos, int16_t vel, float lag_in_seconds)
{
    if (w == 1)
    {
        static int16_t _last_velocity_1;
        int16_t accel_contribution = (vel - _last_velocity_1) * lag_in_seconds * lag_in_seconds;
        int16_t vel_contribution = vel * lag_in_seconds;

        // store velocity for next iteration
        _last_velocity_1 = vel;

        return pos + vel_contribution + accel_contribution;
    }
    else
    {
        static int16_t _last_velocity_2;
        int16_t accel_contribution = (vel - _last_velocity_2) * lag_in_seconds * lag_in_seconds;
        int16_t vel_contribution = vel * lag_in_seconds;

        // store velocity for next iteration
        _last_velocity_2 = vel;

        return pos + vel_contribution + accel_contribution;
    }
}

typedef struct PID_PARAM_
{
    float kP;
    float kI;
    float kD;
    float Imax;
} PID_PARAM;

PID_PARAM posholdPID_PARAM;
PID_PARAM poshold_ratePID_PARAM;
PID_PARAM navPID_PARAM;
PID_PARAM altPID_PARAM;
typedef struct PID_
{
    float integrator;     // integrator value
    int32_t last_input;   // last input for derivative
    float lastderivative; // last derivative for low-pass filter
    float output;
    float derivative;
} PID;
PID posholdPID[2];
PID poshold_ratePID[2];
PID navPID[2];

int32_t get_P(int32_t error, struct PID_PARAM_ *pid)
{
    return (float)error * pid->kP;
}

int32_t get_I(int32_t error, float *dt, struct PID_ *pid, struct PID_PARAM_ *pid_param)
{
    pid->integrator += ((float)error * pid_param->kI) * *dt;
    pid->integrator = constrain(pid->integrator, -pid_param->Imax, pid_param->Imax);
    return pid->integrator;
}

int32_t get_D(int32_t input, float *dt, struct PID_ *pid, struct PID_PARAM_ *pid_param)
{ // dt in milliseconds
    float filter;
    pid->derivative = (input - pid->last_input) / *dt;

    /// Low pass filter cut frequency for derivative calculation.
    filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";
    // Examples for _filter:
    // f_cut = 10 Hz -> _filter = 15.9155e-3
    // f_cut = 15 Hz -> _filter = 10.6103e-3
    // f_cut = 20 Hz -> _filter =  7.9577e-3
    // f_cut = 25 Hz -> _filter =  6.3662e-3
    // f_cut = 30 Hz -> _filter =  5.3052e-3

    // discrete low pass filter, cuts out the
    // high frequency noise that can drive the controller crazy
    pid->derivative = pid->lastderivative + (*dt / (filter + *dt)) * (pid->derivative - pid->lastderivative);
    // update state
    pid->last_input = input;
    pid->lastderivative = pid->derivative;
    // add in derivative component
    return pid_param->kD * pid->derivative;
}

void reset_PID(struct PID_ *pid)
{
    pid->integrator = 0;
    pid->last_input = 0;
    pid->lastderivative = 0;
}

#define _X 1
#define _Y 0

#define RADX100 0.000174532925f

uint8_t land_detect; // Detect land (extern)
static uint32_t land_settle_timer;
uint8_t GPS_Frame; // a valid GPS_Frame was detected, and data is ready for nav computation

static float dTnav; // Delta Time in milliseconds for navigation computations, updated with every good GPS read
int16_t gpsActualSpeed[2] = {0, 0};
static int16_t actual_speed[2] = {0, 0};
static float GPS_scaleLonDown; // this is used to offset the shrinking longitude as we go towards the poles

// The difference between the desired rate of travel and the actual rate of travel
// updated after GPS read - 5-10hz
static int16_t rate_error[2];

int32_t gpsDistanceToHome[2];
#ifdef INS_PH_NAV_ON
int32_t positionToHold[2];
#else
int32_t gpsPositionError[2];
#endif
static int32_t error[2]; // 经纬度之差并放入error[LON],error[LAT]

static int32_t GPS_WP[2];               // Currently used WP
static int32_t GPS_FROM[2];             // the pervious waypoint for precise track following
int32_t target_bearing;                 // This is the angle from the copter to the "next_WP" location in degrees * 100
static int32_t original_target_bearing; // deg * 100, The original angle to the next_WP when the next_WP was set, Also used to check when we pass a WP
static int16_t crosstrack_error;        // The amount of angle correction applied to target_bearing to bring the copter back on its optimum path
uint32_t wp_distance;                   // distance between plane and next_WP in cm
static uint16_t waypoint_speed_gov;     // used for slow speed wind up when start navigation;

////////////////////////////////////////////////////////////////////////////////////
// moving average filter variables
//

#define GPS_FILTER_VECTOR_LENGTH 5

static uint8_t GPS_filter_index = 0;
static int32_t GPS_filter[2][GPS_FILTER_VECTOR_LENGTH];
static int32_t GPS_filter_sum[2];
static int32_t GPS_read[2];
static int32_t GPS_filtered[2];
static int32_t GPS_degree[2]; // the lat lon degree without any decimals (lat/10 000 000)
static uint16_t fraction3[2];

static int16_t nav_takeoff_bearing; // saves the bearing at takeof (1deg = 1) used to rotate to takeoff direction when arrives at home
static uint8_t NAV_flag = 0;

// Main navigation processor and state engine
//  TODO: add proceesing states to ease processing burden
uint8_t GPS_Compute(void)
{
    unsigned char axis;
    uint32_t dist; // temp variable to store dist to copter
    int32_t dir;   // temp variable to store dir to copter
    static uint32_t nav_loopTimer;
    static uint32_t timeMax;
    // Check for stalled GPS, if no frames seen for 1.2sec then consider it LOST
    if (f.GPS_FIX)
    {
        // timeMax2=millis();
        if (millis() - timeMax > 5000) // 5s超时GPS
            f.GPS_FIX = 0;             // GPS超时
    }
    //                    f.GPS_FIX = 0;
    //                    GPS_numSat = 0;
    //                    GPS_LED_OFF;
    //                }
    // check that we have a valid frame, if not then return immediatly
    if (GPS_Frame == 0)
        return 0;
    else
        GPS_Frame = 0;
    NAV_flag = 1;       // 数据帧更新标志
    timeMax = millis(); // 更新数据
    // check home position and set it if it was not set
    if (f.GPS_FIX && GPS_numSat >= 5)
    {
#if !defined(DONT_RESET_HOME_AT_ARM)
        if (!f.ARMED)
        {
            f.GPS_FIX_HOME = 0;
        }
#endif
        if (!f.GPS_FIX_HOME && f.ARMED)
        {
            GPS_reset_home_position();
        }
        // Apply moving average filter to GPS data
        if (GPS_conf.filtering)
        {
            GPS_filter_index = (GPS_filter_index + 1) % GPS_FILTER_VECTOR_LENGTH;
            for (axis = 0; axis < 2; axis++)
            {
                GPS_read[axis] = GPS_coord[axis];             // latest unfiltered data is in GPS_latitude and GPS_longitude
                GPS_degree[axis] = GPS_read[axis] / 10000000; // get the degree to assure the sum fits to the int32_t

                // How close we are to a degree line ? its the first three digits from the fractions of degree
                // later we use it to Check if we are close to a degree line, if yes, disable averaging,
                fraction3[axis] = (GPS_read[axis] - GPS_degree[axis] * 10000000) / 10000;

                GPS_filter_sum[axis] -= GPS_filter[axis][GPS_filter_index];
                GPS_filter[axis][GPS_filter_index] = GPS_read[axis] - (GPS_degree[axis] * 10000000);
                GPS_filter_sum[axis] += GPS_filter[axis][GPS_filter_index];
                GPS_filtered[axis] = GPS_filter_sum[axis] / GPS_FILTER_VECTOR_LENGTH + (GPS_degree[axis] * 10000000);
                if (NAV_state == NAV_STATE_HOLD_INFINIT || NAV_state == NAV_STATE_HOLD_TIMED)
                { // we use gps averaging only in poshold mode...
                    if (fraction3[axis] > 1 && fraction3[axis] < 999)
                        GPS_coord[axis] = GPS_filtered[axis];
                }
            }
        }

        // dTnav calculation
        // Time for calculating x,y speed and navigation pids
        dTnav = (float)(millis() - nav_loopTimer) / 1000.0;
        nav_loopTimer = millis();

        // prevent runup from bad GPS
        dTnav = MIN(dTnav, 1.0);

        // calculate distance and bearings for gui and other stuff continously - From copter to home
        GPS_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_home[LAT], &GPS_home[LON], &dir);
        GPS_distance_cm(&GPS_coord[LAT], &GPS_coord[LON], &GPS_home[LAT], &GPS_home[LON], &dist);
        GPS_distanceToHome = dist / 100;
        GPS_directionToHome = dir / 100;

        if (!f.GPS_FIX_HOME)
        { // If we don't have home set, do not display anything
            GPS_distanceToHome = 0;
            GPS_directionToHome = 0;
        }

        // Check fence setting and execute RTH if neccessary
        // TODO: autolanding
        if ((GPS_conf.fence > 0) && (GPS_conf.fence < GPS_distanceToHome) && (f.GPS_mode != GPS_MODE_RTH))
        {
            rcOptions[BOXGPSHOME] = TRUE; // 应该加上
            init_RTH();
        }

        // calculate the current velocity based on gps coordinates continously to get a valid speed at the moment when we start navigating
        GPS_calc_velocity();

        // Navigation state engine
        if (f.GPS_mode != GPS_MODE_NONE)
        { // ok we are navigating ###0002
            // do gps nav calculations here, these are common for nav and poshold
            GPS_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_WP[LAT], &GPS_WP[LON], &target_bearing);
            if (GPS_conf.lead_filter)
            {
                GPS_distance_cm(&GPS_coord[LAT], &GPS_coord[LON], &GPS_WP[LAT], &GPS_WP[LON], &wp_distance);
                // GPS_distance_cm(&GPS_coord_lead[LAT],&GPS_coord_lead[LON],&GPS_WP[LAT],&GPS_WP[LON],&wp_distance);
                // GPS_calc_location_error(&GPS_WP[LAT],&GPS_WP[LON],&GPS_coord_lead[LAT],&GPS_coord_lead[LON]);  //固定翼无需此句！！！
            }
            else
            {
                GPS_distance_cm(&GPS_coord[LAT], &GPS_coord[LON], &GPS_WP[LAT], &GPS_WP[LON], &wp_distance);
                // GPS_calc_location_error(&GPS_WP[LAT],&GPS_WP[LON],&GPS_coord[LAT],&GPS_coord[LON]);  //固定翼无需此句！！！
            }

            // Adjust altitude
            // if we are holding position and reached target altitude, then ignore altitude nav, and let the user trim alt
            if (!((NAV_state == NAV_STATE_HOLD_INFINIT) && (alt_change_flag == REACHED_ALT)))
            {
                if (!f.LAND_IN_PROGRESS)
                {
                    alt_to_hold = get_new_altitude(); //==============================================================================================================
                    AltHold = alt_to_hold;
                }
            }

            int16_t speed = 0; // Desired navigation speed
            uint16_t dyn_wp_radius = GPS_conf.wp_radius;
#if defined(FIXEDWING) // Adjust radius based on speed
            dyn_wp_radius = GPS_speed / GPS_UPD_HZ;
            dyn_wp_radius = MAX(dyn_wp_radius, GPS_conf.wp_radius);
#endif
            switch (NAV_state) // Navigation state machine
            {
            case NAV_STATE_NONE: // Just for clarity, do nothing when nav_state is none
                break;
#if !defined(SLIM_WING) // Save space for small proc. PatrikE
            /***************************************************************/
            case NAV_STATE_LAND_START:
                GPS_calc_poshold(); // Land in position hold
                land_settle_timer = millis();
                NAV_state = NAV_STATE_LAND_SETTLE;
                break;

            case NAV_STATE_LAND_SETTLE:
                GPS_calc_poshold();
                if (millis() - land_settle_timer > 5000)
                    NAV_state = NAV_STATE_LAND_START_DESCENT;
                break;

            case NAV_STATE_LAND_START_DESCENT:
                GPS_calc_poshold();     // Land in position hold
                f.THROTTLE_IGNORED = 1; // Ignore Throtte stick input
                f.GPS_BARO_MODE = 1;    // Take control of BARO mode
                land_detect = 0;        // Reset land detector
                f.LAND_COMPLETED = 0;
                f.LAND_IN_PROGRESS = 1; // Flag land process
                NAV_state = NAV_STATE_LAND_IN_PROGRESS;
                break;

            case NAV_STATE_LAND_IN_PROGRESS:
                GPS_calc_poshold();
                check_land(); // Call land detector
                if (f.LAND_COMPLETED)
                {
                    nav_timer_stop = millis() + 5000;
                    NAV_state = NAV_STATE_LANDED;
                }
                break;

            case NAV_STATE_LANDED:
                // Disarm if THROTTLE stick is at minimum or 5sec past after land detected
                if (rcData[THROTTLE] < MINCHECK || nav_timer_stop <= millis())
                { // Throttle at minimum or 5sec passed.
                    go_disarm();
                    f.OK_TO_ARM = 0;            // Prevent rearming
                    NAV_state = NAV_STATE_NONE; // Disable position holding.... prevent flippover
                    f.GPS_BARO_MODE = 0;
                    f.LAND_COMPLETED = 0;
                    f.LAND_IN_PROGRESS = 0;
                    land_detect = 0;
                    f.THROTTLE_IGNORED = 0;
                    GPS_reset_nav();
                }
                break;
            case NAV_STATE_RTH_START:
                if ((alt_change_flag == REACHED_ALT) || (!GPS_conf.wait_for_rth_alt))
                {                                                                                      // Wait until we reach RTH altitude
                    GPS_set_next_wp(&GPS_home[LAT], &GPS_home[LON], &GPS_coord[LAT], &GPS_coord[LON]); // If we reached then change mode and start RTH
                    NAV_state = NAV_STATE_RTH_ENROUTE;
                    NAV_error = NAV_ERROR_NONE;
                }
                else
                {
                    GPS_calc_poshold(); // hold position till we reach RTH alt
                    NAV_error = NAV_ERROR_WAIT_FOR_RTH_ALT;
                }
                break;

#endif // End of Not for FixedWing
            /***************************************************************/
            case NAV_STATE_HOLD_INFINIT: // Constant position hold, no timer. Only an rcOption change can exit from this
                GPS_calc_poshold();
                break; // 初保持定点盘旋=================================================
// #ifndef SLIM_WING
//                 GPS_calc_poshold();
//                 break;
// #endif
#if !defined(SLIM_WING) // Save space for small proc. PatrikE
            /***************************************************************/
            case NAV_STATE_HOLD_TIMED:
                if (nav_timer_stop == 0)
                {                              // We are start a timed poshold
                    nav_timer_stop = millis(); // Set when we will continue
                }
                else if (nav_hold_time == 0)
                { // did we reach our time limit ?
                    if (mission_step.flag != MISSION_FLAG_END)
                    {
                        NAV_state = NAV_STATE_PROCESS_NEXT; // if yes then process next mission step
                    }
                    NAV_error = NAV_ERROR_TIMEWAIT;
                }
                else
                {
                    if (nav_loopTimer - nav_timer_stop > 1000)
                    {
                        nav_timer_stop = nav_loopTimer;
                        nav_hold_time--;
                    }
                }
                D_U[3] = nav_hold_time;
                GPS_calc_poshold(); // BTW hold position till next command
                break;
            case NAV_STATE_RTH_ENROUTE: // Doing RTH navigation  //=================================================
                speed = GPS_calc_desired_speed(GPS_conf.nav_speed_max, GPS_conf.slow_nav);
                GPS_calc_nav_rate(speed);
                GPS_adjust_heading();
                if ((wp_distance <= dyn_wp_radius) || check_missed_wp())
                { // if yes switch to poshold mode
                    if (mission_step.parameter1 == 0)
                        NAV_state = NAV_STATE_HOLD_INFINIT;
                    else
                        NAV_state = NAV_STATE_LAND_START; // if parameter 1 in RTH step is non 0 then land at home
                    if (GPS_conf.nav_rth_takeoff_heading)
                    {
                        magHold = nav_takeoff_bearing;
                    }
                }
                break;
#endif
            case NAV_STATE_WP_ENROUTE: // 导航至指定航点，后应该自动切到定点状态，（返航模式）
#if !defined(SLIM_WING)
                speed = GPS_calc_desired_speed(GPS_conf.nav_speed_max, GPS_conf.slow_nav);
                GPS_calc_nav_rate(speed);
                GPS_adjust_heading();
#endif
                // if ((wp_distance <= GPS_conf.wp_radius) || check_missed_wp()) {

                if (user_mission_dorp.drop_wp == mission_step.number)
                { // 预测着弹点
                    // h=1/2gt^2
                    // SET_ALT_ROI 对地高度
                    // InvSqrt 开方 返回浮点
                    // G=10 25m 约2.25s GPS_speed
                    // s=vt
                    float dorp_time_ = 0;
                    int dorp_distance = 0;
                    dorp_time_ = sqrt(2 * (SET_ALT_ROI) / Gravitational__ / 100) + (float)(GPS_conf.dorp_delay_ms) / 100;
                    if (dorp_time_ < 0)
                        dorp_time_ = 0;
                    dorp_distance = dorp_time_ * GPS_speed;
                    if (wp_distance < dorp_distance) // 预测投弹
                    {
                        user_mission_dorp.drop_status = 3;
                    }
                }
                //								debug[0] = SET_ALT_ROI;
                //								debug[1] = GPS_speed;
                //								debug[2] = servo[0]+servo[1];
                //								debug[3] = user_mission_dorp.drop_status;
                if ((wp_distance <= dyn_wp_radius) || check_missed_wp())
                { // This decides what happen when we reached the WP coordinates
                    if (user_mission_dorp.drop_wp)
                    {

                        if (user_mission_dorp.drop_wp - 1 == mission_step.number)
                        {
                            // 到达转弯航点
                            user_mission_dorp.drop_status = 2;
                        }
                        else if (user_mission_dorp.drop_wp == mission_step.number)
                        {
                            static uint8_t conut_miss_wp = 0;
                            if (check_missed_wp() && user_mission_dorp.drop_status != 3)
                            {
                                // 如果错过目标航点再飞一趟

                                conut_miss_wp++;

                                if (conut_miss_wp > 2)                 // 错过航点次数开始投放
                                    user_mission_dorp.drop_status = 3; // 投放标志变量
                                else
                                {
                                    next_step = user_mission_dorp.drop_wp - 1;
                                    NAV_state = NAV_STATE_PROCESS_NEXT;
                                    mission_step.flag = 0;
                                    user_mission_dorp.drop_status = 1;
                                    GPS_prev[LAT] = GPS_coord[LAT]; // 重设航点
                                    GPS_prev[LON] = GPS_coord[LON];
                                    // conut_miss_wp=0;
                                }
                            }
                            else
                            {
                                // 到达航点
                                user_mission_dorp.drop_status = 3; // 投放标志变量
                            }
                            if (user_mission_dorp.drop_status == 3)
                                conut_miss_wp = 0;
                        }
                    }

                    //===暂时屏蔽其它代码，直接设置NAV_state = NAV_STATE_HOLD_INFINIT
                    if (f.GPS_mode == GPS_MODE_RTH)
                    {
                        NAV_state = NAV_STATE_HOLD_INFINIT; // 固定翼测试时，直接设置成定点盘旋！！！多轴不能这样，应该降落！！======================================
                    }
                    else
                    { // mission_step中的action 和flag的都值是什么，应该查一下资料==========================
                        if (mission_step.action == MISSION_LAND)
                        {                                     // Autoland
                            NAV_state = NAV_STATE_LAND_START; // Start landing
                            // set_new_altitude(alt.EstAlt);                                             //Stop any altitude changes
                            set_new_altitude(SET_ALT_ROI); //==单位米===
                        }
                        else if (mission_step.flag == MISSION_FLAG_END)
                        { // If this was the last mission step (flag set by the mission planner), then switch to poshold
                            // NAV_state = NAV_STATE_HOLD_INFINIT; //原来的，多轴用
                            // NAV_error = NAV_ERROR_FINISH;
                            init_RTH(); // 后加，固定翼直接返航！！！
                        }
                        else if (mission_step.action == MISSION_HOLD_UNLIM)
                        { // If mission_step was POSHOLD_UNLIM and we reached the position then switch to poshold unlimited
                            NAV_state = NAV_STATE_HOLD_INFINIT;
                            NAV_error = NAV_ERROR_FINISH;
                        }
                        else if (mission_step.action == MISSION_HOLD_TIME)
                        { // If mission_step was a timed poshold then initiate timed poshold
                            nav_hold_time = mission_step.parameter1;
                            nav_timer_stop = 0; // This indicates that we are starting a timed poshold
                            NAV_state = NAV_STATE_HOLD_TIMED;
                        }
                        else
                        {
                            NAV_state = NAV_STATE_PROCESS_NEXT; // Otherwise process next step
                        }
                    }
                }
                break;

#if !defined(SLIM_WING) // Save space for small proc. PatrikE
                        /***************************************************************/
// NAV_STATE_DO_JUMP: if not needed
#endif
            case NAV_STATE_DO_JUMP:
                if (jump_times < 0)
                { // Jump unconditionally (supposed to be -1) -10 should not be here
                    next_step = mission_step.parameter1;
                    NAV_state = NAV_STATE_PROCESS_NEXT;
                }
                if (jump_times == 0)
                {
                    jump_times = -10; // reset jump counter
                    if (mission_step.flag == MISSION_FLAG_END)
                    { // If this was the last mission step (flag set by the mission planner), then switch to poshold
                        NAV_state = NAV_STATE_HOLD_INFINIT;
                        NAV_error = NAV_ERROR_FINISH;
                    }
                    else
                        NAV_state = NAV_STATE_PROCESS_NEXT;
                }

                if (jump_times > 0)
                { // if zero not reached do a jump
                    next_step = mission_step.parameter1;
                    NAV_state = NAV_STATE_PROCESS_NEXT;
                    jump_times--;
                }
                break;
            case NAV_STATE_PROCESS_NEXT: // Processing next mission step
                NAV_error = NAV_ERROR_NONE;
                if (!recallWP(next_step))
                {
                    abort_mission(NAV_ERROR_WP_CRC);
                }
                else
                {
                    clearNav(); // 清理上一个航点的PID计算结果
                    switch (mission_step.action)
                    {
                    // Waypoiny and hold commands all starts with an enroute status it includes the LAND command too
                    case MISSION_WAYPOINT:
                    case MISSION_HOLD_TIME:
                    case MISSION_HOLD_UNLIM:
                    case MISSION_LAND:
                        GPS_NAV_speed_set = mission_step.parameter3;
                        set_new_altitude(mission_step.altitude);
                        GPS_set_next_wp(&mission_step.pos[LAT], &mission_step.pos[LON], &GPS_prev[LAT], &GPS_prev[LON]);
                        if ((wp_distance / 100) >= GPS_conf.safe_wp_distance)
                            abort_mission(NAV_ERROR_TOOFAR);
                        else
                            NAV_state = NAV_STATE_WP_ENROUTE;
                        //                        GPS_prev_R[LAT]=GPS_prev[LAT];
                        //                        GPS_prev_R[LON]=GPS_prev[LON];
                        GPS_prev[LAT] = mission_step.pos[LAT]; // Save wp coordinates for precise route calc
                        GPS_prev[LON] = mission_step.pos[LON];
                        break;
                    case MISSION_RTH:
                        f.GPS_head_set = 0;
                        if (GPS_conf.rth_altitude == 0 && mission_step.altitude == 0) // if config and mission_step alt is zero
                            set_new_altitude(SET_ALT_ROI);                            // RTH returns at the actual altitude
                        else
                        {
                            uint32_t rth_alt;
                            if (mission_step.altitude == 0)
                                rth_alt = GPS_conf.rth_altitude * 100; // altitude in mission step has priority
                            else
                                rth_alt = mission_step.altitude;

                            if (SET_ALT_ROI < rth_alt)
                                set_new_altitude(rth_alt); // BUt only if we are below it.
                            else
                                set_new_altitude(SET_ALT_ROI);
                        }
                        NAV_state = NAV_STATE_RTH_START;
                        break;
                    case MISSION_JUMP:
                        if (jump_times == -10)
                            jump_times = mission_step.parameter2;
                        if (mission_step.parameter1 > 0 && mission_step.parameter1 < mission_step.number)
                            NAV_state = NAV_STATE_DO_JUMP;
                        else // Error situation, invalid jump target
                            abort_mission(NAV_ERROR_INVALID_JUMP);
                        break;
                        /***************************************************************/
#if !defined(SLIM_WING) // Save space for small proc. PatrikE
                    case MISSION_SET_POI:
                        // static uint8_t flag=0;
                        // Note: Flying around Interest Points
                        // 注意，此处修改为环绕目标点飞行，坐标为圆心，参数2为半径（cm),参数1为环绕时间（s),参数3转速（1000-2000）
                        //                        GPS_poi[LAT] = mission_step.pos[LAT];
                        //                        GPS_poi[LON] = mission_step.pos[LON];
                        //                        f.GPS_head_set = 1;
                        GPS_NAV_speed_set = mission_step.parameter3;
                        nav_hold_time = mission_step.parameter1;
                        set_new_altitude(mission_step.altitude);
                        GPS_set_next_wp(&mission_step.pos[LAT], &mission_step.pos[LON], &GPS_prev[LAT], &GPS_prev[LON]);
                        NAV_state = NAV_STATE_HOLD_TIMED;
                        break;
                    case MISSION_SET_HEADING:
                        GPS_poi[LAT] = 0;
                        GPS_poi[LON] = 0; // zeroing this out clears the possible pervious set_poi
                        if (mission_step.parameter1 < 0)
                            f.GPS_head_set = 0;
                        else
                        {
                            f.GPS_head_set = 1;
                            GPS_directionToPoi = mission_step.parameter1;
                        }
                        break;
#endif
                    default: // if we got an unknown action code abort mission and hold position
                        abort_mission(NAV_ERROR_INVALID_DATA);
                        break;
                    }
                    next_step++; // Prepare for the next step
                }
                break;
            } // switch end
        }     // end of gps calcs ###0002
    }

    return 1;
} // End of GPS_compute

// Abort current mission with the given error code (switch to poshold_infinit)
void abort_mission(unsigned char error_code)
{
    GPS_set_next_wp(&GPS_coord[LAT], &GPS_coord[LON], &GPS_coord[LAT], &GPS_coord[LON]);
    NAV_error = error_code;
    NAV_state = NAV_STATE_NONE;
}

// Adjusting heading according to settings - MAG mode must be enabled
void GPS_adjust_heading()
{
    // TODO: Add slow windup for large heading change
    // This controls the heading
    if (f.GPS_head_set)
    { // We have seen a SET_POI or a SET_HEADING command
        if (GPS_poi[LAT] == 0)
            magHold = wrap_18000((GPS_directionToPoi * 100)) / 100;
        else
        {
            GPS_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_poi[LAT], &GPS_poi[LON], &GPS_directionToPoi);
            GPS_distance_cm(&GPS_coord[LAT], &GPS_coord[LON], &GPS_poi[LAT], &GPS_poi[LON], &wp_distance);
            magHold = GPS_directionToPoi / 100;
        }
    }
    else
    { // heading controlled by the standard defines
        if (GPS_conf.nav_controls_heading)
        {
            if (GPS_conf.nav_tail_first)
            {
                magHold = wrap_18000(target_bearing - 18000) / 100;
            }
            else
            {
                magHold = wrap_18000(target_bearing) / 100;
            }
        }
    }
}

#define LAND_DETECT_THRESHOLD 40 // Counts of land situation
#define BAROPIDMIN -180          // BaroPID reach this if we landed.....

// Check if we landed or not
void check_land()
{
    // detect whether we have landed by watching for low climb rate and throttle control
    if ((ABS(alt.vario) < 20) && (BaroPID < BAROPIDMIN))
    {
        if (!f.LAND_COMPLETED)
        {
            if (land_detect < LAND_DETECT_THRESHOLD)
            {
                land_detect++;
            }
            else
            {
                f.LAND_COMPLETED = 1;
                land_detect = 0;
            }
        }
    }
    else
    {
        // we've detected movement up or down so reset land_detector
        land_detect = 0;
        if (f.LAND_COMPLETED)
        {
            f.LAND_COMPLETED = 0;
        }
    }
}

int32_t get_altitude_error()
{
    return alt_to_hold - alt.EstAlt;
}

void clear_new_altitude()
{
    alt_change_flag = REACHED_ALT;
}
void force_new_altitude(int32_t _new_alt)
{
    alt_to_hold = _new_alt;
    target_altitude = _new_alt;
    alt_change_flag = REACHED_ALT;
}

void set_new_altitude(int32_t _new_alt)
{
    // Limit maximum altitude command
    // if(_new_alt > GPS_conf.nav_max_altitude*100) _new_alt = GPS_conf.nav_max_altitude * 100;   //===条件总是有问题，暂时屏蔽！！！=========================
    // if(_new_alt == alt.EstAlt || f.Fixed_Wing_Nav){
    force_new_altitude(_new_alt);
    return;
    //}
    // We start at the current location altitude and gradually change alt
    alt_to_hold = alt.EstAlt;
    // for calculating the delta time
    alt_change_timer = millis();
    // save the target altitude
    target_altitude = _new_alt;
    // reset our altitude integrator
    alt_change = 0;
    // save the original altitude
    original_altitude = alt.EstAlt;
    // to decide if we have reached the target altitude
    if (target_altitude > original_altitude)
    {
        // we are below, going up
        alt_change_flag = ASCENDING;
    }
    else if (target_altitude < original_altitude)
    {
        // we are above, going down
        alt_change_flag = DESCENDING;
    }
    else
    {
        // No Change
        alt_change_flag = REACHED_ALT;
    }
}

int32_t get_new_altitude()
{ // 估算出本次时刻应该到达的高度
    int32_t diff;
    int8_t _scale;
    int32_t change;
    // returns a new altitude which feeded into the alt.hold controller
    if (alt_change_flag == ASCENDING)
    {
        // we are below, going up
        if (alt.EstAlt >= target_altitude)
            alt_change_flag = REACHED_ALT;
        // we shouldn't command past our target
        if (alt_to_hold >= target_altitude)
            return target_altitude;
    }
    else if (alt_change_flag == DESCENDING)
    {
        // we are above, going down
        if (alt.EstAlt <= target_altitude)
            alt_change_flag = REACHED_ALT;
        // we shouldn't command past our target
        if (alt_to_hold <= target_altitude)
            return target_altitude;
    }
    // if we have reached our target altitude, return the target alt
    if (alt_change_flag == REACHED_ALT)
        return target_altitude;

    diff = ABS(alt_to_hold - target_altitude);
    // scale is how we generate a desired rate from the elapsed time
    // a smaller scale means faster rates
    _scale = 4;

    if (alt_to_hold < target_altitude)
    {
        // we are below the target alt
        if (diff < 200)
            _scale = 4;
        else
            _scale = 3;
    }
    else
    {
        // we are above the target, going down
        if (diff < 400)
            _scale = 5; // Slow down if only 4meters above
        if (diff < 100)
            _scale = 6; // Slow down further if within 1meter
    }

    // we use the elapsed time as our altitude offset
    // 1000 = 1 sec
    // 1000 >> 4 = 64cm/s descent by default
    change = (millis() - alt_change_timer) >> _scale;

    if (alt_change_flag == ASCENDING)
    {
        alt_change += change; // 在set_new_altitude()函数中被重置为0
    }
    else
    {
        alt_change -= change;
    }
    // for generating delta time
    alt_change_timer = millis();

    return original_altitude + alt_change;
}

////////////////////////////////////////////////////////////////////////////////////
// PID based GPS navigation functions
// Author : EOSBandi
// Based on code and ideas from the Arducopter team: Jason Short,Randy Mackay, Pat Hickey, Jose Julio, Jani Hirvinen
// Andrew Tridgell, Justin Beech, Adam Rivera, Jean-Louis Naudin, Roberto Navoni

// original constraint does not work with variables
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high)
{
    return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}
////////////////////////////////////////////////////////////////////////////////////
// this is used to offset the shrinking longitude as we go towards the poles
// It's ok to calculate this once per waypoint setting, since it changes a little within the reach of a multicopter
//
void GPS_calc_longitude_scaling(int32_t lat)
{
    GPS_scaleLonDown = cos(lat * 1.0e-7f * 0.01745329251f);
}

////////////////////////////////////////////////////////////////////////////////////
// Sets the waypoint to navigate, reset neccessary variables and calculate initial values
//
void GPS_set_next_wp(int32_t *lat_to, int32_t *lon_to, int32_t *lat_from, int32_t *lon_from)
{
    GPS_WP[LAT] = *lat_to;
    GPS_WP[LON] = *lon_to;

    GPS_FROM[LAT] = *lat_from;
    GPS_FROM[LON] = *lon_from;

    GPS_calc_longitude_scaling(*lat_to);

#ifdef FIXEDWING
    // TODO build a separate function to call.
    if (f.CRUISE_MODE)
    {                         // PatrikE CruiseMode version
#define GEO_SKALEFACT 89.832f // Scale to match meters
        int32_t hh = att.heading;
        if (hh > 180)
            hh -= 360;

        float scaler = (GEO_SKALEFACT / GPS_scaleLonDown) * GPS_conf.safe_wp_distance; //========直线飞行VS绕点飞行=========================================
        float wp_lat_diff = cos(hh * 0.0174532925f);                                   // PI/180
        float wp_lon_diff = sin(hh * 0.0174532925f) * GPS_scaleLonDown;
        GPS_WP[LAT] += wp_lat_diff * scaler; //==如果距离远，可能不太精确，需要GPS_calc_longitude_scaling( GPS_WP[LAT]);以获得较精确导航=====================
        GPS_WP[LON] += wp_lon_diff * scaler;
    }
#endif

    GPS_bearing(&GPS_FROM[LAT], &GPS_FROM[LON], &GPS_WP[LAT], &GPS_WP[LON], &target_bearing);
    GPS_distance_cm(&GPS_FROM[LAT], &GPS_FROM[LON], &GPS_WP[LAT], &GPS_WP[LON], &wp_distance);

    // GPS_calc_location_error(&GPS_WP[LAT],&GPS_WP[LON],&GPS_FROM[LAT],&GPS_FROM[LON]); //固定翼此句不用================
    // waypoint_speed_gov = GPS_conf.nav_speed_min;
    original_target_bearing = target_bearing;
    clearNav();
}

////////////////////////////////////////////////////////////////////////////////////
// Check if we missed the destination somehow
//
static bool check_missed_wp(void)
{
    int32_t temp;
    temp = target_bearing - original_target_bearing;
    temp = wrap_18000(temp);
    return (ABS(temp) > 10000); // we passed the waypoint by 100 degrees
}

////////////////////////////////////////////////////////////////////////////////////
// Get distance between two points in cm
// Get bearing from pos1 to pos2, returns an 1deg = 100 precision

void GPS_bearing(int32_t *lat1, int32_t *lon1, int32_t *lat2, int32_t *lon2, int32_t *bearing)
{
    int32_t off_x = *lon2 - *lon1;
    int32_t off_y = (*lat2 - *lat1) / GPS_scaleLonDown;

    *bearing = 9000 + atan2((float)-off_y, off_x) * 5729.57795f; // Convert the output redians to 100xdeg
    if (*bearing < 0)
        *bearing += 36000;
}

void GPS_distance_cm(int32_t *lat1, int32_t *lon1, int32_t *lat2, int32_t *lon2, uint32_t *dist)
{
    float dLat = (float)(*lat2 - *lat1);                    // difference of latitude in 1/10 000 000 degrees
    float dLon = (float)(*lon2 - *lon1) * GPS_scaleLonDown; // x
    *dist = sqrt(sq(dLat) + sq(dLon)) * 1.11318845f;        //   πr/180=111.319491
}

//*******************************************************************************************************
// calc_velocity_and_filtered_position - velocity in lon and lat directions calculated from GPS position
//       and accelerometer data
// lon_speed expressed in cm/s.  positive numbers mean moving east
// lat_speed expressed in cm/s.  positive numbers when moving north
// Note: we use gps locations directly to calculate velocity instead of asking gps for velocity because
//       this is more accurate below 1.5m/s
// Note: even though the positions are projected using a lead filter, the velocities are calculated
//       from the unaltered gps locations.  We do not want noise from our lead filter affecting velocity
//*******************************************************************************************************
static void GPS_calc_velocity(void)
{
    static int16_t speed_old[2] = {0, 0};
    static int32_t last[2] = {0, 0};
    static uint8_t init = 0;

    if (init)
    {
        float tmp = 1.0 / dTnav;
        actual_speed[_X] = (float)(GPS_coord[LON] - last[LON]) * GPS_scaleLonDown * tmp;
        actual_speed[_Y] = (float)(GPS_coord[LAT] - last[LAT]) * tmp;

        // TODO: Check unrealistic speed changes and signal navigation about posibble gps signal degradation
        if (!GPS_conf.lead_filter)
        {
            actual_speed[_X] = (actual_speed[_X] + speed_old[_X]) / 2;
            actual_speed[_Y] = (actual_speed[_Y] + speed_old[_Y]) / 2;

            speed_old[_X] = actual_speed[_X];
            speed_old[_Y] = actual_speed[_Y];
        }
    }
    init = 1;

    last[LON] = GPS_coord[LON];
    last[LAT] = GPS_coord[LAT];

    if (GPS_conf.lead_filter)
    {

        // GPS_coord_lead[LON] = xLeadFilter.get_position(GPS_coord[LON], actual_speed[_X], GPS_LAG);  //推导GPS_LAG延时后的经纬度
        // GPS_coord_lead[LAT] = yLeadFilter.get_position2(GPS_coord[LAT], actual_speed[_Y], GPS_LAG);

        GPS_coord_lead[LON] = get_position(1, GPS_coord[LON], actual_speed[_X], GPS_LAG); // 推导GPS_LAG延时后的经纬度
        GPS_coord_lead[LAT] = get_position(2, GPS_coord[LAT], actual_speed[_Y], GPS_LAG);
    }
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate a location error between two gps coordinates
// Because we are using lat and lon to do our distance errors here's a quick chart:
//   100  = 1m
//  1000  = 11m    = 36 feet
//  1800  = 19.80m = 60 feet
//  3000  = 33m
// 10000  = 111m
//
static void GPS_calc_location_error(int32_t *target_lat, int32_t *target_lng, int32_t *gps_lat, int32_t *gps_lng)
{
    error[LON] = (float)(target_lng[0] - gps_lng[0]) * GPS_scaleLonDown; // X Error
    error[LAT] = target_lat[0] - gps_lat[0];                             // Y Error
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate nav_lat and nav_lon from the x and y error and the speed
//
// TODO: check that the poshold target speed constraint can be increased for snappier poshold lock
static void GPS_calc_poshold(void)
{
    int32_t d;
    int32_t target_speed;
    uint8_t axis;
    //
    GPS_update_crosstrack();
    for (axis = 0; axis < 2; axis++)
    {
        target_speed = get_P(error[axis], &posholdPID_PARAM); // calculate desired speed from lat/lon error   //speed即经纬度修正的速度!!!!  //经纬度之差并放入error[LON],error[LAT]
        target_speed = constrain(target_speed, -100, 100);    // Constrain the target speed in poshold mode to 1m/s it helps avoid runaways..
        rate_error[axis] = target_speed - actual_speed[axis]; // calc the speed error // //speed即经纬度修正的速度!!!!
        nav[axis] =
            get_P(rate_error[axis], &poshold_ratePID_PARAM);
        nav[axis] += get_I(rate_error[axis] + error[axis], &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM); // return (ratePID->integrator += ((float)error * pid_param->kI) * *dt;)  //注意dt在I和D中的用法

        d = get_D(error[axis], &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM); //(ratePID->derivative = (input - pid->last_input) / *dt)*ratePID_PARAM->KD;

        d = constrain(d, -2000, 2000);

        // get rid of noise
        if (ABS(actual_speed[axis]) < 50)
            d = 0;

        nav[axis] += d;
        // nav[axis]  = constrain(nav[axis], -NAV_BANK_MAX, NAV_BANK_MAX);
        nav[axis] = constrain_int16(nav[axis], -GPS_conf.nav_bank_max, GPS_conf.nav_bank_max); // GPS_conf.nav_bank_max = 3000;  #define NAV_BANK_MAX 3000
        navPID[axis].integrator = poshold_ratePID[axis].integrator;                            // 在计算get_I()时刚更新
    }
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate the desired nav_lat and nav_lon for distance flying such as RTH and WP
//
static void GPS_calc_nav_rate(uint16_t max_speed)
{
    float trig[2];
    int32_t target_speed[2];
    int32_t tilt;
    uint8_t axis;
    int16_t cross_speed;
    float temp;
    GPS_update_crosstrack();                                             // Meters we are off track line  //保存在crosstrack_error中
    cross_speed = crosstrack_error * (GPS_conf.crosstrack_gain / 100.0); // check is it ok ?
    cross_speed = constrain(cross_speed, -200, 200);
    cross_speed = -cross_speed;

    temp = (9000l - target_bearing) * RADX100; // temp是什么角度???
    trig[_X] = cos(temp);
    trig[_Y] = sin(temp);

    target_speed[_X] = max_speed * trig[_X] - cross_speed * trig[_Y];
    target_speed[_Y] = cross_speed * trig[_X] + max_speed * trig[_Y];

    for (axis = 0; axis < 2; axis++)
    {
        rate_error[axis] = target_speed[axis] - actual_speed[axis];
        rate_error[axis] = constrain(rate_error[axis], -1000, 1000);
        nav[axis] =
            get_P(rate_error[axis], &navPID_PARAM) + get_I(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM) + get_D(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM);

        // nav[axis] = constrain(nav[axis],-NAV_BANK_MAX,NAV_BANK_MAX);
        nav[axis] = constrain_int16(nav[axis], -GPS_conf.nav_bank_max, GPS_conf.nav_bank_max);
        poshold_ratePID[axis].integrator = navPID[axis].integrator;
    }
}

static void GPS_update_crosstrack(void)
{
    //    // Crosstrack Error
    //    // ----------------
    //    // If we are too far off or too close we don't do track following
    //    //target_bearing 目标航点与当前航点的航向角
    //    // original_target_bearing 航点指向
    //    int cross_angle_add=0;
    //    int temp_error_tb_otb=target_bearing - original_target_bearing;

    //    float temp = 0;  //弧度转换
    //    int temp_angle_line=wrap_18000(target_bearing - original_target_bearing);//机身与两航点间角度差
    //    int temp_angle_error_heading=wrap_18000(target_bearing - Current_Heading*100);//与目标航向角度差0-36000
    ////	if(temp_error_tb_otb>+180)temp_error_tb_otb-=360;
    ////	if(temp_error_tb_otb<-180)temp_error_tb_otb+=360;
    //    temp_error_tb_otb=wrap_18000(temp_error_tb_otb);
    //    temp=temp_error_tb_otb* RADX100;

    //    crosstrack_error = sin(temp) * wp_distance; // Meters we are off track line
    //    //nav_bearing = target_bearing;
    // #if defined(FIXEDWING)   // =========================================================================================
    //    cross_angle_add=0;
    //    if(abs(temp_angle_line)<4500) { //第二步，航向偏差已经很小的情况下，趋近两点间航迹，位置与航向夹角过大时直接逼近
    ////		crosstrack_error = sin(temp) * (wp_distance * GPS_conf.crosstrack_gain / 100.0 );
    ////		nav_bearing  = target_bearing+constrain(crosstrack_error, -4000, 4000);
    //        if(abs(crosstrack_error)>2000)//大于10m距离
    //        {
    //            debug[3]=2;
    //            crosstrack_error*=GPS_conf.crosstrack_gain*1.5/100.0 ;
    //        }
    //        else {
    //            crosstrack_error*=GPS_conf.crosstrack_gain/100.0;
    //        }

    //        nav_bearing=target_bearing+ constrain(crosstrack_error, -4000, 4000);
    //    } else {

    //        nav_bearing = target_bearing;
    //    }

    int16_t POI_Diff = 0;
    static uint8_t POI_flag = 0;
    if (mission_step.action == MISSION_SET_POI && f.GPS_mode == GPS_MODE_NAV) // 兴趣点环绕
    {
        /*2019/8/29添加环绕飞行*/
        if (!POI_flag) // 初次进入，未到达半径范围
        {
            nav_bearing = target_bearing;
            if (wp_distance <= mission_step.parameter2)
                POI_flag = 1; // 进入环绕圈内
        }
        else
        {
            // 已经到达预定半径开始绕圈

            POI_Diff = constrain((int16_t)(mission_step.parameter1 - wp_distance) * (GPS_conf.crosstrack_gain) / 10, -3500, +3500); // 算上半径控制半径偏差不远
            nav_bearing = target_bearing - 9000 - POI_Diff;
        }
    }
    else
    {
        POI_flag = 0; // 释放标志位
        nav_bearing = target_bearing;
    }
    nav_bearing = wrap_36000(nav_bearing);

//	/***********************/
//    if (abs(wrap_18000(target_bearing - original_target_bearing)) < 4500) {  // If we are too far off or too close we don't do track following
//            if(crosstrack_error>2500)//大于10m距离
//                crosstrack_error*=GPS_conf.crosstrack_gain*2 / 100.0 ;
//            //crosstrack_error = sin(temp) * (wp_distance * GPS_conf.crosstrack_gain / 100.0 );//CROSSTRACK_GAIN = 1);  // Meters we are off track line
//            else
//                crosstrack_error*=GPS_conf.crosstrack_gain/100.0;

//       nav_bearing+= constrain(crosstrack_error, -4000, 4000);
//    } else {
//            nav_bearing = target_bearing;
//    }
//	//	nav_bearing = target_bearing;
//		nav_bearing = wrap_36000(nav_bearing);

//    debug[0]=original_target_bearing/100;
//    debug[1]=target_bearing/100;
//    debug[2]=nav_bearing/100;

//   nav_bearing = target_bearing;  //===================================================
#endif
    // nav_bearing = target_bearing;
}

////////////////////////////////////////////////////////////////////////////////////
// Determine desired speed when navigating towards a waypoint, also implement slow
// speed rampup when starting a navigation
//
//      |< WP Radius
//      0  1   2   3   4   5   6   7   8m
//      ...|...|...|...|...|...|...|...|
//                100  |  200     300     400cm/s
//                 |                                        +|+
//                 |< we should slow to 1 m/s as we hit the target
//
static uint16_t GPS_calc_desired_speed(uint16_t max_speed, bool _slow)
{
    if (_slow)
    {
        max_speed = MIN(max_speed, wp_distance / 2);
    }
    else
    {
        max_speed = MIN(max_speed, wp_distance);
        max_speed = MAX(max_speed, GPS_conf.nav_speed_min); // go at least nav_speed_min
    }
    // limit the ramp up of the speed  //限制速度的倾斜上升
    // waypoint_speed_gov is reset to 0 at each new WP command
    if (max_speed > waypoint_speed_gov)
    {
        waypoint_speed_gov += (int)(100.0 * dTnav); // increase at .5/ms  //waypoint_speed_gov = GPS_conf.nav_speed_min;  在void GPS_set_next_wp()设置
        max_speed = waypoint_speed_gov;
    }
    return max_speed;
}

////////////////////////////////////////////////////////////////////////////////////
// Utilities
//

int32_t wrap_36000(int32_t ang)
{
    if (ang > 36000)
        ang -= 36000;
    if (ang < 0)
        ang += 36000;
    return ang;
}

/*
 * EOS increased the precision here, even if we think that the gps is not precise enough, with 10e5 precision it has 76cm resolution
 * with 10e7 it's around 1 cm now. Increasing it further is irrelevant, since even 1cm resolution is unrealistic, however increased
 * resolution also increased precision of nav calculations
 */

#define DIGIT_TO_VAL(_x) (_x - '0')
uint32_t GPS_coord_to_degrees(char *s)
{
    char *p, *q;
    uint8_t deg = 0, min = 0;
    unsigned int frac_min = 0;
    uint8_t i;

    // scan for decimal point or end of field
    for (p = s; isdigit(*p); p++)
        ;
    q = s;

    // convert degrees
    while ((p - q) > 2)
    {
        if (deg)
            deg *= 10;
        deg += DIGIT_TO_VAL(*q++);
    }
    // convert minutes
    while (p > q)
    {
        if (min)
            min *= 10;
        min += DIGIT_TO_VAL(*q++);
    }
    // convert fractional minutes
    // expect up to four digits, result is in
    // ten-thousandths of a minute
    if (*p == '.')
    {
        q = p + 1;
        for (i = 0; i < 4; i++)
        {
            frac_min *= 10;
            if (isdigit(*q))
                frac_min += *q++ - '0';
        }
    }
    return deg * 10000000UL + (min * 1000000UL + frac_min * 100UL) / 6;
}

// helper functions
uint16_t grab_fields(char *src, uint8_t mult)
{ // convert string to uint16
    uint8_t i;
    uint16_t tmp = 0;

    for (i = 0; src[i] != 0; i++)
    {
        if (src[i] == '.')
        {
            i++;
            if (mult == 0)
                break;
            else
                src[i + mult] = 0;
        }
        tmp *= 10;
        if (src[i] >= '0' && src[i] <= '9')
            tmp += src[i] - '0';
    }
    return tmp;
}

uint8_t hex_c(uint8_t n)
{ // convert '0'..'9','A'..'F' to 0..15
    n -= '0';
    if (n > 9)
        n -= 7;
    n &= 0x0F;
    return n;
}

//************************************************************************
// Common GPS functions
//

void init_RTH()
{
    f.GPS_mode = GPS_MODE_RTH; // Set GPS_mode to RTH
    f.GPS_BARO_MODE = TRUE;
    GPS_hold[LAT] = GPS_coord[LAT]; // All RTH starts with a poshold
    GPS_hold[LON] = GPS_coord[LON]; // This allows to raise to rth altitude//原地爬升高度
    GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON], &GPS_hold[LAT], &GPS_hold[LON]);
    NAV_paused_at = 0;
    if (GPS_conf.rth_altitude == 0)
        set_new_altitude(SET_ALT_ROI); // Return at actual altitude
    // set_new_altitude(GPS_altitude * 100);//==单位米===
    else
    { // RTH altitude is defined, but we use it only if we are below it
        if (alt.EstAlt < GPS_conf.rth_altitude * 100)
            set_new_altitude(GPS_conf.rth_altitude * 100);
        else
            set_new_altitude(SET_ALT_ROI);
    }
    f.GPS_head_set = 0;              // Allow the RTH ti handle heading
    NAV_state = NAV_STATE_RTH_START; // NAV engine status is Starting RTH.
}

void GPS_reset_home_position(void)
{
    if (f.GPS_FIX && GPS_numSat >= 5)
    {
        GPS_home[LAT] = GPS_coord[LAT];
        GPS_home[LON] = GPS_coord[LON];
        GPS_calc_longitude_scaling(GPS_coord[LAT]); // need an initial value for distance and bearing calc
        nav_takeoff_bearing = att.heading;          // save takeoff heading
        // TODO: Set ground altitude
        f.GPS_FIX_HOME = 1;
    }
}

// reset navigation (stop the navigation processor, and clear nav)
void GPS_reset_nav(void)
{
    uint8_t i;

    for (i = 0; i < 2; i++)
    {
        nav[i] = 0;
        reset_PID(&posholdPID[i]);
        reset_PID(&poshold_ratePID[i]);
        reset_PID(&navPID[i]);
    }
    NAV_state = NAV_STATE_NONE;
    // invalidate JUMP counter
    jump_times = -10;
    // reset next step counter
    next_step = 1;
    // Clear poi
    GPS_poi[LAT] = 0;
    GPS_poi[LON] = 0;
    f.GPS_head_set = 0;
    mission_step.action = 0;
    nav_bearing = target_bearing = 0;
    f.NAV_ROLL_LOCK = 0;
}

// Reset nav
void clearNav(void)
{
#if defined(FIXEDWING) //=============================================================================================
    NaverrorI = 0;
    AlterrorI = 0;
    lastAltDiff = 0;
    lastNavDiff = 0;
    // SpeedBoost=0;
    for (uint8_t i = 0; i < GPS_UPD_HZ; i++)
    {
        AltHist[i] = 0;
        NavDif[i] = 0;
    };
#endif
}
// Get the relevant P I D values and set the PID controllers
void GPS_set_pids(void)
{
    posholdPID_PARAM.kP = (float)conf.pid[PIDPOS].P8 / 100.0;
    posholdPID_PARAM.kI = (float)conf.pid[PIDPOS].I8 / 100.0;
    posholdPID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;

    poshold_ratePID_PARAM.kP = (float)conf.pid[PIDPOSR].P8 / 10.0;
    poshold_ratePID_PARAM.kI = (float)conf.pid[PIDPOSR].I8 / 100.0;
    poshold_ratePID_PARAM.kD = (float)conf.pid[PIDPOSR].D8 / 1000.0;
    poshold_ratePID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;

    navPID_PARAM.kP = (float)conf.pid[PIDNAVR].P8 / 10.0;
    navPID_PARAM.kI = (float)conf.pid[PIDNAVR].I8 / 100.0;
    navPID_PARAM.kD = (float)conf.pid[PIDNAVR].D8 / 1000.0;
    navPID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;

#ifdef FIXEDWING
    altPID_PARAM.kP = (float)conf.pid[PIDALT].P8 / 10.0; // conf.pid[PIDALT].P8   = 30;conf.pid[PIDALT].I8  = 20;conf.pid[PIDALT].D8   = 45;
    altPID_PARAM.kI = (float)conf.pid[PIDALT].I8 / 100.0;
    altPID_PARAM.kD = (float)conf.pid[PIDALT].D8 / 1000.0;
#endif
}
// It was moved here since even i2cgps code needs it
int32_t wrap_18000(int32_t ang)
{
    if (ang > 18000)
        ang -= 36000;
    if (ang < -18000)
        ang += 36000;
    return ang;
}
/**************************************************************************************/
/**************************************************************************************/
/***********************       specific  GPS device section  **************************/
/**************************************************************************************/
/**************************************************************************************/

/**************************************************************************************/
/***********************       UBLOX                         **************************/
/**************************************************************************************/
#if defined(UBLOX)
const char UBLOX_INIT[] = {
    // PROGMEM array must be outside any function !!!
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19, // disable all default NMEA messages
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47,                               // set POSLLH MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x03, 0x01, 0x0F, 0x49,                               // set STATUS MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01, 0x12, 0x4F,                               // set SOL MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1E, 0x67,                               // set VELNED MSG rate
    0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x07, 0x03, 0x00, 0x51, 0x08, 0x00, 0x00, 0x8A, 0x41, // set WAAS to EGNOS
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A              // set rate to 5Hz
};

struct ubx_header
{
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
};
struct ubx_nav_posllh
{
    uint32_t time; // GPS msToW
    int32_t longitude;
    int32_t latitude;
    int32_t altitude_ellipsoid;
    int32_t altitude_msl;
    uint32_t horizontal_accuracy;
    uint32_t vertical_accuracy;
};
struct ubx_nav_solution
{
    uint32_t time;
    int32_t time_nsec;
    int16_t week;
    uint8_t fix_type;
    uint8_t fix_status;
    int32_t ecef_x;
    int32_t ecef_y;
    int32_t ecef_z;
    uint32_t position_accuracy_3d;
    int32_t ecef_x_velocity;
    int32_t ecef_y_velocity;
    int32_t ecef_z_velocity;
    uint32_t speed_accuracy;
    uint16_t position_DOP;
    uint8_t res;
    uint8_t satellites;
    uint32_t res2;
};
struct ubx_nav_velned
{
    uint32_t time; // GPS msToW
    int32_t ned_north;
    int32_t ned_east;
    int32_t ned_down;
    uint32_t speed_3d;
    uint32_t speed_2d;
    int32_t heading_2d;
    uint32_t speed_accuracy;
    uint32_t heading_accuracy;
};

enum ubs_protocol_bytes
{
    PREAMBLE1 = 0xb5,
    PREAMBLE2 = 0x62,
    CLASS_NAV = 0x01,
    CLASS_ACK = 0x05,
    CLASS_CFG = 0x06,
    MSG_ACK_NACK = 0x00,
    MSG_ACK_ACK = 0x01,
    MSG_POSLLH = 0x2,
    MSG_STATUS = 0x3,
    MSG_SOL = 0x6,
    MSG_VELNED = 0x12,
    MSG_CFG_PRT = 0x00,
    MSG_CFG_RATE = 0x08,
    MSG_CFG_SET_RATE = 0x01,
    MSG_CFG_NAV_SETTINGS = 0x24
};
enum ubs_nav_fix_type
{
    FIX_NONE = 0,
    FIX_DEAD_RECKONING = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_GPS_DEAD_RECKONING = 4,
    FIX_TIME = 5
};
enum ubx_nav_status_bits
{
    NAV_STATUS_FIX_VALID = 1
};

// Receive buffer
static union
{
    struct ubx_nav_posllh posllh;
    struct ubx_nav_solution solution;
    struct ubx_nav_velned velned;
    uint8_t bytes[1];
} _buffer;

uint32_t init_speed[5] = {9600, 19200, 38400, 57600, 115200};

// GPS数据包解析
uint8_t GPS_newFrame(uint8_t data)
{
    static uint8_t _step = 0; // State machine state
    static uint8_t _msg_id;
    static uint16_t _payload_length;
    static uint16_t _payload_counter;
    static uint8_t _ck_a; // Packet checksum accumulators
    static uint8_t _ck_b;

    uint8_t ret = FALSE;
    uint8_t st = _step + 1;
    // temp_t++;
    // temp_GPS[temp_t]=data;
    if (st == 2)
        if (PREAMBLE2 != data)
        {
            st--;
            //	temp_t=0;
        } // in case of faillure of the 2nd header byte, still test the first byte
    if (st == 1)
    {
        if (PREAMBLE1 != data)
        {
            st--;
            // temp_t=0;
        }
    }
    else if (st == 3)
    {                         // CLASS byte, not used, assume it is CLASS_NAV
        _ck_b = _ck_a = data; // reset the checksum accumulators
        //	temp[temp_t]=data;
    }
    else if (st > 3 && st < 8)
    {
        _ck_b += (_ck_a += data); // checksum byte
        if (st == 4)
        {
            _msg_id = data;
        }
        else if (st == 5)
        {
            _payload_length = data; // payload length low byte
        }
        else if (st == 6)
        {
            _payload_length += (uint16_t)(data << 8);
            if (_payload_length > 512)
                st = 0;
            _payload_counter = 0; // prepare to receive payload
        }
        else
        {
            if (_payload_counter + 1 < _payload_length)
                st--; // stay in the same state while data inside the frame
            if (_payload_counter < sizeof(_buffer))
                _buffer.bytes[_payload_counter] = data;
            _payload_counter++;
        }
    }
    else if (st == 8)
    {
        if (_ck_a != data)
            st = 0; // bad checksum
    }
    else if (st == 9)
    {
        st = 0;
        if (_ck_b == data)
        { // good checksum
            if (_msg_id == MSG_POSLLH)
            {
                if (f.GPS_FIX)
                {
                    GPS_coord[LON] = _buffer.posllh.longitude;
                    GPS_coord[LAT] = _buffer.posllh.latitude;
                    GPS_altitude = _buffer.posllh.altitude_msl / 1000; // alt in m
                    if (f.ARMED)
                    {
                        GPS_altitude = GPS_altitude - GPS_altitude_cel; // 解锁前保持更新差值
                    }
                    else
                        GPS_altitude_cel = GPS_altitude; // 解锁后减去差值
                    // GPS_time       = _buffer.posllh.time; //not used for the moment
                }
                ret = TRUE; // POSLLH message received, allow blink GUI icon and LED, frame available for nav computation
            }
            else if (_msg_id == MSG_SOL)
            {
                f.GPS_FIX = 0;
                if ((_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D || _buffer.solution.fix_type == FIX_2D))
                {
                    f.GPS_FIX = 1;
                }

                GPS_numSat = _buffer.solution.satellites;
            }
            else if (_msg_id == MSG_VELNED)
            {
                //                for(int axis=0; axis<GPS_USER_SMOOTH_LEN-1; axis++)
                //                    GPS_speed_smooth[axis]=GPS_speed_smooth[axis+1];
                //                GPS_speed_smooth[0]=_buffer.velned.speed_2d;
                //                static uint32_t add_speed=0;
                //                for(int axis=0; axis<GPS_USER_SMOOTH_LEN-1; axis++)
                //                    add_speed+=GPS_speed_smooth[axis];
                //                GPS_speed=add_speed/GPS_USER_SMOOTH_LEN;

                GPS_speed = _buffer.velned.speed_2d;                               // cm/s
                GPS_ground_course = (uint16_t)(_buffer.velned.heading_2d / 10000); // Heading 2D deg * 100000 rescaled to deg * 10 //not used for the moment
            }
        }
    }
    _step = st;
    return ret;
    // return 0;
}
#endif // UBLOX
/*****************************************************************************************/
/*****************************************************************************************/
/*****************************************************************************************/
#if defined(FIXEDWING)

PID_PARAM posholdPID_PARAM;
PID_PARAM poshold_ratePID_PARAM;
PID_PARAM navPID_PARAM;
PID_PARAM altPID_PARAM;

void FW_NAV()
{
    // Navigation with Planes under Development.

    static int16_t NAV_deltaSum = 0, ALT_deltaSum = 0;
    static int16_t GPS_FwTarget = 0; // Gps correction for Fixed wing
    static int16_t GPS_AltErr = 0;
    static int16_t NAV_Thro = 0;
    int16_t GPS_Heading = GPS_ground_course; // Store current bearing

    int16_t altDiff = 0, AlterrorP = 0;
    int16_t RTH_Alt = GPS_conf.rth_altitude; //==========应该已经设置好了!!!？？？？？？======================
    int16_t delta[2] = {0, 0};               // D-Term
    int16_t TX_Thro = rcData[THROTTLE];      // Read and store Throttle pos.
    int16_t navDiff = 0;
    // Calculated Altitude over home in meters
    int16_t curr_Alt = alt.EstAlt / 100; // New  //===应该不应该改成GSP高度???=======可用解锁时的GPS高度做为0高程===气压计变化较大=========

    static int32_t nav_deltalast1 = 0, nav_deltalast2 = 0;

    //    if(NAV_flag) {		//NAV_flag GPS数据更新标志位，GPS不更新不进行计算
    //        NAV_flag=0;
    // Wrap GPS_Heading 1800
    if (GPS_Heading > 1800)
        GPS_Heading -= 3600;
    if (GPS_Heading < -1800)
        GPS_Heading += 3600;

// Only use MAG with small Tilt angle and if Mag and Gps.heading aligns
#if MAG                                                                // 锁头模式下，当GPS方向与磁力计方向偏差超过10并且速度大于800则当前航行以GPS为准
    if (ABS(att.heading - (GPS_Heading / 10)) > 10 && GPS_speed > 800) // GPS and MAG heading diff.
    {
        Current_Heading = GPS_Heading / 10;
        //	debug[0]=1;
    }
    else
    {
        Current_Heading = att.heading;
        //	debug[0]=2;
    }
    //				debug[0]=att.heading;
    //				debug[1]=GPS_Heading/10;

#else
    Current_Heading = GPS_Heading / 10;
#endif

#ifdef SIMDEBUG // TODO remove//===========================================================
                // Current_Heading = att.heading;  //=====使用体身磁力计确定方向，可能在受干扰时不可靠，测试时可使用。 原来使用的GPS方向进行导航！！！！！！
#endif
    // Calculate Navigation errors
    GPS_FwTarget = nav_bearing / 100; // target_bearing/100;
                                      //=============================
                                      // magHold=GPS_FwTarget;

    navDiff = GPS_FwTarget - Current_Heading; // Navigation Error in degrees

    /*2018-8-25测 试 使用地理坐标系计算航向角误差*/

    // Wrap Heading 180

    if (navDiff <= -180)
        navDiff += 360;
    if (navDiff >= +180)
        navDiff -= 360;
    //=====================================================================================================

    // Force a left turn unless Waypointmode.
    if (NAV_state != NAV_STATE_WP_ENROUTE && mission_step.action != MISSION_SET_POI)
        if (navDiff > 165)
            navDiff = -179; //======================

    // Filtering of navDiff 10m around home to stop nervous servos
    // if ( GPS_distanceToHome <10 )navDiff*=0.1;
    // wp_distance
    uint32_t wp_distance_R;
    GPS_distance_cm(&GPS_coord[LAT], &GPS_coord[LON], &GPS_FROM[LAT], &GPS_FROM[LON], &wp_distance_R);
    // 计算上一个航点的距离
    if (mission_step.action != MISSION_SET_POI && (wp_distance < 1000 || wp_distance_R < 1500)) // 半径10m不改变方向
    {
        f.NAV_ROLL_LOCK = 1;
    }
    else
    {
        f.NAV_ROLL_LOCK = 0;
    }

    //=====两种高度设定对比测试===在init_RTH();需要修改对应的高度参考值！！！！！================================================================================================================================
    //  //GPS_AltErr      = -get_altitude_error()/100; //  Altitude error in meters Negative means you're to low  //alt_to_hold - alt.EstAlt;   //====气压计高度做为高度设定===============================================================
    GPS_AltErr = -(alt_to_hold - GPS_altitude * 100) / 100; //====GPS高度做为高度设定=====当set_new_altitude(alt)=alt单位为CM时采用====GPS_altitude单位为米====正为超高==================================================================
    static uint32_t gpsTimer = 0;
    static uint16_t gpsFreq = 1000 / GPS_UPD_HZ; // 5HZ 200ms DT   //可直接定义常量减少计算量！！！！
    if (millis() - gpsTimer >= gpsFreq)
    { //===确保更新GPS数据时才进行处理！！！======
        gpsTimer = millis();

        // Throttle control

        // desired speed to stick setting Test TEST  //==初始油门大小========
        if (rcData[THROTTLE] > MAXCHECK || f.FS_MODE)
        {                              // if throttle is on Full use predefined setting   //MAXCHECK=1900
            NAV_Thro = CRUICETHROTTLE; // 1400
            if (NAV_state == NAV_STATE_WP_ENROUTE && alt_to_hold == 0)
                NAV_Thro = IDLE_THROTTLE; // TODO  Idle For landing Test
        }
        else
        { // pilot has control over desired speed
            NAV_Thro = rcData[THROTTLE];
        }

        if (f.FAILSAFE_RTH_ENABLE)     // Don't listen to Pilot until FAILSAFE_RTH_ENABLE is disabled.
            NAV_Thro = CRUICETHROTTLE; // 1400

        // Deadpan for throttle at correct Alt.
        if (abs(GPS_AltErr) > 1)
        { // Add AltitudeError  and scale up with a factor to throttle  //#define SCALER_THROTTLE  8 ->10
            NAV_Thro = constrain(NAV_Thro - (GPS_AltErr * SCALER_THROTTLE), IDLE_THROTTLE, CLIMBTHROTTLE);
        } // GPS_AltErr      = -get_altitude_error()/100; //  Altitude error in meters Negative means you're to low

        // Reset Climbout Flag when Alt have been reached
        if (f.CLIMBOUT_FW && GPS_AltErr >= 0)
            f.CLIMBOUT_FW = 0; // GPS_AltErr>0为高于目标，小于0低于目标
        // Climb out before RTH
        if (f.GPS_mode == GPS_MODE_RTH)
        {
            if (f.CLIMBOUT_FW)
            {                                      // 按照给定误差控制角度进行爬升！！数据是不是太大？？？！！！！=====================
                GPS_AltErr = -(GPS_MAXCLIMB * 10); // Max climbAngle   //#define GPS_MAXCLIMB   20     // Degrees climbing . To much can stall the plane.
                NAV_Thro = CLIMBTHROTTLE;          // Max Allowed Throttle  //CLIMBTHROTTLE=1900
                if (curr_Alt <= SAFE_NAV_ALT)
                    navDiff = 0; // Force climb with Level Wings below safe Alt  //SAFE_NAV_ALT = 20 meters  //===如果用GPS高度刚需修改curr_Alt==============
            }
        }

        /******************************
        PID for Navigating planes.
        ******************************/
        float NavDT;
        static uint32_t nav_loopT;
        int32_t nav_delta = 0;
        NavDT = (float)(millis() - nav_loopT) / 1000;
        nav_loopT = millis();

        /****************************************************************************************************/
        // Altitude PID

        // 关掉下面这句测试一下效果！！！======================
        if (abs(GPS_AltErr) <= 3)
            AlterrorI *= NavDT; // Remove I-Term in deadspan //什么原理？？？

        GPS_AltErr *= 10;
        AlterrorI += (GPS_AltErr * altPID_PARAM.kI) * NavDT; // Acumulate I from PIDPOSR   //altPID_PARAM.kI=0.2  altPID_PARAM.kP=3,  altPID_PARAM.kD=0.045
        // AlterrorI =constrain(AlterrorI,-500,500);                     // limits I term influence
        AlterrorI = constrain(AlterrorI, -600, 550); // limits I term influence
        // debug[3] = AlterrorI; //===================================================================================================================================================================================================
        delta[0] = (GPS_AltErr - lastAltDiff); // 升降的速度
        lastAltDiff = GPS_AltErr;
        if (ABS(delta[0]) > 100)
            delta[0] = 0; // 限制D值的可信范围！！！！！！！
        //===用最近3个值进行测试=================================================================
        AltHist[2] = AltHist[1]; // 第2次
        AltHist[1] = AltHist[0]; // 前1次
        AltHist[0] = delta[0];   // 当前
        ALT_deltaSum = ((AltHist[0] + AltHist[1] + AltHist[2]) << 4) * altPID_PARAM.kD / NavDT;
        // debug[3] = ALT_deltaSum; //ok==============================================================

        AlterrorP = GPS_AltErr * altPID_PARAM.kP; // Add P in Elevator compensation.//altPID_PARAM.kP = 3
        altDiff = AlterrorP + AlterrorI;          // Add I
        altDiff = constrain(altDiff, -890, 750);  // 修改定高限幅
        // debug[0] = GPS_altitude;    //&&&&&&&&&&&&&&
        // debug[1] = AlterrorP;
        // debug[2] = altDiff/4;
        // debug[3] = ALT_deltaSum;    //&&&&&&&

        //						GPS_AltErr*=10;
        //						//计算P值
        //						AlterrorP = GPS_AltErr * altPID_PARAM.kP ;  // Add P in Elevator compensation.//altPID_PARAM.kP = 3
        //						AlterrorP = constrain(AlterrorI,-300,300);
        //
        //            //计算I值
        //            if (abs(GPS_AltErr)<=3) AlterrorI*=NavDT;  // Remove I-Term in deadspan //什么原理？？？
        //            AlterrorI += (GPS_AltErr * altPID_PARAM.kI) * NavDT;          // Acumulate I from PIDPOSR   //altPID_PARAM.kI=0.2  altPID_PARAM.kP=3,  altPID_PARAM.kD=0.045
        //            AlterrorI = constrain(AlterrorI,-80,80);                     // limits I term influence

        //						//计算D值
        //            delta[0] = (GPS_AltErr - lastAltDiff)*20;		//升降的速度
        //            lastAltDiff = GPS_AltErr;
        //						ALT_deltaSum =  ((delta[0] + AltHist[0] + AltHist[1])/3) * altPID_PARAM.kD / NavDT ;
        //            ALT_deltaSum = constrain(ALT_deltaSum,-300,300);
        //            AltHist[1] = AltHist[0];  //前1次
        //            AltHist[0] = delta[0];    //当前

        //            altDiff = AlterrorP + AlterrorI - ALT_deltaSum;   // Add I
        //            altDiff = constrain(altDiff,-300,300);//修改定高限幅

        /****************************************************************************************************/
        // Nav PID NAV
        // debug[0] = NavDT*1000;

        // navDiff*=10;
        NaverrorP = navDiff * navPID_PARAM.kP; // Add Nav P
        NaverrorP = constrain(NaverrorP, -400, 400);
        // debug[1] = NaverrorP;

        NaverrorI += (navDiff * navPID_PARAM.kI) * NavDT;
        NaverrorI = constrain(NaverrorI, -80, 80);
        if (abs(navDiff) <= 2)
            NaverrorI *= NavDT; // Remove I-Term in deadspan //接近目标高度移除积分
        // navDiff += NaverrorI;        // Add I
        // debug[2] = NaverrorI;

        nav_delta = (navDiff - lastNavDiff) * 10;

        lastNavDiff = navDiff;
        NAV_deltaSum = (nav_delta + nav_deltalast1 + nav_deltalast2) / 3;
        NAV_deltaSum = (NAV_deltaSum * navPID_PARAM.kD) / NavDT; // Add D
        NAV_deltaSum = constrain(NAV_deltaSum, -300, 300);
        nav_deltalast2 = nav_deltalast1;
        nav_deltalast1 = nav_delta;
        if (abs(navDiff) > 30)
            NAV_deltaSum = 0; // Filter out too big values  //限制值可以再考虑一下！！！！
                              // debug[3] = NAV_deltaSum;
        // debug[3] = GPS_angle[ROLL];

        // Store 1 sec history for D-term in shift register
        //            for(uint8_t i=0; i < GPS_UPD_HZ-1; i++) {//此处原文由溢出，循环了9次，修改为GPS_UPD_HZ-1
        //                NavDif[i] = NavDif[i+1];
        //            }
        // 方向导航D值没用上
        //            NavDif[GPS_UPD_HZ-1]=nav_delta;
        //            NAV_deltaSum = 0; // Sum History
        //            for(uint8_t i=0; i<GPS_UPD_HZ; i++) {
        //                NAV_deltaSum += NavDif[i];
        //            }

        /****************************************************************************************************/
        /******* End of PID *******/

        // Limit outputs
        // GPS_angle[PITCH] = constrain(altDiff/10,-GPS_MAXCLIMB*10,GPS_MAXDIVE*10) + ALT_deltaSum; //GPS_MAXDIVE=20   GPS_MAXCLIMB=30
        int16_t temp_angle_pitch; // 升力补偿
        if (!f.CLIMBOUT_FW)
        {
            temp_angle_pitch = constrain(ABS(att.angle[ROLL]) * (0.6), 0, 220); // #define ELEVATORCOMPENSATION   100 升力补偿
        }
        else
        {
            GPS_angle[PITCH] = 0;
        }
        D_U[5] = altDiff / 4;
        D_U[6] = GPS_angle[PITCH]; // 这里检查倾角油门补偿
        // GPS_angle[PITCH] = constrain(altDiff/4 + ALT_deltaSum-temp_angle_pitch,-GPS_MAXCLIMB*10,GPS_MAXDIVE*10); //GPS_MAXDIVE=20   GPS_MAXCLIMB=15  //==原来的P,I值权重太小===========
        // GPS_angle[YAW]   = constrain(NaverrorP+ NaverrorI  + NAV_deltaSum,-GPS_RUDDER*10,  GPS_RUDDER*10 ); //GPS_RUDDER=20
        // GPS_angle[ROLL]  = constrain(navDiff/9 + NAV_deltaSum,-GPS_MAXCORR*10, GPS_MAXCORR*10); //GPS_MAXCORR=32

        GPS_angle[PITCH] = constrain(altDiff / 4 + ALT_deltaSum - temp_angle_pitch, -300, 300);
        GPS_angle[ROLL] = constrain(NaverrorP + NaverrorI + NAV_deltaSum, -400, 400); // GPS_MAXCORR=32

        if (mission_step.parameter2 == 1 || f.NAV_ROLL_LOCK) // 特殊航点锁定横滚
        {
            GPS_angle[ROLL] = 0;
        }
        // GPS_angle[ROLL]  = constrain(navDiff/10,-GPS_conf.nav_bank_max, GPS_conf.nav_bank_max) + NAV_deltaSum; // From Gui
        // debug[0]=GPS_angle[ROLL];
        //***********************************************//
        // Elevator compensation depending on behaviour. //
        //***********************************************//
        // Prevent stall with Disarmed motor  New TEST
        if (f.MOTORS_STOPPED)
        {
            GPS_angle[PITCH] = constrain(GPS_angle[PITCH], 0, GPS_MAXDIVE * 10); //==小油门时的失速保护！！！==================================================
        }

        // Add elevator compared with rollAngle
        // if (!f.CLIMBOUT_FW) {GPS_angle[PITCH]-= abs(att.angle[ROLL]) * (ELEVATORCOMPENSATION /100);}  //#define ELEVATORCOMPENSATION   100
        D_U[4] = GPS_angle[ROLL];
        D_U[7] = GPS_angle[PITCH];
        //***********************************************//
        // Throttle compensation depending on behaviour. //
        //***********************************************//
        // 油门补偿
        // Compensate throttle with pitch Angle
        // NAV_Thro -= constrain(att.angle[PITCH] * PITCH_COMP ,0 ,450 );  //#define PITCH_COMP             0.5f   //只当机头下时，减油？？！！！
        NAV_Thro -= constrain(att.angle[PITCH] * PITCH_COMP, -200, 200); // #define PITCH_COMP             0.5f   //=测试机头向下，减汕，向上时加油==========================================
        // NAV_Thro += constrain(GPS_angle[ROLL] * 0.1,-100,100 );
        NAV_Thro = constrain(NAV_Thro, IDLE_THROTTLE, CLIMBTHROTTLE);

        FW_NavSpeed();
        NAV_Thro += SpeedBoost;
    }
    /******* End of NavTimer *******/
    //   }
    // PassThru for throttle In AcroMode
    if ((!f.ANGLE_MODE && !f.HORIZON_MODE) || (f.PASSTHRU_MODE && !f.FAILSAFE_RTH_ENABLE))
    {
        NAV_Thro = TX_Thro;
        GPS_angle[PITCH] = 0;
        GPS_angle[ROLL] = 0;
        GPS_angle[YAW] = 0;
    }
    // NAV_Thro  = constrain(NAV_Thro,IDLE_THROTTLE,CLIMBTHROTTLE );//对油门进行限幅
    NAV_Thro = constrain(NAV_Thro, 1300, CLIMBTHROTTLE); // 对油门进行限幅

#if 1
    GPS_angle[PITCH] = 0;
    GPS_angle[ROLL] = 0;
    GPS_angle[YAW] = 0;
#endif
    // rcCommand[THROTTLE] = NAV_Thro;
    rcCommand[YAW] += GPS_angle[YAW]; //
    CradleControl(GPS_altitude * 100);
}
/******* End of FixedWing Navigation *******/

/*############### Nav Speed ###############*/
void FW_NavSpeed(void)
{
#define GPS_MINSPEED 950  // 500  // 500= ~18km/h
#define GPS_MAXSPEED 2000 // 2000cm/s =72km/h
#define I_TERM 0.1f
#if defined(AIRSPEED)
    static int16_t air_Navspeed = AIRSPEED * 100;     // AIRSPEED=15
    static int16_t air_Maxspeed = AIR_MAXSPEED * 100; // AIR_MAXSPEED=20
    int spDiff = 0;
    static int spAdd = 0;
    int this_speed__ = 0;

    if ((GPS_speed) < (GPS_MINSPEED))
    {                                                    // check for too slow ground speed  //#define GPS_MINSPEED  500
        int Delta = (GPS_MINSPEED - GPS_speed) * I_TERM; // #define I_TERM        0.1f
        spAdd += (Delta == 0) ? 1 : Delta;               // increase nav air speed
    }
    else
    {
        spAdd -= spAdd * I_TERM; // decrease nav air speed
    }
    spAdd = constrain(spAdd, 0, air_Maxspeed - air_Navspeed - 100); // 怎么确定的？？？？================

    if (airspeedSpeed < air_Navspeed - 100 + spAdd || airspeedSpeed > air_Navspeed + 100 + spAdd)
    {                                                    // maintain air speed between NAVSPEED and MAXSPEED
        spDiff = (air_Navspeed + spAdd - airspeedSpeed); // math bug if put multiplication in same line
        spDiff = spDiff * I_TERM;
        SpeedBoost += spDiff;
    }

    SpeedBoost = constrain(SpeedBoost, -500, 500);
#else
    // Force the Plane move forward in headwind with SpeedBoost
    int16_t groundSpeed = GPS_speed;
    static uint16_t boostlimit = MAXTHROTTLE - CRUICETHROTTLE; // MAXTHROTTLE =2000
    static int16_t boostlimitm = MINTHROTTLE - CRUICETHROTTLE;
    int16_t target_speed = 0;
    // int spDiff=(GPS_MINSPEED -groundSpeed)*I_TERM;   //#define I_TERM        0.1f
    // int spDiff=(GPS_MINSPEED -groundSpeed)/10;   //#define I_TERM        0.1f  //==原来的，速度较慢===============
    int spDiff = 0;
    static int16_t groundSpeedLast1 = 0, groundSpeedLast2 = 0;

    groundSpeed = (GPS_speed + groundSpeedLast1 + groundSpeedLast2) / 3; // GPS三次速度平均
    groundSpeedLast2 = groundSpeedLast1;
    groundSpeedLast1 = GPS_speed;

    target_speed = constrain(GPS_NAV_speed_set, GPS_MINSPEED, GPS_MAXSPEED); // 对预计速度进行控制
    spDiff = (target_speed - groundSpeed) * 0.3f;                            // #define I_TERM        0.2f'    I值控制速度

    if (GPS_speed < target_speed - 50 || GPS_speed > target_speed + 50)
        SpeedBoost += spDiff;
    SpeedBoost = constrain(SpeedBoost, boostlimitm, boostlimit);
    //		debug[0] = groundSpeed;
    //		debug[1] = SpeedBoost;

#endif
}

/*############### End Nav Speed ###############*/

int16_t calcangle_roll = 0, calcangle_pitch = 0;
int16_t crosroll_error = 0, crospitch_error = 0;
int16_t servo0 = 0, servo5 = 0;
// void CradleControl(int16_t altitude){

//    int temp_error_tb_otb=nav_bearing - Current_Heading*100;
//		int32_t wp_distancelast = 999,TargetDistance=0;
//	  int32_t nav_bearinglast;
//
//		GPS_distance_cm(&GPS_coord[LAT],&GPS_coord[LON],&GPS_FROM[LAT],&GPS_FROM[LON],&wp_distancelast);
//		//GPS_bearing(&GPS_FROM[LAT],&GPS_FROM[LON],&GPS_WP[LAT],&GPS_WP[LON],&target_bearing);
//    float temp = 0;  //弧度转换
//
//	if(wp_distancelast<wp_distance){
//		TargetDistance = wp_distancelast;
//		GPS_bearing(&GPS_coord[LAT],&GPS_coord[LON],&GPS_FROM[LAT],&GPS_FROM[LON],&nav_bearinglast);
//		temp_error_tb_otb=nav_bearinglast - Current_Heading*100;
//	}
//	else
//		TargetDistance = wp_distance;
////		if(temp_error_tb_otb>+180)temp_error_tb_otb-=360;
////		if(temp_error_tb_otb<-180)temp_error_tb_otb+=360;
//    temp_error_tb_otb=wrap_18000(temp_error_tb_otb);
//    temp=temp_error_tb_otb* RADX100;

//    int8_t anglelimity = 0;
//    crosroll_error = sin(temp) * TargetDistance;
//	  calcangle_roll = _atan2(crosroll_error,altitude)/10;
//	  calcangle_roll = constrain(calcangle_roll,-90,+90);
//
//	  //calcangle_roll = 0;				//写死60度用于测试*****************
////		if(abs(att.angle[0])>=300 && ((att.angle[0]/10+calcangle_roll)>80||(att.angle[0]/10+calcangle_roll)<-80))
////		{
////			if(att.angle[0]>0)
////				anglelimity = 80-(constrain(att.angle[0]/10,-80,80));
////			else
////				anglelimity = (-80)-(constrain(att.angle[0]/10,-80,80));
////			anglelimity = constrain((-1)*anglelimity,-40,40);
////			//calcangle_roll = constrain(anglelimity,-40,+40);
////			if(anglelimity>0)
////				calcangle_roll = constrain(calcangle_roll,-anglelimity,anglelimity);
////			else
////				calcangle_roll = constrain(calcangle_roll,anglelimity,-anglelimity);
////		}
//
//	  crospitch_error = cos(temp) * TargetDistance;
//	  calcangle_pitch = _atan2(crospitch_error,altitude)/10;
//	  calcangle_pitch = constrain(calcangle_pitch,-90,+90);
//		//calcangle_pitch = 80;  //写死60度用于测试************************
////		if(abs(att.angle[1])>=300 && ((att.angle[1]/10+calcangle_pitch)>70||(att.angle[1]/10+calcangle_pitch)<-70))
////		{
////			if(att.angle[1]>0)
////				anglelimity = 90-(constrain(att.angle[1]/10,-80,80));
////			else
////				anglelimity = (-90)-(constrain(att.angle[1]/10,-80,80));
////			anglelimity = constrain(anglelimity,-60,60);
////			//calcangle_pitch = constrain(anglelimity,-40,+40);
////			if(anglelimity>0)
////				calcangle_pitch = constrain(calcangle_pitch,-anglelimity,anglelimity);
////			else
////				calcangle_pitch = constrain(calcangle_pitch,anglelimity,-anglelimity);
////		}
//		debug[1] = calcangle_pitch;
//		servo5 =  constrain(calcangle_roll * 5,-450,450);
//		servo0 =  constrain(calcangle_pitch * 5 ,-200,450);
//
//}
void CradleControl(int16_t altitude)
{ // 高度差h 单位cm

    int temp_error_tb_otb = nav_bearing - Current_Heading * 100;
    int32_t wp_distancelast = 999, TargetDistance = 0;
    int32_t nav_bearinglast;
    static uint8_t passflag = 0;

    float temp = 0; // 弧度转换

    // 2020年4月24日=============================================================
    TargetDistance = wp_distance; // 两点加距离d 单位cm
    if (wp_distance < 2500)
    { // 向航点飞行，距离小于25米
        wp_distancelast = wp_distance;
        passflag = 1;
    }
    else if (passflag == 1 && wp_distance > 2500)
    { // 经过航点 目标航点跳转为下一个航点
        passflag = 2;
    }
    if (passflag == 2)
    {
        GPS_distance_cm(&GPS_coord[LAT], &GPS_coord[LON], &GPS_FROM[LAT], &GPS_FROM[LON], &wp_distancelast);
        if (wp_distancelast < 2500)
        {
            GPS_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_FROM[LAT], &GPS_FROM[LON], &nav_bearinglast);
            TargetDistance = wp_distancelast;
            temp_error_tb_otb = nav_bearinglast - Current_Heading * 100;
        }
        else
        {
            passflag = 0;
        }
    }

    temp_error_tb_otb = wrap_18000(temp_error_tb_otb); // 方位角φ

    temp = temp_error_tb_otb * RADX100;                     //    RADX100 = PI/180/100
    crosroll_error = sin(temp) * TargetDistance;            // x方向距离dx
    calcangle_roll = _atan2(crosroll_error, altitude) / 10; // x方向角度差φx
    calcangle_roll = constrain(calcangle_roll, -45, +45);

    crospitch_error = cos(temp) * TargetDistance;             // y方向距离dy  单位cm
    calcangle_pitch = _atan2(crospitch_error, altitude) / 10; // y方向角度差φy
    calcangle_pitch = constrain(calcangle_pitch, -45, +45);

    // 增强云台稳定性
    if (abs(att.angle[0] / 10 + calcangle_roll) > 60)
    {
        calcangle_roll = calcangle_roll - (att.angle[0] / 10);
    }

    // 角度差转换为控制量
    servo5 = constrain(calcangle_roll * 10, -450, 450);
    servo0 = constrain(calcangle_pitch * 10, -500, 500);
    debug[0] = TargetDistance / 100;
    debug[1] = calcangle_roll;
    debug[2] = servo5;
    debug[3] = att.angle[0];
    // servo5 = 450;
    //		debug[0] = calcangle_roll;
    //		debug[1] = calcangle_pitch;
}
#endif // FIXEDWING

#endif // GPS Defined
