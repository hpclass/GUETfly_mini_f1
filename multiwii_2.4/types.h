#ifndef TYPES_H_
#define TYPES_H_
#define ABS(x) ((x) > 0 ? (x) : -(x))
#ifdef STM32F10X_MD
#include "stm32f10x.h"
#else
#include "stdint.h"
#endif
// #define RC_CHANS 6
enum rc
{
    ROLL,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4,
    AUX5,
    AUX6,
    AUX7,
    AUX8
};

enum pid
{
    PIDROLL,
    PIDPITCH,
    PIDYAW,
    PIDALT,
    PIDPOS,
    PIDPOSR,
    PIDNAVR,
    PIDLEVEL,
    PIDMAG,
    PIDVEL, // not used currently
    PIDITEMS
};

enum box
{
    BOXARM,
#if ACC
    BOXANGLE,
    BOXHORIZON,
#endif
#if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
    BOXBARO,
#endif
#ifdef VARIOMETER
    BOXVARIO,
#endif
    BOXMAG,
#if defined(HEADFREE)
    BOXHEADFREE,
    BOXHEADADJ, // acquire heading for HEADFREE mode
#endif
#if defined(SERVO_TILT) || defined(GIMBAL) || defined(SERVO_MIX_TILT)
    BOXCAMSTAB,
#endif
#if defined(CAMTRIG)
    BOXCAMTRIG,
#endif
#if GPS
    BOXGPSHOME,
    BOXGPSHOLD,

#endif
#if defined(FIXEDWING) || defined(HELICOPTER)
    BOXPASSTHRU,
#endif
#if defined(BUZZER)
    BOXBEEPERON,
#endif
#if defined(LED_FLASHER)
    BOXLEDMAX, // we want maximum illumination
    BOXLEDLOW, // low/no lights
#endif
#if defined(LANDING_LIGHTS_DDR)
    BOXLLIGHTS, // enable landing lights at any altitude
#endif
#ifdef INFLIGHT_ACC_CALIBRATION
    BOXCALIB,
#endif
#ifdef GOVERNOR_P
    BOXGOV,
#endif
#ifdef OSD_SWITCH
    BOXOSD,
#endif
#if GPS
    BOXGPSNAV,
    BOXLAND,
#endif
#if OPTIC
    BOXOPTIC,
#endif
#if defined(FIXEDWING)
    BOXCRUISE,
#endif
    BOXDROP,
    BOXMASTER,
    BOXSLAVE,
    CHECKBOXITEMS

};
typedef struct
{
    uint32_t lastTime;
    uint32_t dTime;
} multiwii_timer_t;
typedef struct
{
    int16_t accSmooth[3];
    int16_t gyroData[3];
    int16_t magADC[3];
    int16_t gyroADC[3];
    int16_t accADC[3];
} imu_t;
typedef struct
{
    float accelEF[3];          // earth X,Y,Z acceleration in cm/s^2
    float accelEF_Filtered[3]; // earth X,Y,Z acceleration in cm/s^2
    float accelEF_Sum[3];
    uint8_t accelEF_Sum_count[3];
    float accelEF_Horizontal;  // earth averaged horizontal acceleration in cm/s^2
    float velocityEF[3];       // ins earth velocity in cm/s
    float positionEF[3];       // ins position in cm
    float accCorrection[3];    //===???======
    float lastAcceleration[3]; //===???======
    float acceleration[3];     //===???======
} ins_t;
typedef struct
{
    uint8_t vbat; // battery voltage in 0.1V steps
    uint16_t intPowerMeterSum;
    uint16_t rssi;     // range: [0;1023]
    uint16_t amperage; // 1unit == 100mA
    uint16_t watts;    // 1unit == 1W
    uint16_t vbatcells[3];
} analog_t;

typedef struct
{
    int32_t EstAlt; // in cm
    int16_t vario;  // variometer in cm/s
    int32_t rawAlt; // in cm
    // int32_t  lastRawAlt;		   // in cm
    int32_t groundRawAlt; // in cm
    // int16_t  rawVario;           // in cm/s
} alt_t;

typedef struct
{
    int16_t angle[2];  // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
    int16_t heading;   // variometer in cm/s
    int16_t angle_YAW; // angle in multiple of 0.1 degree    180 deg = 1800
} att_t;

typedef struct
{
    uint8_t OK_TO_ARM : 1;
    uint8_t ARMED : 1;
    uint8_t ACC_CALIBRATED : 1;
    uint8_t ANGLE_MODE : 1;
    uint8_t HORIZON_MODE : 1;
    uint8_t MAG_MODE : 1;
    uint8_t BARO_MODE : 1;
    uint8_t OPTIC_MODE : 1;

#ifdef HEADFREE
    uint8_t HEADFREE_MODE : 1;
#endif
#if defined(FIXEDWING) || defined(HELICOPTER)
    uint8_t PASSTHRU_MODE : 1;
#endif
    uint8_t SMALL_ANGLES_25 : 1;
#if MAG
    uint8_t CALIBRATE_MAG : 1;
#endif
#ifdef VARIOMETER
    uint8_t VARIO_MODE : 1;
#endif
    uint8_t GPS_mode : 2; // 0-3 NONE,HOLD, HOME, NAV (see GPS_MODE_* defines
#if BARO || GPS
    uint8_t THROTTLE_IGNORED : 1; // If it is 1 then ignore throttle stick movements in baro mode;
#endif
#if GPS
    uint8_t GPS_FIX : 1;
    uint8_t GPS_FIX_HOME : 1;
    uint8_t GPS_BARO_MODE : 1; // This flag is used when GPS controls baro mode instead of user (it will replace rcOptions[BARO]
    uint8_t GPS_head_set : 1;  // it is 1 if the navigation engine got commands to control heading (SET_POI or SET_HEAD) CLEAR_HEAD will zero it
    uint8_t LAND_COMPLETED : 1;
    uint8_t LAND_IN_PROGRESS : 1;
    uint8_t TAKEOFF_COMPLETED : 1; // 自动起飞
    uint8_t TAKEOFF_IN_PROGRESS : 1;
    uint8_t TAKEOFF : 1;
    uint8_t MOTORS_STOPPED : 1;
    uint8_t FS_MODE : 1; // Failsafe Flag
    uint8_t FAILSAFE_RTH_ENABLE : 1;
    uint8_t CLIMBOUT_FW : 1;
    uint8_t CRUISE_MODE : 1;
    uint8_t Fixed_Wing_Nav : 1;
    uint8_t USER_RTH_FLAG : 1;
    uint8_t USER_DROP_FLAG : 1;
    uint8_t USER_DROP : 1;
    uint8_t NAV_ROLL_LOCK : 1;
    uint8_t Master : 1; //		"Master;"
    uint8_t Slave : 1;  //"Slave;"
#endif
} flags_struct_t;

typedef struct
{
    uint8_t currentSet;
    int16_t accZero[3];
    uint16_t accScale[3]; // sensitivity correction (1000 for acc_1G)
    int16_t magZero[3];
    uint16_t flashsum;
    uint8_t checksum; // MUST BE ON LAST POSITION OF STRUCTURE !
} global_conf_t;

struct pid_
{
    uint8_t P8;
    uint8_t I8;
    uint8_t D8;
};

struct servo_conf_
{                   // this is a generic way to configure a servo, every multi type with a servo should use it
    int16_t min;    // minimum value, must be more than 1020 with the current implementation
    int16_t max;    // maximum value, must be less than 2000 with the current implementation
    int16_t middle; // default should be 1500
    int8_t rate;    // range [-100;+100] ; can be used to ajust a rate 0-100% and a direction
};

typedef struct
{
    struct pid_ pid[PIDITEMS];
    uint8_t rcRate8;
    uint8_t rcExpo8;
    uint8_t rollPitchRate;
    uint8_t yawRate;
    uint8_t dynThrPID;
    uint8_t thrMid8;
    uint8_t thrExpo8;
    int16_t angleTrim[2];
#if defined(EXTENDED_AUX_STATES)
    uint32_t activate[CHECKBOXITEMS]; // Extended aux states define six different aux state for each aux channel
#else
    uint16_t activate[CHECKBOXITEMS];
#endif
    uint8_t powerTrigger1;
#if MAG
    int16_t mag_declination;
#endif
    struct servo_conf_ servoConf[8];
#if defined(GYRO_SMOOTHING)
    uint8_t Smoothing[3];
#endif
#if defined(FAILSAFE)
    int16_t failsafe_throttle;
#endif
#ifdef VBAT
    uint8_t vbatscale;
    uint8_t vbatlevel_warn1;
    uint8_t vbatlevel_warn2;
    uint8_t vbatlevel_crit;
#endif
#ifdef POWERMETER
    uint8_t pint2ma;
#endif
#ifdef POWERMETER_HARD
    uint16_t psensornull;
#endif
#ifdef MMGYRO
    uint8_t mmgyro;
#endif
#ifdef ARMEDTIMEWARNING
    uint16_t armedtimewarning;
#endif
    int16_t minthrottle;
#ifdef GOVERNOR_P
    int16_t governorP;
    int16_t governorD;
#endif
#ifdef YAW_COLL_PRECOMP
    uint8_t yawCollPrecomp;
    uint16_t yawCollPrecompDeadband;
#endif

    uint8_t checksum; // MUST BE ON LAST POSITION OF CONF STRUCTURE !
} conf_t;

#ifdef LOG_PERMANENT
typedef struct
{
    uint16_t arm;        // #arm events
    uint16_t disarm;     // #disarm events
    uint16_t start;      // #powercycle/reset/initialize events
    uint32_t armed_time; // copy of armedTime @ disarm
    uint32_t lifetime;   // sum (armed) lifetime in seconds
    uint16_t failsafe;   // #failsafe state @ disarm
    uint16_t i2c;        // #i2c errs state @ disarm
    uint8_t running;     // toggle on arm & disarm to monitor for clean shutdown vs. powercut
    uint8_t checksum;    // MUST BE ON LAST POSITION OF CONF STRUCTURE !
} plog_t;
#endif

#if GPS

// TODO: cross check with I2C gps and add relevant defines

// This is the mode what is selected via the remote (NONE, HOLD, RTH and NAV (NAV-> exectute mission)
enum gpsmode
{
    GPS_MODE_NONE = 0,
    GPS_MODE_HOLD,
    GPS_MODE_RTH,
    GPS_MODE_NAV
};

enum navstate
{
    NAV_STATE_NONE = 0,
    NAV_STATE_RTH_START,
    NAV_STATE_RTH_ENROUTE,
    NAV_STATE_HOLD_INFINIT,
    NAV_STATE_HOLD_TIMED,
    NAV_STATE_WP_ENROUTE,
    NAV_STATE_PROCESS_NEXT,
    NAV_STATE_DO_JUMP,
    NAV_STATE_LAND_START,
    NAV_STATE_LAND_IN_PROGRESS,
    NAV_STATE_LANDED,
    NAV_STATE_LAND_SETTLE,
    NAV_STATE_LAND_START_DESCENT,
    NAV_STATE_WP_START,
    NAV_STATE_LAND_DETECTED
};

enum naverror
{
    NAV_ERROR_NONE = 0,         // All systems clear
    NAV_ERROR_TOOFAR,           // Next waypoint distance is more than safety distance
    NAV_ERROR_SPOILED_GPS,      // GPS reception is compromised - Nav paused - copter is adrift !
    NAV_ERROR_WP_CRC,           // CRC error reading WP data from EEPROM - Nav stopped
    NAV_ERROR_FINISH,           // End flag detected, navigation finished
    NAV_ERROR_TIMEWAIT,         // Waiting for poshold timer
    NAV_ERROR_INVALID_JUMP,     // Invalid jump target detected, aborting
    NAV_ERROR_INVALID_DATA,     // Invalid mission step action code, aborting, copter is adrift
    NAV_ERROR_WAIT_FOR_RTH_ALT, // Waiting to reach RTH Altitude
    NAV_ERROR_GPS_FIX_LOST,     // Gps fix lost, aborting mission
    NAV_ERROR_DISARMED,         // NAV engine disabled due disarm
    NAV_ERROR_LANDING           // Landing
};

typedef struct
{
    uint8_t number;     // Waypoint number
    int32_t pos[2];     // GPS position
    uint8_t action;     // Action to follow
    int16_t parameter1; // Parameter for the action
    int16_t parameter2; // Parameter for the action
    int16_t parameter3; // Parameter for the action
    uint32_t altitude;  // Altitude in cm (AGL)
    uint8_t flag;       // flags the last wp and other fancy things that are not yet defined
    uint8_t checksum;   // this must be at the last position
#if !defined(USE_EX_EEPROM)
    uint8_t checksum_; // 字符填充
#endif
} mission_step_struct;

typedef struct
{
    // Don't forget to change the reply size in GUI when change this struct;

    // on/off flags
    //	uint8_t filtering1 : 1;//内存对齐
    //	uint8_t filtering2 : 1;
    //	uint8_t filtering3 : 1;
    //	uint8_t filtering4 : 1;
    //	uint8_t filtering5 : 1;
    //	uint8_t filtering6 : 1;
    // First byte
    uint8_t filtering : 1;
    uint8_t lead_filter : 1;
    uint8_t dont_reset_home_at_arm : 1;
    uint8_t nav_controls_heading : 1;
    uint8_t nav_tail_first : 1;
    uint8_t nav_rth_takeoff_heading : 1;
    uint8_t slow_nav : 1;
    uint8_t wait_for_rth_alt : 1;
    // Second byte
    uint8_t ignore_throttle : 1; // Disable stick controls during mission and RTH
    uint8_t takeover_baro : 1;
    uint16_t wp_radius;        // in cm
    uint16_t safe_wp_distance; // in meter
    uint16_t nav_max_altitude; // in meter
    uint16_t nav_speed_max;    // in cm/s
    uint16_t nav_speed_min;    // in cm/s
    uint8_t crosstrack_gain;   // * 100 (0-2.56)
    uint16_t nav_bank_max;     // degree * 100; (3000 default)
    uint16_t rth_altitude;     // in meter
    uint8_t land_speed;        // between 50 and 255 (100 approx = 50cm/sec)
    uint8_t min_nav_vario;     // in cm/s
    uint16_t fence;            // fence control in meters
    uint8_t max_wp_number;
    int16_t dorp_delay_ms;      // 投放机构延迟，正数增加，负数减小
    uint16_t dorp_servor_open;  // 投放舵机舵量
    uint16_t dorp_servor_close; // 投放舵机舵量
    //		uint16_t cadc_pan[2];						//锁定云台
    uint8_t checksum;
} gps_conf_struct;
typedef struct
{
    uint8_t drop_wp, drop_status;
} user_mission_dorp_;
#endif
//////////////
typedef struct
{
    int16_t master_speed; // GPS速度
    int32_t GPS_[2];      // GPS信息
    int16_t GPS_ALT;      // 编队高度
    int16_t distance;     // 编队距离
    int16_t faction;      // 编队模式
    uint8_t flag;         // 数据帧标志
} mission_flow_;

#endif /* TYPES_H_ */
