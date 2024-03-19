#if defined(STM32F10X_MD)
#endif
#if defined(GD32F330)
#include "gd32f3x0.h"
#endif
#include "math.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "Sensors.h"
#include "MultiWii.h"
#include "math_.h"
#include "INS.h"
#include "GPS.h"
#include "AltHold.h"

#define INS_UPDATE_RATE 50 // 50hz update rate

/********** BARO ***********/
#if BARO

// important parameter for INS to calculate correction through the history for Z velocity (vario) and position (altitude)
#define BARO_LAG 200 // ~200ms for ms5611 baro

#define HIST_Z_POINTS (BARO_LAG / HZ2MS(INS_UPDATE_RATE)) // delay = BARO_LAG = HIST_Z_POINTS * HZ2MS(INS_UPDATE_RATE)
uint8_t histZCount;
int32_t histZPosition[HIST_Z_POINTS];
// int16_t histVario[HIST_Z_POINTS];

bool calculateZ_INS()
{

    static multiwii_timer_t calculateZ_INSTimer;
    static bool isActivated = FALSE;
    float dt;
    if (updateTimer(&calculateZ_INSTimer, HZ2US(INS_UPDATE_RATE)))
    {

        // debug[0] = calculateZ_INSTimer.dTime/10;

        calculateRAWAltitude(); //===??ζ???????? 20ms?===========

        updateAccelEF_Filtered(ALT); //==????????????????(20ms)?е?ACC(4ms???????????)??????===================

        if (f.ARMED)
        {

            if (!isActivated)
            {
                isActivated = TRUE;
                resetZState(); //==??Armed??????????=====histZCount = 0;...======
            }

            dt = US2S(calculateZ_INSTimer.dTime); // dt in sec

            // calculateRAWVario(&dt);

            correctZStateWithBaro(&dt); //==???????-??????? ( histZPosition[histZCount]) ????==?????????!!!===========

            updateZState(&dt); //===??ACC???????????????????????ι??????????????????????==============

            saveZPositionToHistory();

            return TRUE;
        }
        else
        {
            if (isActivated)
            {
                isActivated = FALSE;
            }
        }
    }

    return FALSE;
}

// Set the Acc weight for Acc/Baro complementary filter (CF)
// Increasing this value will reduce and delay Baro influence on the output of the filter.
#ifdef KILL_VIBRO
#define INS_Z_CONVERGE_FACTOR 3.0f
#else
// #define INS_Z_CONVERGE_FACTOR   5.0f
#define INS_Z_CONVERGE_FACTOR 3.5f
#endif
#define INS_Z_POS_FACTOR (3.0f / INS_Z_CONVERGE_FACTOR)
// #define INS_Z_VEL_FACTOR        (3.0f / (INS_Z_CONVERGE_FACTOR * INS_Z_CONVERGE_FACTOR))
#define INS_Z_VEL_FACTOR (3.0f / (INS_Z_CONVERGE_FACTOR * INS_Z_CONVERGE_FACTOR))
#define INS_Z_ACC_FACTOR (1.0f / (INS_Z_CONVERGE_FACTOR * INS_Z_CONVERGE_FACTOR * INS_Z_CONVERGE_FACTOR))

void correctZStateWithBaro(float *dt)
{

    // calculate error in position/vario from baro with our estimate

    // reduce effect of air-cushion, i.e. influence of alt.rawAlt on take off and landing,
    // because baro altitude value is dropped for 3-5m near the ground, i.e. it's not correct
    bool isAirCushionEffectDetected = (isTakeOffInProgress() || isGroundDetected()) //==????Ч?=
                                      && (alt.rawAlt < alt.groundRawAlt);

    float posError = (isAirCushionEffectDetected ? alt.groundRawAlt : alt.rawAlt) - histZPosition[histZCount];

    // float varioError = ((isAirCushionEffectDetected && (alt.rawVario < 0)) ? 0.0f : alt.rawVario)
    //                       - histVario[histZCount];

    /* !!! history z-position/vario and raw alt/vario baro values should be at the same phase
     * i.e. looks the same on chart and depends on HIST_Z_POINTS !!! */

    // accCorrection[ALT] += altError * INS_Z_ACC_FACTOR * *dt;
    ins.accCorrection[ALT] += posError * (INS_Z_ACC_FACTOR * *dt); //===???=======
    ins.velocityEF[ALT] += posError * (INS_Z_VEL_FACTOR * *dt);
    // ins.velocityEF[ALT] += varioError * (INS_Z_VEL_FACTOR * *dt);

    ins.positionEF[ALT] += posError * (INS_Z_POS_FACTOR * *dt);
}

void updateZState(float *dt)
{
    float velocityIncrease;
    // calculate velocity increase adding new acceleration from accelerometers
    // float velocityIncrease = (ins.accelEF_Filtered[ALT] + accCorrection[ALT]) * *dt;
    //	float velocityIncrease = ins.accelEF_Filtered[ALT] * *dt;
    ins.lastAcceleration[ALT] = ins.acceleration[ALT];                                  //===???=======
    ins.acceleration[ALT] = ins.accelEF_Filtered[ALT] + ins.accCorrection[ALT];         //====???=====
    velocityIncrease = (ins.lastAcceleration[ALT] + ins.acceleration[ALT]) * *dt / 2.0; //====???==

    // calculate new estimate of position //===??ACC?????????============
    ins.positionEF[ALT] += (ins.velocityEF[ALT] + velocityIncrease * 0.5) * *dt; //==combined by Acc and Bora==========
    alt.EstAlt = ins.positionEF[ALT];

    // calculate new velocity
    ins.velocityEF[ALT] += velocityIncrease; //===??ACC???????????????==============
    alt.vario = ins.velocityEF[ALT];

    //	debug[0] = alt.vario; // Positive when moving up
    // debug[0] = alt.rawAlt;
    //	debug[0] = ins.accelEF_Filtered[ALT];
    //	debug[1] = ins.acceleration[ALT]; // Positive when moving up
    //	debug[2] = alt.EstAlt;
    //	debug[3]=		alt.rawAlt;
}

void saveZPositionToHistory()
{
    // store 3rd order estimate (i.e. estimated vertical position) for future use at 50hz (20ms)
    histZPosition[histZCount] = alt.EstAlt; // ins.positionEF[ALT];
    // histVario[histZCount] = alt.vario;   //ins.velocityEF[ALT];

    histZCount++;
    if (histZCount >= HIST_Z_POINTS)
    {
        histZCount = 0;
    }
}

void resetZState()
{
    uint8_t i;
    // reset ins Z params at ARM event
    ins.positionEF[ALT] = 0.0f;
    ins.velocityEF[ALT] = 0.0f;

    // alt.lastRawAlt = alt.rawAlt;
    // alt.vario = 0;

    histZCount = 0;
    for (i = 0; i < HIST_Z_POINTS; i++)
    {
        // histZPosition[i] = alt.EstAlt; // reset history to current averaged raw alt
        histZPosition[i] = 0;
        // histVario[i] = 0;
    }
}

void calculateRAWAltitude()
{

    static float baroGroundTemperatureScale = 0, logBaroGroundPressureSum = 0;
    static int32_t last_alt_raw = 0;
    if (!f.ARMED)
    {
        logBaroGroundPressureSum = log(baroPressureSum);
        baroGroundTemperatureScale = (baroTemperature + 27315) * 29.271267f;
    }

    // baroGroundPressureSum is not supposed to be 0 here
    // see: https://code.google.com/p/ardupilot-mega/source/browse/libraries/AP_Baro/AP_Baro.cpp
    alt.rawAlt = (logBaroGroundPressureSum - log(baroPressureSum)) * baroGroundTemperatureScale;
    if (ABS(alt.rawAlt - last_alt_raw) > 100)
    {
        // debug[2]++;
        alt.rawAlt = last_alt_raw;
    }
    else
    {
        last_alt_raw = alt.rawAlt;
    }

    // debug[2] = (int16_t)baroPressureSum;
    // debug[3] = (int16_t)alt.rawAlt;
}

/*void calculateRAWVario(float* dt) {
    // get dirty (but w/o drift) velocity from baro
    alt.rawVario = (alt.rawAlt - alt.lastRawAlt) / *dt;
    alt.rawVario = constrain(alt.rawVario, -600, 600); // constrain baro raw velocity +/- 6m/s
    alt.lastRawAlt = alt.rawAlt;
}*/

#endif

/******************************** GPS ********************************************/

#if GPS && defined(INS_PH_NAV_ON)
#define GPS_LAG 1
#define HIST_XY_POINTS (GPS_LAG / HZ2MS(INS_UPDATE_RATE))
uint8_t histXYCount;
int32_t histXYPosition[2][HIST_XY_POINTS];

bool calculateXY_INS()
{

    static multiwii_timer_t calculateXY_INSTimer;

    if (updateTimer(&calculateXY_INSTimer, HZ2US(INS_UPDATE_RATE)))
    {

        // debug[1] = calculateXY_INSTimer.dTime/10;
        float dt;
        static bool isActivated = FALSE;

        updateAccelEF_Filtered(LAT); //===??????????????ACC=======
        updateAccelEF_Filtered(LON);

        if (
#ifdef DISABLE_INS_WHEN_PH_OFF
            isNavStateForPosHold() &&
#endif
            f.ARMED && f.GPS_FIX && (GPS_numSat >= 4) && f.GPS_FIX_HOME)
        {

            if (!isActivated)
            {
                isActivated = TRUE;

                resetXYState();
            }

            dt = US2S(calculateXY_INSTimer.dTime); // dt in sec

            correctXYStateWithGPS(&dt); //==????????????ι?????????????????????????λ??===========================

            updateXYState(&dt); //==????ACC????????????????????????????????????λ??===========================

            saveXYPositionToHistory(); //===?????μ???????????λ????′�?????===========

            return TRUE;
        }
        else
        {
            if (isActivated)
            {
                isActivated = FALSE;
            }
        }

        // debug[0] = gpsActualSpeed[LAT]; // Positive when moving North
        // debug[1] = -(int16_t)gpsPositionError[LAT]; // Negative when moving North (LAT)
        // debug[2] = (int16_t)ins.velocityEF[LAT]; // Positive when moving North
        // debug[3] = (int16_t)ins.positionEF[LAT]; // Positive when moving North (LAT)
    }

    return FALSE;
}

// Set the Acc weight for Acc/GPS complementary filter (CF)
// Increasing this value will reduce and delay GPS influence on the output of the filter.
#ifdef KILL_VIBRO
#define INS_CONVERGE_FACTOR 2.0f
#else
#define INS_CONVERGE_FACTOR 2.5f
#endif
#define INS_POS_FACTOR (3.0f / INS_CONVERGE_FACTOR)
#define INS_VEL_FACTOR (3.0f / (INS_CONVERGE_FACTOR * INS_CONVERGE_FACTOR))
// #define INS_ACC_FACTOR        (1.0f / (INS_CONVERGE_FACTOR * INS_CONVERGE_FACTOR * INS_CONVERGE_FACTOR))

void correctXYStateWithGPS(float *dt)
{

    float posError[2];
    float tmp;
    // calculate error in position from gps with our historical estimate
    posError[LAT] = gpsDistanceToHome[LAT] - histXYPosition[LAT][histXYCount];
    posError[LAT] = constrain(posError[LAT], -1000, 1000); // todo: temp gps glitch protection for diff > 10 meters
    posError[LON] = gpsDistanceToHome[LON] - histXYPosition[LON][histXYCount];
    posError[LON] = constrain(posError[LON], -1000, 1000); // todo: temp gps glitch protection for diff > 10 meters

    // tmp = INS_ACC_FACTOR * *dt;
    // accCorrection[LAT] += posError[LAT] * tmp;
    // accCorrection[LON] += posError[LON] * tmp;

    tmp = INS_VEL_FACTOR * *dt;
    ins.velocityEF[LAT] += posError[LAT] * tmp; //==???????====
    ins.velocityEF[LON] += posError[LON] * tmp;

    tmp = INS_POS_FACTOR * *dt;
    ins.positionEF[LAT] += posError[LAT] * tmp; //==??????====
    ins.positionEF[LON] += posError[LON] * tmp;
}

void updateXYState(float *dt)
{

    // calculate velocity increase adding new acceleration from accelerometers
    float velocityIncrease[2];
    // velocityIncrease[LAT] = (accelEF_Filtered[LAT] + accCorrection[LAT]) * dt;
    // velocityIncrease[LON] = (accelEF_Filtered[LON] + accCorrection[LON]) * dt;
    velocityIncrease[LAT] = ins.accelEF_Filtered[LAT] * *dt;
    velocityIncrease[LON] = ins.accelEF_Filtered[LON] * *dt;

    // calculate new estimate of position
    ins.positionEF[LAT] += (ins.velocityEF[LAT] + velocityIncrease[LAT] * 0.5) * *dt;
    ins.positionEF[LON] += (ins.velocityEF[LON] + velocityIncrease[LON] * 0.5) * *dt;

    // calculate new velocity
    ins.velocityEF[LAT] += velocityIncrease[LAT];
    ins.velocityEF[LON] += velocityIncrease[LON];

    // debug[0] = gpsActualSpeed[LAT]; // Positive when moving North
    // debug[1] = gpsActualSpeed[LON]; // Positive when moving East
    // debug[2] = (int16_t)ins.velocityEF[LAT]; // Positive when moving North
    // debug[3] = (int16_t)ins.velocityEF[LON]; // Positive when moving East

    // debug[0] = (int16_t)gpsPositionError[LAT]; // Negative when moving North (LAT)
    // debug[1] = (int16_t)gpsPositionError[LON]; // Negative when moving East (LON)
    // debug[2] = (int16_t)ins.positionEF[LAT]; // Positive when moving North (LAT)
    // debug[3] = (int16_t)ins.positionEF[LON]; // Positive when moving East (LON)
}

void saveXYPositionToHistory()
{
    // store 3rd order estimate (i.e. horizontal position) for future use at 50hz (20ms)
    histXYPosition[LAT][histXYCount] = ins.positionEF[LAT];
    histXYPosition[LON][histXYCount] = ins.positionEF[LON];

    histXYCount++;
    if (histXYCount >= HIST_XY_POINTS)
    {
        histXYCount = 0;
    }
}

void resetXYState()
{
    // reset ins nav params
    uint8_t i, j;
    histXYCount = 0;

    for (i = 0; i < 2; i++)
    {
        ins.velocityEF[i] = (abs(gpsActualSpeed[i]) > 50) ? gpsActualSpeed[i] : 0.0f;
        ins.positionEF[i] = gpsDistanceToHome[i];

        for (j = 0; j < HIST_XY_POINTS; j++)
        {
            histXYPosition[i][j] = gpsDistanceToHome[i];
        }
    }
}

int32_t getGPS_poshold_LON()
{
    // return GPS_hold[LON] + (int32_t)(positionEF[LON]/GPS_scaleLonDown);
    return (int32_t)ins.positionEF[LON];
}

int32_t getGPS_poshold_LAT()
{
    // return GPS_hold[LAT] + (int32_t)positionEF[LAT];
    return (int32_t)ins.positionEF[LAT];
}

#endif

#define ACC_BIAS_FACTOR 0.9987f
#define ACC_BIAS_FACTOR_NO_ARM 0.985f

#define ACC_Scale (980.665f / ACC_1G); // to cm/s^2

float cy = 0, sy = 0;

// do rotation of acceleration X/Y/Z to earth frame
void rotateAccelToEarthFrame()
{

    float cr = 0, sr = 0, cp = 0, sp = 0, spcy = 0, spsy = 0;
    float diff = 0;
    static float accelBias[3] = {0.0, 0.0, 0.0};
    uint8_t axis = 0;
    static float accelEF_LPF[3] = {0.0, 0.0, 0.0};
    cr = cos_approx(att.angle[ROLL]);
    sr = sin_approx(att.angle[ROLL]);
    cp = cos_approx(-att.angle[PITCH]);
    sp = sin_approx(-att.angle[PITCH]);
    cy = cos_approx(att.angle_YAW);
    sy = sin_approx(att.angle_YAW);

    spcy = sp * cy;
    spsy = sp * sy;
    ins.accelEF[LAT] = -((cp * cy) * imu.accADC[PITCH] + (sr * spcy - cr * sy) * imu.accADC[ROLL] + (sr * sy + cr * spcy) * imu.accADC[YAW]);  // Positive when moving North
    ins.accelEF[LON] = -((cp * sy) * imu.accADC[PITCH] + (cr * cy + sr * spsy) * imu.accADC[ROLL] + (-sr * cy + cr * spsy) * imu.accADC[YAW]); // Positive when moving East
    // more precise accZ comparing with calculated in IMU.ino
    ins.accelEF[ALT] = ((-sp) * imu.accADC[PITCH] + (sr * cp) * imu.accADC[ROLL] + (cr * cp) * imu.accADC[YAW]) - ACC_1G;

    for (axis = 0; axis < 3; axis++)
    {

        // scaled XYZ acceleration cm/s^2
        ins.accelEF[axis] = ins.accelEF[axis] * ACC_Scale;

        // high pass filter: very useful feature! In-flight acc XYZ axis calibration. It helps to avoid acc temperature drift and find bias.
        diff = ins.accelEF[axis] - accelBias[axis];
        if (!f.ARMED)
        {
            accelBias[axis] = accelBias[axis] * ACC_BIAS_FACTOR_NO_ARM + ins.accelEF[axis] * (1.0f - ACC_BIAS_FACTOR_NO_ARM);
        }
        else if (abs(diff) <= 80.0f)
        { // if diff near zero (<= 80 cm/s^2) than we made soft bias correction
            accelBias[axis] = accelBias[axis] * ACC_BIAS_FACTOR + ins.accelEF[axis] * (1.0f - ACC_BIAS_FACTOR);
        }
        ins.accelEF[axis] = diff;

        // remove small noise near zero position, by default 10 cm/s^2
        // accelEF[axis] = applyDeadband(accelEF[axis], ACC_DEADBAND);

#ifdef ACC_EF_LPF_FACTOR

        accelEF_LPF[axis] = accelEF_LPF[axis] * (1.0f - (1.0f / ACC_EF_LPF_FACTOR)) + ins.accelEF[axis] * (1.0f / ACC_EF_LPF_FACTOR);
        ins.accelEF_Sum[axis] += accelEF_LPF[axis];
#else
        ins.accelEF_Sum[axis] += ins.accelEF[axis];
#endif
        ins.accelEF_Sum_count[axis]++;
    }

    // debug[0] = ins.accelEF[LAT]; // Positive when moving North
    // debug[1] = ins.accelEF[LON]; // Positive when moving East
    // debug[2] = ins.accelEF[ALT]; // Positive when moving Up

    // debug[0] = accelBias[LAT];
    // debug[1] = accelBias[LON];
    // debug[2] = accelBias[ALT];
}

void updateAccelEF_Filtered(uint8_t axis)
{
    ins.accelEF_Filtered[axis] = ins.accelEF_Sum[axis] / ins.accelEF_Sum_count[axis]; //===rotateAccelToEarthFrame()?и???=

    // reset sum
    ins.accelEF_Sum[axis] = 0.0f;
    ins.accelEF_Sum_count[axis] = 0;
}
