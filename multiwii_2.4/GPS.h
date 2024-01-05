#ifndef GPS_H_
#define GPS_H_

// Function prototypes for GPS frame parsing
#include "config.h"
// Based on average Earth radius: lat_lon_to_cm = pi/180 * Earth_radius / 10^7, where volumetric mean radius 6371 km
#define LAT_LON_TO_CM 1.11318845f

// extern int32_t gpsPositionError[2];
extern int32_t gpsDistanceToHome[2];
extern int16_t gpsActualSpeed[2];

extern uint8_t GPS_Frame; // a valid GPS_Frame was detected, and data is ready for nav computation

extern int32_t wrap_18000(int32_t ang);
#if GPS
extern mission_flow_ mission_flow;
extern user_mission_dorp_ user_mission_dorp; // ��Ͷ�ṹ��
#endif
void GPS_set_pids(void);
void GPS_SerialInit(void);
uint8_t GPS_Compute(void);
void GPS_reset_home_position(void);
void GPS_set_next_wp(int32_t *lat_to, int32_t *lon_to, int32_t *lat_from, int32_t *lon_from);
void GPS_reset_nav(void);

int32_t get_altitude_error();
void clear_new_altitude();
void force_new_altitude(int32_t _new_alt);
void set_new_altitude(int32_t _new_alt);
int32_t get_new_altitude();
void abort_mission(unsigned char error_code);
void GPS_adjust_heading();
void init_RTH(void);
void check_land(void);
// bool GPS_newFrame(uint8_t c);
uint8_t GPS_newFrame(uint8_t data);
#if defined(I2C_GPS)
uint8_t GPS_NewData(void);
#endif
uint8_t GPS_newFrame(uint8_t data);
extern uint32_t wp_distance;
extern int32_t target_bearing;
extern int16_t servo0, servo5;
void CradleControl(int16_t altitude);

#if (defined(FIXEDWING) || defined(AIRPLANE)) && (defined(GPS_SERIAL) || defined(I2C_GPS))

void FW_NavSpeed(void);
/*****************************************/
/*   Settings for FixedWing navigation   */
/*****************************************/

// Use the Patched I2C GPS for FixedWing and OSD
// #define I2CPATCH
/*
   Values set in GUI.
   Set ABS Target Alt for RTL over home position.
   RTH_Alt is set with (POSR) => D

   for Navigation      (NavR) => P,I & D
   for Altitue.        (ALT)  => P, I &D
*/

#define GPS_UPD_HZ 10            // Set loop time for NavUpdate
#define PITCH_COMP 0.52f         // Compensate throttle relative angle of attack
#define ELEVATORCOMPENSATION 100 // Compensate elevator with % of rollAngle

/* Maximum Limits for controls */
#define GPS_MAXCORR 42 // 25   // Degrees banking applied by GPS.
#define GPS_RUDDER 18  //

#define GPS_MAXCLIMB 30 // Degrees climbing . To much can stall the plane.
#define GPS_MAXDIVE 23  // Degrees Diving . To much can overspeed the plane.

#define CLIMBTHROTTLE 1900  // Max allowed throttle in GPS modes.
#define CRUICETHROTTLE 1400 // Throttle to set for cruisespeed.

#define IDLE_THROTTLE 1200 // Lowest throttleValue during Descend
#define SCALER_THROTTLE 25 // Adjust to Match Power/Weight ratio of your model

// #define FAILSAFE              // Enable RTH failsafe incl Auto DisARM at home to autoland

#define SAFE_NAV_ALT 25 // Safe Altitude during climbouts Wings Level below this Alt. (ex. trees & buildings..)
// #define SAFE_DECSCEND_ZONE  50  // Radius around home where descending is OK
#endif
#endif /* GPS_H_ */
