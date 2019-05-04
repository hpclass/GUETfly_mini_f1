#include "stm32f10x.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Alarms.h"
#include "delay.h"
#include "timer.h"


void initializeServo();
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/**************************************************************************************/
/***************                  Motor Pin order                  ********************/
/**************************************************************************************/
// since we are uing the PWM generation in a direct way, the pin order is just to inizialie the right pins
// its not possible to change a PWM output pin just by changing the order
uint8_t PWM_PIN[8] = {6,7,0,1,6,5,2,12};
#if defined(PROMINI)
// uint8_t PWM_PIN[8] = {9,10,11,3,6,5,2,12};   //for a quad+: rear,right,left,front
#endif
#if defined(PROMICRO)
#if !defined(HWPWM6)
#if defined(TEENSY20)
uint8_t PWM_PIN[8] = {14,15,9,12,22,18,16,17};   //for a quad+: rear,right,left,front
#elif defined(A32U4_4_HW_PWM_SERVOS)
uint8_t PWM_PIN[8] = {6,9,10,11,5,13,SW_PWM_P3,SW_PWM_P4};   //
#else
uint8_t PWM_PIN[8] = {9,10,5,6,4,A2,SW_PWM_P3,SW_PWM_P4};   //for a quad+: rear,right,left,front
#endif
#else
#if defined(TEENSY20)
uint8_t PWM_PIN[8] = {14,15,9,12,4,10,16,17};   //for a quad+: rear,right,left,front
#elif defined(A32U4_4_HW_PWM_SERVOS)
uint8_t PWM_PIN[8] = {6,9,10,11,5,13,SW_PWM_P3,SW_PWM_P4};   //
#else
uint8_t PWM_PIN[8] = {9,10,5,6,11,13,SW_PWM_P3,SW_PWM_P4};   //for a quad+: rear,right,left,front
#endif
#endif
#endif
#if defined(MEGA)
uint8_t PWM_PIN[8] = {3,5,6,2,7,8,9,10};      //for a quad+: rear,right,left,front   //+ for y6: 7:under right  8:under left
#endif

/**************************************************************************************/
/***************         Software PWM & Servo variables            ********************/
/**************************************************************************************/
#if defined(PROMINI) || (defined(PROMICRO) && defined(HWPWM6)) || (defined(MEGA) && defined(MEGA_HW_PWM_SERVOS))
#if (NUMBER_MOTOR > 4)
//for HEX Y6 and HEX6/HEX6X/HEX6H flat for promini
volatile uint8_t atomicPWM_PIN5_lowState;
volatile uint8_t atomicPWM_PIN5_highState;
volatile uint8_t atomicPWM_PIN6_lowState;
volatile uint8_t atomicPWM_PIN6_highState;
#endif
#if (NUMBER_MOTOR > 6)
//for OCTO on promini
volatile uint8_t atomicPWM_PINA2_lowState;
volatile uint8_t atomicPWM_PINA2_highState;
volatile uint8_t atomicPWM_PIN12_lowState;
volatile uint8_t atomicPWM_PIN12_highState;
#endif
#else
#if (NUMBER_MOTOR > 4)
//for HEX Y6 and HEX6/HEX6X/HEX6H and for Promicro
volatile uint16_t atomicPWM_PIN5_lowState;
volatile uint16_t atomicPWM_PIN5_highState;
volatile uint16_t atomicPWM_PIN6_lowState;
volatile uint16_t atomicPWM_PIN6_highState;
#endif
#if (NUMBER_MOTOR > 6)
//for OCTO on Promicro
volatile uint16_t atomicPWM_PINA2_lowState;
volatile uint16_t atomicPWM_PINA2_highState;
volatile uint16_t atomicPWM_PIN12_lowState;
volatile uint16_t atomicPWM_PIN12_highState;
#endif
#endif

#if defined(SERVO)
#if defined(HW_PWM_SERVOS)
// hw servo pwm does not need atomicServo[]
#elif defined(PROMINI) || (defined(PROMICRO) && defined(HWPWM6))
#if defined(AIRPLANE) || defined(HELICOPTER)
// To prevent motor to start at reset. atomicServo[7]=5 or 249 if reversed servo
volatile uint8_t atomicServo[8] = {125,125,125,125,125,125,125,5};
#else
volatile uint8_t atomicServo[8] = {125,125,125,125,125,125,125,125};
#endif
#else
#if defined(AIRPLANE)|| defined(HELICOPTER)
// To prevent motor to start at reset. atomicServo[7]=5 or 249 if reversed servo
volatile uint16_t atomicServo[8] = {8000,8000,8000,8000,8000,8000,8000,320};
#else
volatile uint16_t atomicServo[8] = {8000,8000,8000,8000,8000,8000,8000,8000};
#endif
#endif
#endif

/**************************************************************************************/
/***************       Calculate first and last used servos        ********************/
/**************************************************************************************/
#if defined(SERVO)
#if defined(PRI_SERVO_FROM) && defined(SEC_SERVO_FROM)
#if PRI_SERVO_FROM < SEC_SERVO_FROM
#define SERVO_START PRI_SERVO_FROM
#else
#define SERVO_START SEC_SERVO_FROM
#endif
#else
#if defined(PRI_SERVO_FROM)
#define SERVO_START PRI_SERVO_FROM
#endif
#if defined(SEC_SERVO_FROM)
#define SERVO_START SEC_SERVO_FROM
#endif
#endif
#if defined(PRI_SERVO_TO) && defined(SEC_SERVO_TO)
#if PRI_SERVO_TO > SEC_SERVO_TO
#define SERVO_END PRI_SERVO_TO
#else
#define SERVO_END SEC_SERVO_TO
#endif
#else
#if defined(PRI_SERVO_TO)
#define SERVO_END PRI_SERVO_TO
#endif
#if defined(SEC_SERVO_TO)
#define SERVO_END SEC_SERVO_TO
#endif
#endif


#endif

/**************************************************************************************/
/***************   Writes the Servos values to the needed format   ********************/
/**************************************************************************************/
void writeServos() {

//		uint8_t i ;
//    for (i =0; i<8; i++) {
//        servo[i]=map(servo[i],1000,2000,100,200);
//    }

//    //PC6、7、8、9
//    TIM8->CCR4=servo[0];//PC9
//    TIM8->CCR3=servo[1];//PC8
//    TIM8->CCR2=servo[2];//PC7
//    TIM8->CCR1=servo[3];//PC6
//    //PD12,13,14,15
//
//    TIM4->CCR4=servo[4];//PD12
//    TIM4->CCR3=servo[5];//PD13
//    TIM4->CCR2=servo[6];//PD14
//    TIM4->CCR1=servo[7];//PD15
    //    //PC6、7、8、9
//    TIM8->CCR4=servo[0];//PC9
//    TIM8->CCR3=servo[1];//PC8
//    TIM8->CCR2=servo[3];//PC7
//    TIM8->CCR1=servo[4];//PC6
    //PD12,13,14,15

    TIM4->CCR1=servo[5];//PD12
    TIM4->CCR2=servo[6];//PD13
    TIM4->CCR3=servo[3];//PD14
    TIM4->CCR4=servo[4];//PD15
}

/**************************************************************************************/
/************  Writes the Motors values to the PWM compare register  ******************/
/**************************************************************************************/
void writeMotors() { // [1000;2000] => [125;250]
    TIM_SetCompare1(TIM3,motor[0]);
    TIM_SetCompare2(TIM3,motor[1]);
    TIM_SetCompare3(TIM3,motor[2]);
    TIM_SetCompare4(TIM3,motor[3]);


}

/**************************************************************************************/
/************          Writes the mincommand to all Motors           ******************/
/**************************************************************************************/
void writeAllMotors(int16_t mc) {   // Sends commands to all motors
    uint8_t i ;
    for (i =0; i<NUMBER_MOTOR; i++) {
        motor[i]=mc;
    }
    writeMotors();
}

/**************************************************************************************/
/************        Initialize the PWM Timers and Registers         ******************/
/**************************************************************************************/
void initOutput() {
    uint8_t axis=0;
    for(axis=0; axis<RC_CHANS; axis++)
        rcData[axis]=1500;
    // TIM8_PWM_Init(19999,72-1);//高级定时器 PC6、7、8、9 400Hz,如果需要50Hz修改72为720-1
    //TIM2_PWM_Init(1999,720);//PA0,1,2,3
    TIM3_PWM_Init(1999,72); //400Hz (0-2000) PA6,7,PB0,1
    TIM4_PWM_Init(19999,72-1);//PD12,13,14,15
    /****************            mark all PWM pins as Output             ******************/
// TIM3_PWM_Init(999,143); //、、初始化PWM

    /********  special version of MultiWii to calibrate all attached ESCs ************/
#if defined(ESC_CALIB_CANNOT_FLY)
    writeAllMotors(ESC_CALIB_HIGH);
    blinkLED(2,20, 2);
    delay(4000);
    writeAllMotors(ESC_CALIB_LOW);
    blinkLED(3,20, 2);
    while (1) {
        delay(5000);
        blinkLED(4,20, 2);
        SET_ALARM_BUZZER(ALRM_FAC_CONFIRM, ALRM_LVL_CONFIRM_2);
    }
    exit(); // statement never reached
#endif

    writeAllMotors(MINCOMMAND);
    //delay(300);
#if defined(SERVO)
    initializeServo();
#endif
}


#if defined(SERVO)
/**************************************************************************************/
/************                Initialize the PWM Servos               ******************/
/**************************************************************************************/
void initializeServo() {
    uint8_t i=0;

#if defined(SERVO_RFR_300HZ)

#else // if there are less then 3 servos we need to slow it to not go over 300Hz (the highest working refresh rate for the digital servos for what i know..)

#endif

#if defined(SERVO_RFR_160HZ)
#endif

#if defined(SERVO_RFR_50HZ) // to have ~ 50Hz for all servos

#endif

}
#endif



/**************************************************************************************/
/********** Mixes the Computed stabilize values to the Motors & Servos  ***************/
/**************************************************************************************/
//extern int16_t rcData[RC_CHANS];//stm32 add
// get servo middle point from Config or from RC-Data
int16_t get_middle(uint8_t nr) {
    if(ABS(conf.servoConf[nr].middle) < RC_CHANS)
        return rcData[conf.servoConf[nr].middle];
    else
        return conf.servoConf[nr].middle;
//    return (conf.servoConf[nr].middle < RC_CHANS) ? rcData[conf.servoConf[nr].middle] : conf.servoConf[nr].middle;
}

// int8_t servodir(uint8_t n, uint8_t b) { return ((conf.servoConf[n].rate & b) ? -1 : 1) ; }
int test[2] ;
void mixTable() {
    int16_t maxMotor;
    uint8_t i;
#if defined(DYNBALANCE)
    return;
#endif
#if defined(HELICOPTER)
    int16_t cRange[2] = CONTROL_RANGE;
    int16_t acmd;
#endif
#define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z
#define SERVODIR(n,b) ((conf.servoConf[n].rate & b) ? -1 : 1)

    /****************                   main Mix Table                ******************/
#if defined( MY_PRIVATE_MIXING )
#include MY_PRIVATE_MIXING
#elif defined( BI )
    motor[0] = PIDMIX(+1, 0, 0); //LEFT
    motor[1] = PIDMIX(-1, 0, 0); //RIGHT
    servo[4] = (SERVODIR(4,2) * axisPID[YAW]) + (SERVODIR(4,1) * axisPID[PITCH]) + get_middle(4); //LEFT
    servo[5] = (SERVODIR(5,2) * axisPID[YAW]) + (SERVODIR(5,1) * axisPID[PITCH]) + get_middle(5); //RIGHT
    //servo[4]=3000-servo[4];
#elif defined( TRI )
    motor[0] = PIDMIX( 0,+4/3, 0); //REAR
    motor[1] = PIDMIX(-1,-2/3, 0); //RIGHT
    motor[2] = PIDMIX(+1,-2/3, 0); //LEFT
    servo[5] = (SERVODIR(5, 1) * axisPID[YAW]) + get_middle(5); //REAR
#elif defined( QUADP )
    motor[0] = PIDMIX( 0,+1,-1); //REAR
    motor[1] = PIDMIX(-1, 0,+1); //RIGHT
    motor[2] = PIDMIX(+1, 0,+1); //LEFT
    motor[3] = PIDMIX( 0,-1,-1); //FRONT
#elif defined( QUADX )
    motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    motor[2] = PIDMIX(+1,+1,+1); //REAR_L
    motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
#elif defined( Y4 )
    motor[0] = PIDMIX(+0,+1,-1);   //REAR_1 CW
    motor[1] = PIDMIX(-1,-1, 0); //FRONT_R CCW
    motor[2] = PIDMIX(+0,+1,+1);   //REAR_2 CCW
    motor[3] = PIDMIX(+1,-1, 0); //FRONT_L CW
#elif defined( Y6 )
    motor[0] = PIDMIX(+0,+4/3,+1); //REAR
    motor[1] = PIDMIX(-1,-2/3,-1); //RIGHT
    motor[2] = PIDMIX(+1,-2/3,-1); //LEFT
    motor[3] = PIDMIX(+0,+4/3,-1); //UNDER_REAR
    motor[4] = PIDMIX(-1,-2/3,+1); //UNDER_RIGHT
    motor[5] = PIDMIX(+1,-2/3,+1); //UNDER_LEFT
#elif defined( HEX6 )
    motor[0] = PIDMIX(-7/8,+1/2,+1); //REAR_R
    motor[1] = PIDMIX(-7/8,-1/2,-1); //FRONT_R
    motor[2] = PIDMIX(+7/8,+1/2,+1); //REAR_L
    motor[3] = PIDMIX(+7/8,-1/2,-1); //FRONT_L
    motor[4] = PIDMIX(+0,-1,+1);     //FRONT
    motor[5] = PIDMIX(+0,+1,-1);     //REAR
#elif defined( HEX6X )
    motor[0] = PIDMIX(-1/2,+7/8,+1); //REAR_R
    motor[1] = PIDMIX(-1/2,-7/8,+1); //FRONT_R
    motor[2] = PIDMIX(+1/2,+7/8,-1); //REAR_L
    motor[3] = PIDMIX(+1/2,-7/8,-1); //FRONT_L
    motor[4] = PIDMIX(-1,+0,-1);     //RIGHT
    motor[5] = PIDMIX(+1,+0,+1);     //LEFT
#elif defined( HEX6H )
    motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    motor[2] = PIDMIX(+ 1,+1,+1); //REAR_L
    motor[3] = PIDMIX(+ 1,-1,-1); //FRONT_L
    motor[4] = PIDMIX(0,0,0);   //RIGHT
    motor[5] = PIDMIX(0,0,0);   //LEFT
#elif defined( OCTOX8 )
    motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    motor[2] = PIDMIX(+1,+1,+1); //REAR_L
    motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
    motor[4] = PIDMIX(-1,+1,+1); //UNDER_REAR_R
    motor[5] = PIDMIX(-1,-1,-1); //UNDER_FRONT_R
    motor[6] = PIDMIX(+1,+1,-1); //UNDER_REAR_L
    motor[7] = PIDMIX(+1,-1,+1); //UNDER_FRONT_L
#elif defined( OCTOFLATP )
    motor[0] = PIDMIX(+7/10,-7/10,+1); //FRONT_L
    motor[1] = PIDMIX(-7/10,-7/10,+1); //FRONT_R
    motor[2] = PIDMIX(-7/10,+7/10,+1); //REAR_R
    motor[3] = PIDMIX(+7/10,+7/10,+1); //REAR_L
    motor[4] = PIDMIX(+0,-1,-1);       //FRONT
    motor[5] = PIDMIX(-1,+0,-1);       //RIGHT
    motor[6] = PIDMIX(+0,+1,-1);       //REAR
    motor[7] = PIDMIX(+1,+0,-1);       //LEFT
#elif defined( OCTOFLATX )
    motor[0] = PIDMIX(+1,-1/2,+1);   //MIDFRONT_L
    motor[1] = PIDMIX(-1/2,-1,+1);   //FRONT_R
    motor[2] = PIDMIX(-1,+1/2,+1);   //MIDREAR_R
    motor[3] = PIDMIX(+1/2,+1,+1);   //REAR_L
    motor[4] = PIDMIX(+1/2,-1,-1);   //FRONT_L
    motor[5] = PIDMIX(-1,-1/2,-1);   //MIDFRONT_R
    motor[6] = PIDMIX(-1/2,+1,-1);   //REAR_R
    motor[7] = PIDMIX(+1,+1/2,-1);   //MIDREAR_L
#elif defined( VTAIL4 )
    motor[0] = PIDMIX(+0,+1, +1); //REAR_R
    motor[1] = PIDMIX(-1, -1, +0); //FRONT_R
    motor[2] = PIDMIX(+0,+1, -1); //REAR_L
    motor[3] = PIDMIX(+1, -1, -0); //FRONT_L
#elif defined( FLYING_WING )
    /*****************************             FLYING WING                **************************************/
    if (!f.ARMED) {
        servo[7] = MINCOMMAND;  // Kill throttle when disarmed
    } else {
        servo[7] = constrain(rcCommand[THROTTLE], conf.minthrottle, MAXTHROTTLE);
    }
    motor[0] = servo[7];
    if (f.PASSTHRU_MODE) {    // do not use sensors for correction, simple 2 channel mixing
        servo[3] = (SERVODIR(3,1) * rcCommand[PITCH]) + (SERVODIR(3,2) * -rcCommand[ROLL]);
        servo[4] = (SERVODIR(4,1) * -rcCommand[PITCH]) + (SERVODIR(4,2) * -rcCommand[ROLL]);
    } else {                  // use sensors to correct (gyro only or gyro+acc according to aux1/aux2 configuration
        servo[3] = (SERVODIR(3,1) * axisPID[PITCH])   + (SERVODIR(3,2) * -axisPID[ROLL]);
        servo[4] = (SERVODIR(4,1) * -axisPID[PITCH])   + (SERVODIR(4,2) * -axisPID[ROLL]);
    }
    servo[3] += get_middle(3);
    servo[4] += get_middle(4);
    servo[5] = get_middle(5)-axisPID[YAW];
#elif defined( FLYING_WING_J10 )
    /*****************************             FLYING WING                **************************************/
    if (!f.ARMED) {
        servo[7] = MINCOMMAND;  // Kill throttle when disarmed
    } else {
        servo[7] = constrain(rcCommand[THROTTLE], conf.minthrottle, MAXTHROTTLE);
    }
    motor[0] = servo[7];
    if (f.PASSTHRU_MODE) {    // do not use sensors for correction, simple 2 channel mixing
        servo[3] = (SERVODIR(3,1) * rcCommand[PITCH]) + (SERVODIR(3,2) * rcCommand[ROLL]);
        servo[4] = (SERVODIR(4,1) * rcCommand[PITCH]) + (SERVODIR(4,2) * rcCommand[ROLL]);
			
				servo[5] = (SERVODIR(5,1)	* -rcCommand[YAW]);
			
    } else {                  // use sensors to correct (gyro only or gyro+acc according to aux1/aux2 configuration
        servo[3] = (SERVODIR(3,1) * axisPID[PITCH])   + (SERVODIR(3,2) * axisPID[ROLL]);
        servo[4] = (SERVODIR(4,1) * axisPID[PITCH])   + (SERVODIR(4,2) * axisPID[ROLL]);
				servo[5] = (SERVODIR(5,1)	* -axisPID[YAW]);
    }
		int add_out_of_c[2]={0,0};
		
		if(servo[3]>500)//超出范围
		{
			add_out_of_c[0]=servo[3]-500;
		}else if(servo[3]<-500)//超出范围
		{
			add_out_of_c[0]=servo[3]+500;
		}
		
		
		if(servo[4]>500)//超出范围
		{
			add_out_of_c[1]=servo[4]-500;
		}else if(servo[4]<-500)//超出范围
		{
			add_out_of_c[1]=servo[4]+500;
		}
		if(add_out_of_c[0]!=0)
		{
			servo[4]+=SERVODIR(3,1)*SERVODIR(4,1)*-1*add_out_of_c[0];
		}
		if(add_out_of_c[1]!=0)
		{
			servo[3]+=SERVODIR(3,1)*SERVODIR(4,1)*-1*add_out_of_c[1];
		}
		test[0] =servo[3];
		test[1] =servo[4];
		
    servo[3] += get_middle(3);
    servo[4] += get_middle(4);
		servo[5] += get_middle(5);
		
#elif defined( FLYING_WING_D )
    /*****************************             FLYING WING                **************************************/
    if (!f.ARMED) {
        servo[7] = MINCOMMAND;  // Kill throttle when disarmed
    } else {
        servo[7] = constrain(rcCommand[THROTTLE], conf.minthrottle, MAXTHROTTLE);
    }

    motor[0] = servo[7]+constrain(axisPID[YAW],-DIFF_MOTOR_,+DIFF_MOTOR_);
    motor[1] = servo[7]-constrain(axisPID[YAW],-DIFF_MOTOR_,+DIFF_MOTOR_);
    int temp_thr_l=0;
    int temp_thr_r=0;
    int temp_thr__r=0;
    if(rcCommand[YAW]) {
        temp_thr_l=servo[7]+constrain(rcCommand[YAW],-DIFF_MOTOR_,+DIFF_MOTOR_);//获取油门差动补偿
        temp_thr_r=servo[7]-constrain(rcCommand[YAW],-DIFF_MOTOR_,+DIFF_MOTOR_);
        if(MINTHROTTLE>temp_thr_l)
        {
            //左发动机小于最小油门
            motor[0] = constrain(servo[7]+constrain(rcCommand[YAW],-DIFF_MOTOR_,+DIFF_MOTOR_),MINTHROTTLE,MAXTHROTTLE);
            motor[1] = constrain((MINTHROTTLE-temp_thr_l)+servo[7]-constrain(rcCommand[YAW],-DIFF_MOTOR_,+DIFF_MOTOR_),MINTHROTTLE,MAXTHROTTLE);
        } else if(MAXTHROTTLE<temp_thr_l)
        {
            //左发动机大于最大油门
            motor[0] = constrain(servo[7]+constrain(rcCommand[YAW],-DIFF_MOTOR_,+DIFF_MOTOR_),MINTHROTTLE,MAXTHROTTLE);
            motor[1] = constrain(servo[7]-(temp_thr_l-MAXTHROTTLE)-constrain(rcCommand[YAW],-DIFF_MOTOR_,+DIFF_MOTOR_),MINTHROTTLE,MAXTHROTTLE);
        } else if(MINTHROTTLE>temp_thr_r)
        {
            //右发动机小于最小油门
            motor[0] = constrain(MINTHROTTLE-temp_thr_r+servo[7]+constrain(rcCommand[YAW],-DIFF_MOTOR_,+DIFF_MOTOR_),MINTHROTTLE,MAXTHROTTLE);
            motor[1] = constrain(servo[7]-constrain(rcCommand[YAW],-DIFF_MOTOR_,+DIFF_MOTOR_),MINTHROTTLE,MAXTHROTTLE);

        } else if(MAXTHROTTLE<temp_thr_r)
        {
            //右发动机大于最大油门
            motor[0] = constrain(servo[7]-(temp_thr_r-MAXTHROTTLE)+constrain(rcCommand[YAW],-DIFF_MOTOR_,+DIFF_MOTOR_),MINTHROTTLE,MAXTHROTTLE);
            motor[1] = constrain(servo[7]-constrain(rcCommand[YAW],-DIFF_MOTOR_,+DIFF_MOTOR_),MINTHROTTLE,MAXTHROTTLE);


        } else {
            motor[0] = constrain(servo[7]+constrain(rcCommand[YAW],-DIFF_MOTOR_,+DIFF_MOTOR_),MINTHROTTLE,MAXTHROTTLE);
            motor[1] = constrain(servo[7]-constrain(rcCommand[YAW],-DIFF_MOTOR_,+DIFF_MOTOR_),MINTHROTTLE,MAXTHROTTLE);

        }
    } else {
        motor[0] = constrain(servo[7]+constrain(rcCommand[YAW],-DIFF_MOTOR_,+DIFF_MOTOR_),MINTHROTTLE,MAXTHROTTLE);
        motor[1] = constrain(servo[7]-constrain(rcCommand[YAW],-DIFF_MOTOR_,+DIFF_MOTOR_),MINTHROTTLE,MAXTHROTTLE);


    }
//    motor[0] = constrain(servo[7]+constrain(rcCommand[YAW],-DIFF_MOTOR_,+DIFF_MOTOR_),1000,2000);
//    motor[1] = constrain(servo[7]-constrain(rcCommand[YAW],-DIFF_MOTOR_,+DIFF_MOTOR_),1000,2000);
    if (f.PASSTHRU_MODE) {    // do not use sensors for correction, simple 2 channel mixing
        servo[3] = (SERVODIR(3,1) * rcCommand[PITCH]) + (SERVODIR(3,2) * rcCommand[ROLL]);
        servo[4] = (SERVODIR(4,1) * rcCommand[PITCH]) + (SERVODIR(4,2) * rcCommand[ROLL]);
				servo[5] = get_middle(5)-rcCommand[YAW];
    } else {                  // use sensors to correct (gyro only or gyro+acc according to aux1/aux2 configuration
        servo[3] = (SERVODIR(3,1) * axisPID[PITCH])   + (SERVODIR(3,2) * axisPID[ROLL]);
        servo[4] = (SERVODIR(4,1) * axisPID[PITCH])   + (SERVODIR(4,2) * axisPID[ROLL]);
			servo[5] = get_middle(5)-axisPID[YAW];
    }
    servo[3] += get_middle(3);
    servo[4] += get_middle(4);
#elif defined( AIRPLANE )
    /*****************************               AIRPLANE                **************************************/
    // servo[7] is programmed with safty features to avoid motorstarts when ardu reset..
    // All other servos go to center at reset..  Half throttle can be dangerus
    // Only use servo[7] as motorcontrol if motor is used in the setup            */
    if (!f.ARMED) {
        servo[7] = MINCOMMAND; // Kill throttle when disarmed
    } else {
        servo[7] = constrain(rcCommand[THROTTLE], conf.minthrottle, MAXTHROTTLE);
    }
    motor[0] = servo[7];

    // Flapperon Controll TODO - optimalisation
    int16_t flapperons[2]= {0,0};
#if  defined(FLAPPERONS) && defined(FLAPPERON_EP)
    int8_t flapinv[2] = FLAPPERON_INVERT;
    static int16_t F_Endpoint[2] = FLAPPERON_EP;
    int16_t flap =MIDRC-constrain(rcData[FLAPPERONS],F_Endpoint[0],F_Endpoint[1]);
    static int16_t slowFlaps= flap;
#if defined(FLAPSPEED)
    if (slowFlaps < flap ) {
        slowFlaps+=FLAPSPEED;
    }
    else if(slowFlaps > flap) {
        slowFlaps-=FLAPSPEED;
    }
#else
    slowFlaps = flap;
#endif
    flap = MIDRC-(constrain(MIDRC-slowFlaps,F_Endpoint[0],F_Endpoint[1]));
    for(i=0; i<2; i++) {
        flapperons[i] = flap * flapinv[i] ;
    }
#endif

    // Traditional Flaps on SERVO3
#if defined(FLAPS)
    // configure SERVO3 middle point in GUI to using an AUX channel for FLAPS control
    // use servo min, servo max and servo rate for propper endpoints adjust
    int16_t lFlap = get_middle(2);
    lFlap = constrain(lFlap, conf.servoConf[2].min, conf.servoConf[2].max);
    lFlap = MIDRC - lFlap;
    static int16_t slow_LFlaps= lFlap;
#if defined(FLAPSPEED)
    if (slow_LFlaps < lFlap ) {
        slow_LFlaps+=FLAPSPEED;
    }
    else if(slow_LFlaps > lFlap) {
        slow_LFlaps-=FLAPSPEED;
    }
#else
    slow_LFlaps = lFlap;
#endif
    servo[2] = ((int32_t)conf.servoConf[2].rate * slow_LFlaps)/100L;
    servo[2] += MIDRC;
#endif

    if(f.PASSTHRU_MODE) {  // Direct passthru from RX
        servo[3] = rcCommand[ROLL] + flapperons[0];     //   Wing 1
        servo[4] = rcCommand[ROLL] + flapperons[1];     //   Wing 2
        servo[5] = rcCommand[YAW];                      //   Rudder
        servo[6] = -rcCommand[PITCH];                    //   Elevator
    } else {
        // Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
        servo[3] = axisPID[ROLL] + flapperons[0];   //   Wing 1
        servo[4] = axisPID[ROLL] + flapperons[1];   //   Wing 2
        servo[5] = axisPID[YAW];                    //   Rudder
        servo[6] = -axisPID[PITCH];                  //   Elevator
    }
    for(i=3; i<7; i++) {
        servo[i]  = ((int32_t)conf.servoConf[i].rate * servo[i])/100L;  // servo rates
        servo[i] += get_middle(i);
    }
#elif defined( SINGLECOPTER )
    /***************************          Single & DualCopter          ******************************/
    // Singlecopter
    // This is a beta requested by  xuant9
    // Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
    // http://www.kkmulticopter.kr/multicopter/images/blue_single.jpg
    servo[3]  = (axisPID[YAW]*SERVODIR(3,2)) + (axisPID[PITCH]*SERVODIR(3,1));   //   SideServo  5  D12
    servo[4]  = (axisPID[YAW]*SERVODIR(4,2)) + (axisPID[PITCH]*SERVODIR(4,1));   //   SideServo  3  D11
    servo[5]  = (axisPID[YAW]*SERVODIR(5,2)) + (axisPID[ROLL] *SERVODIR(5,1));   //   FrontServo 2  D3
    servo[6]  = (axisPID[YAW]*SERVODIR(6,2)) + (axisPID[ROLL] *SERVODIR(6,1));   //   RearServo  4  D10
    for(i=3; i<7; i++) {
        servo[i]  = (axisPID[YAW] * SERVODIR(i,2)) + (axisPID[(6-i)>>1] * SERVODIR(i,1));  // mix and setup direction
        servo[i] += get_middle(i);
    }
    motor[0] = rcCommand[THROTTLE];
#elif defined( DUALCOPTER )
    // Dualcopter
    // This is a beta requested by  xuant9
    // Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
    //http://www.kkmulticopter.kr/products_pic/index.html?sn=multicopter_v02_du&name=KKMultiCopter%20Flight%20Controller%20Blackboard%3Cbr%3E
    //servo[4] = ((int32_t)conf.servoConf[4].rate * axisPID[PITCH])/100L;  //  PITCHServo   D11 => Wing2 servo
    //servo[5] = ((int32_t)conf.servoConf[5].rate * axisPID[ROLL] )/100L;   //  ROLLServo    D3  => Rudder servo
    for(i=4; i<6; i++) {
        servo[i] =  axisPID[5-i] * SERVODIR(i,1);    // mix and setup direction
        servo[i] += get_middle(i);
    }
    motor[0] = PIDMIX(0,0,-1);                                 //  Pin D9
    motor[1] = PIDMIX(0,0,+1);                                 //  Pin D10

#elif defined( HELICOPTER )
    /*****************************               HELICOPTERS               **************************************/
    // Common controlls for Helicopters
    int16_t heliRoll,heliNick;
    int16_t collRange[3] = COLLECTIVE_RANGE;
    static int16_t   collective;
#ifdef GOVERNOR_P
    static int16_t last_collective = 0, delta_collective = 0, governorThrottle = 0;
#endif
    /***************************
     * servo settings Heli.
     ***************************/

    // Limit Collective range up/down
    int16_t collect = rcData[COLLECTIVE_PITCH] - (1500 + collRange[1]);
    if   (collect>0) {
        collective = collect * (collRange[2]*0.01);
    } else {
        collective = collect * (collRange[0]*0.01);
    }

    // maybe collective range can be replaced replaced by this ?
    //collective = rcData[COLLECTIVE_PITCH] - get_middle(7);               // zero pitch offset
    //collective = ((int32_t)conf.servoConf[7].rate * collective)/100L;    // collective range

#ifdef GOVERNOR_P

    delta_collective = collective - last_collective;
    last_collective = collective;
    if (! f.ARMED || ! rcOptions[BOXGOV] || (rcCommand[THROTTLE] < conf.minthrottle) )
        governorThrottle = 0; // clear subito
    else if (delta_collective > 0) {
        governorThrottle += delta_collective * conf.governorP; // attack
        // avoid overshooting governor (would result in overly long decay phase)
        if (rcCommand[THROTTLE] + governorThrottle > MAXTHROTTLE) governorThrottle = MAXTHROTTLE - rcCommand[THROTTLE];
    } else {
        static uint8_t d = 0;
        if (! (++d % conf.governorD)) governorThrottle -= 10; // decay; signal stepsize 10 should  be smooth on most ESCs
    }
    if (governorThrottle < 0) governorThrottle = 0; // always beware of negative impact of governor on throttle
#endif

    if(f.PASSTHRU_MODE) { // Use Rcdata Without sensors
        heliRoll = rcCommand[ROLL] ;
        heliNick = rcCommand[PITCH];
    } else { // Assisted modes
        heliRoll = axisPID[ROLL];
        heliNick = axisPID[PITCH];
    }

    // Limit Maximum Rates for Heli

    heliRoll*= cRange[0]*0.01;
    heliNick*= cRange[1]*0.01;

    /* Throttle & YAW
    ******************** */
    // Yaw control is common for Heli 90 & 120
    //servo[5] = (axisPID[YAW] * SERVODIR(5,1)) + conf.servoConf[5].middle;
    // tail precomp from collective
    acmd = abs(collective) - conf.yawCollPrecompDeadband; // abs collective minus deadband
    if (acmd > 0 ) {
        servo[5] = (axisPID[YAW] * SERVODIR(5,1)) + conf.servoConf[5].middle + (acmd * conf.yawCollPrecomp)/10;
    } else {
        servo[5] = (axisPID[YAW] * SERVODIR(5,1)) + conf.servoConf[5].middle;
    }
#if YAWMOTOR
    servo[5] = constrain(servo[5], conf.servoConf[5].min, conf.servoConf[5].max); // limit the values
    if (rcCommand[THROTTLE]<conf.minthrottle || !f.ARMED) {
        servo[5] = MINCOMMAND;   // Kill YawMotor
    }
#endif
    if (!f.ARMED) {
        servo[7] = MINCOMMAND;          // Kill throttle when disarmed
    } else {
        servo[7] = rcCommand[THROTTLE]; //   50hz ESC or servo
#ifdef GOVERNOR_P
        servo[7] += governorThrottle;
#endif
        servo[7] = constrain(servo[7], conf.minthrottle, MAXTHROTTLE);   //  limit min & max    }
    }
#ifndef HELI_USE_SERVO_FOR_THROTTLE
    motor[0] = servo[7];     // use real motor output - ESC capable
#if YAWMOTOR
    motor[1] = servo[5];   // use motor2 output for YAWMOTOR
#endif
#endif

    //              ( Collective, Pitch/Nick, Roll ) Change sign to invert
    /************************************************************************************************************/
#define HeliXPIDMIX(Z,Y,X) ( (collRange[1] + collective)*Z + heliNick*Y +  heliRoll*X)/10
#ifdef HELI_120_CCPM
    static int8_t nickMix[3] = SERVO_NICK;
    static int8_t leftMix[3] = SERVO_LEFT;
    static int8_t rightMix[3]= SERVO_RIGHT;

    servo[3]  =  HeliXPIDMIX( ( SERVODIR(3,4) * nickMix[0]), SERVODIR(3,2) * nickMix[1], SERVODIR(3,1) * nickMix[2]);   //    NICK  servo
    servo[4]  =  HeliXPIDMIX( ( SERVODIR(4,4) * leftMix[0]), SERVODIR(4,2) * leftMix[1], SERVODIR(4,1) * leftMix[2]);   //    LEFT  servo
    servo[6]  =  HeliXPIDMIX( ( SERVODIR(6,4) * rightMix[0]),SERVODIR(6,2) * rightMix[1],SERVODIR(6,1) * rightMix[2]);  //    RIGHT servo
#endif
    /************************************************************************************************************/
#ifdef HELI_90_DEG
    servo[3]  = HeliXPIDMIX( +0, (conf.servoConf[3].rate/10), -0);      //     NICK  servo
    servo[4]  = HeliXPIDMIX( +0, +0, (conf.servoConf[4].rate/10));      //     ROLL servo
    servo[6]  = HeliXPIDMIX( (conf.servoConf[6].rate/10), +0, +0);      //     COLLECTIVE  servo
#endif
    servo[3] += get_middle(3);
    servo[4] += get_middle(4);
    servo[6] += get_middle(6);
#elif defined( GIMBAL )
    for(i=0; i<2; i++) {
        servo[i]  = ((int32_t)conf.servoConf[i].rate * att.angle[1-i]) /50L;
        servo[i] += get_middle(i);
    }
#else
#error "missing coptertype mixtable entry. Either you forgot to define a copter type or the mixing table is lacking neccessary code"
#endif // MY_PRIVATE_MIXING

    /************************************************************************************************************/
    /****************************                Cam stabilize Servos             *******************************/

#if defined(SERVO_TILT)
    servo[0] = get_middle(0);
    servo[1] = get_middle(1);
    if (rcOptions[BOXCAMSTAB]) {
        servo[0] += ((int32_t)conf.servoConf[0].rate * att.angle[PITCH]) /50L;
        servo[1] += ((int32_t)conf.servoConf[1].rate * att.angle[ROLL])  /50L;
    }
#endif

#ifdef SERVO_MIX_TILT
    int16_t angleP = get_middle(0) - MIDRC;
    int16_t angleR = get_middle(1) - MIDRC;
    if (rcOptions[BOXCAMSTAB]) {
        angleP += ((int32_t)conf.servoConf[0].rate * att.angle[PITCH]) /50L;
        angleR += ((int32_t)conf.servoConf[1].rate * att.angle[ROLL])  /50L;
    }
    servo[0] = MIDRC+angleP-angleR;
    servo[1] = MIDRC-angleP-angleR;
#endif

    /****************                    Cam trigger Servo                ******************/
#if defined(CAMTRIG)
    // setup MIDDLE for using as camtrig interval (in msec) or RC channel pointer for interval control
#define CAM_TIME_LOW  conf.servoConf[2].middle
    static uint8_t camCycle = 0;
    static uint8_t camState = 0;
    static uint32_t camTime = 0;
    static uint32_t ctLow;
    if (camCycle==1) {
        if (camState == 0) {
            camState = 1;
            camTime = millis();
        } else if (camState == 1) {
            if ( (millis() - camTime) > CAM_TIME_HIGH ) {
                camState = 2;
                camTime = millis();
                if(CAM_TIME_LOW < RC_CHANS) {
                    ctLow = constrain((rcData[CAM_TIME_LOW]-1000)/4, 30, 250);
                    ctLow *= ctLow;
                } else ctLow = CAM_TIME_LOW;
            }
        } else { //camState ==2
            if (((millis() - camTime) > ctLow) || !rcOptions[BOXCAMTRIG] ) {
                camState = 0;
                camCycle = 0;
            }
        }
    }
    if (rcOptions[BOXCAMTRIG]) camCycle=1;
    servo[2] =(camState==1) ? conf.servoConf[2].max : conf.servoConf[2].min;
    servo[2] = (servo[2]-1500)*SERVODIR(2,1)+1500;
#endif

    /************************************************************************************************************/
    // add midpoint offset, then scale and limit servo outputs - except SERVO8 used commonly as Moror output
    // don't add offset for camtrig servo (SERVO3)
#if defined(SERVO)
    for(i=SERVO_START-1; i<SERVO_END; i++) {
        if(i < 2) {
            servo[i] = map(servo[i], 1020,2000, conf.servoConf[i].min, conf.servoConf[i].max);   // servo travel scaling, only for gimbal servos
        }
#if defined(HELICOPTER) && (YAWMOTOR)
        if(i != 5) // not limit YawMotor
#endif
            servo[i] = constrain(servo[i], conf.servoConf[i].min, conf.servoConf[i].max); // limit the values
    }
#if defined(A0_A1_PIN_HEX) && (NUMBER_MOTOR == 6) && defined(PROMINI)
    servo[3] = servo[0];    // copy CamPitch value to propper output servo for A0_A1_PIN_HEX
    servo[4] = servo[1];    // copy CamRoll  value to propper output servo for A0_A1_PIN_HEX
#endif
#if defined(TRI) && defined(MEGA_HW_PWM_SERVOS) && defined(MEGA)
    servo[5] = constrain(servo[5], conf.servoConf[5].min, conf.servoConf[5].max); // servo[5] is still use by gui for this config (more genereic)
    servo[3] = servo[5];    // copy TRI serwo value to propper output servo for MEGA_HW_PWM_SERVOS
#endif
#endif

    /****************                compensate the Motors values                ******************/
#ifdef VOLTAGEDROP_COMPENSATION
    {
#if (VBATNOMINAL == 84) // 2S
#define GOV_R_NUM 24
        static int8_t g[] = { 0,4,8,12,17,21,25,30,34,39,44,49,54,59,65,70,76,81,87,93,99,106,112,119,126 };
#elif (VBATNOMINAL == 126) // 3S
#define GOV_R_NUM 36
        static int8_t g[] = { 0,3,5,8,11,14,17,19,22,25,28,31,34,38,41,44,47,51,54,58,61,65,68,72,76,79,83,87,91,95,99,104,108,112,117,121,126 };
#elif (VBATNOMINAL == 252) // 6S
#define GOV_R_NUM 72
        static int8_t g[] = { 0,1,3,4,5,7,8,9,11,12,14,15,17,18,19,21,22,24,25,27,28,30,31,33,34,36,38,39,41,
                              42,44,46,47,49,51,52,54,56,58,59,61,63,65,66,68,70,72,74,76,78,79,81,83,85,87,89,91,93,95,97,99,
                              101,104,106,108,110,112,114,117,119,121,123,126
                            };
#elif (VBATNOMINAL == 255) // 6S HV
#define GOV_R_NUM 73
        static int8_t g[] = { 0,1,3,4,5,7,8,9,11,12,14,15,16,18,19,21,22,24,25,26,28,29,31,33,34,36,37,39,40,
                              42,44,45,47,48,50,52,53,55,57,59,60,62,64,66,67,69,71,73,75,76,78,80,82,84,86,88,90,92,94,96,98,
                              100,102,104,106,108,111,113,115,117,119,122,124,126
                            };
#elif (VBATNOMINAL == 129) // 3S HV
#define GOV_R_NUM 37
        static int8_t g[] = { 0,3,5,8,11,13,16,19,22,25,28,31,34,37,40,43,46,49,53,56,59,63,66,70,74,77,81,85,
                              89,93,96,101,105,109,113,117,122,126
                            };
#elif (VBATNOMINAL == 168) // 4S
#define GOV_R_NUM 48
        static int8_t g[] = { 0,2,4,6,8,10,12,14,17,19,21,23,25,28,30,32,34,37,39,42,44,47,49,52,54,57,59,62,
                              65,67,70,73,76,78,81,84,87,90,93,96,99,103,106,109,112,116,119,122,126
                            };
#else
#error "VOLTAGEDROP_COMPENSATION requires correction values which fit VBATNOMINAL; not yet defined for your value of VBATNOMINAL"
#endif
        uint8_t v = constrain( VBATNOMINAL - constrain(analog.vbat, conf.vbatlevel_crit, VBATNOMINAL), 0, GOV_R_NUM);
        for (i = 0; i < NUMBER_MOTOR; i++) {
            motor[i] += ( ( (int32_t)(motor[i]-1000) * (int32_t)g[v] ) )/ 500;
        }
    }
#endif
    /****************                normalize the Motors values                ******************/
    maxMotor=motor[0];
    for(i=1; i< NUMBER_MOTOR; i++)
        if (motor[i]>maxMotor) maxMotor=motor[i];
    for(i=0; i< NUMBER_MOTOR; i++) {
        if (maxMotor > MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
            motor[i] -= maxMotor - MAXTHROTTLE;
        motor[i] = constrain(motor[i], conf.minthrottle, MAXTHROTTLE);
        if ((rcData[THROTTLE] < MINCHECK) && !f.BARO_MODE)
#ifndef MOTOR_STOP
            motor[i] = conf.minthrottle;
#else
            motor[i] = MINCOMMAND;
#endif
        if (!f.ARMED)
            motor[i] = MINCOMMAND;
    }

    /****************                      Powermeter Log                    ******************/
#if (LOG_VALUES >= 3) || defined(POWERMETER_SOFT)
    {
        static uint32_t lastRead = currentTime;
        uint16_t amp;
        uint32_t ampsum, ampus; // pseudo ampere * microseconds
        /* true cubic function;
         * when divided by vbat_max=126 (12.6V) for 3 cell battery this gives maximum value of ~ 500
         * when divided by no_vbat=60 (6V) for 3 cell battery this gives maximum value of ~ 1000
         * */

        static uint16_t amperes[64] =   {   0,  2,  6, 15, 30, 52, 82,123,
                                            175,240,320,415,528,659,811,984,
                                            1181,1402,1648,1923,2226,2559,2924,3322,
                                            3755,4224,4730,5276,5861,6489,7160,7875,
                                            8637,9446,10304,11213,12173,13187,14256,15381,
                                            16564,17805,19108,20472,21900,23392,24951,26578,
                                            28274,30041,31879,33792,35779,37843,39984,42205,
                                            44507,46890,49358,51910,54549,57276,60093,63000
                                        };

        if (analog.vbat > NO_VBAT) { // by all means - must avoid division by zero
            ampsum = 0;
            for (i =0; i<NUMBER_MOTOR; i++) {
                amp = amperes[ ((motor[i] - 1000)>>4) ] / analog.vbat; // range mapped from [1000:2000] => [0:1000]; then break that up into 64 ranges; lookup amp
                ampus = ( (currentTime-lastRead) * (uint32_t)amp * (uint32_t)conf.pint2ma ) / PLEVELDIVSOFT;
#if (LOG_VALUES >= 3)
                pMeter[i]+= ampus; // sum up over time the mapped ESC input
#endif
#if defined(POWERMETER_SOFT)
                ampsum += ampus; // total sum over all motors
#endif
            }
#if defined(POWERMETER_SOFT)
            pMeter[PMOTOR_SUM]+= ampsum / NUMBER_MOTOR; // total sum over all motors
#endif
        }
        lastRead = currentTime;
    }
#endif
}