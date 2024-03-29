#define CAMERA_PAIR_NUM 5
void DJI_Guidance_uart_protocl(uint8_t c);
/**
 *@struct  ultrasonic_data
 *@brief Define ultrasonic data structure. `ultrasonic` is the distance between Guidance Sensor and the nearest object detected by ultrasonic sensor. The Unit is `mm`. `reliability` is the reliability of this distance measurement, with 1 meaning reliable and 0 unreliable.
 */
enum e_guidance_event
{
    e_image = 0,         /**< called back when image comes */
    e_imu,               /**< called back when imu comes */
    e_ultrasonic,        /**< called back when ultrasonic comes */
    e_velocity,          /**< called back when velocity data comes */
    e_obstacle_distance, /**< called back when obstacle data comes */
    e_motion,            /**< called back when global position comes */
    e_event_num
};

typedef struct _VO_OUTPUT
{
    short cnt;

    short vx;
    short vy;
    short vz;

    float x;
    float y;
    float z;

    float reserved[12];

    float height;
    float uncertainty_height;

    unsigned char reserve[4];
} VO_OUTPUT;

#pragma pack()

typedef struct _soc2pc_vo_can_output
{
    VO_OUTPUT m_vo_output;
    unsigned int m_frame_index;
    unsigned int m_time_stamp;
    unsigned int m_reserved[9];
} soc2pc_vo_can_output;
typedef struct _ultrasonic_data
{
    uint32_t frame_index;                /**< correspondent frame index */
    uint32_t time_stamp;                 /**< time stamp of correspondent image captured in ms */
    int16_t ultrasonic[CAMERA_PAIR_NUM]; /**< distance in mm. -1 means invalid measurement. */
    uint16_t reliability[5];             /**< reliability of the distance data */
} ultrasonic_data;
/**
 *@struct  velocity
 *@brief Define velocity in body frame coordinate. Unit is `mm/s`.
 */
typedef struct _velocity
{
    uint32_t frame_index; /**< correspondent frame index */
    uint32_t time_stamp;  /**< time stamp of correspondent image captured in ms */
    int16_t vx;           /**< velocity of x in mm/s */
    int16_t vy;           /**< velocity of y in mm/s */
    int16_t vz;           /**< velocity of z in mm/s */
} velocity;
/**
 *@struct  obstacle_distance
 *@brief Define obstacle distance calculated by fusing vision and ultrasonic sensors. Unit is `cm`.
 */
typedef struct _obstacle_distance
{
    unsigned int frame_index;                 /**< correspondent frame index */
    unsigned int time_stamp;                  /**< time stamp of correspondent image captured in ms */
    unsigned short distance[CAMERA_PAIR_NUM]; /**< distance of obstacle in cm */
} obstacle_distance;

typedef struct _protocal_sdk_uart_header
{
    uint8_t m_header;
    uint16_t m_version : 6;
    uint16_t m_length : 10;
    uint8_t m_R : 2;
    uint8_t m_A : 1;
    uint8_t m_session_id : 5;
    uint8_t m_enc_type : 3;
    uint8_t m_padding : 5;
    uint8_t m_reserved[3];
    uint16_t m_seq_num;
    uint16_t m_header_checksum;
    uint16_t m_data_checksum;
} protocal_sdk_uart_header;

extern obstacle_distance DJI_Guidance_obstacle_distance; // �ϰ�����
extern ultrasonic_data DJI_Guidance_ultrasonic;          // 5·����
extern velocity DJI_Guidance_vo;                         // 3·�ٶȣ���λmm
extern uint8_t DJI_Guidance_flag_alt;
extern uint8_t DJI_Guidance_flag_vel;