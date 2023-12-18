extern uint8_t Mini_Flow_SSI,Mini_Flow_SSI_CNT,mini_flow_flag;

typedef struct __flow {
    int16_t x,y,x1,y1;
    int32_t x_i,y_i;
    int32_t fix_x_i,fix_y_i;
    int16_t ang_x,ang_y;
    int32_t out_x_i,out_y_i;
    int16_t fix_x,fix_y;
    int16_t out_y_i_o,out_x_i_o;
    int16_t pre_ms,now_ms;
} _flow;


extern  _flow flow;
//光流数据接收
void Player_Flow_Receive(uint8_t data);
