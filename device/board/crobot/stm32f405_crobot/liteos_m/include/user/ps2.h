#ifndef USER_PS2_H
#define USER_PS2_H

#include <stdint.h>

typedef enum {
    PS2_INVALID,
    PS2_DIGITAL,
    PS2_ANALOG
} PS2_Mode;

typedef struct {
    PS2_Mode Mode;                                          //模拟(红灯)为1 数字(无灯)为0
    int8_t Rocker_RX, Rocker_RY, Rocker_LX, Rocker_LY;      //摇杆值(模拟状态为实际值0-0xFF)(数字态为等效的值0,0x80,0xFF)
    //按键值0为未触发,1为触发态
    uint8_t Key_L1, Key_L2, Key_R1, Key_R2;                 //后侧大按键
    uint8_t Key_L_Right, Key_L_Left, Key_L_Up, Key_L_Down;  //左侧按键
    uint8_t Key_R_Right, Key_R_Left, Key_R_Up, Key_R_Down;  //右侧按键
    uint8_t Key_Select;                                     //选择键
    uint8_t Key_Start;                                      //开始键
    uint8_t Key_Rocker_Left, Key_Rocker_Right;              //摇杆按键
} PS2_State;

extern volatile PS2_State ps2_state;
void ps2_read_data(void);

#endif // USER_PS2_H
