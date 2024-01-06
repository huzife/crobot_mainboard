#ifndef USER_KINEMATICS_H
#define USER_KINEMATICS_H

#include <stdint.h>

typedef struct {
    double radius;
    double separation;
    double interval;
    uint16_t count_per_cycle;
    uint8_t reverse[2]; // 0: left, 1: right
} Kinematics_2WD_Param;

typedef struct {
    double linear_x;    // X axis linear velocity, m/s
    double angular_z;   // Z axis angular velocity, rad/s
    int16_t speed[2];   // motor speed, 0: right, 1: left
} Kinematics_2WD;

extern volatile Kinematics_2WD kinematics[2]; // 0: inverse, 1: forward
extern volatile Kinematics_2WD_Param kinematics_param;

void kinematics_2WD_param_init(Kinematics_2WD_Param* param);
void kinematics_2WD_init(Kinematics_2WD* kinematics);
void kinematics_2WD_inverse(Kinematics_2WD* kinematics,
                            Kinematics_2WD_Param* param);
void kinematics_2WD_forward(Kinematics_2WD* kinematics,
                            Kinematics_2WD_Param* param);

#endif // USER_KINEMATICS_H
