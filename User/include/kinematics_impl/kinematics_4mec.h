#ifndef USER_KINEMATICS_IMPL_KINEMATICS_4MEC_H
#define USER_KINEMATICS_IMPL_KINEMATICS_4MEC_H

#include "odometry.h"
#include "velocity.h"

typedef struct {
    float radius;
    float distance_x;
    float distance_y;
} Kinematics_4MEC_Param;

void kinematics_set_param_4mec(Kinematics_4MEC_Param param);
void kinematics_update_odometry_4mec(Odometry* odometry, Velocity velocity, float dt);
void kinematics_inverse_4mec(Velocity velocity, float speeds[]);
void kinematics_forward_4mec(float speeds[], Velocity* velocity);

#endif // USER_KINEMATICS_IMPL_KINEMATICS_4MEC_H
