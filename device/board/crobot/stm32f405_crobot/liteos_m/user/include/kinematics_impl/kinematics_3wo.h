#ifndef USER_KINEMATICS_IMPL_KINEMATICS_3WO_H
#define USER_KINEMATICS_IMPL_KINEMATICS_3WO_H

#include "odometry.h"
#include "velocity.h"

typedef struct {
    float radius;
    float distance;
} Kinematics_3WO_Param;

void kinematics_set_param_3wo(Kinematics_3WO_Param param);
void kinematics_update_odometry_3wo(Odometry* odometry, Velocity velocity, float dt);
void kinematics_inverse_3wo(Velocity velocity, float speeds[]);
void kinematics_forward_3wo(float speeds[], Velocity* velocity);

#endif // USER_KINEMATICS_IMPL_KINEMATICS_3WO_H
