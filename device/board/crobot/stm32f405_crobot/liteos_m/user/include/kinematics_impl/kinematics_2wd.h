#ifndef USER_KINEMATICS_IMPL_KINEMATICS_2WD_H
#define USER_KINEMATICS_IMPL_KINEMATICS_2WD_H

#include "odometry.h"
#include "velocity.h"

typedef struct {
    float radius;
    float separation;
} Kinematics_2WD_Param;

void kinematics_set_param_2wd(Kinematics_2WD_Param param);
void kinematics_update_odometry_2wd(Odometry* odometry, Velocity velocity, float dt);
void kinematics_inverse_2wd(Velocity velocity, float speeds[]);
void kinematics_forward_2wd(float speeds[], Velocity* velocity);

#endif // USER_KINEMATICS_IMPL_KINEMATICS_2WD_H
