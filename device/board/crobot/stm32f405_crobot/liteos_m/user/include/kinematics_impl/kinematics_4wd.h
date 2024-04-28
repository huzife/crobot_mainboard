#ifndef USER_KINEMATICS_IMPL_KINEMATICS_4WD_H
#define USER_KINEMATICS_IMPL_KINEMATICS_4WD_H

#include "odometry.h"
#include "velocity.h"

typedef struct {
    float radius;
    float separation;
} Kinematics_4WD_Param;

void kinematics_set_param_4wd(Kinematics_4WD_Param param);
void kinematics_update_odometry_4wd(Odometry* odometry, Velocity velocity, float dt);
void kinematics_inverse_4wd(Velocity velocity, float speeds[]);
void kinematics_forward_4wd(float speeds[], Velocity* velocity);

#endif // USER_KINEMATICS_IMPL_KINEMATICS_4WD_H
