#include "kinematics_impl/kinematics_2wd.h"
#include <math.h>

static Kinematics_2WD_Param kinematics_param;

void kinematics_set_param_2wd(Kinematics_2WD_Param param) {
    kinematics_param = param;
}

void kinematics_update_odometry_2wd(Odometry* odometry, Velocity velocity, float dt) {
    float dx = velocity.linear_x * dt;
    float dyaw = velocity.angular_z * dt;

    if (fabsf(dyaw) < 1e-6f) {
        float dir = odometry->direction + dyaw * 0.5f;
        odometry->position_x += dx * cosf(dir);
        odometry->position_y += dx * sinf(dir);
        odometry->direction += dyaw;
    } else {
        float dir_old = odometry->direction;
        float r = dx / dyaw;
        odometry->direction += dyaw;
        odometry->position_x += r * (sinf(odometry->direction) - sinf(dir_old));
        odometry->position_y += -r * (cosf(odometry->direction) - cosf(dir_old));
    }
}

void kinematics_inverse_2wd(Velocity velocity, float speeds[]) {
    float radius = kinematics_param.radius;
    float separation = kinematics_param.separation;
    float linear = velocity.linear_x;
    float angular = velocity.angular_z;

    // rotate speed of each wheel, rad/s
    speeds[0] = (angular * separation / 2 - linear) / radius;
    speeds[1] = (angular * separation / 2 + linear) / radius;
}

void kinematics_forward_2wd(float speeds[], Velocity* velocity) {
    float radius = kinematics_param.radius;
    float separation = kinematics_param.separation;

    // rotate speed
    float speed_left = speeds[0];
    float speed_right = speeds[1];

    // linear and angular
    velocity->linear_x = (radius * (speed_right - speed_left)) / 2;
    velocity->linear_y = 0.0f;
    velocity->angular_z = (radius * (speed_right + speed_left)) / separation;
}
