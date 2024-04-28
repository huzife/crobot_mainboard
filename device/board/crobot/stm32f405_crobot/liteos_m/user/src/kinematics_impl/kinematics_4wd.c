#include "kinematics_impl/kinematics_4wd.h"
#include <math.h>

static Kinematics_4WD_Param kinematics_param;

void kinematics_set_param_4wd(Kinematics_4WD_Param param) {
    kinematics_param = param;
}

void kinematics_update_odometry_4wd(Odometry* odometry, Velocity velocity, float dt) {
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

void kinematics_inverse_4wd(Velocity velocity, float speeds[]) {
    float radius = kinematics_param.radius;
    float separation = kinematics_param.separation;
    float linear = velocity.linear_x;
    float angular = velocity.angular_z;

    // rotate speed of each wheel, rad/s
    float speed_left = (angular * separation / 2 - linear) / radius;
    float speed_right = (angular * separation / 2 + linear) / radius;

    speeds[0] = speed_left;
    speeds[1] = speed_right;
    speeds[2] = speed_left;
    speeds[3] = speed_right;
}

void kinematics_forward_4wd(float speeds[], Velocity* velocity) {
    float radius = kinematics_param.radius;
    float separation = kinematics_param.separation;

    // rotate speed
    float speed_left = (speeds[0] + speeds[2]) / 2;
    float speed_right = (speeds[1] + speeds[3]) / 2;

    // linear and angular
    velocity->linear_x = (radius * (speed_right - speed_left)) / 2;
    velocity->linear_y = 0.0f;
    velocity->angular_z = (radius * (speed_right + speed_left)) / separation;
}
