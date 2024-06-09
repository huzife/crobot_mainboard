#include "kinematics_impl/kinematics_4mec.h"
#include <math.h>

static Kinematics_4MEC_Param kinematics_param;

void kinematics_set_param_4mec(Kinematics_4MEC_Param param) {
    kinematics_param = param;
}

void kinematics_update_odometry_4mec(Odometry* odometry, Velocity velocity, float dt) {
    float dx = velocity.linear_x * dt;
    float dy = velocity.linear_y * dt;
    float dyaw = velocity.angular_z * dt;

    float dir = odometry->direction;
    odometry->position_x += dx * cosf(dir) - dy * sinf(dir);
    odometry->position_y += dx * sinf(dir) + dy * cosf(dir);
    odometry->direction += dyaw;
}

void kinematics_inverse_4mec(Velocity velocity, float speeds[]) {
    float radius = kinematics_param.radius;
    float d = kinematics_param.distance_x + kinematics_param.distance_y;
    float vx = velocity.linear_x;
    float vy = velocity.linear_y;
    float angular = velocity.angular_z;

    // rotate speed of each wheel, rad/s
    float v = angular * d;
    float speed_fl = (vx - vy - v) / radius;
    float speed_fr = (vx + vy + v) / radius;
    float speed_bl = (vx + vy - v) / radius;
    float speed_br = (vx - vy + v) / radius;
    speeds[0] = -speed_fl;
    speeds[1] = speed_fr;
    speeds[2] = -speed_bl;
    speeds[3] = speed_br;
}

void kinematics_forward_4mec(float speeds[], Velocity* velocity) {
    float radius = kinematics_param.radius;
    float d = kinematics_param.distance_x + kinematics_param.distance_y;

    // rotate speed
    float speed_fl = -speeds[0];
    float speed_fr = speeds[1];
    float speed_bl = -speeds[2];
    float speed_br = speeds[3];

    // linear and angular
    velocity->linear_x = (radius * (speed_fl + speed_fr + speed_bl + speed_br)) / 4;
    velocity->linear_y = (radius * (-speed_fl + speed_fr + speed_bl - speed_br)) / 4;
    velocity->angular_z = (radius * (-speed_fl + speed_fr - speed_bl + speed_br)) / 4 / d;
}
