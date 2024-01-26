#ifndef USER_KINEMATICS_H
#define USER_KINEMATICS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float radius;
    float separation;
    float linear_factor;
    float angular_factor;
    float interval;
    uint16_t count_per_cycle;
    bool reverse[2]; // 0: left, 1: right
} Kinematics_2WD_Param;

typedef struct {
    float linear_x;     // X axis linear velocity, m/s
    float angular_z;    // Z axis angular velocity, rad/s
    int16_t speed[2];   // motor speed, 0: right, 1: left
} Kinematics_2WD;

typedef struct {
    float position_x;
    float position_y;
    float direction;
} Odometry;

extern Kinematics_2WD_Param kinematics_param;
extern Kinematics_2WD kinematics[2];
extern Odometry odom;

/// @brief Init Kinematics_2WD
/// @param[out] kinematics Pointer to Kinematics_2WD
void kinematics_2WD_init(Kinematics_2WD* kinematics);

/// @brief Init Odometry
/// @param[out] odometry Pointer to Odometry
void odom_init(Odometry* odometry);

/// @brief Update odometry
/// @param[in|out] odometry Pointer to Odometry
/// @param[in] linear X-axis linear velocity
/// @param[in] angular Z-axis angular velocity
/// @param[in] interval The interval from last calculation
void update_odom(Odometry* odometry, float linear, float angular,
                 float interval);

/// @brief Kinematic 2WD inverse function
/// @param[in|out] kinematics Pointer to Kinematics_2WD
void kinematics_2WD_inverse(Kinematics_2WD* kinematics);

/// @brief Kinematic 2WD forward function
/// @param[in|out] kinematics Pointer to Kinematics_2WD
void kinematics_2WD_forward(Kinematics_2WD* kinematics);

#endif // USER_KINEMATICS_H
