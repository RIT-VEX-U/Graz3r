#pragma once
#include "TempSubSystems/TempSubSystems.h"
#include "core.h"
#include "core/subsystems/fun/video.h"
// #include "core/subsystems/odometry/odometry_tank_lidar.h"
#include "inttypes.h"
#include "vex.h"

#define WALLSTAKE_POT_OFFSET

extern vex::brain brain;
extern vex::controller con;

// ================ INPUTS ================
// Digital sensors
extern vex::distance clamper_sensor;
extern vex::optical color_sensor;
// Analog sensors

// ================ OUTPUTS ================
// Motors
extern vex::motor left_front_most;
extern vex::motor left_front_middle;
extern vex::motor left_back_middle;
extern vex::motor left_back_most;

extern vex::motor right_front_most;
extern vex::motor right_front_middle;
extern vex::motor right_back_middle;
extern vex::motor right_back_most;

extern vex::motor conveyor;
extern vex::motor intake_motor;

extern vex::motor_group left_drive_motors;
extern vex::motor_group right_drive_motors;

extern vex::motor wallstake_motor;

extern vex::digital_out mcglight_board;

// Pneumatics

extern vex::digital_out goal_grabber_sol;
extern vex::digital_out goal_rush_sol;
extern vex::digital_out wallstake_sol;
extern vex::digital_out climb_sol;

// Button Definitions
extern const controller::button &goal_grabber;
extern const controller::button &goal_rush_arm;
extern const controller::button &conveyor_button;
extern const controller::button &conveyor_button_rev;

extern const controller::button &climb_button;

extern const vex::controller::button &wallstake_toggler;
extern const vex::controller::button &wallstake_stow;
extern const vex::controller::button &wallstake_alliancestake;

extern Pose2d red_positive_pos;
extern Pose2d red_positive_pos_stick;
extern Pose2d blue_positive_pos;
extern Pose2d blue_positive_pos_stick;
extern Pose2d red_negative_pos;
extern Pose2d red_negative_pos_stick;
extern Pose2d blue_negative_pos;
extern bool gps_pos_set;

// ================ SUBSYSTEMS ================
extern ClamperSys clamper_sys;
extern IntakeSys intake_sys;

extern PID drive_pid;
extern PID turn_pid;
extern PID::pid_config_t correction_pid_cfg;

extern OdometryTank odom;
// extern OdometryTankLidar lidar;

extern vex::gps gps_sensor;

extern robot_specs_t robot_cfg;
extern TankDrive drive_sys;

extern WallStakeMech wallstake_sys;

// ================ UTILS ================
void robot_init();