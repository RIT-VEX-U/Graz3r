#pragma once
#include "TempSubSystems/TempSubSystems.h"
#include "core.h"
#include "core/subsystems/fun/video.h"
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
extern vex::motor left_back_bottom;
extern vex::motor left_center_bottom;
extern vex::motor left_front_top;
extern vex::motor left_back_top;

extern vex::motor right_back_bottom;
extern vex::motor right_center_bottom;
extern vex::motor right_front_top;
extern vex::motor right_back_top;

extern vex::motor conveyor;
extern vex::motor intake_motor;

extern vex::motor_group left_drive_motors;
extern vex::motor_group right_drive_motors;

extern vex::digital_out mcglight_board;

// Pneumatics

extern vex::digital_out goal_grabber_sol;
extern vex::digital_out goal_rush_sol;

// Button Definitions
extern const controller::button &goal_grabber;
extern const controller::button &goal_rush_arm;
extern const controller::button &conveyor_button;
extern const controller::button &conveyor_button_rev;

// ================ SUBSYSTEMS ================
extern ClamperSys clamper_sys;
extern IntakeSys intake_sys;

extern PID drive_pid;
extern PID turn_pid;
extern PID::pid_config_t correction_pid_cfg;

extern OdometryTank odom;

extern robot_specs_t robot_cfg;
extern TankDrive drive_sys;

// ================ UTILS ================
void robot_init();