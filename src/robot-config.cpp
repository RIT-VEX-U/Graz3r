#include "robot-config.h"
#include "core/subsystems/odometry/odometry_tank_lidar.h"

vex::brain Brain;
vex::controller con;

// ================ INPUTS ================
// Digital sensors
vex::inertial imu(vex::PORT11);
vex::distance clamper_sensor(vex::PORT18);
vex::optical color_sensor(vex::PORT12);

// ================ OUTPUTS ================
// Motors
vex::motor left_front_most(vex::PORT3, vex::gearSetting::ratio6_1, true);
vex::motor left_front_middle(vex::PORT5, vex::gearSetting::ratio6_1, true);
vex::motor left_back_middle(vex::PORT6, vex::gearSetting::ratio6_1, true);
vex::motor left_back_most(vex::PORT19, vex::gearSetting::ratio6_1, true);
vex::motor_group left_drive_motors({left_front_most, left_front_middle, left_back_middle, left_back_most});

vex::motor right_front_most(vex::PORT8, vex::gearSetting::ratio6_1, false);
vex::motor right_front_middle(vex::PORT9, vex::gearSetting::ratio6_1, false);
vex::motor right_back_middle(vex::PORT10, vex::gearSetting::ratio6_1, false);
vex::motor right_back_most(vex::PORT20, vex::gearSetting::ratio6_1, false);
vex::motor_group right_drive_motors({right_front_most, right_front_middle, right_back_middle, right_back_most});

vex::motor conveyor(vex::PORT7, vex::gearSetting::ratio6_1, true);
vex::motor intake_motor(vex::PORT4, vex::gearSetting::ratio6_1, false);

vex::digital_out mcglight_board(Brain.ThreeWirePort.F);

// pnematices
vex::digital_out goal_grabber_sol{Brain.ThreeWirePort.H};
vex::digital_out goal_rush_sol{Brain.ThreeWirePort.G};

// Button Definitions
const vex::controller::button &goal_grabber = con.ButtonB;
const vex::controller::button &goal_rush_arm = con.ButtonDown;
const vex::controller::button &conveyor_button = con.ButtonR1;
const vex::controller::button &conveyor_button_rev = con.ButtonR2;

// ================ SUBSYSTEMS ================
PID::pid_config_t drive_pid_cfg{
  .p = 0.6,
  .i = 0,
  .d = 0.035,
  .deadband = 0.5,
  .on_target_time = 0.1,
};

PID drive_pid{drive_pid_cfg};

PID::pid_config_t turn_pid_cfg{
  .p = 0.05,
  .i = 0,
  .d = 0.001,
  .deadband = 1,
  .on_target_time = 0.1,
  .error_method = PID::ERROR_TYPE::LINEAR,

};

PID::pid_config_t correction_pid_cfg{
  .p = 0.05,
  .i = 0,
  .d = 0.001,
  .deadband = 1,
};

FeedForward::ff_config_t drive_ff_cfg{.kS = 0.9, .kV = 0.015, .kA = 0.002, .kG = 0};

PID turn_pid{turn_pid_cfg};
// ======== SUBSYSTEMS ========

robot_specs_t robot_cfg = {
  .robot_radius = 10,
  .odom_wheel_diam = 1.75,
  .odom_gear_ratio = 0.75,
  .dist_between_wheels = 12.4,

  .drive_correction_cutoff = 10,

  .drive_feedback = &drive_pid,
  .turn_feedback = &turn_pid,
  .correction_pid = correction_pid_cfg,
};

ClamperSys clamper_sys{};
IntakeSys intake_sys{};

Pose2d zero{0, 0, from_degrees(0)};
Pose2d red_r_test{19.4, 42.4, from_degrees(0)};

OdometryTank odom(left_drive_motors, right_drive_motors, robot_cfg, &imu);
OdometryTankLidar lidar(
  0.75, 5.497786, Pose2d(0, 0, 0), EVec<3>{100, 100, 0.1}, EVec<3>{0.05, 0.05, deg2rad(0.01)},
  EVec<2>{1, 1}, imu, left_drive_motors, right_drive_motors, vex::PORT13, 921600, Transform2d(-5.25, 6.1, from_degrees(180)), 0.188575, 0.024388, 1.6365, 0.1932
);

TankDrive drive_sys(left_drive_motors, right_drive_motors, robot_cfg, &odom);

// A global instance of vex::brain used for printing to the V5 brain screen
void print_multiline(const std::string &str, int y, int x);

vex::gps gps_sensor{vex::PORT17, -6.5, 3, vex::distanceUnits::in, 90, vex::turnType::left};

// ================ UTILS ================

/**
 * Main robot initialization on startup. Runs before opcontrol and autonomous are started.
 */
void robot_init() {
  printf("start\n");
  gps_sensor.calibrate();
  imu.calibrate();
  
  while (gps_sensor.isCalibrating() || imu.isCalibrating()) {
    vexDelay(10);
  }
  printf("calibrated\n"); 
  gps_sensor.setOrigin(1.75, 4, vex::distanceUnits::in); 

  int bad_gps_count = 0;
  
    while (gps_sensor.quality() != 100 && bad_gps_count < 500) {
      bad_gps_count++;

        vexDelay(10);
    }

    if (bad_gps_count >= 500) {
        return;
    }
    double x = gps_sensor.xPosition(vex::distanceUnits::in) + 71.25;
    double y = gps_sensor.yPosition(vex::distanceUnits::in) + 71.25;
    double heading = deg2rad(wrap_degrees_180(gps_sensor.heading(vex::rotationUnits::deg) + 90));

    lidar.set_position(Pose2d(x, y, from_radians(heading)));
    printf("gps pos: %f, %f, %f\n", x, y, rad2deg(heading));
}