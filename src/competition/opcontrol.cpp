#pragma once
#include "competition/opcontrol.h"
#include "robot-config.h"
#include "vex.h"

void testing();
/**
 * Main entrypoint for the driver control period
 */
bool enableDrive = true;
void opcontrol() {
    // autonomous();
    testing();

    goal_grabber.pressed([]() { clamper_sys.toggle_clamp(); });

    conveyor_button.pressed([]() {
        intake_sys.intake();
        intake_sys.conveyor_in();
    });
    conveyor_button_rev.pressed([]() {
        intake_sys.outtake();
        intake_sys.conveyor_out();
    });

    conveyor_button.released([]() {
        intake_sys.intake_stop();
        intake_sys.conveyor_stop();
    });

    conveyor_button_rev.released([]() {
        intake_sys.intake_stop();
        intake_sys.conveyor_stop();
    });

    goal_rush_arm.pressed([]() { clamper_sys.toggle_rush_arm(); });

    // ================ INIT ================

    while (true) {
        if (enableDrive) {

            if (!conveyor_button.pressing() && !conveyor_button_rev.pressing()) {
                intake_sys.intake_stop();
                intake_sys.conveyor_stop();
            }
            OdometryBase *odombase = &odom;
            Pose2d pos = odombase->get_position();

            double left = (double)con.Axis3.position() / 100;
            double right = (double)con.Axis2.position() / 100;
            // printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x(), pos.y(), pos.rotation().degrees());
            drive_sys.drive_tank(left, right, 1, TankDrive::BrakeType::None);
        }
        vexDelay(20);
    }

    // ================ PERIODIC ================
}

void testing() {

    con.ButtonX.pressed([]() {
        printf("running test");
        enableDrive = false;
        CommandController cc{
          new Async(new FunctionCommand([]() {
              while (true) {
                  printf(
                    "ODO X: %f ODO Y: %f, ODO ROT: %f, turnPID Error: %f\n", odom.get_position().x(),
                    odom.get_position().y(), odom.get_position().rotation().degrees(), turn_pid.get_error()
                  );
                  vexDelay(100);
              }
              return true;
          })),
          odom.SetPositionCmd({0, 0, 0}),
          drive_sys.DriveToPointCmd({24, 24}),
        };
        cc.run();
        enableDrive = true;
    });
}
