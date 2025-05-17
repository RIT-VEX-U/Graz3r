#include "competition/opcontrol.h"
#include "competition/autonomous.h"
#include "robot-config.h"
#include "vex.h"

#include "core/utils/math/eigen_interface.h"

void testing();
/**
 * Main entrypoint for the driver control period
 */
bool enableDrive = true;
void opcontrol() {
    // autonomous();
    // testing();

    intake_sys.stop_color_sort();
    // intake_sys.color_to_remove(IntakeSys::RingColor::BLUE);

    wallstake_toggler.pressed([]() {
        wallstake_sys.hold = true;
        if (wallstake_sys.get_angle().degrees() < 10 || wallstake_motor.velocity(vex::velocityUnits::dps) > 5) {
            wallstake_sys.set_setpoint(from_degrees(27));
            wallstake_sol.set(false);
        } else if (wallstake_sys.get_angle().degrees() > 10) {
            wallstake_sys.set_setpoint(from_degrees(143));
            wallstake_sol.set(true);
        }
    });

    wallstake_stow.pressed([]() {
        wallstake_sys.hold = true;
        wallstake_sys.set_setpoint(from_degrees(5));
        wallstake_sol.set(false);
    });

    wallstake_alliancestake.pressed([]() {
        wallstake_sys.hold = true;
        if (wallstake_sys.get_angle().degrees() > 90) {
            wallstake_sys.set_setpoint(from_degrees(5));
            wallstake_sol.set(false);
        } else {
            wallstake_sys.set_setpoint(from_degrees(178));
            wallstake_sol.set(false);
        }
    });

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

    climb_button.pressed([]() {
        climb_sol.set(!climb_sol.value());
    });

    goal_rush_arm.pressed([]() { clamper_sys.toggle_rush_arm(); });

    // ================ INIT ================



    int count = 0;
    while (true) {
        if (enableDrive) {
            count++;

            if (!conveyor_button.pressing() && !conveyor_button_rev.pressing()) {
                intake_sys.intake_stop();
                intake_sys.conveyor_stop();
            }
            OdometryBase *odombase = &odom;
            Pose2d pos = odom.get_position();

            // double left = (double)con.Axis3.position() / 100;
            // double right = (double)con.Axis1.position() / 100;

            double left = (double)con.Axis3.position() / 100;
            double right = (double)con.Axis2.position() / 100;
            // printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x(), pos.y(), pos.rotation().degrees());
            // drive_sys.drive_arcade(left, right, 1, TankDrive::BrakeType::None);
            drive_sys.drive_tank(left, right, 1, TankDrive::BrakeType::None);



            if (count % 10 == 0) {
                printf(
                  "ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x(), pos.y(),
                  pos.rotation().degrees()
                );
            }
        }
        vexDelay(10);
    }

    // ================ PERIODIC ================
}

void testing() {

    con.ButtonUp.pressed([]() {
        printf("resetting position");
        enableDrive = false;
        CommandController cc{
          clamper_sys.ClampCmd(ClamperSys::ClamperState::UNCLAMPED),
          clamper_sys.RushCmd(ClamperSys::RushState::IN),
          drive_sys.DriveToPointCmd({19.4, 42.4}),
          drive_sys.TurnToHeadingCmd(0),
        };
        cc.run();
        enableDrive = true;
    });
    con.ButtonRight.pressed([]() {
        printf(
          "{%.2f, %.2f}, ODO ROT: %f\n", odom.get_position().x(), odom.get_position().y(),
          odom.get_position().rotation().degrees()
        );
    });

    con.ButtonX.pressed([]() {
        printf("button x pressed!\n");
        if (!enableDrive) {
            return;
        } else {
            enableDrive = false;
        }
        autonomous();
        enableDrive = true;
    });
}
