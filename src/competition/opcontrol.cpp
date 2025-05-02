#include "competition/opcontrol.h"
#include "competition/autonomous.h"
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

    wallstake_toggler.pressed([]() {
        wallstake_sys.hold = true;
        if (wallstake_sys.get_angle().degrees() < 10 || wallstake_motor.velocity(vex::velocityUnits::dps) > 5) {
            wallstake_sys.set_setpoint(from_degrees(22));
            wallstake_sol.set(false);
        } else if (wallstake_sys.get_angle().degrees() > 10) {
            wallstake_sys.set_setpoint(from_degrees(180));
            wallstake_sol.set(true);
        }
    });

    wallstake_stow.pressed([]() {
        wallstake_sys.hold = true;
        wallstake_sys.set_setpoint(from_degrees(5));
        wallstake_sol.set(false);
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
            double right = (double)con.Axis1.position() / 100;
            // printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x(), pos.y(), pos.rotation().degrees());
            drive_sys.drive_arcade(left, right, 1, TankDrive::BrakeType::None);
        }
        vexDelay(20);
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
