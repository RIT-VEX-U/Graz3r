#include "competition/autonomous.h"
/**
 * Main entrypoint for the autonomous period
 */

// Negative side autonomous paths rush line goal, deposits it behind the line, intakes ring stack near (but not in)
// corner, picks up alliance-side goal, scores on alliance stake with ring in front of it, deposits alliance-side goal
// near positive corner, and scores remaining non-corner rings on rushed goal. The path ends contacting hang structure
// with a goal in possession.
void blue_negative_path();
void red_negative_path();

// Positive side autonomous paths rush line goal
void blue_positive_path();
void red_positive_path();

// Skills path
void skills_path();

// Main autonomous function
void autonomous() {
    while (!gps_pos_set) {
        vexDelay(10);
    }
    red_negative_path();
    // red_negative_path();
    // blue_positive_path();
    // red_positive_path();
};

// Autonomous path implementations (proceed at your own peril)

void blue_negative_path() {
    printf("running b- autonomous");

    intake_sys.start_color_sort();
    intake_sys.color_to_remove(IntakeSys::RingColor::RED);

    intake_sys.fixConveyorStalling(true);
    CommandController cc{
      // Odometry Logs
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

      // new DelayCommand(5000),

      // Goal Rush (Rush and Deposit)
      intake_sys.ConveyorOutCmd(),
      intake_sys.OuttakeCmd(),
      intake_sys.ConveyorStopCmd(),
      clamper_sys.RushCmd(ClamperSys::RushState::OUT),
      // drive_sys.PurePursuitCmd(PurePursuit::Path({{111.33, 103.18}, {94.2, 107.38}, {83, 109.2}}, 7), vex::forward)
      //   ->withTimeout(1.5),
      // clamper_sys.RushCmd(ClamperSys::RushState::IN),
      // drive_sys.DriveForwardCmd(24, vex::reverse)->withTimeout(1.5),

      drive_sys.DriveTankCmd(1, 1)->withTimeout(0.6),
      clamper_sys.RushCmd(ClamperSys::RushState::IN),
      drive_sys.DriveForwardCmd(24, vex::reverse)->withTimeout(1.5),
      // odom.SetPositionCmd(Pose2d(96+7, 96+12, odom.get_position().rotation())),
      // new DelayCommand(20000),
      drive_sys.TurnDegreesCmd(90, 1)->withTimeout(1.5),

      // Alliance-side Goal (Ring)
      drive_sys.TurnToPointCmd({120, 96}, vex::reverse, .6)->withTimeout(1.5),
      drive_sys.DriveToPointCmd({120, 96}, vex::reverse, .6)->withTimeout(2),
      intake_sys.IntakeCmd(),
      drive_sys.TurnToHeadingCmd(90, .5)->withTimeout(1),
      drive_sys.DriveToPointCmd({120, 120}, vex::forward)->withTimeout(1.5),

      // Alliance-side Goal (Grab Goal and Score)
      drive_sys.TurnToHeadingCmd(90, 1)->withTimeout(1),
      drive_sys.DriveToPointCmd({120, 88}, vex::reverse, .75),
      drive_sys.TurnToHeadingCmd(90, 1)->withTimeout(5),
      clamper_sys.AutoClampCmd(true),
      drive_sys.DriveToPointCmd({120, 72}, vex::reverse, .25), // 70-70.75
      clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
      new DelayCommand(250),
      intake_sys.ConveyorInCmd(),
      new DelayCommand(1850),
      intake_sys.ConveyorStopCmd(),
      drive_sys.DriveToPointCmd({120, 72}, vex::reverse, .25),

      // Alliance Stake
      drive_sys.TurnToHeadingCmd(0),
      wallstake_sys.SetSetPointCmd(from_degrees(26)),
      intake_sys.FixConveyorStallingCmd(false),
      intake_sys.ConveyorInCmd(),
      drive_sys.DriveForwardCmd(11, vex::forward, .5),
      drive_sys.DriveForwardCmd(2, vex::reverse, .5)->withTimeout(1.5),
      // drive_sys.TurnToPointCmd({144, 71}, vex::forward)->withTimeout(1.5),
      // drive_sys.TurnToHeadingCmd(0, 1)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(4.5, vex::forward, .5)->withTimeout(1.5),
      new DelayCommand(500),
      intake_sys.IntakeStopCmd(),
      intake_sys.ConveyorStopCmd(),
      // drive_sys.TurnToHeadingCmd(0, 0.7)->withTimeout(1.5),
      wallstake_sys.SetSetPointCmd(from_degrees(178)),
      new DelayCommand(800),
      drive_sys.DriveTankCmd(-0.8, 0.8)->withTimeout(0.25),
      drive_sys.DriveTankCmd(0.8, -0.8)->withTimeout(0.25),
      drive_sys.DriveTankCmd(-0.8, 0.8)->withTimeout(0.25),
      drive_sys.DriveTankCmd(0.8, -0.8)->withTimeout(0.25),
      drive_sys.DriveForwardCmd(14, vex::reverse, .75),
      wallstake_sys.SetSetPointCmd(from_degrees(5)),
      intake_sys.OuttakeCmd(),
      intake_sys.ConveyorOutCmd(),

      // Alliance-side Goal (Deposit)
      drive_sys.DriveToPointCmd({124, 52}, vex::reverse),
      clamper_sys.ClampCmd(ClamperSys::ClamperState::UNCLAMPED),
      new DelayCommand(50),
      drive_sys.DriveForwardCmd(6, vex::forward),

      // Retrieve Rush Goal
      drive_sys.DriveToPointCmd({120, 96}, vex::forward, .75),
      intake_sys.ConveyorStopCmd(),
      drive_sys.TurnToPointCmd({96, 114}, vex::reverse),
      intake_sys.IntakeStopCmd(),
      clamper_sys.AutoClampCmd(true),
      drive_sys.DriveForwardCmd(34, vex::reverse, .35),
      clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),

      // Touch hang structure
      drive_sys.DriveToPointCmd({84.5, 83.5}, vex::forward, .8),
      new DelayCommand(5000),

      drive_sys.TurnToPointCmd(blue_negative_pos.translation(), vex::reverse, .4),
      drive_sys.DriveToPointCmd(blue_negative_pos.translation(), vex::reverse, .4),
      drive_sys.TurnToHeadingCmd(blue_negative_pos.rotation().degrees(), 0.4),
    };

    cc.run();
    intake_sys.fixConveyorStalling(false);
}

AutoCommand *RecalGPSOr(Pose2d orelse) {
    return new FunctionCommand([bad_gps_count = 0, orelse]() mutable {
        int thresh = 50;

        if (gps_sensor.quality() != 100 && bad_gps_count < thresh) {
            bad_gps_count++;

            return false;
        }

        if (bad_gps_count >= thresh) {
            printf("Orelse: %.2f, %.2f    %.2f", orelse.x(), orelse.y(), orelse.rotation().degrees());
            return true;
        }
        double x = gps_sensor.xPosition(vex::distanceUnits::in) + 71.25;
        double y = gps_sensor.yPosition(vex::distanceUnits::in) + 71.25;
        double heading = deg2rad(wrap_degrees_180(gps_sensor.heading(vex::rotationUnits::deg) + 90));
        printf("Setting to %.2f, %.2f  %.2f\n", x, y, 180 * heading / PI);
        odom.set_position(Pose2d(x, y, from_radians(heading)));
        return true;
    });
}

void red_negative_path() {
    printf("running r- autonomous");
    intake_sys.start_color_sort();
    intake_sys.color_to_remove(IntakeSys::RingColor::BLUE);
    intake_sys.fixConveyorStalling(true);
    CommandController cc{
      // Odometry Logs
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

      // Goal Rush (Rush and Deposit)
      intake_sys.ConveyorOutCmd(),
      intake_sys.OuttakeCmd(),
      intake_sys.ConveyorStopCmd(),
      clamper_sys.RushCmd(ClamperSys::RushState::OUT),
      new DelayCommand(50),
      // drive_sys.DriveForwardCmd(25.5, vex::fwd)->withTimeout(1.5),
      drive_sys.DriveTankCmd(1, 1)->withTimeout(0.58),
      clamper_sys.RushCmd(ClamperSys::RushState::IN),
      drive_sys.DriveForwardCmd(24, vex::reverse)->withTimeout(1.5),
      // odom.SetPositionCmd(Pose2d(odom.get_position().x(), odom.get_position().y(), odom.get_position().rotation())),
      drive_sys.TurnDegreesCmd(179, 1)->withTimeout(1.5),

      drive_sys.TurnToHeadingCmd(120, 1)->withTimeout(1.5),
      // drive_sys.TurnToPointCmd({35, 120}, vex::reverse, .6)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(24, vex::reverse, 0.6)->withTimeout(1.5),
      // drive_sys.DriveToPointCmd({50, 118}, vex::reverse, .6)->withTimeout(2),
      // new DelayCommand(500),
      // RecalGPSOr(odom.get_position()),
      // new DelayCommand(5000),
      drive_sys.TurnToPointCmd({24, 120}, vex::forward)->withTimeout(3),
      intake_sys.IntakeCmd(),
      drive_sys.DriveToPointCmd({24, 120}, vex::forward, 0.6)->withTimeout(5),
      // drive_sys.TurnToHeadingCmd(90, 0.5)->withTimeout(2),

      // Alliance-side Goal (Grab Goal and Score)
      drive_sys.TurnToHeadingCmd(90, 1)->withTimeout(1),
      new DelayCommand(500),
      RecalGPSOr(Pose2d(24, 120, from_degrees(90))),
      new DelayCommand(500),
      drive_sys.DriveToPointCmd({24, 88}, vex::reverse, .75),
      // drive_sys.TurnToHeadingCmd(90, 1)->withTimeout(1),
      clamper_sys.AutoClampCmd(true),
      drive_sys.TurnToHeadingCmd(90, 1)->withTimeout(1),
      drive_sys.DriveToPointCmd({24, 72}, vex::reverse, .25), // 70-70.75
      clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
      new DelayCommand(250),
      intake_sys.ConveyorInCmd(),
      new DelayCommand(1850),
      intake_sys.ConveyorStopCmd(),
      drive_sys.DriveToPointCmd({24, 72}, vex::reverse, .25),

      // Alliance Stake
      drive_sys.TurnToHeadingCmd(180),
      wallstake_sys.SetSetPointCmd(from_degrees(26)),
      intake_sys.FixConveyorStallingCmd(false),
      intake_sys.ConveyorInCmd(),
      drive_sys.DriveForwardCmd(11, vex::forward, .5),
      drive_sys.DriveForwardCmd(2, vex::reverse, .5)->withTimeout(1.5),
      // drive_sys.TurnToPointCmd({144, 71}, vex::forward)->withTimeout(1.5),
      // drive_sys.TurnToHeadingCmd(0, 1)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(4.5, vex::forward, .5)->withTimeout(1.5),
      new DelayCommand(500),
      intake_sys.IntakeStopCmd(),
      intake_sys.ConveyorStopCmd(),
      // drive_sys.TurnToHeadingCmd(0, 0.7)->withTimeout(1.5),
      drive_sys.TurnToHeadingCmd(179.9, 0.5)->withTimeout(1.5),
      wallstake_sys.SetSetPointCmd(from_degrees(178)),
      new DelayCommand(800),
      drive_sys.DriveTankCmd(-0.8, 0.8)->withTimeout(0.25),
      drive_sys.DriveTankCmd(0.8, -0.8)->withTimeout(0.25),
      drive_sys.DriveTankCmd(-0.8, 0.8)->withTimeout(0.25),
      drive_sys.DriveTankCmd(0.8, -0.8)->withTimeout(0.25),
      // drive_sys.TurnToHeadingCmd(179, 1)->withTimeout(1)
      new DelayCommand(250),
      drive_sys.DriveForwardCmd(14, vex::reverse, .75),
      // drive_sys.DriveToPointCmd({24, 86}, vex::reverse, .75),
      wallstake_sys.SetSetPointCmd(from_degrees(5)),
      intake_sys.OuttakeCmd(),
      intake_sys.ConveyorOutCmd(),

      // Alliance-side Goal (Deposit)
      drive_sys.DriveToPointCmd({20, 52}, vex::reverse),
      clamper_sys.ClampCmd(ClamperSys::ClamperState::UNCLAMPED),
      new DelayCommand(50),
      // drive_sys.DriveForwardCmd(6, vex::forward),

      // Retrieve Rush Goal
      drive_sys.DriveToPointCmd({24, 96}, vex::forward, .75),
      intake_sys.ConveyorStopCmd(),
      drive_sys.TurnToPointCmd({52, 114}, vex::reverse),
      intake_sys.IntakeStopCmd(),
      clamper_sys.AutoClampCmd(true),
      drive_sys.DriveForwardCmd(48, vex::reverse, .35),
      clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),

      // Touch hang structure
      drive_sys.DriveToPointCmd({59.5, 83.5}, vex::forward, .8),
      new DelayCommand(5000),

      drive_sys.TurnToPointCmd(red_negative_pos.translation(), vex::reverse, .4),
      drive_sys.DriveToPointCmd(red_negative_pos.translation(), vex::reverse, .4),
      drive_sys.TurnToHeadingCmd(red_negative_pos.rotation().degrees(), 0.4),
    };

    cc.run();
    intake_sys.fixConveyorStalling(false);
}

void blue_positive_path() {
    intake_sys.color_to_remove(IntakeSys::RingColor::RED);
    printf("running b+ autonomous\n");
    CommandController cc{
      intake_sys.OuttakeCmd(), clamper_sys.RushCmd(ClamperSys::RushState::OUT),
      // rush goal 1
      drive_sys.PurePursuitCmd(
        PurePursuit::Path({{112.37, 41.92}, {98.15, 39.96}, {90.15, 35.96}, {87.56, 31.30}}, 7), vex::forward
      ),
      clamper_sys.RushCmd(ClamperSys::RushState::IN),
      drive_sys.PurePursuitCmd(PurePursuit::Path({{87.65, 38.63}, {88.82, 47.30}, {86.24, 51.70}}, 7), vex::reverse),
      // rush goal 2
      drive_sys.TurnDegreesCmd(90), drive_sys.TurnToHeadingCmd(144), intake_sys.IntakeCmd(),
      clamper_sys.RushCmd(ClamperSys::RushState::OUT), drive_sys.DriveForwardCmd(9),
      clamper_sys.RushCmd(ClamperSys::RushState::IN), drive_sys.DriveForwardCmd(26, vex::reverse),
      // get goal 1
      drive_sys.TurnToPointCmd({79.5, 33.5}, vex::reverse),
      // clamper_sys.AutoClampCmd(true),
      drive_sys.DriveForwardCmd(15, vex::reverse, 0.2), clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
      drive_sys.TurnDegreesCmd(15)->withTimeout(1), drive_sys.TurnDegreesCmd(-15)->withTimeout(1),
      // get ring 1
      intake_sys.ColorSortCmd(true), drive_sys.TurnToPointCmd({96, 24}, vex::forward), intake_sys.ConveyorInCmd(),
      intake_sys.IntakeCmd(),
      drive_sys.DriveForwardCmd(32, vex::forward, 0.4)->withCancelCondition(drive_sys.DriveStalledCondition(1)),
      // get ring 2
      drive_sys.TurnToPointCmd({120, 24}), drive_sys.DriveToPointCmd({120, 24}, vex::forward, 0.4),
      new DelayCommand(1000),
      // corner nightmare nightmare nightmare
      drive_sys.TurnToHeadingCmd(303.5), intake_sys.OuttakeCmd(),
      drive_sys.DriveForwardCmd(24, vex::forward, 0.4)
        ->withTimeout(1)
        ->withCancelCondition(drive_sys.DriveStalledCondition(0.5)),
      intake_sys.IntakeCmd(), intake_sys.FixConveyorStallingCmd(true), new DelayCommand(500),
      drive_sys.DriveForwardCmd(10, vex::reverse, 0.4), intake_sys.OuttakeCmd(),
      drive_sys.DriveForwardCmd(14, vex::forward, 0.4)
        ->withTimeout(1)
        ->withCancelCondition(drive_sys.DriveStalledCondition(0.5)),
      intake_sys.IntakeCmd(), new DelayCommand(500), drive_sys.DriveForwardCmd(10, vex::reverse, 0.4),
      intake_sys.OuttakeCmd(),
      drive_sys.DriveForwardCmd(14, vex::forward, 0.4)
        ->withTimeout(1)
        ->withCancelCondition(drive_sys.DriveStalledCondition(0.5)),
      intake_sys.IntakeCmd(), new DelayCommand(500), drive_sys.DriveForwardCmd(10, vex::reverse, 0.4),
      intake_sys.OuttakeCmd(),
      drive_sys.DriveForwardCmd(14, vex::forward, 0.4)
        ->withTimeout(1)
        ->withCancelCondition(drive_sys.DriveStalledCondition(0.5)),
      intake_sys.IntakeCmd(), new DelayCommand(500), drive_sys.DriveForwardCmd(24, vex::reverse, 0.4),
      drive_sys.TurnToHeadingCmd(135)->withTimeout(1), clamper_sys.ClampCmd(ClamperSys::ClamperState::UNCLAMPED),
      drive_sys.DriveForwardCmd(5), clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
      drive_sys.DriveForwardCmd(29, vex::reverse)
        ->withTimeout(1)
        ->withCancelCondition(drive_sys.DriveStalledCondition(0.5)),
      clamper_sys.ClampCmd(ClamperSys::ClamperState::UNCLAMPED),
      // get goal 2
      // delete if we dont get lidar
      // drive_sys.DriveForwardCmd(24),
      // drive_sys.TurnToHeadingCmd(327),
      // clamper_sys.AutoClampCmd(true),
      // drive_sys.DriveForwardCmd(33, vex::reverse, 0.2),
      // clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
    };
    cc.run();
    intake_sys.conveyor_stop();
    intake_sys.intake_stop();
    intake_sys.fixConveyorStalling(false);
    intake_sys.stop_color_sort();
    clamper_sys.auto_clamp_off();
}

void red_positive_path() {
    intake_sys.color_to_remove(IntakeSys::RingColor::BLUE);
    printf("running r+ autonomous\n");
    CommandController cc{
      // put intake down
      intake_sys.OuttakeCmd(),
      // goal rush 1
      clamper_sys.RushCmd(ClamperSys::RushState::OUT),
      drive_sys.PurePursuitCmd(PurePursuit::Path({{36.47, 40.59}, {49.05, 37.84}, {59.86, 34.19}}, 7), vex::forward),
      clamper_sys.RushCmd(ClamperSys::RushState::IN),
      drive_sys.DriveToPointCmd({49.7, 40.9}, vex::reverse),
      intake_sys.IntakeStopCmd(),
      // goal rush 2
      drive_sys.TurnToHeadingCmd(65),
      clamper_sys.RushCmd(ClamperSys::RushState::OUT),
      drive_sys.DriveForwardCmd(26),
      clamper_sys.RushCmd(ClamperSys::RushState::IN),
      drive_sys.DriveForwardCmd(22, vex::reverse),
      // get goal 1
      drive_sys.TurnToPointCmd({62.5, 24.5}, vex::reverse),
      // clamper_sys.AutoClampCmd(true),
      drive_sys.DriveForwardCmd(18, vex::reverse, 0.2),
      clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
      drive_sys.TurnDegreesCmd(15)->withTimeout(1),
      drive_sys.TurnDegreesCmd(-15)->withTimeout(1),
      // get ring 1
      intake_sys.ColorSortCmd(true),
      drive_sys.TurnToPointCmd({52.5, 5.5}, vex::forward),
      intake_sys.ConveyorInCmd(),
      intake_sys.IntakeCmd(),
      drive_sys.DriveForwardCmd(14, vex::forward, 0.4)->withCancelCondition(drive_sys.DriveStalledCondition(2)),
      // get ring 2
      drive_sys.TurnToPointCmd({24, 24}),
      drive_sys.DriveToPointCmd({24, 24}, vex::forward, 0.4),
      new DelayCommand(1000),
      // corner nightmare nightmare nightmare
      drive_sys.TurnToHeadingCmd(213.5),
      intake_sys.OuttakeCmd(),
      drive_sys.DriveForwardCmd(24, vex::forward, 0.4)
        ->withTimeout(1)
        ->withCancelCondition(drive_sys.DriveStalledCondition(0.5)),
      intake_sys.IntakeCmd(),
      intake_sys.FixConveyorStallingCmd(true),
      new DelayCommand(500),
      drive_sys.DriveForwardCmd(10, vex::reverse, 0.4),
      intake_sys.OuttakeCmd(),
      drive_sys.DriveForwardCmd(14, vex::forward, 0.4)
        ->withTimeout(1)
        ->withCancelCondition(drive_sys.DriveStalledCondition(0.5)),
      intake_sys.IntakeCmd(),
      new DelayCommand(500),
      drive_sys.DriveForwardCmd(10, vex::reverse, 0.4),
      intake_sys.OuttakeCmd(),
      drive_sys.DriveForwardCmd(14, vex::forward, 0.4)
        ->withTimeout(1)
        ->withCancelCondition(drive_sys.DriveStalledCondition(0.5)),
      intake_sys.IntakeCmd(),
      new DelayCommand(500),
      drive_sys.DriveForwardCmd(10, vex::reverse, 0.4),
      intake_sys.OuttakeCmd(),
      drive_sys.DriveForwardCmd(14, vex::forward, 0.4)
        ->withTimeout(1)
        ->withCancelCondition(drive_sys.DriveStalledCondition(0.5)),
      intake_sys.IntakeCmd(),
      new DelayCommand(500),
      drive_sys.DriveForwardCmd(24, vex::reverse, 0.4),
      drive_sys.TurnToHeadingCmd(48)->withTimeout(1),
      clamper_sys.ClampCmd(ClamperSys::ClamperState::UNCLAMPED),
      drive_sys.DriveForwardCmd(5),
      clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
      drive_sys.DriveForwardCmd(29, vex::reverse)
        ->withTimeout(1)
        ->withCancelCondition(drive_sys.DriveStalledCondition(0.5)),
      // get goal 2
      // delete if we dont have lidar
      drive_sys.DriveForwardCmd(24),
      drive_sys.TurnToHeadingCmd(237),
      // clamper_sys.AutoClampCmd(true),
      drive_sys.DriveForwardCmd(33, vex::reverse, 0.2),
      clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
    };
    cc.run();
    intake_sys.fixConveyorStalling(false);
    intake_sys.stop_color_sort();
    clamper_sys.auto_clamp_off();
}