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
    blue_negative_path();
    //  red_negative_path();
    //  blue_positive_path();
    // red_positive_path();
};

// Autonomous path implementations (proceed at your own peril)

void blue_negative_path() {
    printf("running b- autonomous");

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
      //drive_sys.PurePursuitCmd(PurePursuit::Path({{111.33, 103.18}, {94.2, 107.38}, {84.25, 110.28}}, 7), vex::forward)//{85.1, 110.325}; {85.06, 110.28} - {85.5, 110.75}
      drive_sys.PurePursuitCmd(PurePursuit::Path({{111.33, 103.18}, {94.2, 107.38}, {83, 109.2}}, 7), vex::forward)
        ->withTimeout(1.5),
      clamper_sys.RushCmd(ClamperSys::RushState::IN),
      drive_sys.DriveForwardCmd(24, vex::reverse)->withTimeout(1.5),
      // drive_sys.TurnDegreesCmd(60),

      // Alliance-side Goal (Ring)
      drive_sys.TurnToPointCmd({118, 96}, vex::reverse, .4)->withTimeout(1.5),
      drive_sys.DriveToPointCmd({118, 96}, vex::reverse, .35),
      intake_sys.IntakeCmd(),
      drive_sys.TurnToHeadingCmd(90, .75),
      drive_sys.DriveForwardCmd(20, vex::forward),

      // Alliance-side Goal (Grab Goal and Score)
      drive_sys.DriveToPointCmd({118, 88}, vex::reverse, .8),
      clamper_sys.AutoClampCmd(true),
      drive_sys.DriveToPointCmd({118, 70.5}, vex::reverse, .125), //70-70.75
      clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
      new DelayCommand(250),
      intake_sys.ConveyorInCmd(),
      new DelayCommand(1850),
      intake_sys.ConveyorStopCmd(),

      // Alliance Stake
      drive_sys.TurnToHeadingCmd(.5),
      wallstake_sys.SetSetPointCmd(from_degrees(26)),
      intake_sys.FixConveyorStallingCmd(false),
      intake_sys.ConveyorInCmd(),
      drive_sys.DriveForwardCmd(11, vex::forward, .5),
      drive_sys.DriveForwardCmd(2, vex::reverse, .5),
      drive_sys.TurnToHeadingCmd(0),
      drive_sys.DriveForwardCmd(3, vex::forward, .5),
      new DelayCommand(1750),
      intake_sys.IntakeStopCmd(),
      intake_sys.ConveyorStopCmd(),
      wallstake_sys.SetSetPointCmd(from_degrees(150)),
      new DelayCommand(2225),
      drive_sys.DriveForwardCmd(11, vex::reverse, .75),
      wallstake_sys.SetSetPointCmd(from_degrees(5)),
      intake_sys.OuttakeCmd(),
      intake_sys.ConveyorOutCmd(),

      // Alliance-side Goal (Deposit)
      drive_sys.DriveToPointCmd({130, 48}, vex::reverse),
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
      drive_sys.DriveToPointCmd({83.5,83.5}, vex::forward, .8)
    };
    
    cc.run();
    intake_sys.fixConveyorStalling(false);
}

void red_negative_path() {
    printf("running r- autonomous");

    intake_sys.fixConveyorStalling(true);
    CommandController cc{// Odometry Logs
                         new Async(new FunctionCommand([]() {
                             while (true) {
                                 printf(
                                   "ODO X: %f ODO Y: %f, ODO ROT: %f, turnPID Error: %f\n", odom.get_position().x(),
                                   odom.get_position().y(), odom.get_position().rotation().degrees(),
                                   turn_pid.get_error()
                                 );
                                 vexDelay(100);
                             }
                             return true;
                         }))
    };
    intake_sys.fixConveyorStalling(false);

    cc.run();
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