#include "competition/autonomous.h"
/**
 * Main entrypoint for the autonomous period
*/

// Negative side autonomous paths rush line goal, deposits it behind the line, intakes ring stack near (but not in) corner, picks up alliance-side goal, scores on alliance stake with ring in front of it, deposits alliance-side goal near positive corner, and scores remaining non-corner rings on rushed goal. The path ends contacting hang structure with a goal in possession.
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
    //red_negative_path();
    //blue_positive_path();
    //red_positive_path();
};


// Autonomous path implementations (proceed at your own peril)

void blue_negative_path() {
    printf("running b- autonomous");

    intake_sys.fixConveyorStalling(true);
    CommandController cc{
        // Odometry Logs
        new Async(new FunctionCommand([]() {
            while (true) {
                printf("ODO X: %f ODO Y: %f, ODO ROT: %f, turnPID Error: %f\n", odom.get_position().x(),
                odom.get_position().y(), odom.get_position().rotation().degrees(), turn_pid.get_error());
                vexDelay(100);
            }
            return true;
        })),

        // Goal Rush (Rush and Deposit)
        intake_sys.OuttakeCmd(),
        clamper_sys.RushCmd(ClamperSys::RushState::OUT),
        // Will's Coords for r+: {{32.67, 40.82}, {49.80, 36.62}, {58.94, 33.72}}
        // My test: {{112.75, 98.25}, {94, 98.75}, {83.5,102.25}}
        drive_sys.PurePursuitCmd(PurePursuit::Path({{111.33, 103.18}, {94.2, 107.38}, {85.06,110.28}}, 7), vex::forward)->withTimeout(1.5),
        clamper_sys.RushCmd(ClamperSys::RushState::IN),
        drive_sys.DriveForwardCmd(24, vex::reverse),
        new DelayCommand(450),

        // Alliance-side Goal (Ring)
        drive_sys.DriveToPointCmd({121, 96}, vex::reverse, .8),
        intake_sys.IntakeCmd(),
        drive_sys.TurnToHeadingCmd(90, .75),
        drive_sys.DriveForwardCmd(20, vex::forward),

        // Aliance-side Goal (Grab Goal and Score)
        drive_sys.DriveForwardCmd(24, vex::reverse, 1, .5),
        clamper_sys.AutoClampCmd(true),
        drive_sys.DriveToPointCmd({120.5, 72}, vex::reverse, .25),
        clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
        intake_sys.ConveyorInCmd(),
        new DelayCommand(1850),


        intake_sys.IntakeStopCmd(),
        intake_sys.ConveyorStopCmd()
        // Alliance Stake
        // Retrieve Rush Goal
        // Score Stack near Goal
        // Score Corner Rings (optional)
        // Touch Hang structure
    };
    intake_sys.fixConveyorStalling(false);
    
    cc.run();
}

void red_negative_path() {
    printf("running r- autonomous");

    intake_sys.fixConveyorStalling(true);
    CommandController cc{
        // Odometry Logs
        new Async(new FunctionCommand([]() {
            while (true) {
                printf("ODO X: %f ODO Y: %f, ODO ROT: %f, turnPID Error: %f\n", odom.get_position().x(),
                odom.get_position().y(), odom.get_position().rotation().degrees(), turn_pid.get_error());
                vexDelay(100);
            }
            return true;
        }))
    };
    intake_sys.fixConveyorStalling(false);
    
    cc.run();
}

void blue_positive_path() {
    printf("running b+ autonomous");

    intake_sys.fixConveyorStalling(true);
    CommandController cc{
        // Odometry Logs
        new Async(new FunctionCommand([]() {
            while (true) {
                printf("ODO X: %f ODO Y: %f, ODO ROT: %f, turnPID Error: %f\n", odom.get_position().x(),
                odom.get_position().y(), odom.get_position().rotation().degrees(), turn_pid.get_error());
                vexDelay(100);
            }
            return true;
        }))
    };
    intake_sys.fixConveyorStalling(false);
    
    cc.run();
}

void red_positive_path() { // ALLEGEDLY
    printf("running r+ autonomous");

    intake_sys.fixConveyorStalling(true);
    CommandController cc{
        // Odometry Logs
        new Async(new FunctionCommand([]() {
            while (true) {
                printf("ODO X: %f ODO Y: %f, ODO ROT: %f, turnPID Error: %f\n", odom.get_position().x(),
                odom.get_position().y(), odom.get_position().rotation().degrees(), turn_pid.get_error());
                vexDelay(100);
            }
            return true;
        })),

        //
        intake_sys.OuttakeCmd(),
        clamper_sys.RushCmd(ClamperSys::RushState::OUT),
        drive_sys.PurePursuitCmd(PurePursuit::Path({{32.67, 40.82}, {49.80, 36.62}, {58.94, 33.72}}, 7), vex::forward)->withTimeout(1.5),
        clamper_sys.RushCmd(ClamperSys::RushState::IN),
        drive_sys.DriveForwardCmd(24, vex::reverse),
        drive_sys.TurnToHeadingCmd(48),
        intake_sys.IntakeCmd(),
        clamper_sys.RushCmd(ClamperSys::RushState::OUT),
        drive_sys.DriveForwardCmd(35)->withTimeout(1),
        clamper_sys.RushCmd(ClamperSys::RushState::IN),
        drive_sys.DriveForwardCmd(31, vex::reverse),
        intake_sys.OuttakeCmd(),
        drive_sys.TurnToPointCmd({50, 30.5}, vex::reverse),
        clamper_sys.AutoClampCmd(true),
        drive_sys.DriveForwardCmd(22, vex::reverse, 0.4),
        clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
        drive_sys.TurnDegreesCmd(15)->withTimeout(0.4),
        drive_sys.TurnDegreesCmd(-15)->withTimeout(0.4),
        intake_sys.ColorSortCmd(true),
        drive_sys.TurnToPointCmd({42.5, 15}, vex::forward),
        intake_sys.IntakeCmd(),
        intake_sys.ConveyorInCmd(),
        drive_sys.DriveForwardCmd(18, vex::forward, 0.4),
        drive_sys.PurePursuitCmd(PurePursuit::Path({{40, 24}, {24, 24}, {12, 24}}, 7), vex::forward, 0.4),
        drive_sys.DriveToPointCmd({24, 24}, vex::reverse),
        drive_sys.TurnToHeadingCmd(223)->withTimeout(1),
        intake_sys.OuttakeCmd(),
        drive_sys.DriveForwardCmd(24, vex::forward, 0.4)->withTimeout(1),
        intake_sys.IntakeCmd(),
        new DelayCommand(500),
        drive_sys.DriveForwardCmd(10, vex::reverse, 0.4),
        intake_sys.OuttakeCmd(),
        drive_sys.DriveForwardCmd(14, vex::forward, 0.4)->withTimeout(1),
        intake_sys.IntakeCmd(),
        new DelayCommand(500),
        drive_sys.DriveForwardCmd(10, vex::reverse, 0.4),
        intake_sys.OuttakeCmd(),
        drive_sys.DriveForwardCmd(14, vex::forward, 0.4)->withTimeout(1),
        intake_sys.IntakeCmd(),
        new DelayCommand(500),
        drive_sys.DriveForwardCmd(10, vex::reverse, 0.4),
        intake_sys.OuttakeCmd(),
        drive_sys.DriveForwardCmd(14, vex::forward, 0.4)->withTimeout(1),
        intake_sys.IntakeCmd(),
        new DelayCommand(500),
        drive_sys.DriveForwardCmd(24, vex::reverse, 0.4),
        //   drive_sys.TurnToHeadingCmd(48),
        //   clamper_sys.ClampCmd(ClamperSys::ClamperState::UNCLAMPED),
        //   drive_sys.DriveForwardCmd(24, vex::reverse),
        //   drive_sys.DriveForwardCmd(24),
        //   drive_sys.TurnToHeadingCmd(227.5),
        //   drive_sys.DriveForwardCmd(32, vex::reverse, 0.4),
        //   clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
    };
    intake_sys.fixConveyorStalling(false);
    
    cc.run();
}