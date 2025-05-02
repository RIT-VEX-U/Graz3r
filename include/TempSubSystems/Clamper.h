#pragma once
#include "core/subsystems/tank_drive.h"
#include "core/utils/command_structure/auto_command.h"
#include "core/utils/geometry.h"
#include "vex.h"

class ClamperSys {
  public:
    ClamperSys();
    enum ClamperState {
        CLAMPED,
        UNCLAMPED,
    };
    enum RushState {
        OUT,
        IN,
    };

    void toggle_clamp();

    void toggle_rush_arm();

    void goalclamp();
    void goalunclamp();

    void rush_out();
    void rush_in();

    void auto_clamp_on();
    void auto_clamp_off();

    void print_clamping_dist(bool true_to_print);

    bool is_auto_clamping();

    void auto_clamp();

    bool is_clamped();
    bool is_rush_out();

    double rush_heading(Translation2d toward_point);
    double rush_heading(Translation2d toward_point, Pose2d from_pose);

    std::vector<Translation2d> last_rush_points(
      Translation2d second_last_point, Translation2d final_rush_point, Translation2d toward_point, double bank_radius
    );

    AutoCommand *ClampCmd(ClamperState state);
    AutoCommand *RushCmd(RushState state);

    AutoCommand *AutoClampCmd(bool do_auto_clamping);

    AutoCommand *PrintClampingDistCmd(bool true_to_print);

  private:
    static int thread_fn(void *ptr);

    bool doAutoClamping = false;

    bool printClampingDist = false;

    vex::task task;
    ClamperState clamper_state = ClamperState::UNCLAMPED;
    RushState rush_arm_state = RushState::IN;
};