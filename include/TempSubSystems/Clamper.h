#pragma once
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

    void clamp();
    void unclamp();

    void rush_out();
    void rush_in();

    void auto_clamp_on();
    void auto_clamp_off();

    bool is_auto_clamping();

    void auto_clamp();

    bool is_clamped();
    bool is_rush_out();

    double rush_heading(Translation2d toward_point);

    AutoCommand *ClampCmd(ClamperState state);
    AutoCommand *RushCmd(RushState state);

    AutoCommand *AutoClampCmd(bool do_auto_clamping);

  private:
    static int thread_fn(void *ptr);

    bool doAutoClamping = false;

    vex::task task;
    ClamperState clamper_state = ClamperState::UNCLAMPED;
    RushState rush_arm_state = RushState::IN;
};