#pragma once
#include "core/utils/command_structure/auto_command.h"
#include "vex.h"
class IntakeSys {
  public:
    IntakeSys();

    enum IntakeState {
        STOP,
        IN,
        OUT,
    };

    enum RingColor { BLUE, RED };

    IntakeState get_intake_state() { return intake_state; }
    IntakeState get_conveyor_state() { return conveyor_state; }

    void colorSort();

    bool seeing_red();

    bool seeing_blue();

    void intake(double volts = 12);

    void outtake(double volts = 12);

    void intake_stop();

    void conveyor_in(double volts = 12.0);
    void conveyor_stop();
    void conveyor_out(double volts = 12.0);

    void print_color_values(bool printColors);
    void print_conveyor_data(bool printConData);

    void start_color_sort();
    void stop_color_sort();

    void color_to_remove(IntakeSys::RingColor ring_color);

    void conveyor_stalled_fix();

    static int thread_fn(void *ptr);

    AutoCommand *IntakeCmd(double amt = 12.0);
    AutoCommand *OuttakeCmd(double amt = 12.0);
    AutoCommand *IntakeStopCmd();

    AutoCommand *ConveyorInCmd(double amt = 12.0);
    AutoCommand *ConveyorOutCmd(double amt = 12.0);
    AutoCommand *ConveyorStopCmd();

    AutoCommand *FixConveyorStallingCmd(bool true_to_fx);

    AutoCommand *PrintConveyorDataCmd(bool printConData);
    AutoCommand *PrintColorHuesCmd(bool printColorHues);

    void fixConveyorStalling(bool true_to_fix);

    AutoCommand *ColorSortCmd(bool do_color_sorting);

  private:
    vex::timer color_sort_timer;
    vex::timer conveyor_stalled_timer;
    vex::task task;
    IntakeState intake_state = IntakeState::STOP;
    IntakeState conveyor_state = IntakeState::STOP;
    RingColor colorToRemove = BLUE;
    double intakeVolts = 12;
    double conveyorVolts = 12;

    bool do_color_sort = false;
    bool fix_conveyor_stalling = false;

    bool printColorHues = false;
    bool printConveyorData = false;

    bool con_reversed_for_fix = false;
    bool con_stopped_for_sort = false;
};