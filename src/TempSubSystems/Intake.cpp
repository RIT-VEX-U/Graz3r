#include "TempSubSystems/Intake.h"
#include "robot-config.h"

IntakeSys::IntakeSys() { task = vex::task(thread_fn, this); }

void IntakeSys::intake(double volts) {
    intake_state = IntakeState::IN;
    intakeVolts = volts;
}

void IntakeSys::outtake(double volts) {
    intake_state = IntakeState::OUT;
    intakeVolts = volts;
}

void IntakeSys::intake_stop() { intake_state = IntakeState::STOP; }

void IntakeSys::conveyor_in(double volts) {
    conveyor_state = IntakeState::IN;
    conveyorVolts = volts;
}
void IntakeSys::conveyor_stop() { conveyor_state = IntakeState::STOP; }
void IntakeSys::conveyor_out(double volts) {
    conveyor_state = IntakeState::OUT;
    conveyorVolts = volts;
}

void IntakeSys::fixConveyorStalling(bool fixConStalling) { fix_conveyor_stalling = fixConStalling; }

void IntakeSys::start_color_sort() { do_color_sort = true; }

void IntakeSys::stop_color_sort() { do_color_sort = false; }

bool IntakeSys::seeing_red() {
    if (color_sensor.hue() > 348 || color_sensor.hue() < 50) {
        return true;
    } else {
        return false;
    }
}
void IntakeSys::conveyor_stalled_fix() {
    // printf("conveyor current: %f\n", conveyor.current());
    if (conveyor.current(vex::currentUnits::amp) > 2.35 && !con_reversed_for_fix) {
        printf("conveyor stalled!\n");
        conveyor_state = IntakeState::OUT;
        conveyor_stalled_timer.reset();
        con_reversed_for_fix = true;
    }
    if (conveyor_stalled_timer.time(timeUnits::msec) > 100 && con_reversed_for_fix) {
        con_reversed_for_fix = false;
        conveyor_state = IntakeState::IN;
    }
}

bool IntakeSys::seeing_blue() {
    if (color_sensor.hue() > 180 && color_sensor.hue() < 230) {
        return true;
    } else {
        return false;
    }
}

void IntakeSys::colorSort() {
    printf("color hue: %f\n", color_sensor.hue());
    if (color_to_remove == BLUE && seeing_blue()) {
        printf("seeing blue!\n");
        conveyor_state = IntakeState::STOP;
        con_stopped_for_sort = true;
        color_sort_timer.reset();
    } else if (color_to_remove == RED && seeing_red()) {
        printf("seeing red!\n");
        conveyor_state = IntakeState::STOP;
        con_stopped_for_sort = true;
        color_sort_timer.reset();
    }
    if (color_sort_timer.time(vex::timeUnits::msec) > 100 && con_stopped_for_sort) {
        printf("resetting conveyor\n");
        conveyor_state = IntakeState::IN;
        con_stopped_for_sort = false;
    }
}

int IntakeSys::thread_fn(void *ptr) {
    IntakeSys &self = *(IntakeSys *)ptr;

    while (true) {
        if (self.intake_state == IntakeState::IN) {
            intake_motor.spin(vex::fwd, self.intakeVolts, vex::volt);
            // printf("IntakeState IN \n");
        } else if (self.intake_state == IntakeState::OUT) {
            // printf("IntakeState OUT \n");
            intake_motor.spin(vex::reverse, self.intakeVolts, vex::volt);
        } else if (self.intake_state == IntakeState::STOP) {
            intake_motor.stop();
        }
        if (self.conveyor_state == IntakeState::IN) {
            intake_motor.spin(vex::fwd, self.intakeVolts, vex::volt);
            conveyor.spin(vex::fwd, self.conveyorVolts, vex::volt);
        } else if (self.conveyor_state == IntakeState::OUT) {
            conveyor.spin(vex::reverse, self.conveyorVolts, vex::volt);
        } else if (self.conveyor_state == IntakeState::STOP) {
            conveyor.stop();
        }
        if (self.do_color_sort) {
            self.colorSort();

            mcglight_board.set(1);
        } else {
            mcglight_board.set(0);
        }
        if (self.fix_conveyor_stalling) {
            self.conveyor_stalled_fix();
        }
        vexDelay(20);
    }
    return 0;
}

AutoCommand *IntakeSys::ColorSortCmd(bool do_color_sorting) {
    return new FunctionCommand([this, do_color_sorting]() {
        this->do_color_sort = do_color_sorting;
        return true;
    });
}

AutoCommand *IntakeSys::IntakeCmd(double amt) {
    return new FunctionCommand([this, amt]() {
        intake(amt);
        return true;
    });
}

AutoCommand *IntakeSys::OuttakeCmd(double amt) {
    return new FunctionCommand([this, amt]() {
        outtake(amt);
        return true;
    });
}

AutoCommand *IntakeSys::IntakeStopCmd() {
    return new FunctionCommand([this]() {
        intake_stop();
        return true;
    });
}

AutoCommand *IntakeSys::ConveyorInCmd(double amt) {
    return new FunctionCommand([this, amt]() {
        conveyor_in(amt);
        return true;
    });
}

AutoCommand *IntakeSys::ConveyorOutCmd(double amt) {
    return new FunctionCommand([this, amt]() {
        conveyor_out(amt);
        return true;
    });
}

AutoCommand *IntakeSys::ConveyorStopCmd() {
    return new FunctionCommand([this]() {
        conveyor_stop();
        return true;
    });
}