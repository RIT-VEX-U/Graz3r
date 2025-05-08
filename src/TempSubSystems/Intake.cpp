#include "TempSubSystems/Intake.h"
#include "robot-config.h"

IntakeSys::IntakeScreen::IntakeScreen(IntakeSys *scr) : self(scr) {}
void IntakeSys::IntakeScreen::update(bool, int, int) { return; }

void IntakeSys::IntakeScreen::draw(vex::brain::lcd &screen, bool fd, unsigned int fn) {
    vex::color seeingBlueCol = vex::color::black;
    if (self->seeing_blue()) {
        seeingBlueCol = vex::color::blue;
    }

    vex::color seeingRedCol = vex::color::black;
    if (self->seeing_red()) {
        seeingRedCol = vex::color::red;
    }

    static constexpr int COL_WIDTH = 50;
    static constexpr int COL_HEIGHT = 25;
    screen.setPenColor(vex::white);
    screen.setFillColor(vex::black);
    screen.printAt(37, 20, "Seeing");

    screen.drawRectangle(40, 20, COL_WIDTH, COL_HEIGHT, seeingBlueCol);
    screen.drawRectangle(40, 20 + COL_HEIGHT, COL_WIDTH, COL_HEIGHT, seeingRedCol);

    screen.setPenColor(vex::red);
    screen.setFillColor(vex::black);
    if (self->con_reversed_for_fix) {
        screen.printAt(100, 20, "STALL");
    }
    if (self->con_stopped_for_sort) {
        screen.printAt(100, 40, "SORT");
    }

    vex::color colorToRemove = vex::color::black;
    if (self->colorToRemove == BLUE && self->do_color_sort) {
        colorToRemove = vex::blue;
    } else if (self->colorToRemove == RED && self->do_color_sort) {
        colorToRemove = vex::red;
    }
    screen.setPenColor(vex::white);
    screen.setFillColor(vex::black);
    screen.printAt(40, 87, "Removing:");
    screen.drawRectangle(40, 100, COL_WIDTH, COL_HEIGHT, colorToRemove);

    double conv_pos = conveyor.position(vex::rotationUnits::deg);
    screen.setPenColor(vex::white);
    screen.setFillColor(vex::black);
    screen.printAt(120, 80, "deg: %.3f", conv_pos);
    screen.printAt(120, 95, "hue: %.3f", color_sensor.hue());
    screen.printAt(120, 110, "ner: %d", (int)color_sensor.isNearObject());

    screen.printAt(40, 137, "Seeing:");
    screen.drawRectangle(40, 140, COL_WIDTH, COL_HEIGHT, color_sensor.color());
}
screen::Page *IntakeSys::Page() { return new IntakeScreen(this); }
IntakeSys::IntakeSys() {
    task = vex::task(thread_fn, this);
    color_sensor.integrationTime(3);
}

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

void IntakeSys::fixConveyorStalling(bool true_to_fix) { this->fix_conveyor_stalling = true_to_fix; }

void IntakeSys::start_color_sort() { do_color_sort = true; }

void IntakeSys::stop_color_sort() { do_color_sort = false; }

void IntakeSys::color_to_remove(IntakeSys::RingColor ring_color) { colorToRemove = ring_color; }

void IntakeSys::conveyor_stalled_fix() {
    if (printConveyorData) {
        printf(
          "conveyor current: %f, conveyor dps: %f\n", conveyor.current(), conveyor.velocity(vex::velocityUnits::dps)
        );
    }
    if (conveyor.current(vex::currentUnits::amp) > 2.1 && conveyor.velocity(vex::velocityUnits::dps) < 100 &&
        !con_reversed_for_fix) {
        printf("conveyor stalled!\n");
        conveyor_state = IntakeState::OUT;
        intake_state = IntakeState::OUT;
        conveyor_stalled_timer.reset();
        con_reversed_for_fix = true;
    }
    if (conveyor_stalled_timer.time(timeUnits::msec) > 150 && con_reversed_for_fix) {
        con_reversed_for_fix = false;
        conveyor_state = IntakeState::IN;
        intake_state = IntakeState::IN;
    }
}

bool IntakeSys::seeing_red() {
    if ((color_sensor.hue() > 348 || color_sensor.hue() < 50) && color_sensor.isNearObject()) {
        return true;
    } else {
        return false;
    }
}

bool IntakeSys::seeing_blue() {
    if (color_sensor.hue() > 150 && color_sensor.hue() < 250 && color_sensor.isNearObject()) {
        return true;
    } else {
        return false;
    }
}

void IntakeSys::colorSort() {

    if (printColorHues) {
        printf("color hue: %f\n", color_sensor.hue());
    }

    double deg = conveyor.position(vex::degrees);
    bool see_blue = seeing_blue();
    bool see_red = seeing_red();
    bool currently_tracking = see_red || see_blue;
    bool should_sort_this_guy = (colorToRemove == BLUE && see_blue) || (colorToRemove == RED && see_red);
    if (should_sort_this_guy) {
        should_be_sorting = true;
    }
    if (currently_tracking && !was_tracking) {
        deg_at_start_of_tracking = deg;
    }
    if (was_tracking && !currently_tracking) {
        deg_at_end_of_tracking = deg;
    }
    double delta_deg = deg - deg_at_start_of_tracking;
    double thresh = 270;
    if (colorToRemove == RED) {
        thresh = 280;
    }

    if (should_be_sorting && delta_deg > thresh) {
        conveyorVolts = -1;
        con_stopped_for_sort = true;
        color_sort_timer.reset();
    }

    double time_thresh = 0.200;

    if ((color_sort_timer.value() > time_thresh) && con_stopped_for_sort) {
        conveyorVolts = 12;
        con_stopped_for_sort = false;
        should_be_sorting = false;
    }
    was_tracking = currently_tracking;
}

void IntakeSys::print_color_values(bool printColors) { printColorHues = printColors; }

void IntakeSys::print_conveyor_data(bool printConData) { printConveyorData = printConData; }

int IntakeSys::thread_fn(void *ptr) {
    IntakeSys &self = *(IntakeSys *)ptr;

    while (true) {
        if (self.do_color_sort) {
            self.colorSort();

            mcglight_board.set(1);
        } else {
            mcglight_board.set(0);
        }

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
        if (self.fix_conveyor_stalling) {
            self.conveyor_stalled_fix();
        }
        vexDelay(10);
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

AutoCommand *IntakeSys::FixConveyorStallingCmd(bool true_to_fix) {
    return new FunctionCommand([this, true_to_fix]() {
        fix_conveyor_stalling = true_to_fix;
        return true;
    });
}

AutoCommand *IntakeSys::PrintConveyorDataCmd(bool printConData) {
    return new FunctionCommand([this, printConData]() {
        printf("printing conveyor data!\n");
        this->printConveyorData = printConData;
        return true;
    });
}

AutoCommand *IntakeSys::PrintColorHuesCmd(bool printColorHues) {
    return new FunctionCommand([this, printColorHues]() {
        print_color_values(printColorHues);
        return true;
    });
}