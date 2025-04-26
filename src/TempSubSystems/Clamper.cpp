#include "TempSubSystems/Clamper.h"
#include "robot-config.h"

ClamperSys::ClamperSys() { task = vex::task(thread_fn, this); }

void ClamperSys::clamp() { clamper_state = ClamperState::CLAMPED; };
void ClamperSys::unclamp() { clamper_state = ClamperState::UNCLAMPED; };

void ClamperSys::rush_out() { rush_arm_state = RushState::OUT; };
void ClamperSys::rush_in() { rush_arm_state = RushState::IN; };

void ClamperSys::toggle_clamp() {
    doAutoClamping = false;
    if (is_clamped()) {
        unclamp();
    } else {
        clamp();
    }
}

void ClamperSys::toggle_rush_arm() {
    if (is_rush_out()) {
        rush_in();
    } else {
        rush_out();
    }
}

void ClamperSys::auto_clamp() {
    if (clamper_sensor.objectDistance(vex::distanceUnits::mm) <= 50) {
        clamper_state = ClamperState::CLAMPED;
    } else {
        clamper_state = ClamperState::UNCLAMPED;
    }
}

void ClamperSys::auto_clamp_on() { doAutoClamping = true; }

void ClamperSys::auto_clamp_off() { doAutoClamping = false; }

bool ClamperSys::is_auto_clamping() { return doAutoClamping; }

// returns true if the piston is clamped down
bool ClamperSys::is_clamped() { return goal_grabber_sol.value(); };
bool ClamperSys::is_rush_out() { return goal_rush_sol.value(); };

double ClamperSys::rush_heading(Translation2d toward_point) {
    Pose2d current_pos = odom.get_position();

    Rotation2d angle_to_rushArm(from_degrees(current_pos.rotation().degrees() - 90));
    Translation2d rush_arm_point(
      current_pos.x() + (7 * cos(angle_to_rushArm.radians())), current_pos.y() + (7 * sin(angle_to_rushArm.radians()))
    );

    double rush_dy = (toward_point.y() - rush_arm_point.y());
    double rush_dx = (toward_point.x() - rush_arm_point.x());
    double rush_heading = atan2(rush_dy, rush_dx);

    printf(
      "rush arm point: (%f, %f), angle to rush arm: %f\n", rush_arm_point.x(), rush_arm_point.y(),
      angle_to_rushArm.degrees()
    );
    printf("rush arm delta: (%f, %f), rush heading: %f\n", rush_dy, rush_dx, rad2deg(rush_heading));
    return rad2deg(rush_heading);
}

AutoCommand *ClamperSys::ClampCmd(ClamperState state) {
    return new FunctionCommand([this, state]() {
        doAutoClamping = false;
        clamper_state = state;
        return true;
    });
}

AutoCommand *ClamperSys::RushCmd(RushState state) {
    return new FunctionCommand([this, state]() {
        doAutoClamping = false;
        rush_arm_state = state;
        return true;
    });
}

AutoCommand *ClamperSys::AutoClampCmd(bool do_auto_clamping) {
    return new FunctionCommand([this, do_auto_clamping]() {
        doAutoClamping = do_auto_clamping;
        return true;
    });
}

int ClamperSys::thread_fn(void *ptr) {
    ClamperSys &self = *(ClamperSys *)ptr;
    while (true) {
        if (self.clamper_state == ClamperState::CLAMPED) {
            goal_grabber_sol.set(true);
        } else if (self.clamper_state == ClamperState::UNCLAMPED) {
            goal_grabber_sol.set(false);
        }
        if (self.rush_arm_state == RushState::OUT) {
            goal_rush_sol.set(true);
        } else if (self.rush_arm_state == RushState::IN) {
            goal_rush_sol.set(false);
        }
        // if (self.doAutoClamping) {
        //     self.auto_clamp();
        // }
        vexDelay(20);
    }
    return 0;
}