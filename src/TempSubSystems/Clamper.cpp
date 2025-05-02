#include "TempSubSystems/Clamper.h"
#include "core/utils/math_util.h"
#include "robot-config.h"

ClamperSys::ClamperSys() { task = vex::task(thread_fn, this); }

void ClamperSys::goalclamp() { clamper_state = ClamperState::CLAMPED; };
void ClamperSys::goalunclamp() { clamper_state = ClamperState::UNCLAMPED; };

void ClamperSys::rush_out() { rush_arm_state = RushState::OUT; };
void ClamperSys::rush_in() { rush_arm_state = RushState::IN; };

void ClamperSys::toggle_clamp() {
    doAutoClamping = false;
    if (is_clamped()) {
        goalunclamp();
    } else {
        goalclamp();
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
    if (printClampingDist) {
        printf("distance: %f\n", clamper_sensor.objectDistance(vex::distanceUnits::mm));
    }

    if (clamper_sensor.objectDistance(vex::distanceUnits::mm) <= 50) {
        clamper_state = ClamperState::CLAMPED;
    } else {
        clamper_state = ClamperState::UNCLAMPED;
    }
}

void ClamperSys::auto_clamp_on() { doAutoClamping = true; }

void ClamperSys::auto_clamp_off() { doAutoClamping = false; }

void ClamperSys::print_clamping_dist(bool true_to_print) { printClampingDist = true_to_print; }

bool ClamperSys::is_auto_clamping() { return doAutoClamping; }

// returns true if the piston is clamped down
bool ClamperSys::is_clamped() { return goal_grabber_sol.value(); };
bool ClamperSys::is_rush_out() { return goal_rush_sol.value(); };

double ClamperSys::rush_heading(Translation2d toward_point) {
    Pose2d current_pos = odom.get_position();

    Rotation2d angle_to_rushArm(from_degrees(current_pos.rotation().degrees() - 90));
    Translation2d rush_arm_offset(7 * cos(angle_to_rushArm.radians()), 7 * sin(angle_to_rushArm.radians()));
    Translation2d rush_arm_point = current_pos.translation() + rush_arm_offset;
    Translation2d rush_delta(toward_point.x() - rush_arm_point.x(), toward_point.y() - rush_arm_point.y());
    Rotation2d rush_heading = rush_delta.theta();
    printf(
      "rush arm point: (%f, %f), angle to rush arm: %f\n", rush_arm_point.x(), rush_arm_point.y(),
      angle_to_rushArm.degrees()
    );
    printf(
      "rush arm delta: (%f, %f), rush heading: %f\n", rush_delta.x(), rush_delta.y(), rush_heading.wrapped_degrees_360()
    );
    return rush_heading.wrapped_degrees_360();
}

double ClamperSys::rush_heading(Translation2d toward_point, Pose2d from_pose) {

    Rotation2d angle_to_rushArm(from_degrees(from_pose.rotation().degrees() - 90));
    Translation2d rush_arm_offset(7 * cos(angle_to_rushArm.radians()), 7 * sin(angle_to_rushArm.radians()));
    Translation2d rush_arm_point = from_pose.translation() + rush_arm_offset;
    Translation2d rush_delta(toward_point.x() - rush_arm_point.x(), toward_point.y() - rush_arm_point.y());
    Rotation2d rush_heading = rush_delta.theta();
    printf(
      "rush arm point: (%f, %f), angle to rush arm: %f\n", rush_arm_point.x(), rush_arm_point.y(),
      angle_to_rushArm.wrapped_degrees_360()
    );
    printf(
      "rush arm delta: (%f, %f), rush heading: %f\n", rush_delta.x(), rush_delta.y(), rush_heading.wrapped_degrees_360()
    );
    return rush_heading.wrapped_degrees_360();
}

std::vector<Translation2d> ClamperSys::last_rush_points(
  Translation2d second_last_point, Translation2d final_rush_point, Translation2d toward_point, double bank_radius
) {
    Translation2d delta_last_points = final_rush_point - second_last_point;
    double final_heading = rush_heading(toward_point, Pose2d(final_rush_point, delta_last_points.theta()));
    Translation2d inserted_point_offset(bank_radius, from_degrees(final_heading));
    Translation2d inserted_point = final_rush_point - inserted_point_offset;

    return {second_last_point, inserted_point, final_rush_point};
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

AutoCommand *ClamperSys::PrintClampingDistCmd(bool true_to_print) {
    return new FunctionCommand([this, true_to_print]() {
        print_clamping_dist(true_to_print);
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
        if (self.doAutoClamping) {
            self.auto_clamp();
        }
        vexDelay(20);
    }
    return 0;
}