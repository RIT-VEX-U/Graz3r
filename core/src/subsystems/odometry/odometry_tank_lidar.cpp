#include "core/subsystems/odometry/odometry_tank_lidar.h"

#include "core/utils/math/eigen_interface.h"
#include "core/utils/math/estimator/unscented_kalman_filter.h"
#include "core/utils/math/geometry/pose2d.h"
#include "core/utils/math/geometry/rotation2d.h"
#include <functional>
#include "robot-config.h"
#include "vex.h"

OdometryTankLidar::OdometryTankLidar(
  const double &gear_ratio, const double &circumference, const Pose2d &initial_pose, const EVec<3> &P, const EVec<3> &Q,
  const EVec<2> &R_lidar, vex::inertial &imu, vex::motor_group &left_side, vex::motor_group &right_side, const uint32_t &port, const uint32_t &baudrate, const Transform2d &lidar_offset, const double &Kvl, const double &Kal, const double &Kva, const double &Kaa
)
    : observer_(
        f, h_lidar, RK2_with_input<3, 2>, Q, R_lidar, mean_func_X, mean_func_Y, residual_func_X, residual_func_Y,
        add_func_X
      ),
      gear_ratio_(gear_ratio), circumference_(circumference), imu_(imu), left_side_(left_side), right_side_(right_side), port_(port), baudrate_(baudrate) {
        this->lidar_offset_ = lidar_offset;
        this->Kvl_ = Kvl;
        this->Kal_ = Kal;
        this->Kva_ = Kva;
        this->Kaa_ = Kaa;
        // auto odom_func = [](void *ptr) {
        //     OdometryTankLidar *self = (OdometryTankLidar *)ptr;
        //     return self->odom_thread(ptr);
        // };
        // auto lidar_func = [](void *ptr) {
        //     OdometryTankLidar *self = (OdometryTankLidar *)ptr;
        //     return self->lidar_thread(ptr);
        // };
        // vex::task *handle = new vex::task{odom_func, (void *)this};
        // printf("inited\n");
        observer_.set_xhat(initial_pose.vector());
        lidar_handle = new vex::task{lidar_thread, (void *)this};  
        
      }

void OdometryTankLidar::set_position(const Pose2d &newpos) {
    observer_.set_xhat(EVec<3>{newpos.x(), newpos.y(), newpos.rotation().wrapped_radians_180()});
}
Pose2d OdometryTankLidar::get_position() const {
    return Pose2d{observer_.xhat(0), observer_.xhat(1), from_radians(observer_.xhat(2))};
}

EVec<3> OdometryTankLidar::h_odom(const EVec<3> &xhat, const EVec<2> &u) {
    // const double h = xhat(2);
    // const double l = xhat(3);
    // const double r = xhat(4);
    return EVec<3>{0, 0, 0};
}

// the walls are 0.25in from the edge of the field, we're measuring inside the walls, not the field
EVec<2> OdometryTankLidar::h_lidar(const EVec<3> &xhat, const EVec<2> &u) {
    const double field_width = 140.875;

    Pose2d robot_pose = Pose2d{xhat(0), xhat(1), from_radians(xhat(2))};
    EVec<3> lidar_xhat = (robot_pose + lidar.lidar_offset_).vector();
    double beam_theta = wrap_radians_180(deg2rad(-u(1))) + lidar_xhat(2);

    double c = std::cos(beam_theta);
    double s = std::sin(beam_theta);

    double d_left = (c < 0) ? ((lidar_xhat(0) - 0.25) / -c) : std::numeric_limits<double>::infinity();
    double d_right = (c > 0) ? ((field_width - (lidar_xhat(0) + 0.25)) / c) : std::numeric_limits<double>::infinity();
    double d_bottom = (s < 0) ? ((lidar_xhat(1) - 0.25)  / -s) : std::numeric_limits<double>::infinity();
    double d_top = (s > 0) ? ((field_width - (lidar_xhat(1) + 0.25)) / s) : std::numeric_limits<double>::infinity();

    double min_distance = std::min({d_left, d_right, d_bottom, d_top});

    return EVec<2>{min_distance, u(1)};
}

EVec<3> OdometryTankLidar::f(const EVec<3> &xhat, const EVec<2> &u) {
    const double x = xhat(0);
    const double y = xhat(1);
    const double h = xhat(2);



    return EVec<3>{0, 0, 0};
}

int OdometryTankLidar::odom_thread(void *ptr) {
    uint32_t count = 0;
    auto thread_start = std::chrono::steady_clock::now();
    prev_left = (left_side_.position(vex::deg) / 360.0) * circumference_ / gear_ratio_;
    prev_right = (right_side_.position(vex::deg) / 360.0) * circumference_ / gear_ratio_;
    while (running_) {
        mutex.lock();
        double left_in = (left_side_.position(vex::deg) / 360.0) * circumference_ / gear_ratio_;
        double right_in = (right_side_.position(vex::deg) / 360.0) * circumference_ / gear_ratio_;
        double theta = wrap_radians_180(deg2rad(imu_.rotation(vex::deg)));
        double omega = deg2rad(imu_.gyroRate(vex::axisType::zaxis, vex::velocityUnits::dps));
        odom_update(left_in, right_in, from_radians(theta), omega);
        prev_left = left_in;
        prev_right = right_in;
        mutex.unlock();

        vex::this_thread::sleep_until(thread_start + std::chrono::milliseconds(10 * ++count));
    }
    return 0;
}
int OdometryTankLidar::lidar_thread(void *ptr) {
    OdometryTankLidar &obj = *((OdometryTankLidar *)ptr);
    vexGenericSerialEnable(obj.port_, 0);
    vexGenericSerialBaudrate(obj.port_, obj.baudrate_);
    vexDelay(100);
    vexGenericSerialFlush(obj.port_);
    constexpr size_t BUFFER_SIZE = 4;
    uint8_t buf[BUFFER_SIZE];

    while (obj.running_) {
        if (receive_packet(obj.port_, buf, BUFFER_SIZE) != 4) {
            continue;
        };

        uint16_t angle_q6;
        uint16_t dist;

        memcpy(&angle_q6, buf, sizeof(angle_q6));
        memcpy(&dist, buf + sizeof(angle_q6), sizeof(dist));

        double angle = fmod((angle_q6 * 0.015625) + 351.5, 360);
        double distance = dist / 25.4; // to inches

        if (angle > 360 || angle < 0) {
            continue;
        }
        if (distance > 203 || distance < 0) {
            continue;
        }
        if (std::abs(h_lidar(obj.observer_.xhat(), EVec<2>{distance, angle})(0) - distance) > 10) {
            continue;
        }
        // printf("%f,%f\n", distance, angle);
        // std::cout << obj.observer_.xhat();

        obj.mutex.lock();
        obj.lidar_update(distance, angle);
        obj.mutex.unlock();
    }
    vex::this_thread::yield();

    return 0;
}
void OdometryTankLidar::kill_threads() {
    running_ = false;
}

// Happens every 10ms, when we get data
// u = [v, omega]
// we compute v based on 10 ms time interval, and hold it constant on all subsequent lidar updates
void OdometryTankLidar::odom_update(const double &left, const double &right, const Rotation2d &theta, double &omega) {
    double dleft = (left - prev_left);
    double dright = (right - prev_right);
    // kinematics then divide by 10ms timestep
    current_velocity = ((dleft + dright) / 2.0) / 0.01;
    current_angular_velocity = omega;
    // force angle to be what the gyro reads............ ugh
    observer_.set_xhat(2, theta.wrapped_radians_180());
    // this is DEFINITELY a bad idea but I think if I don't do this the pose will just not update for >1/2 of the lidar
    // rotation we can't just not update the pose for like 800ms
    observer_.predict(EVec<2>{current_velocity, current_angular_velocity}, 0.01);
    // this might be a bad idea who knows
    last_time = vexSystemHighResTimeGet();
}

// Happens immediately when we get data
// y = [distance, angle]
// the way this does this makes me sad
// we should latency compensate or use a buffer or some such... ugh
void OdometryTankLidar::lidar_update(const double &distance_in, const double &angle_deg_cw) {
    static uint64_t first_time = vexSystemHighResTimeGet();
    uint64_t now = vexSystemHighResTimeGet() - first_time;
    

    // predict using saved velocity and angular velocity
    observer_.predict(EVec<2>{0, 0}, (now - last_time) / 1000000.0);
    observer_.correct(EVec<2>{distance_in, angle_deg_cw}, EVec<2>{distance_in, angle_deg_cw});
    // observer_.set_xhat(2, 0);
    last_time = now;

    printf(
      "%llu, %f, %f, %f, %f, %f\n", now, observer_.xhat(0), observer_.xhat(1),
      rad2deg(observer_.xhat(2)), distance_in, angle_deg_cw
    );
}



    /**
     * COBS decode data from buffer.
     *
     * @param buffer Pointer to encoded input bytes.
     * @param length Number of bytes to decode.
     * @param data Pointer to decoded output data.
     *
     * @return Number of bytes successfully decoded.
     * @note Stops decoding if delimiter byte is found.
     */
    size_t cobsDecode(const uint8_t *buffer, size_t length, void *data) {
        assert(buffer && data);

        const uint8_t *byte = buffer;      // Encoded input byte pointer
        uint8_t *decode = (uint8_t *)data; // Decoded output byte pointer

        for (uint8_t code = 0xff, block = 0; byte < buffer + length; --block) {
            if (block) // Decode block byte
                *decode++ = *byte++;
            else {
                block = *byte++;             // Fetch the next block length
                if (block && (code != 0xff)) // Encoded zero, write it unless it's delimiter.
                    *decode++ = 0;
                code = block;
                if (!code) // Delimiter code found
                    break;
            }
        }
        return (size_t)(decode - (uint8_t *)data);
    }

    /**
     * Attempts to recieve an entire packet encoded with COBS, stops at delimiter or there's a buffer overflow.
     *
     * @param port the port number the serial is plugged into.
     * @param buffer pointer to a uint8_t[] where we put the data.
     * @param buffer_size length in bytes of the encoded buffer.
     * @return 0 success.
     */
    int receive_cobs_packet(uint32_t port, uint8_t *buffer, size_t buffer_size) {
        size_t index = 0;

        while (true) {
            // wait for a byte (we read byte by byte into our own buffer rather than grabbing the whole packet all at
            // once)
            int32_t avail = vexGenericSerialReceiveAvail(port);
            // printf("%ld\n", avail);
            if (avail > 0) {
                uint8_t character = vexGenericSerialReadChar(port);
                // printf("%X\n", character);

                // if delimiter
                if (character == 0x00) {
                    buffer[index++] = character;
                    return index; // return packet length
                }


                // store character in buffer
                if (index < buffer_size) {
                    buffer[index++] = character;
                } else {
                    // buffer overflow
                    printf("bufferoverflow\n");
                    return -1;
                }
            }
            if (vexGenericSerialReceiveAvail(port) == 0) {
                vex::this_thread::yield();
            }
        }
    }

    /**
     * Attempts to receive a packet given a length, this automatically decodes it.
     *
     * @param port the port number the serial is plugged into.
     * @param buffer pointer to a uint8_t[] where we put the data.
     * @param buffer_size length in bytes of the buffer, after being decoded.
     */
    int receive_packet(uint32_t port, uint8_t *buffer, size_t buffer_size) {
        uint8_t cobs_encoded[buffer_size + 2];
        receive_cobs_packet(port, cobs_encoded, buffer_size + 2);

        return cobsDecode(cobs_encoded, buffer_size + 2, buffer);
    }
