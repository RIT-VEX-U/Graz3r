#include "competition/opcontrol.h"
#include "core/subsystems/odometry/odometry_nwheel.h"
#include "core/subsystems/odometry/odometry_tank.h"
#include "core.h"
#include "robot-config.h"
#include "vex.h"
#include <iostream>

#include "core/utils/controls/state_space/linear_plant_inversion_feedforward.h"
#include "core/utils/controls/state_space/linear_quadratic_regulator.h"
#include "core/utils/math/estimator/kalman_filter.h"
#include "core/utils/math/estimator/unscented_kalman_filter.h"
#include "core/utils/math/systems/linear_system.h"

#include "core/device/cobs_device.h"

// vex::motor motorf(vex::PORT13);

COBSSerialDevice lidar_recv{vex::PORT13, 921600};

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
        // wait for a byte (we read byte by byte into our own buffer rather than grabbing the whole packet all at once)
        // printf("%d\n", vexGenericSerialReceiveAvail(port)); 
        if (vexGenericSerialReceiveAvail(port) > 0) {
            uint8_t character = vexGenericSerialReadChar(port);

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
        vex::this_thread::yield();
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

double wrap_radians(const double &angle) {
    double x = fmod(angle + M_PI, M_TWOPI);
    if (x < 0) {
        x += M_TWOPI;
    }
    return x - M_PI;
}

auto f = [](const EVec<3> &xhat, const EVec<2> &u) -> EVec<3> { return EVec<3>{0, 0, 0}; };

// u(1) is the beam angle in degrees as measured by the lidar for this update
auto h = [](const EVec<3> &xhat, const EVec<2> &u) -> EVec<2> {
    const double field_width = 140.875;

    double beam_theta = wrap_radians(deg2rad(-u(1)) + xhat(2));

    double c = std::cos(beam_theta);
    double s = std::sin(beam_theta);

    double d_left = (c < 0) ? (xhat(0) / -c) : std::numeric_limits<double>::infinity();
    double d_right = (c > 0) ? ((field_width - xhat(0)) / c) : std::numeric_limits<double>::infinity();
    double d_bottom = (s < 0) ? (xhat(1) / -s) : std::numeric_limits<double>::infinity();
    double d_top = (s > 0) ? ((field_width - xhat(1)) / s) : std::numeric_limits<double>::infinity();

    double min_distance = std::min({d_left, d_right, d_bottom, d_top});

    return EVec<2>{min_distance, u(1)};
};

auto mean_func_X = [](const EMat<3, 5> &sigmas, const EVec<5> &Wm) -> EVec<3> {
    EVec<3> x = EVec<3>::Zero();

    double c = 0;
    double s = 0;

    for (int i = 0; i < 5; i++) {
        x(0) += sigmas(0, i) * Wm(i);
        x(1) += sigmas(1, i) * Wm(i);

        c += std::cos(sigmas(2, i)) * Wm(i);
        s += std::sin(sigmas(2, i)) * Wm(i);
    }
    x(2) = std::atan2(s, c);

    return x;
};

auto mean_func_Y = [](const EMat<2, 5> &sigmas, const EVec<5> &Wm) -> EVec<2> {
    EVec<2> y = EVec<2>::Zero();

    double c = 0;
    double s = 0;

    for (int i = 0; i < 5; i++) {
        y(0) += sigmas(0, i) * Wm(i);
        c += std::cos(sigmas(1, i)) * Wm(i);
        s += std::sin(sigmas(1, i)) * Wm(i);
    }
    y(1) = std::atan2(s, c);
    return y;
};

auto residual_func_X = [](const EVec<3> &a, const EVec<3> &b) -> EVec<3> {
    return {a(0) - b(0), a(1) - b(1), wrap_radians(a(2) - b(2))};
};

auto add_func_X = [](const EVec<3> &a, const EVec<3> &b) -> EVec<3> {
    return {a(0) + b(0), a(1) + b(1), wrap_radians(a(2) + b(2))};
};

auto residual_func_Y = [](const EVec<2> &a, const EVec<2> &b) -> EVec<2> {
    return {a(0) - b(0), wrap_radians(a(1) - b(1))};
};

EVec<3> Q{1, 1, 0.1};
EVec<2> R{1, 1}; // degree, 2mm to in

UKF<3, 2, 2>
  ukf(f, h, euler_with_input<3, 2>, Q, R, mean_func_X, mean_func_Y, residual_func_X, residual_func_Y, add_func_X);

void lidar_update(float distance, float angle) {
    static uint64_t first_time = vexSystemHighResTimeGet();
    // printf("%f, %f\n", distance, angle);
    // ukf.predict(EVec<2>{0, 0}, 0.0005);
    // ukf.correct(EVec<2>{distance, angle}, EVec<2>{distance, angle});
    // ukf.set_xhat(EVec<3>{ukf.xhat(0), ukf.xhat(1), 0});
    printf(
      "%llu, %f, %f, %f, %f, %f\n", vexSystemHighResTimeGet() - first_time, ukf.xhat(0), ukf.xhat(1),
      rad2deg(ukf.xhat(2)), distance, angle
    );
}

/**
 * Main entrypoint for the driver control period
 */
void opcontrol() {

    EVec<3> P0{0.1, 0.1, 0.00001};
    ukf.set_xhat(EVec<3>{24, 24, 0});
    ukf.set_P(P0.asDiagonal());

    vex::task rec_task{[]() -> int {
        constexpr size_t BUFFER_SIZE = 6;
        uint8_t buf[BUFFER_SIZE];

        while (true) {
            // int num_recd = lidar_recv.receive_cobs_packet_blocking(buf, BUFFER_SIZE, 500000); // 500 ms timeout

            receive_packet(vex::PORT11, buf, 6);

            uint16_t angle_q6;
            uint16_t dist;

            memcpy(&angle_q6, buf, sizeof(angle_q6));
            memcpy(&dist, buf + sizeof(angle_q6), sizeof(dist));

            double angle = angle_q6 * 0.015625;
            if (angle > 360 || angle < 0) {
                continue;
            }
            double distance = dist / 25.4; // to inches
            if (distance > 199 || distance < 0) {
                continue;
            }
            if (abs(h(ukf.xhat(), EVec<2>{distance, angle})(0) - distance) > 20) {
                continue;
            }


            lidar_update(distance, angle);
        }
        vex::this_thread::yield();

        return 0;
    }};

    while (true) {
        printf("opcontroolling\n");
        // printf("%f, %f, %f\n", ukf.xhat(0), ukf.xhat(1), rad2deg(ukf.xhat(2)));
        drive_sys.drive_tank(0.2, -0.2);
        vexDelay(100);
    }
}
