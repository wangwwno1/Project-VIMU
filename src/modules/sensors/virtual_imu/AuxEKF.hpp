//
// Created by Robert Wang on 2022/5/19.
//

#pragma once

#include <matrix/math.hpp>
#include <lib/mathlib/mathlib.h>
#include <modules/ekf2/EKF/common.h>
#include <modules/ekf2/EKF/RingBuffer.h>

#include "RateDownSampler.hpp"

using namespace matrix;

namespace estimator
{

struct AuxEKFParam {
    float gyro_noise{1.5e-2f};
    float process_noise{0.075f};
    int32_t imu_fuse_delay_us{200000};
    int32_t filter_update_interval_us{20000};  // ekf prediction period in microseconds - this should ideally be an integer multiple of the IMU time delta
};

class AuxEKF
{
public:
    static constexpr uint8_t _k_num_states{3};		///< number of EKF states
    static constexpr uint8_t MAX_SENSOR_COUNT = 4;

    AuxEKF() { reset(); };
    ~AuxEKF() {
        for (int i = 0; i < MAX_SENSOR_COUNT; ++i) {
            if (_imu_buffers[i] != nullptr) {
                delete _imu_buffers[i];
                _imu_buffers[i] = nullptr;
            }
        }
    };

    Vector3f getAngularAcceleration() const { return _angular_acceleration; }
    Vector3f getAngularRate() const { return _output_state.angular_rate - _gyro_bias; }
    Vector3f getVariances() const { return P.diag(); }
    AuxEKFParam *getParamHandle() { return &_param; }
    Vector3f getGyroBias() { return _gyro_bias; }
    void setGyroBias(const Vector3f &bias) { _gyro_bias = bias; }

    void setInertiaMatrix(const SquareMatrix3f &mat) {
        _inertia_matrix = mat;
        if (!mat.I(_inertia_matrix_inv)) {
            // Fallback to main inertia inverse
            _inertia_matrix_inv.setIdentity();
            for (int i = 0; i < _k_num_states; ++i) {
                _inertia_matrix_inv(i, i) = 1.f / math::max(mat(i, i), 1e-5f);
            }
        }
    }

    void setGyroData(const imuSample &imu, const uint8_t index) {
        if (!_filter_initialised) {
            _filter_initialised = allocateBuffer();
            if (!_filter_initialised) {
                return;
            }
        }

        if (_imu_buffers[index] == nullptr) {
            imuBuffer *inst = new imuBuffer{_param.filter_update_interval_us};
            if (inst && inst->buffer.allocate(_imu_buffer_length)) {
                _imu_buffers[index] = inst;
            } else {
                delete inst;
                PX4_ERR("AuxEKF - IMU buffer allocation failed");
                return;
            }
        }

        if (_imu_buffers[index]->sampler.update(imu)) {
            const RateSample down_sampled_rate = _imu_buffers[index]->sampler.getAverageRateAndTriggerReset();
            _imu_buffers[index]->buffer.push(down_sampled_rate);
        }
    };

    void integrateTorque(const Vector3f &torque, const Vector3f &ext_angular_accel,
                         const float dt, const hrt_abstime now) {
        if (!_filter_initialised) {
            _filter_initialised = allocateBuffer();
            if (!_filter_initialised) {
                return;
            }
        }

        const Vector3f rate = _output_state.angular_rate - _gyro_bias;
        const Vector3f main_inertia = _inertia_matrix.diag();
        _angular_acceleration = (torque - rate.cross(_inertia_matrix * rate)).edivide(main_inertia) + ext_angular_accel;

        _output_state.time_us = now;
        _output_state.angular_rate += _angular_acceleration * dt;

        _delta_rate += _angular_acceleration * dt;
        _down_sampled_time += dt;

        const float filter_update_period_s = _param.filter_update_interval_us * 1.e-6f;

        if (_down_sampled_time > filter_update_period_s - _down_sample_time_correction) {
            _output_buffer.push(_output_state);
            PredictState();
            MeasurementUpdate();

            _down_sample_time_correction += 0.01f * (_down_sampled_time - filter_update_period_s);
            _down_sample_time_correction = math::constrain(_down_sample_time_correction,
                                                           -0.5f * filter_update_period_s,
                                                           +0.5f * filter_update_period_s);

            const RateSample &output_state_delayed = _output_buffer.get_oldest();
            const float time_delay = fmaxf((_output_state.time_us - output_state_delayed.time_us) * 1e-6f, filter_update_period_s);
            const float rate_gain = filter_update_period_s / time_delay;
            const Vector3f rate_correction = rate_gain * (_state - output_state_delayed.angular_rate);
            applyCorrectionToOutputBuffer(rate_correction);

            _down_sampled_time = 0.f;
            _delta_rate.zero();
        }
    }

    void reset_state() {
        // Reset estimator states
        _state.zero();
        _gyro_bias.zero();
        _angular_acceleration.zero();
        _output_state.time_us = 0;
        _output_state.angular_rate.zero();

        // Reset angular acceleration integrator
        _delta_rate.zero();
        _rate_error_integ.zero();
        _down_sampled_time = 0.f;
        _down_sample_time_correction = 0.f;
    }

    void reset_buffer() {
        RateSample newest_output_sample = _output_buffer.get_newest();
        _output_buffer.pop_first_older_than(newest_output_sample.time_us, &newest_output_sample);
        for (uint8_t i = 0; i < MAX_SENSOR_COUNT; ++i) {
            reset_imu_buffer(i);
        }
    }

    bool reset_imu_buffer(uint8_t index) {
        if (_imu_buffers[index] != nullptr) {
            RateSample newest_imu_sample = _imu_buffers[index]->buffer.get_newest();
            _imu_buffers[index]->buffer.pop_first_older_than(newest_imu_sample.time_us, &newest_imu_sample);
            _imu_buffers[index]->sampler.getAverageRateAndTriggerReset();
            return true;
        } else {
            return false;
        }
    }

private:
    Vector3f _state{0.f, 0.f, 0.f};
    RateSample _output_state;
    RateSample _imu_sample_delayed;
    AuxEKFParam _param;
    bool _filter_initialised{false};

    Vector3f _gyro_bias{0.f, 0.f, 0.f};
    Vector3f _angular_acceleration{0.f, 0.f, 0.f};
    SquareMatrix3f P;
    SquareMatrix3f _inertia_matrix;
    SquareMatrix3f _inertia_matrix_inv;
    float _down_sampled_time{0.f};
    float _down_sample_time_correction{0.f};
    Vector3f _delta_rate{0.f, 0.f, 0.f};
    Vector3f _rate_error_integ{0.f, 0.f, 0.f};

    struct imuBuffer {
        imuBuffer(int32_t &target_dt_us) : sampler(target_dt_us) {}

        RateDownSampler sampler;
        RingBuffer<RateSample> buffer{12};
    };

    uint8_t _imu_buffer_length{12};
    uint8_t _output_buffer_length{12};
    RingBuffer<RateSample> _output_buffer{_output_buffer_length};
    imuBuffer* _imu_buffers[MAX_SENSOR_COUNT] {nullptr, nullptr, nullptr, nullptr};

    bool allocateBuffer() {
        const float imu_delay_s = _param.imu_fuse_delay_us * 1.e-6f;
        const float filter_update_period_s = _param.filter_update_interval_us * 1.e-6f;

        _output_buffer_length = ceilf(imu_delay_s / filter_update_period_s) + 1;
        _imu_buffer_length = ceilf(imu_delay_s / filter_update_period_s) + 1;
        if (!_output_buffer.allocate(_output_buffer_length)) {
            PX4_ERR("AuxEKF - Output buffer allocation failed");
            return false;
        }

        return true;
    }

    void reset() {
        reset_state();

        // Reset covariances and inertia matrix.
        P.zero();
        _inertia_matrix.setIdentity();
        _inertia_matrix_inv.setIdentity();

        reset_buffer();
    }

    void PredictState() {
        const Vector3f corrected_state = _state - _gyro_bias;
        const Vector3f main_inertia = _inertia_matrix.diag();
        const Vector3f inertia_term = main_inertia.cross(Vector3f{1.f, 1.f, 1.f}).edivide(main_inertia);
        // Iyy - Izz, Izz - Ixx, Ixx - Iyy

        // Jacobian derivative matrix of estimation vs. previous state.
        const float F_array[3][3] = {
                {0.f,                corrected_state(2), corrected_state(1)},
                {corrected_state(2), 0.f,                corrected_state(0)},
                {corrected_state(1), corrected_state(0), 0.f}
        };

        SquareMatrix3f F(F_array);
        for (uint8_t row = 0; row < _k_num_states; ++row) {
            const float fac = inertia_term(row);
            for (uint8_t column = 0; column < _k_num_states; ++column) {
                F(row, column) *= fac;
            }
        }

        _state += _delta_rate;
        const float rate_var = math::sq(_param.process_noise);
        for (uint8_t row = 0; row < _k_num_states; ++row) {
            for (uint8_t column = 0; column < _k_num_states; ++column) {
                // Omit non-diagonal process noise
                float P_rate = F(row, column) * P(row, column) + P(row, column) * F(column, row);
                if (row == column) {
                    P_rate += rate_var;
                }
                P(row, column) += P_rate * _down_sampled_time;

            }
        }

        // Ensure P matrix is symmetric
        for (int row = 0; row < _k_num_states; ++row) {
            for (int column = 0; column < row; ++column) {
                P(row, column) = P(column, row);
            }
        }

    };

    void MeasurementUpdate() {
        const RateSample output_sample_delayed = _output_buffer.get_oldest();
        // Looking from the oldest measurement data to the newest
        for (uint8_t i = 0; i < MAX_SENSOR_COUNT; ++i) {
            if (_imu_buffers[i] == nullptr) {
                continue;
            }

            RateSample imu_sample_delayed{};
            if (_imu_buffers[i]->buffer.pop_first_older_than(output_sample_delayed.time_us, &imu_sample_delayed) &&
                output_sample_delayed.time_us < imu_sample_delayed.time_us + _param.imu_fuse_delay_us) {
                for (uint8_t dim = 0; dim < _k_num_states; ++dim) {
                    const float innov = _state(dim) - imu_sample_delayed.angular_rate(dim);
                    const float innov_var = P(dim, dim) + math::sq(_param.gyro_noise);
                    fuse(innov, innov_var, dim);
                }
            }
        }
    }

    void fuse(const float innov, const float innov_var, const uint8_t state_index) {
        Vector3f Kfusion;
        for (uint8_t row = 0; row < _k_num_states; ++row) {
            // K = PHS' = PH(P + Measurement Noise)'
            Kfusion(row) = P(row, state_index) / innov_var;
        }

        SquareMatrix3f KHP;
        for (int row = 0; row < _k_num_states; ++row) {
            for (int column = 0; column < _k_num_states; ++column) {
                // We omit the H matrix, the 1st order derivative between estimation and current state given all previous observations
                // because we assume the rate estimation should converge to the actual state, thus obtain a derivative of 1.
                KHP(row, column) = Kfusion(row) * P(state_index, column);
            }
        }

        bool healthy = true;
        for (int i = 0; i < _k_num_states; ++i) {
            if (P(i, i) < KHP(i, i)) {
                // zero rows and columns
                P.uncorrelateCovarianceSetVariance<1>(i, math::sq(_param.process_noise));

                healthy = false;
            }
        }

        if (healthy) {
            // P(k) = P(k-1) - KHP(k-1)
            P -= KHP;
            // x(k|k) = x(k|k-1) - K * (x(k|k-1) - z(k))
            _state -= Kfusion * innov;
        }

        for (int i = 0; i < _k_num_states; ++i) {
            P(i, i) = math::constrain(P(i, i), 0.f, 1.f);
        }

    }

    void applyCorrectionToOutputBuffer(const Vector3f &rate_correction) {
        // loop through the output filter state history and apply the corrections to the angular rate states
        for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
            // apply constant rate correction
            _output_buffer[index].angular_rate += rate_correction;
        }

        _output_state = _output_buffer.get_newest();
    }

};

}