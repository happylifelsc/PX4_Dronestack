/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file baro_height_control.cpp
 * Control functions for ekf barometric height fusion
 */

#include "ekf.h"

void Ekf::controlBaroHeightFusion()
{
	static constexpr const char *HGT_SRC_NAME = "baro";

	auto &aid_src = _aid_src_baro_hgt;
	HeightBiasEstimator &bias_est = _baro_b_est;

	bias_est.predict(_dt_ekf_avg);

	baroSample baro_sample;

	if (_baro_buffer && _baro_buffer->pop_first_older_than(_time_delayed_us, &baro_sample)) {

		const float measurement = compensateBaroForDynamicPressure(baro_sample.hgt);
		const float measurement_var = sq(_params.baro_noise);

		const float innov_gate = fmaxf(_params.baro_innov_gate, 1.f);

		const bool measurement_valid = PX4_ISFINITE(measurement) && PX4_ISFINITE(measurement_var);

		if (measurement_valid) {
			if ((_baro_counter == 0) || baro_sample.reset) {
				_baro_lpf.reset(measurement);
				_baro_counter = 1;

			} else {
				_baro_lpf.update(measurement);
				_baro_counter++;
			}

			if (_baro_counter <= _obs_buffer_length) {
				// Initialize the pressure offset (included in the baro bias)
				bias_est.setBias(_state.pos(2) + _baro_lpf.getState());
			}
		}

		// vertical position innovation - baro measurement has opposite sign to earth z axis
		updateVerticalPositionAidSrcStatus(baro_sample.time_us,
						   -(measurement - bias_est.getBias()),
						   measurement_var + bias_est.getBiasVar(),
						   innov_gate,
						   aid_src);

		// Compensate for positive static pressure transients (negative vertical position innovations)
		// caused by rotor wash ground interaction by applying a temporary deadzone to baro innovations.
		if (_control_status.flags.gnd_effect && (_params.gnd_effect_deadzone > 0.f)) {

			const float deadzone_start = 0.0f;
			const float deadzone_end = deadzone_start + _params.gnd_effect_deadzone;

			if (aid_src.innovation < -deadzone_start) {
				if (aid_src.innovation <= -deadzone_end) {
					aid_src.innovation += deadzone_end;

				} else {
					aid_src.innovation = -deadzone_start;
				}
			}
		}

		// update the bias estimator before updating the main filter but after
		// using its current state to compute the vertical position innovation
		if (measurement_valid) {
			bias_est.setMaxStateNoise(sqrtf(measurement_var));
			bias_est.setProcessNoiseSpectralDensity(_params.baro_bias_nsd);
			bias_est.fuseBias(measurement - (-_state.pos(2)), measurement_var + P(9, 9));
		}

		// determine if we should use height aiding
		const bool continuing_conditions_passing = (_params.baro_ctrl == 1)
				&& measurement_valid
				&& (_baro_counter > _obs_buffer_length)
				&& !_baro_hgt_faulty;

		const bool starting_conditions_passing = continuing_conditions_passing
				&& isNewestSampleRecent(_time_last_baro_buffer_push, 2 * BARO_MAX_INTERVAL);

		if (_control_status.flags.baro_hgt) {
			aid_src.fusion_enabled = true;

			if (continuing_conditions_passing) {

				fuseVerticalPosition(aid_src);

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.hgt_fusion_timeout_max);

				if (isHeightResetRequired()) {
					// All height sources are failing
					ECL_WARN("%s height fusion reset required, all height sources failing", HGT_SRC_NAME);

					_information_events.flags.reset_hgt_to_baro = true;
					resetVerticalPositionTo(-(_baro_lpf.getState() - bias_est.getBias()), measurement_var);
					bias_est.setBias(_state.pos(2) + _baro_lpf.getState());

					// reset vertical velocity
					resetVerticalVelocityToZero();

					aid_src.time_last_fuse = _time_delayed_us;

				} else if (is_fusion_failing) {
					// Some other height source is still working
					ECL_WARN("stopping %s height fusion, fusion failing", HGT_SRC_NAME);
					stopBaroHgtFusion();
					_baro_hgt_faulty = true;
				}

			} else {
				ECL_WARN("stopping %s height fusion, continuing conditions failing", HGT_SRC_NAME);
				stopBaroHgtFusion();
			}

		} else {
			if (starting_conditions_passing) {
				if (_params.height_sensor_ref == HeightSensor::BARO) {
					ECL_INFO("starting %s height fusion, resetting height", HGT_SRC_NAME);
					_height_sensor_ref = HeightSensor::BARO;

					_information_events.flags.reset_hgt_to_baro = true;
					resetVerticalPositionTo(-(_baro_lpf.getState() - bias_est.getBias()), measurement_var);
					bias_est.setBias(_state.pos(2) + _baro_lpf.getState());

				} else {
					ECL_INFO("starting %s height fusion", HGT_SRC_NAME);
					bias_est.setBias(_state.pos(2) + _baro_lpf.getState());
				}

				aid_src.time_last_fuse = _time_delayed_us;
				bias_est.setFusionActive();
				_control_status.flags.baro_hgt = true;
			}
		}

	} else if (_control_status.flags.baro_hgt
		   && !isNewestSampleRecent(_time_last_baro_buffer_push, 2 * BARO_MAX_INTERVAL)) {
		// No data anymore. Stop until it comes back.
		ECL_WARN("stopping %s height fusion, no data", HGT_SRC_NAME);
		stopBaroHgtFusion();
	}
}

void Ekf::stopBaroHgtFusion()
{
	if (_control_status.flags.baro_hgt) {

		if (_height_sensor_ref == HeightSensor::BARO) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}

		_baro_b_est.setFusionInactive();
		resetEstimatorAidStatus(_aid_src_baro_hgt);

		_control_status.flags.baro_hgt = false;
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file bias_estimator.cpp
 *
 * @author Mathieu Bresciani 	<mathieu@auterion.com>
 */

#include "bias_estimator.hpp"

void BiasEstimator::predict(const float dt)
{
	// State is constant
	// Predict state covariance only
	float delta_state_var = _process_psd * dt;

	if (isOffsetDetected()) {
		// A bias in the state has been detected by the innovation sequence check
		// Boost the process noise until the offset is removed
		delta_state_var *= _process_var_boost_gain;
	}

	_state_var += delta_state_var;
	constrainStateVar();

	if (dt > FLT_EPSILON && fabsf(_dt - dt) > 0.001f) {
		_signed_innov_test_ratio_lpf.setParameters(dt, _innov_sequence_monitnoring_time_constant);
		_dt = dt;
	}

	_status.bias_var = _state_var;
}

void BiasEstimator::constrainStateVar()
{
	_state_var = math::constrain(_state_var, 1e-8f, _state_var_max);
}

void BiasEstimator::fuseBias(const float measurement, const float measurement_var)
{
	const float innov_var = _state_var + math::max(sq(0.01f), measurement_var);
	const float innov = measurement - _state;
	const float K = _state_var / innov_var;
	const float innov_test_ratio = computeInnovTestRatio(innov, innov_var);

	if (isTestRatioPassing(innov_test_ratio)) {
		updateState(K, innov);
		updateStateCovariance(K);

	}

	updateOffsetDetection(innov, innov_test_ratio);

	_status = packStatus(innov, innov_var, innov_test_ratio);
}

inline float BiasEstimator::computeInnovTestRatio(const float innov, const float innov_var) const
{
	return innov * innov / (_gate_size * _gate_size * innov_var);
}

inline bool BiasEstimator::isTestRatioPassing(const float innov_test_ratio) const
{
	return innov_test_ratio < 1.f;
}

inline void BiasEstimator::updateState(const float K, const float innov)
{
	_state = _state + K * innov;
}

inline void BiasEstimator::updateStateCovariance(const float K)
{
	_state_var -= K * _state_var;
	constrainStateVar();
}

inline void BiasEstimator::updateOffsetDetection(const float innov, const float innov_test_ratio)
{
	const float signed_innov_test_ratio = matrix::sign(innov) * innov_test_ratio;
	_signed_innov_test_ratio_lpf.update(math::constrain(signed_innov_test_ratio, -1.f, 1.f));

	if (innov > 0.f) {
		_time_since_last_positive_innov = 0.f;
		_time_since_last_negative_innov += _dt;

	} else {
		_time_since_last_negative_innov = 0.f;
		_time_since_last_positive_innov += _dt;
	}
}

inline bool BiasEstimator::isOffsetDetected() const
{
	// There is an offset in the estimate if the average of innovation is statistically too large
	// or if the sign of the innovation is constantly the same
	return fabsf(_signed_innov_test_ratio_lpf.getState()) > 0.2f
	       || (_time_since_last_positive_innov > _innov_sequence_monitnoring_time_constant)
	       || (_time_since_last_negative_innov > _innov_sequence_monitnoring_time_constant);
}

inline BiasEstimator::status BiasEstimator::packStatus(const float innov, const float innov_var,
		const float innov_test_ratio) const
{
	// Send back status for logging
	status ret{};
	ret.bias = _state;
	ret.bias_var = _state_var;
	ret.innov = innov;
	ret.innov_var = innov_var;
	ret.innov_test_ratio = innov_test_ratio;

	return ret;
}
/****************************************************************************
 *
 *   Copyright (c) 2015-2020 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file control.cpp
 * Control functions for ekf attitude and position estimator.
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */


#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::controlFusionModes(const imuSample &imu_delayed)
{
	// Store the status to enable change detection
	_control_status_prev.value = _control_status.value;
	_state_reset_count_prev = _state_reset_status.reset_count;

	if (_system_flag_buffer) {
		systemFlagUpdate system_flags_delayed;

		if (_system_flag_buffer->pop_first_older_than(imu_delayed.time_us, &system_flags_delayed)) {

			set_vehicle_at_rest(system_flags_delayed.at_rest);
			set_in_air_status(system_flags_delayed.in_air);

			set_is_fixed_wing(system_flags_delayed.is_fixed_wing);

			if (system_flags_delayed.gnd_effect) {
				set_gnd_effect();
			}
		}
	}

	// monitor the tilt alignment
	if (!_control_status.flags.tilt_align) {
		// whilst we are aligning the tilt, monitor the variances
		const Vector3f angle_err_var_vec = calcRotVecVariances();

		// Once the tilt variances have reduced to equivalent of 3deg uncertainty
		// and declare the tilt alignment complete
		if ((angle_err_var_vec(0) + angle_err_var_vec(1)) < sq(math::radians(3.0f))) {
			_control_status.flags.tilt_align = true;

			// send alignment status message to the console
			const char *height_source = nullptr;

			if (_control_status.flags.baro_hgt) {
				height_source = "baro";

			} else if (_control_status.flags.ev_hgt) {
				height_source = "ev";

			} else if (_control_status.flags.gps_hgt) {
				height_source = "gps";

			} else if (_control_status.flags.rng_hgt) {
				height_source = "rng";

			} else {
				height_source = "unknown";

			}

			if (height_source) {
				ECL_INFO("%llu: EKF aligned, (%s hgt, IMU buf: %i, OBS buf: %i)",
					 (unsigned long long)imu_delayed.time_us, height_source, (int)_imu_buffer_length, (int)_obs_buffer_length);
			}
		}
	}

	// control use of observations for aiding
	controlMagFusion();

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	controlOpticalFlowFusion(imu_delayed);
#endif // CONFIG_EKF2_OPTICAL_FLOW

	controlGpsFusion(imu_delayed);

#if defined(CONFIG_EKF2_AIRSPEED)
	controlAirDataFusion(imu_delayed);
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	controlBetaFusion(imu_delayed);
#endif // CONFIG_EKF2_SIDESLIP

#if defined(CONFIG_EKF2_DRAG_FUSION)
	controlDragFusion();
#endif // CONFIG_EKF2_DRAG_FUSION

	controlHeightFusion(imu_delayed);
	controlGravityFusion(imu_delayed);

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// Additional data odometry data from an external estimator can be fused.
	controlExternalVisionFusion();
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_AUXVEL)
	// Additional horizontal velocity data from an auxiliary sensor can be fused
	controlAuxVelFusion();
#endif // CONFIG_EKF2_AUXVEL

	controlZeroInnovationHeadingUpdate();

	controlZeroVelocityUpdate();

	// Fake position measurement for constraining drift when no other velocity or position measurements
	controlFakePosFusion();
	controlFakeHgtFusion();

	// check if we are no longer fusing measurements that directly constrain velocity drift
	updateDeadReckoningStatus();
}
/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file covariance.cpp
 * Contains functions for initialising, predicting and updating the state
 * covariance matrix
 * equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Roman Bast <bastroman@gmail.com>
 *
 */

#include "ekf.h"
#include "python/ekf_derivation/generated/predict_covariance.h"
#include "utils.hpp"

#include <math.h>
#include <mathlib/mathlib.h>

// Sets initial values for the covariance matrix
// Do not call before quaternion states have been initialised
void Ekf::initialiseCovariance()
{
	P.zero();

	_delta_angle_bias_var_accum.setZero();
	_delta_vel_bias_var_accum.setZero();

	const float dt = _dt_ekf_avg;

	resetQuatCov();

	// velocity
	P(4,4) = sq(fmaxf(_params.gps_vel_noise, 0.01f));
	P(5,5) = P(4,4);
	P(6,6) = sq(1.5f) * P(4,4);

	// position
	P(7,7) = sq(fmaxf(_params.gps_pos_noise, 0.01f));
	P(8,8) = P(7,7);
	P(9,9) = sq(fmaxf(_params.baro_noise, 0.01f));

	if (_control_status.flags.gps_hgt) {
		P(9,9) = sq(fmaxf(1.5f * _params.gps_pos_noise, 0.01f));
	}

#if defined(CONFIG_EKF2_RANGE_FINDER)
	if (_control_status.flags.rng_hgt) {
		P(9,9) = sq(fmaxf(_params.range_noise, 0.01f));
	}
#endif // CONFIG_EKF2_RANGE_FINDER

	// gyro bias
	_prev_delta_ang_bias_var(0) = P(10,10) = sq(_params.switch_on_gyro_bias * dt);
	_prev_delta_ang_bias_var(1) = P(11,11) = P(10,10);
	_prev_delta_ang_bias_var(2) = P(12,12) = P(10,10);

	// accel bias
	_prev_dvel_bias_var(0) = P(13,13) = sq(_params.switch_on_accel_bias * dt);
	_prev_dvel_bias_var(1) = P(14,14) = P(13,13);
	_prev_dvel_bias_var(2) = P(15,15) = P(13,13);

	resetMagCov();

	// wind
	P(22,22) = sq(_params.initial_wind_uncertainty);
	P(23,23) = P(22,22);

}

void Ekf::predictCovariance(const imuSample &imu_delayed)
{
	// Use average update interval to reduce accumulated covariance prediction errors due to small single frame dt values
	const float dt = _dt_ekf_avg;
	const float dt_inv = 1.f / dt;

	// convert rate of change of rate gyro bias (rad/s**2) as specified by the parameter to an expected change in delta angle (rad) since the last update
	const float d_ang_bias_sig = dt * dt * math::constrain(_params.gyro_bias_p_noise, 0.0f, 1.0f);

	// convert rate of change of accelerometer bias (m/s**3) as specified by the parameter to an expected change in delta velocity (m/s) since the last update
	const float d_vel_bias_sig = dt * dt * math::constrain(_params.accel_bias_p_noise, 0.0f, 1.0f);

	// inhibit learning of imu accel bias if the manoeuvre levels are too high to protect against the effect of sensor nonlinearities or bad accel data is detected
	// xy accel bias learning is also disabled on ground as those states are poorly observable when perpendicular to the gravity vector
	const float alpha = math::constrain((dt / _params.acc_bias_learn_tc), 0.0f, 1.0f);
	const float beta = 1.0f - alpha;
	_ang_rate_magnitude_filt = fmaxf(dt_inv * imu_delayed.delta_ang.norm(), beta * _ang_rate_magnitude_filt);
	_accel_magnitude_filt = fmaxf(dt_inv * imu_delayed.delta_vel.norm(), beta * _accel_magnitude_filt);
	_accel_vec_filt = alpha * dt_inv * imu_delayed.delta_vel + beta * _accel_vec_filt;

	const bool is_manoeuvre_level_high = _ang_rate_magnitude_filt > _params.acc_bias_learn_gyr_lim
					     || _accel_magnitude_filt > _params.acc_bias_learn_acc_lim;

	// gyro bias inhibit
	const bool do_inhibit_all_gyro_axes = !(_params.imu_ctrl & static_cast<int32_t>(ImuCtrl::GyroBias));

	for (unsigned stateIndex = 10; stateIndex <= 12; stateIndex++) {
		const unsigned index = stateIndex - 10;

		bool is_bias_observable = true;

		// TODO: gyro bias conditions

		const bool do_inhibit_axis = do_inhibit_all_gyro_axes || !is_bias_observable;

		if (do_inhibit_axis) {
			// store the bias state variances to be reinstated later
			if (!_gyro_bias_inhibit[index]) {
				_prev_delta_ang_bias_var(index) = P(stateIndex, stateIndex);
				_gyro_bias_inhibit[index] = true;
			}

		} else {
			if (_gyro_bias_inhibit[index]) {
				// reinstate the bias state variances
				P(stateIndex, stateIndex) = _prev_delta_ang_bias_var(index);
				_gyro_bias_inhibit[index] = false;
			}
		}
	}

	// accel bias inhibit
	const bool do_inhibit_all_accel_axes = !(_params.imu_ctrl & static_cast<int32_t>(ImuCtrl::AccelBias))
					 || is_manoeuvre_level_high
					 || _fault_status.flags.bad_acc_vertical;

	for (unsigned stateIndex = 13; stateIndex <= 15; stateIndex++) {
		const unsigned index = stateIndex - 13;

		bool is_bias_observable = true;

		if (_control_status.flags.vehicle_at_rest) {
			is_bias_observable = true;

		} else if (_control_status.flags.fake_hgt) {
			is_bias_observable = false;

		} else if (_control_status.flags.fake_pos) {
			// when using fake position (but not fake height) only consider an accel bias observable if aligned with the gravity vector
			is_bias_observable = (fabsf(_R_to_earth(2, index)) > 0.966f); // cos 15 degrees ~= 0.966
		}

		const bool do_inhibit_axis = do_inhibit_all_accel_axes || imu_delayed.delta_vel_clipping[index] || !is_bias_observable;

		if (do_inhibit_axis) {
			// store the bias state variances to be reinstated later
			if (!_accel_bias_inhibit[index]) {
				_prev_dvel_bias_var(index) = P(stateIndex, stateIndex);
				_accel_bias_inhibit[index] = true;
			}

		} else {
			if (_accel_bias_inhibit[index]) {
				// reinstate the bias state variances
				P(stateIndex, stateIndex) = _prev_dvel_bias_var(index);
				_accel_bias_inhibit[index] = false;
			}
		}
	}

	// Don't continue to grow the earth field variances if they are becoming too large or we are not doing 3-axis fusion as this can make the covariance matrix badly conditioned
	float mag_I_sig;

	if (_control_status.flags.mag_3D && (P(16, 16) + P(17, 17) + P(18, 18)) < 0.1f) {
		mag_I_sig = dt * math::constrain(_params.mage_p_noise, 0.0f, 1.0f);

	} else {
		mag_I_sig = 0.0f;
	}

	// Don't continue to grow the body field variances if they is becoming too large or we are not doing 3-axis fusion as this can make the covariance matrix badly conditioned
	float mag_B_sig;

	if (_control_status.flags.mag_3D && (P(19, 19) + P(20, 20) + P(21, 21)) < 0.1f) {
		mag_B_sig = dt * math::constrain(_params.magb_p_noise, 0.0f, 1.0f);

	} else {
		mag_B_sig = 0.0f;
	}

	float wind_vel_nsd_scaled;

	// Calculate low pass filtered height rate
	float alpha_height_rate_lpf = 0.1f * dt; // 10 seconds time constant
	_height_rate_lpf = _height_rate_lpf * (1.0f - alpha_height_rate_lpf) + _state.vel(2) * alpha_height_rate_lpf;

	// Don't continue to grow wind velocity state variances if they are becoming too large or we are not using wind velocity states as this can make the covariance matrix badly conditioned
	if (_control_status.flags.wind && (P(22,22) + P(23,23)) < sq(_params.initial_wind_uncertainty)) {
		wind_vel_nsd_scaled = math::constrain(_params.wind_vel_nsd, 0.0f, 1.0f) * (1.0f + _params.wind_vel_nsd_scaler * fabsf(_height_rate_lpf));

	} else {
		wind_vel_nsd_scaled = 0.0f;
	}

	// compute noise variance for stationary processes
	Vector24f process_noise;

	// Construct the process noise variance diagonal for those states with a stationary process model
	// These are kinematic states and their error growth is controlled separately by the IMU noise variances

	// delta angle bias states
	process_noise.slice<3, 1>(10, 0) = sq(d_ang_bias_sig);
	// delta_velocity bias states
	process_noise.slice<3, 1>(13, 0) = sq(d_vel_bias_sig);
	// earth frame magnetic field states
	process_noise.slice<3, 1>(16, 0) = sq(mag_I_sig);
	// body frame magnetic field states
	process_noise.slice<3, 1>(19, 0) = sq(mag_B_sig);
	// wind velocity states
	process_noise.slice<2, 1>(22, 0) = sq(wind_vel_nsd_scaled) * dt;

	// assign IMU noise variances
	// inputs to the system are 3 delta angles and 3 delta velocities
	float gyro_noise = math::constrain(_params.gyro_noise, 0.0f, 1.0f);
	const float d_ang_var = sq(dt * gyro_noise);

	float accel_noise = math::constrain(_params.accel_noise, 0.0f, 1.0f);

	Vector3f d_vel_var;

	for (int i = 0; i <= 2; i++) {
		if (_fault_status.flags.bad_acc_vertical || imu_delayed.delta_vel_clipping[i]) {
			// Increase accelerometer process noise if bad accel data is detected
			d_vel_var(i) = sq(dt * BADACC_BIAS_PNOISE);

		} else {
			d_vel_var(i) = sq(dt * accel_noise);
		}
	}

	// predict the covariance
	SquareMatrix24f nextP;

	// calculate variances and upper diagonal covariances for quaternion, velocity, position and gyro bias states
	sym::PredictCovariance(getStateAtFusionHorizonAsVector(), P, imu_delayed.delta_vel, d_vel_var, imu_delayed.delta_ang, d_ang_var, dt, &nextP);

	// process noise contribution for delta angle states can be very small compared to
	// the variances, therefore use algorithm to minimise numerical error
	for (unsigned i = 10; i <= 12; i++) {
		const int index = i - 10;

		if (!_gyro_bias_inhibit[index]) {
			// add process noise that is not from the IMU
			// process noise contribution for delta velocity states can be very small compared to
			// the variances, therefore use algorithm to minimise numerical error
			nextP(i, i) = kahanSummation(nextP(i, i), process_noise(i), _delta_angle_bias_var_accum(index));

		} else {
			nextP.uncorrelateCovarianceSetVariance<1>(i, _prev_delta_ang_bias_var(index));
			_delta_angle_bias_var_accum(index) = 0.f;
		}
	}

	for (int i = 13; i <= 15; i++) {
		const int index = i - 13;

		if (!_accel_bias_inhibit[index]) {
			// add process noise that is not from the IMU
			// process noise contribution for delta velocity states can be very small compared to
			// the variances, therefore use algorithm to minimise numerical error
			nextP(i, i) = kahanSummation(nextP(i, i), process_noise(i), _delta_vel_bias_var_accum(index));

		} else {
			nextP.uncorrelateCovarianceSetVariance<1>(i, _prev_dvel_bias_var(index));
			_delta_vel_bias_var_accum(index) = 0.f;
		}
	}

	// add process noise that is not from the IMU
	for (unsigned i = 16; i <= 23; i++) {
		nextP(i, i) += process_noise(i);
	}

	// stop position covariance growth if our total position variance reaches 100m
	// this can happen if we lose gps for some time
	if ((P(7, 7) + P(8, 8)) > 1e4f) {
		for (uint8_t i = 7; i <= 8; i++) {
			for (uint8_t j = 0; j < _k_num_states; j++) {
				nextP(i, j) = P(i, j);
				nextP(j, i) = P(j, i);
			}
		}
	}

	// covariance matrix is symmetrical, so copy upper half to lower half
	for (unsigned row = 0; row <= 15; row++) {
		for (unsigned column = 0 ; column < row; column++) {
			P(row, column) = P(column, row) = nextP(column, row);
		}

		P(row, row) = nextP(row, row);
	}

	if (_control_status.flags.mag_3D) {
		for (unsigned row = 16; row <= 21; row++) {
			for (unsigned column = 0 ; column < row; column++) {
				P(row, column) = P(column, row) = nextP(column, row);
			}

			P(row, row) = nextP(row, row);
		}
	}

	if (_control_status.flags.wind) {
		for (unsigned row = 22; row <= 23; row++) {
			for (unsigned column = 0 ; column < row; column++) {
				P(row, column) = P(column, row) = nextP(column, row);
			}

			P(row, row) = nextP(row, row);
		}
	}

	// fix gross errors in the covariance matrix and ensure rows and
	// columns for un-used states are zero
	fixCovarianceErrors(false);

}

void Ekf::fixCovarianceErrors(bool force_symmetry)
{
	// NOTE: This limiting is a last resort and should not be relied on
	// TODO: Split covariance prediction into separate F*P*transpose(F) and Q contributions
	// and set corresponding entries in Q to zero when states exceed 50% of the limit
	// Covariance diagonal limits. Use same values for states which
	// belong to the same group (e.g. vel_x, vel_y, vel_z)
	float P_lim[8] = {};
	P_lim[0] = 1.0f;		// quaternion max var
	P_lim[1] = 1e6f;		// velocity max var
	P_lim[2] = 1e6f;		// position max var
	P_lim[3] = 1.0f;		// gyro bias max var
	P_lim[4] = 1.0f;		// delta velocity z bias max var
	P_lim[5] = 1.0f;		// earth mag field max var
	P_lim[6] = 1.0f;		// body mag field max var
	P_lim[7] = 1e6f;		// wind max var

	for (int i = 0; i <= 3; i++) {
		// quaternion states
		P(i, i) = math::constrain(P(i, i), 0.0f, P_lim[0]);
	}

	for (int i = 4; i <= 6; i++) {
		// NED velocity states
		P(i, i) = math::constrain(P(i, i), 1e-6f, P_lim[1]);
	}

	for (int i = 7; i <= 9; i++) {
		// NED position states
		P(i, i) = math::constrain(P(i, i), 1e-6f, P_lim[2]);
	}

	for (int i = 10; i <= 12; i++) {
		// gyro bias states
		P(i, i) = math::constrain(P(i, i), 0.0f, P_lim[3]);
	}

	// force symmetry on the quaternion, velocity and position state covariances
	if (force_symmetry) {
		P.makeRowColSymmetric<13>(0);
	}

	// the following states are optional and are deactivated when not required
	// by ensuring the corresponding covariance matrix values are kept at zero

	// accelerometer bias states
	if (!_accel_bias_inhibit[0] || !_accel_bias_inhibit[1] || !_accel_bias_inhibit[2]) {
		// Find the maximum delta velocity bias state variance and request a covariance reset if any variance is below the safe minimum
		const float minSafeStateVar = 1e-9f;
		float maxStateVar = minSafeStateVar;
		bool resetRequired = false;

		for (uint8_t stateIndex = 13; stateIndex <= 15; stateIndex++) {
			if (_accel_bias_inhibit[stateIndex - 13]) {
				// Skip the check for the inhibited axis
				continue;
			}

			if (P(stateIndex, stateIndex) > maxStateVar) {
				maxStateVar = P(stateIndex, stateIndex);

			} else if (P(stateIndex, stateIndex) < minSafeStateVar) {
				resetRequired = true;
			}
		}

		// To ensure stability of the covariance matrix operations, the ratio of a max and min variance must
		// not exceed 100 and the minimum variance must not fall below the target minimum
		// Also limit variance to a maximum equivalent to a 0.1g uncertainty
		const float minStateVarTarget = 5E-8f;
		float minAllowedStateVar = fmaxf(0.01f * maxStateVar, minStateVarTarget);

		for (uint8_t stateIndex = 13; stateIndex <= 15; stateIndex++) {
			if (_accel_bias_inhibit[stateIndex - 13]) {
				// Skip the check for the inhibited axis
				continue;
			}

			P(stateIndex, stateIndex) = math::constrain(P(stateIndex, stateIndex), minAllowedStateVar,
						    sq(0.1f * CONSTANTS_ONE_G * _dt_ekf_avg));
		}

		// If any one axis has fallen below the safe minimum, all delta velocity covariance terms must be reset to zero
		if (resetRequired) {
			P.uncorrelateCovariance<3>(13);
		}

		// Run additional checks to see if the delta velocity bias has hit limits in a direction that is clearly wrong
		// calculate accel bias term aligned with the gravity vector
		const float dVel_bias_lim = 0.9f * _params.acc_bias_lim * _dt_ekf_avg;
		const float down_dvel_bias = _state.delta_vel_bias.dot(Vector3f(_R_to_earth.row(2)));

		// check that the vertical component of accel bias is consistent with both the vertical position and velocity innovation
		bool bad_acc_bias = false;
		if (fabsf(down_dvel_bias) > dVel_bias_lim) {

			bool bad_vz_gps = _control_status.flags.gps    && (down_dvel_bias * _aid_src_gnss_vel.innovation[2] < 0.0f);
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
			bool bad_vz_ev  = _control_status.flags.ev_vel && (down_dvel_bias * _aid_src_ev_vel.innovation[2] < 0.0f);
#else
			bool bad_vz_ev  = false;
#endif // CONFIG_EKF2_EXTERNAL_VISION

			if (bad_vz_gps || bad_vz_ev) {
				bool bad_z_baro = _control_status.flags.baro_hgt && (down_dvel_bias * _aid_src_baro_hgt.innovation < 0.0f);
				bool bad_z_gps  = _control_status.flags.gps_hgt  && (down_dvel_bias * _aid_src_gnss_hgt.innovation < 0.0f);

#if defined(CONFIG_EKF2_RANGE_FINDER)
				bool bad_z_rng  = _control_status.flags.rng_hgt  && (down_dvel_bias * _aid_src_rng_hgt.innovation  < 0.0f);
#else
				bool bad_z_rng  = false;
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
				bool bad_z_ev   = _control_status.flags.ev_hgt   && (down_dvel_bias * _aid_src_ev_hgt.innovation   < 0.0f);
#else
				bool bad_z_ev   = false;
#endif // CONFIG_EKF2_EXTERNAL_VISION

				if (bad_z_baro || bad_z_gps || bad_z_rng || bad_z_ev) {
					bad_acc_bias = true;
				}
			}
		}

		// record the pass/fail
		if (!bad_acc_bias) {
			_fault_status.flags.bad_acc_bias = false;
			_time_acc_bias_check = _time_delayed_us;

		} else {
			_fault_status.flags.bad_acc_bias = true;
		}

		// if we have failed for 7 seconds continuously, reset the accel bias covariances to fix bad conditioning of
		// the covariance matrix but preserve the variances (diagonals) to allow bias learning to continue
		if (isTimedOut(_time_acc_bias_check, (uint64_t)7e6)) {

			P.uncorrelateCovariance<3>(13);

			_time_acc_bias_check = _time_delayed_us;
			_fault_status.flags.bad_acc_bias = false;
			_warning_events.flags.invalid_accel_bias_cov_reset = true;
			ECL_WARN("invalid accel bias - covariance reset");

		} else if (force_symmetry) {
			// ensure the covariance values are symmetrical
			P.makeRowColSymmetric<3>(13);
		}

	}

	// magnetic field states
	if (!_control_status.flags.mag_3D) {
		zeroMagCov();

	} else {
		// constrain variances
		for (int i = 16; i <= 18; i++) {
			P(i, i) = math::constrain(P(i, i), 0.0f, P_lim[5]);
		}

		for (int i = 19; i <= 21; i++) {
			P(i, i) = math::constrain(P(i, i), 0.0f, P_lim[6]);
		}

		// force symmetry
		if (force_symmetry) {
			P.makeRowColSymmetric<3>(16);
			P.makeRowColSymmetric<3>(19);
		}

	}

	// wind velocity states
	if (!_control_status.flags.wind) {
		P.uncorrelateCovarianceSetVariance<2>(22, 0.0f);

	} else {
		// constrain variances
		for (int i = 22; i <= 23; i++) {
			P(i, i) = math::constrain(P(i, i), 0.0f, P_lim[7]);
		}

		// force symmetry
		if (force_symmetry) {
			P.makeRowColSymmetric<2>(22);
		}
	}
}

// if the covariance correction will result in a negative variance, then
// the covariance matrix is unhealthy and must be corrected
bool Ekf::checkAndFixCovarianceUpdate(const SquareMatrix24f &KHP)
{
	bool healthy = true;

	for (int i = 0; i < _k_num_states; i++) {
		if (P(i, i) < KHP(i, i)) {
			P.uncorrelateCovarianceSetVariance<1>(i, 0.0f);
			healthy = false;
		}
	}

	return healthy;
}

void Ekf::resetMagRelatedCovariances()
{
	resetQuatCov();
	resetMagCov();
}

void Ekf::resetQuatCov()
{
	zeroQuatCov();

	// define the initial angle uncertainty as variances for a rotation vector
	Vector3f rot_vec_var;
	rot_vec_var.setAll(sq(_params.initial_tilt_err));

	initialiseQuatCovariances(rot_vec_var);

	// update the yaw angle variance using the variance of the measurement
	if (_params.mag_fusion_type <= MagFuseType::MAG_3D) {
		// using magnetic heading tuning parameter
		increaseQuatYawErrVariance(sq(fmaxf(_params.mag_heading_noise, 1.0e-2f)));
	}
}

void Ekf::zeroQuatCov()
{
	P.uncorrelateCovarianceSetVariance<2>(0, 0.0f);
	P.uncorrelateCovarianceSetVariance<2>(2, 0.0f);
}

void Ekf::resetMagCov()
{
	// reset the corresponding rows and columns in the covariance matrix and
	// set the variances on the magnetic field states to the measurement variance
	clearMagCov();

	P.uncorrelateCovarianceSetVariance<3>(16, sq(_params.mag_noise));
	P.uncorrelateCovarianceSetVariance<3>(19, sq(_params.mag_noise));

	if (!_control_status.flags.mag_3D) {
		// save covariance data for re-use when auto-switching between heading and 3-axis fusion
		// if already in 3-axis fusion mode, the covariances are automatically saved when switching out
		// of this mode
		saveMagCovData();
	}
}

void Ekf::clearMagCov()
{
	zeroMagCov();
	_mag_decl_cov_reset = false;
}

void Ekf::zeroMagCov()
{
	P.uncorrelateCovarianceSetVariance<3>(16, 0.0f);
	P.uncorrelateCovarianceSetVariance<3>(19, 0.0f);
}

void Ekf::resetZDeltaAngBiasCov()
{
	const float init_delta_ang_bias_var = sq(_params.switch_on_gyro_bias * _dt_ekf_avg);

	P.uncorrelateCovarianceSetVariance<1>(12, init_delta_ang_bias_var);
}
/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ekf.cpp
 * Core functions for ekf attitude and position estimator.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 */

#include "ekf.h"

#include <mathlib/mathlib.h>

bool Ekf::init(uint64_t timestamp)
{
	bool ret = initialise_interface(timestamp);
	reset();
	return ret;
}

void Ekf::reset()
{
	ECL_INFO("reset");

	_state.vel.setZero();
	_state.pos.setZero();
	_state.delta_ang_bias.setZero();
	_state.delta_vel_bias.setZero();
	_state.mag_I.setZero();
	_state.mag_B.setZero();
	_state.wind_vel.setZero();
	_state.quat_nominal.setIdentity();

#if defined(CONFIG_EKF2_RANGE_FINDER)
	_range_sensor.setPitchOffset(_params.rng_sens_pitch);
	_range_sensor.setCosMaxTilt(_params.range_cos_max_tilt);
	_range_sensor.setQualityHysteresis(_params.range_valid_quality_s);
#endif // CONFIG_EKF2_RANGE_FINDER

	_control_status.value = 0;
	_control_status_prev.value = 0;

	_control_status.flags.in_air = true;
	_control_status_prev.flags.in_air = true;

	_ang_rate_delayed_raw.zero();

	_fault_status.value = 0;
	_innov_check_fail_status.value = 0;

	_prev_dvel_bias_var.zero();

	resetGpsDriftCheckFilters();

	_output_predictor.reset();

	// Ekf private fields
	_time_last_horizontal_aiding = 0;
	_time_last_v_pos_aiding = 0;
	_time_last_v_vel_aiding = 0;

	_time_last_hor_pos_fuse = 0;
	_time_last_hgt_fuse = 0;
	_time_last_hor_vel_fuse = 0;
	_time_last_ver_vel_fuse = 0;
	_time_last_heading_fuse = 0;
	_time_last_zero_velocity_fuse = 0;

	_last_known_pos.setZero();

	_time_acc_bias_check = 0;

	_gps_checks_passed = false;
	_gps_alt_ref = NAN;

	_baro_counter = 0;
	_mag_counter = 0;

	_time_bad_vert_accel = 0;
	_time_good_vert_accel = 0;
	_clip_counter = 0;

	resetEstimatorAidStatus(_aid_src_baro_hgt);
#if defined(CONFIG_EKF2_AIRSPEED)
	resetEstimatorAidStatus(_aid_src_airspeed);
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_SIDESLIP)
	resetEstimatorAidStatus(_aid_src_sideslip);
#endif // CONFIG_EKF2_SIDESLIP

	resetEstimatorAidStatus(_aid_src_fake_pos);
	resetEstimatorAidStatus(_aid_src_fake_hgt);

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	resetEstimatorAidStatus(_aid_src_ev_hgt);
	resetEstimatorAidStatus(_aid_src_ev_pos);
	resetEstimatorAidStatus(_aid_src_ev_vel);
	resetEstimatorAidStatus(_aid_src_ev_yaw);
#endif // CONFIG_EKF2_EXTERNAL_VISION

	resetEstimatorAidStatus(_aid_src_gnss_hgt);
	resetEstimatorAidStatus(_aid_src_gnss_pos);
	resetEstimatorAidStatus(_aid_src_gnss_vel);

#if defined(CONFIG_EKF2_GNSS_YAW)
	resetEstimatorAidStatus(_aid_src_gnss_yaw);
#endif // CONFIG_EKF2_GNSS_YAW

	resetEstimatorAidStatus(_aid_src_mag_heading);
	resetEstimatorAidStatus(_aid_src_mag);

#if defined(CONFIG_EKF2_AUXVEL)
	resetEstimatorAidStatus(_aid_src_aux_vel);
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	resetEstimatorAidStatus(_aid_src_optical_flow);
	resetEstimatorAidStatus(_aid_src_terrain_optical_flow);
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_RANGE_FINDER)
	resetEstimatorAidStatus(_aid_src_rng_hgt);
#endif // CONFIG_EKF2_RANGE_FINDER
}

bool Ekf::update()
{
	if (!_filter_initialised) {
		_filter_initialised = initialiseFilter();

		if (!_filter_initialised) {
			return false;
		}
	}

	// Only run the filter if IMU data in the buffer has been updated
	if (_imu_updated) {
		_imu_updated = false;

		// get the oldest IMU data from the buffer
		// TODO: explicitly pop at desired time horizon
		const imuSample imu_sample_delayed = _imu_buffer.get_oldest();

		// perform state and covariance prediction for the main filter
		predictCovariance(imu_sample_delayed);
		predictState(imu_sample_delayed);

		// control fusion of observation data
		controlFusionModes(imu_sample_delayed);

#if defined(CONFIG_EKF2_RANGE_FINDER)
		// run a separate filter for terrain estimation
		runTerrainEstimator(imu_sample_delayed);
#endif // CONFIG_EKF2_RANGE_FINDER

		_output_predictor.correctOutputStates(imu_sample_delayed.time_us, getGyroBias(), getAccelBias(),
							_state.quat_nominal, _state.vel, _state.pos);

		return true;
	}

	return false;
}

bool Ekf::initialiseFilter()
{
	// Filter accel for tilt initialization
	const imuSample &imu_init = _imu_buffer.get_newest();

	// protect against zero data
	if (imu_init.delta_vel_dt < 1e-4f || imu_init.delta_ang_dt < 1e-4f) {
		return false;
	}

	if (_is_first_imu_sample) {
		_accel_lpf.reset(imu_init.delta_vel / imu_init.delta_vel_dt);
		_gyro_lpf.reset(imu_init.delta_ang / imu_init.delta_ang_dt);
		_is_first_imu_sample = false;

	} else {
		_accel_lpf.update(imu_init.delta_vel / imu_init.delta_vel_dt);
		_gyro_lpf.update(imu_init.delta_ang / imu_init.delta_ang_dt);
	}

	if (!initialiseTilt()) {
		return false;
	}

	// initialise the state covariance matrix now we have starting values for all the states
	initialiseCovariance();

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// Initialise the terrain estimator
	initHagl();
#endif // CONFIG_EKF2_RANGE_FINDER

	// reset the output predictor state history to match the EKF initial values
	_output_predictor.alignOutputFilter(_state.quat_nominal, _state.vel, _state.pos);

	return true;
}

bool Ekf::initialiseTilt()
{
	const float accel_norm = _accel_lpf.getState().norm();
	const float gyro_norm = _gyro_lpf.getState().norm();

	if (accel_norm < 0.8f * CONSTANTS_ONE_G ||
	    accel_norm > 1.2f * CONSTANTS_ONE_G ||
	    gyro_norm > math::radians(15.0f)) {
		return false;
	}

	// get initial tilt estimate from delta velocity vector, assuming vehicle is static
	_state.quat_nominal = Quatf(_accel_lpf.getState(), Vector3f(0.f, 0.f, -1.f));
	_R_to_earth = Dcmf(_state.quat_nominal);

	return true;
}

void Ekf::predictState(const imuSample &imu_delayed)
{
	// apply imu bias corrections
	const Vector3f delta_ang_bias_scaled = getGyroBias() * imu_delayed.delta_ang_dt;
	Vector3f corrected_delta_ang = imu_delayed.delta_ang - delta_ang_bias_scaled;

	// subtract component of angular rate due to earth rotation
	corrected_delta_ang -= _R_to_earth.transpose() * _earth_rate_NED * imu_delayed.delta_ang_dt;

	const Quatf dq(AxisAnglef{corrected_delta_ang});

	// rotate the previous quaternion by the delta quaternion using a quaternion multiplication
	_state.quat_nominal = (_state.quat_nominal * dq).normalized();
	_R_to_earth = Dcmf(_state.quat_nominal);

	// Calculate an earth frame delta velocity
	const Vector3f delta_vel_bias_scaled = getAccelBias() * imu_delayed.delta_vel_dt;
	const Vector3f corrected_delta_vel = imu_delayed.delta_vel - delta_vel_bias_scaled;
	const Vector3f corrected_delta_vel_ef = _R_to_earth * corrected_delta_vel;

	// save the previous value of velocity so we can use trapzoidal integration
	const Vector3f vel_last = _state.vel;

	// calculate the increment in velocity using the current orientation
	_state.vel += corrected_delta_vel_ef;

	// compensate for acceleration due to gravity
	_state.vel(2) += CONSTANTS_ONE_G * imu_delayed.delta_vel_dt;

	// predict position states via trapezoidal integration of velocity
	_state.pos += (vel_last + _state.vel) * imu_delayed.delta_vel_dt * 0.5f;

	constrainStates();

	// calculate an average filter update time
	float input = 0.5f * (imu_delayed.delta_vel_dt + imu_delayed.delta_ang_dt);

	// filter and limit input between -50% and +100% of nominal value
	const float filter_update_s = 1e-6f * _params.filter_update_interval_us;
	input = math::constrain(input, 0.5f * filter_update_s, 2.f * filter_update_s);
	_dt_ekf_avg = 0.99f * _dt_ekf_avg + 0.01f * input;

	// some calculations elsewhere in code require a raw angular rate vector so calculate here to avoid duplication
	// protect against possible small timesteps resulting from timing slip on previous frame that can drive spikes into the rate
	// due to insufficient averaging
	if (imu_delayed.delta_ang_dt > 0.25f * _dt_ekf_avg) {
		_ang_rate_delayed_raw = imu_delayed.delta_ang / imu_delayed.delta_ang_dt;
	}


	// calculate a filtered horizontal acceleration with a 1 sec time constant
	// this are used for manoeuvre detection elsewhere
	const float alpha = 1.0f - imu_delayed.delta_vel_dt;
	_accel_lpf_NE = _accel_lpf_NE * alpha + corrected_delta_vel_ef.xy();

	// calculate a yaw change about the earth frame vertical
	const float spin_del_ang_D = corrected_delta_ang.dot(Vector3f(_R_to_earth.row(2)));
	_yaw_delta_ef += spin_del_ang_D;

	// Calculate filtered yaw rate to be used by the magnetometer fusion type selection logic
	// Note fixed coefficients are used to save operations. The exact time constant is not important.
	_yaw_rate_lpf_ef = 0.95f * _yaw_rate_lpf_ef + 0.05f * spin_del_ang_D / imu_delayed.delta_ang_dt;
}
/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ekf_helper.cpp
 * Definition of ekf helper functions.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 *
 */

#include "ekf.h"

#include <mathlib/mathlib.h>
#include <cstdlib>

void Ekf::resetHorizontalVelocityToZero()
{
	_information_events.flags.reset_vel_to_zero = true;
	ECL_INFO("reset velocity to zero");
	// Used when falling back to non-aiding mode of operation
	resetHorizontalVelocityTo(Vector2f{0.f, 0.f}, 25.f);
}

void Ekf::resetVelocityTo(const Vector3f &new_vel, const Vector3f &new_vel_var)
{
	resetHorizontalVelocityTo(Vector2f(new_vel), Vector2f(new_vel_var(0), new_vel_var(1)));
	resetVerticalVelocityTo(new_vel(2), new_vel_var(2));
}

void Ekf::resetHorizontalVelocityTo(const Vector2f &new_horz_vel, const Vector2f &new_horz_vel_var)
{
	const Vector2f delta_horz_vel = new_horz_vel - Vector2f(_state.vel);
	_state.vel.xy() = new_horz_vel;

	if (PX4_ISFINITE(new_horz_vel_var(0))) {
		P.uncorrelateCovarianceSetVariance<1>(4, math::max(sq(0.01f), new_horz_vel_var(0)));
	}

	if (PX4_ISFINITE(new_horz_vel_var(1))) {
		P.uncorrelateCovarianceSetVariance<1>(5, math::max(sq(0.01f), new_horz_vel_var(1)));
	}

	_output_predictor.resetHorizontalVelocityTo(delta_horz_vel);

	// record the state change
	if (_state_reset_status.reset_count.velNE == _state_reset_count_prev.velNE) {
		_state_reset_status.velNE_change = delta_horz_vel;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.velNE_change += delta_horz_vel;
	}

	_state_reset_status.reset_count.velNE++;

	// Reset the timout timer
	_time_last_hor_vel_fuse = _time_delayed_us;
}

void Ekf::resetVerticalVelocityTo(float new_vert_vel, float new_vert_vel_var)
{
	const float delta_vert_vel = new_vert_vel - _state.vel(2);
	_state.vel(2) = new_vert_vel;

	if (PX4_ISFINITE(new_vert_vel_var)) {
		P.uncorrelateCovarianceSetVariance<1>(6, math::max(sq(0.01f), new_vert_vel_var));
	}

	_output_predictor.resetVerticalVelocityTo(delta_vert_vel);

	// record the state change
	if (_state_reset_status.reset_count.velD == _state_reset_count_prev.velD) {
		_state_reset_status.velD_change = delta_vert_vel;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.velD_change += delta_vert_vel;
	}

	_state_reset_status.reset_count.velD++;

	// Reset the timout timer
	_time_last_ver_vel_fuse = _time_delayed_us;
}

void Ekf::resetHorizontalPositionToLastKnown()
{
	ECL_INFO("reset position to last known (%.3f, %.3f)", (double)_last_known_pos(0), (double)_last_known_pos(1));

	_information_events.flags.reset_pos_to_last_known = true;

	// Used when falling back to non-aiding mode of operation
	resetHorizontalPositionTo(_last_known_pos.xy(), sq(_params.pos_noaid_noise));
}

void Ekf::resetHorizontalPositionTo(const Vector2f &new_horz_pos, const Vector2f &new_horz_pos_var)
{
	const Vector2f delta_horz_pos{new_horz_pos - Vector2f{_state.pos}};
	_state.pos.xy() = new_horz_pos;

	if (PX4_ISFINITE(new_horz_pos_var(0))) {
		P.uncorrelateCovarianceSetVariance<1>(7, math::max(sq(0.01f), new_horz_pos_var(0)));
	}

	if (PX4_ISFINITE(new_horz_pos_var(1))) {
		P.uncorrelateCovarianceSetVariance<1>(8, math::max(sq(0.01f), new_horz_pos_var(1)));
	}

	_output_predictor.resetHorizontalPositionTo(delta_horz_pos);

	// record the state change
	if (_state_reset_status.reset_count.posNE == _state_reset_count_prev.posNE) {
		_state_reset_status.posNE_change = delta_horz_pos;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.posNE_change += delta_horz_pos;
	}

	_state_reset_status.reset_count.posNE++;

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	_ev_pos_b_est.setBias(_ev_pos_b_est.getBias() - _state_reset_status.posNE_change);
#endif // CONFIG_EKF2_EXTERNAL_VISION
	//_gps_pos_b_est.setBias(_gps_pos_b_est.getBias() + _state_reset_status.posNE_change);

	// Reset the timout timer
	_time_last_hor_pos_fuse = _time_delayed_us;
}

bool Ekf::isHeightResetRequired() const
{
	// check if height is continuously failing because of accel errors
	const bool continuous_bad_accel_hgt = isTimedOut(_time_good_vert_accel, (uint64_t)_params.bad_acc_reset_delay_us);

	// check if height has been inertial deadreckoning for too long
	const bool hgt_fusion_timeout = isTimedOut(_time_last_hgt_fuse, _params.hgt_fusion_timeout_max);

	return (continuous_bad_accel_hgt || hgt_fusion_timeout);
}

void Ekf::resetVerticalPositionTo(const float new_vert_pos, float new_vert_pos_var)
{
	const float old_vert_pos = _state.pos(2);
	_state.pos(2) = new_vert_pos;

	if (PX4_ISFINITE(new_vert_pos_var)) {
		// the state variance is the same as the observation
		P.uncorrelateCovarianceSetVariance<1>(9, math::max(sq(0.01f), new_vert_pos_var));
	}

	const float delta_z = new_vert_pos - old_vert_pos;

	// apply the change in height / height rate to our newest height / height rate estimate
	// which have already been taken out from the output buffer
	_output_predictor.resetVerticalPositionTo(new_vert_pos, delta_z);

	// record the state change
	if (_state_reset_status.reset_count.posD == _state_reset_count_prev.posD) {
		_state_reset_status.posD_change = delta_z;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.posD_change += delta_z;
	}

	_state_reset_status.reset_count.posD++;

	_baro_b_est.setBias(_baro_b_est.getBias() + delta_z);
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	_ev_hgt_b_est.setBias(_ev_hgt_b_est.getBias() - delta_z);
#endif // CONFIG_EKF2_EXTERNAL_VISION
	_gps_hgt_b_est.setBias(_gps_hgt_b_est.getBias() + delta_z);
#if defined(CONFIG_EKF2_RANGE_FINDER)
	_rng_hgt_b_est.setBias(_rng_hgt_b_est.getBias() + delta_z);
#endif // CONFIG_EKF2_RANGE_FINDER

	// Reset the timout timer
	_time_last_hgt_fuse = _time_delayed_us;
}

void Ekf::resetVerticalVelocityToZero()
{
	// we don't know what the vertical velocity is, so set it to zero
	// Set the variance to a value large enough to allow the state to converge quickly
	// that does not destabilise the filter
	resetVerticalVelocityTo(0.0f, 10.f);
}

void Ekf::constrainStates()
{
	_state.quat_nominal = matrix::constrain(_state.quat_nominal, -1.0f, 1.0f);
	_state.vel = matrix::constrain(_state.vel, -1000.0f, 1000.0f);
	_state.pos = matrix::constrain(_state.pos, -1.e6f, 1.e6f);

	const float delta_ang_bias_limit = getGyroBiasLimit() * _dt_ekf_avg;
	_state.delta_ang_bias = matrix::constrain(_state.delta_ang_bias, -delta_ang_bias_limit, delta_ang_bias_limit);

	const float delta_vel_bias_limit = getAccelBiasLimit() * _dt_ekf_avg;
	_state.delta_vel_bias = matrix::constrain(_state.delta_vel_bias, -delta_vel_bias_limit, delta_vel_bias_limit);

	_state.mag_I = matrix::constrain(_state.mag_I, -1.0f, 1.0f);
	_state.mag_B = matrix::constrain(_state.mag_B, -getMagBiasLimit(), getMagBiasLimit());
	_state.wind_vel = matrix::constrain(_state.wind_vel, -100.0f, 100.0f);
}

float Ekf::compensateBaroForDynamicPressure(const float baro_alt_uncompensated) const
{
#if defined(CONFIG_EKF2_BARO_COMPENSATION)
	if (_control_status.flags.wind && local_position_is_valid()) {
		// calculate static pressure error = Pmeas - Ptruth
		// model position error sensitivity as a body fixed ellipse with a different scale in the positive and
		// negative X and Y directions. Used to correct baro data for positional errors

		// Calculate airspeed in body frame
		const Vector3f vel_imu_rel_body_ned = _R_to_earth * (_ang_rate_delayed_raw % _params.imu_pos_body);
		const Vector3f velocity_earth = _state.vel - vel_imu_rel_body_ned;

		const Vector3f wind_velocity_earth(_state.wind_vel(0), _state.wind_vel(1), 0.0f);

		const Vector3f airspeed_earth = velocity_earth - wind_velocity_earth;

		const Vector3f airspeed_body = _state.quat_nominal.rotateVectorInverse(airspeed_earth);

		const Vector3f K_pstatic_coef(
			airspeed_body(0) >= 0.f ? _params.static_pressure_coef_xp : _params.static_pressure_coef_xn,
			airspeed_body(1) >= 0.f ? _params.static_pressure_coef_yp : _params.static_pressure_coef_yn,
			_params.static_pressure_coef_z);

		const Vector3f airspeed_squared = matrix::min(airspeed_body.emult(airspeed_body), sq(_params.max_correction_airspeed));

		const float pstatic_err = 0.5f * _air_density * (airspeed_squared.dot(K_pstatic_coef));

		// correct baro measurement using pressure error estimate and assuming sea level gravity
		return baro_alt_uncompensated + pstatic_err / (_air_density * CONSTANTS_ONE_G);
	}
#endif // CONFIG_EKF2_BARO_COMPENSATION

	// otherwise return the uncorrected baro measurement
	return baro_alt_uncompensated;
}

// calculate the earth rotation vector
Vector3f Ekf::calcEarthRateNED(float lat_rad) const
{
	return Vector3f(CONSTANTS_EARTH_SPIN_RATE * cosf(lat_rad),
			0.0f,
			-CONSTANTS_EARTH_SPIN_RATE * sinf(lat_rad));
}

void Ekf::getGpsVelPosInnov(float hvel[2], float &vvel, float hpos[2],  float &vpos) const
{
	hvel[0] = _aid_src_gnss_vel.innovation[0];
	hvel[1] = _aid_src_gnss_vel.innovation[1];
	vvel    = _aid_src_gnss_vel.innovation[2];

	hpos[0] = _aid_src_gnss_pos.innovation[0];
	hpos[1] = _aid_src_gnss_pos.innovation[1];
	vpos    = _aid_src_gnss_hgt.innovation;
}

void Ekf::getGpsVelPosInnovVar(float hvel[2], float &vvel, float hpos[2], float &vpos)  const
{
	hvel[0] = _aid_src_gnss_vel.innovation_variance[0];
	hvel[1] = _aid_src_gnss_vel.innovation_variance[1];
	vvel    = _aid_src_gnss_vel.innovation_variance[2];

	hpos[0] = _aid_src_gnss_pos.innovation_variance[0];
	hpos[1] = _aid_src_gnss_pos.innovation_variance[1];
	vpos    = _aid_src_gnss_hgt.innovation_variance;
}

void Ekf::getGpsVelPosInnovRatio(float &hvel, float &vvel, float &hpos, float &vpos) const
{
	hvel = fmaxf(_aid_src_gnss_vel.test_ratio[0], _aid_src_gnss_vel.test_ratio[1]);
	vvel = _aid_src_gnss_vel.test_ratio[2];

	hpos = fmaxf(_aid_src_gnss_pos.test_ratio[0], _aid_src_gnss_pos.test_ratio[1]);
	vpos = _aid_src_gnss_hgt.test_ratio;
}

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
void Ekf::getEvVelPosInnov(float hvel[2], float &vvel, float hpos[2], float &vpos) const
{
	hvel[0] = _aid_src_ev_vel.innovation[0];
	hvel[1] = _aid_src_ev_vel.innovation[1];
	vvel    = _aid_src_ev_vel.innovation[2];

	hpos[0] = _aid_src_ev_pos.innovation[0];
	hpos[1] = _aid_src_ev_pos.innovation[1];
	vpos    = _aid_src_ev_hgt.innovation;
}

void Ekf::getEvVelPosInnovVar(float hvel[2], float &vvel, float hpos[2], float &vpos) const
{
	hvel[0] = _aid_src_ev_vel.innovation_variance[0];
	hvel[1] = _aid_src_ev_vel.innovation_variance[1];
	vvel    = _aid_src_ev_vel.innovation_variance[2];

	hpos[0] = _aid_src_ev_pos.innovation_variance[0];
	hpos[1] = _aid_src_ev_pos.innovation_variance[1];
	vpos    = _aid_src_ev_hgt.innovation_variance;
}

void Ekf::getEvVelPosInnovRatio(float &hvel, float &vvel, float &hpos, float &vpos) const
{
	hvel = fmaxf(_aid_src_ev_vel.test_ratio[0], _aid_src_ev_vel.test_ratio[1]);
	vvel = _aid_src_ev_vel.test_ratio[2];

	hpos = fmaxf(_aid_src_ev_pos.test_ratio[0], _aid_src_ev_pos.test_ratio[1]);
	vpos = _aid_src_ev_hgt.test_ratio;
}
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_AUXVEL)
void Ekf::getAuxVelInnov(float aux_vel_innov[2]) const
{
	aux_vel_innov[0] = _aid_src_aux_vel.innovation[0];
	aux_vel_innov[1] = _aid_src_aux_vel.innovation[1];
}

void Ekf::getAuxVelInnovVar(float aux_vel_innov_var[2]) const
{
	aux_vel_innov_var[0] = _aid_src_aux_vel.innovation_variance[0];
	aux_vel_innov_var[1] = _aid_src_aux_vel.innovation_variance[1];
}
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
void Ekf::getFlowInnov(float flow_innov[2]) const
{
	flow_innov[0] = _aid_src_optical_flow.innovation[0];
	flow_innov[1] = _aid_src_optical_flow.innovation[1];
}

void Ekf::getFlowInnovVar(float flow_innov_var[2]) const
{
	flow_innov_var[0] = _aid_src_optical_flow.innovation_variance[0];
	flow_innov_var[1] = _aid_src_optical_flow.innovation_variance[1];
}

void Ekf::getTerrainFlowInnov(float flow_innov[2]) const
{
	flow_innov[0] = _aid_src_terrain_optical_flow.innovation[0];
	flow_innov[1] = _aid_src_terrain_optical_flow.innovation[1];
}

void Ekf::getTerrainFlowInnovVar(float flow_innov_var[2]) const
{
	flow_innov_var[0] = _aid_src_terrain_optical_flow.innovation_variance[0];
	flow_innov_var[1] = _aid_src_terrain_optical_flow.innovation_variance[1];
}
#endif // CONFIG_EKF2_OPTICAL_FLOW

// get the state vector at the delayed time horizon
matrix::Vector<float, 24> Ekf::getStateAtFusionHorizonAsVector() const
{
	matrix::Vector<float, 24> state;
	state.slice<4, 1>(0, 0) = _state.quat_nominal;
	state.slice<3, 1>(4, 0) = _state.vel;
	state.slice<3, 1>(7, 0) = _state.pos;
	state.slice<3, 1>(10, 0) = _state.delta_ang_bias;
	state.slice<3, 1>(13, 0) = _state.delta_vel_bias;
	state.slice<3, 1>(16, 0) = _state.mag_I;
	state.slice<3, 1>(19, 0) = _state.mag_B;
	state.slice<2, 1>(22, 0) = _state.wind_vel;
	return state;
}

bool Ekf::getEkfGlobalOrigin(uint64_t &origin_time, double &latitude, double &longitude, float &origin_alt) const
{
	origin_time = _pos_ref.getProjectionReferenceTimestamp();
	latitude = _pos_ref.getProjectionReferenceLat();
	longitude = _pos_ref.getProjectionReferenceLon();
	origin_alt  = getEkfGlobalOriginAltitude();
	return _NED_origin_initialised;
}

bool Ekf::setEkfGlobalOrigin(const double latitude, const double longitude, const float altitude)
{
	// sanity check valid latitude/longitude and altitude anywhere between the Mariana Trench and edge of Space
	if (PX4_ISFINITE(latitude) && (abs(latitude) <= 90)
	&& PX4_ISFINITE(longitude) && (abs(longitude) <= 180)
	&& PX4_ISFINITE(altitude) && (altitude > -12'000.f) && (altitude < 100'000.f)
	) {
		bool current_pos_available = false;
		double current_lat = static_cast<double>(NAN);
		double current_lon = static_cast<double>(NAN);

		// if we are already doing aiding, correct for the change in position since the EKF started navigating
		if (_pos_ref.isInitialized() && isHorizontalAidingActive()) {
			_pos_ref.reproject(_state.pos(0), _state.pos(1), current_lat, current_lon);
			current_pos_available = true;
		}

		const float gps_alt_ref_prev = getEkfGlobalOriginAltitude();

		// reinitialize map projection to latitude, longitude, altitude, and reset position
		_pos_ref.initReference(latitude, longitude, _time_delayed_us);
		_gps_alt_ref = altitude;

		// minimum change in position or height that triggers a reset
		static constexpr float MIN_RESET_DIST_M = 0.01f;

		if (current_pos_available) {
			// reset horizontal position
			Vector2f position = _pos_ref.project(current_lat, current_lon);

			if (Vector2f(position - Vector2f(_state.pos)).longerThan(MIN_RESET_DIST_M)) {
				resetHorizontalPositionTo(position);
			}
		}

		// reset vertical position (if there's any change)
		if (fabsf(altitude - gps_alt_ref_prev) > MIN_RESET_DIST_M) {
			// determine current z
			float current_alt = -_state.pos(2) + gps_alt_ref_prev;

			const float gps_hgt_bias = _gps_hgt_b_est.getBias();
			resetVerticalPositionTo(_gps_alt_ref - current_alt);

			// preserve GPS height bias
			_gps_hgt_b_est.setBias(gps_hgt_bias);
		}

		return true;
	}

	return false;
}

// get the 1-sigma horizontal and vertical position uncertainty of the ekf WGS-84 position
void Ekf::get_ekf_gpos_accuracy(float *ekf_eph, float *ekf_epv) const
{
	// report absolute accuracy taking into account the uncertainty in location of the origin
	// If not aiding, return 0 for horizontal position estimate as no estimate is available
	// TODO - allow for baro drift in vertical position error
	float hpos_err = sqrtf(P(7, 7) + P(8, 8) + sq(_gps_origin_eph));

	// If we are dead-reckoning, use the innovations as a conservative alternate measure of the horizontal position error
	// The reason is that complete rejection of measurements is often caused by heading misalignment or inertial sensing errors
	// and using state variances for accuracy reporting is overly optimistic in these situations
	if (_control_status.flags.inertial_dead_reckoning) {
		if (_control_status.flags.gps) {
			hpos_err = math::max(hpos_err, Vector2f(_aid_src_gnss_pos.innovation).norm());
		}

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
		if (_control_status.flags.ev_pos) {
			hpos_err = math::max(hpos_err, Vector2f(_aid_src_ev_pos.innovation).norm());
		}
#endif // CONFIG_EKF2_EXTERNAL_VISION
	}

	*ekf_eph = hpos_err;
	*ekf_epv = sqrtf(P(9, 9) + sq(_gps_origin_epv));
}

// get the 1-sigma horizontal and vertical position uncertainty of the ekf local position
void Ekf::get_ekf_lpos_accuracy(float *ekf_eph, float *ekf_epv) const
{
	// TODO - allow for baro drift in vertical position error
	float hpos_err = sqrtf(P(7, 7) + P(8, 8));

	// If we are dead-reckoning for too long, use the innovations as a conservative alternate measure of the horizontal position error
	// The reason is that complete rejection of measurements is often caused by heading misalignment or inertial sensing errors
	// and using state variances for accuracy reporting is overly optimistic in these situations
	if (_horizontal_deadreckon_time_exceeded) {
		if (_control_status.flags.gps) {
			hpos_err = math::max(hpos_err, Vector2f(_aid_src_gnss_pos.innovation).norm());
		}

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
		if (_control_status.flags.ev_pos) {
			hpos_err = math::max(hpos_err, Vector2f(_aid_src_ev_pos.innovation).norm());
		}
#endif // CONFIG_EKF2_EXTERNAL_VISION
	}

	*ekf_eph = hpos_err;
	*ekf_epv = sqrtf(P(9, 9));
}

// get the 1-sigma horizontal and vertical velocity uncertainty
void Ekf::get_ekf_vel_accuracy(float *ekf_evh, float *ekf_evv) const
{
	float hvel_err = sqrtf(P(4, 4) + P(5, 5));

	// If we are dead-reckoning for too long, use the innovations as a conservative alternate measure of the horizontal velocity error
	// The reason is that complete rejection of measurements is often caused by heading misalignment or inertial sensing errors
	// and using state variances for accuracy reporting is overly optimistic in these situations
	if (_horizontal_deadreckon_time_exceeded) {
		float vel_err_conservative = 0.0f;

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
		if (_control_status.flags.opt_flow) {
			float gndclearance = math::max(_params.rng_gnd_clearance, 0.1f);
			vel_err_conservative = math::max((_terrain_vpos - _state.pos(2)), gndclearance) * Vector2f(_aid_src_optical_flow.innovation).norm();
		}
#endif // CONFIG_EKF2_OPTICAL_FLOW

		if (_control_status.flags.gps) {
			vel_err_conservative = math::max(vel_err_conservative, Vector2f(_aid_src_gnss_pos.innovation).norm());
		}

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
		if (_control_status.flags.ev_pos) {
			vel_err_conservative = math::max(vel_err_conservative, Vector2f(_aid_src_ev_pos.innovation).norm());
		}

		if (_control_status.flags.ev_vel) {
			vel_err_conservative = math::max(vel_err_conservative, Vector2f(_aid_src_ev_vel.innovation).norm());
		}
#endif // CONFIG_EKF2_EXTERNAL_VISION

		hvel_err = math::max(hvel_err, vel_err_conservative);
	}

	*ekf_evh = hvel_err;
	*ekf_evv = sqrtf(P(6, 6));
}

/*
Returns the following vehicle control limits required by the estimator to keep within sensor limitations.
vxy_max : Maximum ground relative horizontal speed (meters/sec). NaN when limiting is not needed.
vz_max : Maximum ground relative vertical speed (meters/sec). NaN when limiting is not needed.
hagl_min : Minimum height above ground (meters). NaN when limiting is not needed.
hagl_max : Maximum height above ground (meters). NaN when limiting is not needed.
*/
void Ekf::get_ekf_ctrl_limits(float *vxy_max, float *vz_max, float *hagl_min, float *hagl_max) const
{
	// Do not require limiting by default
	*vxy_max = NAN;
	*vz_max = NAN;
	*hagl_min = NAN;
	*hagl_max = NAN;

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// Calculate range finder limits
	const float rangefinder_hagl_min = _range_sensor.getValidMinVal();

	// Allow use of 75% of rangefinder maximum range to allow for angular motion
	const float rangefinder_hagl_max = 0.75f * _range_sensor.getValidMaxVal();

	// TODO : calculate visual odometry limits
	const bool relying_on_rangefinder = isOnlyActiveSourceOfVerticalPositionAiding(_control_status.flags.rng_hgt);

	// Keep within range sensor limit when using rangefinder as primary height source
	if (relying_on_rangefinder) {
		*hagl_min = rangefinder_hagl_min;
		*hagl_max = rangefinder_hagl_max;
	}
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// Keep within flow AND range sensor limits when exclusively using optical flow
	const bool relying_on_optical_flow = isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow);

	if (relying_on_optical_flow) {
		// Calculate optical flow limits
		const float flow_hagl_min = fmaxf(rangefinder_hagl_min, _flow_min_distance);
		const float flow_hagl_max = fminf(rangefinder_hagl_max, _flow_max_distance);

		const float flow_constrained_height = math::constrain(_terrain_vpos - _state.pos(2), flow_hagl_min, flow_hagl_max);

		// Allow ground relative velocity to use 50% of available flow sensor range to allow for angular motion
		const float flow_vxy_max = 0.5f * _flow_max_rate * flow_constrained_height;

		*vxy_max = flow_vxy_max;
		*hagl_min = flow_hagl_min;
		*hagl_max = flow_hagl_max;
	}
#endif // CONFIG_EKF2_OPTICAL_FLOW
}

void Ekf::resetImuBias()
{
	resetGyroBias();
	resetAccelBias();
}

void Ekf::resetGyroBias()
{
	// Zero the delta angle and delta velocity bias states
	_state.delta_ang_bias.zero();

	// Zero the corresponding covariances and set
	// variances to the values use for initial alignment
	P.uncorrelateCovarianceSetVariance<3>(10, sq(_params.switch_on_gyro_bias * _dt_ekf_avg));
}

void Ekf::resetAccelBias()
{
	// Zero the delta angle and delta velocity bias states
	_state.delta_vel_bias.zero();

	// Zero the corresponding covariances and set
	// variances to the values use for initial alignment
	P.uncorrelateCovarianceSetVariance<3>(13, sq(_params.switch_on_accel_bias * _dt_ekf_avg));

	// Set previous frame values
	_prev_dvel_bias_var = P.slice<3, 3>(13, 13).diag();
}

// get EKF innovation consistency check status information comprising of:
// status - a bitmask integer containing the pass/fail status for each EKF measurement innovation consistency check
// Innovation Test Ratios - these are the ratio of the innovation to the acceptance threshold.
// A value > 1 indicates that the sensor measurement has exceeded the maximum acceptable level and has been rejected by the EKF
// Where a measurement type is a vector quantity, eg magnetometer, GPS position, etc, the maximum value is returned.
void Ekf::get_innovation_test_status(uint16_t &status, float &mag, float &vel, float &pos, float &hgt, float &tas,
				     float &hagl, float &beta) const
{
	// return the integer bitmask containing the consistency check pass/fail status
	status = _innov_check_fail_status.value;

	// return the largest magnetometer innovation test ratio
	mag = 0.f;

	if (_control_status.flags.mag_hdg) {
		mag = math::max(mag, sqrtf(_aid_src_mag_heading.test_ratio));
	}

	if (_control_status.flags.mag_3D) {
		mag = math::max(mag, sqrtf(Vector3f(_aid_src_mag.test_ratio).max()));
	}

#if defined(CONFIG_EKF2_GNSS_YAW)
	if (_control_status.flags.gps_yaw) {
		mag = math::max(mag, sqrtf(_aid_src_gnss_yaw.test_ratio));
	}
#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	if (_control_status.flags.ev_yaw) {
		mag = math::max(mag, sqrtf(_aid_src_ev_yaw.test_ratio));
	}
#endif // CONFIG_EKF2_EXTERNAL_VISION

	// return the largest velocity and position innovation test ratio
	vel = NAN;
	pos = NAN;

	if (_control_status.flags.gps) {
		float gps_vel = sqrtf(Vector3f(_aid_src_gnss_vel.test_ratio).max());
		vel = math::max(gps_vel, FLT_MIN);

		float gps_pos = sqrtf(Vector2f(_aid_src_gnss_pos.test_ratio).max());
		pos = math::max(gps_pos, FLT_MIN);
	}

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	if (_control_status.flags.ev_vel) {
		float ev_vel = sqrtf(Vector3f(_aid_src_ev_vel.test_ratio).max());
		vel = math::max(vel, ev_vel, FLT_MIN);
	}

	if (_control_status.flags.ev_pos) {
		float ev_pos = sqrtf(Vector2f(_aid_src_ev_pos.test_ratio).max());
		pos = math::max(pos, ev_pos, FLT_MIN);
	}
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	if (isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow)) {
		float of_vel = sqrtf(Vector2f(_aid_src_optical_flow.test_ratio).max());
		vel = math::max(of_vel, FLT_MIN);
	}
#endif // CONFIG_EKF2_OPTICAL_FLOW

	// return the combined vertical position innovation test ratio
	float hgt_sum = 0.f;
	int n_hgt_sources = 0;

	if (_control_status.flags.baro_hgt) {
		hgt_sum += sqrtf(_aid_src_baro_hgt.test_ratio);
		n_hgt_sources++;
	}

	if (_control_status.flags.gps_hgt) {
		hgt_sum += sqrtf(_aid_src_gnss_hgt.test_ratio);
		n_hgt_sources++;
	}

#if defined(CONFIG_EKF2_RANGE_FINDER)
	if (_control_status.flags.rng_hgt) {
		hgt_sum += sqrtf(_aid_src_rng_hgt.test_ratio);
		n_hgt_sources++;
	}
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	if (_control_status.flags.ev_hgt) {
		hgt_sum += sqrtf(_aid_src_ev_hgt.test_ratio);
		n_hgt_sources++;
	}
#endif // CONFIG_EKF2_EXTERNAL_VISION

	if (n_hgt_sources > 0) {
		hgt = math::max(hgt_sum / static_cast<float>(n_hgt_sources), FLT_MIN);

	} else {
		hgt = NAN;
	}

#if defined(CONFIG_EKF2_AIRSPEED)
	// return the airspeed fusion innovation test ratio
	tas = sqrtf(_aid_src_airspeed.test_ratio);
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// return the terrain height innovation test ratio
	hagl = sqrtf(_hagl_test_ratio);
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_SIDESLIP)
	// return the synthetic sideslip innovation test ratio
	beta = sqrtf(_aid_src_sideslip.test_ratio);
#endif // CONFIG_EKF2_SIDESLIP
}

// return a bitmask integer that describes which state estimates are valid
void Ekf::get_ekf_soln_status(uint16_t *status) const
{
	ekf_solution_status_u soln_status{};
	// TODO: Is this accurate enough?
	soln_status.flags.attitude = _control_status.flags.tilt_align && _control_status.flags.yaw_align && (_fault_status.value == 0);
	soln_status.flags.velocity_horiz = (isHorizontalAidingActive() || (_control_status.flags.fuse_beta && _control_status.flags.fuse_aspd)) && (_fault_status.value == 0);
	soln_status.flags.velocity_vert = (_control_status.flags.baro_hgt || _control_status.flags.ev_hgt || _control_status.flags.gps_hgt || _control_status.flags.rng_hgt) && (_fault_status.value == 0);
	soln_status.flags.pos_horiz_rel = (_control_status.flags.gps || _control_status.flags.ev_pos || _control_status.flags.opt_flow) && (_fault_status.value == 0);
	soln_status.flags.pos_horiz_abs = (_control_status.flags.gps || _control_status.flags.ev_pos) && (_fault_status.value == 0);
	soln_status.flags.pos_vert_abs = soln_status.flags.velocity_vert;
	soln_status.flags.pos_vert_agl = isTerrainEstimateValid();
	soln_status.flags.const_pos_mode = !soln_status.flags.velocity_horiz;
	soln_status.flags.pred_pos_horiz_rel = soln_status.flags.pos_horiz_rel;
	soln_status.flags.pred_pos_horiz_abs = soln_status.flags.pos_horiz_abs;

	bool mag_innov_good = true;

	if (_control_status.flags.mag_hdg) {
		if (_aid_src_mag_heading.test_ratio < 1.f) {
			mag_innov_good = false;
		}

	} else if (_control_status.flags.mag_3D) {
		if (Vector3f(_aid_src_mag.test_ratio).max() < 1.f) {
			mag_innov_good = false;
		}
	}

	const bool gps_vel_innov_bad = Vector3f(_aid_src_gnss_vel.test_ratio).max() > 1.f;
	const bool gps_pos_innov_bad = Vector2f(_aid_src_gnss_pos.test_ratio).max() > 1.f;

	soln_status.flags.gps_glitch = (gps_vel_innov_bad || gps_pos_innov_bad) && mag_innov_good;
	soln_status.flags.accel_error = _fault_status.flags.bad_acc_vertical;
	*status = soln_status.value;
}

void Ekf::fuse(const Vector24f &K, float innovation)
{
	_state.quat_nominal -= K.slice<4, 1>(0, 0) * innovation;
	_state.quat_nominal.normalize();
	_R_to_earth = Dcmf(_state.quat_nominal);

	_state.vel -= K.slice<3, 1>(4, 0) * innovation;
	_state.pos -= K.slice<3, 1>(7, 0) * innovation;
	_state.delta_ang_bias -= K.slice<3, 1>(10, 0) * innovation;
	_state.delta_vel_bias -= K.slice<3, 1>(13, 0) * innovation;
	_state.mag_I -= K.slice<3, 1>(16, 0) * innovation;
	_state.mag_B -= K.slice<3, 1>(19, 0) * innovation;
	_state.wind_vel -= K.slice<2, 1>(22, 0) * innovation;
}

void Ekf::uncorrelateQuatFromOtherStates()
{
	P.slice<_k_num_states - 4, 4>(4, 0) = 0.f;
	P.slice<4, _k_num_states - 4>(0, 4) = 0.f;
}

void Ekf::updateDeadReckoningStatus()
{
	updateHorizontalDeadReckoningstatus();
	updateVerticalDeadReckoningStatus();
}

void Ekf::updateHorizontalDeadReckoningstatus()
{
	const bool velPosAiding = (_control_status.flags.gps || _control_status.flags.ev_pos || _control_status.flags.ev_vel)
				  && (isRecent(_time_last_hor_pos_fuse, _params.no_aid_timeout_max)
				      || isRecent(_time_last_hor_vel_fuse, _params.no_aid_timeout_max));

	bool optFlowAiding = false;
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	optFlowAiding = _control_status.flags.opt_flow && isRecent(_aid_src_optical_flow.time_last_fuse, _params.no_aid_timeout_max);
#endif // CONFIG_EKF2_OPTICAL_FLOW

	bool airDataAiding = false;

#if defined(CONFIG_EKF2_AIRSPEED)
	airDataAiding = _control_status.flags.wind &&
				   isRecent(_aid_src_airspeed.time_last_fuse, _params.no_aid_timeout_max) &&
				   isRecent(_aid_src_sideslip.time_last_fuse, _params.no_aid_timeout_max);

	_control_status.flags.wind_dead_reckoning = !velPosAiding && !optFlowAiding && airDataAiding;
#else
	_control_status.flags.wind_dead_reckoning = false;
#endif // CONFIG_EKF2_AIRSPEED

	_control_status.flags.inertial_dead_reckoning = !velPosAiding && !optFlowAiding && !airDataAiding;

	if (!_control_status.flags.inertial_dead_reckoning) {
		if (_time_delayed_us > _params.no_aid_timeout_max) {
			_time_last_horizontal_aiding = _time_delayed_us - _params.no_aid_timeout_max;
		}
	}

	// report if we have been deadreckoning for too long, initial state is deadreckoning until aiding is present
	bool deadreckon_time_exceeded = isTimedOut(_time_last_horizontal_aiding, (uint64_t)_params.valid_timeout_max);

	if (!_horizontal_deadreckon_time_exceeded && deadreckon_time_exceeded) {
		// deadreckon time now exceeded
		ECL_WARN("dead reckon time exceeded");
	}

	_horizontal_deadreckon_time_exceeded = deadreckon_time_exceeded;
}

void Ekf::updateVerticalDeadReckoningStatus()
{
	if (isVerticalPositionAidingActive()) {
		_time_last_v_pos_aiding = _time_last_hgt_fuse;
		_vertical_position_deadreckon_time_exceeded = false;

	} else if (isTimedOut(_time_last_v_pos_aiding, (uint64_t)_params.valid_timeout_max)) {
		_vertical_position_deadreckon_time_exceeded = true;
	}

	if (isVerticalVelocityAidingActive()) {
		_time_last_v_vel_aiding = _time_last_ver_vel_fuse;
		_vertical_velocity_deadreckon_time_exceeded = false;

	} else if (isTimedOut(_time_last_v_vel_aiding, (uint64_t)_params.valid_timeout_max)
		   && _vertical_position_deadreckon_time_exceeded) {

		_vertical_velocity_deadreckon_time_exceeded = true;
	}
}

// calculate the variances for the rotation vector equivalent
Vector3f Ekf::calcRotVecVariances() const
{
	Vector3f rot_var_vec;
	float q0, q1, q2, q3;

	if (_state.quat_nominal(0) >= 0.0f) {
		q0 = _state.quat_nominal(0);
		q1 = _state.quat_nominal(1);
		q2 = _state.quat_nominal(2);
		q3 = _state.quat_nominal(3);

	} else {
		q0 = -_state.quat_nominal(0);
		q1 = -_state.quat_nominal(1);
		q2 = -_state.quat_nominal(2);
		q3 = -_state.quat_nominal(3);
	}
	float t2 = q0*q0;
	float t3 = acosf(q0);
	float t4 = -t2+1.0f;
	float t5 = t2-1.0f;
	if ((t4 > 1e-9f) && (t5 < -1e-9f)) {
		float t6 = 1.0f/t5;
		float t7 = q1*t6*2.0f;
		float t8 = 1.0f/powf(t4,1.5f);
		float t9 = q0*q1*t3*t8*2.0f;
		float t10 = t7+t9;
		float t11 = 1.0f/sqrtf(t4);
		float t12 = q2*t6*2.0f;
		float t13 = q0*q2*t3*t8*2.0f;
		float t14 = t12+t13;
		float t15 = q3*t6*2.0f;
		float t16 = q0*q3*t3*t8*2.0f;
		float t17 = t15+t16;
		rot_var_vec(0) = t10*(P(0,0)*t10+P(1,0)*t3*t11*2.0f)+t3*t11*(P(0,1)*t10+P(1,1)*t3*t11*2.0f)*2.0f;
		rot_var_vec(1) = t14*(P(0,0)*t14+P(2,0)*t3*t11*2.0f)+t3*t11*(P(0,2)*t14+P(2,2)*t3*t11*2.0f)*2.0f;
		rot_var_vec(2) = t17*(P(0,0)*t17+P(3,0)*t3*t11*2.0f)+t3*t11*(P(0,3)*t17+P(3,3)*t3*t11*2.0f)*2.0f;
	} else {
		rot_var_vec = 4.0f * P.slice<3,3>(1,1).diag();
	}

	return rot_var_vec;
}

// initialise the quaternion covariances using rotation vector variances
// do not call before quaternion states are initialised
void Ekf::initialiseQuatCovariances(Vector3f &rot_vec_var)
{
	// calculate an equivalent rotation vector from the quaternion
	float q0,q1,q2,q3;
	if (_state.quat_nominal(0) >= 0.0f) {
		q0 = _state.quat_nominal(0);
		q1 = _state.quat_nominal(1);
		q2 = _state.quat_nominal(2);
		q3 = _state.quat_nominal(3);

	} else {
		q0 = -_state.quat_nominal(0);
		q1 = -_state.quat_nominal(1);
		q2 = -_state.quat_nominal(2);
		q3 = -_state.quat_nominal(3);
	}
	float delta = 2.0f*acosf(q0);
	float scaler = (delta/sinf(delta*0.5f));
	float rotX = scaler*q1;
	float rotY = scaler*q2;
	float rotZ = scaler*q3;

	// autocode generated using matlab symbolic toolbox
	float t2 = rotX*rotX;
	float t4 = rotY*rotY;
	float t5 = rotZ*rotZ;
	float t6 = t2+t4+t5;
	if (t6 > 1e-9f) {
		float t7 = sqrtf(t6);
		float t8 = t7*0.5f;
		float t3 = sinf(t8);
		float t9 = t3*t3;
		float t10 = 1.0f/t6;
		float t11 = 1.0f/sqrtf(t6);
		float t12 = cosf(t8);
		float t13 = 1.0f/powf(t6,1.5f);
		float t14 = t3*t11;
		float t15 = rotX*rotY*t3*t13;
		float t16 = rotX*rotZ*t3*t13;
		float t17 = rotY*rotZ*t3*t13;
		float t18 = t2*t10*t12*0.5f;
		float t27 = t2*t3*t13;
		float t19 = t14+t18-t27;
		float t23 = rotX*rotY*t10*t12*0.5f;
		float t28 = t15-t23;
		float t20 = rotY*rot_vec_var(1)*t3*t11*t28*0.5f;
		float t25 = rotX*rotZ*t10*t12*0.5f;
		float t31 = t16-t25;
		float t21 = rotZ*rot_vec_var(2)*t3*t11*t31*0.5f;
		float t22 = t20+t21-rotX*rot_vec_var(0)*t3*t11*t19*0.5f;
		float t24 = t15-t23;
		float t26 = t16-t25;
		float t29 = t4*t10*t12*0.5f;
		float t34 = t3*t4*t13;
		float t30 = t14+t29-t34;
		float t32 = t5*t10*t12*0.5f;
		float t40 = t3*t5*t13;
		float t33 = t14+t32-t40;
		float t36 = rotY*rotZ*t10*t12*0.5f;
		float t39 = t17-t36;
		float t35 = rotZ*rot_vec_var(2)*t3*t11*t39*0.5f;
		float t37 = t15-t23;
		float t38 = t17-t36;
		float t41 = rot_vec_var(0)*(t15-t23)*(t16-t25);
		float t42 = t41-rot_vec_var(1)*t30*t39-rot_vec_var(2)*t33*t39;
		float t43 = t16-t25;
		float t44 = t17-t36;

		// zero all the quaternion covariances
		P.uncorrelateCovarianceSetVariance<2>(0, 0.0f);
		P.uncorrelateCovarianceSetVariance<2>(2, 0.0f);


		// Update the quaternion internal covariances using auto-code generated using matlab symbolic toolbox
		P(0,0) = rot_vec_var(0)*t2*t9*t10*0.25f+rot_vec_var(1)*t4*t9*t10*0.25f+rot_vec_var(2)*t5*t9*t10*0.25f;
		P(0,1) = t22;
		P(0,2) = t35+rotX*rot_vec_var(0)*t3*t11*(t15-rotX*rotY*t10*t12*0.5f)*0.5f-rotY*rot_vec_var(1)*t3*t11*t30*0.5f;
		P(0,3) = rotX*rot_vec_var(0)*t3*t11*(t16-rotX*rotZ*t10*t12*0.5f)*0.5f+rotY*rot_vec_var(1)*t3*t11*(t17-rotY*rotZ*t10*t12*0.5f)*0.5f-rotZ*rot_vec_var(2)*t3*t11*t33*0.5f;
		P(1,0) = t22;
		P(1,1) = rot_vec_var(0)*(t19*t19)+rot_vec_var(1)*(t24*t24)+rot_vec_var(2)*(t26*t26);
		P(1,2) = rot_vec_var(2)*(t16-t25)*(t17-rotY*rotZ*t10*t12*0.5f)-rot_vec_var(0)*t19*t28-rot_vec_var(1)*t28*t30;
		P(1,3) = rot_vec_var(1)*(t15-t23)*(t17-rotY*rotZ*t10*t12*0.5f)-rot_vec_var(0)*t19*t31-rot_vec_var(2)*t31*t33;
		P(2,0) = t35-rotY*rot_vec_var(1)*t3*t11*t30*0.5f+rotX*rot_vec_var(0)*t3*t11*(t15-t23)*0.5f;
		P(2,1) = rot_vec_var(2)*(t16-t25)*(t17-t36)-rot_vec_var(0)*t19*t28-rot_vec_var(1)*t28*t30;
		P(2,2) = rot_vec_var(1)*(t30*t30)+rot_vec_var(0)*(t37*t37)+rot_vec_var(2)*(t38*t38);
		P(2,3) = t42;
		P(3,0) = rotZ*rot_vec_var(2)*t3*t11*t33*(-0.5f)+rotX*rot_vec_var(0)*t3*t11*(t16-t25)*0.5f+rotY*rot_vec_var(1)*t3*t11*(t17-t36)*0.5f;
		P(3,1) = rot_vec_var(1)*(t15-t23)*(t17-t36)-rot_vec_var(0)*t19*t31-rot_vec_var(2)*t31*t33;
		P(3,2) = t42;
		P(3,3) = rot_vec_var(2)*(t33*t33)+rot_vec_var(0)*(t43*t43)+rot_vec_var(1)*(t44*t44);

	} else {
		// the equations are badly conditioned so use a small angle approximation
		P.uncorrelateCovarianceSetVariance<1>(0, 0.0f);
		P.uncorrelateCovarianceSetVariance<3>(1, 0.25f * rot_vec_var);
	}
}

void Ekf::updateGroundEffect()
{
	if (_control_status.flags.in_air && !_control_status.flags.fixed_wing) {
		if (isTerrainEstimateValid()) {
			// automatically set ground effect if terrain is valid
			float height = _terrain_vpos - _state.pos(2);
			_control_status.flags.gnd_effect = (height < _params.gnd_effect_max_hgt);

		} else if (_control_status.flags.gnd_effect) {
			// Turn off ground effect compensation if it times out
			if (isTimedOut(_time_last_gnd_effect_on, GNDEFFECT_TIMEOUT)) {
				_control_status.flags.gnd_effect = false;
			}
		}

	} else {
		_control_status.flags.gnd_effect = false;
	}
}

// Increase the yaw error variance of the quaternions
// Argument is additional yaw variance in rad**2
void Ekf::increaseQuatYawErrVariance(float yaw_variance)
{
	// See DeriveYawResetEquations.m for derivation which produces code fragments in C_code4.txt file
	// The auto-code was cleaned up and had terms multiplied by zero removed to give the following:

	// Intermediate variables
	float SG[3];
	SG[0] = sq(_state.quat_nominal(0)) - sq(_state.quat_nominal(1)) - sq(_state.quat_nominal(2)) + sq(_state.quat_nominal(3));
	SG[1] = 2*_state.quat_nominal(0)*_state.quat_nominal(2) - 2*_state.quat_nominal(1)*_state.quat_nominal(3);
	SG[2] = 2*_state.quat_nominal(0)*_state.quat_nominal(1) + 2*_state.quat_nominal(2)*_state.quat_nominal(3);

	float SQ[4];
	SQ[0] = 0.5f * ((_state.quat_nominal(1)*SG[0]) - (_state.quat_nominal(0)*SG[2]) + (_state.quat_nominal(3)*SG[1]));
	SQ[1] = 0.5f * ((_state.quat_nominal(0)*SG[1]) - (_state.quat_nominal(2)*SG[0]) + (_state.quat_nominal(3)*SG[2]));
	SQ[2] = 0.5f * ((_state.quat_nominal(3)*SG[0]) - (_state.quat_nominal(1)*SG[1]) + (_state.quat_nominal(2)*SG[2]));
	SQ[3] = 0.5f * ((_state.quat_nominal(0)*SG[0]) + (_state.quat_nominal(1)*SG[2]) + (_state.quat_nominal(2)*SG[1]));

	// Limit yaw variance increase to prevent a badly conditioned covariance matrix
	yaw_variance = fminf(yaw_variance, 1.0e-2f);

	// Add covariances for additonal yaw uncertainty to existing covariances.
	// This assumes that the additional yaw error is uncorrrelated to existing errors
	P(0,0) += yaw_variance*sq(SQ[2]);
	P(0,1) += yaw_variance*SQ[1]*SQ[2];
	P(1,1) += yaw_variance*sq(SQ[1]);
	P(0,2) += yaw_variance*SQ[0]*SQ[2];
	P(1,2) += yaw_variance*SQ[0]*SQ[1];
	P(2,2) += yaw_variance*sq(SQ[0]);
	P(0,3) -= yaw_variance*SQ[2]*SQ[3];
	P(1,3) -= yaw_variance*SQ[1]*SQ[3];
	P(2,3) -= yaw_variance*SQ[0]*SQ[3];
	P(3,3) += yaw_variance*sq(SQ[3]);
	P(1,0) += yaw_variance*SQ[1]*SQ[2];
	P(2,0) += yaw_variance*SQ[0]*SQ[2];
	P(2,1) += yaw_variance*SQ[0]*SQ[1];
	P(3,0) -= yaw_variance*SQ[2]*SQ[3];
	P(3,1) -= yaw_variance*SQ[1]*SQ[3];
	P(3,2) -= yaw_variance*SQ[0]*SQ[3];
}

// save covariance data for re-use when auto-switching between heading and 3-axis fusion
void Ekf::saveMagCovData()
{
	// save variances for XYZ body axis field
	_saved_mag_bf_variance(0) = P(19, 19);
	_saved_mag_bf_variance(1) = P(20, 20);
	_saved_mag_bf_variance(2) = P(21, 21);

	// save the NE axis covariance sub-matrix
	_saved_mag_ef_ne_covmat = P.slice<2, 2>(16, 16);

	// save variance for the D earth axis
	_saved_mag_ef_d_variance = P(18, 18);
}

void Ekf::loadMagCovData()
{
	// re-instate variances for the XYZ body axis field
	P(19, 19) = _saved_mag_bf_variance(0);
	P(20, 20) = _saved_mag_bf_variance(1);
	P(21, 21) = _saved_mag_bf_variance(2);

	// re-instate the NE axis covariance sub-matrix
	P.slice<2, 2>(16, 16) = _saved_mag_ef_ne_covmat;

	// re-instate the D earth axis variance
	P(18, 18) = _saved_mag_ef_d_variance;
}

void Ekf::resetQuatStateYaw(float yaw, float yaw_variance)
{
	// save a copy of the quaternion state for later use in calculating the amount of reset change
	const Quatf quat_before_reset = _state.quat_nominal;

	// update transformation matrix from body to world frame using the current estimate
	// update the rotation matrix using the new yaw value
	_R_to_earth = updateYawInRotMat(yaw, Dcmf(_state.quat_nominal));

	// calculate the amount that the quaternion has changed by
	const Quatf quat_after_reset(_R_to_earth);
	const Quatf q_error((quat_after_reset * quat_before_reset.inversed()).normalized());

	// update quaternion states
	_state.quat_nominal = quat_after_reset;
	uncorrelateQuatFromOtherStates();

	// update the yaw angle variance
	if (yaw_variance > FLT_EPSILON) {
		increaseQuatYawErrVariance(yaw_variance);
	}

	// add the reset amount to the output observer buffered data
	_output_predictor.resetQuaternion(q_error);

	// record the state change
	if (_state_reset_status.reset_count.quat == _state_reset_count_prev.quat) {
		_state_reset_status.quat_change = q_error;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.quat_change = q_error * _state_reset_status.quat_change;
		_state_reset_status.quat_change.normalize();
	}

	_state_reset_status.reset_count.quat++;

	_time_last_heading_fuse = _time_delayed_us;

	_last_static_yaw = NAN;
}

// Resets the main Nav EKf yaw to the estimator from the EKF-GSF yaw estimator
// Resets the horizontal velocity and position to the default navigation sensor
// Returns true if the reset was successful
bool Ekf::resetYawToEKFGSF()
{
	if (!isYawEmergencyEstimateAvailable()) {
		return false;
	}

	resetQuatStateYaw(_yawEstimator.getYaw(), _yawEstimator.getYawVar());

	// record a magnetic field alignment event to prevent possibility of the EKF trying to reset the yaw to the mag later in flight
	_flt_mag_align_start_time = _time_delayed_us;
	_control_status.flags.yaw_align = true;

	if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
		// stop using the magnetometer in the main EKF otherwise it's fusion could drag the yaw around
		// and cause another navigation failure
		_control_status.flags.mag_fault = true;
		_warning_events.flags.emergency_yaw_reset_mag_stopped = true;

	} else if (_control_status.flags.gps_yaw) {
		_control_status.flags.gps_yaw_fault = true;
		_warning_events.flags.emergency_yaw_reset_gps_yaw_stopped = true;

	}
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	if (_control_status.flags.ev_yaw) {
		_inhibit_ev_yaw_use = true;
	}
#endif // CONFIG_EKF2_EXTERNAL_VISION

	return true;
}

bool Ekf::isYawEmergencyEstimateAvailable() const
{
	// don't allow reet using the EKF-GSF estimate until the filter has started fusing velocity
	// data and the yaw estimate has converged
	if (!_yawEstimator.isActive()) {
		return false;
	}

	return _yawEstimator.getYawVar() < sq(_params.EKFGSF_yaw_err_max);
}

bool Ekf::getDataEKFGSF(float *yaw_composite, float *yaw_variance, float yaw[N_MODELS_EKFGSF],
			float innov_VN[N_MODELS_EKFGSF], float innov_VE[N_MODELS_EKFGSF], float weight[N_MODELS_EKFGSF])
{
	return _yawEstimator.getLogData(yaw_composite, yaw_variance, yaw, innov_VN, innov_VE, weight);
}

void Ekf::resetGpsDriftCheckFilters()
{
	_gps_velNE_filt.setZero();
	_gps_pos_deriv_filt.setZero();

	_gps_horizontal_position_drift_rate_m_s = NAN;
	_gps_vertical_position_drift_rate_m_s = NAN;
	_gps_filtered_horizontal_velocity_m_s = NAN;
}

void Ekf::resetWind()
{
#if defined(CONFIG_EKF2_AIRSPEED)
	if (_control_status.flags.fuse_aspd && isRecent(_airspeed_sample_delayed.time_us, 1e6)) {
		resetWindUsingAirspeed(_airspeed_sample_delayed);
		return;
	}
#endif // CONFIG_EKF2_AIRSPEED

	resetWindToZero();
}

void Ekf::resetWindToZero()
{
	ECL_INFO("reset wind to zero");

	// If we don't have an airspeed measurement, then assume the wind is zero
	_state.wind_vel.setZero();

	// start with a small initial uncertainty to improve the initial estimate
	P.uncorrelateCovarianceSetVariance<2>(22, _params.initial_wind_uncertainty);
}
/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "EKFGSF_yaw.h"
#include <cstdlib>

#include "python/ekf_derivation/generated/yaw_est_predict_covariance.h"
#include "python/ekf_derivation/generated/yaw_est_compute_measurement_update.h"

EKFGSF_yaw::EKFGSF_yaw()
{
	initialiseEKFGSF();
}

void EKFGSF_yaw::update(const imuSample &imu_sample,
			bool run_EKF,			// set to true when flying or movement is suitable for yaw estimation
			const Vector3f &imu_gyro_bias)  // estimated rate gyro bias (rad/sec)
{
	// to reduce effect of vibration, filter using an LPF whose time constant is 1/10 of the AHRS tilt correction time constant
	const float filter_coef = fminf(10.f * imu_sample.delta_vel_dt * _tilt_gain, 1.f);
	const Vector3f accel = imu_sample.delta_vel / fmaxf(imu_sample.delta_vel_dt, 0.001f);
	_ahrs_accel = _ahrs_accel * (1.f - filter_coef) + accel * filter_coef;

	// Initialise states first time
	if (!_ahrs_ekf_gsf_tilt_aligned) {
		// check for excessive acceleration to reduce likelihood of large initial roll/pitch errors
		// due to vehicle movement
		const float accel_norm_sq = accel.norm_squared();
		const float upper_accel_limit = CONSTANTS_ONE_G * 1.1f;
		const float lower_accel_limit = CONSTANTS_ONE_G * 0.9f;
		const bool ok_to_align = (accel_norm_sq > sq(lower_accel_limit)) && (accel_norm_sq < sq(upper_accel_limit));

		if (ok_to_align) {
			ahrsAlignTilt(imu_sample.delta_vel);
			_ahrs_ekf_gsf_tilt_aligned = true;
		}

		return;
	}

	// calculate common values used by the AHRS complementary filter models
	_ahrs_accel_norm = _ahrs_accel.norm();

	// AHRS prediction cycle for each model - this always runs
	_ahrs_accel_fusion_gain = ahrsCalcAccelGain();

	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
		predictEKF(model_index, imu_sample.delta_ang, imu_sample.delta_ang_dt, imu_sample.delta_vel, imu_sample.delta_vel_dt);
	}

	// The 3-state EKF models only run when flying to avoid corrupted estimates due to operator handling and GPS interference
	if (run_EKF && _vel_data_updated) {
		if (!_ekf_gsf_vel_fuse_started) {
			initialiseEKFGSF(_vel_NE, _vel_accuracy);

			// Initialise to gyro bias estimate from main filter because there could be a large
			// uncorrected rate gyro bias error about the gravity vector
			ahrsAlignYaw(imu_gyro_bias);

			_ekf_gsf_vel_fuse_started = true;

		} else {
			bool bad_update = false;

			for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
				// subsequent measurements are fused as direct state observations
				if (!updateEKF(model_index, _vel_NE, _vel_accuracy)) {
					bad_update = true;
				}
			}

			if (!bad_update) {
				float total_weight = 0.0f;
				// calculate weighting for each model assuming a normal distribution
				const float min_weight = 1e-5f;
				uint8_t n_weight_clips = 0;

				for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
					_model_weights(model_index) = gaussianDensity(model_index) * _model_weights(model_index);

					if (_model_weights(model_index) < min_weight) {
						n_weight_clips++;
						_model_weights(model_index) = min_weight;
					}

					total_weight += _model_weights(model_index);
				}

				// normalise the weighting function
				if (n_weight_clips < N_MODELS_EKFGSF) {
					_model_weights /= total_weight;

				} else {
					// all weights have collapsed due to excessive innovation variances so reset filters
					initialiseEKFGSF(_vel_NE, _vel_accuracy);
				}
			}
		}

	} else if (_ekf_gsf_vel_fuse_started && !run_EKF) {
		// wait to fly again
		_ekf_gsf_vel_fuse_started = false;
	}

	// Calculate a composite yaw vector as a weighted average of the states for each model.
	// To avoid issues with angle wrapping, the yaw state is converted to a vector with length
	// equal to the weighting value before it is summed.
	Vector2f yaw_vector;

	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
		yaw_vector(0) += _model_weights(model_index) * cosf(_ekf_gsf[model_index].X(2));
		yaw_vector(1) += _model_weights(model_index) * sinf(_ekf_gsf[model_index].X(2));
	}

	_gsf_yaw = atan2f(yaw_vector(1), yaw_vector(0));

	// calculate a composite variance for the yaw state from a weighted average of the variance for each model
	// models with larger innovations are weighted less
	_gsf_yaw_variance = 0.0f;

	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
		const float yaw_delta = wrap_pi(_ekf_gsf[model_index].X(2) - _gsf_yaw);
		_gsf_yaw_variance += _model_weights(model_index) * (_ekf_gsf[model_index].P(2, 2) + yaw_delta * yaw_delta);
	}

	// prevent the same velocity data being used more than once
	_vel_data_updated = false;
}

void EKFGSF_yaw::ahrsPredict(const uint8_t model_index, const Vector3f &delta_ang, const float delta_ang_dt)
{
	// generate attitude solution using simple complementary filter for the selected model
	const Vector3f ang_rate = delta_ang / fmaxf(delta_ang_dt, 0.001f) - _ahrs_ekf_gsf[model_index].gyro_bias;

	const Dcmf R_to_body = _ahrs_ekf_gsf[model_index].R.transpose();
	const Vector3f gravity_direction_bf = R_to_body.col(2);

	// Perform angular rate correction using accel data and reduce correction as accel magnitude moves away from 1 g (reduces drift when vehicle picked up and moved).
	// During fixed wing flight, compensate for centripetal acceleration assuming coordinated turns and X axis forward
	Vector3f tilt_correction;

	if (_ahrs_accel_fusion_gain > 0.0f) {

		Vector3f accel = _ahrs_accel;

		if (PX4_ISFINITE(_true_airspeed) && (_true_airspeed > FLT_EPSILON)) {
			// Calculate body frame centripetal acceleration with assumption X axis is aligned with the airspeed vector
			// Use cross product of body rate and body frame airspeed vector
			const Vector3f centripetal_accel_bf = Vector3f(0.0f, _true_airspeed * ang_rate(2), - _true_airspeed * ang_rate(1));

			// correct measured accel for centripetal acceleration
			accel -= centripetal_accel_bf;
		}

		tilt_correction = (gravity_direction_bf % accel) * _ahrs_accel_fusion_gain / _ahrs_accel_norm;
	}

	// Gyro bias estimation
	constexpr float gyro_bias_limit = 0.05f;
	const float spin_rate = ang_rate.length();

	if (spin_rate < math::radians(10.f)) {
		_ahrs_ekf_gsf[model_index].gyro_bias -= tilt_correction * (_gyro_bias_gain * delta_ang_dt);
		_ahrs_ekf_gsf[model_index].gyro_bias = matrix::constrain(_ahrs_ekf_gsf[model_index].gyro_bias, -gyro_bias_limit,
						       gyro_bias_limit);
	}

	// delta angle from previous to current frame
	const Vector3f delta_angle_corrected = delta_ang + (tilt_correction - _ahrs_ekf_gsf[model_index].gyro_bias) *
					       delta_ang_dt;

	// Apply delta angle to rotation matrix
	_ahrs_ekf_gsf[model_index].R = ahrsPredictRotMat(_ahrs_ekf_gsf[model_index].R, delta_angle_corrected);
}

void EKFGSF_yaw::ahrsAlignTilt(const Vector3f &delta_vel)
{
	// Rotation matrix is constructed directly from acceleration measurement and will be the same for
	// all models so only need to calculate it once. Assumptions are:
	// 1) Yaw angle is zero - yaw is aligned later for each model when velocity fusion commences.
	// 2) The vehicle is not accelerating so all of the measured acceleration is due to gravity.

	// Calculate earth frame Down axis unit vector rotated into body frame
	const Vector3f down_in_bf = -delta_vel.normalized();

	// Calculate earth frame North axis unit vector rotated into body frame, orthogonal to 'down_in_bf'
	const Vector3f i_vec_bf(1.f, 0.f, 0.f);
	Vector3f north_in_bf = i_vec_bf - down_in_bf * (i_vec_bf.dot(down_in_bf));
	north_in_bf.normalize();

	// Calculate earth frame East axis unit vector rotated into body frame, orthogonal to 'down_in_bf' and 'north_in_bf'
	const Vector3f east_in_bf = down_in_bf % north_in_bf;

	// Each column in a rotation matrix from earth frame to body frame represents the projection of the
	// corresponding earth frame unit vector rotated into the body frame, eg 'north_in_bf' would be the first column.
	// We need the rotation matrix from body frame to earth frame so the earth frame unit vectors rotated into body
	// frame are copied into corresponding rows instead.
	Dcmf R;
	R.setRow(0, north_in_bf);
	R.setRow(1, east_in_bf);
	R.setRow(2, down_in_bf);

	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) {
		_ahrs_ekf_gsf[model_index].R = R;
	}
}

void EKFGSF_yaw::ahrsAlignYaw(const Vector3f &imu_gyro_bias)
{
	// Align yaw angle for each model
	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) {
		Dcmf &R = _ahrs_ekf_gsf[model_index].R;
		const float yaw = wrap_pi(_ekf_gsf[model_index].X(2));
		R = updateYawInRotMat(yaw, R);

		_ahrs_ekf_gsf[model_index].aligned = true;

		_ahrs_ekf_gsf[model_index].gyro_bias = imu_gyro_bias;
	}
}

void EKFGSF_yaw::predictEKF(const uint8_t model_index, const Vector3f &delta_ang, const float delta_ang_dt,
			    const Vector3f &delta_vel, const float delta_vel_dt)
{
	// generate an attitude reference using IMU data
	ahrsPredict(model_index, delta_ang, delta_ang_dt);

	// we don't start running the EKF part of the algorithm until there are regular velocity observations
	if (!_ekf_gsf_vel_fuse_started) {
		return;
	}

	// Calculate the yaw state using a projection onto the horizontal that avoids gimbal lock
	_ekf_gsf[model_index].X(2) = getEulerYaw(_ahrs_ekf_gsf[model_index].R);

	// calculate delta velocity in a horizontal front-right frame
	const Vector3f del_vel_NED = _ahrs_ekf_gsf[model_index].R * delta_vel;
	const float cos_yaw = cosf(_ekf_gsf[model_index].X(2));
	const float sin_yaw = sinf(_ekf_gsf[model_index].X(2));
	const float dvx =   del_vel_NED(0) * cos_yaw + del_vel_NED(1) * sin_yaw;
	const float dvy = - del_vel_NED(0) * sin_yaw + del_vel_NED(1) * cos_yaw;

	// Use fixed values for delta velocity and delta angle process noise variances
	const float d_vel_var = sq(_accel_noise * delta_vel_dt);
	const float d_ang_var = sq(_gyro_noise * delta_ang_dt);

	sym::YawEstPredictCovariance(_ekf_gsf[model_index].X, _ekf_gsf[model_index].P, Vector2f(dvx, dvy), d_vel_var, d_ang_var, &_ekf_gsf[model_index].P);

	// covariance matrix is symmetrical, so copy upper half to lower half
	_ekf_gsf[model_index].P(1, 0) = _ekf_gsf[model_index].P(0, 1);
	_ekf_gsf[model_index].P(2, 0) = _ekf_gsf[model_index].P(0, 2);
	_ekf_gsf[model_index].P(2, 1) = _ekf_gsf[model_index].P(1, 2);

	// constrain variances
	const float min_var = 1e-6f;

	for (unsigned index = 0; index < 3; index++) {
		_ekf_gsf[model_index].P(index, index) = fmaxf(_ekf_gsf[model_index].P(index, index), min_var);
	}

	// sum delta velocities in earth frame:
	_ekf_gsf[model_index].X(0) += del_vel_NED(0);
	_ekf_gsf[model_index].X(1) += del_vel_NED(1);
}

// Update EKF states and covariance for specified model index using velocity measurement
bool EKFGSF_yaw::updateEKF(const uint8_t model_index, const Vector2f &vel_NE, const float vel_accuracy)
{
	// set observation variance from accuracy estimate supplied by GPS and apply a sanity check minimum
	const float vel_obs_var = sq(fmaxf(vel_accuracy, 0.01f));

	// calculate velocity observation innovations
	_ekf_gsf[model_index].innov(0) = _ekf_gsf[model_index].X(0) - vel_NE(0);
	_ekf_gsf[model_index].innov(1) = _ekf_gsf[model_index].X(1) - vel_NE(1);

	matrix::Matrix<float, 3, 2> K;
	matrix::SquareMatrix<float, 3> P_new;

	sym::YawEstComputeMeasurementUpdate(_ekf_gsf[model_index].P,
					    vel_obs_var,
					    FLT_EPSILON,
					    &_ekf_gsf[model_index].S_inverse,
					    &_ekf_gsf[model_index].S_det_inverse,
					    &K,
					    &P_new);

	_ekf_gsf[model_index].P = P_new;

	// copy upper to lower diagonal
	_ekf_gsf[model_index].P(1, 0) = _ekf_gsf[model_index].P(0, 1);
	_ekf_gsf[model_index].P(2, 0) = _ekf_gsf[model_index].P(0, 2);
	_ekf_gsf[model_index].P(2, 1) = _ekf_gsf[model_index].P(1, 2);

	// constrain variances
	const float min_var = 1e-6f;

	for (unsigned index = 0; index < 3; index++) {
		_ekf_gsf[model_index].P(index, index) = fmaxf(_ekf_gsf[model_index].P(index, index), min_var);
	}

	// test ratio = transpose(innovation) * inverse(innovation variance) * innovation = [1x2] * [2,2] * [2,1] = [1,1]
	const float test_ratio = _ekf_gsf[model_index].innov * (_ekf_gsf[model_index].S_inverse * _ekf_gsf[model_index].innov);

	// Perform a chi-square innovation consistency test and calculate a compression scale factor
	// that limits the magnitude of innovations to 5-sigma
	// If the test ratio is greater than 25 (5 Sigma) then reduce the length of the innovation vector to clip it at 5-Sigma
	// This protects from large measurement spikes
	const float innov_comp_scale_factor = test_ratio > 25.f ? sqrtf(25.0f / test_ratio) : 1.f;

	// Correct the state vector and capture the change in yaw angle
	const float oldYaw = _ekf_gsf[model_index].X(2);

	_ekf_gsf[model_index].X -= (K * _ekf_gsf[model_index].innov) * innov_comp_scale_factor;

	const float yawDelta = _ekf_gsf[model_index].X(2) - oldYaw;

	// apply the change in yaw angle to the AHRS
	// take advantage of sparseness in the yaw rotation matrix
	const float cosYaw = cosf(yawDelta);
	const float sinYaw = sinf(yawDelta);
	const float R_prev00 = _ahrs_ekf_gsf[model_index].R(0, 0);
	const float R_prev01 = _ahrs_ekf_gsf[model_index].R(0, 1);
	const float R_prev02 = _ahrs_ekf_gsf[model_index].R(0, 2);

	_ahrs_ekf_gsf[model_index].R(0, 0) = R_prev00 * cosYaw - _ahrs_ekf_gsf[model_index].R(1, 0) * sinYaw;
	_ahrs_ekf_gsf[model_index].R(0, 1) = R_prev01 * cosYaw - _ahrs_ekf_gsf[model_index].R(1, 1) * sinYaw;
	_ahrs_ekf_gsf[model_index].R(0, 2) = R_prev02 * cosYaw - _ahrs_ekf_gsf[model_index].R(1, 2) * sinYaw;
	_ahrs_ekf_gsf[model_index].R(1, 0) = R_prev00 * sinYaw + _ahrs_ekf_gsf[model_index].R(1, 0) * cosYaw;
	_ahrs_ekf_gsf[model_index].R(1, 1) = R_prev01 * sinYaw + _ahrs_ekf_gsf[model_index].R(1, 1) * cosYaw;
	_ahrs_ekf_gsf[model_index].R(1, 2) = R_prev02 * sinYaw + _ahrs_ekf_gsf[model_index].R(1, 2) * cosYaw;

	return true;
}

void EKFGSF_yaw::initialiseEKFGSF(const Vector2f &vel_NE, const float vel_accuracy)
{
	_gsf_yaw = 0.0f;
	_ekf_gsf_vel_fuse_started = false;
	_gsf_yaw_variance = sq(M_PI_F / 2.f);
	_model_weights.setAll(1.0f / (float)N_MODELS_EKFGSF);  // All filter models start with the same weight

	memset(&_ekf_gsf, 0, sizeof(_ekf_gsf));
	const float yaw_increment = 2.0f * M_PI_F / (float)N_MODELS_EKFGSF;

	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) {
		// evenly space initial yaw estimates in the region between +-Pi
		_ekf_gsf[model_index].X(2) = -M_PI_F + (0.5f * yaw_increment) + ((float)model_index * yaw_increment);

		// take velocity states and corresponding variance from last measurement
		_ekf_gsf[model_index].X(0) = vel_NE(0);
		_ekf_gsf[model_index].X(1) = vel_NE(1);
		_ekf_gsf[model_index].P(0, 0) = sq(fmaxf(vel_accuracy, 0.01f));
		_ekf_gsf[model_index].P(1, 1) = _ekf_gsf[model_index].P(0, 0);

		// use half yaw interval for yaw uncertainty
		_ekf_gsf[model_index].P(2, 2) = sq(0.5f * yaw_increment);
	}
}

float EKFGSF_yaw::gaussianDensity(const uint8_t model_index) const
{
	// calculate transpose(innovation) * inv(S) * innovation
	const float normDist = _ekf_gsf[model_index].innov.dot(_ekf_gsf[model_index].S_inverse * _ekf_gsf[model_index].innov);

	return (1.f / (2.f * M_PI_F)) * sqrtf(_ekf_gsf[model_index].S_det_inverse) * expf(-0.5f * normDist);
}

bool EKFGSF_yaw::getLogData(float *yaw_composite, float *yaw_variance, float yaw[N_MODELS_EKFGSF],
			    float innov_VN[N_MODELS_EKFGSF], float innov_VE[N_MODELS_EKFGSF], float weight[N_MODELS_EKFGSF]) const
{
	if (_ekf_gsf_vel_fuse_started) {
		*yaw_composite = _gsf_yaw;
		*yaw_variance = _gsf_yaw_variance;

		for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) {
			yaw[model_index] = _ekf_gsf[model_index].X(2);
			innov_VN[model_index] = _ekf_gsf[model_index].innov(0);
			innov_VE[model_index] = _ekf_gsf[model_index].innov(1);
			weight[model_index] = _model_weights(model_index);
		}

		return true;
	}

	return false;
}

float EKFGSF_yaw::ahrsCalcAccelGain() const
{
	// Calculate the acceleration fusion gain using a continuous function that is unity at 1g and zero
	// at the min and max g value. Allow for more acceleration when flying as a fixed wing vehicle using centripetal
	// acceleration correction as higher and more sustained g will be experienced.
	// Use a quadratic instead of linear function to prevent vibration around 1g reducing the tilt correction effectiveness.
	// see https://www.desmos.com/calculator/dbqbxvnwfg

	float attenuation = 2.f;
	const bool centripetal_accel_compensation_enabled = PX4_ISFINITE(_true_airspeed) && (_true_airspeed > FLT_EPSILON);

	if (centripetal_accel_compensation_enabled && (_ahrs_accel_norm > CONSTANTS_ONE_G)) {
		attenuation = 1.f;
	}

	const float delta_accel_g = (_ahrs_accel_norm - CONSTANTS_ONE_G) / CONSTANTS_ONE_G;
	return _tilt_gain * sq(1.f - math::min(attenuation * fabsf(delta_accel_g), 1.f));
}

Matrix3f EKFGSF_yaw::ahrsPredictRotMat(const Matrix3f &R, const Vector3f &g)
{
	Matrix3f ret = R;
	ret(0, 0) += R(0, 1) * g(2) - R(0, 2) * g(1);
	ret(0, 1) += R(0, 2) * g(0) - R(0, 0) * g(2);
	ret(0, 2) += R(0, 0) * g(1) - R(0, 1) * g(0);
	ret(1, 0) += R(1, 1) * g(2) - R(1, 2) * g(1);
	ret(1, 1) += R(1, 2) * g(0) - R(1, 0) * g(2);
	ret(1, 2) += R(1, 0) * g(1) - R(1, 1) * g(0);
	ret(2, 0) += R(2, 1) * g(2) - R(2, 2) * g(1);
	ret(2, 1) += R(2, 2) * g(0) - R(2, 0) * g(2);
	ret(2, 2) += R(2, 0) * g(1) - R(2, 1) * g(0);

	// Renormalise rows
	for (uint8_t r = 0; r < 3; r++) {
		const float rowLengthSq = ret.row(r).norm_squared();

		if (rowLengthSq > FLT_EPSILON) {
			// Use linear approximation for inverse sqrt taking advantage of the row length being close to 1.0
			const float rowLengthInv = 1.5f - 0.5f * rowLengthSq;
			ret.row(r) *=  rowLengthInv;
		}
	}

	return ret;
}

void EKFGSF_yaw::setVelocity(const Vector2f &velocity, float accuracy)
{
	_vel_NE = velocity;
	_vel_accuracy = accuracy;
	_vel_data_updated = true;
}
/****************************************************************************
 *
 *   Copyright (c) 2013 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file estimator_interface.cpp
 * Definition of base class for attitude estimators
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 * @author Siddharth B Purohit <siddharthbharatpurohit@gmail.com>
 */

#include "estimator_interface.h"

#include <mathlib/mathlib.h>

EstimatorInterface::~EstimatorInterface()
{
	delete _gps_buffer;
	delete _mag_buffer;
	delete _baro_buffer;
#if defined(CONFIG_EKF2_RANGE_FINDER)
	delete _range_buffer;
#endif // CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_AIRSPEED)
	delete _airspeed_buffer;
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	delete _flow_buffer;
#endif // CONFIG_EKF2_OPTICAL_FLOW
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	delete _ext_vision_buffer;
#endif // CONFIG_EKF2_EXTERNAL_VISION
#if defined(CONFIG_EKF2_DRAG_FUSION)
	delete _drag_buffer;
#endif // CONFIG_EKF2_DRAG_FUSION
#if defined(CONFIG_EKF2_AUXVEL)
	delete _auxvel_buffer;
#endif // CONFIG_EKF2_AUXVEL
}

// Accumulate imu data and store to buffer at desired rate
void EstimatorInterface::setIMUData(const imuSample &imu_sample)
{
	// TODO: resolve misplaced responsibility
	if (!_initialised) {
		_initialised = init(imu_sample.time_us);
	}

	_time_latest_us = imu_sample.time_us;

	// the output observer always runs
	_output_predictor.calculateOutputStates(imu_sample.time_us, imu_sample.delta_ang, imu_sample.delta_ang_dt, imu_sample.delta_vel, imu_sample.delta_vel_dt);

	// accumulate and down-sample imu data and push to the buffer when new downsampled data becomes available
	if (_imu_down_sampler.update(imu_sample)) {

		_imu_updated = true;

		_imu_buffer.push(_imu_down_sampler.getDownSampledImuAndTriggerReset());

		// get the oldest data from the buffer
		_time_delayed_us = _imu_buffer.get_oldest().time_us;

		// calculate the minimum interval between observations required to guarantee no loss of data
		// this will occur if data is overwritten before its time stamp falls behind the fusion time horizon
		_min_obs_interval_us = (imu_sample.time_us - _time_delayed_us) / (_obs_buffer_length - 1);
	}

#if defined(CONFIG_EKF2_DRAG_FUSION)
	setDragData(imu_sample);
#endif // CONFIG_EKF2_DRAG_FUSION
}

void EstimatorInterface::setMagData(const magSample &mag_sample)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_mag_buffer == nullptr) {
		_mag_buffer = new RingBuffer<magSample>(_obs_buffer_length);

		if (_mag_buffer == nullptr || !_mag_buffer->valid()) {
			delete _mag_buffer;
			_mag_buffer = nullptr;
			printBufferAllocationFailed("mag");
			return;
		}
	}

	const int64_t time_us = mag_sample.time_us
				- static_cast<int64_t>(_params.mag_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	// limit data rate to prevent data being lost
	if (time_us >= static_cast<int64_t>(_mag_buffer->get_newest().time_us + _min_obs_interval_us)) {

		magSample mag_sample_new{mag_sample};
		mag_sample_new.time_us = time_us;

		_mag_buffer->push(mag_sample_new);
		_time_last_mag_buffer_push = _time_latest_us;

	} else {
		ECL_WARN("mag data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _mag_buffer->get_newest().time_us, _min_obs_interval_us);
	}
}

void EstimatorInterface::setGpsData(const gpsMessage &gps)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_gps_buffer == nullptr) {
		_gps_buffer = new RingBuffer<gpsSample>(_obs_buffer_length);

		if (_gps_buffer == nullptr || !_gps_buffer->valid()) {
			delete _gps_buffer;
			_gps_buffer = nullptr;
			printBufferAllocationFailed("GPS");
			return;
		}
	}

	const int64_t time_us = gps.time_usec
				- static_cast<int64_t>(_params.gps_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	if (time_us >= static_cast<int64_t>(_gps_buffer->get_newest().time_us + _min_obs_interval_us)) {

		if (!gps.vel_ned_valid || (gps.fix_type == 0)) {
			return;
		}

		gpsSample gps_sample_new;

		gps_sample_new.time_us = time_us;

		gps_sample_new.vel = gps.vel_ned;

		gps_sample_new.sacc = gps.sacc;
		gps_sample_new.hacc = gps.eph;
		gps_sample_new.vacc = gps.epv;

		gps_sample_new.hgt = (float)gps.alt * 1e-3f;

#if defined(CONFIG_EKF2_GNSS_YAW)

		if (PX4_ISFINITE(gps.yaw)) {
			_time_last_gps_yaw_buffer_push = _time_latest_us;
			gps_sample_new.yaw = gps.yaw;
			gps_sample_new.yaw_acc = PX4_ISFINITE(gps.yaw_accuracy) ? gps.yaw_accuracy : 0.f;

		} else {
			gps_sample_new.yaw = NAN;
			gps_sample_new.yaw_acc = 0.f;
		}

		if (PX4_ISFINITE(gps.yaw_offset)) {
			_gps_yaw_offset = gps.yaw_offset;

		} else {
			_gps_yaw_offset = 0.0f;
		}

#endif // CONFIG_EKF2_GNSS_YAW

		// Only calculate the relative position if the WGS-84 location of the origin is set
		if (collect_gps(gps)) {
			gps_sample_new.pos = _pos_ref.project((gps.lat / 1.0e7), (gps.lon / 1.0e7));

		} else {
			gps_sample_new.pos(0) = 0.0f;
			gps_sample_new.pos(1) = 0.0f;
		}

		_gps_buffer->push(gps_sample_new);
		_time_last_gps_buffer_push = _time_latest_us;


	} else {
		ECL_WARN("GPS data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _gps_buffer->get_newest().time_us, _min_obs_interval_us);
	}
}

void EstimatorInterface::setBaroData(const baroSample &baro_sample)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_baro_buffer == nullptr) {
		_baro_buffer = new RingBuffer<baroSample>(_obs_buffer_length);

		if (_baro_buffer == nullptr || !_baro_buffer->valid()) {
			delete _baro_buffer;
			_baro_buffer = nullptr;
			printBufferAllocationFailed("baro");
			return;
		}
	}

	const int64_t time_us = baro_sample.time_us
				- static_cast<int64_t>(_params.baro_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	// limit data rate to prevent data being lost
	if (time_us >= static_cast<int64_t>(_baro_buffer->get_newest().time_us + _min_obs_interval_us)) {

		baroSample baro_sample_new{baro_sample};
		baro_sample_new.time_us = time_us;

		_baro_buffer->push(baro_sample_new);
		_time_last_baro_buffer_push = _time_latest_us;

	} else {
		ECL_WARN("baro data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _baro_buffer->get_newest().time_us, _min_obs_interval_us);
	}
}

#if defined(CONFIG_EKF2_AIRSPEED)
void EstimatorInterface::setAirspeedData(const airspeedSample &airspeed_sample)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_airspeed_buffer == nullptr) {
		_airspeed_buffer = new RingBuffer<airspeedSample>(_obs_buffer_length);

		if (_airspeed_buffer == nullptr || !_airspeed_buffer->valid()) {
			delete _airspeed_buffer;
			_airspeed_buffer = nullptr;
			printBufferAllocationFailed("airspeed");
			return;
		}
	}

	const int64_t time_us = airspeed_sample.time_us
				- static_cast<int64_t>(_params.airspeed_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	// limit data rate to prevent data being lost
	if (time_us >= static_cast<int64_t>(_airspeed_buffer->get_newest().time_us + _min_obs_interval_us)) {

		airspeedSample airspeed_sample_new{airspeed_sample};
		airspeed_sample_new.time_us = time_us;

		_airspeed_buffer->push(airspeed_sample_new);

	} else {
		ECL_WARN("airspeed data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _airspeed_buffer->get_newest().time_us, _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_RANGE_FINDER)
void EstimatorInterface::setRangeData(const rangeSample &range_sample)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_range_buffer == nullptr) {
		_range_buffer = new RingBuffer<rangeSample>(_obs_buffer_length);

		if (_range_buffer == nullptr || !_range_buffer->valid()) {
			delete _range_buffer;
			_range_buffer = nullptr;
			printBufferAllocationFailed("range");
			return;
		}
	}

	const int64_t time_us = range_sample.time_us
				- static_cast<int64_t>(_params.range_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	// limit data rate to prevent data being lost
	if (time_us >= static_cast<int64_t>(_range_buffer->get_newest().time_us + _min_obs_interval_us)) {

		rangeSample range_sample_new{range_sample};
		range_sample_new.time_us = time_us;

		_range_buffer->push(range_sample_new);
		_time_last_range_buffer_push = _time_latest_us;

	} else {
		ECL_WARN("range data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _range_buffer->get_newest().time_us, _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
void EstimatorInterface::setOpticalFlowData(const flowSample &flow)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_flow_buffer == nullptr) {
		_flow_buffer = new RingBuffer<flowSample>(_imu_buffer_length);

		if (_flow_buffer == nullptr || !_flow_buffer->valid()) {
			delete _flow_buffer;
			_flow_buffer = nullptr;
			printBufferAllocationFailed("flow");
			return;
		}
	}

	const int64_t time_us = flow.time_us
				- static_cast<int64_t>(_params.flow_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	// limit data rate to prevent data being lost
	if (time_us >= static_cast<int64_t>(_flow_buffer->get_newest().time_us + _min_obs_interval_us)) {

		flowSample optflow_sample_new{flow};
		optflow_sample_new.time_us = time_us;

		_flow_buffer->push(optflow_sample_new);

	} else {
		ECL_WARN("optical flow data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _flow_buffer->get_newest().time_us, _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
void EstimatorInterface::setExtVisionData(const extVisionSample &evdata)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_ext_vision_buffer == nullptr) {
		_ext_vision_buffer = new RingBuffer<extVisionSample>(_obs_buffer_length);

		if (_ext_vision_buffer == nullptr || !_ext_vision_buffer->valid()) {
			delete _ext_vision_buffer;
			_ext_vision_buffer = nullptr;
			printBufferAllocationFailed("vision");
			return;
		}
	}

	// calculate the system time-stamp for the mid point of the integration period
	const int64_t time_us = evdata.time_us
				- static_cast<int64_t>(_params.ev_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	// limit data rate to prevent data being lost
	if (time_us >= static_cast<int64_t>(_ext_vision_buffer->get_newest().time_us + _min_obs_interval_us)) {

		extVisionSample ev_sample_new{evdata};
		ev_sample_new.time_us = time_us;

		_ext_vision_buffer->push(ev_sample_new);
		_time_last_ext_vision_buffer_push = _time_latest_us;

	} else {
		ECL_WARN("EV data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _ext_vision_buffer->get_newest().time_us, _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_AUXVEL)
void EstimatorInterface::setAuxVelData(const auxVelSample &auxvel_sample)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_auxvel_buffer == nullptr) {
		_auxvel_buffer = new RingBuffer<auxVelSample>(_obs_buffer_length);

		if (_auxvel_buffer == nullptr || !_auxvel_buffer->valid()) {
			delete _auxvel_buffer;
			_auxvel_buffer = nullptr;
			printBufferAllocationFailed("aux vel");
			return;
		}
	}

	const int64_t time_us = auxvel_sample.time_us
				- static_cast<int64_t>(_params.auxvel_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	// limit data rate to prevent data being lost
	if (time_us >= static_cast<int64_t>(_auxvel_buffer->get_newest().time_us + _min_obs_interval_us)) {

		auxVelSample auxvel_sample_new{auxvel_sample};
		auxvel_sample_new.time_us = time_us;

		_auxvel_buffer->push(auxvel_sample_new);

	} else {
		ECL_WARN("aux velocity data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _auxvel_buffer->get_newest().time_us, _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_AUXVEL

void EstimatorInterface::setSystemFlagData(const systemFlagUpdate &system_flags)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_system_flag_buffer == nullptr) {
		_system_flag_buffer = new RingBuffer<systemFlagUpdate>(_obs_buffer_length);

		if (_system_flag_buffer == nullptr || !_system_flag_buffer->valid()) {
			delete _system_flag_buffer;
			_system_flag_buffer = nullptr;
			printBufferAllocationFailed("system flag");
			return;
		}
	}

	const int64_t time_us = system_flags.time_us
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	// limit data rate to prevent data being lost
	if (time_us >= static_cast<int64_t>(_system_flag_buffer->get_newest().time_us + _min_obs_interval_us)) {

		systemFlagUpdate system_flags_new{system_flags};
		system_flags_new.time_us = time_us;

		_system_flag_buffer->push(system_flags_new);

	} else {
		ECL_WARN("system flag update too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _system_flag_buffer->get_newest().time_us, _min_obs_interval_us);
	}
}

#if defined(CONFIG_EKF2_DRAG_FUSION)
void EstimatorInterface::setDragData(const imuSample &imu)
{
	// down-sample the drag specific force data by accumulating and calculating the mean when
	// sufficient samples have been collected
	if (_params.drag_ctrl > 0) {

		// Allocate the required buffer size if not previously done
		if (_drag_buffer == nullptr) {
			_drag_buffer = new RingBuffer<dragSample>(_obs_buffer_length);

			if (_drag_buffer == nullptr || !_drag_buffer->valid()) {
				delete _drag_buffer;
				_drag_buffer = nullptr;
				printBufferAllocationFailed("drag");
				return;
			}
		}

		_drag_sample_count++;
		// note acceleration is accumulated as a delta velocity
		_drag_down_sampled.accelXY(0) += imu.delta_vel(0);
		_drag_down_sampled.accelXY(1) += imu.delta_vel(1);
		_drag_down_sampled.time_us += imu.time_us;
		_drag_sample_time_dt += imu.delta_vel_dt;

		// calculate the downsample ratio for drag specific force data
		uint8_t min_sample_ratio = (uint8_t) ceilf((float)_imu_buffer_length / _obs_buffer_length);

		if (min_sample_ratio < 5) {
			min_sample_ratio = 5;
		}

		// calculate and store means from accumulated values
		if (_drag_sample_count >= min_sample_ratio) {
			// note conversion from accumulated delta velocity to acceleration
			_drag_down_sampled.accelXY(0) /= _drag_sample_time_dt;
			_drag_down_sampled.accelXY(1) /= _drag_sample_time_dt;
			_drag_down_sampled.time_us /= _drag_sample_count;

			// write to buffer
			_drag_buffer->push(_drag_down_sampled);

			// reset accumulators
			_drag_sample_count = 0;
			_drag_down_sampled.accelXY.zero();
			_drag_down_sampled.time_us = 0;
			_drag_sample_time_dt = 0.0f;
		}
	}
}
#endif // CONFIG_EKF2_DRAG_FUSION

bool EstimatorInterface::initialise_interface(uint64_t timestamp)
{
	// find the maximum time delay the buffers are required to handle

	// it's reasonable to assume that aux velocity device has low delay. TODO: check the delay only if the aux device is used
	float max_time_delay_ms = _params.sensor_interval_max_ms;

	// aux vel
#if defined(CONFIG_EKF2_AUXVEL)
	max_time_delay_ms = math::max(_params.auxvel_delay_ms, max_time_delay_ms);
#endif // CONFIG_EKF2_AUXVEL

	// using baro
	if (_params.baro_ctrl > 0) {
		max_time_delay_ms = math::max(_params.baro_delay_ms, max_time_delay_ms);
	}

#if defined(CONFIG_EKF2_AIRSPEED)
	// using airspeed
	if (_params.arsp_thr > FLT_EPSILON) {
		max_time_delay_ms = math::max(_params.airspeed_delay_ms, max_time_delay_ms);
	}
#endif // CONFIG_EKF2_AIRSPEED

	// mag mode
	if (_params.mag_fusion_type != MagFuseType::NONE) {
		max_time_delay_ms = math::max(_params.mag_delay_ms, max_time_delay_ms);
	}

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// using range finder
	if ((_params.rng_ctrl != RngCtrl::DISABLED)) {
		max_time_delay_ms = math::max(_params.range_delay_ms, max_time_delay_ms);
	}
#endif // CONFIG_EKF2_RANGE_FINDER

	if (_params.gnss_ctrl > 0) {
		max_time_delay_ms = math::max(_params.gps_delay_ms, max_time_delay_ms);
	}

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	if (_params.flow_ctrl > 0) {
		max_time_delay_ms = math::max(_params.flow_delay_ms, max_time_delay_ms);
	}
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	if (_params.ev_ctrl > 0) {
		max_time_delay_ms = math::max(_params.ev_delay_ms, max_time_delay_ms);
	}
#endif // CONFIG_EKF2_EXTERNAL_VISION

	const float filter_update_period_ms = _params.filter_update_interval_us / 1000.f;

	// calculate the IMU buffer length required to accomodate the maximum delay with some allowance for jitter
	_imu_buffer_length = math::max(2, (int)ceilf(max_time_delay_ms / filter_update_period_ms));

	// set the observation buffer length to handle the minimum time of arrival between observations in combination
	// with the worst case delay from current time to ekf fusion time
	// allow for worst case 50% extension of the ekf fusion time horizon delay due to timing jitter
	const float ekf_delay_ms = max_time_delay_ms * 1.5f;
	_obs_buffer_length = roundf(ekf_delay_ms / filter_update_period_ms);

	// limit to be no longer than the IMU buffer (we can't process data faster than the EKF prediction rate)
	_obs_buffer_length = math::min(_obs_buffer_length, _imu_buffer_length);

	ECL_DEBUG("EKF max time delay %.1f ms, OBS length %d\n", (double)ekf_delay_ms, _obs_buffer_length);

	if (!_imu_buffer.allocate(_imu_buffer_length) || !_output_predictor.allocate(_imu_buffer_length)) {

		printBufferAllocationFailed("IMU and output");
		return false;
	}

	_time_delayed_us = timestamp;
	_time_latest_us = timestamp;

	_fault_status.value = 0;

	return true;
}

bool EstimatorInterface::isOnlyActiveSourceOfHorizontalAiding(const bool aiding_flag) const
{
	return aiding_flag && !isOtherSourceOfHorizontalAidingThan(aiding_flag);
}

bool EstimatorInterface::isOtherSourceOfHorizontalAidingThan(const bool aiding_flag) const
{
	const int nb_sources = getNumberOfActiveHorizontalAidingSources();
	return aiding_flag ? nb_sources > 1 : nb_sources > 0;
}

int EstimatorInterface::getNumberOfActiveHorizontalAidingSources() const
{
	return int(_control_status.flags.gps)
	       + int(_control_status.flags.opt_flow)
	       + int(_control_status.flags.ev_pos)
	       + int(_control_status.flags.ev_vel)
	       // Combined airspeed and sideslip fusion allows sustained wind relative dead reckoning
	       // and so is treated as a single aiding source.
	       + int(_control_status.flags.fuse_aspd && _control_status.flags.fuse_beta);
}

bool EstimatorInterface::isHorizontalAidingActive() const
{
	return getNumberOfActiveHorizontalAidingSources() > 0;
}

bool EstimatorInterface::isOtherSourceOfVerticalPositionAidingThan(const bool aiding_flag) const
{
	const int nb_sources = getNumberOfActiveVerticalPositionAidingSources();
	return aiding_flag ? nb_sources > 1 : nb_sources > 0;
}

bool EstimatorInterface::isVerticalPositionAidingActive() const
{
	return getNumberOfActiveVerticalPositionAidingSources() > 0;
}

bool EstimatorInterface::isOnlyActiveSourceOfVerticalPositionAiding(const bool aiding_flag) const
{
	return aiding_flag && !isOtherSourceOfVerticalPositionAidingThan(aiding_flag);
}

int EstimatorInterface::getNumberOfActiveVerticalPositionAidingSources() const
{
	return int(_control_status.flags.gps_hgt)
	       + int(_control_status.flags.baro_hgt)
	       + int(_control_status.flags.rng_hgt)
	       + int(_control_status.flags.ev_hgt);
}

bool EstimatorInterface::isVerticalAidingActive() const
{
	return isVerticalPositionAidingActive() || isVerticalVelocityAidingActive();
}

bool EstimatorInterface::isVerticalVelocityAidingActive() const
{
	return getNumberOfActiveVerticalVelocityAidingSources() > 0;
}

int EstimatorInterface::getNumberOfActiveVerticalVelocityAidingSources() const
{
	return int(_control_status.flags.gps)
	       + int(_control_status.flags.ev_vel);
}

void EstimatorInterface::printBufferAllocationFailed(const char *buffer_name)
{
	if (buffer_name) {
		ECL_ERR("%s buffer allocation failed", buffer_name);
	}
}

void EstimatorInterface::print_status()
{
	printf("EKF average dt: %.6f seconds\n", (double)_dt_ekf_avg);

	printf("IMU buffer: %d (%d Bytes)\n", _imu_buffer.get_length(), _imu_buffer.get_total_size());

	printf("minimum observation interval %d us\n", _min_obs_interval_us);

	if (_gps_buffer) {
		printf("gps buffer: %d/%d (%d Bytes)\n", _gps_buffer->entries(), _gps_buffer->get_length(), _gps_buffer->get_total_size());
	}

	if (_mag_buffer) {
		printf("mag buffer: %d/%d (%d Bytes)\n", _mag_buffer->entries(), _mag_buffer->get_length(), _mag_buffer->get_total_size());
	}

	if (_baro_buffer) {
		printf("baro buffer: %d/%d (%d Bytes)\n", _baro_buffer->entries(), _baro_buffer->get_length(), _baro_buffer->get_total_size());
	}

#if defined(CONFIG_EKF2_RANGE_FINDER)
	if (_range_buffer) {
		printf("range buffer: %d/%d (%d Bytes)\n", _range_buffer->entries(), _range_buffer->get_length(), _range_buffer->get_total_size());
	}
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_AIRSPEED)
	if (_airspeed_buffer) {
		printf("airspeed buffer: %d/%d (%d Bytes)\n", _airspeed_buffer->entries(), _airspeed_buffer->get_length(), _airspeed_buffer->get_total_size());
	}
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	if (_flow_buffer) {
		printf("flow buffer: %d/%d (%d Bytes)\n", _flow_buffer->entries(), _flow_buffer->get_length(), _flow_buffer->get_total_size());
	}
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	if (_ext_vision_buffer) {
		printf("vision buffer: %d/%d (%d Bytes)\n", _ext_vision_buffer->entries(), _ext_vision_buffer->get_length(), _ext_vision_buffer->get_total_size());
	}
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_DRAG_FUSION)
	if (_drag_buffer) {
		printf("drag buffer: %d/%d (%d Bytes)\n", _drag_buffer->entries(), _drag_buffer->get_length(), _drag_buffer->get_total_size());
	}
#endif // CONFIG_EKF2_DRAG_FUSION

	_output_predictor.print_status();
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file fake_height_control.cpp
 * Control functions for ekf fake height fusion
 */

#include "ekf.h"

void Ekf::controlFakeHgtFusion()
{
	auto &aid_src = _aid_src_fake_hgt;

	// If we aren't doing any aiding, fake position measurements at the last known vertical position to constrain drift
	const bool fake_hgt_data_ready = !isVerticalAidingActive()
					 && isTimedOut(aid_src.time_last_fuse, (uint64_t)2e5); // Fuse fake height at a limited rate

	if (fake_hgt_data_ready) {

		const float obs_var = sq(_params.pos_noaid_noise);
		const float innov_gate = 3.f;

		updateVerticalPositionAidSrcStatus(_time_delayed_us, _last_known_pos(2), obs_var, innov_gate, aid_src);


		const bool continuing_conditions_passing = !isVerticalAidingActive();
		const bool starting_conditions_passing = continuing_conditions_passing
				&& _vertical_velocity_deadreckon_time_exceeded
				&& _vertical_position_deadreckon_time_exceeded;

		if (_control_status.flags.fake_hgt) {
			if (continuing_conditions_passing) {

				// always protect against extreme values that could result in a NaN
				aid_src.fusion_enabled = aid_src.test_ratio < sq(100.0f / innov_gate);

				fuseVerticalPosition(aid_src);

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, (uint64_t)4e5);

				if (is_fusion_failing) {
					resetFakeHgtFusion();
				}

			} else {
				stopFakeHgtFusion();
			}

		} else {
			if (starting_conditions_passing) {
				ECL_INFO("start fake height fusion");
				_control_status.flags.fake_hgt = true;
				resetFakeHgtFusion();
			}
		}

	} else if (_control_status.flags.fake_hgt && isVerticalAidingActive()) {
		stopFakeHgtFusion();
	}
}

void Ekf::resetFakeHgtFusion()
{
	ECL_INFO("reset fake height fusion");
	_last_known_pos(2) = _state.pos(2);

	resetVerticalVelocityToZero();
	resetHeightToLastKnown();

	_aid_src_fake_hgt.time_last_fuse = _time_delayed_us;
}

void Ekf::resetHeightToLastKnown()
{
	_information_events.flags.reset_pos_to_last_known = true;
	ECL_INFO("reset height to last known (%.3f)", (double)_last_known_pos(2));
	resetVerticalPositionTo(_last_known_pos(2), sq(_params.pos_noaid_noise));
}

void Ekf::stopFakeHgtFusion()
{
	if (_control_status.flags.fake_hgt) {
		ECL_INFO("stop fake height fusion");
		_control_status.flags.fake_hgt = false;

		resetEstimatorAidStatus(_aid_src_fake_hgt);
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file fake_pos_control.cpp
 * Control functions for ekf fake position fusion
 */

#include "ekf.h"

void Ekf::controlFakePosFusion()
{
	auto &aid_src = _aid_src_fake_pos;

	// If we aren't doing any aiding, fake position measurements at the last known position to constrain drift
	// During intial tilt aligment, fake position is used to perform a "quasi-stationary" leveling of the EKF
	const bool fake_pos_data_ready = !isHorizontalAidingActive()
					 && isTimedOut(aid_src.time_last_fuse, (uint64_t)2e5); // Fuse fake position at a limited rate

	if (fake_pos_data_ready) {

		Vector2f obs_var;

		if (_control_status.flags.in_air && _control_status.flags.tilt_align) {
			obs_var(0) = obs_var(1) = sq(fmaxf(_params.pos_noaid_noise, _params.gps_pos_noise));

		} else if (!_control_status.flags.in_air && _control_status.flags.vehicle_at_rest) {
			// Accelerate tilt fine alignment by fusing more
			// aggressively when the vehicle is at rest
			obs_var(0) = obs_var(1) = sq(0.01f);

		} else {
			obs_var(0) = obs_var(1) = sq(0.5f);
		}

		const float innov_gate = 3.f;

		updateHorizontalPositionAidSrcStatus(_time_delayed_us, Vector2f(_last_known_pos), obs_var, innov_gate, aid_src);


		const bool continuing_conditions_passing = !isHorizontalAidingActive();
		const bool starting_conditions_passing = continuing_conditions_passing
				&& _horizontal_deadreckon_time_exceeded;

		if (_control_status.flags.fake_pos) {
			if (continuing_conditions_passing) {

				// always protect against extreme values that could result in a NaN
				aid_src.fusion_enabled = (aid_src.test_ratio[0] < sq(100.0f / innov_gate))
							 && (aid_src.test_ratio[1] < sq(100.0f / innov_gate));

				fuseHorizontalPosition(aid_src);

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, (uint64_t)4e5);

				if (is_fusion_failing) {
					resetFakePosFusion();
				}

			} else {
				stopFakePosFusion();
			}

		} else {
			if (starting_conditions_passing) {
				ECL_INFO("start fake position fusion");
				_control_status.flags.fake_pos = true;
				resetFakePosFusion();

				if (_control_status.flags.tilt_align) {
					// The fake position fusion is not started for initial alignement
					_warning_events.flags.stopping_navigation = true;
					ECL_WARN("stopping navigation");
				}
			}
		}

	} else if (_control_status.flags.fake_pos && isHorizontalAidingActive()) {
		stopFakePosFusion();
	}
}

void Ekf::resetFakePosFusion()
{
	ECL_INFO("reset fake position fusion");
	_last_known_pos.xy() = _state.pos.xy();

	resetHorizontalPositionToLastKnown();
	resetHorizontalVelocityToZero();

	_aid_src_fake_pos.time_last_fuse = _time_delayed_us;
}

void Ekf::stopFakePosFusion()
{
	if (_control_status.flags.fake_pos) {
		ECL_INFO("stop fake position fusion");
		_control_status.flags.fake_pos = false;

		resetEstimatorAidStatus(_aid_src_fake_pos);
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file gnss_height_control.cpp
 * Control functions for ekf GNSS height fusion
 */

#include "ekf.h"

void Ekf::controlGnssHeightFusion(const gpsSample &gps_sample)
{
	static constexpr const char *HGT_SRC_NAME = "GNSS";

	auto &aid_src = _aid_src_gnss_hgt;
	HeightBiasEstimator &bias_est = _gps_hgt_b_est;

	bias_est.predict(_dt_ekf_avg);

	if (_gps_data_ready) {

		// relax the upper observation noise limit which prevents bad GPS perturbing the position estimate
		float noise = math::max(gps_sample.vacc, 1.5f * _params.gps_pos_noise); // use 1.5 as a typical ratio of vacc/hacc

		if (!isOnlyActiveSourceOfVerticalPositionAiding(_control_status.flags.gps_hgt)) {
			// if we are not using another source of aiding, then we are reliant on the GPS
			// observations to constrain attitude errors and must limit the observation noise value.
			if (noise > _params.pos_noaid_noise) {
				noise = _params.pos_noaid_noise;
			}
		}

		const float measurement = gps_sample.hgt - getEkfGlobalOriginAltitude();
		const float measurement_var = sq(noise);

		const float innov_gate = math::max(_params.gps_pos_innov_gate, 1.f);

		const bool measurement_valid = PX4_ISFINITE(measurement) && PX4_ISFINITE(measurement_var);

		// GNSS position, vertical position GNSS measurement has opposite sign to earth z axis
		updateVerticalPositionAidSrcStatus(gps_sample.time_us,
						   -(measurement - bias_est.getBias()),
						   measurement_var + bias_est.getBiasVar(),
						   innov_gate,
						   aid_src);

		const bool gps_checks_passing = isTimedOut(_last_gps_fail_us, (uint64_t)5e6);
		const bool gps_checks_failing = isTimedOut(_last_gps_pass_us, (uint64_t)5e6);

		// update the bias estimator before updating the main filter but after
		// using its current state to compute the vertical position innovation
		if (measurement_valid && gps_checks_passing && !gps_checks_failing) {
			bias_est.setMaxStateNoise(sqrtf(measurement_var));
			bias_est.setProcessNoiseSpectralDensity(_params.gps_hgt_bias_nsd);
			bias_est.fuseBias(measurement - (-_state.pos(2)), measurement_var + P(9, 9));
		}

		// determine if we should use height aiding
		const bool continuing_conditions_passing = (_params.gnss_ctrl & GnssCtrl::VPOS)
				&& measurement_valid
				&& _NED_origin_initialised
				&& _gps_checks_passed;

		const bool starting_conditions_passing = continuing_conditions_passing
				&& isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GNSS_MAX_INTERVAL)
				&& _gps_checks_passed
				&& gps_checks_passing
				&& !gps_checks_failing;

		if (_control_status.flags.gps_hgt) {
			aid_src.fusion_enabled = true;

			if (continuing_conditions_passing) {

				fuseVerticalPosition(aid_src);

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.hgt_fusion_timeout_max);

				if (isHeightResetRequired()) {
					// All height sources are failing
					ECL_WARN("%s height fusion reset required, all height sources failing", HGT_SRC_NAME);

					_information_events.flags.reset_hgt_to_gps = true;
					resetVerticalPositionTo(-(measurement - bias_est.getBias()), measurement_var);
					bias_est.setBias(_state.pos(2) + measurement);

					// reset vertical velocity
					if (PX4_ISFINITE(gps_sample.vel(2)) && (_params.gnss_ctrl & GnssCtrl::VEL)) {
						// use 1.5 as a typical ratio of vacc/hacc
						resetVerticalVelocityTo(gps_sample.vel(2), sq(math::max(1.5f * gps_sample.sacc, _params.gps_vel_noise)));

					} else {
						resetVerticalVelocityToZero();
					}

					aid_src.time_last_fuse = _time_delayed_us;

				} else if (is_fusion_failing) {
					// Some other height source is still working
					ECL_WARN("stopping %s height fusion, fusion failing", HGT_SRC_NAME);
					stopGpsHgtFusion();
				}

			} else {
				ECL_WARN("stopping %s height fusion, continuing conditions failing", HGT_SRC_NAME);
				stopGpsHgtFusion();
			}

		} else {
			if (starting_conditions_passing) {
				if (_params.height_sensor_ref == HeightSensor::GNSS) {
					ECL_INFO("starting %s height fusion, resetting height", HGT_SRC_NAME);
					_height_sensor_ref = HeightSensor::GNSS;

					_information_events.flags.reset_hgt_to_gps = true;
					resetVerticalPositionTo(-measurement, measurement_var);
					bias_est.reset();

				} else {
					ECL_INFO("starting %s height fusion", HGT_SRC_NAME);
					bias_est.setBias(_state.pos(2) + measurement);
				}

				aid_src.time_last_fuse = _time_delayed_us;
				bias_est.setFusionActive();
				_control_status.flags.gps_hgt = true;
			}
		}

	} else if (_control_status.flags.gps_hgt
		   && !isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GNSS_MAX_INTERVAL)) {
		// No data anymore. Stop until it comes back.
		ECL_WARN("stopping %s height fusion, no data", HGT_SRC_NAME);
		stopGpsHgtFusion();
	}
}

void Ekf::stopGpsHgtFusion()
{
	if (_control_status.flags.gps_hgt) {

		if (_height_sensor_ref == HeightSensor::GNSS) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}

		_gps_hgt_b_est.setFusionInactive();
		resetEstimatorAidStatus(_aid_src_gnss_hgt);

		_control_status.flags.gps_hgt = false;
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2013 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file gps_checks.cpp
 * Perform pre-flight and in-flight GPS quality checks
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

#include <lib/world_magnetic_model/geo_mag_declination.h>
#include <mathlib/mathlib.h>

// GPS pre-flight check bit locations
#define MASK_GPS_NSATS  (1<<0)
#define MASK_GPS_PDOP   (1<<1)
#define MASK_GPS_HACC   (1<<2)
#define MASK_GPS_VACC   (1<<3)
#define MASK_GPS_SACC   (1<<4)
#define MASK_GPS_HDRIFT (1<<5)
#define MASK_GPS_VDRIFT (1<<6)
#define MASK_GPS_HSPD   (1<<7)
#define MASK_GPS_VSPD   (1<<8)

bool Ekf::collect_gps(const gpsMessage &gps)
{
	// Run GPS checks always
	_gps_checks_passed = gps_is_good(gps);

	if (_filter_initialised && !_NED_origin_initialised && _gps_checks_passed) {
		// If we have good GPS data set the origin's WGS-84 position to the last gps fix
		const double lat = gps.lat * 1.0e-7;
		const double lon = gps.lon * 1.0e-7;

		if (!_pos_ref.isInitialized()) {
			_pos_ref.initReference(lat, lon, gps.time_usec);

			// if we are already doing aiding, correct for the change in position since the EKF started navigating
			if (isHorizontalAidingActive()) {
				double est_lat;
				double est_lon;
				_pos_ref.reproject(-_state.pos(0), -_state.pos(1), est_lat, est_lon);
				_pos_ref.initReference(est_lat, est_lon, gps.time_usec);
			}
		}

		// Take the current GPS height and subtract the filter height above origin to estimate the GPS height of the origin
		if (!PX4_ISFINITE(_gps_alt_ref)) {
			_gps_alt_ref = 1e-3f * (float)gps.alt + _state.pos(2);
		}

		_NED_origin_initialised = true;

		_earth_rate_NED = calcEarthRateNED((float)math::radians(_pos_ref.getProjectionReferenceLat()));

		const bool declination_was_valid = PX4_ISFINITE(_mag_declination_gps);

		// set the magnetic field data returned by the geo library using the current GPS position
		_mag_declination_gps = get_mag_declination_radians(lat, lon);
		_mag_inclination_gps = get_mag_inclination_radians(lat, lon);
		_mag_strength_gps = get_mag_strength_gauss(lat, lon);

		// request a reset of the yaw using the new declination
		if ((_params.mag_fusion_type != MagFuseType::NONE)
		     && !declination_was_valid) {
			_mag_yaw_reset_req = true;
		}

		// save the horizontal and vertical position uncertainty of the origin
		_gps_origin_eph = gps.eph;
		_gps_origin_epv = gps.epv;

		_information_events.flags.gps_checks_passed = true;
		ECL_INFO("GPS checks passed");

	} else if (!_NED_origin_initialised) {
		// a rough 2D fix is still sufficient to lookup declination
		if ((gps.fix_type >= 2) && (gps.eph < 1000)) {

			const bool declination_was_valid = PX4_ISFINITE(_mag_declination_gps);

			// If we have good GPS data set the origin's WGS-84 position to the last gps fix
			const double lat = gps.lat * 1.0e-7;
			const double lon = gps.lon * 1.0e-7;

			// set the magnetic field data returned by the geo library using the current GPS position
			_mag_declination_gps = get_mag_declination_radians(lat, lon);
			_mag_inclination_gps = get_mag_inclination_radians(lat, lon);
			_mag_strength_gps = get_mag_strength_gauss(lat, lon);

			// request mag yaw reset if there's a mag declination for the first time
			if (_params.mag_fusion_type != MagFuseType::NONE) {
				if (!declination_was_valid && PX4_ISFINITE(_mag_declination_gps)) {
					_mag_yaw_reset_req = true;
				}
			}

			_earth_rate_NED = calcEarthRateNED((float)math::radians(lat));
		}
	}

	// start collecting GPS if there is a 3D fix and the NED origin has been set
	return _NED_origin_initialised && (gps.fix_type >= 3);
}

/*
 * Return true if the GPS solution quality is adequate to set an origin for the EKF
 * and start GPS aiding.
 * All activated checks must pass for 10 seconds.
 * Checks are activated using the EKF2_GPS_CHECK bitmask parameter
 * Checks are adjusted using the EKF2_REQ_* parameters
*/
bool Ekf::gps_is_good(const gpsMessage &gps)
{
	// Check the fix type
	_gps_check_fail_status.flags.fix = (gps.fix_type < 3);

	// Check the number of satellites
	_gps_check_fail_status.flags.nsats = (gps.nsats < _params.req_nsats);

	// Check the position dilution of precision
	_gps_check_fail_status.flags.pdop = (gps.pdop > _params.req_pdop);

	// Check the reported horizontal and vertical position accuracy
	_gps_check_fail_status.flags.hacc = (gps.eph > _params.req_hacc);
	_gps_check_fail_status.flags.vacc = (gps.epv > _params.req_vacc);

	// Check the reported speed accuracy
	_gps_check_fail_status.flags.sacc = (gps.sacc > _params.req_sacc);

	// check if GPS quality is degraded
	_gps_error_norm = fmaxf((gps.eph / _params.req_hacc), (gps.epv / _params.req_vacc));
	_gps_error_norm = fmaxf(_gps_error_norm, (gps.sacc / _params.req_sacc));

	// Calculate time lapsed since last update, limit to prevent numerical errors and calculate a lowpass filter coefficient
	constexpr float filt_time_const = 10.0f;
	const float dt = math::constrain(float(int64_t(gps.time_usec) - int64_t(_gps_pos_prev.getProjectionReferenceTimestamp())) * 1e-6f, 0.001f, filt_time_const);
	const float filter_coef = dt / filt_time_const;

	// The following checks are only valid when the vehicle is at rest
	const double lat = gps.lat * 1.0e-7;
	const double lon = gps.lon * 1.0e-7;

	if (!_control_status.flags.in_air && _control_status.flags.vehicle_at_rest) {
		// Calculate position movement since last measurement
		float delta_pos_n = 0.0f;
		float delta_pos_e = 0.0f;

		// calculate position movement since last GPS fix
		if (_gps_pos_prev.getProjectionReferenceTimestamp() > 0) {
			_gps_pos_prev.project(lat, lon, delta_pos_n, delta_pos_e);

		} else {
			// no previous position has been set
			_gps_pos_prev.initReference(lat, lon, gps.time_usec);
			_gps_alt_prev = 1e-3f * (float)gps.alt;
		}

		// Calculate the horizontal and vertical drift velocity components and limit to 10x the threshold
		const Vector3f vel_limit(_params.req_hdrift, _params.req_hdrift, _params.req_vdrift);
		Vector3f pos_derived(delta_pos_n, delta_pos_e, (_gps_alt_prev - 1e-3f * (float)gps.alt));
		pos_derived = matrix::constrain(pos_derived / dt, -10.0f * vel_limit, 10.0f * vel_limit);

		// Apply a low pass filter
		_gps_pos_deriv_filt = pos_derived * filter_coef + _gps_pos_deriv_filt * (1.0f - filter_coef);

		// Calculate the horizontal drift speed and fail if too high
		_gps_horizontal_position_drift_rate_m_s = Vector2f(_gps_pos_deriv_filt.xy()).norm();
		_gps_check_fail_status.flags.hdrift = (_gps_horizontal_position_drift_rate_m_s > _params.req_hdrift);

		// Fail if the vertical drift speed is too high
		_gps_vertical_position_drift_rate_m_s = fabsf(_gps_pos_deriv_filt(2));
		_gps_check_fail_status.flags.vdrift = (_gps_vertical_position_drift_rate_m_s > _params.req_vdrift);

		// Check the magnitude of the filtered horizontal GPS velocity
		const Vector2f gps_velNE = matrix::constrain(Vector2f(gps.vel_ned.xy()),
					   -10.0f * _params.req_hdrift,
					   10.0f * _params.req_hdrift);
		_gps_velNE_filt = gps_velNE * filter_coef + _gps_velNE_filt * (1.0f - filter_coef);
		_gps_filtered_horizontal_velocity_m_s = _gps_velNE_filt.norm();
		_gps_check_fail_status.flags.hspeed = (_gps_filtered_horizontal_velocity_m_s > _params.req_hdrift);

	} else if (_control_status.flags.in_air) {
		// These checks are always declared as passed when flying
		// If on ground and moving, the last result before movement commenced is kept
		_gps_check_fail_status.flags.hdrift = false;
		_gps_check_fail_status.flags.vdrift = false;
		_gps_check_fail_status.flags.hspeed = false;

		resetGpsDriftCheckFilters();

	} else {
		// This is the case where the vehicle is on ground and IMU movement is blocking the drift calculation
		resetGpsDriftCheckFilters();
	}

	// save GPS fix for next time
	_gps_pos_prev.initReference(lat, lon, gps.time_usec);
	_gps_alt_prev = 1e-3f * (float)gps.alt;

	// Check  the filtered difference between GPS and EKF vertical velocity
	const float vz_diff_limit = 10.0f * _params.req_vdrift;
	const float vertVel = math::constrain(gps.vel_ned(2) - _state.vel(2), -vz_diff_limit, vz_diff_limit);
	_gps_velD_diff_filt = vertVel * filter_coef + _gps_velD_diff_filt * (1.0f - filter_coef);
	_gps_check_fail_status.flags.vspeed = (fabsf(_gps_velD_diff_filt) > _params.req_vdrift);

	// assume failed first time through
	if (_last_gps_fail_us == 0) {
		_last_gps_fail_us = _time_delayed_us;
	}

	// if any user selected checks have failed, record the fail time
	if (
		_gps_check_fail_status.flags.fix ||
		(_gps_check_fail_status.flags.nsats   && (_params.gps_check_mask & MASK_GPS_NSATS)) ||
		(_gps_check_fail_status.flags.pdop    && (_params.gps_check_mask & MASK_GPS_PDOP)) ||
		(_gps_check_fail_status.flags.hacc    && (_params.gps_check_mask & MASK_GPS_HACC)) ||
		(_gps_check_fail_status.flags.vacc    && (_params.gps_check_mask & MASK_GPS_VACC)) ||
		(_gps_check_fail_status.flags.sacc    && (_params.gps_check_mask & MASK_GPS_SACC)) ||
		(_gps_check_fail_status.flags.hdrift  && (_params.gps_check_mask & MASK_GPS_HDRIFT)) ||
		(_gps_check_fail_status.flags.vdrift  && (_params.gps_check_mask & MASK_GPS_VDRIFT)) ||
		(_gps_check_fail_status.flags.hspeed  && (_params.gps_check_mask & MASK_GPS_HSPD)) ||
		(_gps_check_fail_status.flags.vspeed  && (_params.gps_check_mask & MASK_GPS_VSPD))
	) {
		_last_gps_fail_us = _time_delayed_us;

	} else {
		_last_gps_pass_us = _time_delayed_us;
	}

	// continuous period without fail of x seconds required to return a healthy status
	return isTimedOut(_last_gps_fail_us, (uint64_t)_min_gps_health_time_us);
}
/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file gps_control.cpp
 * Control functions for ekf GNSS fusion
 */

#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::controlGpsFusion(const imuSample &imu_delayed)
{
	if (!_gps_buffer || !((_params.gnss_ctrl & GnssCtrl::HPOS) || (_params.gnss_ctrl & GnssCtrl::VEL))) {
		stopGpsFusion();
		return;
	}

	_gps_intermittent = !isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GNSS_MAX_INTERVAL);

	// check for arrival of new sensor data at the fusion time horizon
	_gps_data_ready = _gps_buffer->pop_first_older_than(imu_delayed.time_us, &_gps_sample_delayed);

	if (_gps_data_ready) {
		// correct velocity for offset relative to IMU
		const Vector3f pos_offset_body = _params.gps_pos_body - _params.imu_pos_body;
		const Vector3f vel_offset_body = _ang_rate_delayed_raw % pos_offset_body;
		const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;
		_gps_sample_delayed.vel -= vel_offset_earth;

		// correct position and height for offset relative to IMU
		const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
		_gps_sample_delayed.pos -= pos_offset_earth.xy();
		_gps_sample_delayed.hgt += pos_offset_earth(2);

		// update GSF yaw estimator velocity (basic sanity check on GNSS velocity data)
		if ((_gps_sample_delayed.sacc > 0.f) && (_gps_sample_delayed.sacc < _params.req_sacc)
		    && _gps_sample_delayed.vel.isAllFinite()
		   ) {
			_yawEstimator.setVelocity(_gps_sample_delayed.vel.xy(), math::max(_gps_sample_delayed.sacc, _params.gps_vel_noise));
		}
	}

	// run EKF-GSF yaw estimator once per imu_delayed update after all main EKF data samples available
	_yawEstimator.update(imu_delayed, _control_status.flags.in_air, getGyroBias());

	// Check for new GPS data that has fallen behind the fusion time horizon
	if (_gps_data_ready) {

		const gpsSample &gps_sample{_gps_sample_delayed};

		const bool gps_checks_passing = isTimedOut(_last_gps_fail_us, (uint64_t)5e6);
		const bool gps_checks_failing = isTimedOut(_last_gps_pass_us, (uint64_t)5e6);

#if defined(CONFIG_EKF2_GNSS_YAW)
		controlGpsYawFusion(gps_sample, gps_checks_passing, gps_checks_failing);
#endif // CONFIG_EKF2_GNSS_YAW

		// GNSS velocity
		const Vector3f velocity{gps_sample.vel};
		const float vel_var = sq(math::max(gps_sample.sacc, _params.gps_vel_noise));
		const Vector3f vel_obs_var(vel_var, vel_var, vel_var * sq(1.5f));
		updateVelocityAidSrcStatus(gps_sample.time_us,
					   velocity,                                                   // observation
					   vel_obs_var,                                                // observation variance
					   math::max(_params.gps_vel_innov_gate, 1.f),                 // innovation gate
					   _aid_src_gnss_vel);
		_aid_src_gnss_vel.fusion_enabled = (_params.gnss_ctrl & GnssCtrl::VEL);

		// GNSS position
		const Vector2f position{gps_sample.pos};
		// relax the upper observation noise limit which prevents bad GPS perturbing the position estimate
		float pos_noise = math::max(gps_sample.hacc, _params.gps_pos_noise);

		if (!isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {
			// if we are not using another source of aiding, then we are reliant on the GPS
			// observations to constrain attitude errors and must limit the observation noise value.
			if (pos_noise > _params.pos_noaid_noise) {
				pos_noise = _params.pos_noaid_noise;
			}
		}

		const float pos_var = sq(pos_noise);
		const Vector2f pos_obs_var(pos_var, pos_var);
		updateHorizontalPositionAidSrcStatus(gps_sample.time_us,
						     position,                                   // observation
						     pos_obs_var,                                // observation variance
						     math::max(_params.gps_pos_innov_gate, 1.f), // innovation gate
						     _aid_src_gnss_pos);
		_aid_src_gnss_pos.fusion_enabled = (_params.gnss_ctrl & GnssCtrl::HPOS);

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
		// if GPS is otherwise ready to go, but yaw_align is blocked by EV give mag a chance to start
		if (_control_status.flags.tilt_align && _NED_origin_initialised
		    && gps_checks_passing && !gps_checks_failing) {

			if (!_control_status.flags.yaw_align) {
				if (_control_status.flags.ev_yaw && !_control_status.flags.yaw_align) {

					// give mag a chance to start and yaw align if currently blocked by EV yaw
					const bool mag_enabled = (_params.mag_fusion_type <= MagFuseType::MAG_3D);
					const bool mag_available = (_mag_counter != 0);

					if (mag_enabled && mag_available
					    && !_control_status.flags.mag_field_disturbed
					    && !_control_status.flags.mag_fault) {

						stopEvYawFusion();
					}
				}
			}
		}
#endif // CONFIG_EKF2_EXTERNAL_VISION

		// Determine if we should use GPS aiding for velocity and horizontal position
		// To start using GPS we need angular alignment completed, the local NED origin set and GPS data that has not failed checks recently
		const bool mandatory_conditions_passing = ((_params.gnss_ctrl & GnssCtrl::HPOS) || (_params.gnss_ctrl & GnssCtrl::VEL))
				&& _control_status.flags.tilt_align
				&& _control_status.flags.yaw_align
				&& _NED_origin_initialised;

		const bool continuing_conditions_passing = mandatory_conditions_passing && !gps_checks_failing;
		const bool starting_conditions_passing = continuing_conditions_passing && gps_checks_passing;

		if (_control_status.flags.gps) {
			if (mandatory_conditions_passing) {
				if (continuing_conditions_passing
				    || !isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {

					fuseVelocity(_aid_src_gnss_vel);
					fuseHorizontalPosition(_aid_src_gnss_pos);

					bool do_vel_pos_reset = shouldResetGpsFusion();

					if (isYawFailure()
					    && _control_status.flags.in_air
					    && isTimedOut(_time_last_hor_vel_fuse, _params.EKFGSF_reset_delay)
					    && (_time_last_hor_vel_fuse > _time_last_on_ground_us)) {
						/* A rapid reset to the yaw emergency estimate is performed if horizontal velocity innovation checks continuously
						 * fails while the difference between the yaw emergency estimator and the yas estimate is large.
						 * This enables recovery from a bad yaw estimate. A reset is not performed if the fault condition was
						 * present before flight to prevent triggering due to GPS glitches or other sensor errors.
						 */
						if (resetYawToEKFGSF()) {
							ECL_WARN("GPS emergency yaw reset");
							do_vel_pos_reset = true;
						}
					}

					if (do_vel_pos_reset) {
						ECL_WARN("GPS fusion timeout, resetting velocity and position");

						// reset velocity
						_information_events.flags.reset_vel_to_gps = true;
						resetVelocityTo(velocity, vel_obs_var);
						_aid_src_gnss_vel.time_last_fuse = _time_delayed_us;

						// reset position
						_information_events.flags.reset_pos_to_gps = true;
						resetHorizontalPositionTo(position, pos_obs_var);
						_aid_src_gnss_pos.time_last_fuse = _time_delayed_us;
					}

				} else {
					stopGpsFusion();
					_warning_events.flags.gps_quality_poor = true;
					ECL_WARN("GPS quality poor - stopping use");
				}

			} else { // mandatory conditions are not passing
				stopGpsFusion();
			}

		} else {
			if (starting_conditions_passing) {
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
				// Do not use external vision for yaw if using GPS because yaw needs to be
				// defined relative to an NED reference frame
				if (_control_status.flags.ev_yaw) {
					// Stop the vision for yaw fusion and do not allow it to start again
					stopEvYawFusion();
					_inhibit_ev_yaw_use = true;
				}
#endif // CONFIG_EKF2_EXTERNAL_VISION

				ECL_INFO("starting GPS fusion");
				_information_events.flags.starting_gps_fusion = true;

				// when already using another velocity source velocity reset is not necessary
				if (!isHorizontalAidingActive()
				    || isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)
				    || !_control_status_prev.flags.yaw_align
				   ) {
					// reset velocity
					_information_events.flags.reset_vel_to_gps = true;
					resetVelocityTo(velocity, vel_obs_var);
					_aid_src_gnss_vel.time_last_fuse = _time_delayed_us;
				}

				// reset position
				_information_events.flags.reset_pos_to_gps = true;
				resetHorizontalPositionTo(position, pos_obs_var);
				_aid_src_gnss_pos.time_last_fuse = _time_delayed_us;

				_control_status.flags.gps = true;

			} else if (gps_checks_passing && !_control_status.flags.yaw_align && (_params.mag_fusion_type == MagFuseType::NONE)) {
				// If no mag is used, align using the yaw estimator (if available)
				if (resetYawToEKFGSF()) {
					_information_events.flags.yaw_aligned_to_imu_gps = true;
					ECL_INFO("GPS yaw aligned using IMU, resetting vel and pos");

					// reset velocity
					_information_events.flags.reset_vel_to_gps = true;
					resetVelocityTo(velocity, vel_obs_var);
					_aid_src_gnss_vel.time_last_fuse = _time_delayed_us;

					// reset position
					_information_events.flags.reset_pos_to_gps = true;
					resetHorizontalPositionTo(position, pos_obs_var);
					_aid_src_gnss_pos.time_last_fuse = _time_delayed_us;
				}
			}
		}

	} else if (_control_status.flags.gps && !isNewestSampleRecent(_time_last_gps_buffer_push, (uint64_t)10e6)) {
		stopGpsFusion();
		_warning_events.flags.gps_data_stopped = true;
		ECL_WARN("GPS data stopped");

	}  else if (_control_status.flags.gps && !isNewestSampleRecent(_time_last_gps_buffer_push, (uint64_t)1e6)
		    && isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {

		// Handle the case where we are fusing another position source along GPS,
		// stop waiting for GPS after 1 s of lost signal
		stopGpsFusion();
		_warning_events.flags.gps_data_stopped_using_alternate = true;
		ECL_WARN("GPS data stopped, using only EV, OF or air data");
	}
}

bool Ekf::shouldResetGpsFusion() const
{
	/* We are relying on aiding to constrain drift so after a specified time
	 * with no aiding we need to do something
	 */
	bool has_horizontal_aiding_timed_out = isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)
			&& isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max);

#if defined(CONFIG_EKF2_OPTICAL_FLOW)

	if (has_horizontal_aiding_timed_out) {
		// horizontal aiding hasn't timed out if optical flow still active
		if (_control_status.flags.opt_flow && isRecent(_aid_src_optical_flow.time_last_fuse, _params.reset_timeout_max)) {
			has_horizontal_aiding_timed_out = false;
		}
	}

#endif // CONFIG_EKF2_OPTICAL_FLOW

	const bool is_reset_required = has_horizontal_aiding_timed_out
				       || isTimedOut(_time_last_hor_pos_fuse, 2 * _params.reset_timeout_max);

	const bool is_inflight_nav_failure = _control_status.flags.in_air
					     && isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)
					     && isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)
					     && (_time_last_hor_vel_fuse > _time_last_on_ground_us)
					     && (_time_last_hor_pos_fuse > _time_last_on_ground_us);

	return (is_reset_required || is_inflight_nav_failure);
}

bool Ekf::isYawFailure() const
{
	if (!isYawEmergencyEstimateAvailable()) {
		return false;
	}

	const float euler_yaw = getEulerYaw(_R_to_earth);
	const float yaw_error = wrap_pi(euler_yaw - _yawEstimator.getYaw());

	return fabsf(yaw_error) > math::radians(25.f);
}

#if defined(CONFIG_EKF2_GNSS_YAW)
void Ekf::controlGpsYawFusion(const gpsSample &gps_sample, bool gps_checks_passing, bool gps_checks_failing)
{
	if (!(_params.gnss_ctrl & GnssCtrl::YAW)
	    || _control_status.flags.gps_yaw_fault) {

		stopGpsYawFusion();
		return;
	}

	updateGpsYaw(gps_sample);

	const bool is_new_data_available = PX4_ISFINITE(gps_sample.yaw);

	if (is_new_data_available) {

		const bool continuing_conditions_passing = !gps_checks_failing;

		const bool is_gps_yaw_data_intermittent = !isNewestSampleRecent(_time_last_gps_yaw_buffer_push,
				2 * GNSS_YAW_MAX_INTERVAL);

		const bool starting_conditions_passing = continuing_conditions_passing
				&& _control_status.flags.tilt_align
				&& gps_checks_passing
				&& !is_gps_yaw_data_intermittent
				&& !_gps_intermittent;

		if (_control_status.flags.gps_yaw) {

			if (continuing_conditions_passing) {

				fuseGpsYaw();

				const bool is_fusion_failing = isTimedOut(_aid_src_gnss_yaw.time_last_fuse, _params.reset_timeout_max);

				if (is_fusion_failing) {
					if (_nb_gps_yaw_reset_available > 0) {
						// Data seems good, attempt a reset
						resetYawToGps(gps_sample.yaw);

						if (_control_status.flags.in_air) {
							_nb_gps_yaw_reset_available--;
						}

					} else if (starting_conditions_passing) {
						// Data seems good, but previous reset did not fix the issue
						// something else must be wrong, declare the sensor faulty and stop the fusion
						_control_status.flags.gps_yaw_fault = true;
						stopGpsYawFusion();

					} else {
						// A reset did not fix the issue but all the starting checks are not passing
						// This could be a temporary issue, stop the fusion without declaring the sensor faulty
						stopGpsYawFusion();
					}

					// TODO: should we give a new reset credit when the fusion does not fail for some time?
				}

			} else {
				// Stop GPS yaw fusion but do not declare it faulty
				stopGpsYawFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Try to activate GPS yaw fusion
				startGpsYawFusion(gps_sample);

				if (_control_status.flags.gps_yaw) {
					_nb_gps_yaw_reset_available = 1;
				}
			}
		}

	} else if (_control_status.flags.gps_yaw
		   && !isNewestSampleRecent(_time_last_gps_yaw_buffer_push, _params.reset_timeout_max)) {

		// No yaw data in the message anymore. Stop until it comes back.
		stopGpsYawFusion();
	}

	// Before takeoff, we do not want to continue to rely on the current heading
	// if we had to stop the fusion
	if (!_control_status.flags.in_air
	    && !_control_status.flags.gps_yaw
	    && _control_status_prev.flags.gps_yaw) {
		_control_status.flags.yaw_align = false;
	}
}

void Ekf::startGpsYawFusion(const gpsSample &gps_sample)
{
	if (!_control_status.flags.gps_yaw && resetYawToGps(gps_sample.yaw)) {
		ECL_INFO("starting GPS yaw fusion");
		_control_status.flags.yaw_align = true;
		_control_status.flags.mag_dec = false;

		stopMagHdgFusion();
		stopMag3DFusion();
		_control_status.flags.gps_yaw = true;
	}
}
#endif // CONFIG_EKF2_GNSS_YAW

void Ekf::stopGpsYawFusion()
{
#if defined(CONFIG_EKF2_GNSS_YAW)
	if (_control_status.flags.gps_yaw) {
		ECL_INFO("stopping GPS yaw fusion");
		_control_status.flags.gps_yaw = false;
		resetEstimatorAidStatus(_aid_src_gnss_yaw);
	}
#endif // CONFIG_EKF2_GNSS_YAW
}

void Ekf::stopGpsFusion()
{
	if (_control_status.flags.gps) {
		stopGpsPosFusion();
		stopGpsVelFusion();

		_control_status.flags.gps = false;
	}

	if (_control_status.flags.gps_yaw) {
		stopGpsYawFusion();
	}

	// We do not need to know the true North anymore
	// EV yaw can start again
	_inhibit_ev_yaw_use = false;
}

void Ekf::stopGpsPosFusion()
{
	if (_control_status.flags.gps) {
		ECL_INFO("stopping GPS position fusion");
		_control_status.flags.gps = false;

		resetEstimatorAidStatus(_aid_src_gnss_pos);
	}
}

void Ekf::stopGpsVelFusion()
{
	ECL_INFO("stopping GPS velocity fusion");

	resetEstimatorAidStatus(_aid_src_gnss_vel);
}
/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file gravity_fusion.cpp
 * Fuse observations from the gravity vector to constrain roll
 * and pitch (a la complementary filter).
 *
 * @author Daniel M. Sahu <danielmohansahu@gmail.com>
 */

#include "ekf.h"
#include "python/ekf_derivation/generated/compute_gravity_innov_var_and_k_and_h.h"

#include <mathlib/mathlib.h>

void Ekf::controlGravityFusion(const imuSample &imu)
{
	// fuse gravity observation if our overall acceleration isn't too big
	const float gravity_scale = _accel_vec_filt.norm() / CONSTANTS_ONE_G;

	_control_status.flags.gravity_vector = (_params.imu_ctrl & static_cast<int32_t>(ImuCtrl::GravityVector))
					       && (((gravity_scale >= 0.9f && gravity_scale <= 1.1f)) || _control_status.flags.vehicle_at_rest)
					       && !isHorizontalAidingActive();

	// get raw accelerometer reading at delayed horizon and expected measurement noise (gaussian)
	const Vector3f measurement = imu.delta_vel / imu.delta_vel_dt - getAccelBias();
	const float measurement_var = sq(_params.gravity_noise);

	// calculate kalman gains and innovation variances
	Vector3f innovation; // innovation of the last gravity fusion observation (m/s**2)
	Vector3f innovation_variance;
	Vector24f Kx, Ky, Kz; // Kalman gain vectors
	sym::ComputeGravityInnovVarAndKAndH(
		getStateAtFusionHorizonAsVector(), P, measurement, measurement_var, FLT_EPSILON,
		&innovation, &innovation_variance, &Kx, &Ky, &Kz);

	// fill estimator aid source status
	resetEstimatorAidStatus(_aid_src_gravity);
	_aid_src_gravity.timestamp_sample = imu.time_us;
	measurement.copyTo(_aid_src_gravity.observation);

	for (auto &var : _aid_src_gravity.observation_variance) {
		var = measurement_var;
	}

	innovation.copyTo(_aid_src_gravity.innovation);
	innovation_variance.copyTo(_aid_src_gravity.innovation_variance);

	float innovation_gate = 1.f;
	setEstimatorAidStatusTestRatio(_aid_src_gravity, innovation_gate);

	_aid_src_gravity.fusion_enabled = _control_status.flags.gravity_vector;

	if (_aid_src_gravity.fusion_enabled && !_aid_src_gravity.innovation_rejected) {
		// perform fusion for each axis
		_aid_src_gravity.fused = measurementUpdate(Kx, innovation_variance(0), innovation(0))
					 && measurementUpdate(Ky, innovation_variance(1), innovation(1))
					 && measurementUpdate(Kz, innovation_variance(2), innovation(2));

		if (_aid_src_gravity.fused) {
			_aid_src_gravity.time_last_fuse = imu.time_us;
		}
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file height_control.cpp
 */

#include "ekf.h"

void Ekf::controlHeightFusion(const imuSample &imu_delayed)
{
	checkVerticalAccelerationHealth(imu_delayed);

	updateGroundEffect();

	controlBaroHeightFusion();
	controlGnssHeightFusion(_gps_sample_delayed);
#if defined(CONFIG_EKF2_RANGE_FINDER)
	controlRangeHeightFusion();
#endif // CONFIG_EKF2_RANGE_FINDER

	checkHeightSensorRefFallback();
}

void Ekf::checkHeightSensorRefFallback()
{
	if (_height_sensor_ref != HeightSensor::UNKNOWN) {
		// The reference sensor is running, all good
		return;
	}

	HeightSensor fallback_list[4];

	switch (_params.height_sensor_ref) {
	default:

	/* FALLTHROUGH */
	case HeightSensor::UNKNOWN:
		fallback_list[0] = HeightSensor::GNSS;
		fallback_list[1] = HeightSensor::BARO;
		fallback_list[2] = HeightSensor::EV;
		fallback_list[3] = HeightSensor::RANGE;
		break;

	case HeightSensor::BARO:
		fallback_list[0] = HeightSensor::BARO;
		fallback_list[1] = HeightSensor::GNSS;
		fallback_list[2] = HeightSensor::EV;
		fallback_list[3] = HeightSensor::RANGE;
		break;

	case HeightSensor::GNSS:
		fallback_list[0] = HeightSensor::GNSS;
		fallback_list[1] = HeightSensor::BARO;
		fallback_list[2] = HeightSensor::EV;
		fallback_list[3] = HeightSensor::RANGE;
		break;

	case HeightSensor::RANGE:
		fallback_list[0] = HeightSensor::RANGE;
		fallback_list[1] = HeightSensor::EV;
		fallback_list[2] = HeightSensor::BARO;
		fallback_list[3] = HeightSensor::GNSS;
		break;

	case HeightSensor::EV:
		fallback_list[0] = HeightSensor::EV;
		fallback_list[1] = HeightSensor::RANGE;
		fallback_list[2] = HeightSensor::BARO;
		fallback_list[3] = HeightSensor::GNSS;
		break;
	}

	for (unsigned i = 0; i < 4; i++) {
		if (((fallback_list[i] == HeightSensor::BARO) && _control_status.flags.baro_hgt)
		    || ((fallback_list[i] == HeightSensor::GNSS) && _control_status.flags.gps_hgt)
		    || ((fallback_list[i] == HeightSensor::RANGE) && _control_status.flags.rng_hgt)
		    || ((fallback_list[i] == HeightSensor::EV) && _control_status.flags.ev_hgt)) {
			ECL_INFO("fallback to secondary height reference");
			_height_sensor_ref = fallback_list[i];
			break;
		}
	}
}

void Ekf::checkVerticalAccelerationHealth(const imuSample &imu_delayed)
{
	// Check for IMU accelerometer vibration induced clipping as evidenced by the vertical
	// innovations being positive and not stale.
	// Clipping usually causes the average accel reading to move towards zero which makes the INS
	// think it is falling and produces positive vertical innovations.

	Likelihood inertial_nav_falling_likelihood = estimateInertialNavFallingLikelihood();

	// Check for more than 50% clipping affected IMU samples within the past 1 second
	const uint16_t clip_count_limit = 1.f / _dt_ekf_avg;
	const bool is_clipping = imu_delayed.delta_vel_clipping[0] ||
				 imu_delayed.delta_vel_clipping[1] ||
				 imu_delayed.delta_vel_clipping[2];

	if (is_clipping && _clip_counter < clip_count_limit) {
		_clip_counter++;

	} else if (_clip_counter > 0) {
		_clip_counter--;
	}

	_fault_status.flags.bad_acc_clipping = _clip_counter > clip_count_limit / 2;

	const bool is_clipping_frequently = _clip_counter > 0;

	// Do not require evidence of clipping if the likelihood of having the INS falling is high
	const bool bad_vert_accel = (is_clipping_frequently && (inertial_nav_falling_likelihood == Likelihood::MEDIUM))
				    || (inertial_nav_falling_likelihood == Likelihood::HIGH);

	if (bad_vert_accel) {
		_time_bad_vert_accel = imu_delayed.time_us;

	} else {
		_time_good_vert_accel = imu_delayed.time_us;
	}

	// declare a bad vertical acceleration measurement and make the declaration persist
	// for a minimum of BADACC_PROBATION seconds
	if (_fault_status.flags.bad_acc_vertical) {
		_fault_status.flags.bad_acc_vertical = isRecent(_time_bad_vert_accel, BADACC_PROBATION);

	} else {
		_fault_status.flags.bad_acc_vertical = bad_vert_accel;
	}
}

Likelihood Ekf::estimateInertialNavFallingLikelihood() const
{
	bool likelihood_high = false;
	bool likelihood_medium = false;

	enum class ReferenceType { PRESSURE, GNSS, GROUND };

	struct {
		ReferenceType ref_type{};
		float innov{0.f};
		float innov_var{0.f};
		bool failed_min{false};
		bool failed_lim{false};
	} checks[6] {};

	if (_control_status.flags.baro_hgt) {
		checks[0] = {ReferenceType::PRESSURE, _aid_src_baro_hgt.innovation, _aid_src_baro_hgt.innovation_variance};
	}

	if (_control_status.flags.gps_hgt) {
		checks[1] = {ReferenceType::GNSS, _aid_src_gnss_hgt.innovation, _aid_src_gnss_hgt.innovation_variance};
	}

	if (_control_status.flags.gps) {
		checks[2] = {ReferenceType::GNSS, _aid_src_gnss_vel.innovation[2], _aid_src_gnss_vel.innovation_variance[2]};
	}

#if defined(CONFIG_EKF2_RANGE_FINDER)
	if (_control_status.flags.rng_hgt) {
		checks[3] = {ReferenceType::GROUND, _aid_src_rng_hgt.innovation, _aid_src_rng_hgt.innovation_variance};
	}
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	if (_control_status.flags.ev_hgt) {
		checks[4] = {ReferenceType::GROUND, _aid_src_ev_hgt.innovation, _aid_src_ev_hgt.innovation_variance};
	}

	if (_control_status.flags.ev_vel) {
		checks[5] = {ReferenceType::GROUND, _aid_src_ev_vel.innovation[2], _aid_src_ev_vel.innovation_variance[2]};
	}
#endif // CONFIG_EKF2_EXTERNAL_VISION

	// Compute the check based on innovation ratio for all the sources
	for (unsigned i = 0; i < 6; i++) {
		if (checks[i].innov_var < FLT_EPSILON) {
			continue;
		}

		const float innov_ratio = checks[i].innov / sqrtf(checks[i].innov_var);
		checks[i].failed_min = innov_ratio > _params.vert_innov_test_min;
		checks[i].failed_lim = innov_ratio > _params.vert_innov_test_lim;
	}

	// Check all the sources agains each other
	for (unsigned i = 0; i < 6; i++) {
		if (checks[i].failed_lim) {
			// There is a chance that the inertial nav is falling if one source is failing the test
			likelihood_medium = true;
		}

		for (unsigned j = 0; j < 6; j++) {

			if ((checks[i].ref_type != checks[j].ref_type) && checks[i].failed_lim && checks[j].failed_min) {
				// There is a high chance that the inertial nav is failing if two sources are failing the test
				likelihood_high = true;
			}
		}
	}

	if (likelihood_high) {
		return Likelihood::HIGH;

	} else if (likelihood_medium) {
		return Likelihood::MEDIUM;
	}

	return Likelihood::LOW;
}
#include "imu_down_sampler.hpp"

#include <lib/mathlib/mathlib.h>

ImuDownSampler::ImuDownSampler(int32_t &target_dt_us) : _target_dt_us(target_dt_us)
{
	reset();
}

// integrate imu samples until target dt reached
// assumes that dt of the gyroscope is close to the dt of the accelerometer
// returns true if target dt is reached
bool ImuDownSampler::update(const imuSample &imu_sample_new)
{
	_delta_ang_dt_avg = 0.9f * _delta_ang_dt_avg + 0.1f * imu_sample_new.delta_ang_dt;

	// accumulate time deltas
	_imu_down_sampled.time_us = imu_sample_new.time_us;
	_imu_down_sampled.delta_ang_dt += imu_sample_new.delta_ang_dt;
	_imu_down_sampled.delta_vel_dt += imu_sample_new.delta_vel_dt;
	_imu_down_sampled.delta_vel_clipping[0] |= imu_sample_new.delta_vel_clipping[0];
	_imu_down_sampled.delta_vel_clipping[1] |= imu_sample_new.delta_vel_clipping[1];
	_imu_down_sampled.delta_vel_clipping[2] |= imu_sample_new.delta_vel_clipping[2];

	// use a quaternion to accumulate delta angle data
	// this quaternion represents the rotation from the start to end of the accumulation period
	const Quatf delta_q(AxisAnglef(imu_sample_new.delta_ang));
	_delta_angle_accumulated = _delta_angle_accumulated * delta_q;
	_delta_angle_accumulated.normalize();

	// rotate the accumulated delta velocity data forward each time so it is always in the updated rotation frame
	const Dcmf delta_R(delta_q.inversed());
	_imu_down_sampled.delta_vel = delta_R * _imu_down_sampled.delta_vel;

	// accumulate the most recent delta velocity data at the updated rotation frame
	// assume effective sample time is halfway between the previous and current rotation frame
	_imu_down_sampled.delta_vel += (imu_sample_new.delta_vel + delta_R * imu_sample_new.delta_vel) * 0.5f;

	_accumulated_samples++;


	// required number of samples accumulated and the total time is at least half of the target
	//  OR total time already exceeds the target
	if ((_accumulated_samples >= _required_samples && _imu_down_sampled.delta_ang_dt > _min_dt_s)
	    || (_imu_down_sampled.delta_ang_dt > _target_dt_s)) {

		_imu_down_sampled.delta_ang = AxisAnglef(_delta_angle_accumulated);
		return true;
	}

	return false;
}

void ImuDownSampler::reset()
{
	_imu_down_sampled = {};
	_delta_angle_accumulated.setIdentity();
	_accumulated_samples = 0;

	// target dt in seconds safely constrained
	float target_dt_s = math::constrain(_target_dt_us, (int32_t)1000, (int32_t)100000) * 1e-6f;

	_required_samples = math::max((int)roundf(target_dt_s / _delta_ang_dt_avg), 1);

	_target_dt_s = _required_samples * _delta_ang_dt_avg;

	// minimum delta angle dt (in addition to number of samples)
	_min_dt_s = math::max(_delta_ang_dt_avg * (_required_samples - 1.f), _delta_ang_dt_avg * 0.5f);
}
/****************************************************************************
 *
 *   Copyright (c) 2019 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mag_control.cpp
 * Control functions for ekf magnetic field fusion
 */

#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::controlMagFusion()
{
	bool mag_data_ready = false;

	magSample mag_sample;

	if (_mag_buffer) {
		mag_data_ready = _mag_buffer->pop_first_older_than(_time_delayed_us, &mag_sample);

		if (mag_data_ready) {

			// sensor or calibration has changed, clear any mag bias and reset low pass filter
			if (mag_sample.reset || (_mag_counter == 0)) {
				// Zero the magnetometer bias states
				_state.mag_B.zero();

				// Zero the corresponding covariances and set
				// variances to the values use for initial alignment
				P.uncorrelateCovarianceSetVariance<3>(19, sq(_params.mag_noise));

				// reset any saved covariance data for re-use when auto-switching between heading and 3-axis fusion
				_saved_mag_bf_variance.zero();

				_control_status.flags.mag_fault = false;

				_mag_lpf.reset(mag_sample.mag);
				_mag_counter = 1;

			} else {
				_mag_lpf.update(mag_sample.mag);
				_mag_counter++;
			}

			// if enabled, use knowledge of theoretical magnetic field vector to calculate a synthetic magnetomter Z component value.
			// this is useful if there is a lot of interference on the sensor measurement.
			if (_params.synthesize_mag_z && (_params.mag_declination_source & GeoDeclinationMask::USE_GEO_DECL)
			    && (_NED_origin_initialised || PX4_ISFINITE(_mag_declination_gps))
			   ) {
				const Vector3f mag_earth_pred = Dcmf(Eulerf(0, -_mag_inclination_gps, _mag_declination_gps)) * Vector3f(_mag_strength_gps, 0, 0);
				mag_sample.mag(2) = calculate_synthetic_mag_z_measurement(mag_sample.mag, mag_earth_pred);
				_control_status.flags.synthetic_mag_z = true;

			} else {
				_control_status.flags.synthetic_mag_z = false;
			}

			_control_status.flags.mag_field_disturbed = magFieldStrengthDisturbed(mag_sample.mag);


			// compute mag heading innovation (for estimator_aid_src_mag_heading logging)
			const Vector3f mag_observation = mag_sample.mag - _state.mag_B;
			const Dcmf R_to_earth = updateYawInRotMat(0.f, _R_to_earth);
			const Vector3f mag_earth_pred = R_to_earth * mag_observation;

			resetEstimatorAidStatus(_aid_src_mag_heading);
			_aid_src_mag_heading.timestamp_sample = mag_sample.time_us;
			_aid_src_mag_heading.observation = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + getMagDeclination();
			_aid_src_mag_heading.innovation = wrap_pi(getEulerYaw(_R_to_earth) - _aid_src_mag_heading.observation);

			// compute magnetometer innovations (for estimator_aid_src_mag logging)
			//  rotate magnetometer earth field state into body frame
			const Vector3f mag_I_body = _state.quat_nominal.rotateVectorInverse(_state.mag_I);
			const Vector3f mag_innov = mag_I_body - mag_observation;

			resetEstimatorAidStatus(_aid_src_mag);
			_aid_src_mag.timestamp_sample = mag_sample.time_us;
			mag_observation.copyTo(_aid_src_mag.observation);
			mag_innov.copyTo(_aid_src_mag.innovation);

		} else if (!isNewestSampleRecent(_time_last_mag_buffer_push, 2 * MAG_MAX_INTERVAL)) {
			// No data anymore. Stop until it comes back.
			stopMagFusion();
		}
	}

	// If we are on ground, reset the flight alignment flag so that the mag fields will be
	// re-initialised next time we achieve flight altitude
	if (!_control_status.flags.in_air) {
		_control_status.flags.mag_aligned_in_flight = false;
	}

	if (mag_data_ready && !_control_status.flags.tilt_align && !_control_status.flags.yaw_align) {
		// calculate the initial magnetic field and yaw alignment
		// but do not mark the yaw alignement complete as it needs to be
		// reset once the leveling phase is done
		if (_params.mag_fusion_type <= MagFuseType::MAG_3D) {
			if ((_mag_counter > 1) && isTimedOut(_aid_src_mag_heading.time_last_fuse, (uint64_t)100'000)) {
				// rotate the magnetometer measurements into earth frame using a zero yaw angle
				// the angle of the projection onto the horizontal gives the yaw angle
				const Vector3f mag_earth_pred = updateYawInRotMat(0.f, _R_to_earth) * _mag_lpf.getState();
				const float yaw_new = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + getMagDeclination();

				const float yaw_prev = getEulerYaw(_R_to_earth);

				if (fabsf(yaw_new - yaw_prev) > math::radians(1.f)) {

					ECL_INFO("mag heading init %.3f -> %.3f rad (declination %.1f)", (double)yaw_prev, (double)yaw_new, (double)getMagDeclination());

					// update the rotation matrix using the new yaw value
					_R_to_earth = updateYawInRotMat(yaw_new, Dcmf(_state.quat_nominal));
					_state.quat_nominal = _R_to_earth;

					// reset the output predictor state history to match the EKF initial values
					_output_predictor.alignOutputFilter(_state.quat_nominal, _state.vel, _state.pos);

					// set the earth magnetic field states using the updated rotation
					_state.mag_I = _R_to_earth * _mag_lpf.getState();
					_state.mag_B.zero();

					_aid_src_mag_heading.time_last_fuse = _time_delayed_us;
					_time_last_heading_fuse = _time_delayed_us;

					_last_static_yaw = NAN;
				}
			}
		}

		return;
	}

	if (_params.mag_fusion_type >= MagFuseType::NONE
	    || _control_status.flags.mag_fault
	    || !_control_status.flags.tilt_align) {

		stopMagFusion();
		return;
	}

	_mag_yaw_reset_req |= !_control_status.flags.yaw_align;

	if (mag_data_ready && !_control_status.flags.ev_yaw && !_control_status.flags.gps_yaw) {

		if (shouldInhibitMag()) {
			if (uint32_t(_time_delayed_us - _mag_use_not_inhibit_us) > (uint32_t)5e6) {
				// If magnetometer use has been inhibited continuously then stop the fusion
				stopMagFusion();
			}

			return;

		} else {
			_mag_use_not_inhibit_us = _time_delayed_us;
		}

		const bool mag_enabled_previously = _control_status_prev.flags.mag_hdg || _control_status_prev.flags.mag_3D;

		// Determine if we should use simple magnetic heading fusion which works better when
		// there are large external disturbances or the more accurate 3-axis fusion
		switch (_params.mag_fusion_type) {
		default:
		// FALLTHROUGH
		case MagFuseType::AUTO:
			selectMagAuto();
			break;

		case MagFuseType::INDOOR:

		/* fallthrough */
		case MagFuseType::HEADING:
			startMagHdgFusion();
			break;

		case MagFuseType::MAG_3D:
			startMag3DFusion();
			break;
		}

		if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {

			if (_mag_yaw_reset_req || !_control_status.flags.yaw_align || mag_sample.reset || !mag_enabled_previously || haglYawResetReq()) {

				if (magReset()) {
					_mag_yaw_reset_req = false;

				} else {
					// mag reset failed, try again next time
					_mag_yaw_reset_req = true;
				}
			}
		}

		if (!_control_status.flags.yaw_align) {
			// Having the yaw aligned is mandatory to continue
			return;
		}

		checkMagDeclRequired();

		runMagAndMagDeclFusions(mag_sample.mag);
	}
}

bool Ekf::haglYawResetReq()
{
	// We need to reset the yaw angle after climbing away from the ground to enable
	// recovery from ground level magnetic interference.
	if (_control_status.flags.in_air && _control_status.flags.yaw_align && !_control_status.flags.mag_aligned_in_flight) {
		// Check if height has increased sufficiently to be away from ground magnetic anomalies
		// and request a yaw reset if not already requested.
#if defined(CONFIG_EKF2_RANGE_FINDER)
		static constexpr float mag_anomalies_max_hagl = 1.5f;
		const bool above_mag_anomalies = (getTerrainVPos() - _state.pos(2)) > mag_anomalies_max_hagl;
		return above_mag_anomalies;
#else
		return true;
#endif // CONFIG_EKF2_RANGE_FINDER
	}

	return false;
}

bool Ekf::magReset()
{
	// prevent a reset being performed more than once on the same frame
	if ((_flt_mag_align_start_time == _time_delayed_us)
	    || (_control_status_prev.flags.yaw_align != _control_status.flags.yaw_align)) {
		return false;
	}

	bool has_realigned_yaw = false;

	// use yaw estimator if available
	if (_control_status.flags.gps && isYawEmergencyEstimateAvailable()
	    && (_mag_counter > 1) // mag LPF available
	   ) {

		resetQuatStateYaw(_yawEstimator.getYaw(), _yawEstimator.getYawVar());

		_information_events.flags.yaw_aligned_to_imu_gps = true;

		// if world magnetic model (inclination, declination, strength) available then use it to reset mag states
		if (PX4_ISFINITE(_mag_inclination_gps) && PX4_ISFINITE(_mag_declination_gps) && PX4_ISFINITE(_mag_strength_gps)) {
			// use predicted earth field to reset states
			const Vector3f mag_earth_pred = Dcmf(Eulerf(0, -_mag_inclination_gps, _mag_declination_gps)) * Vector3f(_mag_strength_gps, 0, 0);
			_state.mag_I = mag_earth_pred;

			const Dcmf R_to_body = quatToInverseRotMat(_state.quat_nominal);
			_state.mag_B = _mag_lpf.getState() - (R_to_body * mag_earth_pred);

		} else {
			// Use the last magnetometer measurements to reset the field states
			// calculate initial earth magnetic field states
			_state.mag_I = _R_to_earth * _mag_lpf.getState();
			_state.mag_B.zero();
		}

		ECL_DEBUG("resetting mag I: [%.3f, %.3f, %.3f], B: [%.3f, %.3f, %.3f]",
			(double)_state.mag_I(0), (double)_state.mag_I(1), (double)_state.mag_I(2),
			(double)_state.mag_B(0), (double)_state.mag_B(1), (double)_state.mag_B(2)
			);

		resetMagCov();

		has_realigned_yaw = true;
	}

	if (!has_realigned_yaw) {
		has_realigned_yaw = resetMagHeading();
	}

	if (has_realigned_yaw) {
		_control_status.flags.yaw_align = true;

		if (_control_status.flags.in_air) {
			_control_status.flags.mag_aligned_in_flight = true;

			// record the time for the magnetic field alignment event
			_flt_mag_align_start_time = _time_delayed_us;
		}

		return true;
	}

	return false;
}

void Ekf::selectMagAuto()
{
	check3DMagFusionSuitability();
	canUse3DMagFusion() ? startMag3DFusion() : startMagHdgFusion();
}

void Ekf::check3DMagFusionSuitability()
{
	checkYawAngleObservability();
	checkMagBiasObservability();

	if (_mag_bias_observable || _yaw_angle_observable) {
		_time_last_mov_3d_mag_suitable = _time_delayed_us;
	}
}

void Ekf::checkYawAngleObservability()
{
	if (_control_status.flags.gps) {
		// Check if there has been enough change in horizontal velocity to make yaw observable
		// Apply hysteresis to check to avoid rapid toggling
		if (_yaw_angle_observable) {
			_yaw_angle_observable = _accel_lpf_NE.norm() > _params.mag_acc_gate;

		} else {
			_yaw_angle_observable = _accel_lpf_NE.norm() > _params.mag_acc_gate * 2.f;
		}

	} else {
		_yaw_angle_observable = false;
	}
}

void Ekf::checkMagBiasObservability()
{
	// check if there is enough yaw rotation to make the mag bias states observable
	if (!_mag_bias_observable && (fabsf(_yaw_rate_lpf_ef) > _params.mag_yaw_rate_gate)) {
		// initial yaw motion is detected
		_mag_bias_observable = true;

	} else if (_mag_bias_observable) {
		// require sustained yaw motion of 50% the initial yaw rate threshold
		const float yaw_dt = 1e-6f * (float)(_time_delayed_us - _time_yaw_started);
		const float min_yaw_change_req =  0.5f * _params.mag_yaw_rate_gate * yaw_dt;
		_mag_bias_observable = fabsf(_yaw_delta_ef) > min_yaw_change_req;
	}

	_yaw_delta_ef = 0.0f;
	_time_yaw_started = _time_delayed_us;
}

bool Ekf::canUse3DMagFusion() const
{
	// Use of 3D fusion requires an in-air heading alignment but it should not
	// be used when the heading and mag biases are not observable for more than 2 seconds
	return _control_status.flags.mag_aligned_in_flight
	       && ((_time_delayed_us - _time_last_mov_3d_mag_suitable) < (uint64_t)2e6);
}

void Ekf::checkMagDeclRequired()
{
	// if we are using 3-axis magnetometer fusion, but without external NE aiding,
	// then the declination must be fused as an observation to prevent long term heading drift
	// fusing declination when gps aiding is available is optional, but recommended to prevent
	// problem if the vehicle is static for extended periods of time
	const bool user_selected = (_params.mag_declination_source & GeoDeclinationMask::FUSE_DECL);
	const bool not_using_ne_aiding = !_control_status.flags.gps;
	_control_status.flags.mag_dec = (_control_status.flags.mag_3D && (not_using_ne_aiding || user_selected));
}

bool Ekf::shouldInhibitMag() const
{
	// If the user has selected auto protection against indoor magnetic field errors, only use the magnetometer
	// if a yaw angle relative to true North is required for navigation. If no GPS or other earth frame aiding
	// is available, assume that we are operating indoors and the magnetometer should not be used.
	// Also inhibit mag fusion when a strong magnetic field interference is detected or the user
	// has explicitly stopped magnetometer use.
	const bool user_selected = (_params.mag_fusion_type == MagFuseType::INDOOR);

	const bool heading_not_required_for_navigation = !_control_status.flags.gps;

	return (user_selected && heading_not_required_for_navigation) || _control_status.flags.mag_field_disturbed;
}

bool Ekf::magFieldStrengthDisturbed(const Vector3f &mag_sample) const
{
	if (_params.check_mag_strength
	    && ((_params.mag_fusion_type <= MagFuseType::MAG_3D) || (_params.mag_fusion_type == MagFuseType::INDOOR && _control_status.flags.gps))) {

		if (PX4_ISFINITE(_mag_strength_gps)) {
			constexpr float wmm_gate_size = 0.2f; // +/- Gauss
			return !isMeasuredMatchingExpected(mag_sample.length(), _mag_strength_gps, wmm_gate_size);

		} else {
			constexpr float average_earth_mag_field_strength = 0.45f; // Gauss
			constexpr float average_earth_mag_gate_size = 0.40f; // +/- Gauss
			return !isMeasuredMatchingExpected(mag_sample.length(), average_earth_mag_field_strength, average_earth_mag_gate_size);
		}
	}

	return false;
}

bool Ekf::isMeasuredMatchingExpected(const float measured, const float expected, const float gate)
{
	return (measured >= expected - gate)
	       && (measured <= expected + gate);
}

void Ekf::runMagAndMagDeclFusions(const Vector3f &mag)
{
	if (_control_status.flags.mag_3D) {
		run3DMagAndDeclFusions(mag);

	} else if (_control_status.flags.mag_hdg) {
		// Rotate the measurements into earth frame using the zero yaw angle
		Dcmf R_to_earth = updateYawInRotMat(0.f, _R_to_earth);

		Vector3f mag_earth_pred = R_to_earth * (mag - _state.mag_B);

		// the angle of the projection onto the horizontal gives the yaw angle
		// calculate the yaw innovation and wrap to the interval between +-pi
		float measured_hdg = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + getMagDeclination();

		float innovation = wrap_pi(getEulerYaw(_R_to_earth) - measured_hdg);
		float obs_var = fmaxf(sq(_params.mag_heading_noise), 1.e-4f);

		_aid_src_mag_heading.fusion_enabled = _control_status.flags.mag_hdg;

		fuseYaw(innovation, obs_var, _aid_src_mag_heading);
	}
}

void Ekf::run3DMagAndDeclFusions(const Vector3f &mag)
{
	// sanity check mag_B before they are used to constrain heading drift
	const Vector3f mag_bias_var = P.slice<3, 3>(19, 19).diag();
	const bool mag_bias_var_good = (mag_bias_var.min() > 0.f) && (mag_bias_var.max() < sq(0.02f));

	const bool update_all_states = _control_status.flags.mag_aligned_in_flight && mag_bias_var_good;

	if (!_mag_decl_cov_reset) {
		// After any magnetic field covariance reset event the earth field state
		// covariances need to be corrected to incorporate knowledge of the declination
		// before fusing magnetometer data to prevent rapid rotation of the earth field
		// states for the first few observations.
		fuseDeclination(0.02f);
		_mag_decl_cov_reset = true;
		fuseMag(mag, _aid_src_mag, update_all_states);

	} else {
		// The normal sequence is to fuse the magnetometer data first before fusing
		// declination angle at a higher uncertainty to allow some learning of
		// declination angle over time.
		fuseMag(mag, _aid_src_mag, update_all_states);

		if (_control_status.flags.mag_dec) {
			fuseDeclination(0.5f);
		}
	}
}

bool Ekf::resetMagHeading()
{
	// prevent a reset being performed more than once on the same frame
	if ((_flt_mag_align_start_time == _time_delayed_us) || (_control_status_prev.flags.yaw_align != _control_status.flags.yaw_align)) {
		return false;
	}

	const Vector3f mag_init = _mag_lpf.getState();

	const bool mag_available = (_mag_counter > 1) && !magFieldStrengthDisturbed(mag_init);

	// low pass filtered mag required
	if (!mag_available) {
		return false;
	}

	const bool heading_required_for_navigation = _control_status.flags.gps;

	if ((_params.mag_fusion_type <= MagFuseType::MAG_3D) || ((_params.mag_fusion_type == MagFuseType::INDOOR) && heading_required_for_navigation)) {

		// rotate the magnetometer measurements into earth frame using a zero yaw angle
		const Dcmf R_to_earth = updateYawInRotMat(0.f, _R_to_earth);

		// the angle of the projection onto the horizontal gives the yaw angle
		const Vector3f mag_earth_pred = R_to_earth * mag_init;

		// calculate the observed yaw angle and yaw variance
		float yaw_new = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + getMagDeclination();
		float yaw_new_variance = sq(fmaxf(_params.mag_heading_noise, 1.e-2f));

		ECL_INFO("reset mag heading %.3f -> %.3f rad (declination %.1f)", (double)getEulerYaw(_R_to_earth), (double)yaw_new, (double)getMagDeclination());

		// update quaternion states and corresponding covarainces
		resetQuatStateYaw(yaw_new, yaw_new_variance);

		// set the earth magnetic field states using the updated rotation
		_state.mag_I = _R_to_earth * mag_init;

		resetMagCov();

		// record the time for the magnetic field alignment event
		_flt_mag_align_start_time = _time_delayed_us;

		return true;
	}

	return false;
}

float Ekf::getMagDeclination()
{
	// set source of magnetic declination for internal use
	if (_control_status.flags.mag_aligned_in_flight) {
		// Use value consistent with earth field state
		return atan2f(_state.mag_I(1), _state.mag_I(0));

	} else if (_params.mag_declination_source & GeoDeclinationMask::USE_GEO_DECL) {
		// use parameter value until GPS is available, then use value returned by geo library
		if (_NED_origin_initialised || PX4_ISFINITE(_mag_declination_gps)) {
			return _mag_declination_gps;

		} else {
			return math::radians(_params.mag_declination_deg);
		}

	} else {
		// always use the parameter value
		return math::radians(_params.mag_declination_deg);
	}
}

void Ekf::stopMagFusion()
{
	if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
		ECL_INFO("stopping all mag fusion");
		stopMag3DFusion();
		stopMagHdgFusion();
		clearMagCov();
	}
}

void Ekf::stopMag3DFusion()
{
	// save covariance data for re-use if currently doing 3-axis fusion
	if (_control_status.flags.mag_3D) {
		saveMagCovData();

		_control_status.flags.mag_3D = false;
		_control_status.flags.mag_dec = false;

		_fault_status.flags.bad_mag_x = false;
		_fault_status.flags.bad_mag_y = false;
		_fault_status.flags.bad_mag_z = false;

		_fault_status.flags.bad_mag_decl = false;
	}
}

void Ekf::stopMagHdgFusion()
{
	if (_control_status.flags.mag_hdg) {
		_control_status.flags.mag_hdg = false;

		_fault_status.flags.bad_hdg = false;
	}
}

void Ekf::startMagHdgFusion()
{
	if (!_control_status.flags.mag_hdg) {
		stopMag3DFusion();
		ECL_INFO("starting mag heading fusion");
		_control_status.flags.mag_hdg = true;
	}
}

void Ekf::startMag3DFusion()
{
	if (!_control_status.flags.mag_3D) {

		stopMagHdgFusion();

		zeroMagCov();
		loadMagCovData();
		_control_status.flags.mag_3D = true;
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file heading_fusion.cpp
 * Magnetometer fusion methods.
 * Equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"
#include "python/ekf_derivation/generated/compute_mag_innov_innov_var_and_hx.h"
#include "python/ekf_derivation/generated/compute_mag_y_innov_var_and_h.h"
#include "python/ekf_derivation/generated/compute_mag_z_innov_var_and_h.h"
#include "python/ekf_derivation/generated/compute_yaw_321_innov_var_and_h.h"
#include "python/ekf_derivation/generated/compute_yaw_312_innov_var_and_h.h"
#include "python/ekf_derivation/generated/compute_mag_declination_pred_innov_var_and_h.h"

#include <mathlib/mathlib.h>

bool Ekf::fuseMag(const Vector3f &mag, estimator_aid_source3d_s &aid_src_mag, bool update_all_states)
{
	// XYZ Measurement uncertainty. Need to consider timing errors for fast rotations
	const float R_MAG = sq(fmaxf(_params.mag_noise, 0.0f));

	// calculate intermediate variables used for X axis innovation variance, observation Jacobians and Kalman gains
	const char* numerical_error_covariance_reset_string = "numerical error - covariance reset";
	Vector3f mag_innov;
	Vector3f innov_var;

	// Observation jacobian and Kalman gain vectors
	SparseVector24f<0,1,2,3,16,17,18,19,20,21> Hfusion;
	Vector24f H;
	const Vector24f state_vector = getStateAtFusionHorizonAsVector();
	sym::ComputeMagInnovInnovVarAndHx(state_vector, P, mag, R_MAG, FLT_EPSILON, &mag_innov, &innov_var, &H);
	Hfusion = H;

	innov_var.copyTo(aid_src_mag.innovation_variance);

	if (aid_src_mag.innovation_variance[0] < R_MAG) {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_mag_x = true;

		// we need to re-initialise covariances and abort this fusion step
		resetMagRelatedCovariances();
		ECL_ERR("magX %s", numerical_error_covariance_reset_string);
		return false;
	}

	_fault_status.flags.bad_mag_x = false;

	// check innovation variances for being badly conditioned
	if (aid_src_mag.innovation_variance[1] < R_MAG) {
		// the innovation variance contribution from the state covariances is negtive which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_mag_y = true;

		// we need to re-initialise covariances and abort this fusion step
		resetMagRelatedCovariances();
		ECL_ERR("magY %s", numerical_error_covariance_reset_string);
		return false;
	}

	_fault_status.flags.bad_mag_y = false;

	if (aid_src_mag.innovation_variance[2] < R_MAG) {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_mag_z = true;

		// we need to re-initialise covariances and abort this fusion step
		resetMagRelatedCovariances();
		ECL_ERR("magZ %s", numerical_error_covariance_reset_string);
		return false;
	}

	_fault_status.flags.bad_mag_z = false;

	// do not use the synthesized measurement for the magnetomter Z component for 3D fusion
	if (_control_status.flags.synthetic_mag_z) {
		mag_innov(2) = 0.0f;
	}

	for (int i = 0; i < 3; i++) {
		aid_src_mag.observation[i] = mag(i) - _state.mag_B(i);
		aid_src_mag.observation_variance[i] = R_MAG;
		aid_src_mag.innovation[i] = mag_innov(i);
	}

	aid_src_mag.fusion_enabled = _control_status.flags.mag_3D && update_all_states;

	// do not use the synthesized measurement for the magnetomter Z component for 3D fusion
	if (_control_status.flags.synthetic_mag_z) {
		aid_src_mag.innovation[2] = 0.0f;
	}

	const float innov_gate = math::max(_params.mag_innov_gate, 1.f);
	setEstimatorAidStatusTestRatio(aid_src_mag, innov_gate);

	// Perform an innovation consistency check and report the result
	_innov_check_fail_status.flags.reject_mag_x = (aid_src_mag.test_ratio[0] > 1.f);
	_innov_check_fail_status.flags.reject_mag_y = (aid_src_mag.test_ratio[1] > 1.f);
	_innov_check_fail_status.flags.reject_mag_z = (aid_src_mag.test_ratio[2] > 1.f);

	// if any axis fails, abort the mag fusion
	if (aid_src_mag.innovation_rejected) {
		return false;
	}

	bool fused[3] {false, false, false};

	// update the states and covariance using sequential fusion of the magnetometer components
	for (uint8_t index = 0; index <= 2; index++) {
		// Calculate Kalman gains and observation jacobians
		if (index == 0) {
			// everything was already computed above

		} else if (index == 1) {
			// recalculate innovation variance because state covariances have changed due to previous fusion (linearise using the same initial state for all axes)
			sym::ComputeMagYInnovVarAndH(state_vector, P, R_MAG, FLT_EPSILON, &aid_src_mag.innovation_variance[index], &H);
			Hfusion = H;

			// recalculate innovation using the updated state
			aid_src_mag.innovation[index] = _state.quat_nominal.rotateVectorInverse(_state.mag_I)(index) + _state.mag_B(index) - mag(index);

			if (aid_src_mag.innovation_variance[index] < R_MAG) {
				// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
				_fault_status.flags.bad_mag_y = true;

				// we need to re-initialise covariances and abort this fusion step
				resetMagRelatedCovariances();
				ECL_ERR("magY %s", numerical_error_covariance_reset_string);
				return false;
			}

		} else if (index == 2) {
			// we do not fuse synthesized magnetomter measurements when doing 3D fusion
			if (_control_status.flags.synthetic_mag_z) {
				continue;
			}

			// recalculate innovation variance because state covariances have changed due to previous fusion (linearise using the same initial state for all axes)
			sym::ComputeMagZInnovVarAndH(state_vector, P, R_MAG, FLT_EPSILON, &aid_src_mag.innovation_variance[index], &H);
			Hfusion = H;

			// recalculate innovation using the updated state
			aid_src_mag.innovation[index] = _state.quat_nominal.rotateVectorInverse(_state.mag_I)(index) + _state.mag_B(index) - mag(index);

			if (aid_src_mag.innovation_variance[index] < R_MAG) {
				// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
				_fault_status.flags.bad_mag_z = true;

				// we need to re-initialise covariances and abort this fusion step
				resetMagRelatedCovariances();
				ECL_ERR("magZ %s", numerical_error_covariance_reset_string);
				return false;
			}
		}

		Vector24f Kfusion = P * Hfusion / aid_src_mag.innovation_variance[index];

		if (!update_all_states) {
			for (unsigned row = 0; row <= 15; row++) {
				Kfusion(row) = 0.f;
			}

			for (unsigned row = 22; row <= 23; row++) {
				Kfusion(row) = 0.f;
			}
		}

		if (measurementUpdate(Kfusion, aid_src_mag.innovation_variance[index], aid_src_mag.innovation[index])) {
			fused[index] = true;
			limitDeclination();

		} else {
			fused[index] = false;
		}
	}

	_fault_status.flags.bad_mag_x = !fused[0];
	_fault_status.flags.bad_mag_y = !fused[1];
	_fault_status.flags.bad_mag_z = !fused[2];

	if (fused[0] && fused[1] && fused[2]) {
		aid_src_mag.fused = true;
		aid_src_mag.time_last_fuse = _time_delayed_us;
		return true;
	}

	aid_src_mag.fused = false;
	return false;
}

// update quaternion states and covariances using the yaw innovation and yaw observation variance
bool Ekf::fuseYaw(const float innovation, const float variance, estimator_aid_source1d_s& aid_src_status)
{
	Vector24f H_YAW;
	computeYawInnovVarAndH(variance, aid_src_status.innovation_variance, H_YAW);

	return fuseYaw(innovation, variance, aid_src_status, H_YAW);
}

bool Ekf::fuseYaw(const float innovation, const float variance, estimator_aid_source1d_s& aid_src_status, const Vector24f &H_YAW)
{
	aid_src_status.innovation = innovation;

	float heading_innov_var_inv = 0.f;

	// check if the innovation variance calculation is badly conditioned
	if (aid_src_status.innovation_variance >= variance) {
		// the innovation variance contribution from the state covariances is not negative, no fault
		_fault_status.flags.bad_hdg = false;
		heading_innov_var_inv = 1.f / aid_src_status.innovation_variance;

	} else {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_hdg = true;

		// we reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		ECL_ERR("yaw fusion numerical error - covariance reset");
		return false;
	}

	// calculate the Kalman gains
	// only calculate gains for states we are using
	Vector24f Kfusion;

	for (uint8_t row = 0; row < _k_num_states; row++) {
		for (uint8_t col = 0; col <= 3; col++) {
			Kfusion(row) += P(row, col) * H_YAW(col);
		}

		Kfusion(row) *= heading_innov_var_inv;
	}

	// define the innovation gate size
	float gate_sigma = math::max(_params.heading_innov_gate, 1.f);

	// innovation test ratio
	setEstimatorAidStatusTestRatio(aid_src_status, gate_sigma);

	// set the magnetometer unhealthy if the test fails
	if (aid_src_status.innovation_rejected) {
		_innov_check_fail_status.flags.reject_yaw = true;

		// if we are in air we don't want to fuse the measurement
		// we allow to use it when on the ground because the large innovation could be caused
		// by interference or a large initial gyro bias
		if (!_control_status.flags.in_air
		&& isTimedOut(_time_last_in_air, (uint64_t)5e6)
		&& isTimedOut(aid_src_status.time_last_fuse, (uint64_t)1e6)
		) {
			// constrain the innovation to the maximum set by the gate
			// we need to delay this forced fusion to avoid starting it
			// immediately after touchdown, when the drone is still armed
			float gate_limit = sqrtf((sq(gate_sigma) * aid_src_status.innovation_variance));
			aid_src_status.innovation = math::constrain(aid_src_status.innovation, -gate_limit, gate_limit);

			// also reset the yaw gyro variance to converge faster and avoid
			// being stuck on a previous bad estimate
			resetZDeltaAngBiasCov();

		} else {
			return false;
		}

	} else {
		_innov_check_fail_status.flags.reject_yaw = false;
	}

	if (measurementUpdate(Kfusion, aid_src_status.innovation_variance, aid_src_status.innovation)) {

		_time_last_heading_fuse = _time_delayed_us;

		aid_src_status.time_last_fuse = _time_delayed_us;
		aid_src_status.fused = true;

		_fault_status.flags.bad_hdg = false;

		return true;
	}

	// otherwise
	aid_src_status.fused = false;
	_fault_status.flags.bad_hdg = true;
	return false;
}

void Ekf::computeYawInnovVarAndH(float variance, float &innovation_variance, Vector24f &H_YAW) const
{
	if (shouldUse321RotationSequence(_R_to_earth)) {
		sym::ComputeYaw321InnovVarAndH(getStateAtFusionHorizonAsVector(), P, variance, FLT_EPSILON, &innovation_variance, &H_YAW);

	} else {
		sym::ComputeYaw312InnovVarAndH(getStateAtFusionHorizonAsVector(), P, variance, FLT_EPSILON, &innovation_variance, &H_YAW);
	}
}

bool Ekf::fuseDeclination(float decl_sigma)
{
	// observation variance (rad**2)
	const float R_DECL = sq(decl_sigma);

	Vector24f H;
	float decl_pred;
	float innovation_variance;

	sym::ComputeMagDeclinationPredInnovVarAndH(getStateAtFusionHorizonAsVector(), P, R_DECL, FLT_EPSILON, &decl_pred, &innovation_variance, &H);

	const float innovation = wrap_pi(decl_pred - getMagDeclination());

	if (innovation_variance < R_DECL) {
		// variance calculation is badly conditioned
		return false;
	}

	SparseVector24f<16,17> Hfusion(H);

	// Calculate the Kalman gains
	Vector24f Kfusion = P * Hfusion / innovation_variance;

	const bool is_fused = measurementUpdate(Kfusion, innovation_variance, innovation);

	_fault_status.flags.bad_mag_decl = !is_fused;

	if (is_fused) {
		limitDeclination();
	}

	return is_fused;
}

void Ekf::limitDeclination()
{
	// get a reference value for the earth field declinaton and minimum plausible horizontal field strength
	// set to 50% of the horizontal strength from geo tables if location is known
	float decl_reference;
	float h_field_min = 0.001f;

	if (_params.mag_declination_source & GeoDeclinationMask::USE_GEO_DECL) {
		// use parameter value until GPS is available, then use value returned by geo library
		if (_NED_origin_initialised || PX4_ISFINITE(_mag_declination_gps)) {
			decl_reference = _mag_declination_gps;
			h_field_min = fmaxf(h_field_min, 0.5f * _mag_strength_gps * cosf(_mag_inclination_gps));

		} else {
			decl_reference = math::radians(_params.mag_declination_deg);
		}

	} else {
		// always use the parameter value
		decl_reference = math::radians(_params.mag_declination_deg);
	}

	// do not allow the horizontal field length to collapse - this will make the declination fusion badly conditioned
	// and can result in a reversal of the NE field states which the filter cannot recover from
	// apply a circular limit
	float h_field = sqrtf(_state.mag_I(0) * _state.mag_I(0) + _state.mag_I(1) * _state.mag_I(1));

	if (h_field < h_field_min) {
		if (h_field > 0.001f * h_field_min) {
			const float h_scaler = h_field_min / h_field;
			_state.mag_I(0) *= h_scaler;
			_state.mag_I(1) *= h_scaler;

		} else {
			// too small to scale radially so set to expected value
			const float mag_declination = getMagDeclination();
			_state.mag_I(0) = 2.0f * h_field_min * cosf(mag_declination);
			_state.mag_I(1) = 2.0f * h_field_min * sinf(mag_declination);
		}

		h_field = h_field_min;
	}

	// do not allow the declination estimate to vary too much relative to the reference value
	constexpr float decl_tolerance = 0.5f;
	const float decl_max = decl_reference + decl_tolerance;
	const float decl_min = decl_reference - decl_tolerance;
	const float decl_estimate = atan2f(_state.mag_I(1), _state.mag_I(0));

	if (decl_estimate > decl_max)  {
		_state.mag_I(0) = h_field * cosf(decl_max);
		_state.mag_I(1) = h_field * sinf(decl_max);

	} else if (decl_estimate < decl_min)  {
		_state.mag_I(0) = h_field * cosf(decl_min);
		_state.mag_I(1) = h_field * sinf(decl_min);
	}
}

float Ekf::calculate_synthetic_mag_z_measurement(const Vector3f &mag_meas, const Vector3f &mag_earth_predicted)
{
	// theoretical magnitude of the magnetometer Z component value given X and Y sensor measurement and our knowledge
	// of the earth magnetic field vector at the current location
	const float mag_z_abs = sqrtf(math::max(sq(mag_earth_predicted.length()) - sq(mag_meas(0)) - sq(mag_meas(1)), 0.0f));

	// calculate sign of synthetic magnetomter Z component based on the sign of the predicted magnetometer Z component
	const float mag_z_body_pred = mag_earth_predicted.dot(_R_to_earth.col(2));

	return (mag_z_body_pred < 0) ? -mag_z_abs : mag_z_abs;
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "output_predictor.h"

using matrix::AxisAnglef;
using matrix::Dcmf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;

void OutputPredictor::print_status()
{
	printf("output predictor: IMU dt: %.4f, EKF dt: %.4f\n", (double)_dt_update_states_avg, (double)_dt_correct_states_avg);

	printf("output predictor: tracking error, angular: %.6f rad, velocity: %.3f m/s, position: %.3f m\n",
	       (double)_output_tracking_error(0), (double)_output_tracking_error(1), (double)_output_tracking_error(2));

	printf("output buffer: %d/%d (%d Bytes)\n", _output_buffer.entries(), _output_buffer.get_length(),
	       _output_buffer.get_total_size());

	printf("output vert buffer: %d/%d (%d Bytes)\n", _output_vert_buffer.entries(), _output_vert_buffer.get_length(),
	       _output_vert_buffer.get_total_size());
}

void OutputPredictor::alignOutputFilter(const Quatf &quat_state, const Vector3f &vel_state, const Vector3f &pos_state)
{
	const outputSample &output_delayed = _output_buffer.get_oldest();

	// calculate the quaternion rotation delta from the EKF to output observer states at the EKF fusion time horizon
	Quatf q_delta{quat_state * output_delayed.quat_nominal.inversed()};
	q_delta.normalize();

	// calculate the velocity and position deltas between the output and EKF at the EKF fusion time horizon
	const Vector3f vel_delta = vel_state - output_delayed.vel;
	const Vector3f pos_delta = pos_state - output_delayed.pos;

	// loop through the output filter state history and add the deltas
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		_output_buffer[i].quat_nominal = q_delta * _output_buffer[i].quat_nominal;
		_output_buffer[i].quat_nominal.normalize();
		_output_buffer[i].vel += vel_delta;
		_output_buffer[i].pos += pos_delta;
	}

	_output_new = _output_buffer.get_newest();
}

void OutputPredictor::reset()
{
	// TODO: who resets the output buffer content?
	_output_new = {};
	_output_vert_new = {};

	_accel_bias.setZero();
	_gyro_bias.setZero();

	_time_last_update_states_us = 0;
	_time_last_correct_states_us = 0;

	_R_to_earth_now.setIdentity();
	_vel_imu_rel_body_ned.setZero();
	_vel_deriv.setZero();

	_delta_angle_corr.setZero();

	_vel_err_integ.setZero();
	_pos_err_integ.setZero();

	_output_tracking_error.setZero();

	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		_output_buffer[index] = {};
	}

	for (uint8_t index = 0; index < _output_vert_buffer.get_length(); index++) {
		_output_vert_buffer[index] = {};
	}
}

void OutputPredictor::resetQuaternion(const Quatf &quat_change)
{
	// add the reset amount to the output observer buffered data
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		_output_buffer[i].quat_nominal = quat_change * _output_buffer[i].quat_nominal;
	}

	// apply the change in attitude quaternion to our newest quaternion estimate
	// which was already taken out from the output buffer
	_output_new.quat_nominal = quat_change * _output_new.quat_nominal;
}

void OutputPredictor::resetHorizontalVelocityTo(const Vector2f &delta_horz_vel)
{
	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		_output_buffer[index].vel.xy() += delta_horz_vel;
	}

	_output_new.vel.xy() += delta_horz_vel;
}

void OutputPredictor::resetVerticalVelocityTo(float delta_vert_vel)
{
	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		_output_buffer[index].vel(2) += delta_vert_vel;
		_output_vert_buffer[index].vert_vel += delta_vert_vel;
	}

	_output_new.vel(2) += delta_vert_vel;
	_output_vert_new.vert_vel += delta_vert_vel;
}

void OutputPredictor::resetHorizontalPositionTo(const Vector2f &delta_horz_pos)
{
	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		_output_buffer[index].pos.xy() += delta_horz_pos;
	}

	_output_new.pos.xy() += delta_horz_pos;
}

void OutputPredictor::resetVerticalPositionTo(const float new_vert_pos, const float vert_pos_change)
{
	// apply the change in height / height rate to our newest height / height rate estimate
	// which have already been taken out from the output buffer
	_output_new.pos(2) += vert_pos_change;

	// add the reset amount to the output observer buffered data
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		_output_buffer[i].pos(2) += vert_pos_change;
		_output_vert_buffer[i].vert_vel_integ += vert_pos_change;
	}

	// add the reset amount to the output observer vertical position state
	_output_vert_new.vert_vel_integ = new_vert_pos;
}

void OutputPredictor::calculateOutputStates(const uint64_t time_us, const Vector3f &delta_angle,
		const float delta_angle_dt, const Vector3f &delta_velocity, const float delta_velocity_dt)
{
	// Use full rate IMU data at the current time horizon
	if (_time_last_update_states_us != 0) {
		const float dt = math::constrain((time_us - _time_last_update_states_us) * 1e-6f, 0.0001f, 0.03f);
		_dt_update_states_avg = 0.8f * _dt_update_states_avg + 0.2f * dt;
	}

	_time_last_update_states_us = time_us;

	// correct delta angle and delta velocity for bias offsets
	// Apply corrections to the delta angle required to track the quaternion states at the EKF fusion time horizon
	const Vector3f delta_angle_bias_scaled = _gyro_bias * delta_angle_dt;
	const Vector3f delta_angle_corrected(delta_angle - delta_angle_bias_scaled + _delta_angle_corr);

	const Vector3f delta_vel_bias_scaled = _accel_bias * delta_velocity_dt;
	const Vector3f delta_velocity_corrected(delta_velocity - delta_vel_bias_scaled);

	_output_new.time_us = time_us;
	_output_vert_new.time_us = time_us;

	const Quatf dq(AxisAnglef{delta_angle_corrected});

	// rotate the previous INS quaternion by the delta quaternions
	_output_new.quat_nominal = _output_new.quat_nominal * dq;

	// the quaternions must always be normalised after modification
	_output_new.quat_nominal.normalize();

	// calculate the rotation matrix from body to earth frame
	_R_to_earth_now = Dcmf(_output_new.quat_nominal);

	// rotate the delta velocity to earth frame
	Vector3f delta_vel_earth{_R_to_earth_now * delta_velocity_corrected};

	// correct for measured acceleration due to gravity
	delta_vel_earth(2) += CONSTANTS_ONE_G * delta_velocity_dt;

	// calculate the earth frame velocity derivatives
	if (delta_velocity_dt > 0.001f) {
		_vel_deriv = delta_vel_earth / delta_velocity_dt;
	}

	// save the previous velocity so we can use trapezoidal integration
	const Vector3f vel_last(_output_new.vel);

	// increment the INS velocity states by the measurement plus corrections
	// do the same for vertical state used by alternative correction algorithm
	_output_new.vel += delta_vel_earth;
	_output_vert_new.vert_vel += delta_vel_earth(2);

	// use trapezoidal integration to calculate the INS position states
	// do the same for vertical state used by alternative correction algorithm
	const Vector3f delta_pos_NED = (_output_new.vel + vel_last) * (delta_velocity_dt * 0.5f);
	_output_new.pos += delta_pos_NED;
	_output_vert_new.vert_vel_integ += delta_pos_NED(2);

	// accumulate the time for each update
	_output_vert_new.dt += delta_velocity_dt;

	// correct velocity for IMU offset
	if (delta_angle_dt > 0.001f) {
		// calculate the average angular rate across the last IMU update
		const Vector3f ang_rate = delta_angle_corrected / delta_angle_dt;

		// calculate the velocity of the IMU relative to the body origin
		const Vector3f vel_imu_rel_body = ang_rate % _imu_pos_body;

		// rotate the relative velocity into earth frame
		_vel_imu_rel_body_ned = _R_to_earth_now * vel_imu_rel_body;
	}
}

void OutputPredictor::correctOutputStates(const uint64_t time_delayed_us,
		const matrix::Vector3f &gyro_bias, const matrix::Vector3f &accel_bias,
		const Quatf &quat_state, const Vector3f &vel_state, const Vector3f &pos_state)
{
	// calculate an average filter update time
	if (_time_last_correct_states_us != 0) {
		const float dt = math::constrain((time_delayed_us - _time_last_correct_states_us) * 1e-6f, 0.0001f, 0.03f);
		_dt_correct_states_avg = 0.8f * _dt_correct_states_avg + 0.2f * dt;
	}

	_time_last_correct_states_us = time_delayed_us;

	// store IMU bias for calculateOutputStates
	_gyro_bias = gyro_bias;
	_accel_bias = accel_bias;

	// store the INS states in a ring buffer with the same length and time coordinates as the IMU data buffer
	_output_buffer.push(_output_new);
	_output_vert_buffer.push(_output_vert_new);

	// get the oldest INS state data from the ring buffer
	// this data will be at the EKF fusion time horizon
	// TODO: there is no guarantee that data is at delayed fusion horizon
	//       Shouldnt we use pop_first_older_than?
	const outputSample &output_delayed = _output_buffer.get_oldest();
	const outputVert &output_vert_delayed = _output_vert_buffer.get_oldest();

	// calculate the quaternion delta between the INS and EKF quaternions at the EKF fusion time horizon
	const Quatf q_error((quat_state.inversed() * output_delayed.quat_nominal).normalized());

	// convert the quaternion delta to a delta angle
	const float scalar = (q_error(0) >= 0.0f) ? -2.f : 2.f;

	const Vector3f delta_ang_error{scalar * q_error(1), scalar * q_error(2), scalar * q_error(3)};

	// calculate a gain that provides tight tracking of the estimator attitude states and
	// adjust for changes in time delay to maintain consistent damping ratio of ~0.7
	const uint64_t time_latest_us = _time_last_update_states_us;
	const float time_delay = fmaxf((time_latest_us - time_delayed_us) * 1e-6f, _dt_update_states_avg);
	const float att_gain = 0.5f * _dt_update_states_avg / time_delay;

	// calculate a corrrection to the delta angle
	// that will cause the INS to track the EKF quaternions
	_delta_angle_corr = delta_ang_error * att_gain;
	_output_tracking_error(0) = delta_ang_error.norm();

	/*
	* Loop through the output filter state history and apply the corrections to the velocity and position states.
	* This method is too expensive to use for the attitude states due to the quaternion operations required
	* but because it eliminates the time delay in the 'correction loop' it allows higher tracking gains
	* to be used and reduces tracking error relative to EKF states.
	*/

	// Complementary filter gains
	const float vel_gain = _dt_correct_states_avg / math::constrain(_vel_tau, _dt_correct_states_avg, 10.f);
	const float pos_gain = _dt_correct_states_avg / math::constrain(_pos_tau, _dt_correct_states_avg, 10.f);

	// calculate down velocity and position tracking errors
	const float vert_vel_err = (vel_state(2) - output_vert_delayed.vert_vel);
	const float vert_vel_integ_err = (pos_state(2) - output_vert_delayed.vert_vel_integ);

	// calculate a velocity correction that will be applied to the output state history
	// using a PD feedback tuned to a 5% overshoot
	const float vert_vel_correction = vert_vel_integ_err * pos_gain + vert_vel_err * vel_gain * 1.1f;

	applyCorrectionToVerticalOutputBuffer(vert_vel_correction);

	// calculate velocity and position tracking errors
	const Vector3f vel_err(vel_state - output_delayed.vel);
	const Vector3f pos_err(pos_state - output_delayed.pos);

	_output_tracking_error(1) = vel_err.norm();
	_output_tracking_error(2) = pos_err.norm();

	// calculate a velocity correction that will be applied to the output state history
	_vel_err_integ += vel_err;
	const Vector3f vel_correction = vel_err * vel_gain + _vel_err_integ * sq(vel_gain) * 0.1f;

	// calculate a position correction that will be applied to the output state history
	_pos_err_integ += pos_err;
	const Vector3f pos_correction = pos_err * pos_gain + _pos_err_integ * sq(pos_gain) * 0.1f;

	applyCorrectionToOutputBuffer(vel_correction, pos_correction);
}

void OutputPredictor::applyCorrectionToVerticalOutputBuffer(float vert_vel_correction)
{
	// loop through the vertical output filter state history starting at the oldest and apply the corrections to the
	// vert_vel states and propagate vert_vel_integ forward using the corrected vert_vel
	uint8_t index = _output_vert_buffer.get_oldest_index();

	const uint8_t size = _output_vert_buffer.get_length();

	for (uint8_t counter = 0; counter < (size - 1); counter++) {
		const uint8_t index_next = (index + 1) % size;
		outputVert &current_state = _output_vert_buffer[index];
		outputVert &next_state = _output_vert_buffer[index_next];

		// correct the velocity
		if (counter == 0) {
			current_state.vert_vel += vert_vel_correction;
		}

		next_state.vert_vel += vert_vel_correction;

		// position is propagated forward using the corrected velocity and a trapezoidal integrator
		next_state.vert_vel_integ = current_state.vert_vel_integ + (current_state.vert_vel + next_state.vert_vel) * 0.5f * next_state.dt;

		// advance the index
		index = (index + 1) % size;
	}

	// update output state to corrected values
	_output_vert_new = _output_vert_buffer.get_newest();

	// reset time delta to zero for the next accumulation of full rate IMU data
	_output_vert_new.dt = 0.0f;
}

void OutputPredictor::applyCorrectionToOutputBuffer(const Vector3f &vel_correction, const Vector3f &pos_correction)
{
	// loop through the output filter state history and apply the corrections to the velocity and position states
	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		// a constant velocity correction is applied
		_output_buffer[index].vel += vel_correction;

		// a constant position correction is applied
		_output_buffer[index].pos += pos_correction;
	}

	// update output state to corrected values
	_output_new = _output_buffer.get_newest();
}
/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file vel_pos_fusion.cpp
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Siddharth Bharat Purohit <siddharthbharatpurohit@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include <mathlib/mathlib.h>
#include "ekf.h"

void Ekf::updateVelocityAidSrcStatus(const uint64_t &time_us, const Vector2f &obs, const Vector2f &obs_var,
				     const float innov_gate, estimator_aid_source2d_s &aid_src) const
{
	resetEstimatorAidStatus(aid_src);

	for (int i = 0; i < 2; i++) {
		aid_src.observation[i] = obs(i);
		aid_src.innovation[i] = _state.vel(i) - aid_src.observation[i];

		aid_src.observation_variance[i] = math::max(sq(0.01f), obs_var(i));
		aid_src.innovation_variance[i] = P(4 + i, 4 + i) + aid_src.observation_variance[i];
	}

	setEstimatorAidStatusTestRatio(aid_src, innov_gate);

	aid_src.timestamp_sample = time_us;
}

void Ekf::updateVelocityAidSrcStatus(const uint64_t &time_us, const Vector3f &obs, const Vector3f &obs_var,
				     const float innov_gate, estimator_aid_source3d_s &aid_src) const
{
	resetEstimatorAidStatus(aid_src);

	for (int i = 0; i < 3; i++) {
		aid_src.observation[i] = obs(i);
		aid_src.innovation[i] = _state.vel(i) - aid_src.observation[i];

		aid_src.observation_variance[i] = math::max(sq(0.01f), obs_var(i));
		aid_src.innovation_variance[i] = P(4 + i, 4 + i) + aid_src.observation_variance[i];
	}

	setEstimatorAidStatusTestRatio(aid_src, innov_gate);

	// vz special case if there is bad vertical acceleration data, then don't reject measurement,
	// but limit innovation to prevent spikes that could destabilise the filter
	if (_fault_status.flags.bad_acc_vertical && aid_src.innovation_rejected) {
		const float innov_limit = innov_gate * sqrtf(aid_src.innovation_variance[2]);
		aid_src.innovation[2] = math::constrain(aid_src.innovation[2], -innov_limit, innov_limit);
		aid_src.innovation_rejected = false;
	}

	aid_src.timestamp_sample = time_us;
}

void Ekf::updateVerticalPositionAidSrcStatus(const uint64_t &time_us, const float obs, const float obs_var,
		const float innov_gate, estimator_aid_source1d_s &aid_src) const
{
	resetEstimatorAidStatus(aid_src);

	aid_src.observation = obs;
	aid_src.innovation = _state.pos(2) - aid_src.observation;

	aid_src.observation_variance = math::max(sq(0.01f), obs_var);
	aid_src.innovation_variance = P(9, 9) + aid_src.observation_variance;

	setEstimatorAidStatusTestRatio(aid_src, innov_gate);

	// z special case if there is bad vertical acceleration data, then don't reject measurement,
	// but limit innovation to prevent spikes that could destabilise the filter
	if (_fault_status.flags.bad_acc_vertical && aid_src.innovation_rejected) {
		const float innov_limit = innov_gate * sqrtf(aid_src.innovation_variance);
		aid_src.innovation = math::constrain(aid_src.innovation, -innov_limit, innov_limit);
		aid_src.innovation_rejected = false;
	}

	aid_src.timestamp_sample = time_us;
}

void Ekf::updateHorizontalPositionAidSrcStatus(const uint64_t &time_us, const Vector2f &obs, const Vector2f &obs_var,
		const float innov_gate, estimator_aid_source2d_s &aid_src) const
{
	resetEstimatorAidStatus(aid_src);

	for (int i = 0; i < 2; i++) {
		aid_src.observation[i] = obs(i);
		aid_src.innovation[i] = _state.pos(i) - aid_src.observation[i];

		aid_src.observation_variance[i] = math::max(sq(0.01f), obs_var(i));
		aid_src.innovation_variance[i] = P(7 + i, 7 + i) + aid_src.observation_variance[i];
	}

	setEstimatorAidStatusTestRatio(aid_src, innov_gate);

	aid_src.timestamp_sample = time_us;
}

void Ekf::fuseVelocity(estimator_aid_source2d_s &aid_src)
{
	if (aid_src.fusion_enabled && !aid_src.innovation_rejected) {
		// vx, vy
		if (fuseVelPosHeight(aid_src.innovation[0], aid_src.innovation_variance[0], 0)
		    && fuseVelPosHeight(aid_src.innovation[1], aid_src.innovation_variance[1], 1)
		   ) {
			aid_src.fused = true;
			aid_src.time_last_fuse = _time_delayed_us;

		} else {
			aid_src.fused = false;
		}
	}
}

void Ekf::fuseVelocity(estimator_aid_source3d_s &aid_src)
{
	if (aid_src.fusion_enabled && !aid_src.innovation_rejected) {
		// vx, vy, vz
		if (fuseVelPosHeight(aid_src.innovation[0], aid_src.innovation_variance[0], 0)
		    && fuseVelPosHeight(aid_src.innovation[1], aid_src.innovation_variance[1], 1)
		    && fuseVelPosHeight(aid_src.innovation[2], aid_src.innovation_variance[2], 2)
		   ) {
			aid_src.fused = true;
			aid_src.time_last_fuse = _time_delayed_us;

		} else {
			aid_src.fused = false;
		}
	}
}

void Ekf::fuseHorizontalPosition(estimator_aid_source2d_s &aid_src)
{
	// x & y
	if (aid_src.fusion_enabled && !aid_src.innovation_rejected) {
		if (fuseVelPosHeight(aid_src.innovation[0], aid_src.innovation_variance[0], 3)
		    && fuseVelPosHeight(aid_src.innovation[1], aid_src.innovation_variance[1], 4)
		   ) {
			aid_src.fused = true;
			aid_src.time_last_fuse = _time_delayed_us;

		} else {
			aid_src.fused = false;
		}
	}
}

void Ekf::fuseVerticalPosition(estimator_aid_source1d_s &aid_src)
{
	// z
	if (aid_src.fusion_enabled && !aid_src.innovation_rejected) {
		if (fuseVelPosHeight(aid_src.innovation, aid_src.innovation_variance, 5)) {
			aid_src.fused = true;
			aid_src.time_last_fuse = _time_delayed_us;
		}
	}
}

// Helper function that fuses a single velocity or position measurement
bool Ekf::fuseVelPosHeight(const float innov, const float innov_var, const int obs_index)
{
	Vector24f Kfusion;  // Kalman gain vector for any single observation - sequential fusion is used.
	const unsigned state_index = obs_index + 4;  // we start with vx and this is the 4. state

	// calculate kalman gain K = PHS, where S = 1/innovation variance
	for (int row = 0; row < _k_num_states; row++) {
		Kfusion(row) = P(row, state_index) / innov_var;
	}

	clearInhibitedStateKalmanGains(Kfusion);

	SquareMatrix24f KHP;

	for (unsigned row = 0; row < _k_num_states; row++) {
		for (unsigned column = 0; column < _k_num_states; column++) {
			KHP(row, column) = Kfusion(row) * P(state_index, column);
		}
	}

	const bool healthy = checkAndFixCovarianceUpdate(KHP);

	setVelPosStatus(obs_index, healthy);

	if (healthy) {
		// apply the covariance corrections
		P -= KHP;

		fixCovarianceErrors(true);

		// apply the state corrections
		fuse(Kfusion, innov);

		return true;
	}

	return false;
}

void Ekf::setVelPosStatus(const int index, const bool healthy)
{
	switch (index) {
	case 0:
		if (healthy) {
			_fault_status.flags.bad_vel_N = false;
			_time_last_hor_vel_fuse = _time_delayed_us;

		} else {
			_fault_status.flags.bad_vel_N = true;
		}

		break;

	case 1:
		if (healthy) {
			_fault_status.flags.bad_vel_E = false;
			_time_last_hor_vel_fuse = _time_delayed_us;

		} else {
			_fault_status.flags.bad_vel_E = true;
		}

		break;

	case 2:
		if (healthy) {
			_fault_status.flags.bad_vel_D = false;
			_time_last_ver_vel_fuse = _time_delayed_us;

		} else {
			_fault_status.flags.bad_vel_D = true;
		}

		break;

	case 3:
		if (healthy) {
			_fault_status.flags.bad_pos_N = false;
			_time_last_hor_pos_fuse = _time_delayed_us;

		} else {
			_fault_status.flags.bad_pos_N = true;
		}

		break;

	case 4:
		if (healthy) {
			_fault_status.flags.bad_pos_E = false;
			_time_last_hor_pos_fuse = _time_delayed_us;

		} else {
			_fault_status.flags.bad_pos_E = true;
		}

		break;

	case 5:
		if (healthy) {
			_fault_status.flags.bad_pos_D = false;
			_time_last_hgt_fuse = _time_delayed_us;

		} else {
			_fault_status.flags.bad_pos_D = true;
		}

		break;
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file zero_innovation_heading_update.cpp
 * Control function for ekf heading update when at rest or no other heading source available
 */

#include "ekf.h"

void Ekf::controlZeroInnovationHeadingUpdate()
{
	const bool yaw_aiding = _control_status.flags.mag_hdg || _control_status.flags.mag_3D
				|| _control_status.flags.ev_yaw || _control_status.flags.gps_yaw;

	if (!_control_status.flags.tilt_align) {
		// fuse zero heading innovation during the leveling fine alignment step to keep the yaw variance low
		float innovation = 0.f;
		float obs_var = _control_status.flags.vehicle_at_rest ? 0.001f : 0.1f;
		estimator_aid_source1d_s unused;
		fuseYaw(innovation, obs_var, unused);
		_time_last_heading_fuse = 0;
		_last_static_yaw = NAN;

	} else if (_control_status.flags.vehicle_at_rest) {
		// When at rest or no source of yaw aiding is active yaw fusion is run selectively to enable yaw gyro
		// bias learning when stationary on ground and to prevent uncontrolled yaw variance growth
		const float euler_yaw = getEulerYaw(_R_to_earth);

		if (PX4_ISFINITE(_last_static_yaw)) {
			// fuse last static yaw at a limited rate (every 200 milliseconds)
			if (!yaw_aiding && isTimedOut(_time_last_heading_fuse, (uint64_t)200'000)) {
				float innovation = wrap_pi(euler_yaw - _last_static_yaw);
				float obs_var = 0.01f;
				estimator_aid_source1d_s unused;
				fuseYaw(innovation, obs_var, unused);
			}

		} else {
			// record static yaw when transitioning to at rest
			_last_static_yaw = euler_yaw;
		}

	} else {
		// vehicle moving and tilt alignment completed

		// fuse zero innovation at a limited rate if the yaw variance is too large
		if (!yaw_aiding && isTimedOut(_time_last_heading_fuse, (uint64_t)200'000)) {
			float obs_var = 0.25f;
			estimator_aid_source1d_s aid_src_status;
			Vector24f H_YAW;

			computeYawInnovVarAndH(obs_var, aid_src_status.innovation_variance, H_YAW);

			if ((aid_src_status.innovation_variance - obs_var) > sq(_params.mag_heading_noise)) {
				// The yaw variance is too large, fuse fake measurement
				float innovation = 0.f;
				fuseYaw(innovation, obs_var, aid_src_status, H_YAW);
			}
		}

		_last_static_yaw = NAN;
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file zero_velocity_update.cpp
 * Control function for ekf zero velocity update
 */

#include "ekf.h"

void Ekf::controlZeroVelocityUpdate()
{
	// Fuse zero velocity at a limited rate (every 200 milliseconds)
	const bool zero_velocity_update_data_ready = isTimedOut(_time_last_zero_velocity_fuse, (uint64_t)2e5);

	if (zero_velocity_update_data_ready) {
		const bool continuing_conditions_passing = _control_status.flags.vehicle_at_rest
				&& _control_status_prev.flags.vehicle_at_rest
				&& !isVerticalVelocityAidingActive(); // otherwise the filter is "too rigid" to follow a position drift

		if (continuing_conditions_passing) {
			Vector3f vel_obs{0, 0, 0};
			Vector3f innovation = _state.vel - vel_obs;

			// Set a low variance initially for faster leveling and higher
			// later to let the states follow the measurements
			const float obs_var = _control_status.flags.tilt_align ? sq(0.2f) : sq(0.001f);
			Vector3f innov_var{
				P(4, 4) + obs_var,
				P(5, 5) + obs_var,
				P(6, 6) + obs_var};

			fuseVelPosHeight(innovation(0), innov_var(0), 0);
			fuseVelPosHeight(innovation(1), innov_var(1), 1);
			fuseVelPosHeight(innovation(2), innov_var(2), 2);

			_time_last_zero_velocity_fuse = _time_delayed_us;
		}
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file airspeed_fusion.cpp
 * airspeed fusion methods.
 * equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Carl Olsson <carlolsson.co@gmail.com>
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

#include "python/ekf_derivation/generated/compute_airspeed_h_and_k.h"
#include "python/ekf_derivation/generated/compute_airspeed_innov_and_innov_var.h"

#include <mathlib/mathlib.h>

void Ekf::controlAirDataFusion(const imuSample &imu_delayed)
{
	// control activation and initialisation/reset of wind states required for airspeed fusion

	// If both airspeed and sideslip fusion have timed out and we are not using a drag observation model then we no longer have valid wind estimates
	const bool airspeed_timed_out = isTimedOut(_aid_src_airspeed.time_last_fuse, (uint64_t)10e6);
	const bool sideslip_timed_out = isTimedOut(_aid_src_sideslip.time_last_fuse, (uint64_t)10e6);

	if (_control_status.flags.fake_pos || (airspeed_timed_out && sideslip_timed_out && (_params.drag_ctrl == 0))) {
		_control_status.flags.wind = false;
	}

	// clear yaw estimator airspeed (updated later with true airspeed if airspeed fusion is active)
	if (_control_status.flags.fixed_wing) {
		if (_control_status.flags.in_air && !_control_status.flags.vehicle_at_rest) {
			if (!_control_status.flags.fuse_aspd) {
				_yawEstimator.setTrueAirspeed(_params.EKFGSF_tas_default);
			}

		} else {
			_yawEstimator.setTrueAirspeed(0.f);
		}
	}

	if (_params.arsp_thr <= 0.f) {
		stopAirspeedFusion();
		return;
	}

	if (_airspeed_buffer && _airspeed_buffer->pop_first_older_than(imu_delayed.time_us, &_airspeed_sample_delayed)) {

		const airspeedSample &airspeed_sample = _airspeed_sample_delayed;

		updateAirspeed(airspeed_sample, _aid_src_airspeed);

		_innov_check_fail_status.flags.reject_airspeed = _aid_src_airspeed.innovation_rejected; // TODO: remove this redundant flag

		const bool continuing_conditions_passing = _control_status.flags.in_air && _control_status.flags.fixed_wing && !_control_status.flags.fake_pos;
		const bool is_airspeed_significant = airspeed_sample.true_airspeed > _params.arsp_thr;
		const bool is_airspeed_consistent = (_aid_src_airspeed.test_ratio > 0.f && _aid_src_airspeed.test_ratio < 1.f);
		const bool starting_conditions_passing = continuing_conditions_passing && is_airspeed_significant
		                                         && (is_airspeed_consistent || !_control_status.flags.wind); // if wind isn't already estimated, the states are reset when starting airspeed fusion

		if (_control_status.flags.fuse_aspd) {
			if (continuing_conditions_passing) {
				if (is_airspeed_significant) {
					fuseAirspeed(airspeed_sample, _aid_src_airspeed);
				}

				_yawEstimator.setTrueAirspeed(airspeed_sample.true_airspeed);

				const bool is_fusion_failing = isTimedOut(_aid_src_airspeed.time_last_fuse, (uint64_t)10e6);

				if (is_fusion_failing) {
					stopAirspeedFusion();
				}

			} else {
				stopAirspeedFusion();
			}

		} else if (starting_conditions_passing) {
			ECL_INFO("starting airspeed fusion");

			// If starting wind state estimation, reset the wind states and covariances before fusing any data
			// Also catch the case where sideslip fusion enabled wind estimation recently and didn't converge yet.
			const Vector2f wind_var_xy = getWindVelocityVariance();

			if (!_control_status.flags.wind || (wind_var_xy(0) + wind_var_xy(1) > sq(_params.initial_wind_uncertainty))) {
				// activate the wind states
				_control_status.flags.wind = true;
				// reset the wind speed states and corresponding covariances
				resetWindUsingAirspeed(airspeed_sample);
			}

			_control_status.flags.fuse_aspd = true;
		}

	} else if (_control_status.flags.fuse_aspd && !isRecent(_airspeed_sample_delayed.time_us, (uint64_t)1e6)) {
		ECL_WARN("Airspeed data stopped");
		stopAirspeedFusion();
	}
}

void Ekf::updateAirspeed(const airspeedSample &airspeed_sample, estimator_aid_source1d_s &aid_src) const
{
	// reset flags
	resetEstimatorAidStatus(aid_src);

	// Variance for true airspeed measurement - (m/sec)^2
	const float R = sq(math::constrain(_params.eas_noise, 0.5f, 5.0f) *
			   math::constrain(airspeed_sample.eas2tas, 0.9f, 10.0f));

	float innov = 0.f;
	float innov_var = 0.f;
	sym::ComputeAirspeedInnovAndInnovVar(getStateAtFusionHorizonAsVector(), P, airspeed_sample.true_airspeed, R, FLT_EPSILON, &innov, &innov_var);

	aid_src.observation = airspeed_sample.true_airspeed;
	aid_src.observation_variance = R;
	aid_src.innovation = innov;
	aid_src.innovation_variance = innov_var;

	aid_src.fusion_enabled = _control_status.flags.fuse_aspd;

	aid_src.timestamp_sample = airspeed_sample.time_us;

	const float innov_gate = fmaxf(_params.tas_innov_gate, 1.f);
	setEstimatorAidStatusTestRatio(aid_src, innov_gate);
}

void Ekf::fuseAirspeed(const airspeedSample &airspeed_sample, estimator_aid_source1d_s &aid_src)
{
	if (aid_src.innovation_rejected) {
		return;
	}

	// determine if we need the airspeed fusion to correct states other than wind
	const bool update_wind_only = !_control_status.flags.wind_dead_reckoning;

	const float innov_var = aid_src.innovation_variance;

	if (innov_var < aid_src.observation_variance || innov_var < FLT_EPSILON) {
		// Reset the estimator covariance matrix
		// if we are getting aiding from other sources, warn and reset the wind states and covariances only
		const char *action_string = nullptr;

		if (update_wind_only) {
			resetWindUsingAirspeed(airspeed_sample);
			action_string = "wind";

		} else {
			initialiseCovariance();
			_state.wind_vel.setZero();
			action_string = "full";
		}

		ECL_ERR("airspeed badly conditioned - %s covariance reset", action_string);

		_fault_status.flags.bad_airspeed = true;

		return;
	}

	_fault_status.flags.bad_airspeed = false;

	Vector24f H; // Observation jacobian
	Vector24f K; // Kalman gain vector

	sym::ComputeAirspeedHAndK(getStateAtFusionHorizonAsVector(), P, innov_var, FLT_EPSILON, &H, &K);

	if (update_wind_only) {
		for (unsigned row = 0; row <= 21; row++) {
			K(row) = 0.f;
		}
	}

	const bool is_fused = measurementUpdate(K, aid_src.innovation_variance, aid_src.innovation);

	aid_src.fused = is_fused;
	_fault_status.flags.bad_airspeed = !is_fused;

	if (is_fused) {
		aid_src.time_last_fuse = _time_delayed_us;
	}
}

void Ekf::stopAirspeedFusion()
{
	if (_control_status.flags.fuse_aspd) {
		ECL_INFO("stopping airspeed fusion");
		resetEstimatorAidStatus(_aid_src_airspeed);
		_yawEstimator.setTrueAirspeed(NAN);
		_control_status.flags.fuse_aspd = false;
	}
}

void Ekf::resetWindUsingAirspeed(const airspeedSample &airspeed_sample)
{
	const float euler_yaw = getEulerYaw(_R_to_earth);

	// estimate wind using zero sideslip assumption and airspeed measurement if airspeed available
	_state.wind_vel(0) = _state.vel(0) - airspeed_sample.true_airspeed * cosf(euler_yaw);
	_state.wind_vel(1) = _state.vel(1) - airspeed_sample.true_airspeed * sinf(euler_yaw);

	ECL_INFO("reset wind using airspeed to (%.3f, %.3f)", (double)_state.wind_vel(0), (double)_state.wind_vel(1));

	resetWindCovarianceUsingAirspeed(airspeed_sample);

	_aid_src_airspeed.time_last_fuse = _time_delayed_us;
}

void Ekf::resetWindCovarianceUsingAirspeed(const airspeedSample &airspeed_sample)
{
	// Derived using EKF/matlab/scripts/Inertial Nav EKF/wind_cov.py
	// TODO: explicitly include the sideslip angle in the derivation
	const float euler_yaw = getEulerYaw(_R_to_earth);
	const float R_TAS = sq(math::constrain(_params.eas_noise, 0.5f, 5.0f) * math::constrain(airspeed_sample.eas2tas, 0.9f, 10.0f));
	constexpr float initial_sideslip_uncertainty = math::radians(15.0f);
	const float initial_wind_var_body_y = sq(airspeed_sample.true_airspeed * sinf(initial_sideslip_uncertainty));
	constexpr float R_yaw = sq(math::radians(10.0f));

	const float cos_yaw = cosf(euler_yaw);
	const float sin_yaw = sinf(euler_yaw);

	// rotate wind velocity into earth frame aligned with vehicle yaw
	const float Wx = _state.wind_vel(0) * cos_yaw + _state.wind_vel(1) * sin_yaw;
	const float Wy = -_state.wind_vel(0) * sin_yaw + _state.wind_vel(1) * cos_yaw;

	// it is safer to remove all existing correlations to other states at this time
	P.uncorrelateCovarianceSetVariance<2>(22, 0.0f);

	P(22, 22) = R_TAS * sq(cos_yaw) + R_yaw * sq(-Wx * sin_yaw - Wy * cos_yaw) + initial_wind_var_body_y * sq(sin_yaw);
	P(22, 23) = R_TAS * sin_yaw * cos_yaw + R_yaw * (-Wx * sin_yaw - Wy * cos_yaw) * (Wx * cos_yaw - Wy * sin_yaw) -
		    initial_wind_var_body_y * sin_yaw * cos_yaw;
	P(23, 22) = P(22, 23);
	P(23, 23) = R_TAS * sq(sin_yaw) + R_yaw * sq(Wx * cos_yaw - Wy * sin_yaw) + initial_wind_var_body_y * sq(cos_yaw);

	// Now add the variance due to uncertainty in vehicle velocity that was used to calculate the initial wind speed
	P(22, 22) += P(4, 4);
	P(23, 23) += P(5, 5);
}
/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ekf.h"

void Ekf::controlAuxVelFusion()
{
	if (_auxvel_buffer) {
		auxVelSample auxvel_sample_delayed;

		if (_auxvel_buffer->pop_first_older_than(_time_delayed_us, &auxvel_sample_delayed)) {

			resetEstimatorAidStatus(_aid_src_aux_vel);

			updateVelocityAidSrcStatus(auxvel_sample_delayed.time_us, auxvel_sample_delayed.vel, auxvel_sample_delayed.velVar, fmaxf(_params.auxvel_gate, 1.f), _aid_src_aux_vel);

			if (isHorizontalAidingActive()) {
				_aid_src_aux_vel.fusion_enabled = true;
				fuseVelocity(_aid_src_aux_vel);
			}
		}
	}
}

void Ekf::stopAuxVelFusion()
{
	ECL_INFO("stopping aux vel fusion");
	//_control_status.flags.aux_vel = false;
	resetEstimatorAidStatus(_aid_src_aux_vel);
}
/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file drag_fusion.cpp
 * Body frame drag fusion methods used for multi-rotor wind estimation.
 */

#include "ekf.h"
#include "python/ekf_derivation/generated/compute_drag_x_innov_var_and_k.h"
#include "python/ekf_derivation/generated/compute_drag_y_innov_var_and_k.h"

#include <mathlib/mathlib.h>

void Ekf::controlDragFusion()
{
	if ((_params.drag_ctrl > 0) && _drag_buffer &&
	    !_control_status.flags.fake_pos && _control_status.flags.in_air) {

		if (!_control_status.flags.wind) {
			// reset the wind states and covariances when starting drag accel fusion
			_control_status.flags.wind = true;
			resetWindToZero();
		}

		dragSample drag_sample;

		if (_drag_buffer->pop_first_older_than(_time_delayed_us, &drag_sample)) {
			fuseDrag(drag_sample);
		}
	}
}

void Ekf::fuseDrag(const dragSample &drag_sample)
{
	const float R_ACC = fmaxf(_params.drag_noise, 0.5f); // observation noise variance in specific force drag (m/sec**2)**2
	const float rho = fmaxf(_air_density, 0.1f); // air density (kg/m**3)

	// correct rotor momentum drag for increase in required rotor mass flow with altitude
	// obtained from momentum disc theory
	const float mcoef_corrrected = fmaxf(_params.mcoef * sqrtf(rho / CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C), 0.f);

	// drag model parameters
	const bool using_bcoef_x = _params.bcoef_x > 1.0f;
	const bool using_bcoef_y = _params.bcoef_y > 1.0f;
	const bool using_mcoef   = _params.mcoef   > 0.001f;

	if (!using_bcoef_x && !using_bcoef_y && !using_mcoef) {
		return;
	}

	// calculate relative wind velocity in earth frame and rotate into body frame
	const Vector3f rel_wind_earth(_state.vel(0) - _state.wind_vel(0),
				      _state.vel(1) - _state.wind_vel(1),
				      _state.vel(2));
	const Vector3f rel_wind_body = _state.quat_nominal.rotateVectorInverse(rel_wind_earth);
	const float rel_wind_speed = rel_wind_body.norm();
	const Vector24f state_vector_prev = getStateAtFusionHorizonAsVector();

	Vector2f bcoef_inv;

	if (using_bcoef_x) {
		bcoef_inv(0) = 1.0f / _params.bcoef_x;
	}

	if (using_bcoef_y) {
		bcoef_inv(1) = 1.0f / _params.bcoef_y;
	}

	if (using_bcoef_x && using_bcoef_y) {

		// Interpolate between the X and Y bluff body drag coefficients using current relative velocity
		// This creates an elliptic drag distribution around the XY plane
		bcoef_inv(0) = Vector2f(bcoef_inv.emult(rel_wind_body.xy()) / rel_wind_body.xy().norm()).norm();
		bcoef_inv(1) = bcoef_inv(0);
	}

	Vector24f Kfusion;

	// perform sequential fusion of XY specific forces
	for (uint8_t axis_index = 0; axis_index < 2; axis_index++) {
		// measured drag acceleration corrected for sensor bias
		const float mea_acc = drag_sample.accelXY(axis_index)  - _state.delta_vel_bias(axis_index) / _dt_ekf_avg;

		// Drag is modelled as an arbitrary combination of bluff body drag that proportional to
		// equivalent airspeed squared, and rotor momentum drag that is proportional to true airspeed
		// parallel to the rotor disc and mass flow through the rotor disc.

		if (axis_index == 0) {
			if (!using_bcoef_x && !using_mcoef) {
				continue;
			}

			sym::ComputeDragXInnovVarAndK(state_vector_prev, P, rho, bcoef_inv(axis_index), mcoef_corrrected, R_ACC, FLT_EPSILON, &_drag_innov_var(axis_index), &Kfusion);

		} else if (axis_index == 1) {
			if (!using_bcoef_y && !using_mcoef) {
				continue;
			}

			sym::ComputeDragYInnovVarAndK(state_vector_prev, P, rho, bcoef_inv(axis_index), mcoef_corrrected, R_ACC, FLT_EPSILON, &_drag_innov_var(axis_index), &Kfusion);
		}

		if (_drag_innov_var(axis_index) < R_ACC) {
			// calculation is badly conditioned
			return;
		}

		const float pred_acc = -0.5f * bcoef_inv(axis_index) * rho * rel_wind_body(axis_index) * rel_wind_speed - rel_wind_body(axis_index) * mcoef_corrrected;

		// Apply an innovation consistency check with a 5 Sigma threshold
		_drag_innov(axis_index) = pred_acc - mea_acc;
		_drag_test_ratio(axis_index) = sq(_drag_innov(axis_index)) / (sq(5.0f) * _drag_innov_var(axis_index));

		// if the innovation consistency check fails then don't fuse the sample
		if (_drag_test_ratio(axis_index) <= 1.0f) {
			measurementUpdate(Kfusion, _drag_innov_var(axis_index), _drag_innov(axis_index));
		}
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ev_control.cpp
 * Control functions for ekf external vision control
 */

#include "ekf.h"

void Ekf::controlExternalVisionFusion()
{
	_ev_pos_b_est.predict(_dt_ekf_avg);

	// Check for new external vision data
	extVisionSample ev_sample;

	if (_ext_vision_buffer && _ext_vision_buffer->pop_first_older_than(_time_delayed_us, &ev_sample)) {

		bool ev_reset = (ev_sample.reset_counter != _ev_sample_prev.reset_counter);

		// determine if we should use the horizontal position observations
		bool quality_sufficient = (_params.ev_quality_minimum <= 0) || (ev_sample.quality >= _params.ev_quality_minimum);

		const bool starting_conditions_passing = quality_sufficient
				&& ((ev_sample.time_us - _ev_sample_prev.time_us) < EV_MAX_INTERVAL)
				&& ((_params.ev_quality_minimum <= 0) || (_ev_sample_prev.quality >= _params.ev_quality_minimum)) // previous quality sufficient
				&& ((_params.ev_quality_minimum <= 0) || (_ext_vision_buffer->get_newest().quality >= _params.ev_quality_minimum)) // newest quality sufficient
				&& isNewestSampleRecent(_time_last_ext_vision_buffer_push, EV_MAX_INTERVAL);

		controlEvYawFusion(ev_sample, starting_conditions_passing, ev_reset, quality_sufficient, _aid_src_ev_yaw);
		controlEvVelFusion(ev_sample, starting_conditions_passing, ev_reset, quality_sufficient, _aid_src_ev_vel);
		controlEvPosFusion(ev_sample, starting_conditions_passing, ev_reset, quality_sufficient, _aid_src_ev_pos);
		controlEvHeightFusion(ev_sample, starting_conditions_passing, ev_reset, quality_sufficient, _aid_src_ev_hgt);

		if (quality_sufficient) {
			_ev_sample_prev = ev_sample;
		}

		// record corresponding yaw state for future EV delta heading innovation (logging only)
		_ev_yaw_pred_prev = getEulerYaw(_state.quat_nominal);

	} else if ((_control_status.flags.ev_pos || _control_status.flags.ev_vel || _control_status.flags.ev_yaw
		    || _control_status.flags.ev_hgt)
		   && isTimedOut(_ev_sample_prev.time_us, 2 * EV_MAX_INTERVAL)) {

		// Turn off EV fusion mode if no data has been received
		stopEvPosFusion();
		stopEvVelFusion();
		stopEvYawFusion();
		stopEvHgtFusion();

		_warning_events.flags.vision_data_stopped = true;
		ECL_WARN("vision data stopped");
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ev_height_control.cpp
 * Control functions for ekf external vision height fusion
 */

#include "ekf.h"

void Ekf::controlEvHeightFusion(const extVisionSample &ev_sample, const bool common_starting_conditions_passing,
				const bool ev_reset, const bool quality_sufficient, estimator_aid_source1d_s &aid_src)
{
	static constexpr const char *AID_SRC_NAME = "EV height";

	HeightBiasEstimator &bias_est = _ev_hgt_b_est;

	bias_est.predict(_dt_ekf_avg);

	// correct position for offset relative to IMU
	const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
	const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;

	// rotate measurement into correct earth frame if required
	Vector3f pos{ev_sample.pos};
	Matrix3f pos_cov{matrix::diag(ev_sample.position_var)};

	// rotate EV to the EKF reference frame unless we're operating entirely in vision frame
	// TODO: only necessary if there's a roll/pitch offset between VIO and EKF
	if (!(_control_status.flags.ev_yaw && _control_status.flags.ev_pos)) {

		const Quatf q_error((_state.quat_nominal * ev_sample.quat.inversed()).normalized());

		if (q_error.isAllFinite()) {
			const Dcmf R_ev_to_ekf(q_error);

			pos = R_ev_to_ekf * ev_sample.pos;
			pos_cov = R_ev_to_ekf * matrix::diag(ev_sample.position_var) * R_ev_to_ekf.transpose();

			// increase minimum variance to include EV orientation variance
			// TODO: do this properly
			const float orientation_var_max = math::max(ev_sample.orientation_var(0), ev_sample.orientation_var(1));
			pos_cov(2, 2) = math::max(pos_cov(2, 2), orientation_var_max);
		}
	}

	const float measurement = pos(2) - pos_offset_earth(2);
	float measurement_var = math::max(pos_cov(2, 2), sq(_params.ev_pos_noise), sq(0.01f));

	// increase minimum variance if GPS active
	if (_control_status.flags.gps_hgt) {
		measurement_var = math::max(measurement_var, sq(_params.gps_pos_noise));
	}

	const bool measurement_valid = PX4_ISFINITE(measurement) && PX4_ISFINITE(measurement_var);

	updateVerticalPositionAidSrcStatus(ev_sample.time_us,
					   measurement - bias_est.getBias(),
					   measurement_var + bias_est.getBiasVar(),
					   math::max(_params.ev_pos_innov_gate, 1.f),
					   aid_src);

	// update the bias estimator before updating the main filter but after
	// using its current state to compute the vertical position innovation
	if (measurement_valid && quality_sufficient) {
		bias_est.setMaxStateNoise(sqrtf(measurement_var));
		bias_est.setProcessNoiseSpectralDensity(_params.ev_hgt_bias_nsd);
		bias_est.fuseBias(measurement - _state.pos(2), measurement_var + P(9, 9));
	}

	const bool continuing_conditions_passing = (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::VPOS))
			&& measurement_valid;

	const bool starting_conditions_passing = common_starting_conditions_passing
			&& continuing_conditions_passing;

	if (_control_status.flags.ev_hgt) {
		aid_src.fusion_enabled = true;

		if (continuing_conditions_passing) {

			if (ev_reset) {

				if (quality_sufficient) {
					ECL_INFO("reset to %s", AID_SRC_NAME);

					if (_height_sensor_ref == HeightSensor::EV) {
						_information_events.flags.reset_hgt_to_ev = true;
						resetVerticalPositionTo(measurement, measurement_var);
						bias_est.reset();

					} else {
						bias_est.setBias(-_state.pos(2) + measurement);
					}

					aid_src.time_last_fuse = _time_delayed_us;

				} else {
					// EV has reset, but quality isn't sufficient
					// we have no choice but to stop EV and try to resume once quality is acceptable
					stopEvHgtFusion();
					return;
				}

			} else if (quality_sufficient) {
				fuseVerticalPosition(aid_src);

			} else {
				aid_src.innovation_rejected = true;
			}

			const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.hgt_fusion_timeout_max);

			if (isHeightResetRequired() && quality_sufficient) {
				// All height sources are failing
				ECL_WARN("%s fusion reset required, all height sources failing", AID_SRC_NAME);
				_information_events.flags.reset_hgt_to_ev = true;
				resetVerticalPositionTo(measurement - bias_est.getBias(), measurement_var);
				bias_est.setBias(-_state.pos(2) + measurement);

				// reset vertical velocity
				if (ev_sample.vel.isAllFinite() && (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::VEL))) {

					// correct velocity for offset relative to IMU
					const Vector3f vel_offset_body = _ang_rate_delayed_raw % pos_offset_body;
					const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;

					switch (ev_sample.vel_frame) {
					case VelocityFrame::LOCAL_FRAME_NED:
					case VelocityFrame::LOCAL_FRAME_FRD: {
							const Vector3f reset_vel = ev_sample.vel - vel_offset_earth;
							resetVerticalVelocityTo(reset_vel(2), math::max(ev_sample.velocity_var(2), sq(_params.ev_vel_noise)));
						}
						break;

					case VelocityFrame::BODY_FRAME_FRD: {
							const Vector3f reset_vel = _R_to_earth * (ev_sample.vel - vel_offset_body);
							const Matrix3f reset_vel_cov = _R_to_earth * matrix::diag(ev_sample.velocity_var) * _R_to_earth.transpose();
							resetVerticalVelocityTo(reset_vel(2), math::max(reset_vel_cov(2, 2), sq(_params.ev_vel_noise)));
						}
						break;
					}

				} else {
					resetVerticalVelocityToZero();
				}

				aid_src.time_last_fuse = _time_delayed_us;

			} else if (is_fusion_failing) {
				// A reset did not fix the issue but all the starting checks are not passing
				// This could be a temporary issue, stop the fusion without declaring the sensor faulty
				ECL_WARN("stopping %s, fusion failing", AID_SRC_NAME);
				stopEvHgtFusion();
			}

		} else {
			// Stop fusion but do not declare it faulty
			ECL_WARN("stopping %s fusion, continuing conditions failing", AID_SRC_NAME);
			stopEvHgtFusion();
		}

	} else {
		if (starting_conditions_passing) {
			// activate fusion, only reset if necessary
			if (_params.height_sensor_ref == HeightSensor::EV) {
				ECL_INFO("starting %s fusion, resetting state", AID_SRC_NAME);
				_information_events.flags.reset_hgt_to_ev = true;
				resetVerticalPositionTo(measurement, measurement_var);

				_height_sensor_ref = HeightSensor::EV;
				bias_est.reset();

			} else {
				ECL_INFO("starting %s fusion", AID_SRC_NAME);
				bias_est.setBias(-_state.pos(2) + measurement);
			}

			aid_src.time_last_fuse = _time_delayed_us;
			bias_est.setFusionActive();
			_control_status.flags.ev_hgt = true;
		}
	}
}

void Ekf::stopEvHgtFusion()
{
	if (_control_status.flags.ev_hgt) {

		if (_height_sensor_ref == HeightSensor::EV) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}

		_ev_hgt_b_est.setFusionInactive();
		resetEstimatorAidStatus(_aid_src_ev_hgt);

		_control_status.flags.ev_hgt = false;
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ev_pos_control.cpp
 * Control functions for ekf external vision position fusion
 */

#include "ekf.h"

static constexpr const char *EV_AID_SRC_NAME = "EV position";


void Ekf::controlEvPosFusion(const extVisionSample &ev_sample, const bool common_starting_conditions_passing,
			     const bool ev_reset, const bool quality_sufficient, estimator_aid_source2d_s &aid_src)
{
	const bool yaw_alignment_changed = (!_control_status_prev.flags.ev_yaw && _control_status.flags.ev_yaw)
					   || (_control_status_prev.flags.yaw_align != _control_status.flags.yaw_align);

	// determine if we should use EV position aiding
	bool continuing_conditions_passing = (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::HPOS))
					     && _control_status.flags.tilt_align
					     && PX4_ISFINITE(ev_sample.pos(0))
					     && PX4_ISFINITE(ev_sample.pos(1));

	// correct position for offset relative to IMU
	const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
	const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;

	const bool bias_fusion_was_active = _ev_pos_b_est.fusionActive();

	// rotate measurement into correct earth frame if required
	Vector3f pos{NAN, NAN, NAN};
	Matrix3f pos_cov{};

	switch (ev_sample.pos_frame) {
	case PositionFrame::LOCAL_FRAME_NED:
		if (_control_status.flags.yaw_align) {
			pos = ev_sample.pos - pos_offset_earth;
			pos_cov = matrix::diag(ev_sample.position_var);

			if (_control_status.flags.gps) {
				_ev_pos_b_est.setFusionActive();

			} else {
				_ev_pos_b_est.setFusionInactive();
			}

		} else {
			continuing_conditions_passing = false;
			_ev_pos_b_est.setFusionInactive();
			_ev_pos_b_est.reset();
		}

		break;

	case PositionFrame::LOCAL_FRAME_FRD:
		if (_control_status.flags.ev_yaw) {
			// using EV frame
			pos = ev_sample.pos - pos_offset_earth;
			pos_cov = matrix::diag(ev_sample.position_var);

			_ev_pos_b_est.setFusionInactive();
			_ev_pos_b_est.reset();

		} else {
			// rotate EV to the EKF reference frame
			const Quatf q_error((_state.quat_nominal * ev_sample.quat.inversed()).normalized());
			const Dcmf R_ev_to_ekf = Dcmf(q_error);

			pos = R_ev_to_ekf * ev_sample.pos - pos_offset_earth;
			pos_cov = R_ev_to_ekf * matrix::diag(ev_sample.position_var) * R_ev_to_ekf.transpose();

			// increase minimum variance to include EV orientation variance
			// TODO: do this properly
			const float orientation_var_max = ev_sample.orientation_var.max();

			for (int i = 0; i < 2; i++) {
				pos_cov(i, i) = math::max(pos_cov(i, i), orientation_var_max);
			}

			if (_control_status.flags.gps) {
				_ev_pos_b_est.setFusionActive();

			} else {
				_ev_pos_b_est.setFusionInactive();
			}
		}

		break;

	default:
		continuing_conditions_passing = false;
		_ev_pos_b_est.setFusionInactive();
		_ev_pos_b_est.reset();
		break;
	}

	// increase minimum variance if GPS active (position reference)
	if (_control_status.flags.gps) {
		for (int i = 0; i < 2; i++) {
			pos_cov(i, i) = math::max(pos_cov(i, i), sq(_params.gps_pos_noise));
		}
	}

	const Vector2f measurement{pos(0), pos(1)};

	const Vector2f measurement_var{
		math::max(pos_cov(0, 0), sq(_params.ev_pos_noise), sq(0.01f)),
		math::max(pos_cov(1, 1), sq(_params.ev_pos_noise), sq(0.01f))
	};

	const bool measurement_valid = measurement.isAllFinite() && measurement_var.isAllFinite();

	// bias fusion activated (GPS activated)
	if (!bias_fusion_was_active && _ev_pos_b_est.fusionActive()) {
		if (quality_sufficient) {
			// reset the bias estimator
			_ev_pos_b_est.setBias(-Vector2f(_state.pos.xy()) + measurement);

		} else if (isOtherSourceOfHorizontalAidingThan(_control_status.flags.ev_pos)) {
			// otherwise stop EV position, when quality is good again it will restart with reset bias
			stopEvPosFusion();
		}
	}

	updateHorizontalPositionAidSrcStatus(ev_sample.time_us,
					     measurement - _ev_pos_b_est.getBias(),        // observation
					     measurement_var + _ev_pos_b_est.getBiasVar(), // observation variance
					     math::max(_params.ev_pos_innov_gate, 1.f),    // innovation gate
					     aid_src);

	// update the bias estimator before updating the main filter but after
	// using its current state to compute the vertical position innovation
	if (measurement_valid && quality_sufficient) {
		_ev_pos_b_est.setMaxStateNoise(Vector2f(sqrtf(measurement_var(0)), sqrtf(measurement_var(1))));
		_ev_pos_b_est.setProcessNoiseSpectralDensity(_params.ev_hgt_bias_nsd); // TODO
		_ev_pos_b_est.fuseBias(measurement - Vector2f(_state.pos.xy()), measurement_var + Vector2f(P(7, 7), P(8, 8)));
	}

	if (!measurement_valid) {
		continuing_conditions_passing = false;
	}

	const bool starting_conditions_passing = common_starting_conditions_passing
			&& continuing_conditions_passing;

	if (_control_status.flags.ev_pos) {
		aid_src.fusion_enabled = true;

		if (continuing_conditions_passing) {
			const bool bias_estimator_change = (bias_fusion_was_active != _ev_pos_b_est.fusionActive());
			const bool reset = ev_reset || yaw_alignment_changed || bias_estimator_change;

			updateEvPosFusion(measurement, measurement_var, quality_sufficient, reset, aid_src);

		} else {
			// Stop fusion but do not declare it faulty
			ECL_WARN("stopping %s fusion, continuing conditions failing", EV_AID_SRC_NAME);
			stopEvPosFusion();
		}

	} else {
		if (starting_conditions_passing) {
			startEvPosFusion(measurement, measurement_var, aid_src);
		}
	}
}

void Ekf::startEvPosFusion(const Vector2f &measurement, const Vector2f &measurement_var, estimator_aid_source2d_s &aid_src)
{
	// activate fusion
	// TODO:  (_params.position_sensor_ref == PositionSensor::EV)
	if (_control_status.flags.gps) {
		ECL_INFO("starting %s fusion", EV_AID_SRC_NAME);
		_ev_pos_b_est.setBias(-Vector2f(_state.pos.xy()) + measurement);
		_ev_pos_b_est.setFusionActive();

	} else {
		ECL_INFO("starting %s fusion, resetting state", EV_AID_SRC_NAME);
		//_position_sensor_ref = PositionSensor::EV;
		_information_events.flags.reset_pos_to_vision = true;
		resetHorizontalPositionTo(measurement, measurement_var);
		_ev_pos_b_est.reset();
	}

	aid_src.time_last_fuse = _time_delayed_us;

	_nb_ev_pos_reset_available = 5;
	_information_events.flags.starting_vision_pos_fusion = true;
	_control_status.flags.ev_pos = true;
}

void Ekf::updateEvPosFusion(const Vector2f &measurement, const Vector2f &measurement_var, bool quality_sufficient, bool reset, estimator_aid_source2d_s &aid_src)
{
	if (reset) {

		if (quality_sufficient) {

			if (!_control_status.flags.gps) {
				ECL_INFO("reset to %s", EV_AID_SRC_NAME);
				_information_events.flags.reset_pos_to_vision = true;
				resetHorizontalPositionTo(measurement, measurement_var);
				_ev_pos_b_est.reset();

			} else {
				_ev_pos_b_est.setBias(-Vector2f(_state.pos.xy()) + measurement);
			}

			aid_src.time_last_fuse = _time_delayed_us;

		} else {
			// EV has reset, but quality isn't sufficient
			// we have no choice but to stop EV and try to resume once quality is acceptable
			stopEvPosFusion();
			return;
		}

	} else if (quality_sufficient) {
		fuseHorizontalPosition(aid_src);

	} else {
		aid_src.innovation_rejected = true;
	}

	const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.no_aid_timeout_max); // 1 second

	if (is_fusion_failing) {
		bool pos_xy_fusion_failing = isTimedOut(_time_last_hor_pos_fuse, _params.no_aid_timeout_max);

		if ((_nb_ev_pos_reset_available > 0) && quality_sufficient) {
			// Data seems good, attempt a reset
			ECL_WARN("%s fusion failing, resetting", EV_AID_SRC_NAME);

			if (_control_status.flags.gps && !pos_xy_fusion_failing) {
				// reset EV position bias
				_ev_pos_b_est.setBias(-Vector2f(_state.pos.xy()) + measurement);

			} else {
				_information_events.flags.reset_pos_to_vision = true;

				if (_control_status.flags.gps) {
					resetHorizontalPositionTo(measurement - _ev_pos_b_est.getBias(), measurement_var + _ev_pos_b_est.getBiasVar());
					_ev_pos_b_est.setBias(-Vector2f(_state.pos.xy()) + measurement);

				} else {
					resetHorizontalPositionTo(measurement, measurement_var);
					_ev_pos_b_est.reset();
				}
			}

			aid_src.time_last_fuse = _time_delayed_us;

			if (_control_status.flags.in_air) {
				_nb_ev_pos_reset_available--;
			}

		} else {
			// A reset did not fix the issue but all the starting checks are not passing
			// This could be a temporary issue, stop the fusion without declaring the sensor faulty
			ECL_WARN("stopping %s, fusion failing", EV_AID_SRC_NAME);
			stopEvPosFusion();
		}
	}
}

void Ekf::stopEvPosFusion()
{
	if (_control_status.flags.ev_pos) {
		resetEstimatorAidStatus(_aid_src_ev_pos);

		_control_status.flags.ev_pos = false;
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ev_vel_control.cpp
 * Control functions for ekf external vision velocity fusion
 */

#include "ekf.h"

void Ekf::controlEvVelFusion(const extVisionSample &ev_sample, const bool common_starting_conditions_passing,
			     const bool ev_reset, const bool quality_sufficient, estimator_aid_source3d_s &aid_src)
{
	static constexpr const char *AID_SRC_NAME = "EV velocity";

	const bool yaw_alignment_changed = (!_control_status_prev.flags.ev_yaw && _control_status.flags.ev_yaw)
					   || (_control_status_prev.flags.yaw_align != _control_status.flags.yaw_align);

	// determine if we should use EV velocity aiding
	bool continuing_conditions_passing = (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::VEL))
					     && _control_status.flags.tilt_align
					     && ev_sample.vel.isAllFinite();

	// correct velocity for offset relative to IMU
	const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
	const Vector3f vel_offset_body = _ang_rate_delayed_raw % pos_offset_body;
	const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;

	// rotate measurement into correct earth frame if required
	Vector3f vel{NAN, NAN, NAN};
	Matrix3f vel_cov{};

	switch (ev_sample.vel_frame) {
	case VelocityFrame::LOCAL_FRAME_NED:
		if (_control_status.flags.yaw_align) {
			vel = ev_sample.vel - vel_offset_earth;
			vel_cov = matrix::diag(ev_sample.velocity_var);

		} else {
			continuing_conditions_passing = false;
		}

		break;

	case VelocityFrame::LOCAL_FRAME_FRD:
		if (_control_status.flags.ev_yaw) {
			// using EV frame
			vel = ev_sample.vel - vel_offset_earth;
			vel_cov = matrix::diag(ev_sample.velocity_var);

		} else {
			// rotate EV to the EKF reference frame
			const Quatf q_error((_state.quat_nominal * ev_sample.quat.inversed()).normalized());
			const Dcmf R_ev_to_ekf = Dcmf(q_error);

			vel = R_ev_to_ekf * ev_sample.vel - vel_offset_earth;
			vel_cov = R_ev_to_ekf * matrix::diag(ev_sample.velocity_var) * R_ev_to_ekf.transpose();

			// increase minimum variance to include EV orientation variance
			// TODO: do this properly
			const float orientation_var_max = ev_sample.orientation_var.max();

			for (int i = 0; i < 2; i++) {
				vel_cov(i, i) = math::max(vel_cov(i, i), orientation_var_max);
			}
		}

		break;

	case VelocityFrame::BODY_FRAME_FRD:
		vel = _R_to_earth * (ev_sample.vel - vel_offset_body);
		vel_cov = _R_to_earth * matrix::diag(ev_sample.velocity_var) * _R_to_earth.transpose();
		break;

	default:
		continuing_conditions_passing = false;
		break;
	}

	// increase minimum variance if GPS active (position reference)
	if (_control_status.flags.gps) {
		for (int i = 0; i < 2; i++) {
			vel_cov(i, i) = math::max(vel_cov(i, i), sq(_params.gps_vel_noise));
		}
	}

	const Vector3f measurement{vel};

	const Vector3f measurement_var{
		math::max(vel_cov(0, 0), sq(_params.ev_vel_noise), sq(0.01f)),
		math::max(vel_cov(1, 1), sq(_params.ev_vel_noise), sq(0.01f)),
		math::max(vel_cov(2, 2), sq(_params.ev_vel_noise), sq(0.01f))
	};

	const bool measurement_valid = measurement.isAllFinite() && measurement_var.isAllFinite();

	updateVelocityAidSrcStatus(ev_sample.time_us,
				   measurement,                               // observation
				   measurement_var,                           // observation variance
				   math::max(_params.ev_vel_innov_gate, 1.f), // innovation gate
				   aid_src);

	if (!measurement_valid) {
		continuing_conditions_passing = false;
	}

	const bool starting_conditions_passing = common_starting_conditions_passing
			&& continuing_conditions_passing
			&& ((Vector3f(aid_src.test_ratio).max() < 0.1f) || !isHorizontalAidingActive());

	if (_control_status.flags.ev_vel) {
		aid_src.fusion_enabled = true;

		if (continuing_conditions_passing) {

			if ((ev_reset && isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.ev_vel)) || yaw_alignment_changed) {

				if (quality_sufficient) {
					ECL_INFO("reset to %s", AID_SRC_NAME);
					_information_events.flags.reset_vel_to_vision = true;
					resetVelocityTo(measurement, measurement_var);
					aid_src.time_last_fuse = _time_delayed_us;

				} else {
					// EV has reset, but quality isn't sufficient
					// we have no choice but to stop EV and try to resume once quality is acceptable
					stopEvVelFusion();
					return;
				}

			} else if (quality_sufficient) {
				fuseVelocity(aid_src);

			} else {
				aid_src.innovation_rejected = true;
			}

			const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.no_aid_timeout_max); // 1 second

			if (is_fusion_failing) {

				if ((_nb_ev_vel_reset_available > 0) && quality_sufficient) {
					// Data seems good, attempt a reset
					_information_events.flags.reset_vel_to_vision = true;
					ECL_WARN("%s fusion failing, resetting", AID_SRC_NAME);
					resetVelocityTo(measurement, measurement_var);
					aid_src.time_last_fuse = _time_delayed_us;

					if (_control_status.flags.in_air) {
						_nb_ev_vel_reset_available--;
					}

				} else if (starting_conditions_passing) {
					// Data seems good, but previous reset did not fix the issue
					// something else must be wrong, declare the sensor faulty and stop the fusion
					//_control_status.flags.ev_vel_fault = true;
					ECL_WARN("stopping %s fusion, starting conditions failing", AID_SRC_NAME);
					stopEvVelFusion();

				} else {
					// A reset did not fix the issue but all the starting checks are not passing
					// This could be a temporary issue, stop the fusion without declaring the sensor faulty
					ECL_WARN("stopping %s, fusion failing", AID_SRC_NAME);
					stopEvVelFusion();
				}
			}

		} else {
			// Stop fusion but do not declare it faulty
			ECL_WARN("stopping %s fusion, continuing conditions failing", AID_SRC_NAME);
			stopEvVelFusion();
		}

	} else {
		if (starting_conditions_passing) {
			// activate fusion, only reset if necessary
			if (!isHorizontalAidingActive() || yaw_alignment_changed) {
				ECL_INFO("starting %s fusion, resetting velocity to (%.3f, %.3f, %.3f)", AID_SRC_NAME, (double)measurement(0), (double)measurement(1), (double)measurement(2));
				_information_events.flags.reset_vel_to_vision = true;
				resetVelocityTo(measurement, measurement_var);

			} else {
				ECL_INFO("starting %s fusion", AID_SRC_NAME);
			}

			aid_src.time_last_fuse = _time_delayed_us;

			_nb_ev_vel_reset_available = 5;
			_information_events.flags.starting_vision_vel_fusion = true;
			_control_status.flags.ev_vel = true;
		}
	}
}

void Ekf::stopEvVelFusion()
{
	if (_control_status.flags.ev_vel) {
		resetEstimatorAidStatus(_aid_src_ev_vel);

		_control_status.flags.ev_vel = false;
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ev_yaw_control.cpp
 * Control functions for ekf external vision yaw fusion
 */

#include "ekf.h"

void Ekf::controlEvYawFusion(const extVisionSample &ev_sample, const bool common_starting_conditions_passing,
			     const bool ev_reset, const bool quality_sufficient, estimator_aid_source1d_s &aid_src)
{
	static constexpr const char *AID_SRC_NAME = "EV yaw";

	resetEstimatorAidStatus(aid_src);
	aid_src.timestamp_sample = ev_sample.time_us;
	aid_src.observation = getEulerYaw(ev_sample.quat);
	aid_src.observation_variance = math::max(ev_sample.orientation_var(2), _params.ev_att_noise, sq(0.01f));
	aid_src.innovation = wrap_pi(getEulerYaw(_R_to_earth) - aid_src.observation);

	// determine if we should use EV yaw aiding
	bool continuing_conditions_passing = (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::YAW))
					     && _control_status.flags.tilt_align
					     && !_inhibit_ev_yaw_use
					     && PX4_ISFINITE(aid_src.observation)
					     && PX4_ISFINITE(aid_src.observation_variance);

	// if GPS enabled don't allow EV yaw if EV isn't NED
	if (_control_status.flags.gps && _control_status.flags.yaw_align
	    && (ev_sample.pos_frame != PositionFrame::LOCAL_FRAME_NED)
	   ) {
		continuing_conditions_passing = false;

		// use delta yaw for innovation logging
		aid_src.innovation = wrap_pi(wrap_pi(getEulerYaw(_R_to_earth) - _ev_yaw_pred_prev)
					     - wrap_pi(getEulerYaw(ev_sample.quat) - getEulerYaw(_ev_sample_prev.quat)));
	}

	const bool starting_conditions_passing = common_starting_conditions_passing
			&& continuing_conditions_passing
			&& isTimedOut(aid_src.time_last_fuse, (uint32_t)1e6);

	if (_control_status.flags.ev_yaw) {
		aid_src.fusion_enabled = true;

		if (continuing_conditions_passing) {

			if (ev_reset) {

				if (quality_sufficient) {
					ECL_INFO("reset to %s", AID_SRC_NAME);
					//_information_events.flags.reset_yaw_to_vision = true; // TODO
					resetQuatStateYaw(aid_src.observation, aid_src.observation_variance);
					aid_src.time_last_fuse = _time_delayed_us;

				} else {
					// EV has reset, but quality isn't sufficient
					// we have no choice but to stop EV and try to resume once quality is acceptable
					stopEvYawFusion();
					return;
				}

			} else if (quality_sufficient) {
				fuseYaw(aid_src.innovation, aid_src.observation_variance, aid_src);

			} else {
				aid_src.innovation_rejected = true;
			}

			const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.no_aid_timeout_max);

			if (is_fusion_failing) {
				if ((_nb_ev_yaw_reset_available > 0) && quality_sufficient) {
					// Data seems good, attempt a reset
					//_information_events.flags.reset_yaw_to_vision = true; // TODO
					ECL_WARN("%s fusion failing, resetting", AID_SRC_NAME);
					resetQuatStateYaw(aid_src.observation, aid_src.observation_variance);
					aid_src.time_last_fuse = _time_delayed_us;

					if (_control_status.flags.in_air) {
						_nb_ev_yaw_reset_available--;
					}

				} else if (starting_conditions_passing) {
					// Data seems good, but previous reset did not fix the issue
					// something else must be wrong, declare the sensor faulty and stop the fusion
					//_control_status.flags.ev_yaw_fault = true;
					ECL_WARN("stopping %s fusion, starting conditions failing", AID_SRC_NAME);
					stopEvYawFusion();

				} else {
					// A reset did not fix the issue but all the starting checks are not passing
					// This could be a temporary issue, stop the fusion without declaring the sensor faulty
					ECL_WARN("stopping %s, fusion failing", AID_SRC_NAME);
					stopEvYawFusion();
				}
			}

		} else {
			// Stop fusion but do not declare it faulty
			ECL_WARN("stopping %s fusion, continuing conditions failing", AID_SRC_NAME);
			stopEvYawFusion();
		}

	} else {
		if (starting_conditions_passing) {
			// activate fusion
			if (ev_sample.pos_frame == PositionFrame::LOCAL_FRAME_NED) {

				if (_control_status.flags.yaw_align) {
					ECL_INFO("starting %s fusion", AID_SRC_NAME);

				} else {
					// reset yaw to EV and set yaw_align
					ECL_INFO("starting %s fusion, resetting state", AID_SRC_NAME);
					resetQuatStateYaw(aid_src.observation, aid_src.observation_variance);
					_control_status.flags.yaw_align = true;
				}

				aid_src.time_last_fuse = _time_delayed_us;
				_information_events.flags.starting_vision_yaw_fusion = true;
				_control_status.flags.ev_yaw = true;

			} else if (ev_sample.pos_frame == PositionFrame::LOCAL_FRAME_FRD) {
				// turn on fusion of external vision yaw measurements and disable all other heading fusion
				stopMagFusion();
				stopGpsYawFusion();
				stopGpsFusion();

				ECL_INFO("starting %s fusion, resetting state", AID_SRC_NAME);

				// reset yaw to EV
				resetQuatStateYaw(aid_src.observation, aid_src.observation_variance);
				aid_src.time_last_fuse = _time_delayed_us;

				_information_events.flags.starting_vision_yaw_fusion = true;
				_control_status.flags.yaw_align = false;
				_control_status.flags.ev_yaw = true;
			}

			if (_control_status.flags.ev_yaw) {
				_nb_ev_yaw_reset_available = 5;
			}
		}
	}
}

void Ekf::stopEvYawFusion()
{
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	if (_control_status.flags.ev_yaw) {
		resetEstimatorAidStatus(_aid_src_ev_yaw);

		_control_status.flags.ev_yaw = false;
	}
#endif // CONFIG_EKF2_EXTERNAL_VISION
}
/****************************************************************************
 *
 *   Copyright (c) 2018 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file gps_yaw_fusion.cpp
 * Definition of functions required to use yaw obtained from GPS dual antenna measurements.
 * Equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"
#include "python/ekf_derivation/generated/compute_gnss_yaw_pred_innov_var_and_h.h"

#include <mathlib/mathlib.h>
#include <cstdlib>

void Ekf::updateGpsYaw(const gpsSample &gps_sample)
{
	if (PX4_ISFINITE(gps_sample.yaw)) {

		auto &gnss_yaw = _aid_src_gnss_yaw;
		resetEstimatorAidStatus(gnss_yaw);

		// initially populate for estimator_aid_src_gnss_yaw logging

		// calculate the observed yaw angle of antenna array, converting a from body to antenna yaw measurement
		const float measured_hdg = wrap_pi(gps_sample.yaw + _gps_yaw_offset);

		const float R_YAW = sq(fmaxf(gps_sample.yaw_acc, _params.gps_heading_noise));

		float heading_pred;
		float heading_innov_var;

		{
		Vector24f H;
		sym::ComputeGnssYawPredInnovVarAndH(getStateAtFusionHorizonAsVector(), P, _gps_yaw_offset, R_YAW, FLT_EPSILON, &heading_pred, &heading_innov_var, &H);
		}

		gnss_yaw.observation = measured_hdg;
		gnss_yaw.observation_variance = R_YAW;
		gnss_yaw.innovation = wrap_pi(heading_pred - measured_hdg);
		gnss_yaw.innovation_variance = heading_innov_var;

		gnss_yaw.fusion_enabled = _control_status.flags.gps_yaw;

		gnss_yaw.timestamp_sample = gps_sample.time_us;

		const float innov_gate = math::max(_params.heading_innov_gate, 1.0f);
		setEstimatorAidStatusTestRatio(gnss_yaw, innov_gate);
	}
}

void Ekf::fuseGpsYaw()
{
	auto &gnss_yaw = _aid_src_gnss_yaw;

	if (gnss_yaw.innovation_rejected) {
		_innov_check_fail_status.flags.reject_yaw = true;
		return;
	}

	Vector24f H;

	{
	float heading_pred;
	float heading_innov_var;

	// Note: we recompute innov and innov_var because it doesn't cost much more than just computing H
	// making a separate function just for H uses more flash space without reducing CPU load significantly
	sym::ComputeGnssYawPredInnovVarAndH(getStateAtFusionHorizonAsVector(), P, _gps_yaw_offset, gnss_yaw.observation_variance, FLT_EPSILON, &heading_pred, &heading_innov_var, &H);
	}

	const SparseVector24f<0,1,2,3> Hfusion(H);

	// check if the innovation variance calculation is badly conditioned
	if (gnss_yaw.innovation_variance < gnss_yaw.observation_variance) {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_hdg = true;

		// we reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		ECL_ERR("GPS yaw numerical error - covariance reset");
		return;
	}

	_fault_status.flags.bad_hdg = false;
	_innov_check_fail_status.flags.reject_yaw = false;

	_gnss_yaw_signed_test_ratio_lpf.update(matrix::sign(gnss_yaw.innovation) * gnss_yaw.test_ratio);

	if ((fabsf(_gnss_yaw_signed_test_ratio_lpf.getState()) > 0.2f)
		&& !_control_status.flags.in_air && isTimedOut(gnss_yaw.time_last_fuse, (uint64_t)1e6)) {

		// A constant large signed test ratio is a sign of wrong gyro bias
		// Reset the yaw gyro variance to converge faster and avoid
		// being stuck on a previous bad estimate
		resetZDeltaAngBiasCov();
	}

	// calculate the Kalman gains
	// only calculate gains for states we are using
	Vector24f Kfusion = P * Hfusion / gnss_yaw.innovation_variance;

	const bool is_fused = measurementUpdate(Kfusion, gnss_yaw.innovation_variance, gnss_yaw.innovation);
	_fault_status.flags.bad_hdg = !is_fused;
	gnss_yaw.fused = is_fused;

	if (is_fused) {
		_time_last_heading_fuse = _time_delayed_us;
		gnss_yaw.time_last_fuse = _time_delayed_us;
	}
}

bool Ekf::resetYawToGps(const float gnss_yaw)
{
	// define the predicted antenna array vector and rotate into earth frame
	const Vector3f ant_vec_bf = {cosf(_gps_yaw_offset), sinf(_gps_yaw_offset), 0.0f};
	const Vector3f ant_vec_ef = _R_to_earth * ant_vec_bf;

	// check if antenna array vector is within 30 degrees of vertical and therefore unable to provide a reliable heading
	if (fabsf(ant_vec_ef(2)) > cosf(math::radians(30.0f)))  {
		return false;
	}

	// GPS yaw measurement is alreday compensated for antenna offset in the driver
	const float measured_yaw = gnss_yaw;

	const float yaw_variance = sq(fmaxf(_params.gps_heading_noise, 1.e-2f));
	resetQuatStateYaw(measured_yaw, yaw_variance);

	_aid_src_gnss_yaw.time_last_fuse = _time_delayed_us;
	_gnss_yaw_signed_test_ratio_lpf.reset(0.f);

	return true;
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file optical_flow_control.cpp
 * Control functions for optical flow fusion
 */

#include "ekf.h"

void Ekf::controlOpticalFlowFusion(const imuSample &imu_delayed)
{
	if (_flow_buffer) {
		// We don't fuse flow data immediately because we have to wait for the mid integration point to fall behind the fusion time horizon.
		// This means we stop looking for new data until the old data has been fused, unless we are not fusing optical flow,
		// in this case we need to empty the buffer
		if (!_flow_data_ready || (!_control_status.flags.opt_flow && !_hagl_sensor_status.flags.flow)) {
			_flow_data_ready = _flow_buffer->pop_first_older_than(imu_delayed.time_us, &_flow_sample_delayed);
		}
	}

	// Check if on ground motion is un-suitable for use of optical flow
	if (!_control_status.flags.in_air) {
		updateOnGroundMotionForOpticalFlowChecks();

	} else {
		resetOnGroundMotionForOpticalFlowChecks();
	}

	// Accumulate autopilot gyro data across the same time interval as the flow sensor
	const Vector3f delta_angle(imu_delayed.delta_ang - (getGyroBias() * imu_delayed.delta_ang_dt));
	if (_delta_time_of < 0.2f) {
		_imu_del_ang_of += delta_angle;
		_delta_time_of += imu_delayed.delta_ang_dt;

	} else {
		// reset the accumulators if the time interval is too large
		_imu_del_ang_of = delta_angle;
		_delta_time_of = imu_delayed.delta_ang_dt;
	}

	if (_flow_data_ready) {
		int32_t min_quality = _params.flow_qual_min;
		if (!_control_status.flags.in_air) {
			min_quality = _params.flow_qual_min_gnd;
		}

		const bool is_quality_good = (_flow_sample_delayed.quality >= min_quality);
		const bool is_magnitude_good = !_flow_sample_delayed.flow_xy_rad.longerThan(_flow_sample_delayed.dt * _flow_max_rate);
		const bool is_tilt_good = (_R_to_earth(2, 2) > _params.range_cos_max_tilt);

		const float delta_time_min = fmaxf(0.7f * _delta_time_of, 0.001f);
		const float delta_time_max = fminf(1.3f * _delta_time_of, 0.2f);
		bool is_delta_time_good = _flow_sample_delayed.dt >= delta_time_min && _flow_sample_delayed.dt <= delta_time_max;

		if (!is_delta_time_good && (_flow_sample_delayed.dt > FLT_EPSILON)) {

			if (fabsf(imu_delayed.delta_ang_dt - _flow_sample_delayed.dt) < 0.1f) {
				// reset accumulators to current IMU
				_imu_del_ang_of = delta_angle;
				_delta_time_of = imu_delayed.delta_ang_dt;

				is_delta_time_good = true;
			}

			if (is_quality_good && !is_delta_time_good) {
				ECL_DEBUG("Optical flow: bad delta time: OF dt %.6f s (min: %.3f, max: %.3f), IMU dt %.6f s",
					  (double)_flow_sample_delayed.dt, (double)delta_time_min, (double)delta_time_max,
					  (double)imu_delayed.delta_ang_dt);
			}
		}

		const bool is_body_rate_comp_available = calcOptFlowBodyRateComp();

		// don't allow invalid flow gyro_xyz to propagate
		if (!_flow_sample_delayed.gyro_xyz.isAllFinite()) {
			_flow_sample_delayed.gyro_xyz.zero();
		}

		if (is_quality_good
		    && is_magnitude_good
		    && is_tilt_good
		    && is_body_rate_comp_available
		    && is_delta_time_good) {
			// compensate for body motion to give a LOS rate
			_flow_compensated_XY_rad = _flow_sample_delayed.flow_xy_rad - _flow_sample_delayed.gyro_xyz.xy();

		} else if (!_control_status.flags.in_air) {

			if (!is_delta_time_good) {
				// handle special case of SITL and PX4Flow where dt is forced to
				// zero when the quaity is 0
				_flow_sample_delayed.dt = delta_time_min;
			}

			// when on the ground with poor flow quality,
			// assume zero ground relative velocity and LOS rate
			_flow_compensated_XY_rad.setZero();

		} else {
			// don't use this flow data and wait for the next data to arrive
			_flow_data_ready = false;
			_flow_compensated_XY_rad.setZero();
		}

		updateOptFlow(_aid_src_optical_flow);

	} else {
		_flow_compensated_XY_rad.setZero();
	}

	// New optical flow data is available and is ready to be fused when the midpoint of the sample falls behind the fusion time horizon
	if (_flow_data_ready) {

		// Inhibit flow use if motion is un-suitable or we have good quality GPS
		// Apply hysteresis to prevent rapid mode switching
		const float gps_err_norm_lim = _control_status.flags.opt_flow ? 0.7f : 1.0f;

		// Check if we are in-air and require optical flow to control position drift
		const bool is_flow_required = _control_status.flags.in_air
					      && (_control_status.flags.inertial_dead_reckoning // is doing inertial dead-reckoning so must constrain drift urgently
						  || isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow)
						  || (_control_status.flags.gps && (_gps_error_norm > gps_err_norm_lim))); // is using GPS, but GPS is bad

		// inhibit use of optical flow if motion is unsuitable and we are not reliant on it for flight navigation
		const bool preflight_motion_not_ok = !_control_status.flags.in_air
						     && ((_time_delayed_us > (_time_good_motion_us + (uint64_t)1E5))
								     || (_time_delayed_us < (_time_bad_motion_us + (uint64_t)5E6)));
		const bool flight_condition_not_ok = _control_status.flags.in_air && !isTerrainEstimateValid();

		const bool inhibit_flow_use = ((preflight_motion_not_ok || flight_condition_not_ok) && !is_flow_required)
				    || !_control_status.flags.tilt_align;

		// Handle cases where we are using optical flow but we should not use it anymore
		if (_control_status.flags.opt_flow) {
			if (!(_params.flow_ctrl == 1)
			    || inhibit_flow_use) {

				stopFlowFusion();
				return;
			}
		}

		// optical flow fusion mode selection logic
		if ((_params.flow_ctrl == 1) // optical flow has been selected by the user
		    && !_control_status.flags.opt_flow // we are not yet using flow data
		    && !inhibit_flow_use) {

			// set the flag and reset the fusion timeout
			ECL_INFO("starting optical flow fusion");

			// if we are not using GPS or external vision aiding, then the velocity and position states and covariances need to be set
			if (!isHorizontalAidingActive()) {
				ECL_INFO("reset velocity to flow");
				_information_events.flags.reset_vel_to_flow = true;
				resetHorizontalVelocityTo(_flow_vel_ne, calcOptFlowMeasVar(_flow_sample_delayed));

				// reset position, estimate is relative to initial position in this mode, so we start with zero error
				if (!_control_status.flags.in_air) {
					ECL_INFO("reset position to zero");
					resetHorizontalPositionTo(Vector2f(0.f, 0.f), 0.f);
					_last_known_pos.xy() = _state.pos.xy();

				} else {
					_information_events.flags.reset_pos_to_last_known = true;
					ECL_INFO("reset position to last known (%.3f, %.3f)", (double)_last_known_pos(0), (double)_last_known_pos(1));
					resetHorizontalPositionTo(_last_known_pos.xy(), 0.f);
				}
			}

			_aid_src_optical_flow.time_last_fuse = _time_delayed_us;
			_control_status.flags.opt_flow = true;

			return;
		}

		if (_control_status.flags.opt_flow) {
			// Wait until the midpoint of the flow sample has fallen behind the fusion time horizon
			if (_time_delayed_us > (_flow_sample_delayed.time_us - uint32_t(1e6f * _flow_sample_delayed.dt) / 2)) {
				// Fuse optical flow LOS rate observations into the main filter only if height above ground has been updated recently
				// but use a relaxed time criteria to enable it to coast through bad range finder data
				if (isRecent(_time_last_hagl_fuse, (uint64_t)10e6)) {
					fuseOptFlow();
					_last_known_pos.xy() = _state.pos.xy();
				}

				_flow_data_ready = false;
			}

			// handle the case when we have optical flow, are reliant on it, but have not been using it for an extended period
			if (isTimedOut(_aid_src_optical_flow.time_last_fuse, _params.no_aid_timeout_max)
			    && !isOtherSourceOfHorizontalAidingThan(_control_status.flags.opt_flow)) {

				ECL_INFO("reset velocity to flow");
				_information_events.flags.reset_vel_to_flow = true;
				resetHorizontalVelocityTo(_flow_vel_ne, calcOptFlowMeasVar(_flow_sample_delayed));

				// reset position, estimate is relative to initial position in this mode, so we start with zero error
				ECL_INFO("reset position to last known (%.3f, %.3f)", (double)_last_known_pos(0), (double)_last_known_pos(1));
				_information_events.flags.reset_pos_to_last_known = true;
				resetHorizontalPositionTo(_last_known_pos.xy(), 0.f);

				_aid_src_optical_flow.time_last_fuse = _time_delayed_us;
			}
		}

	} else if (_control_status.flags.opt_flow && !isRecent(_flow_sample_delayed.time_us, (uint64_t)10e6)) {

		stopFlowFusion();
	}
}

void Ekf::updateOnGroundMotionForOpticalFlowChecks()
{
	// When on ground check if the vehicle is being shaken or moved in a way that could cause a loss of navigation
	const float accel_norm = _accel_vec_filt.norm();

	const bool motion_is_excessive = ((accel_norm > (CONSTANTS_ONE_G * 1.5f)) // upper g limit
					  || (accel_norm < (CONSTANTS_ONE_G * 0.5f)) // lower g limit
					  || (_ang_rate_magnitude_filt > _flow_max_rate) // angular rate exceeds flow sensor limit
					  || (_R_to_earth(2, 2) < cosf(math::radians(30.0f)))); // tilted excessively

	if (motion_is_excessive) {
		_time_bad_motion_us = _time_delayed_us;

	} else {
		_time_good_motion_us = _time_delayed_us;
	}
}

void Ekf::resetOnGroundMotionForOpticalFlowChecks()
{
	_time_bad_motion_us = 0;
	_time_good_motion_us = _time_delayed_us;
}

void Ekf::stopFlowFusion()
{
	if (_control_status.flags.opt_flow) {
		ECL_INFO("stopping optical flow fusion");
		_control_status.flags.opt_flow = false;

		resetEstimatorAidStatus(_aid_src_optical_flow);
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file vel_pos_fusion.cpp
 * Function for fusing gps and baro measurements/
 * equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 * @author Siddharth Bharat Purohit <siddharthbharatpurohit@gmail.com>
 *
 */

#include "ekf.h"

#include <mathlib/mathlib.h>
#include <float.h>
#include "python/ekf_derivation/generated/compute_flow_xy_innov_var_and_hx.h"
#include "python/ekf_derivation/generated/compute_flow_y_innov_var_and_h.h"
#include "utils.hpp"

void Ekf::updateOptFlow(estimator_aid_source2d_s &aid_src)
{
	resetEstimatorAidStatus(aid_src);
	aid_src.timestamp_sample = _flow_sample_delayed.time_us;

	const Vector2f vel_body = predictFlowVelBody();
	const float range = predictFlowRange();

	// calculate optical LOS rates using optical flow rates that have had the body angular rate contribution removed
	// correct for gyro bias errors in the data used to do the motion compensation
	// Note the sign convention used: A positive LOS rate is a RH rotation of the scene about that axis.
	const Vector2f opt_flow_rate = _flow_compensated_XY_rad / _flow_sample_delayed.dt;

	// compute the velocities in body and local frames from corrected optical flow measurement for logging only
	_flow_vel_body(0) = -opt_flow_rate(1) * range;
	_flow_vel_body(1) =  opt_flow_rate(0) * range;
	_flow_vel_ne = Vector2f(_R_to_earth * Vector3f(_flow_vel_body(0), _flow_vel_body(1), 0.f));

	aid_src.observation[0] = opt_flow_rate(0); // flow around the X axis
	aid_src.observation[1] = opt_flow_rate(1); // flow around the Y axis

	aid_src.innovation[0] =  (vel_body(1) / range) - aid_src.observation[0];
	aid_src.innovation[1] = (-vel_body(0) / range) - aid_src.observation[1];

	// calculate the optical flow observation variance
	const float R_LOS = calcOptFlowMeasVar(_flow_sample_delayed);
	aid_src.observation_variance[0] = R_LOS;
	aid_src.observation_variance[1] = R_LOS;

	const Vector24f state_vector = getStateAtFusionHorizonAsVector();

	Vector2f innov_var;
	Vector24f H;
	sym::ComputeFlowXyInnovVarAndHx(state_vector, P, range, R_LOS, FLT_EPSILON, &innov_var, &H);
	innov_var.copyTo(aid_src.innovation_variance);

	// run the innovation consistency check and record result
	setEstimatorAidStatusTestRatio(aid_src, math::max(_params.flow_innov_gate, 1.f));
}

void Ekf::fuseOptFlow()
{
	_aid_src_optical_flow.fusion_enabled = true;

	const float R_LOS = _aid_src_optical_flow.observation_variance[0];

	// calculate the height above the ground of the optical flow camera. Since earth frame is NED
	// a positive offset in earth frame leads to a smaller height above the ground.
	float range = predictFlowRange();

	const Vector24f state_vector = getStateAtFusionHorizonAsVector();

	Vector2f innov_var;
	Vector24f H;
	sym::ComputeFlowXyInnovVarAndHx(state_vector, P, range, R_LOS, FLT_EPSILON, &innov_var, &H);
	innov_var.copyTo(_aid_src_optical_flow.innovation_variance);

	if ((_aid_src_optical_flow.innovation_variance[0] < R_LOS)
	    || (_aid_src_optical_flow.innovation_variance[1] < R_LOS)) {
		// we need to reinitialise the covariance matrix and abort this fusion step
		ECL_ERR("Opt flow error - covariance reset");
		initialiseCovariance();
		return;
	}

	// run the innovation consistency check and record result
	setEstimatorAidStatusTestRatio(_aid_src_optical_flow, math::max(_params.flow_innov_gate, 1.f));

	_innov_check_fail_status.flags.reject_optflow_X = (_aid_src_optical_flow.test_ratio[0] > 1.f);
	_innov_check_fail_status.flags.reject_optflow_Y = (_aid_src_optical_flow.test_ratio[1] > 1.f);

	// if either axis fails we abort the fusion
	if (_aid_src_optical_flow.innovation_rejected) {
		return;
	}

	bool fused[2] {false, false};

	// fuse observation axes sequentially
	for (uint8_t index = 0; index <= 1; index++) {
		if (index == 0) {
			// everything was already computed above

		} else if (index == 1) {
			// recalculate innovation variance because state covariances have changed due to previous fusion (linearise using the same initial state for all axes)
			sym::ComputeFlowYInnovVarAndH(state_vector, P, range, R_LOS, FLT_EPSILON, &_aid_src_optical_flow.innovation_variance[1], &H);

			// recalculate the innovation using the updated state
			const Vector2f vel_body = predictFlowVelBody();
			range = predictFlowRange();
			_aid_src_optical_flow.innovation[1] = (-vel_body(0) / range) - _aid_src_optical_flow.observation[1];

			if (_aid_src_optical_flow.innovation_variance[1] < R_LOS) {
				// we need to reinitialise the covariance matrix and abort this fusion step
				ECL_ERR("Opt flow error - covariance reset");
				initialiseCovariance();
				return;
			}
		}

		SparseVector24f<0,1,2,3,4,5,6> Hfusion(H);
		Vector24f Kfusion = P * Hfusion / _aid_src_optical_flow.innovation_variance[index];

		if (measurementUpdate(Kfusion, _aid_src_optical_flow.innovation_variance[index], _aid_src_optical_flow.innovation[index])) {
			fused[index] = true;
		}
	}

	_fault_status.flags.bad_optflow_X = !fused[0];
	_fault_status.flags.bad_optflow_Y = !fused[1];

	if (fused[0] && fused[1]) {
		_aid_src_optical_flow.time_last_fuse = _time_delayed_us;
		_aid_src_optical_flow.fused = true;
	}
}

float Ekf::predictFlowRange()
{
	// calculate the sensor position relative to the IMU
	const Vector3f pos_offset_body = _params.flow_pos_body - _params.imu_pos_body;

	// calculate the sensor position relative to the IMU in earth frame
	const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;

	// calculate the height above the ground of the optical flow camera. Since earth frame is NED
	// a positive offset in earth frame leads to a smaller height above the ground.
	const float height_above_gnd_est = math::max(_terrain_vpos - _state.pos(2) - pos_offset_earth(2), fmaxf(_params.rng_gnd_clearance, 0.01f));

	// calculate range from focal point to centre of image
	return height_above_gnd_est / _R_to_earth(2, 2); // absolute distance to the frame region in view
}

Vector2f Ekf::predictFlowVelBody()
{
	// calculate the sensor position relative to the IMU
	const Vector3f pos_offset_body = _params.flow_pos_body - _params.imu_pos_body;

	// calculate the velocity of the sensor relative to the imu in body frame
	// Note: _flow_sample_delayed.gyro_xyz is the negative of the body angular velocity, thus use minus sign
	const Vector3f vel_rel_imu_body = Vector3f(-_flow_sample_delayed.gyro_xyz / _flow_sample_delayed.dt) % pos_offset_body;

	// calculate the velocity of the sensor in the earth frame
	const Vector3f vel_rel_earth = _state.vel + _R_to_earth * vel_rel_imu_body;

	// rotate into body frame
	return _state.quat_nominal.rotateVectorInverse(vel_rel_earth).xy();
}


// calculate optical flow body angular rate compensation
// returns false if bias corrected body rate data is unavailable
bool Ekf::calcOptFlowBodyRateComp()
{
	bool is_body_rate_comp_available = false;
	const bool use_flow_sensor_gyro = _flow_sample_delayed.gyro_xyz.isAllFinite();

	if (use_flow_sensor_gyro) {

		// if accumulation time differences are not excessive and accumulation time is adequate
		// compare the optical flow and and navigation rate data and calculate a bias error
		if ((_delta_time_of > FLT_EPSILON)
		    && (_flow_sample_delayed.dt > FLT_EPSILON)
		    && (fabsf(_delta_time_of - _flow_sample_delayed.dt) < 0.1f)) {

			const Vector3f reference_body_rate(_imu_del_ang_of * (1.0f / _delta_time_of));

			const Vector3f measured_body_rate(_flow_sample_delayed.gyro_xyz * (1.0f / _flow_sample_delayed.dt));

			// calculate the bias estimate using  a combined LPF and spike filter
			_flow_gyro_bias = _flow_gyro_bias * 0.99f + matrix::constrain(measured_body_rate - reference_body_rate, -0.1f, 0.1f) * 0.01f;

			// apply gyro bias
			_flow_sample_delayed.gyro_xyz -= (_flow_gyro_bias * _flow_sample_delayed.dt);

			is_body_rate_comp_available = true;
		}

	} else {
		// Use the EKF gyro data if optical flow sensor gyro data is not available
		// for clarification of the sign see definition of flowSample and imuSample in common.h
		if ((_delta_time_of > FLT_EPSILON)
		    && (_flow_sample_delayed.dt > FLT_EPSILON)) {

			_flow_sample_delayed.gyro_xyz = -_imu_del_ang_of / _delta_time_of * _flow_sample_delayed.dt;
			_flow_gyro_bias.zero();

			is_body_rate_comp_available = true;
		}
	}

	// reset the accumulators
	_imu_del_ang_of.setZero();
	_delta_time_of = 0.0f;
	return is_body_rate_comp_available;
}

// calculate the measurement variance for the optical flow sensor (rad/sec)^2
float Ekf::calcOptFlowMeasVar(const flowSample &flow_sample)
{
	// calculate the observation noise variance - scaling noise linearly across flow quality range
	const float R_LOS_best = fmaxf(_params.flow_noise, 0.05f);
	const float R_LOS_worst = fmaxf(_params.flow_noise_qual_min, 0.05f);

	// calculate a weighting that varies between 1 when flow quality is best and 0 when flow quality is worst
	float weighting = (255.f - (float)_params.flow_qual_min);

	if (weighting >= 1.f) {
		weighting = math::constrain((float)(flow_sample.quality - _params.flow_qual_min) / weighting, 0.f, 1.f);

	} else {
		weighting = 0.0f;
	}

	// take the weighted average of the observation noise for the best and wort flow quality
	const float R_LOS = sq(R_LOS_best * weighting + R_LOS_worst * (1.f - weighting));

	return R_LOS;
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file range_finder_consistency_check.cpp
 */

#include "range_finder_consistency_check.hpp"

void RangeFinderConsistencyCheck::update(float dist_bottom, float dist_bottom_var, float vz, float vz_var, bool horizontal_motion, uint64_t time_us)
{
	if (horizontal_motion) {
		_time_last_horizontal_motion = time_us;
	}

	const float dt = static_cast<float>(time_us - _time_last_update_us) * 1e-6f;

	if ((_time_last_update_us == 0)
	    || (dt < 0.001f) || (dt > 0.5f)) {
		_time_last_update_us = time_us;
		_dist_bottom_prev = dist_bottom;
		return;
	}

	const float vel_bottom = (dist_bottom - _dist_bottom_prev) / dt;
	_innov = -vel_bottom - vz; // vel_bottom is +up while vz is +down

	const float var = 2.f * dist_bottom_var / (dt * dt); // Variance of the time derivative of a random variable: var(dz/dt) = 2*var(z) / dt^2
	_innov_var = var + vz_var;

	const float normalized_innov_sq = (_innov * _innov) / _innov_var;
	_test_ratio = normalized_innov_sq / (_gate * _gate);
	_signed_test_ratio_lpf.setParameters(dt, _signed_test_ratio_tau);
	const float signed_test_ratio = matrix::sign(_innov) * _test_ratio;
	_signed_test_ratio_lpf.update(signed_test_ratio);

	updateConsistency(vz, time_us);

	_time_last_update_us = time_us;
	_dist_bottom_prev = dist_bottom;
}

void RangeFinderConsistencyCheck::updateConsistency(float vz, uint64_t time_us)
{
	if (fabsf(vz) < _min_vz_for_valid_consistency) {
		// We can only check consistency if there is vertical motion
		return;
	}

	if (fabsf(_signed_test_ratio_lpf.getState()) >= 1.f) {
		if ((time_us - _time_last_horizontal_motion) > _signed_test_ratio_tau) {
			_is_kinematically_consistent = false;
			_time_last_inconsistent_us = time_us;
		}

	} else {
		if (_test_ratio < 1.f
		   && ((time_us - _time_last_inconsistent_us) > _consistency_hyst_time_us)) {
			_is_kinematically_consistent = true;
		}
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file range_height_control.cpp
 * Control functions for ekf range finder height fusion
 */

#include "ekf.h"

void Ekf::controlRangeHeightFusion()
{
	static constexpr const char *HGT_SRC_NAME = "RNG";

	bool rng_data_ready = false;

	if (_range_buffer) {
		// Get range data from buffer and check validity
		rng_data_ready = _range_buffer->pop_first_older_than(_time_delayed_us, _range_sensor.getSampleAddress());
		_range_sensor.setDataReadiness(rng_data_ready);

		// update range sensor angle parameters in case they have changed
		_range_sensor.setPitchOffset(_params.rng_sens_pitch);
		_range_sensor.setCosMaxTilt(_params.range_cos_max_tilt);
		_range_sensor.setQualityHysteresis(_params.range_valid_quality_s);

		_range_sensor.runChecks(_time_delayed_us, _R_to_earth);

		if (_range_sensor.isDataHealthy()) {
			// correct the range data for position offset relative to the IMU
			const Vector3f pos_offset_body = _params.rng_pos_body - _params.imu_pos_body;
			const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
			_range_sensor.setRange(_range_sensor.getRange() + pos_offset_earth(2) / _range_sensor.getCosTilt());

			if (_control_status.flags.in_air) {
				const bool horizontal_motion = _control_status.flags.fixed_wing
								|| (sq(_state.vel(0)) + sq(_state.vel(1)) > fmaxf(P(4, 4) + P(5, 5), 0.1f));

				const float dist_dependant_var = sq(_params.range_noise_scaler * _range_sensor.getDistBottom());
				const float var = sq(_params.range_noise) + dist_dependant_var;

				_rng_consistency_check.setGate(_params.range_kin_consistency_gate);
				_rng_consistency_check.update(_range_sensor.getDistBottom(), math::max(var, 0.001f), _state.vel(2), P(6, 6), horizontal_motion, _time_delayed_us);
			}

		} else {
			// If we are supposed to be using range finder data as the primary height sensor, have bad range measurements
			// and are on the ground, then synthesise a measurement at the expected on ground value
			if (!_control_status.flags.in_air
			    && _range_sensor.isRegularlySendingData()
			    && _range_sensor.isDataReady()) {

				_range_sensor.setRange(_params.rng_gnd_clearance);
				_range_sensor.setValidity(true); // bypass the checks
			}
		}

		_control_status.flags.rng_kin_consistent = _rng_consistency_check.isKinematicallyConsistent();

	} else {
		return;
	}

	auto &aid_src = _aid_src_rng_hgt;
	HeightBiasEstimator &bias_est = _rng_hgt_b_est;

	bias_est.predict(_dt_ekf_avg);

	if (rng_data_ready && _range_sensor.getSampleAddress()) {

		const float measurement = math::max(_range_sensor.getDistBottom(), _params.rng_gnd_clearance);
		const float measurement_var = sq(_params.range_noise) + sq(_params.range_noise_scaler * _range_sensor.getDistBottom());

		const float innov_gate = math::max(_params.range_innov_gate, 1.f);

		const bool measurement_valid = PX4_ISFINITE(measurement) && PX4_ISFINITE(measurement_var);

		// vertical position innovation - baro measurement has opposite sign to earth z axis
		updateVerticalPositionAidSrcStatus(_range_sensor.getSampleAddress()->time_us,
						   -(measurement - bias_est.getBias()),
						   measurement_var + bias_est.getBiasVar(),
						   innov_gate,
						   aid_src);

		// update the bias estimator before updating the main filter but after
		// using its current state to compute the vertical position innovation
		if (measurement_valid && _range_sensor.isDataHealthy()) {
			bias_est.setMaxStateNoise(sqrtf(measurement_var));
			bias_est.setProcessNoiseSpectralDensity(_params.rng_hgt_bias_nsd);
			bias_est.fuseBias(measurement - (-_state.pos(2)), measurement_var + P(9, 9));
		}

		// determine if we should use height aiding
		const bool do_conditional_range_aid = (_params.rng_ctrl == RngCtrl::CONDITIONAL) && isConditionalRangeAidSuitable();
		const bool continuing_conditions_passing = ((_params.rng_ctrl == RngCtrl::ENABLED) || do_conditional_range_aid)
				&& measurement_valid
				&& _range_sensor.isDataHealthy();

		const bool starting_conditions_passing = continuing_conditions_passing
				&& isNewestSampleRecent(_time_last_range_buffer_push, 2 * RNG_MAX_INTERVAL)
				&& _range_sensor.isRegularlySendingData();

		if (_control_status.flags.rng_hgt) {
			aid_src.fusion_enabled = true;

			if (continuing_conditions_passing) {

				fuseVerticalPosition(aid_src);

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.hgt_fusion_timeout_max);

				if (isHeightResetRequired()) {
					// All height sources are failing
					ECL_WARN("%s height fusion reset required, all height sources failing", HGT_SRC_NAME);

					_information_events.flags.reset_hgt_to_rng = true;
					resetVerticalPositionTo(-(measurement - bias_est.getBias()));
					bias_est.setBias(_state.pos(2) + measurement);

					// reset vertical velocity
					resetVerticalVelocityToZero();

					aid_src.time_last_fuse = _time_delayed_us;

				} else if (is_fusion_failing) {
					// Some other height source is still working
					ECL_WARN("stopping %s height fusion, fusion failing", HGT_SRC_NAME);
					stopRngHgtFusion();
					_control_status.flags.rng_fault = true;
					_range_sensor.setFaulty();
				}

			} else {
				ECL_WARN("stopping %s height fusion, continuing conditions failing", HGT_SRC_NAME);
				stopRngHgtFusion();
			}

		} else {
			if (starting_conditions_passing) {
				if ((_params.height_sensor_ref == HeightSensor::RANGE) && (_params.rng_ctrl == RngCtrl::CONDITIONAL)) {
					// Range finder is used while hovering to stabilize the height estimate. Don't reset but use it as height reference.
					ECL_INFO("starting conditional %s height fusion", HGT_SRC_NAME);
					_height_sensor_ref = HeightSensor::RANGE;
					bias_est.setBias(_state.pos(2) + measurement);

				} else if ((_params.height_sensor_ref == HeightSensor::RANGE) && (_params.rng_ctrl != RngCtrl::CONDITIONAL)) {
					// Range finder is the primary height source, the ground is now the datum used
					// to compute the local vertical position
					ECL_INFO("starting %s height fusion, resetting height", HGT_SRC_NAME);
					_height_sensor_ref = HeightSensor::RANGE;

					_information_events.flags.reset_hgt_to_rng = true;
					resetVerticalPositionTo(-measurement, measurement_var);
					bias_est.reset();

				} else {
					ECL_INFO("starting %s height fusion", HGT_SRC_NAME);
					bias_est.setBias(_state.pos(2) + measurement);
				}

				aid_src.time_last_fuse = _time_delayed_us;
				bias_est.setFusionActive();
				_control_status.flags.rng_hgt = true;
			}
		}

	} else if (_control_status.flags.rng_hgt
		   && !isNewestSampleRecent(_time_last_range_buffer_push, 2 * RNG_MAX_INTERVAL)) {
		// No data anymore. Stop until it comes back.
		ECL_WARN("stopping %s height fusion, no data", HGT_SRC_NAME);
		stopRngHgtFusion();
	}
}

bool Ekf::isConditionalRangeAidSuitable()
{
	if (_control_status.flags.in_air
	    && _range_sensor.isHealthy()
	    && isTerrainEstimateValid()) {
		// check if we can use range finder measurements to estimate height, use hysteresis to avoid rapid switching
		// Note that the 0.7 coefficients and the innovation check are arbitrary values but work well in practice
		float range_hagl_max = _params.max_hagl_for_range_aid;
		float max_vel_xy = _params.max_vel_for_range_aid;

		const float hagl_test_ratio = (_hagl_innov * _hagl_innov / (sq(_params.range_aid_innov_gate) * _hagl_innov_var));

		bool is_hagl_stable = (hagl_test_ratio < 1.f);

		if (!_control_status.flags.rng_hgt) {
			range_hagl_max = 0.7f * _params.max_hagl_for_range_aid;
			max_vel_xy = 0.7f * _params.max_vel_for_range_aid;
			is_hagl_stable = (hagl_test_ratio < 0.01f);
		}

		const float range_hagl = _terrain_vpos - _state.pos(2);

		const bool is_in_range = (range_hagl < range_hagl_max);

		bool is_below_max_speed = true;

		if (isHorizontalAidingActive()) {
			is_below_max_speed = !_state.vel.xy().longerThan(max_vel_xy);
		}

		return is_in_range && is_hagl_stable && is_below_max_speed;
	}

	return false;
}

void Ekf::stopRngHgtFusion()
{
	if (_control_status.flags.rng_hgt) {

		if (_height_sensor_ref == HeightSensor::RANGE) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}

		_rng_hgt_b_est.setFusionInactive();
		resetEstimatorAidStatus(_aid_src_rng_hgt);

		_control_status.flags.rng_hgt = false;
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2020 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file sensor_range_finder.cpp
 *
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 *
 */

#include "sensor_range_finder.hpp"

namespace estimator
{
namespace sensor
{

void SensorRangeFinder::runChecks(const uint64_t current_time_us, const Dcmf &R_to_earth)
{
	updateSensorToEarthRotation(R_to_earth);
	updateValidity(current_time_us);
}

void SensorRangeFinder::updateSensorToEarthRotation(const Dcmf &R_to_earth)
{
	// calculate 2,2 element of rotation matrix from sensor frame to earth frame
	// this is required for use of range finder and flow data
	_cos_tilt_rng_to_earth = R_to_earth(2, 0) * _sin_pitch_offset + R_to_earth(2, 2) * _cos_pitch_offset;
}

void SensorRangeFinder::updateValidity(uint64_t current_time_us)
{
	updateDtDataLpf(current_time_us);

	if (_is_faulty || isSampleOutOfDate(current_time_us) || !isDataContinuous()) {
		_is_sample_valid = false;
		_is_regularly_sending_data = false;
		return;
	}

	_is_regularly_sending_data = true;

	// Don't run the checks unless we have retrieved new data from the buffer
	if (_is_sample_ready) {
		_is_sample_valid = false;

		if (_sample.quality == 0) {
			_time_bad_quality_us = current_time_us;

		} else if (current_time_us - _time_bad_quality_us > _quality_hyst_us) {
			// We did not receive bad quality data for some time

			if (isTiltOk() && isDataInRange()) {
				updateStuckCheck();

				if (!_is_stuck) {
					_is_sample_valid = true;
					_time_last_valid_us = _sample.time_us;
				}
			}
		}
	}
}

void SensorRangeFinder::updateDtDataLpf(uint64_t current_time_us)
{
	// Calculate a first order IIR low-pass filtered time of arrival between samples using a 2 second time constant.
	float alpha = 0.5f * _dt_update;
	_dt_data_lpf = _dt_data_lpf * (1.0f - alpha) + alpha * (current_time_us - _sample.time_us);

	// Apply spike protection to the filter state.
	_dt_data_lpf = fminf(_dt_data_lpf, 4e6f);
}

inline bool SensorRangeFinder::isSampleOutOfDate(uint64_t current_time_us) const
{
	return (current_time_us - _sample.time_us) > 2 * RNG_MAX_INTERVAL;
}

inline bool SensorRangeFinder::isDataInRange() const
{
	return (_sample.rng >= _rng_valid_min_val) && (_sample.rng <= _rng_valid_max_val);
}

void SensorRangeFinder::updateStuckCheck()
{
	// Check for "stuck" range finder measurements when range was not valid for certain period
	// This handles a failure mode observed with some lidar sensors
	if (((_sample.time_us - _time_last_valid_us) > (uint64_t)10e6)) {

		// require a variance of rangefinder values to check for "stuck" measurements
		if (_stuck_max_val - _stuck_min_val > _stuck_threshold) {
			_stuck_min_val = 0.0f;
			_stuck_max_val = 0.0f;
			_is_stuck = false;

		} else {
			if (_sample.rng > _stuck_max_val) {
				_stuck_max_val = _sample.rng;
			}

			if (_stuck_min_val < 0.1f || _sample.rng < _stuck_min_val) {
				_stuck_min_val = _sample.rng;
			}

			_is_stuck = true;
		}
	}
}

} // namespace sensor
} // namespace estimator
/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file terrain_estimator.cpp
 * Function for fusing rangefinder and optical flow measurements
 * to estimate terrain vertical position
 */

#include "ekf.h"
#include "python/ekf_derivation/generated/terr_est_compute_flow_xy_innov_var_and_hx.h"
#include "python/ekf_derivation/generated/terr_est_compute_flow_y_innov_var_and_h.h"

#include <mathlib/mathlib.h>

void Ekf::initHagl()
{
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	stopHaglFlowFusion();
#endif // CONFIG_EKF2_OPTICAL_FLOW

	stopHaglRngFusion();

	// assume a ground clearance
	_terrain_vpos = _state.pos(2) + _params.rng_gnd_clearance;

	// use the ground clearance value as our uncertainty
	_terrain_var = sq(_params.rng_gnd_clearance);
}

void Ekf::runTerrainEstimator(const imuSample &imu_delayed)
{
	// If we are on ground, store the local position and time to use as a reference
	if (!_control_status.flags.in_air) {
		_last_on_ground_posD = _state.pos(2);
		_control_status.flags.rng_fault = false;
	}

	predictHagl(imu_delayed);

	controlHaglRngFusion();
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	controlHaglFlowFusion();
#endif // CONFIG_EKF2_OPTICAL_FLOW
	controlHaglFakeFusion();

	// constrain _terrain_vpos to be a minimum of _params.rng_gnd_clearance larger than _state.pos(2)
	if (_terrain_vpos - _state.pos(2) < _params.rng_gnd_clearance) {
		_terrain_vpos = _params.rng_gnd_clearance + _state.pos(2);
	}
}

void Ekf::predictHagl(const imuSample &imu_delayed)
{
	// predict the state variance growth where the state is the vertical position of the terrain underneath the vehicle

	// process noise due to errors in vehicle height estimate
	_terrain_var += sq(imu_delayed.delta_vel_dt * _params.terrain_p_noise);

	// process noise due to terrain gradient
	_terrain_var += sq(imu_delayed.delta_vel_dt * _params.terrain_gradient)
			* (sq(_state.vel(0)) + sq(_state.vel(1)));

	// limit the variance to prevent it becoming badly conditioned
	_terrain_var = math::constrain(_terrain_var, 0.0f, 1e4f);
}

void Ekf::controlHaglRngFusion()
{
	if (!(_params.terrain_fusion_mode & TerrainFusionMask::TerrainFuseRangeFinder)
	    || _control_status.flags.rng_fault) {

		stopHaglRngFusion();
		return;
	}

	if (_range_sensor.isDataHealthy()) {
		const bool continuing_conditions_passing = _control_status.flags.in_air && _rng_consistency_check.isKinematicallyConsistent();
		//const bool continuing_conditions_passing = _control_status.flags.in_air && !_control_status.flags.rng_hgt; // TODO: should not be fused when using range height
		const bool starting_conditions_passing = continuing_conditions_passing && _range_sensor.isRegularlySendingData() && (_rng_consistency_check.getTestRatio() < 1.f);

		_time_last_healthy_rng_data = _time_delayed_us;

		if (_hagl_sensor_status.flags.range_finder) {
			if (continuing_conditions_passing) {
				fuseHaglRng();

				// We have been rejecting range data for too long
				const uint64_t timeout = static_cast<uint64_t>(_params.terrain_timeout * 1e6f);
				const bool is_fusion_failing = isTimedOut(_time_last_hagl_fuse, timeout);

				if (is_fusion_failing) {
					if (_range_sensor.getDistBottom() > 2.f * _params.rng_gnd_clearance) {
						// Data seems good, attempt a reset
						resetHaglRng();

					} else if (starting_conditions_passing) {
						// The sensor can probably not detect the ground properly
						// declare the sensor faulty and stop the fusion
						_control_status.flags.rng_fault = true;
						_range_sensor.setFaulty(true);
						stopHaglRngFusion();

					} else {
						// This could be a temporary issue, stop the fusion without declaring the sensor faulty
						stopHaglRngFusion();
					}
				}

			} else {
				stopHaglRngFusion();
			}

		} else {
			if (starting_conditions_passing) {
				startHaglRngFusion();
			}
		}

	} else if (_hagl_sensor_status.flags.range_finder && isTimedOut(_time_last_healthy_rng_data, _params.reset_timeout_max)) {
		// No data anymore. Stop until it comes back.
		stopHaglRngFusion();
	}
}

void Ekf::startHaglRngFusion()
{
	_hagl_sensor_status.flags.range_finder = true;
	resetHaglRngIfNeeded();
}

void Ekf::resetHaglRngIfNeeded()
{
	if (_hagl_sensor_status.flags.flow) {
		const float meas_hagl = _range_sensor.getDistBottom();
		const float pred_hagl = _terrain_vpos - _state.pos(2);
		const float hagl_innov = pred_hagl - meas_hagl;
		const float obs_variance = getRngVar();

		const float hagl_innov_var = fmaxf(_terrain_var + obs_variance, obs_variance);

		const float gate_size = fmaxf(_params.range_innov_gate, 1.0f);
		const float hagl_test_ratio = sq(hagl_innov) / (sq(gate_size) * hagl_innov_var);

		// Reset the state to the measurement only if the test ratio is large,
		// otherwise let it converge through the fusion
		if (hagl_test_ratio > 0.2f) {
			resetHaglRng();

		} else {
			fuseHaglRng();
		}

	} else {
		resetHaglRng();
	}
}

float Ekf::getRngVar()
{
	return fmaxf(P(9, 9) * _params.vehicle_variance_scaler, 0.0f)
	       + sq(_params.range_noise)
	       + sq(_params.range_noise_scaler * _range_sensor.getRange());
}

void Ekf::resetHaglRng()
{
	_terrain_vpos = _state.pos(2) + _range_sensor.getDistBottom();
	_terrain_var = getRngVar();
	_terrain_vpos_reset_counter++;
	_time_last_hagl_fuse = _time_delayed_us;
}

void Ekf::stopHaglRngFusion()
{
	if (_hagl_sensor_status.flags.range_finder) {

		_hagl_innov = 0.f;
		_hagl_innov_var = 0.f;
		_hagl_test_ratio = 0.f;
		_innov_check_fail_status.flags.reject_hagl = false;

		_hagl_sensor_status.flags.range_finder = false;
	}

	_time_last_healthy_rng_data = 0;
}

void Ekf::fuseHaglRng()
{
	// get a height above ground measurement from the range finder assuming a flat earth
	const float meas_hagl = _range_sensor.getDistBottom();

	// predict the hagl from the vehicle position and terrain height
	const float pred_hagl = _terrain_vpos - _state.pos(2);

	// calculate the innovation
	_hagl_innov = pred_hagl - meas_hagl;

	// calculate the observation variance adding the variance of the vehicles own height uncertainty
	const float obs_variance = getRngVar();

	// calculate the innovation variance - limiting it to prevent a badly conditioned fusion
	_hagl_innov_var = fmaxf(_terrain_var + obs_variance, obs_variance);

	// perform an innovation consistency check and only fuse data if it passes
	const float gate_size = fmaxf(_params.range_innov_gate, 1.0f);
	_hagl_test_ratio = sq(_hagl_innov) / (sq(gate_size) * _hagl_innov_var);

	if (_hagl_test_ratio <= 1.0f) {
		// calculate the Kalman gain
		const float gain = _terrain_var / _hagl_innov_var;
		// correct the state
		_terrain_vpos -= gain * _hagl_innov;
		// correct the variance
		_terrain_var = fmaxf(_terrain_var * (1.0f - gain), 0.0f);
		// record last successful fusion event
		_time_last_hagl_fuse = _time_delayed_us;
		_innov_check_fail_status.flags.reject_hagl = false;

	} else {
		_innov_check_fail_status.flags.reject_hagl = true;
	}
}

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
void Ekf::controlHaglFlowFusion()
{
	if (!(_params.terrain_fusion_mode & TerrainFusionMask::TerrainFuseOpticalFlow)) {
		stopHaglFlowFusion();
		return;
	}

	if (_flow_data_ready) {
		updateOptFlow(_aid_src_terrain_optical_flow);

		const bool continuing_conditions_passing = _control_status.flags.in_air
		                                           && !_control_status.flags.opt_flow
							   && _control_status.flags.gps
							   && isTimedOut(_time_last_hagl_fuse, 5e6f); // TODO: check for range_finder hagl aiding instead?
							   /* && !_hagl_sensor_status.flags.range_finder; */
		const bool starting_conditions_passing = continuing_conditions_passing;

		if (_hagl_sensor_status.flags.flow) {
			if (continuing_conditions_passing) {

				// TODO: wait until the midpoint of the flow sample has fallen behind the fusion time horizon
				fuseFlowForTerrain(_aid_src_terrain_optical_flow);
				_flow_data_ready = false;

				// TODO: do something when failing continuously the innovation check
				/* const bool is_fusion_failing = isTimedOut(_time_last_flow_terrain_fuse, _params.reset_timeout_max); */

				/* if (is_fusion_failing) { */
				/* 	resetHaglFlow(); */
				/* } */

			} else {
				stopHaglFlowFusion();
			}

		} else {
			if (starting_conditions_passing) {
				startHaglFlowFusion();
			}
		}

	} else if (_hagl_sensor_status.flags.flow
		   && (_time_delayed_us > _flow_sample_delayed.time_us + (uint64_t)5e6)) {
		// No data anymore. Stop until it comes back.
		stopHaglFlowFusion();
	}
}

void Ekf::startHaglFlowFusion()
{
	_hagl_sensor_status.flags.flow = true;
	// TODO: do a reset instead of trying to fuse the data?
	fuseFlowForTerrain(_aid_src_terrain_optical_flow);
	_flow_data_ready = false;
}

void Ekf::stopHaglFlowFusion()
{
	if (_hagl_sensor_status.flags.flow) {
		_hagl_sensor_status.flags.flow = false;
		resetEstimatorAidStatus(_aid_src_terrain_optical_flow);
	}
}

void Ekf::resetHaglFlow()
{
	// TODO: use the flow data
	_terrain_vpos = fmaxf(0.0f, _state.pos(2));
	_terrain_var = 100.0f;
	_terrain_vpos_reset_counter++;
}

void Ekf::fuseFlowForTerrain(estimator_aid_source2d_s &flow)
{
	flow.fusion_enabled = true;

	const float R_LOS = flow.observation_variance[0];

	// calculate the height above the ground of the optical flow camera. Since earth frame is NED
	// a positive offset in earth frame leads to a smaller height above the ground.
	float range = predictFlowRange();

	const float state = _terrain_vpos; // linearize both axes using the same state value
	Vector2f innov_var;
	float H;
	sym::TerrEstComputeFlowXyInnovVarAndHx(state, _terrain_var, _state.quat_nominal, _state.vel, _state.pos(2), R_LOS, FLT_EPSILON, &innov_var, &H);
	innov_var.copyTo(flow.innovation_variance);

	if ((flow.innovation_variance[0] < R_LOS)
	    || (flow.innovation_variance[1] < R_LOS)) {
		// we need to reinitialise the covariance matrix and abort this fusion step
		ECL_ERR("Opt flow error - covariance reset");
		_terrain_var = 100.0f;
		return;
	}

	// run the innovation consistency check and record result
	setEstimatorAidStatusTestRatio(flow, math::max(_params.flow_innov_gate, 1.f));

	_innov_check_fail_status.flags.reject_optflow_X = (flow.test_ratio[0] > 1.f);
	_innov_check_fail_status.flags.reject_optflow_Y = (flow.test_ratio[1] > 1.f);

	// if either axis fails we abort the fusion
	if (flow.innovation_rejected) {
		return;
	}

	// fuse observation axes sequentially
	for (uint8_t index = 0; index <= 1; index++) {
		if (index == 0) {
			// everything was already computed above

		} else if (index == 1) {
			// recalculate innovation variance because state covariances have changed due to previous fusion (linearise using the same initial state for all axes)
			sym::TerrEstComputeFlowYInnovVarAndH(state, _terrain_var, _state.quat_nominal, _state.vel, _state.pos(2), R_LOS, FLT_EPSILON, &flow.innovation_variance[1], &H);

			// recalculate the innovation using the updated state
			const Vector2f vel_body = predictFlowVelBody();
			range = predictFlowRange();
			flow.innovation[1] = (-vel_body(0) / range) - flow.observation[1];

			if (flow.innovation_variance[1] < R_LOS) {
				// we need to reinitialise the covariance matrix and abort this fusion step
				ECL_ERR("Opt flow error - covariance reset");
				_terrain_var = 100.0f;
				return;
			}
		}

		float Kfusion = _terrain_var * H / flow.innovation_variance[index];

		_terrain_vpos += Kfusion * flow.innovation[0];
		// constrain terrain to minimum allowed value and predict height above ground
		_terrain_vpos = fmaxf(_terrain_vpos, _params.rng_gnd_clearance + _state.pos(2));

		// guard against negative variance
		_terrain_var = fmaxf(_terrain_var - Kfusion * H * _terrain_var, sq(0.01f));
	}

	_fault_status.flags.bad_optflow_X = false;
	_fault_status.flags.bad_optflow_Y = false;

	_time_last_flow_terrain_fuse = _time_delayed_us;
	//_aid_src_optical_flow.time_last_fuse = _time_delayed_us; // TODO: separate aid source status for OF terrain?
	_aid_src_optical_flow.fused = true;
}
#endif // CONFIG_EKF2_OPTICAL_FLOW

void Ekf::controlHaglFakeFusion()
{
	if (!_control_status.flags.in_air
	    && !_hagl_sensor_status.flags.range_finder
	    && !_hagl_sensor_status.flags.flow) {

		initHagl();
	}
}

bool Ekf::isTerrainEstimateValid() const
{
#if defined(CONFIG_EKF2_RANGE_FINDER)
	// we have been fusing range finder measurements in the last 5 seconds
	if (_hagl_sensor_status.flags.range_finder && isRecent(_time_last_hagl_fuse, (uint64_t)5e6)) {
		return true;
	}
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// we have been fusing optical flow measurements for terrain estimation within the last 5 seconds
	// this can only be the case if the main filter does not fuse optical flow
	if (_hagl_sensor_status.flags.flow && isRecent(_time_last_flow_terrain_fuse, (uint64_t)5e6)) {
		return true;
	}
#endif // CONFIG_EKF2_OPTICAL_FLOW

	return false;
}
/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file sideslip_fusion.cpp
 * sideslip fusion methods.
 * equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Carl Olsson <carlolsson.co@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"
#include "python/ekf_derivation/generated/compute_sideslip_innov_and_innov_var.h"
#include "python/ekf_derivation/generated/compute_sideslip_h_and_k.h"

#include <mathlib/mathlib.h>

void Ekf::controlBetaFusion(const imuSample &imu_delayed)
{
	_control_status.flags.fuse_beta = _params.beta_fusion_enabled && _control_status.flags.fixed_wing
		&& _control_status.flags.in_air && !_control_status.flags.fake_pos;

	if (_control_status.flags.fuse_beta) {

		// Perform synthetic sideslip fusion at regular intervals when in-air and sideslip fusion had been enabled externally:
		const bool beta_fusion_time_triggered = isTimedOut(_aid_src_sideslip.time_last_fuse, _params.beta_avg_ft_us);

		if (beta_fusion_time_triggered) {

			updateSideslip(_aid_src_sideslip);
			_innov_check_fail_status.flags.reject_sideslip = _aid_src_sideslip.innovation_rejected;

			// If starting wind state estimation, reset the wind states and covariances before fusing any data
			if (!_control_status.flags.wind) {
				// activate the wind states
				_control_status.flags.wind = true;
				// reset the timeout timers to prevent repeated resets
				_aid_src_sideslip.time_last_fuse = imu_delayed.time_us;
				resetWindToZero();
			}

			if (Vector2f(Vector2f(_state.vel) - _state.wind_vel).longerThan(7.f)) {
				fuseSideslip(_aid_src_sideslip);
			}
		}
	}
}

void Ekf::updateSideslip(estimator_aid_source1d_s &sideslip) const
{
	// reset flags
	resetEstimatorAidStatus(sideslip);

	const float R = sq(_params.beta_noise); // observation noise variance

	float innov = 0.f;
	float innov_var = 0.f;
	sym::ComputeSideslipInnovAndInnovVar(getStateAtFusionHorizonAsVector(), P, R, FLT_EPSILON, &innov, &innov_var);

	sideslip.observation = 0.f;
	sideslip.observation_variance = R;
	sideslip.innovation = innov;
	sideslip.innovation_variance = innov_var;

	sideslip.fusion_enabled = _control_status.flags.fuse_aspd;

	sideslip.timestamp_sample = _time_delayed_us;

	const float innov_gate = fmaxf(_params.beta_innov_gate, 1.f);
	setEstimatorAidStatusTestRatio(sideslip, innov_gate);
}

void Ekf::fuseSideslip(estimator_aid_source1d_s &sideslip)
{
	if (sideslip.innovation_rejected) {
		return;
	}
	// determine if we need the sideslip fusion to correct states other than wind
	bool update_wind_only = !_control_status.flags.wind_dead_reckoning;

	// Reset covariance and states if the calculation is badly conditioned
	if ((sideslip.innovation_variance < sideslip.observation_variance)
	    || (sideslip.innovation_variance < FLT_EPSILON)) {
		_fault_status.flags.bad_sideslip = true;

		// if we are getting aiding from other sources, warn and reset the wind states and covariances only
		const char *action_string = nullptr;

		if (update_wind_only) {
			resetWind();
			action_string = "wind";

		} else {
			initialiseCovariance();
			_state.wind_vel.setZero();
			action_string = "full";
		}

		ECL_ERR("sideslip badly conditioned - %s covariance reset", action_string);

		return;
	}

	_fault_status.flags.bad_sideslip = false;

	Vector24f H; // Observation jacobian
	Vector24f K; // Kalman gain vector

	sym::ComputeSideslipHAndK(getStateAtFusionHorizonAsVector(), P, sideslip.innovation_variance, FLT_EPSILON, &H, &K);

	if (update_wind_only) {
		for (unsigned row = 0; row <= 21; row++) {
			K(row) = 0.f;
		}
	}

	const bool is_fused = measurementUpdate(K, sideslip.innovation_variance, sideslip.innovation);

	sideslip.fused = is_fused;
	_fault_status.flags.bad_sideslip = !is_fused;

	if (is_fused) {
		sideslip.time_last_fuse = _time_delayed_us;
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <px4_platform_common/events.h>
#include "EKF2.hpp"

using namespace time_literals;
using math::constrain;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

pthread_mutex_t ekf2_module_mutex = PTHREAD_MUTEX_INITIALIZER;
static px4::atomic<EKF2 *> _objects[EKF2_MAX_INSTANCES] {};
#if defined(CONFIG_EKF2_MULTI_INSTANCE)
static px4::atomic<EKF2Selector *> _ekf2_selector {nullptr};
#endif // CONFIG_EKF2_MULTI_INSTANCE

EKF2::EKF2(bool multi_mode, const px4::wq_config_t &config, bool replay_mode):
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, config),
	_replay_mode(replay_mode && !multi_mode),
	_multi_mode(multi_mode),
	_instance(multi_mode ? -1 : 0),
	_attitude_pub(multi_mode ? ORB_ID(estimator_attitude) : ORB_ID(vehicle_attitude)),
	_local_position_pub(multi_mode ? ORB_ID(estimator_local_position) : ORB_ID(vehicle_local_position)),
	_global_position_pub(multi_mode ? ORB_ID(estimator_global_position) : ORB_ID(vehicle_global_position)),
	_odometry_pub(multi_mode ? ORB_ID(estimator_odometry) : ORB_ID(vehicle_odometry)),
	_wind_pub(multi_mode ? ORB_ID(estimator_wind) : ORB_ID(wind)),
	_params(_ekf.getParamHandle()),
	_param_ekf2_predict_us(_params->filter_update_interval_us),
	_param_ekf2_imu_ctrl(_params->imu_ctrl),
	_param_ekf2_mag_delay(_params->mag_delay_ms),
	_param_ekf2_baro_delay(_params->baro_delay_ms),
	_param_ekf2_gps_delay(_params->gps_delay_ms),
#if defined(CONFIG_EKF2_AUXVEL)
	_param_ekf2_avel_delay(_params->auxvel_delay_ms),
#endif // CONFIG_EKF2_AUXVEL
	_param_ekf2_gyr_noise(_params->gyro_noise),
	_param_ekf2_acc_noise(_params->accel_noise),
	_param_ekf2_gyr_b_noise(_params->gyro_bias_p_noise),
	_param_ekf2_acc_b_noise(_params->accel_bias_p_noise),
	_param_ekf2_mag_e_noise(_params->mage_p_noise),
	_param_ekf2_mag_b_noise(_params->magb_p_noise),
	_param_ekf2_wind_nsd(_params->wind_vel_nsd),
	_param_ekf2_gps_v_noise(_params->gps_vel_noise),
	_param_ekf2_gps_p_noise(_params->gps_pos_noise),
	_param_ekf2_noaid_noise(_params->pos_noaid_noise),
	_param_ekf2_baro_noise(_params->baro_noise),
	_param_ekf2_baro_gate(_params->baro_innov_gate),
	_param_ekf2_gnd_eff_dz(_params->gnd_effect_deadzone),
	_param_ekf2_gnd_max_hgt(_params->gnd_effect_max_hgt),
	_param_ekf2_gps_p_gate(_params->gps_pos_innov_gate),
	_param_ekf2_gps_v_gate(_params->gps_vel_innov_gate),
#if defined(CONFIG_EKF2_AIRSPEED)
	_param_ekf2_asp_delay(_params->airspeed_delay_ms),
	_param_ekf2_tas_gate(_params->tas_innov_gate),
	_param_ekf2_eas_noise(_params->eas_noise),
	_param_ekf2_arsp_thr(_params->arsp_thr),
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_SIDESLIP)
	_param_ekf2_beta_gate(_params->beta_innov_gate),
	_param_ekf2_beta_noise(_params->beta_noise),
	_param_ekf2_fuse_beta(_params->beta_fusion_enabled),
#endif // CONFIG_EKF2_SIDESLIP
	_param_ekf2_head_noise(_params->mag_heading_noise),
	_param_ekf2_mag_noise(_params->mag_noise),
	_param_ekf2_mag_decl(_params->mag_declination_deg),
	_param_ekf2_hdg_gate(_params->heading_innov_gate),
	_param_ekf2_mag_gate(_params->mag_innov_gate),
	_param_ekf2_decl_type(_params->mag_declination_source),
	_param_ekf2_mag_type(_params->mag_fusion_type),
	_param_ekf2_mag_acclim(_params->mag_acc_gate),
	_param_ekf2_mag_yawlim(_params->mag_yaw_rate_gate),
	_param_ekf2_gps_check(_params->gps_check_mask),
	_param_ekf2_req_eph(_params->req_hacc),
	_param_ekf2_req_epv(_params->req_vacc),
	_param_ekf2_req_sacc(_params->req_sacc),
	_param_ekf2_req_nsats(_params->req_nsats),
	_param_ekf2_req_pdop(_params->req_pdop),
	_param_ekf2_req_hdrift(_params->req_hdrift),
	_param_ekf2_req_vdrift(_params->req_vdrift),
	_param_ekf2_hgt_ref(_params->height_sensor_ref),
	_param_ekf2_baro_ctrl(_params->baro_ctrl),
	_param_ekf2_gps_ctrl(_params->gnss_ctrl),
	_param_ekf2_noaid_tout(_params->valid_timeout_max),
#if defined(CONFIG_EKF2_RANGE_FINDER)
	_param_ekf2_rng_ctrl(_params->rng_ctrl),
	_param_ekf2_rng_delay(_params->range_delay_ms),
	_param_ekf2_rng_noise(_params->range_noise),
	_param_ekf2_rng_sfe(_params->range_noise_scaler),
	_param_ekf2_rng_gate(_params->range_innov_gate),
	_param_ekf2_min_rng(_params->rng_gnd_clearance),
	_param_ekf2_rng_pitch(_params->rng_sens_pitch),
	_param_ekf2_rng_a_vmax(_params->max_vel_for_range_aid),
	_param_ekf2_rng_a_hmax(_params->max_hagl_for_range_aid),
	_param_ekf2_rng_a_igate(_params->range_aid_innov_gate),
	_param_ekf2_rng_qlty_t(_params->range_valid_quality_s),
	_param_ekf2_rng_k_gate(_params->range_kin_consistency_gate),
	_param_ekf2_rng_pos_x(_params->rng_pos_body(0)),
	_param_ekf2_rng_pos_y(_params->rng_pos_body(1)),
	_param_ekf2_rng_pos_z(_params->rng_pos_body(2)),
	_param_ekf2_terr_mask(_params->terrain_fusion_mode),
	_param_ekf2_terr_noise(_params->terrain_p_noise),
	_param_ekf2_terr_grad(_params->terrain_gradient),
#endif // CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	_param_ekf2_ev_delay(_params->ev_delay_ms),
	_param_ekf2_ev_ctrl(_params->ev_ctrl),
	_param_ekf2_ev_qmin(_params->ev_quality_minimum),
	_param_ekf2_evp_noise(_params->ev_pos_noise),
	_param_ekf2_evv_noise(_params->ev_vel_noise),
	_param_ekf2_eva_noise(_params->ev_att_noise),
	_param_ekf2_evv_gate(_params->ev_vel_innov_gate),
	_param_ekf2_evp_gate(_params->ev_pos_innov_gate),
	_param_ekf2_ev_pos_x(_params->ev_pos_body(0)),
	_param_ekf2_ev_pos_y(_params->ev_pos_body(1)),
	_param_ekf2_ev_pos_z(_params->ev_pos_body(2)),
#endif // CONFIG_EKF2_EXTERNAL_VISION
	_param_ekf2_grav_noise(_params->gravity_noise),
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	_param_ekf2_of_ctrl(_params->flow_ctrl),
	_param_ekf2_of_delay(_params->flow_delay_ms),
	_param_ekf2_of_n_min(_params->flow_noise),
	_param_ekf2_of_n_max(_params->flow_noise_qual_min),
	_param_ekf2_of_qmin(_params->flow_qual_min),
	_param_ekf2_of_qmin_gnd(_params->flow_qual_min_gnd),
	_param_ekf2_of_gate(_params->flow_innov_gate),
	_param_ekf2_of_pos_x(_params->flow_pos_body(0)),
	_param_ekf2_of_pos_y(_params->flow_pos_body(1)),
	_param_ekf2_of_pos_z(_params->flow_pos_body(2)),
#endif // CONFIG_EKF2_OPTICAL_FLOW
	_param_ekf2_imu_pos_x(_params->imu_pos_body(0)),
	_param_ekf2_imu_pos_y(_params->imu_pos_body(1)),
	_param_ekf2_imu_pos_z(_params->imu_pos_body(2)),
	_param_ekf2_gps_pos_x(_params->gps_pos_body(0)),
	_param_ekf2_gps_pos_y(_params->gps_pos_body(1)),
	_param_ekf2_gps_pos_z(_params->gps_pos_body(2)),
	_param_ekf2_gbias_init(_params->switch_on_gyro_bias),
	_param_ekf2_abias_init(_params->switch_on_accel_bias),
	_param_ekf2_angerr_init(_params->initial_tilt_err),
	_param_ekf2_abl_lim(_params->acc_bias_lim),
	_param_ekf2_abl_acclim(_params->acc_bias_learn_acc_lim),
	_param_ekf2_abl_gyrlim(_params->acc_bias_learn_gyr_lim),
	_param_ekf2_abl_tau(_params->acc_bias_learn_tc),
	_param_ekf2_gyr_b_lim(_params->gyro_bias_lim),
#if defined(CONFIG_EKF2_DRAG_FUSION)
	_param_ekf2_drag_ctrl(_params->drag_ctrl),
	_param_ekf2_drag_noise(_params->drag_noise),
	_param_ekf2_bcoef_x(_params->bcoef_x),
	_param_ekf2_bcoef_y(_params->bcoef_y),
	_param_ekf2_mcoef(_params->mcoef),
#endif // CONFIG_EKF2_DRAG_FUSION
#if defined(CONFIG_EKF2_BARO_COMPENSATION)
	_param_ekf2_aspd_max(_params->max_correction_airspeed),
	_param_ekf2_pcoef_xp(_params->static_pressure_coef_xp),
	_param_ekf2_pcoef_xn(_params->static_pressure_coef_xn),
	_param_ekf2_pcoef_yp(_params->static_pressure_coef_yp),
	_param_ekf2_pcoef_yn(_params->static_pressure_coef_yn),
	_param_ekf2_pcoef_z(_params->static_pressure_coef_z),
#endif // CONFIG_EKF2_BARO_COMPENSATION
	_param_ekf2_mag_check(_params->check_mag_strength),
	_param_ekf2_synthetic_mag_z(_params->synthesize_mag_z),
	_param_ekf2_gsf_tas_default(_params->EKFGSF_tas_default)
{
	// advertise expected minimal topic set immediately to ensure logging
	_attitude_pub.advertise();
	_local_position_pub.advertise();

	_estimator_event_flags_pub.advertise();
	_estimator_innovation_test_ratios_pub.advertise();
	_estimator_innovation_variances_pub.advertise();
	_estimator_innovations_pub.advertise();
	_estimator_sensor_bias_pub.advertise();
	_estimator_states_pub.advertise();
	_estimator_status_flags_pub.advertise();
	_estimator_status_pub.advertise();
}

EKF2::~EKF2()
{
	perf_free(_ecl_ekf_update_perf);
	perf_free(_ecl_ekf_update_full_perf);
	perf_free(_msg_missed_imu_perf);
	perf_free(_msg_missed_air_data_perf);
#if defined(CONFIG_EKF2_AIRSPEED)
	perf_free(_msg_missed_airspeed_perf);
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_RANGE_FINDER)
	perf_free(_msg_missed_distance_sensor_perf);
#endif // CONFIG_EKF2_RANGE_FINDER
	perf_free(_msg_missed_gps_perf);
#if defined(CONFIG_EKF2_AUXVEL)
	perf_free(_msg_missed_landing_target_pose_perf);
#endif // CONFIG_EKF2_AUXVEL
	perf_free(_msg_missed_magnetometer_perf);
	perf_free(_msg_missed_odometry_perf);
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	perf_free(_msg_missed_optical_flow_perf);
#endif // CONFIG_EKF2_OPTICAL_FLOW
}

#if defined(CONFIG_EKF2_MULTI_INSTANCE)
bool EKF2::multi_init(int imu, int mag)
{
	// advertise all topics to ensure consistent uORB instance numbering
	_ekf2_timestamps_pub.advertise();
	_estimator_event_flags_pub.advertise();
	_estimator_innovation_test_ratios_pub.advertise();
	_estimator_innovation_variances_pub.advertise();
	_estimator_innovations_pub.advertise();
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	_estimator_optical_flow_vel_pub.advertise();
#endif // CONFIG_EKF2_OPTICAL_FLOW
	_estimator_sensor_bias_pub.advertise();
	_estimator_states_pub.advertise();
	_estimator_status_flags_pub.advertise();
	_estimator_status_pub.advertise();
	_yaw_est_pub.advertise();

	// baro advertise
	if (_param_ekf2_baro_ctrl.get()) {
		_estimator_aid_src_baro_hgt_pub.advertise();
		_estimator_baro_bias_pub.advertise();
	}

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	// EV advertise
	if (_param_ekf2_ev_ctrl.get() & static_cast<int32_t>(EvCtrl::VPOS)) {
		_estimator_aid_src_ev_hgt_pub.advertise();
		_estimator_ev_pos_bias_pub.advertise();
	}

	if (_param_ekf2_ev_ctrl.get() & static_cast<int32_t>(EvCtrl::HPOS)) {
		_estimator_aid_src_ev_pos_pub.advertise();
		_estimator_ev_pos_bias_pub.advertise();
	}

	if (_param_ekf2_ev_ctrl.get() & static_cast<int32_t>(EvCtrl::VEL)) {
		_estimator_aid_src_ev_vel_pub.advertise();
	}

	if (_param_ekf2_ev_ctrl.get() & static_cast<int32_t>(EvCtrl::YAW)) {
		_estimator_aid_src_ev_yaw_pub.advertise();
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	// GNSS advertise
	if (_param_ekf2_gps_ctrl.get() & static_cast<int32_t>(GnssCtrl::VPOS)) {
		_estimator_aid_src_gnss_hgt_pub.advertise();
		_estimator_gnss_hgt_bias_pub.advertise();
	}

	if (_param_ekf2_gps_ctrl.get() & static_cast<int32_t>(GnssCtrl::HPOS)) {
		_estimator_aid_src_gnss_pos_pub.advertise();
		_estimator_gps_status_pub.advertise();
	}

	if (_param_ekf2_gps_ctrl.get() & static_cast<int32_t>(GnssCtrl::VEL)) {
		_estimator_aid_src_gnss_vel_pub.advertise();
	}

#if defined(CONFIG_EKF2_GNSS_YAW)

	if (_param_ekf2_gps_ctrl.get() & static_cast<int32_t>(GnssCtrl::YAW)) {
		_estimator_aid_src_gnss_yaw_pub.advertise();
	}

#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_RANGE_FINDER)

	// RNG advertise
	if (_param_ekf2_rng_ctrl.get()) {
		_estimator_aid_src_rng_hgt_pub.advertise();
		_estimator_rng_hgt_bias_pub.advertise();
	}

#endif // CONFIG_EKF2_RANGE_FINDER

	_attitude_pub.advertise();
	_local_position_pub.advertise();
	_global_position_pub.advertise();
	_odometry_pub.advertise();
	_wind_pub.advertise();

	bool changed_instance = _vehicle_imu_sub.ChangeInstance(imu) && _magnetometer_sub.ChangeInstance(mag);

	const int status_instance = _estimator_states_pub.get_instance();

	if ((status_instance >= 0) && changed_instance
	    && (_attitude_pub.get_instance() == status_instance)
	    && (_local_position_pub.get_instance() == status_instance)
	    && (_global_position_pub.get_instance() == status_instance)) {

		_instance = status_instance;

		ScheduleNow();
		return true;
	}

	PX4_ERR("publication instance problem: %d att: %d lpos: %d gpos: %d", status_instance,
		_attitude_pub.get_instance(), _local_position_pub.get_instance(), _global_position_pub.get_instance());

	return false;
}
#endif // CONFIG_EKF2_MULTI_INSTANCE

int EKF2::print_status()
{
	PX4_INFO_RAW("ekf2:%d EKF dt: %.4fs, attitude: %d, local position: %d, global position: %d\n",
		     _instance, (double)_ekf.get_dt_ekf_avg(), _ekf.attitude_valid(),
		     _ekf.local_position_is_valid(), _ekf.global_position_is_valid());

	perf_print_counter(_ecl_ekf_update_perf);
	perf_print_counter(_ecl_ekf_update_full_perf);
	perf_print_counter(_msg_missed_imu_perf);
	perf_print_counter(_msg_missed_air_data_perf);
#if defined(CONFIG_EKF2_AIRSPEED)
	perf_print_counter(_msg_missed_airspeed_perf);
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_RANGE_FINDER)
	perf_print_counter(_msg_missed_distance_sensor_perf);
#endif // CONFIG_EKF2_RANGE_FINDER
	perf_print_counter(_msg_missed_gps_perf);
#if defined(CONFIG_EKF2_AUXVEL)
	perf_print_counter(_msg_missed_landing_target_pose_perf);
#endif // CONFIG_EKF2_AUXVEL
	perf_print_counter(_msg_missed_magnetometer_perf);
	perf_print_counter(_msg_missed_odometry_perf);
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	perf_print_counter(_msg_missed_optical_flow_perf);
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(DEBUG_BUILD)
	_ekf.print_status();
#endif // DEBUG_BUILD

	return 0;
}

void EKF2::Run()
{
	if (should_exit()) {
		_sensor_combined_sub.unregisterCallback();
		_vehicle_imu_sub.unregisterCallback();

		return;
	}

	// check for parameter updates
	if (_parameter_update_sub.updated() || !_callback_registered) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

		VerifyParams();

		_ekf.set_min_required_gps_health_time(_param_ekf2_req_gps_h.get() * 1_s);

		const matrix::Vector3f imu_pos_body(_param_ekf2_imu_pos_x.get(),
						    _param_ekf2_imu_pos_y.get(),
						    _param_ekf2_imu_pos_z.get());
		_ekf.output_predictor().set_imu_offset(imu_pos_body);
		_ekf.output_predictor().set_pos_correction_tc(_param_ekf2_tau_pos.get());
		_ekf.output_predictor().set_vel_correction_tc(_param_ekf2_tau_vel.get());

#if defined(CONFIG_EKF2_AIRSPEED)
		// The airspeed scale factor correcton is only available via parameter as used by the airspeed module
		param_t param_aspd_scale = param_find("ASPD_SCALE_1");

		if (param_aspd_scale != PARAM_INVALID) {
			param_get(param_aspd_scale, &_airspeed_scale_factor);
		}

#endif // CONFIG_EKF2_AIRSPEED

		// if using baro ensure sensor interval minimum is sufficient to accommodate system averaged baro output
		if (_params->baro_ctrl == 1) {
			float sens_baro_rate = 0.f;

			if (param_get(param_find("SENS_BARO_RATE"), &sens_baro_rate) == PX4_OK) {
				if (sens_baro_rate > 0) {
					float interval_ms = roundf(1000.f / sens_baro_rate);

					if (PX4_ISFINITE(interval_ms) && (interval_ms > _params->sensor_interval_max_ms)) {
						PX4_DEBUG("updating sensor_interval_max_ms %.3f -> %.3f", (double)_params->sensor_interval_max_ms, (double)interval_ms);
						_params->sensor_interval_max_ms = interval_ms;
					}
				}
			}
		}

		// if using mag ensure sensor interval minimum is sufficient to accommodate system averaged mag output
		if (_params->mag_fusion_type != MagFuseType::NONE) {
			float sens_mag_rate = 0.f;

			if (param_get(param_find("SENS_MAG_RATE"), &sens_mag_rate) == PX4_OK) {
				if (sens_mag_rate > 0) {
					float interval_ms = roundf(1000.f / sens_mag_rate);

					if (PX4_ISFINITE(interval_ms) && (interval_ms > _params->sensor_interval_max_ms)) {
						PX4_DEBUG("updating sensor_interval_max_ms %.3f -> %.3f", (double)_params->sensor_interval_max_ms, (double)interval_ms);
						_params->sensor_interval_max_ms = interval_ms;
					}
				}
			}
		}
	}

	if (!_callback_registered) {
#if defined(CONFIG_EKF2_MULTI_INSTANCE)

		if (_multi_mode) {
			_callback_registered = _vehicle_imu_sub.registerCallback();

		} else
#endif // CONFIG_EKF2_MULTI_INSTANCE
		{
			_callback_registered = _sensor_combined_sub.registerCallback();
		}

		if (!_callback_registered) {
			ScheduleDelayed(10_ms);
			return;
		}
	}

	if (_vehicle_command_sub.updated()) {
		vehicle_command_s vehicle_command;

		if (_vehicle_command_sub.update(&vehicle_command)) {
			if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN) {
				double latitude = vehicle_command.param5;
				double longitude = vehicle_command.param6;
				float altitude = vehicle_command.param7;

				if (_ekf.setEkfGlobalOrigin(latitude, longitude, altitude)) {
					// Validate the ekf origin status.
					uint64_t origin_time {};
					_ekf.getEkfGlobalOrigin(origin_time, latitude, longitude, altitude);
					PX4_INFO("%d - New NED origin (LLA): %3.10f, %3.10f, %4.3f\n",
						 _instance, latitude, longitude, static_cast<double>(altitude));

				} else {
					PX4_ERR("%d - Failed to set new NED origin (LLA): %3.10f, %3.10f, %4.3f\n",
						_instance, latitude, longitude, static_cast<double>(altitude));
				}
			}
		}
	}

	bool imu_updated = false;
	imuSample imu_sample_new {};

	hrt_abstime imu_dt = 0; // for tracking time slip later

#if defined(CONFIG_EKF2_MULTI_INSTANCE)

	if (_multi_mode) {
		const unsigned last_generation = _vehicle_imu_sub.get_last_generation();
		vehicle_imu_s imu;
		imu_updated = _vehicle_imu_sub.update(&imu);

		if (imu_updated && (_vehicle_imu_sub.get_last_generation() != last_generation + 1)) {
			perf_count(_msg_missed_imu_perf);
		}

		if (imu_updated) {
			imu_sample_new.time_us = imu.timestamp_sample;
			imu_sample_new.delta_ang_dt = imu.delta_angle_dt * 1.e-6f;
			imu_sample_new.delta_ang = Vector3f{imu.delta_angle};
			imu_sample_new.delta_vel_dt = imu.delta_velocity_dt * 1.e-6f;
			imu_sample_new.delta_vel = Vector3f{imu.delta_velocity};

			if (imu.delta_velocity_clipping > 0) {
				imu_sample_new.delta_vel_clipping[0] = imu.delta_velocity_clipping & vehicle_imu_s::CLIPPING_X;
				imu_sample_new.delta_vel_clipping[1] = imu.delta_velocity_clipping & vehicle_imu_s::CLIPPING_Y;
				imu_sample_new.delta_vel_clipping[2] = imu.delta_velocity_clipping & vehicle_imu_s::CLIPPING_Z;
			}

			imu_dt = imu.delta_angle_dt;

			if ((_device_id_accel == 0) || (_device_id_gyro == 0)) {
				_device_id_accel = imu.accel_device_id;
				_device_id_gyro = imu.gyro_device_id;
				_accel_calibration_count = imu.accel_calibration_count;
				_gyro_calibration_count = imu.gyro_calibration_count;

			} else {
				if ((imu.accel_calibration_count != _accel_calibration_count)
				    || (imu.accel_device_id != _device_id_accel)) {

					PX4_DEBUG("%d - resetting accelerometer bias", _instance);
					_device_id_accel = imu.accel_device_id;

					_ekf.resetAccelBias();
					_accel_calibration_count = imu.accel_calibration_count;

					// reset bias learning
					_accel_cal = {};
				}

				if ((imu.gyro_calibration_count != _gyro_calibration_count)
				    || (imu.gyro_device_id != _device_id_gyro)) {

					PX4_DEBUG("%d - resetting rate gyro bias", _instance);
					_device_id_gyro = imu.gyro_device_id;

					_ekf.resetGyroBias();
					_gyro_calibration_count = imu.gyro_calibration_count;

					// reset bias learning
					_gyro_cal = {};
				}
			}
		}

	} else
#endif // CONFIG_EKF2_MULTI_INSTANCE
	{
		const unsigned last_generation = _sensor_combined_sub.get_last_generation();
		sensor_combined_s sensor_combined;
		imu_updated = _sensor_combined_sub.update(&sensor_combined);

		if (imu_updated && (_sensor_combined_sub.get_last_generation() != last_generation + 1)) {
			perf_count(_msg_missed_imu_perf);
		}

		if (imu_updated) {
			imu_sample_new.time_us = sensor_combined.timestamp;
			imu_sample_new.delta_ang_dt = sensor_combined.gyro_integral_dt * 1.e-6f;
			imu_sample_new.delta_ang = Vector3f{sensor_combined.gyro_rad} * imu_sample_new.delta_ang_dt;
			imu_sample_new.delta_vel_dt = sensor_combined.accelerometer_integral_dt * 1.e-6f;
			imu_sample_new.delta_vel = Vector3f{sensor_combined.accelerometer_m_s2} * imu_sample_new.delta_vel_dt;

			if (sensor_combined.accelerometer_clipping > 0) {
				imu_sample_new.delta_vel_clipping[0] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_X;
				imu_sample_new.delta_vel_clipping[1] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_Y;
				imu_sample_new.delta_vel_clipping[2] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_Z;
			}

			imu_dt = sensor_combined.gyro_integral_dt;

			if (sensor_combined.accel_calibration_count != _accel_calibration_count) {

				PX4_DEBUG("%d - resetting accelerometer bias", _instance);

				_ekf.resetAccelBias();
				_accel_calibration_count = sensor_combined.accel_calibration_count;

				// reset bias learning
				_accel_cal = {};
			}

			if (sensor_combined.gyro_calibration_count != _gyro_calibration_count) {

				PX4_DEBUG("%d - resetting rate gyro bias", _instance);

				_ekf.resetGyroBias();
				_gyro_calibration_count = sensor_combined.gyro_calibration_count;

				// reset bias learning
				_gyro_cal = {};
			}
		}

		if (_sensor_selection_sub.updated() || (_device_id_accel == 0 || _device_id_gyro == 0)) {
			sensor_selection_s sensor_selection;

			if (_sensor_selection_sub.copy(&sensor_selection)) {
				if (_device_id_accel != sensor_selection.accel_device_id) {

					_device_id_accel = sensor_selection.accel_device_id;

					_ekf.resetAccelBias();

					// reset bias learning
					_accel_cal = {};
				}

				if (_device_id_gyro != sensor_selection.gyro_device_id) {

					_device_id_gyro = sensor_selection.gyro_device_id;

					_ekf.resetGyroBias();

					// reset bias learning
					_gyro_cal = {};
				}
			}
		}
	}

	if (imu_updated) {
		const hrt_abstime now = imu_sample_new.time_us;

		// push imu data into estimator
		_ekf.setIMUData(imu_sample_new);
		PublishAttitude(now); // publish attitude immediately (uses quaternion from output predictor)

		// integrate time to monitor time slippage
		if (_start_time_us > 0) {
			_integrated_time_us += imu_dt;
			_last_time_slip_us = (imu_sample_new.time_us - _start_time_us) - _integrated_time_us;

		} else {
			_start_time_us = imu_sample_new.time_us;
			_last_time_slip_us = 0;
		}

		// ekf2_timestamps (using 0.1 ms relative timestamps)
		ekf2_timestamps_s ekf2_timestamps {
			.timestamp = now,
			.airspeed_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.distance_sensor_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.optical_flow_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.vehicle_air_data_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.vehicle_magnetometer_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.visual_odometry_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
		};

#if defined(CONFIG_EKF2_AIRSPEED)
		UpdateAirspeedSample(ekf2_timestamps);
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_AUXVEL)
		UpdateAuxVelSample(ekf2_timestamps);
#endif // CONFIG_EKF2_AUXVEL
		UpdateBaroSample(ekf2_timestamps);
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
		UpdateExtVisionSample(ekf2_timestamps);
#endif // CONFIG_EKF2_EXTERNAL_VISION
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
		UpdateFlowSample(ekf2_timestamps);
#endif // CONFIG_EKF2_OPTICAL_FLOW
		UpdateGpsSample(ekf2_timestamps);
		UpdateMagSample(ekf2_timestamps);
#if defined(CONFIG_EKF2_RANGE_FINDER)
		UpdateRangeSample(ekf2_timestamps);
#endif // CONFIG_EKF2_RANGE_FINDER
		UpdateSystemFlagsSample(ekf2_timestamps);

		// run the EKF update and output
		const hrt_abstime ekf_update_start = hrt_absolute_time();

		if (_ekf.update()) {
			perf_set_elapsed(_ecl_ekf_update_full_perf, hrt_elapsed_time(&ekf_update_start));

			PublishLocalPosition(now);
			PublishOdometry(now, imu_sample_new);
			PublishGlobalPosition(now);
			PublishWindEstimate(now);

			// publish status/logging messages
			PublishBaroBias(now);
			PublishGnssHgtBias(now);
#if defined(CONFIG_EKF2_RANGE_FINDER)
			PublishRngHgtBias(now);
#endif // CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
			PublishEvPosBias(now);
#endif // CONFIG_EKF2_EXTERNAL_VISION
			PublishEventFlags(now);
			PublishGpsStatus(now);
			PublishInnovations(now);
			PublishInnovationTestRatios(now);
			PublishInnovationVariances(now);
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
			PublishOpticalFlowVel(now);
#endif // CONFIG_EKF2_OPTICAL_FLOW
			PublishStates(now);
			PublishStatus(now);
			PublishStatusFlags(now);
			PublishYawEstimatorStatus(now);

			UpdateAccelCalibration(now);
			UpdateGyroCalibration(now);
			UpdateMagCalibration(now);
			PublishSensorBias(now);

			PublishAidSourceStatus(now);

		} else {
			// ekf no update
			perf_set_elapsed(_ecl_ekf_update_perf, hrt_elapsed_time(&ekf_update_start));
		}

		// publish ekf2_timestamps
		_ekf2_timestamps_pub.publish(ekf2_timestamps);
	}

	// re-schedule as backup timeout
	ScheduleDelayed(100_ms);
}

void EKF2::VerifyParams()
{
	if ((_param_ekf2_aid_mask.get() & SensorFusionMask::DEPRECATED_USE_GPS)
	    || (_param_ekf2_aid_mask.get() & SensorFusionMask::DEPRECATED_USE_GPS_YAW)) {
		_param_ekf2_aid_mask.set(_param_ekf2_aid_mask.get() & ~(SensorFusionMask::DEPRECATED_USE_GPS |
					 SensorFusionMask::DEPRECATED_USE_GPS_YAW));
		_param_ekf2_aid_mask.commit();
		mavlink_log_critical(&_mavlink_log_pub, "Use EKF2_GPS_CTRL instead\n");
		/* EVENT
		 * @description <param>EKF2_AID_MASK</param> is set to {1:.0}.
		 */
		events::send<float>(events::ID("ekf2_aid_mask_gps"), events::Log::Warning,
				    "Use EKF2_GPS_CTRL instead", _param_ekf2_aid_mask.get());
	}

	if ((_param_ekf2_gps_ctrl.get() & GnssCtrl::VPOS) && !(_param_ekf2_gps_ctrl.get() & GnssCtrl::HPOS)) {
		_param_ekf2_gps_ctrl.set(_param_ekf2_gps_ctrl.get() & ~GnssCtrl::VPOS);
		_param_ekf2_gps_ctrl.commit();
		mavlink_log_critical(&_mavlink_log_pub, "GPS lon/lat is required for altitude fusion\n");
		/* EVENT
		 * @description <param>EKF2_GPS_CTRL</param> is set to {1:.0}.
		 */
		events::send<float>(events::ID("ekf2_gps_ctrl_alt"), events::Log::Warning,
				    "GPS lon/lat is required for altitude fusion", _param_ekf2_gps_ctrl.get());
	}

	if ((_param_ekf2_hgt_ref.get() == HeightSensor::BARO) && (_param_ekf2_baro_ctrl.get() == 0)) {
		_param_ekf2_baro_ctrl.set(1);
		_param_ekf2_baro_ctrl.commit();
		mavlink_log_critical(&_mavlink_log_pub, "Baro enabled by EKF2_HGT_REF\n");
		/* EVENT
		 * @description <param>EKF2_BARO_CTRL</param> is set to {1:.0}.
		 */
		events::send<float>(events::ID("ekf2_hgt_ref_baro"), events::Log::Warning,
				    "Baro enabled by EKF2_HGT_REF", _param_ekf2_baro_ctrl.get());
	}

#if defined(CONFIG_EKF2_RANGE_FINDER)

	if ((_param_ekf2_hgt_ref.get() == HeightSensor::RANGE) && (_param_ekf2_rng_ctrl.get() == RngCtrl::DISABLED)) {
		_param_ekf2_rng_ctrl.set(1);
		_param_ekf2_rng_ctrl.commit();
		mavlink_log_critical(&_mavlink_log_pub, "Range enabled by EKF2_HGT_REF\n");
		/* EVENT
		 * @description <param>EKF2_RNG_CTRL</param> is set to {1:.0}.
		 */
		events::send<float>(events::ID("ekf2_hgt_ref_rng"), events::Log::Warning,
				    "Range enabled by EKF2_HGT_REF", _param_ekf2_rng_ctrl.get());
	}

#endif // CONFIG_EKF2_RANGE_FINDER

	if ((_param_ekf2_hgt_ref.get() == HeightSensor::GNSS) && !(_param_ekf2_gps_ctrl.get() & GnssCtrl::VPOS)) {
		_param_ekf2_gps_ctrl.set(_param_ekf2_gps_ctrl.get() | (GnssCtrl::VPOS | GnssCtrl::HPOS | GnssCtrl::VEL));
		_param_ekf2_gps_ctrl.commit();
		mavlink_log_critical(&_mavlink_log_pub, "GPS enabled by EKF2_HGT_REF\n");
		/* EVENT
		 * @description <param>EKF2_GPS_CTRL</param> is set to {1:.0}.
		 */
		events::send<float>(events::ID("ekf2_hgt_ref_gps"), events::Log::Warning,
				    "GPS enabled by EKF2_HGT_REF", _param_ekf2_gps_ctrl.get());
	}

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if ((_param_ekf2_hgt_ref.get() == HeightSensor::EV)
	    && !(_param_ekf2_ev_ctrl.get() & static_cast<int32_t>(EvCtrl::VPOS))) {
		_param_ekf2_ev_ctrl.set(_param_ekf2_ev_ctrl.get() | static_cast<int32_t>(EvCtrl::VPOS));
		_param_ekf2_ev_ctrl.commit();
		mavlink_log_critical(&_mavlink_log_pub, "EV vertical position enabled by EKF2_HGT_REF\n");
		/* EVENT
		 * @description <param>EKF2_EV_CTRL</param> is set to {1:.0}.
		 */
		events::send<float>(events::ID("ekf2_hgt_ref_ev"), events::Log::Warning,
				    "EV vertical position enabled by EKF2_HGT_REF", _param_ekf2_ev_ctrl.get());
	}

	// EV EKF2_AID_MASK -> EKF2_EV_CTRL
	if ((_param_ekf2_aid_mask.get() & SensorFusionMask::DEPRECATED_USE_EXT_VIS_VEL)
	    || (_param_ekf2_aid_mask.get() & SensorFusionMask::DEPRECATED_USE_EXT_VIS_POS)
	    || (_param_ekf2_aid_mask.get() & SensorFusionMask::DEPRECATED_USE_EXT_VIS_YAW)
	   ) {

		// EKF2_EV_CTRL set VEL bit
		if ((_param_ekf2_aid_mask.get() & SensorFusionMask::DEPRECATED_USE_EXT_VIS_VEL)) {
			_param_ekf2_ev_ctrl.set(_param_ekf2_ev_ctrl.get() | static_cast<int32_t>(EvCtrl::VEL));
		}

		// EKF2_EV_CTRL set HPOS/VPOS bits
		if ((_param_ekf2_aid_mask.get() & SensorFusionMask::DEPRECATED_USE_EXT_VIS_POS)) {
			_param_ekf2_ev_ctrl.set(_param_ekf2_ev_ctrl.get()
						| static_cast<int32_t>(EvCtrl::HPOS) | static_cast<int32_t>(EvCtrl::VPOS));
		}

		// EKF2_EV_CTRL set YAW bit
		if ((_param_ekf2_aid_mask.get() & SensorFusionMask::DEPRECATED_USE_EXT_VIS_YAW)) {
			_param_ekf2_ev_ctrl.set(_param_ekf2_ev_ctrl.get() | static_cast<int32_t>(EvCtrl::YAW));
		}

		_param_ekf2_aid_mask.set(_param_ekf2_aid_mask.get() & ~(SensorFusionMask::DEPRECATED_USE_EXT_VIS_VEL));
		_param_ekf2_aid_mask.set(_param_ekf2_aid_mask.get() & ~(SensorFusionMask::DEPRECATED_USE_EXT_VIS_POS));
		_param_ekf2_aid_mask.set(_param_ekf2_aid_mask.get() & ~(SensorFusionMask::DEPRECATED_USE_EXT_VIS_YAW));

		_param_ekf2_ev_ctrl.commit();
		_param_ekf2_aid_mask.commit();

		mavlink_log_critical(&_mavlink_log_pub, "EKF2 EV use EKF2_EV_CTRL instead of EKF2_AID_MASK\n");
		/* EVENT
		 * @description <param>EKF2_AID_MASK</param> is set to {1:.0}.
		 */
		events::send<float>(events::ID("ekf2_aid_mask_ev"), events::Log::Warning,
				    "Use EKF2_EV_CTRL instead", _param_ekf2_aid_mask.get());
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	// IMU EKF2_AID_MASK -> EKF2_IMU_CTRL (2023-01-31)
	if (_param_ekf2_aid_mask.get() & SensorFusionMask::DEPRECATED_INHIBIT_ACC_BIAS) {

		// EKF2_IMU_CTRL set disable accel bias bit
		_param_ekf2_imu_ctrl.set(_param_ekf2_imu_ctrl.get() & ~(static_cast<int32_t>(ImuCtrl::AccelBias)));

		// EKF2_AID_MASK clear inhibit accel bias bit
		_param_ekf2_aid_mask.set(_param_ekf2_aid_mask.get() & ~(SensorFusionMask::DEPRECATED_INHIBIT_ACC_BIAS));

		_param_ekf2_imu_ctrl.commit();
		_param_ekf2_aid_mask.commit();

		mavlink_log_critical(&_mavlink_log_pub, "EKF2 IMU accel bias inhibit use EKF2_IMU_CTRL instead of EKF2_AID_MASK\n");
		/* EVENT
		 * @description <param>EKF2_AID_MASK</param> is set to {1:.0}.
		 */
		events::send<float>(events::ID("ekf2_aid_mask_imu"), events::Log::Warning,
				    "Use EKF2_IMU_CTRL instead", _param_ekf2_aid_mask.get());
	}

#if defined(CONFIG_EKF2_DRAG_FUSION)

	if (_param_ekf2_aid_mask.get() & SensorFusionMask::DEPRECATED_USE_DRAG) {
		// EKF2_DRAG_CTRL enable drag fusion
		_param_ekf2_drag_ctrl.set(1);

		// EKF2_AID_MASK clear deprecated bits
		_param_ekf2_aid_mask.set(_param_ekf2_aid_mask.get() & ~(SensorFusionMask::DEPRECATED_USE_DRAG));

		_param_ekf2_drag_ctrl.commit();
		_param_ekf2_aid_mask.commit();

		mavlink_log_critical(&_mavlink_log_pub, "EKF2 drag fusion use EKF2_DRAG_CTRL instead of EKF2_AID_MASK\n");
		/* EVENT
		 * @description <param>EKF2_AID_MASK</param> is set to {1:.0}.
		 */
		events::send<float>(events::ID("ekf2_aid_mask_drag"), events::Log::Warning,
				    "Use EKF2_DRAG_CTRL instead", _param_ekf2_aid_mask.get());
	}

#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_OPTICAL_FLOW)

	// IMU EKF2_AID_MASK -> EKF2_OF_CTRL (2023-04-26)
	if (_param_ekf2_aid_mask.get() & SensorFusionMask::DEPRECATED_USE_OPT_FLOW) {
		// EKF2_OF_CTRL enable flow fusion
		_param_ekf2_of_ctrl.set(1);

		// EKF2_AID_MASK clear deprecated bit
		_param_ekf2_aid_mask.set(_param_ekf2_aid_mask.get() & ~(SensorFusionMask::DEPRECATED_USE_OPT_FLOW));

		_param_ekf2_of_ctrl.commit();
		_param_ekf2_aid_mask.commit();

		mavlink_log_critical(&_mavlink_log_pub, "EKF2 optical flow use EKF2_OF_CTRL instead of EKF2_AID_MASK\n");
		/* EVENT
		 * @description <param>EKF2_AID_MASK</param> is set to {1:.0}.
		 */
		events::send<float>(events::ID("ekf2_aid_mask_opt_flow"), events::Log::Warning,
				    "Use EKF2_OF_CTRL instead", _param_ekf2_aid_mask.get());
	}

#endif // CONFIG_EKF2_OPTICAL_FLOW
}

void EKF2::PublishAidSourceStatus(const hrt_abstime &timestamp)
{
#if defined(CONFIG_EKF2_AIRSPEED)
	// airspeed
	PublishAidSourceStatus(_ekf.aid_src_airspeed(), _status_airspeed_pub_last, _estimator_aid_src_airspeed_pub);
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_SIDESLIP)
	// sideslip
	PublishAidSourceStatus(_ekf.aid_src_sideslip(), _status_sideslip_pub_last, _estimator_aid_src_sideslip_pub);
#endif // CONFIG_EKF2_SIDESLIP
	// baro height
	PublishAidSourceStatus(_ekf.aid_src_baro_hgt(), _status_baro_hgt_pub_last, _estimator_aid_src_baro_hgt_pub);

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// RNG height
	PublishAidSourceStatus(_ekf.aid_src_rng_hgt(), _status_rng_hgt_pub_last, _estimator_aid_src_rng_hgt_pub);
#endif // CONFIG_EKF2_RANGE_FINDER

	// fake position
	PublishAidSourceStatus(_ekf.aid_src_fake_pos(), _status_fake_pos_pub_last, _estimator_aid_src_fake_pos_pub);
	PublishAidSourceStatus(_ekf.aid_src_fake_hgt(), _status_fake_hgt_pub_last, _estimator_aid_src_fake_hgt_pub);

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// external vision (EV) hgt/pos/vel/yaw
	PublishAidSourceStatus(_ekf.aid_src_ev_hgt(), _status_ev_hgt_pub_last, _estimator_aid_src_ev_hgt_pub);
	PublishAidSourceStatus(_ekf.aid_src_ev_pos(), _status_ev_pos_pub_last, _estimator_aid_src_ev_pos_pub);
	PublishAidSourceStatus(_ekf.aid_src_ev_vel(), _status_ev_vel_pub_last, _estimator_aid_src_ev_vel_pub);
	PublishAidSourceStatus(_ekf.aid_src_ev_yaw(), _status_ev_yaw_pub_last, _estimator_aid_src_ev_yaw_pub);
#endif // CONFIG_EKF2_EXTERNAL_VISION

	// GNSS hgt/pos/vel/yaw
	PublishAidSourceStatus(_ekf.aid_src_gnss_hgt(), _status_gnss_hgt_pub_last, _estimator_aid_src_gnss_hgt_pub);
	PublishAidSourceStatus(_ekf.aid_src_gnss_pos(), _status_gnss_pos_pub_last, _estimator_aid_src_gnss_pos_pub);
	PublishAidSourceStatus(_ekf.aid_src_gnss_vel(), _status_gnss_vel_pub_last, _estimator_aid_src_gnss_vel_pub);
#if defined(CONFIG_EKF2_GNSS_YAW)
	PublishAidSourceStatus(_ekf.aid_src_gnss_yaw(), _status_gnss_yaw_pub_last, _estimator_aid_src_gnss_yaw_pub);
#endif // CONFIG_EKF2_GNSS_YAW

	// mag heading
	PublishAidSourceStatus(_ekf.aid_src_mag_heading(), _status_mag_heading_pub_last, _estimator_aid_src_mag_heading_pub);

	// mag 3d
	PublishAidSourceStatus(_ekf.aid_src_mag(), _status_mag_pub_last, _estimator_aid_src_mag_pub);

	// gravity
	PublishAidSourceStatus(_ekf.aid_src_gravity(), _status_gravity_pub_last, _estimator_aid_src_gravity_pub);

#if defined(CONFIG_EKF2_AUXVEL)
	// aux velocity
	PublishAidSourceStatus(_ekf.aid_src_aux_vel(), _status_aux_vel_pub_last, _estimator_aid_src_aux_vel_pub);
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// optical flow
	PublishAidSourceStatus(_ekf.aid_src_optical_flow(), _status_optical_flow_pub_last, _estimator_aid_src_optical_flow_pub);
	PublishAidSourceStatus(_ekf.aid_src_terrain_optical_flow(), _status_terrain_optical_flow_pub_last,
			       _estimator_aid_src_terrain_optical_flow_pub);
#endif // CONFIG_EKF2_OPTICAL_FLOW
}

void EKF2::PublishAttitude(const hrt_abstime &timestamp)
{
	if (_ekf.attitude_valid()) {
		// generate vehicle attitude quaternion data
		vehicle_attitude_s att;
		att.timestamp_sample = timestamp;
		_ekf.getQuaternion().copyTo(att.q);

		_ekf.get_quat_reset(&att.delta_q_reset[0], &att.quat_reset_counter);
		att.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_attitude_pub.publish(att);

	}  else if (_replay_mode) {
		// in replay mode we have to tell the replay module not to wait for an update
		// we do this by publishing an attitude with zero timestamp
		vehicle_attitude_s att{};
		_attitude_pub.publish(att);
	}
}

void EKF2::PublishBaroBias(const hrt_abstime &timestamp)
{
	if (_ekf.aid_src_baro_hgt().timestamp_sample != 0) {
		const BiasEstimator::status &status = _ekf.getBaroBiasEstimatorStatus();

		if (fabsf(status.bias - _last_baro_bias_published) > 0.001f) {
			_estimator_baro_bias_pub.publish(fillEstimatorBiasMsg(status, _ekf.aid_src_baro_hgt().timestamp_sample, timestamp,
							 _device_id_baro));

			_last_baro_bias_published = status.bias;
		}
	}
}

void EKF2::PublishGnssHgtBias(const hrt_abstime &timestamp)
{
	if (_ekf.get_gps_sample_delayed().time_us != 0) {
		const BiasEstimator::status &status = _ekf.getGpsHgtBiasEstimatorStatus();

		if (fabsf(status.bias - _last_gnss_hgt_bias_published) > 0.001f) {
			_estimator_gnss_hgt_bias_pub.publish(fillEstimatorBiasMsg(status, _ekf.get_gps_sample_delayed().time_us, timestamp));

			_last_gnss_hgt_bias_published = status.bias;
		}
	}
}

#if defined(CONFIG_EKF2_RANGE_FINDER)
void EKF2::PublishRngHgtBias(const hrt_abstime &timestamp)
{
	if (_ekf.get_rng_sample_delayed().time_us != 0) {
		const BiasEstimator::status &status = _ekf.getRngHgtBiasEstimatorStatus();

		if (fabsf(status.bias - _last_rng_hgt_bias_published) > 0.001f) {
			_estimator_rng_hgt_bias_pub.publish(fillEstimatorBiasMsg(status, _ekf.get_rng_sample_delayed().time_us, timestamp));

			_last_rng_hgt_bias_published = status.bias;
		}
	}
}
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
void EKF2::PublishEvPosBias(const hrt_abstime &timestamp)
{
	if (_ekf.aid_src_ev_hgt().timestamp_sample) {

		estimator_bias3d_s bias{};

		// height
		BiasEstimator::status bias_est_status[3];
		bias_est_status[0] = _ekf.getEvPosBiasEstimatorStatus(0);
		bias_est_status[1] = _ekf.getEvPosBiasEstimatorStatus(1);
		bias_est_status[2] = _ekf.getEvHgtBiasEstimatorStatus();

		for (int i = 0; i < 3; i++) {
			bias.bias[i] = bias_est_status[i].bias;
			bias.bias_var[i] = bias_est_status[i].bias_var;

			bias.innov[i] = bias_est_status[i].innov;
			bias.innov_var[i] = bias_est_status[i].innov_var;
			bias.innov_test_ratio[i] = bias_est_status[i].innov_test_ratio;
		}

		const Vector3f bias_vec{bias.bias};

		if ((bias_vec - _last_ev_bias_published).longerThan(0.01f)) {
			bias.timestamp_sample = _ekf.aid_src_ev_hgt().timestamp_sample;
			bias.timestamp = hrt_absolute_time();
			_estimator_ev_pos_bias_pub.publish(bias);

			_last_ev_bias_published = Vector3f(bias.bias);
		}
	}
}
#endif // CONFIG_EKF2_EXTERNAL_VISION

estimator_bias_s EKF2::fillEstimatorBiasMsg(const BiasEstimator::status &status, uint64_t timestamp_sample_us,
		uint64_t timestamp, uint32_t device_id)
{
	estimator_bias_s bias{};
	bias.timestamp_sample = timestamp_sample_us;
	bias.device_id = device_id;
	bias.bias = status.bias;
	bias.bias_var = status.bias_var;
	bias.innov = status.innov;
	bias.innov_var = status.innov_var;
	bias.innov_test_ratio = status.innov_test_ratio;
	bias.timestamp = _replay_mode ? timestamp : hrt_absolute_time();

	return bias;
}

void EKF2::PublishEventFlags(const hrt_abstime &timestamp)
{
	// information events
	uint32_t information_events = _ekf.information_event_status().value;
	bool information_event_updated = false;

	if (information_events != 0) {
		information_event_updated = true;
		_filter_information_event_changes++;
	}

	// warning events
	uint32_t warning_events = _ekf.warning_event_status().value;
	bool warning_event_updated = false;

	if (warning_events != 0) {
		warning_event_updated = true;
		_filter_warning_event_changes++;
	}

	if (information_event_updated || warning_event_updated) {
		estimator_event_flags_s event_flags{};
		event_flags.timestamp_sample = _ekf.time_delayed_us();

		event_flags.information_event_changes           = _filter_information_event_changes;
		event_flags.gps_checks_passed                   = _ekf.information_event_flags().gps_checks_passed;
		event_flags.reset_vel_to_gps                    = _ekf.information_event_flags().reset_vel_to_gps;
		event_flags.reset_vel_to_flow                   = _ekf.information_event_flags().reset_vel_to_flow;
		event_flags.reset_vel_to_vision                 = _ekf.information_event_flags().reset_vel_to_vision;
		event_flags.reset_vel_to_zero                   = _ekf.information_event_flags().reset_vel_to_zero;
		event_flags.reset_pos_to_last_known             = _ekf.information_event_flags().reset_pos_to_last_known;
		event_flags.reset_pos_to_gps                    = _ekf.information_event_flags().reset_pos_to_gps;
		event_flags.reset_pos_to_vision                 = _ekf.information_event_flags().reset_pos_to_vision;
		event_flags.starting_gps_fusion                 = _ekf.information_event_flags().starting_gps_fusion;
		event_flags.starting_vision_pos_fusion          = _ekf.information_event_flags().starting_vision_pos_fusion;
		event_flags.starting_vision_vel_fusion          = _ekf.information_event_flags().starting_vision_vel_fusion;
		event_flags.starting_vision_yaw_fusion          = _ekf.information_event_flags().starting_vision_yaw_fusion;
		event_flags.yaw_aligned_to_imu_gps              = _ekf.information_event_flags().yaw_aligned_to_imu_gps;
		event_flags.reset_hgt_to_baro                   = _ekf.information_event_flags().reset_hgt_to_baro;
		event_flags.reset_hgt_to_gps                    = _ekf.information_event_flags().reset_hgt_to_gps;
		event_flags.reset_hgt_to_rng                    = _ekf.information_event_flags().reset_hgt_to_rng;
		event_flags.reset_hgt_to_ev                     = _ekf.information_event_flags().reset_hgt_to_ev;

		event_flags.warning_event_changes               = _filter_warning_event_changes;
		event_flags.gps_quality_poor                    = _ekf.warning_event_flags().gps_quality_poor;
		event_flags.gps_fusion_timout                   = _ekf.warning_event_flags().gps_fusion_timout;
		event_flags.gps_data_stopped                    = _ekf.warning_event_flags().gps_data_stopped;
		event_flags.gps_data_stopped_using_alternate    = _ekf.warning_event_flags().gps_data_stopped_using_alternate;
		event_flags.height_sensor_timeout               = _ekf.warning_event_flags().height_sensor_timeout;
		event_flags.stopping_navigation                 = _ekf.warning_event_flags().stopping_mag_use;
		event_flags.invalid_accel_bias_cov_reset        = _ekf.warning_event_flags().invalid_accel_bias_cov_reset;
		event_flags.bad_yaw_using_gps_course            = _ekf.warning_event_flags().bad_yaw_using_gps_course;
		event_flags.stopping_mag_use                    = _ekf.warning_event_flags().stopping_mag_use;
		event_flags.vision_data_stopped                 = _ekf.warning_event_flags().vision_data_stopped;
		event_flags.emergency_yaw_reset_mag_stopped     = _ekf.warning_event_flags().emergency_yaw_reset_mag_stopped;
		event_flags.emergency_yaw_reset_gps_yaw_stopped = _ekf.warning_event_flags().emergency_yaw_reset_gps_yaw_stopped;

		event_flags.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_estimator_event_flags_pub.update(event_flags);

		_last_event_flags_publish = event_flags.timestamp;

		_ekf.clear_information_events();
		_ekf.clear_warning_events();

	} else if ((_last_event_flags_publish != 0) && (timestamp >= _last_event_flags_publish + 1_s)) {
		// continue publishing periodically
		_estimator_event_flags_pub.get().timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_estimator_event_flags_pub.update();
		_last_event_flags_publish = _estimator_event_flags_pub.get().timestamp;
	}
}

void EKF2::PublishGlobalPosition(const hrt_abstime &timestamp)
{
	if (_ekf.global_position_is_valid()) {
		const Vector3f position{_ekf.getPosition()};

		// generate and publish global position data
		vehicle_global_position_s global_pos{};
		global_pos.timestamp_sample = timestamp;

		// Position of local NED origin in GPS / WGS84 frame
		_ekf.global_origin().reproject(position(0), position(1), global_pos.lat, global_pos.lon);

		global_pos.alt = -position(2) + _ekf.getEkfGlobalOriginAltitude(); // Altitude AMSL in meters
		global_pos.alt_ellipsoid = filter_altitude_ellipsoid(global_pos.alt);

		// delta_alt, alt_reset_counter
		//  global altitude has opposite sign of local down position
		float delta_z = 0.f;
		uint8_t z_reset_counter = 0;
		_ekf.get_posD_reset(&delta_z, &z_reset_counter);
		global_pos.delta_alt = -delta_z;
		global_pos.alt_reset_counter = z_reset_counter;

		// lat_lon_reset_counter
		float delta_xy[2] {};
		uint8_t xy_reset_counter = 0;
		_ekf.get_posNE_reset(delta_xy, &xy_reset_counter);
		global_pos.lat_lon_reset_counter = xy_reset_counter;

		_ekf.get_ekf_gpos_accuracy(&global_pos.eph, &global_pos.epv);

		global_pos.terrain_alt = NAN;
		global_pos.terrain_alt_valid = false;

#if defined(CONFIG_EKF2_RANGE_FINDER)

		if (_ekf.isTerrainEstimateValid()) {
			// Terrain altitude in m, WGS84
			global_pos.terrain_alt = _ekf.getEkfGlobalOriginAltitude() - _ekf.getTerrainVertPos();
			global_pos.terrain_alt_valid = true;
		}

#endif // CONFIG_EKF2_RANGE_FINDER

		global_pos.dead_reckoning = _ekf.control_status_flags().inertial_dead_reckoning
					    || _ekf.control_status_flags().wind_dead_reckoning;

		global_pos.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_global_position_pub.publish(global_pos);
	}
}

void EKF2::PublishGpsStatus(const hrt_abstime &timestamp)
{
	const hrt_abstime timestamp_sample = _ekf.get_gps_sample_delayed().time_us;

	if (timestamp_sample == _last_gps_status_published) {
		return;
	}

	estimator_gps_status_s estimator_gps_status{};
	estimator_gps_status.timestamp_sample = timestamp_sample;

	estimator_gps_status.position_drift_rate_horizontal_m_s = _ekf.gps_horizontal_position_drift_rate_m_s();
	estimator_gps_status.position_drift_rate_vertical_m_s   = _ekf.gps_vertical_position_drift_rate_m_s();
	estimator_gps_status.filtered_horizontal_speed_m_s      = _ekf.gps_filtered_horizontal_velocity_m_s();

	estimator_gps_status.checks_passed = _ekf.gps_checks_passed();

	estimator_gps_status.check_fail_gps_fix          = _ekf.gps_check_fail_status_flags().fix;
	estimator_gps_status.check_fail_min_sat_count    = _ekf.gps_check_fail_status_flags().nsats;
	estimator_gps_status.check_fail_max_pdop         = _ekf.gps_check_fail_status_flags().pdop;
	estimator_gps_status.check_fail_max_horz_err     = _ekf.gps_check_fail_status_flags().hacc;
	estimator_gps_status.check_fail_max_vert_err     = _ekf.gps_check_fail_status_flags().vacc;
	estimator_gps_status.check_fail_max_spd_err      = _ekf.gps_check_fail_status_flags().sacc;
	estimator_gps_status.check_fail_max_horz_drift   = _ekf.gps_check_fail_status_flags().hdrift;
	estimator_gps_status.check_fail_max_vert_drift   = _ekf.gps_check_fail_status_flags().vdrift;
	estimator_gps_status.check_fail_max_horz_spd_err = _ekf.gps_check_fail_status_flags().hspeed;
	estimator_gps_status.check_fail_max_vert_spd_err = _ekf.gps_check_fail_status_flags().vspeed;

	estimator_gps_status.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_gps_status_pub.publish(estimator_gps_status);


	_last_gps_status_published = timestamp_sample;
}

void EKF2::PublishInnovations(const hrt_abstime &timestamp)
{
	// publish estimator innovation data
	estimator_innovations_s innovations{};
	innovations.timestamp_sample = _ekf.time_delayed_us();
	_ekf.getGpsVelPosInnov(innovations.gps_hvel, innovations.gps_vvel, innovations.gps_hpos, innovations.gps_vpos);
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	_ekf.getEvVelPosInnov(innovations.ev_hvel, innovations.ev_vvel, innovations.ev_hpos, innovations.ev_vpos);
#endif // CONFIG_EKF2_EXTERNAL_VISION
	_ekf.getBaroHgtInnov(innovations.baro_vpos);
#if defined(CONFIG_EKF2_RANGE_FINDER)
	_ekf.getRngHgtInnov(innovations.rng_vpos);
#endif // CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_AUXVEL)
	_ekf.getAuxVelInnov(innovations.aux_hvel);
#endif // CONFIG_EKF2_AUXVEL
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	_ekf.getFlowInnov(innovations.flow);
	_ekf.getTerrainFlowInnov(innovations.terr_flow);
#endif // CONFIG_EKF2_OPTICAL_FLOW
	_ekf.getHeadingInnov(innovations.heading);
	_ekf.getMagInnov(innovations.mag_field);
#if defined(CONFIG_EKF2_DRAG_FUSION)
	_ekf.getDragInnov(innovations.drag);
#endif // CONFIG_EKF2_DRAG_FUSION
#if defined(CONFIG_EKF2_AIRSPEED)
	_ekf.getAirspeedInnov(innovations.airspeed);
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_SIDESLIP)
	_ekf.getBetaInnov(innovations.beta);
#endif // CONFIG_EKF2_SIDESLIP
#if defined(CONFIG_EKF2_RANGE_FINDER)
	_ekf.getHaglInnov(innovations.hagl);
	_ekf.getHaglRateInnov(innovations.hagl_rate);
#endif // CONFIG_EKF2_RANGE_FINDER
	_ekf.getGravityInnov(innovations.gravity);
	// Not yet supported
	innovations.aux_vvel = NAN;

	innovations.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_innovations_pub.publish(innovations);

	// calculate noise filtered velocity innovations which are used for pre-flight checking
	if (_ekf.control_status_prev_flags().in_air != _ekf.control_status_flags().in_air) {
		// fully reset on takeoff or landing
		_preflt_checker.reset();
	}

	if (!_ekf.control_status_flags().in_air) {
		// TODO: move to run before publications
		_preflt_checker.setUsingGpsAiding(_ekf.control_status_flags().gps);
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
		_preflt_checker.setUsingFlowAiding(_ekf.control_status_flags().opt_flow);

		// set dist bottom to scale flow innovation
		const float dist_bottom = _ekf.getTerrainVertPos() - _ekf.getPosition()(2);
		_preflt_checker.setDistBottom(dist_bottom);
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
		_preflt_checker.setUsingEvPosAiding(_ekf.control_status_flags().ev_pos);
		_preflt_checker.setUsingEvVelAiding(_ekf.control_status_flags().ev_vel);
		_preflt_checker.setUsingEvHgtAiding(_ekf.control_status_flags().ev_hgt);
#endif // CONFIG_EKF2_EXTERNAL_VISION

		_preflt_checker.setUsingBaroHgtAiding(_ekf.control_status_flags().baro_hgt);
		_preflt_checker.setUsingGpsHgtAiding(_ekf.control_status_flags().gps_hgt);
#if defined(CONFIG_EKF2_RANGE_FINDER)
		_preflt_checker.setUsingRngHgtAiding(_ekf.control_status_flags().rng_hgt);
#endif // CONFIG_EKF2_RANGE_FINDER

		_preflt_checker.setVehicleCanObserveHeadingInFlight(_ekf.control_status_flags().fixed_wing);

		_preflt_checker.update(_ekf.get_dt_ekf_avg(), innovations);
	}
}

void EKF2::PublishInnovationTestRatios(const hrt_abstime &timestamp)
{
	// publish estimator innovation test ratio data
	estimator_innovations_s test_ratios{};
	test_ratios.timestamp_sample = _ekf.time_delayed_us();
	_ekf.getGpsVelPosInnovRatio(test_ratios.gps_hvel[0], test_ratios.gps_vvel, test_ratios.gps_hpos[0],
				    test_ratios.gps_vpos);
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	_ekf.getEvVelPosInnovRatio(test_ratios.ev_hvel[0], test_ratios.ev_vvel, test_ratios.ev_hpos[0], test_ratios.ev_vpos);
#endif // CONFIG_EKF2_EXTERNAL_VISION
	_ekf.getBaroHgtInnovRatio(test_ratios.baro_vpos);
#if defined(CONFIG_EKF2_RANGE_FINDER)
	_ekf.getRngHgtInnovRatio(test_ratios.rng_vpos);
#endif // CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_AUXVEL)
	_ekf.getAuxVelInnovRatio(test_ratios.aux_hvel[0]);
#endif // CONFIG_EKF2_AUXVEL
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	_ekf.getFlowInnovRatio(test_ratios.flow[0]);
	_ekf.getTerrainFlowInnovRatio(test_ratios.terr_flow[0]);
#endif // CONFIG_EKF2_OPTICAL_FLOW
	_ekf.getHeadingInnovRatio(test_ratios.heading);
	_ekf.getMagInnovRatio(test_ratios.mag_field[0]);
#if defined(CONFIG_EKF2_DRAG_FUSION)
	_ekf.getDragInnovRatio(&test_ratios.drag[0]);
#endif // CONFIG_EKF2_DRAG_FUSION
#if defined(CONFIG_EKF2_AIRSPEED)
	_ekf.getAirspeedInnovRatio(test_ratios.airspeed);
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_SIDESLIP)
	_ekf.getBetaInnovRatio(test_ratios.beta);
#endif // CONFIG_EKF2_SIDESLIP
#if defined(CONFIG_EKF2_RANGE_FINDER)
	_ekf.getHaglInnovRatio(test_ratios.hagl);
	_ekf.getHaglRateInnovRatio(test_ratios.hagl_rate);
#endif // CONFIG_EKF2_RANGE_FINDER
	_ekf.getGravityInnovRatio(test_ratios.gravity[0]);
	// Not yet supported
	test_ratios.aux_vvel = NAN;

	test_ratios.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_innovation_test_ratios_pub.publish(test_ratios);
}

void EKF2::PublishInnovationVariances(const hrt_abstime &timestamp)
{
	// publish estimator innovation variance data
	estimator_innovations_s variances{};
	variances.timestamp_sample = _ekf.time_delayed_us();
	_ekf.getGpsVelPosInnovVar(variances.gps_hvel, variances.gps_vvel, variances.gps_hpos, variances.gps_vpos);
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	_ekf.getEvVelPosInnovVar(variances.ev_hvel, variances.ev_vvel, variances.ev_hpos, variances.ev_vpos);
#endif // CONFIG_EKF2_EXTERNAL_VISION
	_ekf.getBaroHgtInnovVar(variances.baro_vpos);
#if defined(CONFIG_EKF2_RANGE_FINDER)
	_ekf.getRngHgtInnovVar(variances.rng_vpos);
#endif // CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_AUXVEL)
	_ekf.getAuxVelInnovVar(variances.aux_hvel);
#endif // CONFIG_EKF2_AUXVEL
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	_ekf.getFlowInnovVar(variances.flow);
	_ekf.getTerrainFlowInnovVar(variances.terr_flow);
#endif // CONFIG_EKF2_OPTICAL_FLOW
	_ekf.getHeadingInnovVar(variances.heading);
	_ekf.getMagInnovVar(variances.mag_field);
#if defined(CONFIG_EKF2_DRAG_FUSION)
	_ekf.getDragInnovVar(variances.drag);
#endif // CONFIG_EKF2_DRAG_FUSION
#if defined(CONFIG_EKF2_AIRSPEED)
	_ekf.getAirspeedInnovVar(variances.airspeed);
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_SIDESLIP)
	_ekf.getBetaInnovVar(variances.beta);
#endif // CONFIG_EKF2_SIDESLIP
#if defined(CONFIG_EKF2_RANGE_FINDER)
	_ekf.getHaglInnovVar(variances.hagl);
	_ekf.getHaglRateInnovVar(variances.hagl_rate);
#endif // CONFIG_EKF2_RANGE_FINDER
	_ekf.getGravityInnovVar(variances.gravity);
	// Not yet supported
	variances.aux_vvel = NAN;

	variances.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_innovation_variances_pub.publish(variances);
}

void EKF2::PublishLocalPosition(const hrt_abstime &timestamp)
{
	vehicle_local_position_s lpos{};
	// generate vehicle local position data
	lpos.timestamp_sample = timestamp;

	// Position of body origin in local NED frame
	const Vector3f position{_ekf.getPosition()};
	lpos.x = position(0);
	lpos.y = position(1);
	lpos.z = position(2);

	// Velocity of body origin in local NED frame (m/s)
	const Vector3f velocity{_ekf.getVelocity()};
	lpos.vx = velocity(0);
	lpos.vy = velocity(1);
	lpos.vz = velocity(2);

	// vertical position time derivative (m/s)
	lpos.z_deriv = _ekf.getVerticalPositionDerivative();

	// Acceleration of body origin in local frame
	const Vector3f vel_deriv{_ekf.getVelocityDerivative()};
	lpos.ax = vel_deriv(0);
	lpos.ay = vel_deriv(1);
	lpos.az = vel_deriv(2);

	lpos.xy_valid = _ekf.local_position_is_valid();
	lpos.v_xy_valid = _ekf.local_position_is_valid();

	// TODO: some modules (e.g.: mc_pos_control) don't handle v_z_valid != z_valid properly
	lpos.z_valid = _ekf.isLocalVerticalPositionValid() || _ekf.isLocalVerticalVelocityValid();
	lpos.v_z_valid = _ekf.isLocalVerticalVelocityValid() || _ekf.isLocalVerticalPositionValid();

	// Position of local NED origin in GPS / WGS84 frame
	if (_ekf.global_origin_valid()) {
		lpos.ref_timestamp = _ekf.global_origin().getProjectionReferenceTimestamp();
		lpos.ref_lat = _ekf.global_origin().getProjectionReferenceLat(); // Reference point latitude in degrees
		lpos.ref_lon = _ekf.global_origin().getProjectionReferenceLon(); // Reference point longitude in degrees
		lpos.ref_alt = _ekf.getEkfGlobalOriginAltitude();           // Reference point in MSL altitude meters
		lpos.xy_global = true;
		lpos.z_global = true;

	} else {
		lpos.ref_timestamp = 0;
		lpos.ref_lat = static_cast<double>(NAN);
		lpos.ref_lon = static_cast<double>(NAN);
		lpos.ref_alt = NAN;
		lpos.xy_global = false;
		lpos.z_global = false;
	}

	Quatf delta_q_reset;
	_ekf.get_quat_reset(&delta_q_reset(0), &lpos.heading_reset_counter);

	lpos.heading = Eulerf(_ekf.getQuaternion()).psi();
	lpos.delta_heading = Eulerf(delta_q_reset).psi();
	lpos.heading_good_for_control = _ekf.isYawFinalAlignComplete();

	// Distance to bottom surface (ground) in meters, must be positive
	lpos.dist_bottom = math::max(_ekf.getTerrainVertPos() - lpos.z, 0.f);
	lpos.dist_bottom_valid = _ekf.isTerrainEstimateValid();
	lpos.dist_bottom_sensor_bitfield = _ekf.getTerrainEstimateSensorBitfield();

	_ekf.get_ekf_lpos_accuracy(&lpos.eph, &lpos.epv);
	_ekf.get_ekf_vel_accuracy(&lpos.evh, &lpos.evv);

	// get state reset information of position and velocity
	_ekf.get_posD_reset(&lpos.delta_z, &lpos.z_reset_counter);
	_ekf.get_velD_reset(&lpos.delta_vz, &lpos.vz_reset_counter);
	_ekf.get_posNE_reset(&lpos.delta_xy[0], &lpos.xy_reset_counter);
	_ekf.get_velNE_reset(&lpos.delta_vxy[0], &lpos.vxy_reset_counter);

	lpos.dead_reckoning = _ekf.control_status_flags().inertial_dead_reckoning
			      || _ekf.control_status_flags().wind_dead_reckoning;

	// get control limit information
	_ekf.get_ekf_ctrl_limits(&lpos.vxy_max, &lpos.vz_max, &lpos.hagl_min, &lpos.hagl_max);

	// convert NaN to INFINITY
	if (!PX4_ISFINITE(lpos.vxy_max)) {
		lpos.vxy_max = INFINITY;
	}

	if (!PX4_ISFINITE(lpos.vz_max)) {
		lpos.vz_max = INFINITY;
	}

	if (!PX4_ISFINITE(lpos.hagl_min)) {
		lpos.hagl_min = INFINITY;
	}

	if (!PX4_ISFINITE(lpos.hagl_max)) {
		lpos.hagl_max = INFINITY;
	}

	// publish vehicle local position data
	lpos.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_local_position_pub.publish(lpos);
}

void EKF2::PublishOdometry(const hrt_abstime &timestamp, const imuSample &imu_sample)
{
	// generate vehicle odometry data
	vehicle_odometry_s odom;
	odom.timestamp_sample = imu_sample.time_us;

	// position
	odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
	_ekf.getPosition().copyTo(odom.position);

	// orientation quaternion
	_ekf.getQuaternion().copyTo(odom.q);

	// velocity
	odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_NED;
	_ekf.getVelocity().copyTo(odom.velocity);

	// angular_velocity
	const Vector3f rates{imu_sample.delta_ang / imu_sample.delta_ang_dt};
	const Vector3f angular_velocity = rates - _ekf.getGyroBias();
	angular_velocity.copyTo(odom.angular_velocity);

	// velocity covariances
	_ekf.velocity_covariances().diag().copyTo(odom.velocity_variance);

	// position covariances
	_ekf.position_covariances().diag().copyTo(odom.position_variance);

	// orientation covariance
	_ekf.calcRotVecVariances().copyTo(odom.orientation_variance);

	odom.reset_counter = _ekf.get_quat_reset_count()
			     + _ekf.get_velNE_reset_count() + _ekf.get_velD_reset_count()
			     + _ekf.get_posNE_reset_count() + _ekf.get_posD_reset_count();

	odom.quality = 0;

	// publish vehicle odometry data
	odom.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_odometry_pub.publish(odom);
}

void EKF2::PublishSensorBias(const hrt_abstime &timestamp)
{
	// estimator_sensor_bias
	const Vector3f gyro_bias{_ekf.getGyroBias()};
	const Vector3f accel_bias{_ekf.getAccelBias()};
	const Vector3f mag_bias{_ekf.getMagBias()};

	// publish at ~1 Hz, or sooner if there's a change
	if ((gyro_bias - _last_gyro_bias_published).longerThan(0.001f)
	    || (accel_bias - _last_accel_bias_published).longerThan(0.001f)
	    || (mag_bias - _last_mag_bias_published).longerThan(0.001f)
	    || (timestamp >= _last_sensor_bias_published + 1_s)) {

		estimator_sensor_bias_s bias{};
		bias.timestamp_sample = _ekf.time_delayed_us();

		// take device ids from sensor_selection_s if not using specific vehicle_imu_s
		if ((_device_id_gyro != 0) && (_param_ekf2_imu_ctrl.get() & static_cast<int32_t>(ImuCtrl::GyroBias))) {
			bias.gyro_device_id = _device_id_gyro;
			gyro_bias.copyTo(bias.gyro_bias);
			bias.gyro_bias_limit = _ekf.getGyroBiasLimit();
			_ekf.getGyroBiasVariance().copyTo(bias.gyro_bias_variance);
			bias.gyro_bias_valid = true;  // TODO
			bias.gyro_bias_stable = _gyro_cal.cal_available;
			_last_gyro_bias_published = gyro_bias;
		}

		if ((_device_id_accel != 0) && (_param_ekf2_imu_ctrl.get() & static_cast<int32_t>(ImuCtrl::AccelBias))) {
			bias.accel_device_id = _device_id_accel;
			accel_bias.copyTo(bias.accel_bias);
			bias.accel_bias_limit = _ekf.getAccelBiasLimit();
			_ekf.getAccelBiasVariance().copyTo(bias.accel_bias_variance);
			bias.accel_bias_valid = true;  // TODO
			bias.accel_bias_stable = _accel_cal.cal_available;
			_last_accel_bias_published = accel_bias;
		}

		if (_device_id_mag != 0) {
			bias.mag_device_id = _device_id_mag;
			mag_bias.copyTo(bias.mag_bias);
			bias.mag_bias_limit = _ekf.getMagBiasLimit();
			_ekf.getMagBiasVariance().copyTo(bias.mag_bias_variance);
			bias.mag_bias_valid = true; // TODO
			bias.mag_bias_stable = _mag_cal.cal_available;
			_last_mag_bias_published = mag_bias;
		}

		bias.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_estimator_sensor_bias_pub.publish(bias);

		_last_sensor_bias_published = bias.timestamp;
	}
}

void EKF2::PublishStates(const hrt_abstime &timestamp)
{
	// publish estimator states
	estimator_states_s states;
	states.timestamp_sample = _ekf.time_delayed_us();
	states.n_states = Ekf::_k_num_states;
	_ekf.getStateAtFusionHorizonAsVector().copyTo(states.states);
	_ekf.covariances_diagonal().copyTo(states.covariances);
	states.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_states_pub.publish(states);
}

void EKF2::PublishStatus(const hrt_abstime &timestamp)
{
	estimator_status_s status{};
	status.timestamp_sample = _ekf.time_delayed_us();

	_ekf.getOutputTrackingError().copyTo(status.output_tracking_error);

	// only report enabled GPS check failures (the param indexes are shifted by 1 bit, because they don't include
	// the GPS Fix bit, which is always checked)
	status.gps_check_fail_flags = _ekf.gps_check_fail_status().value & (((uint16_t)_params->gps_check_mask << 1) | 1);

	status.control_mode_flags = _ekf.control_status().value;
	status.filter_fault_flags = _ekf.fault_status().value;

	uint16_t innov_check_flags_temp = 0;
	_ekf.get_innovation_test_status(innov_check_flags_temp, status.mag_test_ratio,
					status.vel_test_ratio, status.pos_test_ratio,
					status.hgt_test_ratio, status.tas_test_ratio,
					status.hagl_test_ratio, status.beta_test_ratio);

	// Bit mismatch between ecl and Firmware, combine the 2 first bits to preserve msg definition
	// TODO: legacy use only, those flags are also in estimator_status_flags
	status.innovation_check_flags = (innov_check_flags_temp >> 1) | (innov_check_flags_temp & 0x1);

	_ekf.get_ekf_lpos_accuracy(&status.pos_horiz_accuracy, &status.pos_vert_accuracy);
	_ekf.get_ekf_soln_status(&status.solution_status_flags);

	// reset counters
	status.reset_count_vel_ne = _ekf.state_reset_status().reset_count.velNE;
	status.reset_count_vel_d = _ekf.state_reset_status().reset_count.velD;
	status.reset_count_pos_ne = _ekf.state_reset_status().reset_count.posNE;
	status.reset_count_pod_d = _ekf.state_reset_status().reset_count.posD;
	status.reset_count_quat = _ekf.state_reset_status().reset_count.quat;

	status.time_slip = _last_time_slip_us * 1e-6f;

	status.pre_flt_fail_innov_heading = _preflt_checker.hasHeadingFailed();
	status.pre_flt_fail_innov_vel_horiz = _preflt_checker.hasHorizVelFailed();
	status.pre_flt_fail_innov_vel_vert = _preflt_checker.hasVertVelFailed();
	status.pre_flt_fail_innov_height = _preflt_checker.hasHeightFailed();
	status.pre_flt_fail_mag_field_disturbed = _ekf.control_status_flags().mag_field_disturbed;

	status.accel_device_id = _device_id_accel;
	status.baro_device_id = _device_id_baro;
	status.gyro_device_id = _device_id_gyro;
	status.mag_device_id = _device_id_mag;

	status.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_status_pub.publish(status);
}

void EKF2::PublishStatusFlags(const hrt_abstime &timestamp)
{
	// publish at ~ 1 Hz (or immediately if filter control status or fault status changes)
	bool update = (timestamp >= _last_status_flags_publish + 1_s);

	// filter control status
	if (_ekf.control_status().value != _filter_control_status) {
		update = true;
		_filter_control_status = _ekf.control_status().value;
		_filter_control_status_changes++;
	}

	// filter fault status
	if (_ekf.fault_status().value != _filter_fault_status) {
		update = true;
		_filter_fault_status = _ekf.fault_status().value;
		_filter_fault_status_changes++;
	}

	// innovation check fail status
	if (_ekf.innov_check_fail_status().value != _innov_check_fail_status) {
		update = true;
		_innov_check_fail_status = _ekf.innov_check_fail_status().value;
		_innov_check_fail_status_changes++;
	}

	if (update) {
		estimator_status_flags_s status_flags{};
		status_flags.timestamp_sample = _ekf.time_delayed_us();

		status_flags.control_status_changes   = _filter_control_status_changes;
		status_flags.cs_tilt_align            = _ekf.control_status_flags().tilt_align;
		status_flags.cs_yaw_align             = _ekf.control_status_flags().yaw_align;
		status_flags.cs_gps                   = _ekf.control_status_flags().gps;
		status_flags.cs_opt_flow              = _ekf.control_status_flags().opt_flow;
		status_flags.cs_mag_hdg               = _ekf.control_status_flags().mag_hdg;
		status_flags.cs_mag_3d                = _ekf.control_status_flags().mag_3D;
		status_flags.cs_mag_dec               = _ekf.control_status_flags().mag_dec;
		status_flags.cs_in_air                = _ekf.control_status_flags().in_air;
		status_flags.cs_wind                  = _ekf.control_status_flags().wind;
		status_flags.cs_baro_hgt              = _ekf.control_status_flags().baro_hgt;
		status_flags.cs_rng_hgt               = _ekf.control_status_flags().rng_hgt;
		status_flags.cs_gps_hgt               = _ekf.control_status_flags().gps_hgt;
		status_flags.cs_ev_pos                = _ekf.control_status_flags().ev_pos;
		status_flags.cs_ev_yaw                = _ekf.control_status_flags().ev_yaw;
		status_flags.cs_ev_hgt                = _ekf.control_status_flags().ev_hgt;
		status_flags.cs_fuse_beta             = _ekf.control_status_flags().fuse_beta;
		status_flags.cs_mag_field_disturbed   = _ekf.control_status_flags().mag_field_disturbed;
		status_flags.cs_fixed_wing            = _ekf.control_status_flags().fixed_wing;
		status_flags.cs_mag_fault             = _ekf.control_status_flags().mag_fault;
		status_flags.cs_fuse_aspd             = _ekf.control_status_flags().fuse_aspd;
		status_flags.cs_gnd_effect            = _ekf.control_status_flags().gnd_effect;
		status_flags.cs_rng_stuck             = _ekf.control_status_flags().rng_stuck;
		status_flags.cs_gps_yaw               = _ekf.control_status_flags().gps_yaw;
		status_flags.cs_mag_aligned_in_flight = _ekf.control_status_flags().mag_aligned_in_flight;
		status_flags.cs_ev_vel                = _ekf.control_status_flags().ev_vel;
		status_flags.cs_synthetic_mag_z       = _ekf.control_status_flags().synthetic_mag_z;
		status_flags.cs_vehicle_at_rest       = _ekf.control_status_flags().vehicle_at_rest;
		status_flags.cs_gps_yaw_fault         = _ekf.control_status_flags().gps_yaw_fault;
		status_flags.cs_rng_fault             = _ekf.control_status_flags().rng_fault;
		status_flags.cs_inertial_dead_reckoning = _ekf.control_status_flags().inertial_dead_reckoning;
		status_flags.cs_wind_dead_reckoning     = _ekf.control_status_flags().wind_dead_reckoning;
		status_flags.cs_rng_kin_consistent      = _ekf.control_status_flags().rng_kin_consistent;
		status_flags.cs_fake_pos                = _ekf.control_status_flags().fake_pos;
		status_flags.cs_fake_hgt                = _ekf.control_status_flags().fake_hgt;
		status_flags.cs_gravity_vector          = _ekf.control_status_flags().gravity_vector;

		status_flags.fault_status_changes     = _filter_fault_status_changes;
		status_flags.fs_bad_mag_x             = _ekf.fault_status_flags().bad_mag_x;
		status_flags.fs_bad_mag_y             = _ekf.fault_status_flags().bad_mag_y;
		status_flags.fs_bad_mag_z             = _ekf.fault_status_flags().bad_mag_z;
		status_flags.fs_bad_hdg               = _ekf.fault_status_flags().bad_hdg;
		status_flags.fs_bad_mag_decl          = _ekf.fault_status_flags().bad_mag_decl;
		status_flags.fs_bad_airspeed          = _ekf.fault_status_flags().bad_airspeed;
		status_flags.fs_bad_sideslip          = _ekf.fault_status_flags().bad_sideslip;
		status_flags.fs_bad_optflow_x         = _ekf.fault_status_flags().bad_optflow_X;
		status_flags.fs_bad_optflow_y         = _ekf.fault_status_flags().bad_optflow_Y;
		status_flags.fs_bad_vel_n             = _ekf.fault_status_flags().bad_vel_N;
		status_flags.fs_bad_vel_e             = _ekf.fault_status_flags().bad_vel_E;
		status_flags.fs_bad_vel_d             = _ekf.fault_status_flags().bad_vel_D;
		status_flags.fs_bad_pos_n             = _ekf.fault_status_flags().bad_pos_N;
		status_flags.fs_bad_pos_e             = _ekf.fault_status_flags().bad_pos_E;
		status_flags.fs_bad_pos_d             = _ekf.fault_status_flags().bad_pos_D;
		status_flags.fs_bad_acc_bias          = _ekf.fault_status_flags().bad_acc_bias;
		status_flags.fs_bad_acc_vertical      = _ekf.fault_status_flags().bad_acc_vertical;
		status_flags.fs_bad_acc_clipping      = _ekf.fault_status_flags().bad_acc_clipping;

		status_flags.innovation_fault_status_changes = _innov_check_fail_status_changes;
		status_flags.reject_hor_vel                  = _ekf.innov_check_fail_status_flags().reject_hor_vel;
		status_flags.reject_ver_vel                  = _ekf.innov_check_fail_status_flags().reject_ver_vel;
		status_flags.reject_hor_pos                  = _ekf.innov_check_fail_status_flags().reject_hor_pos;
		status_flags.reject_ver_pos                  = _ekf.innov_check_fail_status_flags().reject_ver_pos;
		status_flags.reject_yaw                      = _ekf.innov_check_fail_status_flags().reject_yaw;
		status_flags.reject_airspeed                 = _ekf.innov_check_fail_status_flags().reject_airspeed;
		status_flags.reject_sideslip                 = _ekf.innov_check_fail_status_flags().reject_sideslip;
		status_flags.reject_hagl                     = _ekf.innov_check_fail_status_flags().reject_hagl;
		status_flags.reject_optflow_x                = _ekf.innov_check_fail_status_flags().reject_optflow_X;
		status_flags.reject_optflow_y                = _ekf.innov_check_fail_status_flags().reject_optflow_Y;

		status_flags.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_estimator_status_flags_pub.publish(status_flags);

		_last_status_flags_publish = status_flags.timestamp;
	}
}

void EKF2::PublishYawEstimatorStatus(const hrt_abstime &timestamp)
{
	static_assert(sizeof(yaw_estimator_status_s::yaw) / sizeof(float) == N_MODELS_EKFGSF,
		      "yaw_estimator_status_s::yaw wrong size");

	yaw_estimator_status_s yaw_est_test_data;

	if (_ekf.getDataEKFGSF(&yaw_est_test_data.yaw_composite, &yaw_est_test_data.yaw_variance,
			       yaw_est_test_data.yaw,
			       yaw_est_test_data.innov_vn, yaw_est_test_data.innov_ve,
			       yaw_est_test_data.weight)) {

		yaw_est_test_data.yaw_composite_valid = _ekf.isYawEmergencyEstimateAvailable();
		yaw_est_test_data.timestamp_sample = _ekf.time_delayed_us();
		yaw_est_test_data.timestamp = _replay_mode ? timestamp : hrt_absolute_time();

		_yaw_est_pub.publish(yaw_est_test_data);
	}
}

void EKF2::PublishWindEstimate(const hrt_abstime &timestamp)
{
	if (_ekf.get_wind_status()) {
		// Publish wind estimate only if ekf declares them valid
		wind_s wind{};
		wind.timestamp_sample = _ekf.time_delayed_us();

		const Vector2f wind_vel = _ekf.getWindVelocity();
		const Vector2f wind_vel_var = _ekf.getWindVelocityVariance();

#if defined(CONFIG_EKF2_AIRSPEED)
		_ekf.getAirspeedInnov(wind.tas_innov);
		_ekf.getAirspeedInnovVar(wind.tas_innov_var);
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_SIDESLIP)
		_ekf.getBetaInnov(wind.beta_innov);
		_ekf.getBetaInnovVar(wind.beta_innov_var);
#endif // CONFIG_EKF2_SIDESLIP

		wind.windspeed_north = wind_vel(0);
		wind.windspeed_east = wind_vel(1);
		wind.variance_north = wind_vel_var(0);
		wind.variance_east = wind_vel_var(1);
		wind.timestamp = _replay_mode ? timestamp : hrt_absolute_time();

		_wind_pub.publish(wind);
	}
}

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
void EKF2::PublishOpticalFlowVel(const hrt_abstime &timestamp)
{
	const hrt_abstime timestamp_sample = _ekf.aid_src_optical_flow().timestamp_sample;

	if ((timestamp_sample != 0) && (timestamp_sample > _status_optical_flow_pub_last)) {

		vehicle_optical_flow_vel_s flow_vel{};
		flow_vel.timestamp_sample = _ekf.aid_src_optical_flow().timestamp_sample;

		_ekf.getFlowVelBody().copyTo(flow_vel.vel_body);
		_ekf.getFlowVelNE().copyTo(flow_vel.vel_ne);

		_ekf.getFlowUncompensated().copyTo(flow_vel.flow_uncompensated_integral);
		_ekf.getFlowCompensated().copyTo(flow_vel.flow_compensated_integral);

		_ekf.getFlowGyro().copyTo(flow_vel.gyro_rate);
		_ekf.getFlowGyroIntegral().copyTo(flow_vel.gyro_rate_integral);

		flow_vel.timestamp = _replay_mode ? timestamp : hrt_absolute_time();

		_estimator_optical_flow_vel_pub.publish(flow_vel);
	}
}
#endif // CONFIG_EKF2_OPTICAL_FLOW

float EKF2::filter_altitude_ellipsoid(float amsl_hgt)
{
	float height_diff = static_cast<float>(_gps_alttitude_ellipsoid) * 1e-3f - amsl_hgt;

	if (_gps_alttitude_ellipsoid_previous_timestamp == 0) {

		_wgs84_hgt_offset = height_diff;
		_gps_alttitude_ellipsoid_previous_timestamp = _gps_time_usec;

	} else if (_gps_time_usec != _gps_alttitude_ellipsoid_previous_timestamp) {

		// apply a 10 second first order low pass filter to baro offset
		float dt = 1e-6f * (_gps_time_usec - _gps_alttitude_ellipsoid_previous_timestamp);
		_gps_alttitude_ellipsoid_previous_timestamp = _gps_time_usec;
		float offset_rate_correction = 0.1f * (height_diff - _wgs84_hgt_offset);
		_wgs84_hgt_offset += dt * constrain(offset_rate_correction, -0.1f, 0.1f);
	}

	return amsl_hgt + _wgs84_hgt_offset;
}

#if defined(CONFIG_EKF2_AIRSPEED)
void EKF2::UpdateAirspeedSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF airspeed sample
	// prefer ORB_ID(airspeed_validated) if available, otherwise fallback to raw airspeed ORB_ID(airspeed)
	if (_airspeed_validated_sub.updated()) {
		const unsigned last_generation = _airspeed_validated_sub.get_last_generation();
		airspeed_validated_s airspeed_validated;

		if (_airspeed_validated_sub.update(&airspeed_validated)) {
			if (_msg_missed_airspeed_validated_perf == nullptr) {
				_msg_missed_airspeed_validated_perf = perf_alloc(PC_COUNT, MODULE_NAME": airspeed validated messages missed");

			} else if (_airspeed_validated_sub.get_last_generation() != last_generation + 1) {
				perf_count(_msg_missed_airspeed_validated_perf);
			}

			if (PX4_ISFINITE(airspeed_validated.true_airspeed_m_s)
			    && PX4_ISFINITE(airspeed_validated.calibrated_airspeed_m_s)
			    && (airspeed_validated.calibrated_airspeed_m_s > 0.f)
			    && (airspeed_validated.selected_airspeed_index > 0)
			   ) {
				airspeedSample airspeed_sample {
					.time_us = airspeed_validated.timestamp,
					.true_airspeed = airspeed_validated.true_airspeed_m_s,
					.eas2tas = airspeed_validated.true_airspeed_m_s / airspeed_validated.calibrated_airspeed_m_s,
				};
				_ekf.setAirspeedData(airspeed_sample);
			}

			_airspeed_validated_timestamp_last = airspeed_validated.timestamp;
		}

	} else if (((ekf2_timestamps.timestamp - _airspeed_validated_timestamp_last) > 3_s) && _airspeed_sub.updated()) {
		// use ORB_ID(airspeed) if ORB_ID(airspeed_validated) is unavailable
		const unsigned last_generation = _airspeed_sub.get_last_generation();
		airspeed_s airspeed;

		if (_airspeed_sub.update(&airspeed)) {
			if (_msg_missed_airspeed_perf == nullptr) {
				_msg_missed_airspeed_perf = perf_alloc(PC_COUNT, MODULE_NAME": airspeed messages missed");

			} else if (_airspeed_sub.get_last_generation() != last_generation + 1) {
				perf_count(_msg_missed_airspeed_perf);
			}

			// The airspeed measurement received via ORB_ID(airspeed) topic has not been corrected
			// for scale factor errors and requires the ASPD_SCALE correction to be applied.
			const float true_airspeed_m_s = airspeed.true_airspeed_m_s * _airspeed_scale_factor;

			if (PX4_ISFINITE(airspeed.true_airspeed_m_s)
			    && PX4_ISFINITE(airspeed.indicated_airspeed_m_s)
			    && (airspeed.indicated_airspeed_m_s > 0.f)
			   ) {
				airspeedSample airspeed_sample {
					.time_us = airspeed.timestamp_sample,
					.true_airspeed = true_airspeed_m_s,
					.eas2tas = airspeed.true_airspeed_m_s / airspeed.indicated_airspeed_m_s,
				};
				_ekf.setAirspeedData(airspeed_sample);
			}

			ekf2_timestamps.airspeed_timestamp_rel = (int16_t)((int64_t)airspeed.timestamp / 100 -
					(int64_t)ekf2_timestamps.timestamp / 100);
		}
	}
}
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_AUXVEL)
void EKF2::UpdateAuxVelSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF auxiliary velocity sample
	//  - use the landing target pose estimate as another source of velocity data
	const unsigned last_generation = _landing_target_pose_sub.get_last_generation();
	landing_target_pose_s landing_target_pose;

	if (_landing_target_pose_sub.update(&landing_target_pose)) {
		if (_msg_missed_landing_target_pose_perf == nullptr) {
			_msg_missed_landing_target_pose_perf = perf_alloc(PC_COUNT, MODULE_NAME": landing_target_pose messages missed");

		} else if (_landing_target_pose_sub.get_last_generation() != last_generation + 1) {
			perf_count(_msg_missed_landing_target_pose_perf);
		}

		// we can only use the landing target if it has a fixed position and  a valid velocity estimate
		if (landing_target_pose.is_static && landing_target_pose.rel_vel_valid) {
			// velocity of vehicle relative to target has opposite sign to target relative to vehicle
			auxVelSample auxvel_sample{
				.time_us = landing_target_pose.timestamp,
				.vel = Vector2f{-landing_target_pose.vx_rel, -landing_target_pose.vy_rel},
				.velVar = Vector2f{landing_target_pose.cov_vx_rel, landing_target_pose.cov_vy_rel},
			};
			_ekf.setAuxVelData(auxvel_sample);
		}
	}
}
#endif // CONFIG_EKF2_AUXVEL

void EKF2::UpdateBaroSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF baro sample
	const unsigned last_generation = _airdata_sub.get_last_generation();
	vehicle_air_data_s airdata;

	if (_airdata_sub.update(&airdata)) {
		if (_msg_missed_air_data_perf == nullptr) {
			_msg_missed_air_data_perf = perf_alloc(PC_COUNT, MODULE_NAME": vehicle_air_data messages missed");

		} else if (_airdata_sub.get_last_generation() != last_generation + 1) {
			perf_count(_msg_missed_air_data_perf);
		}

		bool reset = false;

		// check if barometer has changed
		if (airdata.baro_device_id != _device_id_baro) {
			if (_device_id_baro != 0) {
				PX4_DEBUG("%d - baro sensor ID changed %" PRIu32 " -> %" PRIu32, _instance, _device_id_baro, airdata.baro_device_id);
			}

			reset = true;

		} else if (airdata.calibration_count != _baro_calibration_count) {
			// existing calibration has changed, reset saved baro bias
			PX4_DEBUG("%d - baro %" PRIu32 " calibration updated, resetting bias", _instance, _device_id_baro);
			reset = true;
		}

		if (reset) {
			_device_id_baro = airdata.baro_device_id;
			_baro_calibration_count = airdata.calibration_count;
		}

		_ekf.set_air_density(airdata.rho);

		_ekf.setBaroData(baroSample{airdata.timestamp_sample, airdata.baro_alt_meter, reset});

		ekf2_timestamps.vehicle_air_data_timestamp_rel = (int16_t)((int64_t)airdata.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}
}

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
bool EKF2::UpdateExtVisionSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF external vision sample
	bool new_ev_odom = false;
	const unsigned last_generation = _ev_odom_sub.get_last_generation();

	vehicle_odometry_s ev_odom;

	if (_ev_odom_sub.update(&ev_odom)) {
		if (_msg_missed_odometry_perf == nullptr) {
			_msg_missed_odometry_perf = perf_alloc(PC_COUNT, MODULE_NAME": vehicle_visual_odometry messages missed");

		} else if (_ev_odom_sub.get_last_generation() != last_generation + 1) {
			perf_count(_msg_missed_odometry_perf);
		}

		extVisionSample ev_data{};
		ev_data.pos.setNaN();
		ev_data.vel.setNaN();
		ev_data.quat.setNaN();

		// check for valid velocity data
		const Vector3f ev_odom_vel(ev_odom.velocity);
		const Vector3f ev_odom_vel_var(ev_odom.velocity_variance);

		if (ev_odom_vel.isAllFinite()) {
			bool velocity_frame_valid = false;

			switch (ev_odom.velocity_frame) {
			case vehicle_odometry_s::VELOCITY_FRAME_NED:
				ev_data.vel_frame = VelocityFrame::LOCAL_FRAME_NED;
				velocity_frame_valid = true;
				break;

			case vehicle_odometry_s::VELOCITY_FRAME_FRD:
				ev_data.vel_frame = VelocityFrame::LOCAL_FRAME_FRD;
				velocity_frame_valid = true;
				break;

			case vehicle_odometry_s::VELOCITY_FRAME_BODY_FRD:
				ev_data.vel_frame = VelocityFrame::BODY_FRAME_FRD;
				velocity_frame_valid = true;
				break;
			}

			if (velocity_frame_valid) {
				ev_data.vel = ev_odom_vel;

				const float evv_noise_var = sq(_param_ekf2_evv_noise.get());

				// velocity measurement error from ev_data or parameters
				if ((_param_ekf2_ev_noise_md.get() == 0) && ev_odom_vel_var.isAllFinite()) {

					ev_data.velocity_var(0) = fmaxf(evv_noise_var, ev_odom_vel_var(0));
					ev_data.velocity_var(1) = fmaxf(evv_noise_var, ev_odom_vel_var(1));
					ev_data.velocity_var(2) = fmaxf(evv_noise_var, ev_odom_vel_var(2));

				} else {
					ev_data.velocity_var.setAll(evv_noise_var);
				}

				new_ev_odom = true;
			}
		}

		// check for valid position data
		const Vector3f ev_odom_pos(ev_odom.position);
		const Vector3f ev_odom_pos_var(ev_odom.position_variance);

		if (ev_odom_pos.isAllFinite()) {
			bool position_frame_valid = false;

			switch (ev_odom.pose_frame) {
			case vehicle_odometry_s::POSE_FRAME_NED:
				ev_data.pos_frame = PositionFrame::LOCAL_FRAME_NED;
				position_frame_valid = true;
				break;

			case vehicle_odometry_s::POSE_FRAME_FRD:
				ev_data.pos_frame = PositionFrame::LOCAL_FRAME_FRD;
				position_frame_valid = true;
				break;
			}

			if (position_frame_valid) {
				ev_data.pos = ev_odom_pos;

				const float evp_noise_var = sq(_param_ekf2_evp_noise.get());

				// position measurement error from ev_data or parameters
				if ((_param_ekf2_ev_noise_md.get() == 0) && ev_odom_pos_var.isAllFinite()) {

					ev_data.position_var(0) = fmaxf(evp_noise_var, ev_odom_pos_var(0));
					ev_data.position_var(1) = fmaxf(evp_noise_var, ev_odom_pos_var(1));
					ev_data.position_var(2) = fmaxf(evp_noise_var, ev_odom_pos_var(2));

				} else {
					ev_data.position_var.setAll(evp_noise_var);
				}

				new_ev_odom = true;
			}
		}

		// check for valid orientation data
		const Quatf ev_odom_q(ev_odom.q);
		const Vector3f ev_odom_q_var(ev_odom.orientation_variance);
		const bool non_zero = (fabsf(ev_odom_q(0)) > 0.f) || (fabsf(ev_odom_q(1)) > 0.f)
				      || (fabsf(ev_odom_q(2)) > 0.f) || (fabsf(ev_odom_q(3)) > 0.f);
		const float eps = 1e-5f;
		const bool no_element_larger_than_one = (fabsf(ev_odom_q(0)) <= 1.f + eps)
							&& (fabsf(ev_odom_q(1)) <= 1.f + eps)
							&& (fabsf(ev_odom_q(2)) <= 1.f + eps)
							&& (fabsf(ev_odom_q(3)) <= 1.f + eps);
		const bool norm_in_tolerance = fabsf(1.f - ev_odom_q.norm()) <= eps;

		const bool orientation_valid = ev_odom_q.isAllFinite() && non_zero && no_element_larger_than_one && norm_in_tolerance;

		if (orientation_valid) {
			ev_data.quat = ev_odom_q;
			ev_data.quat.normalize();

			// orientation measurement error from ev_data or parameters
			const float eva_noise_var = sq(_param_ekf2_eva_noise.get());

			if ((_param_ekf2_ev_noise_md.get() == 0) && ev_odom_q_var.isAllFinite()) {

				ev_data.orientation_var(0) = fmaxf(eva_noise_var, ev_odom_q_var(0));
				ev_data.orientation_var(1) = fmaxf(eva_noise_var, ev_odom_q_var(1));
				ev_data.orientation_var(2) = fmaxf(eva_noise_var, ev_odom_q_var(2));

			} else {
				ev_data.orientation_var.setAll(eva_noise_var);
			}

			new_ev_odom = true;
		}

		// use timestamp from external computer, clocks are synchronized when using MAVROS
		ev_data.time_us = ev_odom.timestamp_sample;
		ev_data.reset_counter = ev_odom.reset_counter;
		ev_data.quality = ev_odom.quality;

		if (new_ev_odom)  {
			_ekf.setExtVisionData(ev_data);
		}

		ekf2_timestamps.visual_odometry_timestamp_rel = (int16_t)((int64_t)ev_odom.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}

	return new_ev_odom;
}
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
bool EKF2::UpdateFlowSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF flow sample
	bool new_optical_flow = false;
	const unsigned last_generation = _vehicle_optical_flow_sub.get_last_generation();
	vehicle_optical_flow_s optical_flow;

	if (_vehicle_optical_flow_sub.update(&optical_flow)) {
		if (_msg_missed_optical_flow_perf == nullptr) {
			_msg_missed_optical_flow_perf = perf_alloc(PC_COUNT, MODULE_NAME": optical_flow messages missed");

		} else if (_vehicle_optical_flow_sub.get_last_generation() != last_generation + 1) {
			perf_count(_msg_missed_optical_flow_perf);
		}

		flowSample flow {
			.time_us = optical_flow.timestamp_sample,
			// NOTE: the EKF uses the reverse sign convention to the flow sensor. EKF assumes positive LOS rate
			// is produced by a RH rotation of the image about the sensor axis.
			.flow_xy_rad = Vector2f{-optical_flow.pixel_flow[0], -optical_flow.pixel_flow[1]},
			.gyro_xyz = Vector3f{-optical_flow.delta_angle[0], -optical_flow.delta_angle[1], -optical_flow.delta_angle[2]},
			.dt = 1e-6f * (float)optical_flow.integration_timespan_us,
			.quality = optical_flow.quality,
		};

		if (Vector2f(optical_flow.pixel_flow).isAllFinite() && flow.dt < 1) {

			// Save sensor limits reported by the optical flow sensor
			_ekf.set_optical_flow_limits(optical_flow.max_flow_rate, optical_flow.min_ground_distance,
						     optical_flow.max_ground_distance);

			_ekf.setOpticalFlowData(flow);

			new_optical_flow = true;
		}

		// use optical_flow distance as range sample if distance_sensor unavailable
		if (PX4_ISFINITE(optical_flow.distance_m) && (ekf2_timestamps.timestamp > _last_range_sensor_update + 1_s)) {

			int8_t quality = static_cast<float>(optical_flow.quality) / static_cast<float>(UINT8_MAX) * 100.f;

			rangeSample range_sample {
				.time_us = optical_flow.timestamp_sample,
				.rng = optical_flow.distance_m,
				.quality = quality,
			};
			_ekf.setRangeData(range_sample);

			// set sensor limits
			_ekf.set_rangefinder_limits(optical_flow.min_ground_distance, optical_flow.max_ground_distance);
		}

		ekf2_timestamps.optical_flow_timestamp_rel = (int16_t)((int64_t)optical_flow.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}

	return new_optical_flow;
}
#endif // CONFIG_EKF2_OPTICAL_FLOW

void EKF2::UpdateGpsSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF GPS message
	const unsigned last_generation = _vehicle_gps_position_sub.get_last_generation();
	sensor_gps_s vehicle_gps_position;

	if (_vehicle_gps_position_sub.update(&vehicle_gps_position)) {
		if (_msg_missed_gps_perf == nullptr) {
			_msg_missed_gps_perf = perf_alloc(PC_COUNT, MODULE_NAME": vehicle_gps_position messages missed");

		} else if (_vehicle_gps_position_sub.get_last_generation() != last_generation + 1) {
			perf_count(_msg_missed_gps_perf);
		}

		gpsMessage gps_msg{
			.time_usec = vehicle_gps_position.timestamp,
			.lat = vehicle_gps_position.lat,
			.lon = vehicle_gps_position.lon,
			.alt = vehicle_gps_position.alt,
			.yaw = vehicle_gps_position.heading,
			.yaw_offset = vehicle_gps_position.heading_offset,
			.yaw_accuracy = vehicle_gps_position.heading_accuracy,
			.fix_type = vehicle_gps_position.fix_type,
			.eph = vehicle_gps_position.eph,
			.epv = vehicle_gps_position.epv,
			.sacc = vehicle_gps_position.s_variance_m_s,
			.vel_m_s = vehicle_gps_position.vel_m_s,
			.vel_ned = Vector3f{
				vehicle_gps_position.vel_n_m_s,
				vehicle_gps_position.vel_e_m_s,
				vehicle_gps_position.vel_d_m_s
			},
			.vel_ned_valid = vehicle_gps_position.vel_ned_valid,
			.nsats = vehicle_gps_position.satellites_used,
			.pdop = sqrtf(vehicle_gps_position.hdop *vehicle_gps_position.hdop
				      + vehicle_gps_position.vdop * vehicle_gps_position.vdop),
		};
		_ekf.setGpsData(gps_msg);

		_gps_time_usec = gps_msg.time_usec;
		_gps_alttitude_ellipsoid = vehicle_gps_position.alt_ellipsoid;
	}
}

void EKF2::UpdateMagSample(ekf2_timestamps_s &ekf2_timestamps)
{
	const unsigned last_generation = _magnetometer_sub.get_last_generation();
	vehicle_magnetometer_s magnetometer;

	if (_magnetometer_sub.update(&magnetometer)) {
		if (_msg_missed_magnetometer_perf == nullptr) {
			_msg_missed_magnetometer_perf = perf_alloc(PC_COUNT, MODULE_NAME": vehicle_magnetometer messages missed");

		} else if (_magnetometer_sub.get_last_generation() != last_generation + 1) {
			perf_count(_msg_missed_magnetometer_perf);
		}

		bool reset = false;

		// check if magnetometer has changed
		if (magnetometer.device_id != _device_id_mag) {
			if (_device_id_mag != 0) {
				PX4_DEBUG("%d - mag sensor ID changed %" PRIu32 " -> %" PRIu32, _instance, _device_id_mag, magnetometer.device_id);
			}

			reset = true;

		} else if (magnetometer.calibration_count != _mag_calibration_count) {
			// existing calibration has changed, reset saved mag bias
			PX4_DEBUG("%d - mag %" PRIu32 " calibration updated, resetting bias", _instance, _device_id_mag);
			reset = true;
		}

		if (reset) {
			_device_id_mag = magnetometer.device_id;
			_mag_calibration_count = magnetometer.calibration_count;

			// reset magnetometer bias learning
			_mag_cal = {};
		}

		_ekf.setMagData(magSample{magnetometer.timestamp_sample, Vector3f{magnetometer.magnetometer_ga}, reset});

		ekf2_timestamps.vehicle_magnetometer_timestamp_rel = (int16_t)((int64_t)magnetometer.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}
}

#if defined(CONFIG_EKF2_RANGE_FINDER)
void EKF2::UpdateRangeSample(ekf2_timestamps_s &ekf2_timestamps)
{
	distance_sensor_s distance_sensor;

	if (_distance_sensor_selected < 0) {

		// only consider distance sensors that have updated within the last 0.1s
		const hrt_abstime timestamp_stale = math::max(ekf2_timestamps.timestamp, 100_ms) - 100_ms;

		if (_distance_sensor_subs.advertised()) {
			for (unsigned i = 0; i < _distance_sensor_subs.size(); i++) {

				if (_distance_sensor_subs[i].update(&distance_sensor)) {
					// only use the first instace which has the correct orientation
					if ((distance_sensor.timestamp != 0) && (distance_sensor.timestamp > timestamp_stale)
					    && (distance_sensor.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING)) {

						int ndist = orb_group_count(ORB_ID(distance_sensor));

						if (ndist > 1) {
							PX4_INFO("%d - selected distance_sensor:%d (%d advertised)", _instance, i, ndist);
						}

						_distance_sensor_selected = i;
						_last_range_sensor_update = distance_sensor.timestamp;
						_distance_sensor_last_generation = _distance_sensor_subs[_distance_sensor_selected].get_last_generation() - 1;
						break;
					}
				}
			}
		}
	}

	if (_distance_sensor_selected >= 0 && _distance_sensor_subs[_distance_sensor_selected].update(&distance_sensor)) {
		// EKF range sample

		if (_msg_missed_distance_sensor_perf == nullptr) {
			_msg_missed_distance_sensor_perf = perf_alloc(PC_COUNT, MODULE_NAME": distance_sensor messages missed");

		} else if (_distance_sensor_subs[_distance_sensor_selected].get_last_generation() != _distance_sensor_last_generation +
			   1) {
			perf_count(_msg_missed_distance_sensor_perf);
		}

		_distance_sensor_last_generation = _distance_sensor_subs[_distance_sensor_selected].get_last_generation();

		if (distance_sensor.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING) {
			rangeSample range_sample {
				.time_us = distance_sensor.timestamp,
				.rng = distance_sensor.current_distance,
				.quality = distance_sensor.signal_quality,
			};
			_ekf.setRangeData(range_sample);

			// Save sensor limits reported by the rangefinder
			_ekf.set_rangefinder_limits(distance_sensor.min_distance, distance_sensor.max_distance);

			_last_range_sensor_update = ekf2_timestamps.timestamp;
		}

		ekf2_timestamps.distance_sensor_timestamp_rel = (int16_t)((int64_t)distance_sensor.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}

	if (_last_range_sensor_update < ekf2_timestamps.timestamp - 1_s) {
		// force reselection after timeout
		_distance_sensor_selected = -1;
	}
}
#endif // CONFIG_EKF2_RANGE_FINDER

void EKF2::UpdateSystemFlagsSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF system flags
	if (_status_sub.updated() || _vehicle_land_detected_sub.updated()) {

		systemFlagUpdate flags{};
		flags.time_us = ekf2_timestamps.timestamp;

		// vehicle_status
		vehicle_status_s vehicle_status;

		if (_status_sub.copy(&vehicle_status)
		    && (ekf2_timestamps.timestamp < vehicle_status.timestamp + 3_s)) {

			// initially set in_air from arming_state (will be overridden if land detector is available)
			flags.in_air = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

			// let the EKF know if the vehicle motion is that of a fixed wing (forward flight only relative to wind)
			flags.is_fixed_wing = (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING);

#if defined(CONFIG_EKF2_SIDESLIP)

			if (vehicle_status.is_vtol_tailsitter && _params->beta_fusion_enabled) {
				PX4_WARN("Disable EKF beta fusion as unsupported for tailsitter");
				_param_ekf2_fuse_beta.set(0);
				_param_ekf2_fuse_beta.commit_no_notification();
			}

#endif // CONFIG_EKF2_SIDESLIP
		}

		// vehicle_land_detected
		vehicle_land_detected_s vehicle_land_detected;

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)
		    && (ekf2_timestamps.timestamp < vehicle_land_detected.timestamp + 3_s)) {

			flags.at_rest = vehicle_land_detected.at_rest;
			flags.in_air = !vehicle_land_detected.landed;
			flags.gnd_effect = vehicle_land_detected.in_ground_effect;
		}

		_ekf.setSystemFlagData(flags);
	}
}

void EKF2::UpdateCalibration(const hrt_abstime &timestamp, InFlightCalibration &cal, const matrix::Vector3f &bias,
			     const matrix::Vector3f &bias_variance, float bias_limit, bool bias_valid, bool learning_valid)
{
	// reset existing cal on takeoff
	if (!_ekf.control_status_prev_flags().in_air && _ekf.control_status_flags().in_air) {
		cal = {};
	}

	// Check if conditions are OK for learning of accelerometer bias values
	// the EKF is operating in the correct mode and there are no filter faults
	static constexpr float max_var_allowed = 1e-3f;
	static constexpr float max_var_ratio = 1e2f;

	const bool valid = bias_valid
			   && (bias_variance.max() < max_var_allowed)
			   && (bias_variance.max() < max_var_ratio * bias_variance.min());

	if (valid && learning_valid) {
		// consider bias estimates stable when all checks pass consistently and bias hasn't changed more than 10% of the limit
		const float bias_change_limit = 0.1f * bias_limit;

		if (!(cal.bias - bias).longerThan(bias_change_limit)) {
			if (cal.last_us != 0) {
				cal.total_time_us += timestamp - cal.last_us;
			}

			if (cal.total_time_us > 30_s) {
				cal.cal_available = true;
			}

		} else {
			cal.total_time_us = 0;
			cal.bias = bias;
			cal.cal_available = false;
		}

		cal.last_us = timestamp;

	} else {
		// conditions are NOT OK for learning bias, reset timestamp
		// but keep the accumulated calibration time
		cal.last_us = 0;

		if (!valid && (cal.total_time_us != 0)) {
			// if a filter fault has occurred, assume previous learning was invalid and do not
			// count it towards total learning time.
			cal = {};
		}
	}
}

void EKF2::UpdateAccelCalibration(const hrt_abstime &timestamp)
{
	// the EKF is operating in the correct mode and there are no filter faults
	const bool bias_valid = (_param_ekf2_imu_ctrl.get() & static_cast<int32_t>(ImuCtrl::AccelBias))
				&& _ekf.control_status_flags().tilt_align
				&& (_ekf.fault_status().value == 0)
				&& !_ekf.fault_status_flags().bad_acc_bias
				&& !_ekf.fault_status_flags().bad_acc_clipping
				&& !_ekf.fault_status_flags().bad_acc_vertical
				&& !_ekf.warning_event_flags().invalid_accel_bias_cov_reset;

	const bool learning_valid = bias_valid && !_ekf.accel_bias_inhibited();

	UpdateCalibration(timestamp, _accel_cal, _ekf.getAccelBias(), _ekf.getAccelBiasVariance(), _ekf.getAccelBiasLimit(),
			  bias_valid, learning_valid);
}

void EKF2::UpdateGyroCalibration(const hrt_abstime &timestamp)
{
	// the EKF is operating in the correct mode and there are no filter faults
	const bool bias_valid = (_param_ekf2_imu_ctrl.get() & static_cast<int32_t>(ImuCtrl::GyroBias))
				&& _ekf.control_status_flags().tilt_align
				&& (_ekf.fault_status().value == 0);

	const bool learning_valid = bias_valid && !_ekf.gyro_bias_inhibited();

	UpdateCalibration(timestamp, _gyro_cal, _ekf.getGyroBias(), _ekf.getGyroBiasVariance(), _ekf.getGyroBiasLimit(),
			  bias_valid, learning_valid);
}

void EKF2::UpdateMagCalibration(const hrt_abstime &timestamp)
{
	const bool bias_valid = (_ekf.control_status_flags().mag_hdg || _ekf.control_status_flags().mag_3D)
				&& !_ekf.control_status_flags().mag_fault
				&& !_ekf.control_status_flags().mag_field_disturbed;

	const bool learning_valid = bias_valid && _ekf.control_status_flags().mag_3D;

	UpdateCalibration(timestamp, _mag_cal, _ekf.getMagBias(), _ekf.getMagBiasVariance(), _ekf.getMagBiasLimit(),
			  bias_valid, learning_valid);

	// update stored declination value
	if (!_mag_decl_saved) {
		float declination_deg;

		if (_ekf.get_mag_decl_deg(&declination_deg)) {
			_param_ekf2_mag_decl.update();

			if (PX4_ISFINITE(declination_deg) && (fabsf(declination_deg - _param_ekf2_mag_decl.get()) > 0.1f)) {
				_param_ekf2_mag_decl.set(declination_deg);
				_param_ekf2_mag_decl.commit_no_notification();
			}

			_mag_decl_saved = true;
		}
	}
}

int EKF2::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int EKF2::task_spawn(int argc, char *argv[])
{
	bool success = false;
	bool replay_mode = false;

	if (argc > 1 && !strcmp(argv[1], "-r")) {
		PX4_INFO("replay mode enabled");
		replay_mode = true;
	}

#if defined(CONFIG_EKF2_MULTI_INSTANCE)
	bool multi_mode = false;
	int32_t imu_instances = 0;
	int32_t mag_instances = 0;

	int32_t sens_imu_mode = 1;
	param_get(param_find("SENS_IMU_MODE"), &sens_imu_mode);

	if (sens_imu_mode == 0) {
		// ekf selector requires SENS_IMU_MODE = 0
		multi_mode = true;

		// IMUs (1 - 4 supported)
		param_get(param_find("EKF2_MULTI_IMU"), &imu_instances);

		if (imu_instances < 1 || imu_instances > 4) {
			const int32_t imu_instances_limited = math::constrain(imu_instances, static_cast<int32_t>(1), static_cast<int32_t>(4));
			PX4_WARN("EKF2_MULTI_IMU limited %" PRId32 " -> %" PRId32, imu_instances, imu_instances_limited);
			param_set_no_notification(param_find("EKF2_MULTI_IMU"), &imu_instances_limited);
			imu_instances = imu_instances_limited;
		}

		int32_t sens_mag_mode = 1;
		const param_t param_sens_mag_mode = param_find("SENS_MAG_MODE");
		param_get(param_sens_mag_mode, &sens_mag_mode);

		if (sens_mag_mode == 0) {
			const param_t param_ekf2_mult_mag = param_find("EKF2_MULTI_MAG");
			param_get(param_ekf2_mult_mag, &mag_instances);

			// Mags (1 - 4 supported)
			if (mag_instances > 4) {
				const int32_t mag_instances_limited = math::constrain(mag_instances, static_cast<int32_t>(1), static_cast<int32_t>(4));
				PX4_WARN("EKF2_MULTI_MAG limited %" PRId32 " -> %" PRId32, mag_instances, mag_instances_limited);
				param_set_no_notification(param_ekf2_mult_mag, &mag_instances_limited);
				mag_instances = mag_instances_limited;

			} else if (mag_instances <= 1) {
				// properly disable multi-magnetometer at sensors hub level
				PX4_WARN("EKF2_MULTI_MAG disabled, resetting SENS_MAG_MODE");

				// re-enable at sensors level
				sens_mag_mode = 1;
				param_set(param_sens_mag_mode, &sens_mag_mode);

				mag_instances = 1;
			}

		} else {
			mag_instances = 1;
		}
	}

	if (multi_mode && !replay_mode) {
		// Start EKF2Selector if it's not already running
		if (_ekf2_selector.load() == nullptr) {
			EKF2Selector *inst = new EKF2Selector();

			if (inst) {
				_ekf2_selector.store(inst);

			} else {
				PX4_ERR("Failed to create EKF2 selector");
				return PX4_ERROR;
			}
		}

		const hrt_abstime time_started = hrt_absolute_time();
		const int multi_instances = math::min(imu_instances * mag_instances, static_cast<int32_t>(EKF2_MAX_INSTANCES));
		int multi_instances_allocated = 0;

		// allocate EKF2 instances until all found or arming
		uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};

		bool ekf2_instance_created[MAX_NUM_IMUS][MAX_NUM_MAGS] {}; // IMUs * mags

		while ((multi_instances_allocated < multi_instances)
		       && (vehicle_status_sub.get().arming_state != vehicle_status_s::ARMING_STATE_ARMED)
		       && ((hrt_elapsed_time(&time_started) < 30_s)
			   || (vehicle_status_sub.get().hil_state == vehicle_status_s::HIL_STATE_ON))) {

			vehicle_status_sub.update();

			for (uint8_t mag = 0; mag < mag_instances; mag++) {
				uORB::SubscriptionData<vehicle_magnetometer_s> vehicle_mag_sub{ORB_ID(vehicle_magnetometer), mag};

				for (uint8_t imu = 0; imu < imu_instances; imu++) {

					uORB::SubscriptionData<vehicle_imu_s> vehicle_imu_sub{ORB_ID(vehicle_imu), imu};
					vehicle_mag_sub.update();

					// Mag & IMU data must be valid, first mag can be ignored initially
					if ((vehicle_mag_sub.advertised() || mag == 0) && (vehicle_imu_sub.advertised())) {

						if (!ekf2_instance_created[imu][mag]) {
							EKF2 *ekf2_inst = new EKF2(true, px4::ins_instance_to_wq(imu), false);

							if (ekf2_inst && ekf2_inst->multi_init(imu, mag)) {
								int actual_instance = ekf2_inst->instance(); // match uORB instance numbering

								if ((actual_instance >= 0) && (_objects[actual_instance].load() == nullptr)) {
									_objects[actual_instance].store(ekf2_inst);
									success = true;
									multi_instances_allocated++;
									ekf2_instance_created[imu][mag] = true;

									PX4_DEBUG("starting instance %d, IMU:%" PRIu8 " (%" PRIu32 "), MAG:%" PRIu8 " (%" PRIu32 ")", actual_instance,
										  imu, vehicle_imu_sub.get().accel_device_id,
										  mag, vehicle_mag_sub.get().device_id);

									_ekf2_selector.load()->ScheduleNow();

								} else {
									PX4_ERR("instance numbering problem instance: %d", actual_instance);
									delete ekf2_inst;
									break;
								}

							} else {
								PX4_ERR("alloc and init failed imu: %" PRIu8 " mag:%" PRIu8, imu, mag);
								px4_usleep(100000);
								break;
							}
						}

					} else {
						px4_usleep(1000); // give the sensors extra time to start
						break;
					}
				}
			}

			if (multi_instances_allocated < multi_instances) {
				px4_usleep(10000);
			}
		}

	} else

#endif // CONFIG_EKF2_MULTI_INSTANCE

	{
		// otherwise launch regular
		EKF2 *ekf2_inst = new EKF2(false, px4::wq_configurations::INS0, replay_mode);

		if (ekf2_inst) {
			_objects[0].store(ekf2_inst);
			ekf2_inst->ScheduleNow();
			success = true;
		}
	}

	return success ? PX4_OK : PX4_ERROR;
}

int EKF2::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Attitude and position estimator using an Extended Kalman Filter. It is used for Multirotors and Fixed-Wing.

The documentation can be found on the [ECL/EKF Overview & Tuning](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf.html) page.

ekf2 can be started in replay mode (`-r`): in this mode, it does not access the system time, but only uses the
timestamps from the sensor topics.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ekf2", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('r', "Enable replay mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
#if defined(CONFIG_EKF2_MULTI_INSTANCE)
	PRINT_MODULE_USAGE_COMMAND_DESCR("select_instance", "Request switch to new estimator instance");
	PRINT_MODULE_USAGE_ARG("<instance>", "Specify desired estimator instance", false);
#endif // CONFIG_EKF2_MULTI_INSTANCE
	return 0;
}

extern "C" __EXPORT int ekf2_main(int argc, char *argv[])
{
	if (argc <= 1 || strcmp(argv[1], "-h") == 0) {
		return EKF2::print_usage();
	}

	if (strcmp(argv[1], "start") == 0) {
		int ret = 0;
		EKF2::lock_module();

		ret = EKF2::task_spawn(argc - 1, argv + 1);

		if (ret < 0) {
			PX4_ERR("start failed (%i)", ret);
		}

		EKF2::unlock_module();
		return ret;

#if defined(CONFIG_EKF2_MULTI_INSTANCE)
	} else if (strcmp(argv[1], "select_instance") == 0) {

		if (EKF2::trylock_module()) {
			if (_ekf2_selector.load()) {
				if (argc > 2) {
					int instance = atoi(argv[2]);
					_ekf2_selector.load()->RequestInstance(instance);
				} else {
					EKF2::unlock_module();
					return EKF2::print_usage("instance required");
				}

			} else {
				PX4_ERR("multi-EKF not active, unable to select instance");
			}

			EKF2::unlock_module();

		} else {
			PX4_WARN("module locked, try again later");
		}

		return 0;
#endif // CONFIG_EKF2_MULTI_INSTANCE
	} else if (strcmp(argv[1], "status") == 0) {
		if (EKF2::trylock_module()) {
#if defined(CONFIG_EKF2_MULTI_INSTANCE)
			if (_ekf2_selector.load()) {
				_ekf2_selector.load()->PrintStatus();
			}
#endif // CONFIG_EKF2_MULTI_INSTANCE

			for (int i = 0; i < EKF2_MAX_INSTANCES; i++) {
				if (_objects[i].load()) {
					PX4_INFO_RAW("\n");
					_objects[i].load()->print_status();
				}
			}

			EKF2::unlock_module();

		} else {
			PX4_WARN("module locked, try again later");
		}

		return 0;

	} else if (strcmp(argv[1], "stop") == 0) {
		EKF2::lock_module();

		if (argc > 2) {
			int instance = atoi(argv[2]);

			if (instance >= 0 && instance < EKF2_MAX_INSTANCES) {
				PX4_INFO("stopping instance %d", instance);
				EKF2 *inst = _objects[instance].load();

				if (inst) {
					inst->request_stop();
					px4_usleep(20000); // 20 ms
					delete inst;
					_objects[instance].store(nullptr);
				}
			} else {
				PX4_ERR("invalid instance %d", instance);
			}

		} else {
			// otherwise stop everything
			bool was_running = false;

#if defined(CONFIG_EKF2_MULTI_INSTANCE)
			if (_ekf2_selector.load()) {
				PX4_INFO("stopping ekf2 selector");
				_ekf2_selector.load()->Stop();
				delete _ekf2_selector.load();
				_ekf2_selector.store(nullptr);
				was_running = true;
			}
#endif // CONFIG_EKF2_MULTI_INSTANCE

			for (int i = 0; i < EKF2_MAX_INSTANCES; i++) {
				EKF2 *inst = _objects[i].load();

				if (inst) {
					PX4_INFO("stopping ekf2 instance %d", i);
					was_running = true;
					inst->request_stop();
					px4_usleep(20000); // 20 ms
					delete inst;
					_objects[i].store(nullptr);
				}
			}

			if (!was_running) {
				PX4_WARN("not running");
			}
		}

		EKF2::unlock_module();
		return PX4_OK;
	}

	EKF2::lock_module(); // Lock here, as the method could access _object.
	int ret = EKF2::custom_command(argc - 1, argv + 1);
	EKF2::unlock_module();

	return ret;
}
/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file EKF2.cpp
 * Implementation of the attitude and position estimator.
 *
 * @author Roman Bapst
 */

#ifndef EKF2_HPP
#define EKF2_HPP

#include "EKF/ekf.h"
#include "Utility/PreFlightChecker.hpp"

#include "EKF2Selector.hpp"

#include <float.h>

#include <containers/LockGuard.hpp>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/ekf2_timestamps.h>
#include <uORB/topics/estimator_bias.h>
#include <uORB/topics/estimator_bias3d.h>
#include <uORB/topics/estimator_event_flags.h>
#include <uORB/topics/estimator_gps_status.h>
#include <uORB/topics/estimator_innovations.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/estimator_states.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind.h>
#include <uORB/topics/yaw_estimator_status.h>

#if defined(CONFIG_EKF2_AIRSPEED)
# include <uORB/topics/airspeed.h>
# include <uORB/topics/airspeed_validated.h>
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_AUXVEL)
# include <uORB/topics/landing_target_pose.h>
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
# include <uORB/topics/vehicle_optical_flow.h>
# include <uORB/topics/vehicle_optical_flow_vel.h>
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_RANGE_FINDER)
# include <uORB/topics/distance_sensor.h>
#endif // CONFIG_EKF2_RANGE_FINDER

extern pthread_mutex_t ekf2_module_mutex;

class EKF2 final : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	EKF2() = delete;
	EKF2(bool multi_mode, const px4::wq_config_t &config, bool replay_mode);
	~EKF2() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	int print_status();

	bool should_exit() const { return _task_should_exit.load(); }

	void request_stop() { _task_should_exit.store(true); }

	static void lock_module() { pthread_mutex_lock(&ekf2_module_mutex); }
	static bool trylock_module() { return (pthread_mutex_trylock(&ekf2_module_mutex) == 0); }
	static void unlock_module() { pthread_mutex_unlock(&ekf2_module_mutex); }

#if defined(CONFIG_EKF2_MULTI_INSTANCE)
	bool multi_init(int imu, int mag);
#endif // CONFIG_EKF2_MULTI_INSTANCE

	int instance() const { return _instance; }

private:
	//TODO: remove after 1.14 release
	enum SensorFusionMask : uint16_t {
		// Bit locations for fusion_mode
		DEPRECATED_USE_GPS = (1 << 0),  ///< set to true to use GPS data (DEPRECATED, use gnss_ctrl)
		DEPRECATED_USE_OPT_FLOW     = (1 << 1), ///< set to true to use optical flow data
		DEPRECATED_INHIBIT_ACC_BIAS = (1 << 2), ///< set to true to inhibit estimation of accelerometer delta velocity bias
		DEPRECATED_USE_EXT_VIS_POS = (1 << 3), ///< set to true to use external vision position data
		DEPRECATED_USE_EXT_VIS_YAW = (1 << 4), ///< set to true to use external vision quaternion data for yaw
		DEPRECATED_USE_DRAG         = (1 << 5), ///< set to true to use the multi-rotor drag model to estimate wind
		DEPRECATED_ROTATE_EXT_VIS  = (1 << 6), ///< set to true to if the EV observations are in a non NED reference frame and need to be rotated before being used
		DEPRECATED_USE_GPS_YAW     = (1 << 7), ///< set to true to use GPS yaw data if available (DEPRECATED, use gnss_ctrl)
		DEPRECATED_USE_EXT_VIS_VEL = (1 << 8), ///< set to true to use external vision velocity data
	};

	static constexpr uint8_t MAX_NUM_IMUS = 4;
	static constexpr uint8_t MAX_NUM_MAGS = 4;

	void Run() override;

	void VerifyParams();

	void PublishAidSourceStatus(const hrt_abstime &timestamp);
	void PublishAttitude(const hrt_abstime &timestamp);
	void PublishBaroBias(const hrt_abstime &timestamp);
	void PublishGnssHgtBias(const hrt_abstime &timestamp);

#if defined(CONFIG_EKF2_RANGE_FINDER)
	void PublishRngHgtBias(const hrt_abstime &timestamp);
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	void PublishEvPosBias(const hrt_abstime &timestamp);
#endif // CONFIG_EKF2_EXTERNAL_VISION
	estimator_bias_s fillEstimatorBiasMsg(const BiasEstimator::status &status, uint64_t timestamp_sample_us,
					      uint64_t timestamp, uint32_t device_id = 0);
	void PublishEventFlags(const hrt_abstime &timestamp);
	void PublishGlobalPosition(const hrt_abstime &timestamp);
	void PublishGpsStatus(const hrt_abstime &timestamp);
	void PublishInnovations(const hrt_abstime &timestamp);
	void PublishInnovationTestRatios(const hrt_abstime &timestamp);
	void PublishInnovationVariances(const hrt_abstime &timestamp);
	void PublishLocalPosition(const hrt_abstime &timestamp);
	void PublishOdometry(const hrt_abstime &timestamp, const imuSample &imu_sample);
	void PublishOdometryAligned(const hrt_abstime &timestamp, const vehicle_odometry_s &ev_odom);
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	void PublishOpticalFlowVel(const hrt_abstime &timestamp);
#endif // CONFIG_EKF2_OPTICAL_FLOW
	void PublishSensorBias(const hrt_abstime &timestamp);
	void PublishStates(const hrt_abstime &timestamp);
	void PublishStatus(const hrt_abstime &timestamp);
	void PublishStatusFlags(const hrt_abstime &timestamp);
	void PublishWindEstimate(const hrt_abstime &timestamp);
	void PublishYawEstimatorStatus(const hrt_abstime &timestamp);

#if defined(CONFIG_EKF2_AIRSPEED)
	void UpdateAirspeedSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_AUXVEL)
	void UpdateAuxVelSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_AUXVEL
	void UpdateBaroSample(ekf2_timestamps_s &ekf2_timestamps);
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	bool UpdateExtVisionSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_EXTERNAL_VISION
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	bool UpdateFlowSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_OPTICAL_FLOW
	void UpdateGpsSample(ekf2_timestamps_s &ekf2_timestamps);
	void UpdateMagSample(ekf2_timestamps_s &ekf2_timestamps);
#if defined(CONFIG_EKF2_RANGE_FINDER)
	void UpdateRangeSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_RANGE_FINDER
	void UpdateSystemFlagsSample(ekf2_timestamps_s &ekf2_timestamps);

	// Used to check, save and use learned accel/gyro/mag biases
	struct InFlightCalibration {
		hrt_abstime last_us{0};         ///< last time the EKF was operating a mode that estimates accelerometer biases (uSec)
		hrt_abstime total_time_us{0};   ///< accumulated calibration time since the last save
		matrix::Vector3f bias{};
		bool cal_available{false};      ///< true when an unsaved valid calibration for the XYZ accelerometer bias is available
	};

	void UpdateCalibration(const hrt_abstime &timestamp, InFlightCalibration &cal, const matrix::Vector3f &bias,
			       const matrix::Vector3f &bias_variance, float bias_limit, bool bias_valid, bool learning_valid);
	void UpdateAccelCalibration(const hrt_abstime &timestamp);
	void UpdateGyroCalibration(const hrt_abstime &timestamp);
	void UpdateMagCalibration(const hrt_abstime &timestamp);

	// publish helper for estimator_aid_source topics
	template <typename T>
	void PublishAidSourceStatus(const T &status, hrt_abstime &status_publish_last, uORB::PublicationMulti<T> &pub)
	{
		if (status.timestamp_sample > status_publish_last) {
			// publish if updated
			T status_out{status};
			status_out.estimator_instance = _instance;
			status_out.timestamp = hrt_absolute_time();
			pub.publish(status_out);

			// record timestamp sample
			status_publish_last = status.timestamp_sample;
		}
	}

	/*
	 * Calculate filtered WGS84 height from estimated AMSL height
	 */
	float filter_altitude_ellipsoid(float amsl_hgt);

	static constexpr float sq(float x) { return x * x; };

	const bool _replay_mode{false};			///< true when we use replay data from a log
	const bool _multi_mode;
	int _instance{0};

	px4::atomic_bool _task_should_exit{false};

	// time slip monitoring
	uint64_t _integrated_time_us = 0;	///< integral of gyro delta time from start (uSec)
	uint64_t _start_time_us = 0;		///< system time at EKF start (uSec)
	int64_t _last_time_slip_us = 0;		///< Last time slip (uSec)

	perf_counter_t _ecl_ekf_update_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": ECL update")};
	perf_counter_t _ecl_ekf_update_full_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": ECL full update")};
	perf_counter_t _msg_missed_imu_perf{perf_alloc(PC_COUNT, MODULE_NAME": IMU message missed")};
	perf_counter_t _msg_missed_air_data_perf{nullptr};
	perf_counter_t _msg_missed_gps_perf{nullptr};
	perf_counter_t _msg_missed_magnetometer_perf{nullptr};
	perf_counter_t _msg_missed_odometry_perf{nullptr};

	// Used to control saving of mag declination to be used on next startup
	bool _mag_decl_saved = false;	///< true when the magnetic declination has been saved

	InFlightCalibration _accel_cal{};
	InFlightCalibration _gyro_cal{};
	InFlightCalibration _mag_cal{};

	uint64_t _gps_time_usec{0};
	int32_t _gps_alttitude_ellipsoid{0};			///< altitude in 1E-3 meters (millimeters) above ellipsoid
	uint64_t _gps_alttitude_ellipsoid_previous_timestamp{0}; ///< storage for previous timestamp to compute dt
	float   _wgs84_hgt_offset = 0;  ///< height offset between AMSL and WGS84

	uint8_t _accel_calibration_count{0};
	uint8_t _baro_calibration_count{0};
	uint8_t _gyro_calibration_count{0};
	uint8_t _mag_calibration_count{0};

	uint32_t _device_id_accel{0};
	uint32_t _device_id_baro{0};
	uint32_t _device_id_gyro{0};
	uint32_t _device_id_mag{0};

	Vector3f _last_accel_bias_published{};
	Vector3f _last_gyro_bias_published{};
	Vector3f _last_mag_bias_published{};

	hrt_abstime _last_sensor_bias_published{0};
	hrt_abstime _last_gps_status_published{0};

	hrt_abstime _status_baro_hgt_pub_last{0};
	hrt_abstime _status_rng_hgt_pub_last{0};

	hrt_abstime _status_fake_hgt_pub_last{0};
	hrt_abstime _status_fake_pos_pub_last{0};

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_ev_hgt_pub {ORB_ID(estimator_aid_src_ev_hgt)};
	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_ev_pos_pub{ORB_ID(estimator_aid_src_ev_pos)};
	uORB::PublicationMulti<estimator_aid_source3d_s> _estimator_aid_src_ev_vel_pub{ORB_ID(estimator_aid_src_ev_vel)};
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_ev_yaw_pub{ORB_ID(estimator_aid_src_ev_yaw)};
	hrt_abstime _status_ev_hgt_pub_last{0};
	hrt_abstime _status_ev_pos_pub_last{0};
	hrt_abstime _status_ev_vel_pub_last{0};
	hrt_abstime _status_ev_yaw_pub_last{0};

	matrix::Vector3f _last_ev_bias_published{};

	uORB::Subscription _ev_odom_sub{ORB_ID(vehicle_visual_odometry)};

	uORB::PublicationMulti<estimator_bias3d_s> _estimator_ev_pos_bias_pub{ORB_ID(estimator_ev_pos_bias)};
#endif // CONFIG_EKF2_EXTERNAL_VISION

	hrt_abstime _status_gnss_hgt_pub_last{0};
	hrt_abstime _status_gnss_pos_pub_last{0};
	hrt_abstime _status_gnss_vel_pub_last{0};
#if defined(CONFIG_EKF2_GNSS_YAW)
	hrt_abstime _status_gnss_yaw_pub_last {0};
#endif // CONFIG_EKF2_GNSS_YAW

	hrt_abstime _status_mag_pub_last{0};
	hrt_abstime _status_mag_heading_pub_last{0};

	hrt_abstime _status_gravity_pub_last{0};

#if defined(CONFIG_EKF2_AUXVEL)
	uORB::Subscription _landing_target_pose_sub {ORB_ID(landing_target_pose)};

	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_aux_vel_pub{ORB_ID(estimator_aid_src_aux_vel)};
	hrt_abstime _status_aux_vel_pub_last{0};

	perf_counter_t _msg_missed_landing_target_pose_perf{nullptr};
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	uORB::Subscription _vehicle_optical_flow_sub {ORB_ID(vehicle_optical_flow)};
	uORB::PublicationMulti<vehicle_optical_flow_vel_s> _estimator_optical_flow_vel_pub{ORB_ID(estimator_optical_flow_vel)};
	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_optical_flow_pub{ORB_ID(estimator_aid_src_optical_flow)};
	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_terrain_optical_flow_pub{ORB_ID(estimator_aid_src_terrain_optical_flow)};

	hrt_abstime _status_optical_flow_pub_last{0};
	hrt_abstime _status_terrain_optical_flow_pub_last{0};
	perf_counter_t _msg_missed_optical_flow_perf{nullptr};
#endif // CONFIG_EKF2_OPTICAL_FLOW

	float _last_baro_bias_published{};
	float _last_gnss_hgt_bias_published{};
	float _last_rng_hgt_bias_published{};

#if defined(CONFIG_EKF2_AIRSPEED)
	uORB::Subscription _airspeed_sub {ORB_ID(airspeed)};
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};

	float _airspeed_scale_factor{1.0f}; ///< scale factor correction applied to airspeed measurements
	hrt_abstime _airspeed_validated_timestamp_last{0};

	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_airspeed_pub {ORB_ID(estimator_aid_src_airspeed)};
	hrt_abstime _status_airspeed_pub_last{0};

	perf_counter_t _msg_missed_airspeed_perf{nullptr};
	perf_counter_t _msg_missed_airspeed_validated_perf{nullptr};
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_sideslip_pub {ORB_ID(estimator_aid_src_sideslip)};
	hrt_abstime _status_sideslip_pub_last {0};
#endif // CONFIG_EKF2_SIDESLIP

	orb_advert_t _mavlink_log_pub{nullptr};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _airdata_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _magnetometer_sub{ORB_ID(vehicle_magnetometer)};
	uORB::Subscription _sensor_selection_sub{ORB_ID(sensor_selection)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	uORB::SubscriptionCallbackWorkItem _sensor_combined_sub{this, ORB_ID(sensor_combined)};
	uORB::SubscriptionCallbackWorkItem _vehicle_imu_sub{this, ORB_ID(vehicle_imu)};

#if defined(CONFIG_EKF2_RANGE_FINDER)
	uORB::PublicationMulti<estimator_bias_s> _estimator_rng_hgt_bias_pub {ORB_ID(estimator_rng_hgt_bias)};
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_rng_hgt_pub{ORB_ID(estimator_aid_src_rng_hgt)};

	uORB::SubscriptionMultiArray<distance_sensor_s> _distance_sensor_subs{ORB_ID::distance_sensor};
	hrt_abstime _last_range_sensor_update{0};
	int _distance_sensor_selected{-1}; // because we can have several distance sensor instances with different orientations
	unsigned _distance_sensor_last_generation{0};
	perf_counter_t _msg_missed_distance_sensor_perf{nullptr};
#endif // CONFIG_EKF2_RANGE_FINDER

	bool _callback_registered{false};

	hrt_abstime _last_event_flags_publish{0};
	hrt_abstime _last_status_flags_publish{0};

	uint64_t _filter_control_status{0};
	uint32_t _filter_fault_status{0};
	uint32_t _innov_check_fail_status{0};

	uint32_t _filter_control_status_changes{0};
	uint32_t _filter_fault_status_changes{0};
	uint32_t _innov_check_fail_status_changes{0};
	uint32_t _filter_warning_event_changes{0};
	uint32_t _filter_information_event_changes{0};

	uORB::PublicationMulti<ekf2_timestamps_s>            _ekf2_timestamps_pub{ORB_ID(ekf2_timestamps)};
	uORB::PublicationMulti<estimator_bias_s>             _estimator_baro_bias_pub{ORB_ID(estimator_baro_bias)};
	uORB::PublicationMulti<estimator_bias_s>             _estimator_gnss_hgt_bias_pub{ORB_ID(estimator_gnss_hgt_bias)};
	uORB::PublicationMultiData<estimator_event_flags_s>  _estimator_event_flags_pub{ORB_ID(estimator_event_flags)};
	uORB::PublicationMulti<estimator_gps_status_s>       _estimator_gps_status_pub{ORB_ID(estimator_gps_status)};
	uORB::PublicationMulti<estimator_innovations_s>      _estimator_innovation_test_ratios_pub{ORB_ID(estimator_innovation_test_ratios)};
	uORB::PublicationMulti<estimator_innovations_s>      _estimator_innovation_variances_pub{ORB_ID(estimator_innovation_variances)};
	uORB::PublicationMulti<estimator_innovations_s>      _estimator_innovations_pub{ORB_ID(estimator_innovations)};
	uORB::PublicationMulti<estimator_sensor_bias_s>      _estimator_sensor_bias_pub{ORB_ID(estimator_sensor_bias)};
	uORB::PublicationMulti<estimator_states_s>           _estimator_states_pub{ORB_ID(estimator_states)};
	uORB::PublicationMulti<estimator_status_flags_s>     _estimator_status_flags_pub{ORB_ID(estimator_status_flags)};
	uORB::PublicationMulti<estimator_status_s>           _estimator_status_pub{ORB_ID(estimator_status)};
	uORB::PublicationMulti<yaw_estimator_status_s>       _yaw_est_pub{ORB_ID(yaw_estimator_status)};

	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_baro_hgt_pub {ORB_ID(estimator_aid_src_baro_hgt)};

	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_fake_hgt_pub{ORB_ID(estimator_aid_src_fake_hgt)};
	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_fake_pos_pub{ORB_ID(estimator_aid_src_fake_pos)};

	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_gnss_hgt_pub{ORB_ID(estimator_aid_src_gnss_hgt)};
	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_gnss_pos_pub{ORB_ID(estimator_aid_src_gnss_pos)};
	uORB::PublicationMulti<estimator_aid_source3d_s> _estimator_aid_src_gnss_vel_pub{ORB_ID(estimator_aid_src_gnss_vel)};
#if defined(CONFIG_EKF2_GNSS_YAW)
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_gnss_yaw_pub {ORB_ID(estimator_aid_src_gnss_yaw)};
#endif // CONFIG_EKF2_GNSS_YAW

	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_mag_heading_pub{ORB_ID(estimator_aid_src_mag_heading)};
	uORB::PublicationMulti<estimator_aid_source3d_s> _estimator_aid_src_mag_pub{ORB_ID(estimator_aid_src_mag)};

	uORB::PublicationMulti<estimator_aid_source3d_s> _estimator_aid_src_gravity_pub{ORB_ID(estimator_aid_src_gravity)};

	// publications with topic dependent on multi-mode
	uORB::PublicationMulti<vehicle_attitude_s>           _attitude_pub;
	uORB::PublicationMulti<vehicle_local_position_s>     _local_position_pub;
	uORB::PublicationMulti<vehicle_global_position_s>    _global_position_pub;
	uORB::PublicationMulti<vehicle_odometry_s>           _odometry_pub;
	uORB::PublicationMulti<wind_s>              _wind_pub;


	PreFlightChecker _preflt_checker;

	Ekf _ekf;

	parameters *_params;	///< pointer to ekf parameter struct (located in _ekf class instance)

	DEFINE_PARAMETERS(
		(ParamExtInt<px4::params::EKF2_PREDICT_US>) _param_ekf2_predict_us,
		(ParamExtInt<px4::params::EKF2_IMU_CTRL>) _param_ekf2_imu_ctrl,

		(ParamExtFloat<px4::params::EKF2_MAG_DELAY>)
		_param_ekf2_mag_delay,	///< magnetometer measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_BARO_DELAY>)
		_param_ekf2_baro_delay,	///< barometer height measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_GPS_DELAY>)
		_param_ekf2_gps_delay,	///< GPS measurement delay relative to the IMU (mSec)

#if defined(CONFIG_EKF2_AUXVEL)
		(ParamExtFloat<px4::params::EKF2_AVEL_DELAY>)
		_param_ekf2_avel_delay,	///< auxiliary velocity measurement delay relative to the IMU (mSec)
#endif // CONFIG_EKF2_AUXVEL

		(ParamExtFloat<px4::params::EKF2_GYR_NOISE>)
		_param_ekf2_gyr_noise,	///< IMU angular rate noise used for covariance prediction (rad/sec)
		(ParamExtFloat<px4::params::EKF2_ACC_NOISE>)
		_param_ekf2_acc_noise,	///< IMU acceleration noise use for covariance prediction (m/sec**2)

		// process noise
		(ParamExtFloat<px4::params::EKF2_GYR_B_NOISE>)
		_param_ekf2_gyr_b_noise,	///< process noise for IMU rate gyro bias prediction (rad/sec**2)
		(ParamExtFloat<px4::params::EKF2_ACC_B_NOISE>)
		_param_ekf2_acc_b_noise,///< process noise for IMU accelerometer bias prediction (m/sec**3)
		(ParamExtFloat<px4::params::EKF2_MAG_E_NOISE>)
		_param_ekf2_mag_e_noise,	///< process noise for earth magnetic field prediction (Gauss/sec)
		(ParamExtFloat<px4::params::EKF2_MAG_B_NOISE>)
		_param_ekf2_mag_b_noise,	///< process noise for body magnetic field prediction (Gauss/sec)
		(ParamExtFloat<px4::params::EKF2_WIND_NSD>)
		_param_ekf2_wind_nsd,	///< process noise spectral density for wind velocity prediction (m/sec**2/sqrt(Hz))

		(ParamExtFloat<px4::params::EKF2_GPS_V_NOISE>)
		_param_ekf2_gps_v_noise,	///< minimum allowed observation noise for gps velocity fusion (m/sec)
		(ParamExtFloat<px4::params::EKF2_GPS_P_NOISE>)
		_param_ekf2_gps_p_noise,	///< minimum allowed observation noise for gps position fusion (m)
		(ParamExtFloat<px4::params::EKF2_NOAID_NOISE>)
		_param_ekf2_noaid_noise,	///< observation noise for non-aiding position fusion (m)
		(ParamExtFloat<px4::params::EKF2_BARO_NOISE>)
		_param_ekf2_baro_noise,	///< observation noise for barometric height fusion (m)
		(ParamExtFloat<px4::params::EKF2_BARO_GATE>)
		_param_ekf2_baro_gate,	///< barometric height innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_GND_EFF_DZ>)
		_param_ekf2_gnd_eff_dz,	///< barometric deadzone range for negative innovations (m)
		(ParamExtFloat<px4::params::EKF2_GND_MAX_HGT>)
		_param_ekf2_gnd_max_hgt,	///< maximum height above the ground level for expected negative baro innovations (m)
		(ParamExtFloat<px4::params::EKF2_GPS_P_GATE>)
		_param_ekf2_gps_p_gate,	///< GPS horizontal position innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_GPS_V_GATE>)
		_param_ekf2_gps_v_gate,	///< GPS velocity innovation consistency gate size (STD)

#if defined(CONFIG_EKF2_AIRSPEED)
		(ParamExtFloat<px4::params::EKF2_ASP_DELAY>)
		_param_ekf2_asp_delay, ///< airspeed measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_TAS_GATE>)
		_param_ekf2_tas_gate, ///< True Airspeed innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_EAS_NOISE>)
		_param_ekf2_eas_noise, ///< measurement noise used for airspeed fusion (m/sec)

		// control of airspeed fusion
		(ParamExtFloat<px4::params::EKF2_ARSP_THR>)
		_param_ekf2_arsp_thr, ///< A value of zero will disabled airspeed fusion. Any positive value sets the minimum airspeed which will be used (m/sec)
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
		(ParamExtFloat<px4::params::EKF2_BETA_GATE>)
		_param_ekf2_beta_gate, ///< synthetic sideslip innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_BETA_NOISE>) _param_ekf2_beta_noise, ///< synthetic sideslip noise (rad)

		(ParamExtInt<px4::params::EKF2_FUSE_BETA>)
		_param_ekf2_fuse_beta, ///< Controls synthetic sideslip fusion, 0 disables, 1 enables
#endif // CONFIG_EKF2_SIDESLIP

		// control of magnetometer fusion
		(ParamExtFloat<px4::params::EKF2_HEAD_NOISE>)
		_param_ekf2_head_noise,	///< measurement noise used for simple heading fusion (rad)
		(ParamExtFloat<px4::params::EKF2_MAG_NOISE>)
		_param_ekf2_mag_noise,		///< measurement noise used for 3-axis magnetoemeter fusion (Gauss)

		(ParamExtFloat<px4::params::EKF2_MAG_DECL>) _param_ekf2_mag_decl,///< magnetic declination (degrees)
		(ParamExtFloat<px4::params::EKF2_HDG_GATE>)
		_param_ekf2_hdg_gate,///< heading fusion innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_MAG_GATE>)
		_param_ekf2_mag_gate,	///< magnetometer fusion innovation consistency gate size (STD)
		(ParamExtInt<px4::params::EKF2_DECL_TYPE>)
		_param_ekf2_decl_type,	///< bitmask used to control the handling of declination data
		(ParamExtInt<px4::params::EKF2_MAG_TYPE>)
		_param_ekf2_mag_type,	///< integer used to specify the type of magnetometer fusion used
		(ParamExtFloat<px4::params::EKF2_MAG_ACCLIM>)
		_param_ekf2_mag_acclim,	///< integer used to specify the type of magnetometer fusion used
		(ParamExtFloat<px4::params::EKF2_MAG_YAWLIM>)
		_param_ekf2_mag_yawlim,	///< yaw rate threshold used by mode select logic (rad/sec)

		(ParamExtInt<px4::params::EKF2_GPS_CHECK>)
		_param_ekf2_gps_check,	///< bitmask used to control which GPS quality checks are used
		(ParamExtFloat<px4::params::EKF2_REQ_EPH>) _param_ekf2_req_eph,	///< maximum acceptable horiz position error (m)
		(ParamExtFloat<px4::params::EKF2_REQ_EPV>) _param_ekf2_req_epv,	///< maximum acceptable vert position error (m)
		(ParamExtFloat<px4::params::EKF2_REQ_SACC>) _param_ekf2_req_sacc,	///< maximum acceptable speed error (m/s)
		(ParamExtInt<px4::params::EKF2_REQ_NSATS>) _param_ekf2_req_nsats,	///< minimum acceptable satellite count
		(ParamExtFloat<px4::params::EKF2_REQ_PDOP>)
		_param_ekf2_req_pdop,	///< maximum acceptable position dilution of precision
		(ParamExtFloat<px4::params::EKF2_REQ_HDRIFT>)
		_param_ekf2_req_hdrift,	///< maximum acceptable horizontal drift speed (m/s)
		(ParamExtFloat<px4::params::EKF2_REQ_VDRIFT>) _param_ekf2_req_vdrift,	///< maximum acceptable vertical drift speed (m/s)

		// measurement source control
		(ParamInt<px4::params::EKF2_AID_MASK>)
		_param_ekf2_aid_mask,		///< bitmasked integer that selects which of the GPS and optical flow aiding sources will be used
		(ParamExtInt<px4::params::EKF2_HGT_REF>) _param_ekf2_hgt_ref,    ///< selects the primary source for height data
		(ParamExtInt<px4::params::EKF2_BARO_CTRL>) _param_ekf2_baro_ctrl,///< barometer control selection
		(ParamExtInt<px4::params::EKF2_GPS_CTRL>) _param_ekf2_gps_ctrl,  ///< GPS control selection

		(ParamExtInt<px4::params::EKF2_NOAID_TOUT>)
		_param_ekf2_noaid_tout,	///< maximum lapsed time from last fusion of measurements that constrain drift before the EKF will report the horizontal nav solution invalid (uSec)

#if defined(CONFIG_EKF2_RANGE_FINDER)
		// range finder fusion
		(ParamExtInt<px4::params::EKF2_RNG_CTRL>) _param_ekf2_rng_ctrl, ///< range finder control selection
		(ParamExtFloat<px4::params::EKF2_RNG_DELAY>)
		_param_ekf2_rng_delay,   ///< range finder measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_RNG_NOISE>)
		_param_ekf2_rng_noise,   ///< observation noise for range finder measurements (m)
		(ParamExtFloat<px4::params::EKF2_RNG_SFE>)
		_param_ekf2_rng_sfe,     ///< scale factor from range to range noise (m/m)
		(ParamExtFloat<px4::params::EKF2_RNG_GATE>)
		_param_ekf2_rng_gate,    ///< range finder fusion innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_MIN_RNG>)
		_param_ekf2_min_rng,     ///< minimum valid value for range when on ground (m)
		(ParamExtFloat<px4::params::EKF2_RNG_PITCH>)   _param_ekf2_rng_pitch,   ///< range sensor pitch offset (rad)
		(ParamExtFloat<px4::params::EKF2_RNG_A_VMAX>)
		_param_ekf2_rng_a_vmax,  ///< maximum allowed horizontal velocity for range aid (m/s)
		(ParamExtFloat<px4::params::EKF2_RNG_A_HMAX>)
		_param_ekf2_rng_a_hmax,  ///< maximum allowed absolute altitude (AGL) for range aid (m)
		(ParamExtFloat<px4::params::EKF2_RNG_A_IGATE>)
		_param_ekf2_rng_a_igate, ///< gate size used for innovation consistency checks for range aid fusion (STD)
		(ParamExtFloat<px4::params::EKF2_RNG_QLTY_T>)
		_param_ekf2_rng_qlty_t,  ///< Minimum duration during which the reported range finder signal quality needs to be non-zero in order to be declared valid (s)
		(ParamExtFloat<px4::params::EKF2_RNG_K_GATE>)
		_param_ekf2_rng_k_gate,  ///< range finder kinematic consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_RNG_POS_X>)
		_param_ekf2_rng_pos_x,   ///< X position of range finder in body frame (m)
		(ParamExtFloat<px4::params::EKF2_RNG_POS_Y>)
		_param_ekf2_rng_pos_y,   ///< Y position of range finder in body frame (m)
		(ParamExtFloat<px4::params::EKF2_RNG_POS_Z>)
		_param_ekf2_rng_pos_z,   ///< Z position of range finder in body frame (m)

		(ParamExtInt<px4::params::EKF2_TERR_MASK>)
		_param_ekf2_terr_mask, ///< bitmasked integer that selects which of range finder and optical flow aiding sources will be used for terrain estimation
		(ParamExtFloat<px4::params::EKF2_TERR_NOISE>) _param_ekf2_terr_noise, ///< process noise for terrain offset (m/sec)
		(ParamExtFloat<px4::params::EKF2_TERR_GRAD>)
		_param_ekf2_terr_grad, ///< gradient of terrain used to estimate process noise due to changing position (m/m)
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
		// vision estimate fusion
		(ParamExtFloat<px4::params::EKF2_EV_DELAY>)
		_param_ekf2_ev_delay, ///< off-board vision measurement delay relative to the IMU (mSec)

		(ParamExtInt<px4::params::EKF2_EV_CTRL>) _param_ekf2_ev_ctrl,	 ///< external vision (EV) control selection
		(ParamInt<px4::params::EKF2_EV_NOISE_MD>) _param_ekf2_ev_noise_md, ///< determine source of vision observation noise
		(ParamExtInt<px4::params::EKF2_EV_QMIN>) _param_ekf2_ev_qmin,
		(ParamExtFloat<px4::params::EKF2_EVP_NOISE>)
		_param_ekf2_evp_noise, ///< default position observation noise for exernal vision measurements (m)
		(ParamExtFloat<px4::params::EKF2_EVV_NOISE>)
		_param_ekf2_evv_noise, ///< default velocity observation noise for exernal vision measurements (m/s)
		(ParamExtFloat<px4::params::EKF2_EVA_NOISE>)
		_param_ekf2_eva_noise, ///< default angular observation noise for exernal vision measurements (rad)
		(ParamExtFloat<px4::params::EKF2_EVV_GATE>)
		_param_ekf2_evv_gate, ///< external vision velocity innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_EVP_GATE>)
		_param_ekf2_evp_gate, ///< external vision position innovation consistency gate size (STD)

		(ParamExtFloat<px4::params::EKF2_EV_POS_X>)
		_param_ekf2_ev_pos_x, ///< X position of VI sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_EV_POS_Y>)
		_param_ekf2_ev_pos_y, ///< Y position of VI sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_EV_POS_Z>)
		_param_ekf2_ev_pos_z, ///< Z position of VI sensor focal point in body frame (m)
#endif // CONFIG_EKF2_EXTERNAL_VISION

		(ParamExtFloat<px4::params::EKF2_GRAV_NOISE>)
		_param_ekf2_grav_noise,	///< default accelerometer noise for gravity fusion measurements (m/s**2)

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
		// optical flow fusion
		(ParamExtInt<px4::params::EKF2_OF_CTRL>)
		_param_ekf2_of_ctrl, ///< optical flow fusion selection
		(ParamExtFloat<px4::params::EKF2_OF_DELAY>)
		_param_ekf2_of_delay, ///< optical flow measurement delay relative to the IMU (mSec) - this is to the middle of the optical flow integration interval
		(ParamExtFloat<px4::params::EKF2_OF_N_MIN>)
		_param_ekf2_of_n_min, ///< best quality observation noise for optical flow LOS rate measurements (rad/sec)
		(ParamExtFloat<px4::params::EKF2_OF_N_MAX>)
		_param_ekf2_of_n_max, ///< worst quality observation noise for optical flow LOS rate measurements (rad/sec)
		(ParamExtInt<px4::params::EKF2_OF_QMIN>)
		_param_ekf2_of_qmin, ///< minimum acceptable quality integer from  the flow sensor when in air
		(ParamExtInt<px4::params::EKF2_OF_QMIN_GND>)
		_param_ekf2_of_qmin_gnd, ///< minimum acceptable quality integer from  the flow sensor when on ground
		(ParamExtFloat<px4::params::EKF2_OF_GATE>)
		_param_ekf2_of_gate, ///< optical flow fusion innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_OF_POS_X>)
		_param_ekf2_of_pos_x, ///< X position of optical flow sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_OF_POS_Y>)
		_param_ekf2_of_pos_y, ///< Y position of optical flow sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_OF_POS_Z>)
		_param_ekf2_of_pos_z, ///< Z position of optical flow sensor focal point in body frame (m)
#endif // CONFIG_EKF2_OPTICAL_FLOW

		// sensor positions in body frame
		(ParamExtFloat<px4::params::EKF2_IMU_POS_X>) _param_ekf2_imu_pos_x,		///< X position of IMU in body frame (m)
		(ParamExtFloat<px4::params::EKF2_IMU_POS_Y>) _param_ekf2_imu_pos_y,		///< Y position of IMU in body frame (m)
		(ParamExtFloat<px4::params::EKF2_IMU_POS_Z>) _param_ekf2_imu_pos_z,		///< Z position of IMU in body frame (m)
		(ParamExtFloat<px4::params::EKF2_GPS_POS_X>) _param_ekf2_gps_pos_x,		///< X position of GPS antenna in body frame (m)
		(ParamExtFloat<px4::params::EKF2_GPS_POS_Y>) _param_ekf2_gps_pos_y,		///< Y position of GPS antenna in body frame (m)
		(ParamExtFloat<px4::params::EKF2_GPS_POS_Z>) _param_ekf2_gps_pos_z,		///< Z position of GPS antenna in body frame (m)

		// output predictor filter time constants
		(ParamFloat<px4::params::EKF2_TAU_VEL>)
		_param_ekf2_tau_vel,		///< time constant used by the output velocity complementary filter (sec)
		(ParamFloat<px4::params::EKF2_TAU_POS>)
		_param_ekf2_tau_pos,		///< time constant used by the output position complementary filter (sec)

		// IMU switch on bias parameters
		(ParamExtFloat<px4::params::EKF2_GBIAS_INIT>)
		_param_ekf2_gbias_init,	///< 1-sigma gyro bias uncertainty at switch on (rad/sec)
		(ParamExtFloat<px4::params::EKF2_ABIAS_INIT>)
		_param_ekf2_abias_init,	///< 1-sigma accelerometer bias uncertainty at switch on (m/sec**2)
		(ParamExtFloat<px4::params::EKF2_ANGERR_INIT>)
		_param_ekf2_angerr_init,	///< 1-sigma tilt error after initial alignment using gravity vector (rad)

		// EKF accel bias learning control
		(ParamExtFloat<px4::params::EKF2_ABL_LIM>) _param_ekf2_abl_lim,	///< Accelerometer bias learning limit (m/s**2)
		(ParamExtFloat<px4::params::EKF2_ABL_ACCLIM>)
		_param_ekf2_abl_acclim,	///< Maximum IMU accel magnitude that allows IMU bias learning (m/s**2)
		(ParamExtFloat<px4::params::EKF2_ABL_GYRLIM>)
		_param_ekf2_abl_gyrlim,	///< Maximum IMU gyro angular rate magnitude that allows IMU bias learning (m/s**2)
		(ParamExtFloat<px4::params::EKF2_ABL_TAU>)
		_param_ekf2_abl_tau,	///< Time constant used to inhibit IMU delta velocity bias learning (sec)

		(ParamExtFloat<px4::params::EKF2_GYR_B_LIM>) _param_ekf2_gyr_b_lim,	///< Gyro bias learning limit (rad/s)

#if defined(CONFIG_EKF2_DRAG_FUSION)
		(ParamExtInt<px4::params::EKF2_DRAG_CTRL>) _param_ekf2_drag_ctrl,		///< drag fusion selection
		// Multi-rotor drag specific force fusion
		(ParamExtFloat<px4::params::EKF2_DRAG_NOISE>)
		_param_ekf2_drag_noise,	///< observation noise variance for drag specific force measurements (m/sec**2)**2
		(ParamExtFloat<px4::params::EKF2_BCOEF_X>) _param_ekf2_bcoef_x,		///< ballistic coefficient along the X-axis (kg/m**2)
		(ParamExtFloat<px4::params::EKF2_BCOEF_Y>) _param_ekf2_bcoef_y,		///< ballistic coefficient along the Y-axis (kg/m**2)
		(ParamExtFloat<px4::params::EKF2_MCOEF>) _param_ekf2_mcoef,		///< propeller momentum drag coefficient (1/s)
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_BARO_COMPENSATION)
		// Corrections for static pressure position error where Ps_error = Ps_meas - Ps_truth
		// Coef = Ps_error / Pdynamic, where Pdynamic = 1/2 * density * TAS**2
		(ParamExtFloat<px4::params::EKF2_ASPD_MAX>)
		_param_ekf2_aspd_max,		///< upper limit on airspeed used for correction  (m/s**2)
		(ParamExtFloat<px4::params::EKF2_PCOEF_XP>)
		_param_ekf2_pcoef_xp,	///< static pressure position error coefficient along the positive X body axis
		(ParamExtFloat<px4::params::EKF2_PCOEF_XN>)
		_param_ekf2_pcoef_xn,	///< static pressure position error coefficient along the negative X body axis
		(ParamExtFloat<px4::params::EKF2_PCOEF_YP>)
		_param_ekf2_pcoef_yp,	///< static pressure position error coefficient along the positive Y body axis
		(ParamExtFloat<px4::params::EKF2_PCOEF_YN>)
		_param_ekf2_pcoef_yn,	///< static pressure position error coefficient along the negative Y body axis
		(ParamExtFloat<px4::params::EKF2_PCOEF_Z>)
		_param_ekf2_pcoef_z,	///< static pressure position error coefficient along the Z body axis
#endif // CONFIG_EKF2_BARO_COMPENSATION

		(ParamFloat<px4::params::EKF2_REQ_GPS_H>) _param_ekf2_req_gps_h, ///< Required GPS health time
		(ParamExtInt<px4::params::EKF2_MAG_CHECK>) _param_ekf2_mag_check, ///< Mag field strength check
		(ParamExtInt<px4::params::EKF2_SYNT_MAG_Z>)
		_param_ekf2_synthetic_mag_z, ///< Enables the use of a synthetic value for the Z axis of the magnetometer calculated from the 3D magnetic field vector at the location of the drone.

		// Used by EKF-GSF experimental yaw estimator
		(ParamExtFloat<px4::params::EKF2_GSF_TAS>)
		_param_ekf2_gsf_tas_default	///< default value of true airspeed assumed during fixed wing operation

	)
};
#endif // !EKF2_HPP
/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "EKF2Selector.hpp"

using namespace time_literals;
using matrix::Quatf;
using matrix::Vector2f;
using math::constrain;
using math::radians;

EKF2Selector::EKF2Selector() :
	ModuleParams(nullptr),
	ScheduledWorkItem("ekf2_selector", px4::wq_configurations::nav_and_controllers)
{
	_estimator_selector_status_pub.advertise();
	_sensor_selection_pub.advertise();
	_vehicle_attitude_pub.advertise();
	_vehicle_global_position_pub.advertise();
	_vehicle_local_position_pub.advertise();
	_vehicle_odometry_pub.advertise();
	_wind_pub.advertise();
}

EKF2Selector::~EKF2Selector()
{
	Stop();
}

void EKF2Selector::Stop()
{
	for (int i = 0; i < EKF2_MAX_INSTANCES; i++) {
		_instance[i].estimator_attitude_sub.unregisterCallback();
		_instance[i].estimator_status_sub.unregisterCallback();
	}

	ScheduleClear();
}

void EKF2Selector::PrintInstanceChange(const uint8_t old_instance, uint8_t new_instance)
{
	const char *old_reason = nullptr;

	if (_instance[old_instance].filter_fault) {
		old_reason = " (filter fault)";

	} else if (_instance[old_instance].timeout) {
		old_reason = " (timeout)";

	} else if (_gyro_fault_detected) {
		old_reason = " (gyro fault)";

	} else if (_accel_fault_detected) {
		old_reason = " (accel fault)";

	} else if (!_instance[_selected_instance].healthy.get_state() && (_instance[_selected_instance].healthy_count > 0)) {
		// skipped if previous instance was never healthy in the first place (eg initialization)
		old_reason = " (unhealthy)";
	}

	const char *new_reason = nullptr;

	if (_request_instance.load() == new_instance) {
		new_reason = " (user selected)";
	}

	if (old_reason || new_reason) {
		if (old_reason == nullptr) {
			old_reason = "";
		}

		if (new_reason == nullptr) {
			new_reason = "";
		}

		PX4_WARN("primary EKF changed %" PRIu8 "%s -> %" PRIu8 "%s", old_instance, old_reason, new_instance, new_reason);
	}
}

bool EKF2Selector::SelectInstance(uint8_t ekf_instance)
{
	if ((ekf_instance != _selected_instance) && (ekf_instance < _available_instances)) {
		// update sensor_selection immediately
		sensor_selection_s sensor_selection{};
		sensor_selection.accel_device_id = _instance[ekf_instance].accel_device_id;
		sensor_selection.gyro_device_id = _instance[ekf_instance].gyro_device_id;
		sensor_selection.timestamp = hrt_absolute_time();
		_sensor_selection_pub.publish(sensor_selection);

		if (_selected_instance != INVALID_INSTANCE) {
			// switch callback registration
			_instance[_selected_instance].estimator_attitude_sub.unregisterCallback();
			_instance[_selected_instance].estimator_status_sub.unregisterCallback();

			PrintInstanceChange(_selected_instance, ekf_instance);
		}

		_instance[ekf_instance].estimator_attitude_sub.registerCallback();
		_instance[ekf_instance].estimator_status_sub.registerCallback();

		_selected_instance = ekf_instance;
		_instance_changed_count++;
		_last_instance_change = sensor_selection.timestamp;
		_instance[ekf_instance].time_last_selected = _last_instance_change;

		// reset all relative test ratios
		for (uint8_t i = 0; i < _available_instances; i++) {
			_instance[i].relative_test_ratio = 0;
		}

		return true;
	}

	return false;
}

bool EKF2Selector::UpdateErrorScores()
{
	// first check imu inconsistencies
	_gyro_fault_detected = false;
	uint32_t faulty_gyro_id = 0;
	_accel_fault_detected = false;
	uint32_t faulty_accel_id = 0;

	if (_sensors_status_imu.updated()) {
		sensors_status_imu_s sensors_status_imu;

		if (_sensors_status_imu.copy(&sensors_status_imu)) {

			const float time_step_s = constrain((sensors_status_imu.timestamp - _last_update_us) * 1e-6f, 0.f, 0.02f);
			_last_update_us = sensors_status_imu.timestamp;

			{
				const float angle_rate_threshold = radians(_param_ekf2_sel_imu_angle_rate.get());
				const float angle_threshold = radians(_param_ekf2_sel_imu_angle.get());
				uint8_t n_gyros = 0;
				uint8_t n_gyro_exceedances = 0;
				float largest_accumulated_gyro_error = 0.0f;
				uint8_t largest_gyro_error_index = 0;

				for (unsigned i = 0; i < IMU_STATUS_SIZE; i++) {
					// check for gyros with excessive difference to mean using accumulated error
					if (sensors_status_imu.gyro_device_ids[i] != 0) {
						n_gyros++;
						_accumulated_gyro_error[i] += (sensors_status_imu.gyro_inconsistency_rad_s[i] - angle_rate_threshold) * time_step_s;
						_accumulated_gyro_error[i] = fmaxf(_accumulated_gyro_error[i], 0.f);

						if (_accumulated_gyro_error[i] > angle_threshold) {
							n_gyro_exceedances++;
						}

						if (_accumulated_gyro_error[i] > largest_accumulated_gyro_error) {
							largest_accumulated_gyro_error = _accumulated_gyro_error[i];
							largest_gyro_error_index = i;
						}

					} else {
						// no sensor
						_accumulated_gyro_error[i] = NAN;
					}
				}

				if (n_gyro_exceedances > 0) {
					if (n_gyros >= 3) {
						// If there are 3 or more sensors, the one with the largest accumulated error is faulty
						_gyro_fault_detected = true;
						faulty_gyro_id = sensors_status_imu.gyro_device_ids[largest_gyro_error_index];

					} else if (n_gyros == 2) {
						// A fault is present, but the faulty sensor identity cannot be determined
						_gyro_fault_detected = true;
					}
				}
			}

			{
				const float accel_threshold = _param_ekf2_sel_imu_accel.get();
				const float velocity_threshold = _param_ekf2_sel_imu_velocity.get();
				uint8_t n_accels = 0;
				uint8_t n_accel_exceedances = 0;
				float largest_accumulated_accel_error = 0.0f;
				uint8_t largest_accel_error_index = 0;

				for (unsigned i = 0; i < IMU_STATUS_SIZE; i++) {
					// check for accelerometers with excessive difference to mean using accumulated error
					if (sensors_status_imu.accel_device_ids[i] != 0) {
						n_accels++;
						_accumulated_accel_error[i] += (sensors_status_imu.accel_inconsistency_m_s_s[i] - accel_threshold) * time_step_s;
						_accumulated_accel_error[i] = fmaxf(_accumulated_accel_error[i], 0.f);

						if (_accumulated_accel_error[i] > velocity_threshold) {
							n_accel_exceedances++;
						}

						if (_accumulated_accel_error[i] > largest_accumulated_accel_error) {
							largest_accumulated_accel_error = _accumulated_accel_error[i];
							largest_accel_error_index = i;
						}

					} else {
						// no sensor
						_accumulated_accel_error[i] = NAN;
					}
				}

				if (n_accel_exceedances > 0) {
					if (n_accels >= 3) {
						// If there are 3 or more sensors, the one with the largest accumulated error is faulty
						_accel_fault_detected = true;
						faulty_accel_id = sensors_status_imu.accel_device_ids[largest_accel_error_index];

					} else if (n_accels == 2) {
						// A fault is present, but the faulty sensor identity cannot be determined
						_accel_fault_detected = true;
					}
				}
			}
		}
	}

	bool updated = false;
	bool primary_updated = false;

	// default estimator timeout
	const hrt_abstime status_timeout = 50_ms;

	// calculate individual error scores
	for (uint8_t i = 0; i < EKF2_MAX_INSTANCES; i++) {
		const bool prev_healthy = _instance[i].healthy.get_state();

		estimator_status_s status;

		if (_instance[i].estimator_status_sub.update(&status)) {

			_instance[i].timestamp_last = status.timestamp;

			_instance[i].accel_device_id = status.accel_device_id;
			_instance[i].gyro_device_id = status.gyro_device_id;
			_instance[i].baro_device_id = status.baro_device_id;
			_instance[i].mag_device_id = status.mag_device_id;

			if ((i + 1) > _available_instances) {
				_available_instances = i + 1;
				updated = true;
			}

			if (i == _selected_instance) {
				primary_updated = true;
			}

			// test ratios are invalid when 0, >= 1 is a failure
			if (!PX4_ISFINITE(status.vel_test_ratio) || (status.vel_test_ratio <= 0.f)) {
				status.vel_test_ratio = 1.f;
			}

			if (!PX4_ISFINITE(status.pos_test_ratio) || (status.pos_test_ratio <= 0.f)) {
				status.pos_test_ratio = 1.f;
			}

			if (!PX4_ISFINITE(status.hgt_test_ratio) || (status.hgt_test_ratio <= 0.f)) {
				status.hgt_test_ratio = 1.f;
			}

			float combined_test_ratio = fmaxf(0.5f * (status.vel_test_ratio + status.pos_test_ratio), status.hgt_test_ratio);

			_instance[i].combined_test_ratio = combined_test_ratio;

			const bool healthy = (status.filter_fault_flags == 0) && (combined_test_ratio > 0.f);
			_instance[i].healthy.set_state_and_update(healthy, status.timestamp);

			_instance[i].warning = (combined_test_ratio >= 1.f);
			_instance[i].filter_fault = (status.filter_fault_flags != 0);
			_instance[i].timeout = false;

			if (!_instance[i].warning) {
				_instance[i].time_last_no_warning = status.timestamp;
			}

			if (!PX4_ISFINITE(_instance[i].relative_test_ratio)) {
				_instance[i].relative_test_ratio = 0;
			}

		} else if (!_instance[i].timeout && (hrt_elapsed_time(&_instance[i].timestamp_last) > status_timeout)) {
			_instance[i].healthy.set_state_and_update(false, hrt_absolute_time());
			_instance[i].timeout = true;
		}

		// if the gyro used by the EKF is faulty, declare the EKF unhealthy without delay
		if (_gyro_fault_detected && (faulty_gyro_id != 0) && (_instance[i].gyro_device_id == faulty_gyro_id)) {
			_instance[i].healthy.set_state_and_update(false, hrt_absolute_time());
		}

		// if the accelerometer used by the EKF is faulty, declare the EKF unhealthy without delay
		if (_accel_fault_detected && (faulty_accel_id != 0) && (_instance[i].accel_device_id == faulty_accel_id)) {
			_instance[i].healthy.set_state_and_update(false, hrt_absolute_time());
		}

		if (prev_healthy != _instance[i].healthy.get_state()) {
			updated = true;
			_selector_status_publish = true;

			if (!prev_healthy) {
				_instance[i].healthy_count++;
			}
		}
	}

	// update relative test ratios if primary has updated
	if (primary_updated) {
		for (uint8_t i = 0; i < _available_instances; i++) {
			if (i != _selected_instance) {

				const float error_delta = _instance[i].combined_test_ratio - _instance[_selected_instance].combined_test_ratio;

				// reduce error only if its better than the primary instance by at least EKF2_SEL_ERR_RED to prevent unnecessary selection changes
				const float threshold = _gyro_fault_detected ? 0.0f : fmaxf(_param_ekf2_sel_err_red.get(), 0.05f);

				if (error_delta > 0 || error_delta < -threshold) {
					_instance[i].relative_test_ratio += error_delta;
					_instance[i].relative_test_ratio = constrain(_instance[i].relative_test_ratio, -_rel_err_score_lim, _rel_err_score_lim);

					if ((error_delta < -threshold) && (_instance[i].relative_test_ratio < 1.f)) {
						// increase status publication rate if there's movement towards a potential instance change
						_selector_status_publish = true;
					}
				}
			}
		}
	}

	return (primary_updated || updated);
}

void EKF2Selector::PublishVehicleAttitude()
{
	// selected estimator_attitude -> vehicle_attitude
	vehicle_attitude_s attitude;

	if (_instance[_selected_instance].estimator_attitude_sub.update(&attitude)) {
		bool instance_change = false;

		if (_instance[_selected_instance].estimator_attitude_sub.get_instance() != _attitude_instance_prev) {
			_attitude_instance_prev = _instance[_selected_instance].estimator_attitude_sub.get_instance();
			instance_change = true;
		}

		if (_attitude_last.timestamp != 0) {
			if (!instance_change && (attitude.quat_reset_counter == _attitude_last.quat_reset_counter + 1)) {
				// propogate deltas from estimator data while maintaining the overall reset counts
				++_quat_reset_counter;
				_delta_q_reset = Quatf{attitude.delta_q_reset};

			} else if (instance_change || (attitude.quat_reset_counter != _attitude_last.quat_reset_counter)) {
				// on reset compute deltas from last published data
				++_quat_reset_counter;
				_delta_q_reset = (Quatf(attitude.q) * Quatf(_attitude_last.q).inversed()).normalized();
			}

		} else {
			_quat_reset_counter = attitude.quat_reset_counter;
			_delta_q_reset = Quatf{attitude.delta_q_reset};
		}

		bool publish = true;

		// ensure monotonically increasing timestamp_sample through reset, don't publish
		//  estimator's attitude for system (vehicle_attitude) if it's stale
		if ((attitude.timestamp_sample <= _attitude_last.timestamp_sample)
		    || (hrt_elapsed_time(&attitude.timestamp) > 10_ms)) {

			publish = false;
		}

		// save last primary estimator_attitude as published with original resets
		_attitude_last = attitude;

		if (publish) {
			// republish with total reset count and current timestamp
			attitude.quat_reset_counter = _quat_reset_counter;
			_delta_q_reset.copyTo(attitude.delta_q_reset);

			attitude.timestamp = hrt_absolute_time();
			_vehicle_attitude_pub.publish(attitude);
		}
	}
}

void EKF2Selector::PublishVehicleLocalPosition()
{
	// selected estimator_local_position -> vehicle_local_position
	vehicle_local_position_s local_position;

	if (_instance[_selected_instance].estimator_local_position_sub.update(&local_position)) {
		bool instance_change = false;

		if (_instance[_selected_instance].estimator_local_position_sub.get_instance() != _local_position_instance_prev) {
			_local_position_instance_prev = _instance[_selected_instance].estimator_local_position_sub.get_instance();
			instance_change = true;
		}

		if (_local_position_last.timestamp != 0) {
			// XY reset
			if (!instance_change && (local_position.xy_reset_counter == _local_position_last.xy_reset_counter + 1)) {
				++_xy_reset_counter;
				_delta_xy_reset = Vector2f{local_position.delta_xy};

			} else if (instance_change || (local_position.xy_reset_counter != _local_position_last.xy_reset_counter)) {
				++_xy_reset_counter;
				_delta_xy_reset = Vector2f{local_position.x, local_position.y} - Vector2f{_local_position_last.x, _local_position_last.y};
			}

			// Z reset
			if (!instance_change && (local_position.z_reset_counter == _local_position_last.z_reset_counter + 1)) {
				++_z_reset_counter;
				_delta_z_reset = local_position.delta_z;

			} else if (instance_change || (local_position.z_reset_counter != _local_position_last.z_reset_counter)) {
				++_z_reset_counter;
				_delta_z_reset = local_position.z - _local_position_last.z;
			}

			// VXY reset
			if (!instance_change && (local_position.vxy_reset_counter == _local_position_last.vxy_reset_counter + 1)) {
				++_vxy_reset_counter;
				_delta_vxy_reset = Vector2f{local_position.delta_vxy};

			} else if (instance_change || (local_position.vxy_reset_counter != _local_position_last.vxy_reset_counter)) {
				++_vxy_reset_counter;
				_delta_vxy_reset = Vector2f{local_position.vx, local_position.vy} - Vector2f{_local_position_last.vx, _local_position_last.vy};
			}

			// VZ reset
			if (!instance_change && (local_position.vz_reset_counter == _local_position_last.vz_reset_counter + 1)) {
				++_vz_reset_counter;
				_delta_vz_reset = local_position.delta_vz;

			} else if (instance_change || (local_position.vz_reset_counter != _local_position_last.vz_reset_counter)) {
				++_vz_reset_counter;
				_delta_vz_reset = local_position.vz - _local_position_last.vz;
			}

			// heading reset
			if (!instance_change && (local_position.heading_reset_counter == _local_position_last.heading_reset_counter + 1)) {
				++_heading_reset_counter;
				_delta_heading_reset = local_position.delta_heading;

			} else if (instance_change || (local_position.heading_reset_counter != _local_position_last.heading_reset_counter)) {
				++_heading_reset_counter;
				_delta_heading_reset = matrix::wrap_pi(local_position.heading - _local_position_last.heading);
			}

		} else {
			_xy_reset_counter = local_position.xy_reset_counter;
			_z_reset_counter = local_position.z_reset_counter;
			_vxy_reset_counter = local_position.vxy_reset_counter;
			_vz_reset_counter = local_position.vz_reset_counter;
			_heading_reset_counter = local_position.heading_reset_counter;

			_delta_xy_reset = Vector2f{local_position.delta_xy};
			_delta_z_reset = local_position.delta_z;
			_delta_vxy_reset = Vector2f{local_position.delta_vxy};
			_delta_vz_reset = local_position.delta_vz;
			_delta_heading_reset = local_position.delta_heading;
		}

		bool publish = true;

		// ensure monotonically increasing timestamp_sample through reset, don't publish
		//  estimator's local position for system (vehicle_local_position) if it's stale
		if ((local_position.timestamp_sample <= _local_position_last.timestamp_sample)
		    || (hrt_elapsed_time(&local_position.timestamp) > 20_ms)) {

			publish = false;
		}

		// save last primary estimator_local_position as published with original resets
		_local_position_last = local_position;

		if (publish) {
			// republish with total reset count and current timestamp
			local_position.xy_reset_counter = _xy_reset_counter;
			local_position.z_reset_counter = _z_reset_counter;
			local_position.vxy_reset_counter = _vxy_reset_counter;
			local_position.vz_reset_counter = _vz_reset_counter;
			local_position.heading_reset_counter = _heading_reset_counter;

			_delta_xy_reset.copyTo(local_position.delta_xy);
			local_position.delta_z = _delta_z_reset;
			_delta_vxy_reset.copyTo(local_position.delta_vxy);
			local_position.delta_vz = _delta_vz_reset;
			local_position.delta_heading = _delta_heading_reset;

			local_position.timestamp = hrt_absolute_time();
			_vehicle_local_position_pub.publish(local_position);
		}
	}
}

void EKF2Selector::PublishVehicleOdometry()
{
	// selected estimator_odometry -> vehicle_odometry
	vehicle_odometry_s odometry;

	if (_instance[_selected_instance].estimator_odometry_sub.update(&odometry)) {

		bool instance_change = false;

		if (_instance[_selected_instance].estimator_odometry_sub.get_instance() != _odometry_instance_prev) {
			_odometry_instance_prev = _instance[_selected_instance].estimator_odometry_sub.get_instance();
			instance_change = true;
		}

		if (_odometry_last.timestamp != 0) {
			// reset
			if (instance_change || (odometry.reset_counter != _odometry_last.reset_counter)) {
				++_odometry_reset_counter;
			}

		} else {
			_odometry_reset_counter = odometry.reset_counter;
		}

		bool publish = true;

		// ensure monotonically increasing timestamp_sample through reset, don't publish
		//  estimator's odometry for system (vehicle_odometry) if it's stale
		if ((odometry.timestamp_sample <= _odometry_last.timestamp_sample)
		    || (hrt_elapsed_time(&odometry.timestamp) > 20_ms)) {

			publish = false;
		}

		// save last primary estimator_odometry as published with original resets
		_odometry_last = odometry;

		if (publish) {
			// republish with total reset count and current timestamp
			odometry.reset_counter = _odometry_reset_counter;

			odometry.timestamp = hrt_absolute_time();
			_vehicle_odometry_pub.publish(odometry);
		}
	}
}

void EKF2Selector::PublishVehicleGlobalPosition()
{
	// selected estimator_global_position -> vehicle_global_position
	vehicle_global_position_s global_position;

	if (_instance[_selected_instance].estimator_global_position_sub.update(&global_position)) {
		bool instance_change = false;

		if (_instance[_selected_instance].estimator_global_position_sub.get_instance() != _global_position_instance_prev) {
			_global_position_instance_prev = _instance[_selected_instance].estimator_global_position_sub.get_instance();
			instance_change = true;
		}

		if (_global_position_last.timestamp != 0) {
			// lat/lon reset
			if (!instance_change && (global_position.lat_lon_reset_counter == _global_position_last.lat_lon_reset_counter + 1)) {
				++_lat_lon_reset_counter;

				// TODO: delta latitude/longitude
				_delta_lat_reset = global_position.lat - _global_position_last.lat;
				_delta_lon_reset = global_position.lon - _global_position_last.lon;

			} else if (instance_change || (global_position.lat_lon_reset_counter != _global_position_last.lat_lon_reset_counter)) {
				++_lat_lon_reset_counter;

				_delta_lat_reset = global_position.lat - _global_position_last.lat;
				_delta_lon_reset = global_position.lon - _global_position_last.lon;
			}

			// alt reset
			if (!instance_change && (global_position.alt_reset_counter == _global_position_last.alt_reset_counter + 1)) {
				++_alt_reset_counter;
				_delta_alt_reset = global_position.delta_alt;

			} else if (instance_change || (global_position.alt_reset_counter != _global_position_last.alt_reset_counter)) {
				++_alt_reset_counter;
				_delta_alt_reset = global_position.delta_alt - _global_position_last.delta_alt;
			}

		} else {
			_lat_lon_reset_counter = global_position.lat_lon_reset_counter;
			_alt_reset_counter = global_position.alt_reset_counter;

			_delta_alt_reset = global_position.delta_alt;
		}

		bool publish = true;

		// ensure monotonically increasing timestamp_sample through reset, don't publish
		//  estimator's global position for system (vehicle_global_position) if it's stale
		if ((global_position.timestamp_sample <= _global_position_last.timestamp_sample)
		    || (hrt_elapsed_time(&global_position.timestamp) > 20_ms)) {

			publish = false;
		}

		// save last primary estimator_global_position as published with original resets
		_global_position_last = global_position;

		if (publish) {
			// republish with total reset count and current timestamp
			global_position.lat_lon_reset_counter = _lat_lon_reset_counter;
			global_position.alt_reset_counter = _alt_reset_counter;
			global_position.delta_alt = _delta_alt_reset;

			global_position.timestamp = hrt_absolute_time();
			_vehicle_global_position_pub.publish(global_position);
		}
	}
}

void EKF2Selector::PublishWindEstimate()
{
	// selected estimator_wind -> wind
	wind_s wind;

	if (_instance[_selected_instance].estimator_wind_sub.update(&wind)) {
		bool publish = true;

		// ensure monotonically increasing timestamp_sample through reset, don't publish
		//  estimator's wind for system (wind) if it's stale
		if ((wind.timestamp_sample <= _wind_last.timestamp_sample)
		    || (hrt_elapsed_time(&wind.timestamp) > 100_ms)) {

			publish = false;
		}

		// save last primary wind
		_wind_last = wind;

		// publish estimator's wind for system unless it's stale
		if (publish) {
			// republish with current timestamp
			wind.timestamp = hrt_absolute_time();
			_wind_pub.publish(wind);
		}
	}
}

void EKF2Selector::Run()
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	// update combined test ratio for all estimators
	const bool updated = UpdateErrorScores();

	// if no valid instance then force select first instance with valid IMU
	if (_selected_instance == INVALID_INSTANCE) {
		for (uint8_t i = 0; i < EKF2_MAX_INSTANCES; i++) {
			if ((_instance[i].accel_device_id != 0)
			    && (_instance[i].gyro_device_id != 0)) {

				if (SelectInstance(i)) {
					break;
				}
			}
		}

		// if still invalid return early and check again on next scheduled run
		if (_selected_instance == INVALID_INSTANCE) {
			ScheduleDelayed(100_ms);
			return;
		}
	}

	if (updated) {
		const uint8_t available_instances_prev = _available_instances;
		const uint8_t selected_instance_prev = _selected_instance;
		const uint32_t instance_changed_count_prev = _instance_changed_count;
		const hrt_abstime last_instance_change_prev = _last_instance_change;

		bool lower_error_available = false;
		float alternative_error = 0.f; // looking for instances that have error lower than the current primary
		float best_test_ratio = FLT_MAX;

		uint8_t best_ekf = _selected_instance;
		uint8_t best_ekf_alternate = INVALID_INSTANCE;
		uint8_t best_ekf_different_imu = INVALID_INSTANCE;

		// loop through all available instances to find if an alternative is available
		for (int i = 0; i < _available_instances; i++) {
			// Use an alternative instance if  -
			// (healthy and has updated recently)
			// AND
			// (has relative error less than selected instance and has not been the selected instance for at least 10 seconds
			// OR
			// selected instance has stopped updating
			if (_instance[i].healthy.get_state() && (i != _selected_instance)) {
				const float test_ratio = _instance[i].combined_test_ratio;
				const float relative_error = _instance[i].relative_test_ratio;

				if (relative_error < alternative_error) {
					best_ekf_alternate = i;
					alternative_error = relative_error;

					// relative error less than selected instance and has not been the selected instance for at least 10 seconds
					if ((relative_error <= -_rel_err_thresh) && hrt_elapsed_time(&_instance[i].time_last_selected) > 10_s) {
						lower_error_available = true;
					}
				}

				if ((test_ratio > 0) && (test_ratio < best_test_ratio)) {
					best_ekf = i;
					best_test_ratio = test_ratio;

					// also check next best available ekf using a different IMU
					if (_instance[i].accel_device_id != _instance[_selected_instance].accel_device_id) {
						best_ekf_different_imu = i;
					}
				}
			}
		}

		if (!_instance[_selected_instance].healthy.get_state()) {
			// prefer the best healthy instance using a different IMU
			if (!SelectInstance(best_ekf_different_imu)) {
				// otherwise switch to the healthy instance with best overall test ratio
				SelectInstance(best_ekf);
			}

		} else if (lower_error_available
			   && ((hrt_elapsed_time(&_last_instance_change) > 10_s)
			       || (_instance[_selected_instance].warning
				   && (hrt_elapsed_time(&_instance[_selected_instance].time_last_no_warning) > 1_s)))) {

			// if this instance has a significantly lower relative error to the active primary, we consider it as a
			// better instance and would like to switch to it even if the current primary is healthy
			SelectInstance(best_ekf_alternate);

		} else if (_request_instance.load() != INVALID_INSTANCE) {

			const uint8_t new_instance = _request_instance.load();

			// attempt to switch to user manually selected instance
			if (!SelectInstance(new_instance)) {
				PX4_ERR("unable to switch to user selected instance %d", new_instance);
			}

			// reset
			_request_instance.store(INVALID_INSTANCE);
		}

		// publish selector status at ~1 Hz or immediately on any change
		if (_selector_status_publish || (hrt_elapsed_time(&_last_status_publish) > 1_s)
		    || (available_instances_prev != _available_instances)
		    || (selected_instance_prev != _selected_instance)
		    || (instance_changed_count_prev != _instance_changed_count)
		    || (last_instance_change_prev != _last_instance_change)
		    || _accel_fault_detected || _gyro_fault_detected) {

			PublishEstimatorSelectorStatus();
			_selector_status_publish = false;
		}
	}

	// republish selected estimator data for system
	PublishVehicleAttitude();
	PublishVehicleLocalPosition();
	PublishVehicleGlobalPosition();
	PublishVehicleOdometry();
	PublishWindEstimate();

	// re-schedule as backup timeout
	ScheduleDelayed(FILTER_UPDATE_PERIOD);
}

void EKF2Selector::PublishEstimatorSelectorStatus()
{
	estimator_selector_status_s selector_status{};
	selector_status.primary_instance = _selected_instance;
	selector_status.instances_available = _available_instances;
	selector_status.instance_changed_count = _instance_changed_count;
	selector_status.last_instance_change = _last_instance_change;
	selector_status.accel_device_id = _instance[_selected_instance].accel_device_id;
	selector_status.baro_device_id = _instance[_selected_instance].baro_device_id;
	selector_status.gyro_device_id = _instance[_selected_instance].gyro_device_id;
	selector_status.mag_device_id = _instance[_selected_instance].mag_device_id;
	selector_status.gyro_fault_detected = _gyro_fault_detected;
	selector_status.accel_fault_detected = _accel_fault_detected;

	for (int i = 0; i < EKF2_MAX_INSTANCES; i++) {
		selector_status.combined_test_ratio[i] = _instance[i].combined_test_ratio;
		selector_status.relative_test_ratio[i] = _instance[i].relative_test_ratio;
		selector_status.healthy[i] = _instance[i].healthy.get_state();
	}

	for (int i = 0; i < IMU_STATUS_SIZE; i++) {
		selector_status.accumulated_gyro_error[i] = _accumulated_gyro_error[i];
		selector_status.accumulated_accel_error[i] = _accumulated_accel_error[i];
	}

	selector_status.timestamp = hrt_absolute_time();
	_estimator_selector_status_pub.publish(selector_status);
	_last_status_publish = selector_status.timestamp;
}

void EKF2Selector::PrintStatus()
{
	PX4_INFO("available instances: %" PRIu8, _available_instances);

	if (_selected_instance == INVALID_INSTANCE) {
		PX4_WARN("selected instance: None");
	}

	for (int i = 0; i < _available_instances; i++) {
		const EstimatorInstance &inst = _instance[i];

		PX4_INFO("%" PRIu8 ": ACC: %" PRIu32 ", GYRO: %" PRIu32 ", MAG: %" PRIu32 ", %s, test ratio: %.7f (%.5f) %s",
			 inst.instance, inst.accel_device_id, inst.gyro_device_id, inst.mag_device_id,
			 inst.healthy.get_state() ? "healthy" : "unhealthy",
			 (double)inst.combined_test_ratio, (double)inst.relative_test_ratio,
			 (_selected_instance == i) ? "*" : "");
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef EKF2SELECTOR_HPP
#define EKF2SELECTOR_HPP

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/time.h>
#include <lib/hysteresis/hysteresis.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/sensors_status_imu.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/wind.h>

#if CONSTRAINED_MEMORY
# define EKF2_MAX_INSTANCES 2
#else
# define EKF2_MAX_INSTANCES 9
#endif

using namespace time_literals;

class EKF2Selector : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	EKF2Selector();
	~EKF2Selector() override;

	void Stop();

	void PrintStatus();

	void RequestInstance(uint8_t instance) { _request_instance.store(instance); }

private:
	static constexpr uint8_t INVALID_INSTANCE{UINT8_MAX};
	static constexpr uint64_t FILTER_UPDATE_PERIOD{10_ms};

	void Run() override;

	void PrintInstanceChange(const uint8_t old_instance, uint8_t new_instance);

	void PublishEstimatorSelectorStatus();
	void PublishVehicleAttitude();
	void PublishVehicleLocalPosition();
	void PublishVehicleGlobalPosition();
	void PublishVehicleOdometry();
	void PublishWindEstimate();

	bool SelectInstance(uint8_t instance);

	// Update the error scores for all available instances
	bool UpdateErrorScores();

	// Subscriptions (per estimator instance)
	struct EstimatorInstance {

		EstimatorInstance(EKF2Selector *selector, uint8_t i) :
			estimator_attitude_sub{selector, ORB_ID(estimator_attitude), i},
			estimator_status_sub{selector, ORB_ID(estimator_status), i},
			estimator_local_position_sub{ORB_ID(estimator_local_position), i},
			estimator_global_position_sub{ORB_ID(estimator_global_position), i},
			estimator_odometry_sub{ORB_ID(estimator_odometry), i},
			estimator_wind_sub{ORB_ID(estimator_wind), i},
			instance(i)
		{
			healthy.set_hysteresis_time_from(false, 1_s);
		}

		uORB::SubscriptionCallbackWorkItem estimator_attitude_sub;
		uORB::SubscriptionCallbackWorkItem estimator_status_sub;

		uORB::Subscription estimator_local_position_sub;
		uORB::Subscription estimator_global_position_sub;
		uORB::Subscription estimator_odometry_sub;
		uORB::Subscription estimator_wind_sub;

		uint64_t timestamp_last{0};

		uint32_t accel_device_id{0};
		uint32_t gyro_device_id{0};
		uint32_t baro_device_id{0};
		uint32_t mag_device_id{0};

		hrt_abstime time_last_selected{0};
		hrt_abstime time_last_no_warning{0};

		float combined_test_ratio{NAN};
		float relative_test_ratio{NAN};

		systemlib::Hysteresis healthy{false};

		bool warning{false};
		bool filter_fault{false};
		bool timeout{false};

		uint8_t healthy_count{0};

		const uint8_t instance;
	};

	static constexpr float _rel_err_score_lim{1.0f}; // +- limit applied to the relative error score
	static constexpr float _rel_err_thresh{0.5f};    // the relative score difference needs to be greater than this to switch from an otherwise healthy instance

	EstimatorInstance _instance[EKF2_MAX_INSTANCES] {
		{this, 0},
		{this, 1},
#if EKF2_MAX_INSTANCES > 2
		{this, 2},
		{this, 3},
#if EKF2_MAX_INSTANCES > 4
		{this, 4},
		{this, 5},
		{this, 6},
		{this, 7},
		{this, 8},
#endif
#endif
	};

	static constexpr uint8_t IMU_STATUS_SIZE = (sizeof(sensors_status_imu_s::gyro_inconsistency_rad_s) / sizeof(
				sensors_status_imu_s::gyro_inconsistency_rad_s[0]));
	static_assert(IMU_STATUS_SIZE == sizeof(estimator_selector_status_s::accumulated_gyro_error) / sizeof(
			      estimator_selector_status_s::accumulated_gyro_error[0]),
		      "increase estimator_selector_status_s::accumulated_gyro_error size");
	static_assert(IMU_STATUS_SIZE == sizeof(estimator_selector_status_s::accumulated_accel_error) / sizeof(
			      estimator_selector_status_s::accumulated_accel_error[0]),
		      "increase estimator_selector_status_s::accumulated_accel_error size");
	static_assert(EKF2_MAX_INSTANCES <= sizeof(estimator_selector_status_s::combined_test_ratio) / sizeof(
			      estimator_selector_status_s::combined_test_ratio[0]),
		      "increase estimator_selector_status_s::combined_test_ratio size");

	float _accumulated_gyro_error[IMU_STATUS_SIZE] {};
	float _accumulated_accel_error[IMU_STATUS_SIZE] {};
	hrt_abstime _last_update_us{0};
	bool _gyro_fault_detected{false};
	bool _accel_fault_detected{false};

	uint8_t _available_instances{0};
	uint8_t _selected_instance{INVALID_INSTANCE};
	px4::atomic<uint8_t> _request_instance{INVALID_INSTANCE};

	uint32_t _instance_changed_count{0};
	hrt_abstime _last_instance_change{0};

	hrt_abstime _last_status_publish{0};
	bool _selector_status_publish{false};

	// vehicle_attitude: reset counters
	vehicle_attitude_s _attitude_last{};
	matrix::Quatf _delta_q_reset{};
	uint8_t _quat_reset_counter{0};

	// vehicle_local_position: reset counters
	vehicle_local_position_s _local_position_last{};
	matrix::Vector2f _delta_xy_reset{};
	float _delta_z_reset{0.f};
	matrix::Vector2f _delta_vxy_reset{};
	float _delta_vz_reset{0.f};
	float _delta_heading_reset{0};
	uint8_t _xy_reset_counter{0};
	uint8_t _z_reset_counter{0};
	uint8_t _vxy_reset_counter{0};
	uint8_t _vz_reset_counter{0};
	uint8_t _heading_reset_counter{0};

	// vehicle_odometry
	vehicle_odometry_s _odometry_last{};
	uint8_t _odometry_reset_counter{0};

	// vehicle_global_position: reset counters
	vehicle_global_position_s _global_position_last{};
	double _delta_lat_reset{0};
	double _delta_lon_reset{0};
	float _delta_alt_reset{0.f};
	uint8_t _lat_lon_reset_counter{0};
	uint8_t _alt_reset_counter{0};

	// wind estimate
	wind_s _wind_last{};

	uint8_t _attitude_instance_prev{INVALID_INSTANCE};
	uint8_t _local_position_instance_prev{INVALID_INSTANCE};
	uint8_t _global_position_instance_prev{INVALID_INSTANCE};
	uint8_t _odometry_instance_prev{INVALID_INSTANCE};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _sensors_status_imu{ORB_ID(sensors_status_imu)};

	// Publications
	uORB::Publication<estimator_selector_status_s> _estimator_selector_status_pub{ORB_ID(estimator_selector_status)};
	uORB::Publication<sensor_selection_s>          _sensor_selection_pub{ORB_ID(sensor_selection)};
	uORB::Publication<vehicle_attitude_s>          _vehicle_attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_global_position_s>   _vehicle_global_position_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_local_position_s>    _vehicle_local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<vehicle_odometry_s>          _vehicle_odometry_pub{ORB_ID(vehicle_odometry)};
	uORB::Publication<wind_s>             _wind_pub{ORB_ID(wind)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::EKF2_SEL_ERR_RED>) _param_ekf2_sel_err_red,
		(ParamFloat<px4::params::EKF2_SEL_IMU_RAT>) _param_ekf2_sel_imu_angle_rate,
		(ParamFloat<px4::params::EKF2_SEL_IMU_ANG>) _param_ekf2_sel_imu_angle,
		(ParamFloat<px4::params::EKF2_SEL_IMU_ACC>) _param_ekf2_sel_imu_accel,
		(ParamFloat<px4::params::EKF2_SEL_IMU_VEL>) _param_ekf2_sel_imu_velocity
	)
};
#endif // !EKF2SELECTOR_HPP
