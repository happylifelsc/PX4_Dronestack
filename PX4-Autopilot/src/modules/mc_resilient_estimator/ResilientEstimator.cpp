
#include "ResilientEstimator.hpp"
#include <drivers/drv_hrt.h> // added by zyl
#include <px4_platform_common/events.h> // added by zyl
#include <cmath> // Ensure this is included for powf function
using namespace matrix;


// Define variables for time measurement LUO
hrt_abstime start_time, end_time;
uint64_t execution_time;

//------------------------------------------Parameter Initializations------------------------------------------//


bool ekf_initialized = false; // Flag to indicate whether the EKF object has been created
ResilientEKF* curr_ekf; // variable for EKF object;


constexpr float mass = 2.65;
constexpr float ixx = 0.0257;
constexpr float iyy = 0.0258;
constexpr float izz = 0.044;
constexpr float im = 5.2999e-05;
constexpr float d = 0.246;
constexpr float c_t = 1.239e-05;
constexpr float c_m = 1.982e-07;

// Define the macro
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define ABS(x) ((x) < 0 ? -(x) : (x))

// Inline function for power calculation
template<typename T>
inline T pow(T base, int exp) {
    T result = 1;
    while (exp) {
        if (exp % 2) result *= base;
        exp /= 2;
        base *= base;
    }
    return result;
}

struct TorqueAndThrust {
    matrix::Vector3f torque;
    matrix::Vector3f thrust;
};


// notice: _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")) added by zyl
ResilientEstimator::ResilientEstimator() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
}

ResilientEstimator::~ResilientEstimator()
{
    perf_free(_loop_perf);
}

bool ResilientEstimator::init()
{
	// The problem is here, I think this returns false, and it will disconnect the serial communication if you run this and comment out the
	if (!_vehicle_attitude_sub.registerCallback())
	{
		PX4_ERR("vehicle_attitude callback registration failed!");
		return false;
	}

	PX4_INFO("Resilient estimator module started");

	return true;
}

void ResilientEstimator::Run()
{
	if (should_exit())
	{
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Record the start time LUO
    	start_time = hrt_absolute_time();

	PX4_INFO("resilient estimator is running");


	// static auto x_hat_prev = new matrix::Vector<float, 12>();
	// static auto x_cond_prev = new matrix::Vector<float, 12>();
	// static auto P_prev = new matrix::Matrix<float, 12, 12>();
	// static auto U_prev = new matrix::Vector<float, 4>();
	static auto x_output = new matrix::Vector<float, 12>();

	//----------------------------------------------Time interval----------------------------------------------//
	//------------------------------------------dt = 0.004(250hz)------------------------------------------//
	// Use angular velocity header stamp to record time
	static hrt_abstime _last_run{0};
	vehicle_angular_velocity_s angular_velocity;
	_vehicle_angular_velocity_sub.copy(&angular_velocity);
	const hrt_abstime now = angular_velocity.timestamp_sample;
	const double dt_calculated = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
	_last_run = now;
	double current_time = now / 1e6;
	// PX4_INFO ("Current time: %f",current_time);
	// PX4_INFO ("Time interval dt calculated:: %.3f",dt_calculated);

	//----------------------------------------------Previous Step Values----------------------------------------------//
	static matrix::Vector3f gps_velocity{0.0f, 0.0f, 0.0f};

	//-------------------------------------------------------------------------------------------------------------------------------------------------//
	//----------------------------------------------------------------Resilient Control----------------------------------------------------------------//
	//-------------------------------------------------------------------------------------------------------------------------------------------------//

	//----------------------------------GPS and Compass Data from VICON----------------------------------//
	static hrt_abstime visual_odom_last_run{0};
	vehicle_odometry_s visual_odom;

	bool visual_odom_update_flag = 0;
	if (_visual_odometry_sub.update(&visual_odom)) {


		const hrt_abstime visual_odom_last_run_now = visual_odom.timestamp_sample;
		const double visual_odom_dt_calculated = (visual_odom_last_run_now - visual_odom_last_run) * 1e-6f;
		visual_odom_last_run = visual_odom_last_run_now;

		visual_odom_update_flag = 1;
		PX4_INFO("ROS visual odometry: Delta_t (dt: %f) Position (X: %f, Y: %f, Z: %f); Quaternion (q_w: %f, q_x: %f, q_y: %f, q_z: %f)",
			(double)visual_odom_dt_calculated, (double)visual_odom.position[0], (double)visual_odom.position[1], (double)visual_odom.position[2],
			(double)visual_odom.q[0], (double)visual_odom.q[1], (double)visual_odom.q[2], (double)visual_odom.q[3]);

	}

	//----------------------------------Torques and thrusts estimation----------------------------------//
	//https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=5509980

	matrix::Vector4f w_gazebo{0.0, 0.0, 0.0, 0.0};
	for (int i = 0; i < _rpm_subs.size() && i < 4; i++) {
		rpm_s rpm;
		if(_rpm_subs[i].update(&rpm)) {
			// Eigen::Vector4d w_gazebo{rotor_speeds_from_q.q[0], rotor_speeds_from_q.q[1], rotor_speeds_from_q.q[2], rotor_speeds_from_q.q[3]};
			// Eigen::Vector3d v_gazeno_enu{gps_velocity(1), gps_velocity(0), -1 * gps_velocity(2)};
			// Eigen::Vector3d v_wind_gazebo{0.0, 0.0, 0.0};

			w_gazebo(i) =rpm.indicated_frequency_rpm;


		}
	}



	matrix::Vector3f v_gazeno_enu{(visual_odom.position[0] - prev_visual_odom.position[0]) / (float)dt_calculated, (visual_odom.position[1] - prev_visual_odom.position[1]) / (float)dt_calculated, (visual_odom.position[2] - prev_visual_odom.position[2]) / (float)dt_calculated};

	TorqueAndThrust torque_and_thrust_estimate_result = ResilientEstimator::calculate_individual_torques_and_thrust(w_gazebo, v_gazeno_enu);


	matrix::Vector3f gazebo_cal_force{0.0, 0.0, 0.0};
	matrix::Vector3f gazebo_cal_torque{0.0, 0.0, 0.0};

	gazebo_cal_force = torque_and_thrust_estimate_result.thrust;
	gazebo_cal_torque = torque_and_thrust_estimate_result.torque;

	// // Rotating from FLU body frame to FRD
	// gazebo_cal_torque(1) = -1 * gazebo_cal_torque(1);
	// gazebo_cal_torque(2) = -1 * gazebo_cal_torque(2);
	// // gazebo_cal_force = -1 * gazebo_cal_force;


	//----------------------------------Incorporate the data----------------------------------//
	// Define the 16-dimensional vector
	matrix::Vector<float, 16>* curr_data = new matrix::Vector<float, 16>();

	// Convert quaternion to yaw angle
	matrix::Vector3f* euler_from_visual = new matrix::Vector3f();
	*euler_from_visual = ResilientEstimator::quaternionToEuler(visual_odom.q[0], visual_odom.q[1], visual_odom.q[2], visual_odom.q[3]);

	// Assign values to curr_data vector
	(*curr_data)(0) = 0; // roll
	(*curr_data)(1) = 0; // pitch
	(*curr_data)(2) = (*euler_from_visual)(2); // yaw
	(*curr_data)(3) = 0; // roll_rate
	(*curr_data)(4) = 0; // pitch_rate
	(*curr_data)(5) = 0; // yaw_rate
	(*curr_data)(6) = visual_odom.position[0]; // x
	(*curr_data)(7) = visual_odom.position[1]; // y
	(*curr_data)(8) = visual_odom.position[2]; // z
	(*curr_data)(9) = 0;
	(*curr_data)(10) = 0;
	(*curr_data)(11) = 0;

	// Control inputs
	(*curr_data)(12) = gazebo_cal_torque(0);// torque_x
	(*curr_data)(13) = gazebo_cal_torque(1);  // torque_y
	(*curr_data)(14) = gazebo_cal_torque(2); // torque_z
	(*curr_data)(15) = gazebo_cal_force(2); // thrust



	//----------------------------------Resilient EKF Parameters----------------------------------//
	double dt = dt_calculated;
	int n_states = 12; // Number of states
	int n_control = 4; // Number of control inputs
	int n_obs = 7; // Number of observations


	//----------------------------------Initialize Resilient EKF----------------------------------//
	// delete: *obs_matrix, *process_noise_matrix, *obs_noise_matrix,



	if (!ekf_initialized) {
		curr_ekf = new ResilientEKF(*curr_data, dt, n_states, n_control, n_obs, mass, ixx, iyy, izz, im, d, c_t, c_m);
		ekf_initialized = true;
	}
	else {
		curr_ekf->data = *curr_data;
		curr_ekf->dt = dt;
	}

	*x_output = curr_ekf->run(visual_odom_update_flag);

	PX4_INFO("%f", (double)curr_ekf->x_hat(0));
	// // // Update previous data
	// curr_ekf->update_previous_data(*x_hat_prev, *x_cond_prev, *U_prev, *P_prev);

	// // // Calculate priors
	// curr_ekf->calculate_priors();

	// // Determine whether to do estimation or not
	// //// GPS is updated, calculate posteriors
	// if (visual_odom_update_flag){
	// 	curr_ekf->calculate_posteriors();
	// 	*x_output = curr_ekf->x_hat;
	// 	*x_hat_prev = curr_ekf->x_hat;
	// 	*P_prev = curr_ekf->P_k;
	// }

	// //// GPS is not updated, calculate priors only
	// else if (!visual_odom_update_flag){
	// 	*x_output = curr_ekf->x_hat_cond;
	// 	*x_hat_prev = curr_ekf->x_hat_cond;
	// 	*P_prev = curr_ekf->P_cond;
	// }

	// // Update next previous control input and prediction
	// *x_cond_prev = curr_ekf->x_hat_cond;
	// *U_prev = curr_ekf->U;

	// Update the next previous visual_odom
	// prev_visual_odom = visual_odom;


	quaternion_output = ResilientEstimator::eulerToQuaternionVector((*x_output)(0), (*x_output)(1), (*x_output)(2));


	// Filter angular velocity estimate before sending out
	// discard: // Vector3f angular_velocity_output_filtered{_lp_filter_velocity.apply(angular_velocity_output_raw)};

	angular_velocity_output_raw = {static_cast<float>((*x_output)(3)), static_cast<float>((*x_output)(4)), static_cast<float>((*x_output)(5))};
	angular_velocity_output_filtered = {angular_velocity_output_raw(0), angular_velocity_output_raw(1), angular_velocity_output_raw(2)};

	// Calculate angular acceleration and send out
	robust_angular_acc.timestamp = hrt_absolute_time();


	angular_velocity_output_filtered_vector = {angular_velocity_output_filtered(0), angular_velocity_output_filtered(1), angular_velocity_output_filtered(2)};
	torque_vector = {gazebo_cal_torque(0), gazebo_cal_torque(1), gazebo_cal_torque(2)};

	// Calculate the robust angular acceleration vector using the updated function
	robust_angular_acc_vector = ResilientEstimator::calculate_angular_acc_from_dynamics(angular_velocity_output_filtered_vector, torque_vector);


	//-------------------------------------------------------------------------------------------------------------------------------------------------//
	//--------------------------------------------------------Construct Robust Attitude Message--------------------------------------------------------//
	//-------------------------------------------------------------------------------------------------------------------------------------------------//
	// Gather ground truth data for comparison
	imu_attack_s imu_attack;
	if (_imu_attack_sub.update(&imu_attack)){

	}

	// Optional
	sensor_accel_s raw_accel;
	if (_sensor_accel_sub.update(&raw_accel)){

	}

	sensor_gyro_s raw_gyro;
	if (_sensor_gyro_sub.update(&raw_gyro)){

	}


	vehicle_attitude_s v_att;
	if (_vehicle_attitude_sub.update(&v_att)) {
		// std::cout << "Ground truth q_0 " << v_att.q[0] << std::endl;
		// std::cout << "Ground truth q_1 " << v_att.q[1] << std::endl;
		// std::cout << "Ground truth q_2 " << v_att.q[2] << std::endl;
		// std::cout << "Ground truth q_3 " << v_att.q[3] << std::endl;
	}

	// Publish the robust_attitude message
	robust_att.timestamp = hrt_absolute_time();


	constexpr int resilient_control_level = 3;
	// Standard control (PX4 LPF)
	if (resilient_control_level == 0)
	{
		robust_att.orientation[0] = v_att.q[0];
		robust_att.orientation[1] = v_att.q[1];
		robust_att.orientation[2] = v_att.q[2];
		robust_att.orientation[3] = v_att.q[3];

		robust_att.body_rate[0] = angular_velocity.xyz[0];
		robust_att.body_rate[1] = angular_velocity.xyz[1];
		robust_att.body_rate[2] = angular_velocity.xyz[2];

	}

	// Our Resilient controller
	else if (resilient_control_level == 3)
	{
		robust_att.orientation[0] = quaternion_output(0);
		robust_att.orientation[1] = quaternion_output(1);
		robust_att.orientation[2] = quaternion_output(2);
		robust_att.orientation[3] = quaternion_output(3);

		robust_att.body_rate[0] = angular_velocity_output_raw(0);
		robust_att.body_rate[1] = angular_velocity_output_raw(1);
		robust_att.body_rate[2] = angular_velocity_output_raw(2);

		robust_angular_acc.angular_acc[0] = robust_angular_acc_vector(0);
		robust_angular_acc.angular_acc[1] = robust_angular_acc_vector(1);
		robust_angular_acc.angular_acc[2] = robust_angular_acc_vector(2);

	}

	// Print for skipping the unused variable checking
	PX4_INFO("%f", (double)robust_att.timestamp);
	PX4_INFO("%f", (double)robust_angular_acc.timestamp);

	//-------------------------------------------------------------------------------------------------------------------------------------------------//
	//----------------------------------------------------------------Anomaly Detection----------------------------------------------------------------//
	//-------------------------------------------------------------------------------------------------------------------------------------------------//
	// Comparing the estimate redisual
	angular_velocity_estimate_residuals(0) = angular_velocity_output_filtered(0) - angular_velocity.xyz[0];
	angular_velocity_estimate_residuals(1) = angular_velocity_output_filtered(1) - angular_velocity.xyz[1];
	angular_velocity_estimate_residuals(2) = angular_velocity_output_filtered(2) - angular_velocity.xyz[2];
	double angular_velocity_estimate_residual_norm = angular_velocity_estimate_residuals.norm();
	// std::cout << "Estimate residual norm: " << angular_velocity_estimate_residual_norm << std::endl;

	// CUSUM Statistics
	// CUSUM and threshold settings
	// 1. Hovering
	// threshold = 1.4, k = 0.8, P_td = 0.005

	// 2. Moving
	// threshold = 1.4, k = 1.5, P_td = 0.005

	static double cusum = 0.0;
	static double detection_rate = 0.0;
	constexpr double reference_value = 0.0;
	constexpr double threshold_standard = 1.4;
	constexpr double k_value = 1.5;


	// CUSUM window: optional

	double cusum_deviation = angular_velocity_estimate_residual_norm - reference_value;
	PX4_INFO("CUSUM deviation: %.2f", cusum_deviation);

	// Update CUSUM statistics
	cusum += (cusum_deviation - k_value);

	if (cusum < 0){
		cusum = 0;
	}

	cusum = MAX(0.0, cusum);

	if (cusum > threshold_standard) {
		ResilientEstimator::SlidingWindowResult detection_result = ResilientEstimator::sliding_window_detection(cusum_window, 1000, 1);
		cusum_window = *detection_result.window;
		detection_rate = detection_result.anomaly_percentage;
		cusum = 0;
	}
	else {
		ResilientEstimator::SlidingWindowResult detection_result = ResilientEstimator::sliding_window_detection(cusum_window, 1000, 0);
		cusum_window = *detection_result.window;
		detection_rate = detection_result.anomaly_percentage;
	}

	PX4_INFO("Detection rate: %.2f", detection_rate);
	// PX4_INFO("CUSUM value: %.2f", cus_resilient_state_estimate_pubum);

	// std::cout << "Estimate residual wx: " << angular_velocity_output_filtered(0) - angular_velocity.xyz[0] << std::endl;
	// std::cout << "Estimate residual wy: " << angular_velocity_output_filtered(1) - angular_velocity.xyz[1] << std::endl;
	// std::cout << "Estimate residual wz: " << angular_velocity_output_filtered(2) - angular_velocity.xyz[2] << std::endl;

	//----------------------------------------------Control Mode Switch---------------------------------------------//
	anomaly_detection_msg.timestamp = hrt_absolute_time();
	if (detection_rate > 0.00 && current_time > 23.0)
	{
		anomaly_detection_msg.anomaly_flag = 0;
		// PX4_INFO("Anomaly detected!");
	}
	else{
		anomaly_detection_msg.anomaly_flag = 0;
	}

	_anomaly_detection_pub.publish(anomaly_detection_msg);

	// ----------------------------------------------Publish to Attitude Controller---------------------------------------------//
	if (!anomaly_detection_msg.anomaly_flag)
	{
		// PX4_INFO("Standard control");
	}
	else
	{
		// // PX4_INFO("Resilient control activated");
		_robust_attitude_pub.publish(robust_att); // sending this message can lead to memory overflow
		_robust_angular_acc_pub.publish(robust_angular_acc);
	}


	//-------------------------------------------------------------------------------------------------------------------------------------------------//
	//------------------------------------------------------Logging Resilient State Estimate Data------------------------------------------------------//
	//-------------------------------------------------------------------------------------------------------------------------------------------------//

	resilient_state_estimate_msg.timestamp = hrt_absolute_time();

	// Gazebo rotor speed, torques and thrust
	resilient_state_estimate_msg.rotor_speed[0] = w_gazebo(0);
	resilient_state_estimate_msg.rotor_speed[1] = w_gazebo(1);
	resilient_state_estimate_msg.rotor_speed[2] = w_gazebo(2);
	resilient_state_estimate_msg.rotor_speed[3] = w_gazebo(3);

	// // do not know where to get torques ground truth;
	// resilient_state_estimate_msg.torque_ground_truth[0] = torques_from_gazebo.xyz[0];
	// resilient_state_estimate_msg.torque_ground_truth[1] = torques_from_gazebo.xyz[1];
	// resilient_state_estimate_msg.torque_ground_truth[2] = torques_from_gazebo.xyz[2];

	// add by me
	vehicle_global_position_s vehicle_global_position;
	_gpos_ground_truth_sub.update(&vehicle_global_position);

	vehicle_acceleration_s vehicle_acceleration;
	_vehicle_acceleration_sub.update(&vehicle_acceleration);

	resilient_state_estimate_msg.thrust_ground_truth = vehicle_global_position.alt;

	// Raw IMU measurements
	resilient_state_estimate_msg.raw_angular_vel[0] = raw_gyro.x;
	resilient_state_estimate_msg.raw_angular_vel[1] = raw_gyro.y;
	resilient_state_estimate_msg.raw_angular_vel[2] = raw_gyro.z;

	resilient_state_estimate_msg.raw_acc[0] = raw_accel.x;
	resilient_state_estimate_msg.raw_acc[1] = raw_accel.y;
	resilient_state_estimate_msg.raw_acc[2] = raw_accel.z;

	// PX4 filtered IMU measurements
	resilient_state_estimate_msg.imu_angular_vel[0] = angular_velocity.xyz[0];
	resilient_state_estimate_msg.imu_angular_vel[1] = angular_velocity.xyz[1];
	resilient_state_estimate_msg.imu_angular_vel[2] = angular_velocity.xyz[2];

	resilient_state_estimate_msg.imu_acc[0] = vehicle_acceleration.xyz[0];
	resilient_state_estimate_msg.imu_acc[1] = vehicle_acceleration.xyz[1];
	resilient_state_estimate_msg.imu_acc[2] = vehicle_acceleration.xyz[2];

	// Standard estimates
	resilient_state_estimate_msg.attitude_standard[0] =  v_att.q[0];
	resilient_state_estimate_msg.attitude_standard[1] =  v_att.q[1];
	resilient_state_estimate_msg.attitude_standard[2] =  v_att.q[2];
	resilient_state_estimate_msg.attitude_standard[3] =  v_att.q[3];

	// Resilient estimates
	resilient_state_estimate_msg.torque_estimate[0] = gazebo_cal_torque(0);
	resilient_state_estimate_msg.torque_estimate[1] = gazebo_cal_torque(1);
	resilient_state_estimate_msg.torque_estimate[2] = gazebo_cal_torque(2);
	resilient_state_estimate_msg.thrust_estimate = gazebo_cal_force(2);

	resilient_state_estimate_msg.attitude_estimate[0] = robust_att.orientation[0];
	resilient_state_estimate_msg.attitude_estimate[1] = robust_att.orientation[1];
	resilient_state_estimate_msg.attitude_estimate[2] = robust_att.orientation[2];
	resilient_state_estimate_msg.attitude_estimate[3] = robust_att.orientation[3];

	resilient_state_estimate_msg.angular_vel_estimate[0] = robust_att.body_rate[0];
	resilient_state_estimate_msg.angular_vel_estimate[1] = robust_att.body_rate[1];
	resilient_state_estimate_msg.angular_vel_estimate[2] = robust_att.body_rate[2];

	// Anomaly detection
	resilient_state_estimate_msg.anomaly_score = cusum_deviation;
	resilient_state_estimate_msg.anomaly_detection_rate = detection_rate;
	resilient_state_estimate_msg.anomaly_flag = anomaly_detection_msg.anomaly_flag;

	// Attack information
	resilient_state_estimate_msg.imu_acc_offset[0] = imu_attack.x_acc_offset;
	resilient_state_estimate_msg.imu_acc_offset[1] = imu_attack.y_acc_offset;
	resilient_state_estimate_msg.imu_acc_offset[2] = imu_attack.z_acc_offset;

	resilient_state_estimate_msg.imu_gyro_offset[0] = imu_attack.x_gyro_offset;
	resilient_state_estimate_msg.imu_gyro_offset[1] = imu_attack.y_gyro_offset;
	resilient_state_estimate_msg.imu_gyro_offset[2] = imu_attack.z_gyro_offset;

	resilient_state_estimate_msg.attack_flag = imu_attack.attack_flag;

	// Publish resilient estimate message

	PX4_INFO("check resilient_state_estimate_msg: %.2f", (double)resilient_state_estimate_msg.imu_acc[0]);
	_resilient_state_estimate_pub.publish(resilient_state_estimate_msg);

	// delete x_hat_prev;
	// delete x_cond_prev;
	// delete P_prev;
	// delete U_prev;
	// delete x_output;
	delete euler_from_visual;
	// delete obs_matrix;
	// delete noise_identity_matrix;
	// delete process_noise_matrix;
	// delete obs_noise_matrix;
	delete curr_data;
	delete curr_ekf;


	// Record the end time LUO
    	end_time = hrt_absolute_time();

	 // Calculate the execution time LUO
    	execution_time = end_time - start_time;

    	// Log the execution time LUO
    	PX4_INFO("Execution time: %" PRIu64 " us", execution_time);

	perf_end(_loop_perf);
}


ResilientEstimator::SlidingWindowResult ResilientEstimator::sliding_window_detection(matrix::Vector<int, 1000>& window, size_t window_size, int new_score) {
    // Update the window with the new score without shifting
    // Circular buffer technique to maintain constant time complexity
    static size_t start_index = 0;

    window(start_index) = new_score;

    // Move the start index forward and wrap around if necessary
    start_index = (start_index + 1) % window_size;

    // Calculate the percentage of anomalies (assuming anomalies are represented by the score of 1)
    int count_anomalies = 0;
    for (size_t i = 0; i < window_size; i++) {
        if (window(i) == 1) {
            count_anomalies++;
        }
    }

    double anomaly_percentage = static_cast<double>(count_anomalies) / window_size;

    // Return the updated window reference and the anomaly percentage
    SlidingWindowResult result;
    result.window = &window; // Return reference instead of a new copy
    result.anomaly_percentage = anomaly_percentage;
    return result;
}


matrix::Vector3f ResilientEstimator::quaternionToEuler(double q0, double q1, double q2, double q3) {
    matrix::Vector3f euler;

    // Roll (phi)
    double sinr_cosp = 2 * (q0 * q1 + q2 * q3);
    double cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2);
    euler(0) = std::atan2(sinr_cosp, cosr_cosp); // Roll

    // Pitch (theta)
    double sinp = 2 * (q0 * q2 - q3 * q1);
    if (std::abs(sinp) >= 1)
        euler(1) = copysign(M_PI / 2, sinp); // Use pi/2 if out of range
    else
        euler(1) = std::asin(sinp); // Pitch

    // Yaw (psi)
    double siny_cosp = 2 * (q0 * q3 + q1 * q2);
    double cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
    euler(2) = std::atan2(siny_cosp, cosy_cosp); // Yaw

    return euler;
}

matrix::Vector4f ResilientEstimator::eulerToQuaternionVector(double roll, double pitch, double yaw) {
    // Create and normalize the quaternion from Euler angles (roll, pitch, yaw)
    q = matrix::Quatf(matrix::Eulerf(roll, pitch, yaw)).normalized();

    // Update the static quaternion with the new values
    quaternionVector(0) = q(0);
    quaternionVector(1) = q(1);
    quaternionVector(2) = q(2);
    quaternionVector(3) = q(3);

    return quaternionVector;
}

matrix::Vector3f ResilientEstimator::calculate_angular_acc_from_dynamics(const matrix::Vector3f& angular_vels, const matrix::Vector3f& torques) {

    constexpr float Ixx = 0.011f;
    constexpr float Iyy = 0.015f;
    constexpr float Izz = 0.021f;
    constexpr float Im = 5.7823e-06f;
    constexpr float a1 = (Iyy - Izz) / Ixx;
    constexpr float a2 = (Izz - Ixx) / Iyy;
    constexpr float a3 = (Ixx - Iyy) / Izz;
    constexpr float b1 = 1.0f / Ixx;
    constexpr float b2 = 1.0f / Iyy;
    constexpr float b3 = 1.0f / Izz;

    matrix::Vector3f angular_accs;
    angular_accs(0) = a1 * angular_vels(1) * angular_vels(2) - Im * 10.0f * angular_vels(1) + b1 * torques(0);
    angular_accs(1) = a2 * angular_vels(0) * angular_vels(2) + Im * 10.0f * angular_vels(0) + b2 * torques(1);
    angular_accs(2) = a3 * angular_vels(0) * angular_vels(1) + b3 * torques(2);

    return angular_accs;
}




// Function to calculate individual torques and thrust
ResilientEstimator::TorqueAndThrust ResilientEstimator::calculate_individual_torques_and_thrust(matrix::Vector4f& rotor_speeds, matrix::Vector3f& velocity) {



    //--------------------------------------------------Rotor parameters--------------------------------------------------//
    constexpr double motor_constant = 8.54858e-6;
    constexpr double rotor_drag_constant = 0.000806428;
    constexpr double moment_constant = 0.06;
    constexpr double rolling_moment_constant = 1e-6;
    static const matrix::Vector3f joint_axis(0, 0, 1);

    // CAi (Rotor positions)
    static const matrix::Vector3f ca_0(0.174, -0.174, 0.06);
    static const matrix::Vector3f ca_1(-0.174, 0.174, 0.06);
    static const matrix::Vector3f ca_2(0.174, 0.174, 0.06);
    static const matrix::Vector3f ca_3(-0.174, -0.174, 0.06);

    //--------------------------------------------------Rotor forces--------------------------------------------------//
    matrix::Vector3f rotor_forces[4];
    matrix::Vector3f rotor_torques[4];

    double body_velocity_norm = velocity.norm();
    double force_scaler = 1 - body_velocity_norm / 25;
    force_scaler = MAX(0.0, MIN(1.0, force_scaler));

    for (int i = 0; i < 4; ++i) {
        rotor_forces[i] = matrix::Vector3f(0, 0, (double)powf(rotor_speeds(i), 2) * (double)motor_constant * (double)force_scaler);
    }

    //--------------------------------------------------Air drag forces and torques--------------------------------------------------//
    matrix::Vector3f relative_velocity = velocity;
    matrix::Vector3f relative_velocity_perpendicular = relative_velocity - (relative_velocity.dot(joint_axis) * joint_axis);

    matrix::Vector3f drag_forces[4];
    matrix::Vector3f drag_torques[4];

    for (int i = 0; i < 4; ++i) {
        drag_forces[i] = -ABS(rotor_speeds(i)) * relative_velocity_perpendicular * rotor_drag_constant;
        drag_torques[i](2) = (i < 2 ? -1 : 1) * (double)rotor_forces[i](2) * (double)moment_constant;  // Assuming rotors 0 and 1 are CCW, 2 and 3 are CW
    }

    //--------------------------------------------------Rotor torques--------------------------------------------------//
    // Calculate rotor torques (CA x Fi)
    rotor_torques[0] = ca_0.cross(rotor_forces[0] + drag_forces[0]);
    rotor_torques[1] = ca_1.cross(rotor_forces[1] + drag_forces[1]);
    rotor_torques[2] = ca_2.cross(rotor_forces[2] + drag_forces[2]);
    rotor_torques[3] = ca_3.cross(rotor_forces[3] + drag_forces[3]);

    //--------------------------------------------------Rolling moment effects--------------------------------------------------//
    matrix::Vector3f rolling_moments[4];
    for (int i = 0; i < 4; ++i) {
        rolling_moments[i] = -ABS(rotor_speeds(i)) * relative_velocity_perpendicular * rolling_moment_constant;
    }

    //--------------------------------------------------Collective force and torques--------------------------------------------------//
    // \tau_C = sum[(CAi x Fi) + Mi]
    matrix::Vector3f collective_force = matrix::Vector3f();
    matrix::Vector3f collective_torque = matrix::Vector3f();

    for (int i = 0; i < 4; ++i) {
        collective_force += rotor_forces[i] + drag_forces[i];
        collective_torque += rotor_torques[i] + drag_torques[i] + rolling_moments[i];
    }

    //--------------------------------------------------Compensate the torques--------------------------------------------------//
// // discard for now
//     double torque_x_bias = 0; // For hovering
//     double torque_y_bias = 0.014 + 0.02 * velocity(0); // For hovering
//     // double torque_y_bias = 0.010 + 0.025 * velocity(0); // For moving at 0.5m/s
//     // double torque_y_bias = 0.010 + 0.025 * velocity(0); // For moving at 1.0m/s
//     std::cout << "Velocity x: " << velocity(0) << std::endl;
//     std::cout << "Adaptive Y-axis bias: " << torque_y_bias << std::endl;
//     collective_torque[0] += torque_x_bias;
//     collective_torque[1] += torque_y_bias;

    ResilientEstimator::TorqueAndThrust result;
    result.thrust = collective_force;
    result.torque = collective_torque;

    return result;
}



//----------------------------------------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------PX4 Internal Functions---------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------------------//

int ResilientEstimator::task_spawn(int argc, char *argv[])
{
    ResilientEstimator *instance = new ResilientEstimator();

	if (instance) {
		_object.store(instance);


		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			PX4_INFO("check if task_spawn ran init");
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}


ResilientEstimator *ResilientEstimator::instantiate(int argc, char *argv[])
{
	ResilientEstimator *instance = new ResilientEstimator();
	if (instance->init()) {
		return instance;
	}
	else {
		delete instance;
		return nullptr;
	}
}

int ResilientEstimator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ResilientEstimator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
This module subscribes to vehicle_attitude messages and processes them to output a more robust attitude estimation. It is designed to demonstrate a simple extension of PX4 via modules that enhance its capabilities.

### Implementation
The module subscribes to the `vehicle_attitude` topic, processes the data to potentially enhance its resilience to errors or disturbances, and then publishes the result to a new `robust_attitude` topic.

### Examples
$ mc_resilient_estimator start
$ mc_resilient_estimator stop
$ mc_resilient_estimator status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_resilient_estimator", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the module");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop the module");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print the current status");
	// Add any custom commands your module supports:
	// PRINT_MODULE_USAGE_COMMAND("custom");
	// PRINT_MODULE_USAGE_COMMAND_DESCR("custom", "Describe what custom does");

	return 0;
}

extern "C" __EXPORT int mc_resilient_estimator_main(int argc, char *argv[])
{

	return ResilientEstimator::main(argc, argv);

}
