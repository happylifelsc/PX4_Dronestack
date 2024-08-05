
#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#pragma GCC diagnostic ignored "-Wshadow"
// #pragma GCC diagnostic ignored "-Werror=frame-larger-than"
#pragma GCC diagnostic pop


#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
// #include <lib/mathlib/math/filter/LowPassFilter2pVector3f.hpp>
#include <lib/mathlib/math/filter/NotchFilter.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/time.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/robust_attitude.h>
#include <uORB/topics/robust_angular_acceleration.h>
#include <uORB/topics/estimator_innovations.h>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/imu_attack.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/anomaly_detection.h>
#include <uORB/topics/resilient_state_estimate.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/rpm.h>
#include <cmath>


#include <ResilientEKF.hpp>


extern "C" __EXPORT int mc_resilient_estimator_main(int argc, char *argv[]);
class ResilientEstimator : public ModuleBase<ResilientEstimator>,
        public ModuleParams, public px4::WorkItem
{
public:
    ResilientEstimator();
    ~ResilientEstimator();

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static ResilientEstimator *instantiate(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    bool init();

    void Run() override;

    struct SlidingWindowResult {
        matrix::Vector<int, 1000>* window;
        double anomaly_percentage;
    };

    struct TorqueAndThrust {
        matrix::Vector3f torque;
        matrix::Vector3f thrust;
    };

    vehicle_odometry_s prev_visual_odom;


    SlidingWindowResult sliding_window_detection(matrix::Vector<int, 1000>& window, size_t window_size, int new_score);

    matrix::Vector3f quaternionToEuler(double q0, double q1, double q2, double q3);

    matrix::Vector4f eulerToQuaternionVector(double roll, double pitch, double yaw);

    matrix::Vector3f calculate_angular_acc_from_dynamics(const matrix::Vector3f& angular_vels, const matrix::Vector3f& torques);

    TorqueAndThrust calculate_individual_torques_and_thrust(matrix::Vector4f& rotor_speeds, matrix::Vector3f& velocity);


private:
    uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)};
	uORB::SubscriptionCallbackWorkItem _sensor_gyro_sub{this, ORB_ID(sensor_gyro)};

    uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};
    uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
    uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};

    uORB::Subscription _visual_odometry_sub{ORB_ID(vehicle_visual_odometry)};
    // uORB::Subscription _actuator_0_sub{ORB_ID(actuator_controls_0)};
    // uORB::Subscription _actuator_output_sub{ORB_ID(actuator_outputs)};

    uORB::Subscription _attitude_ground_truth_sub{ORB_ID(vehicle_attitude_groundtruth)};
    uORB::Subscription _vehicle_angular_velocity_ground_truth_sub{ORB_ID(vehicle_angular_velocity_groundtruth)};

    uORB::Subscription _estimator_innovations_sub{ORB_ID(estimator_innovations)};

    uORB::Subscription	_lpos_ground_truth_sub{ORB_ID(vehicle_local_position_groundtruth)};
    uORB::Subscription _imu_attack_sub{ORB_ID(imu_attack)};

    uORB::Subscription	_gpos_ground_truth_sub{ORB_ID(vehicle_global_position_groundtruth)};

    // uORB::Subscription	_filtered_imu_sub{ORB_ID(filtered_imu)};

    uORB::Subscription _anomaly_detection_sub{ORB_ID(anomaly_detection)};
    uORB::SubscriptionMultiArray<rpm_s> _rpm_subs{ORB_ID::rpm};

    uORB::Publication<anomaly_detection_s> _anomaly_detection_pub{ORB_ID(anomaly_detection)};
    uORB::Publication<robust_attitude_s> _robust_attitude_pub{ORB_ID(robust_attitude)};
    uORB::Publication<robust_angular_acceleration_s> _robust_angular_acc_pub{ORB_ID(robust_angular_acceleration)};

    uORB::Publication<resilient_state_estimate_s> _resilient_state_estimate_pub{ORB_ID(resilient_state_estimate)};

    perf_counter_t	_loop_perf;


    matrix::Vector4f quaternion_output;
    matrix::Quatf q;
    matrix::Vector4f quaternionVector;
    matrix::Vector3f angular_velocity_output_raw;
    matrix::Vector3f angular_velocity_output_filtered;
    matrix::Vector3f angular_velocity_output_filtered_vector;
    matrix::Vector3f torque_vector;
    matrix::Vector3f robust_angular_acc_vector;
    matrix::Vector<int, 1000> cusum_window;
    matrix::Vector3f angular_velocity_estimate_residuals;

    robust_angular_acceleration_s robust_angular_acc;
    robust_attitude_s robust_att;
    anomaly_detection_s anomaly_detection_msg;
    resilient_state_estimate_s resilient_state_estimate_msg;



//     DEFINE_PARAMETERS(
//         // Add module parameters here
//     )

    void update();
};


// math::LowPassFilter2pVector3f _lp_filter_torque{1000.0f, 30.0f};
// math::LowPassFilter2p _lp_filter_thrust(1000.0f, 30.0f);
// math::LowPassFilter2pVector3f _lp_filter_velocity{1000.0f, 30.0f};

math::LowPassFilter2p<float> _lp_filter_velocity[3] {};
math::NotchFilter<matrix::Vector3f> _notch_filter_velocity{};
