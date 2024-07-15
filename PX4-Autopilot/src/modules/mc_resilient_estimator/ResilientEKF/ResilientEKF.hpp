#ifndef RESILIENT_EKF_H
#define RESILIENT_EKF_H

# pragma GCC diagnostic ignored "-Wshadow"

#include <lib/matrix/matrix/math.hpp>


class ResilientEKF {
public:
    ResilientEKF(matrix::Vector<float, 16>& Data, float timeStep, int n_states, int n_control, int n_obs,
                 matrix::Matrix<float, 7, 12>& obs_matrix, matrix::Matrix<float, 12, 12>& process_noise_matrix,
                 matrix::Matrix<float, 7, 7>& observation_noise_matrix, float Mass, float Ixx, float Iyy, float Izz, float Im, float d, float C_T, float C_M);

    ResilientEKF(const ResilientEKF& other); // Copy constructor

    ResilientEKF& operator=(const ResilientEKF& other); // Copy assignment operator

    matrix::Vector<float, 16> data;

    // Coefficients
    float Mass, Ixx, Iyy, Izz, Im, d, C_T, C_M;

    int n, m, p; // Number of states, control inputs, and observations
    float dt; // Time interval

    matrix::Vector<float, 12> x_hat_prev, x_cond_prev;
    matrix::Vector<float, 4>  U_prev;
    matrix::Matrix<float, 12, 12> P_prev;

    matrix::Vector<float, 4> U;
    matrix::Matrix<float, 12, 12> F;
    matrix::Matrix<float, 7, 12> H;
    matrix::Matrix<float, 12, 12> Q;
    matrix::Matrix<float, 7, 7> R;

    matrix::Vector<float, 12> x_hat_cond;
    matrix::Vector<float, 7> y;
    matrix::Vector<float, 7> z;
    matrix::Matrix<float, 12, 12> P_cond;
    matrix::Matrix<float, 7, 7> S;
    matrix::Matrix<float, 12, 7> K;

    matrix::Vector<float, 12> x_hat;
    matrix::Matrix<float, 12, 12> P_k;

    float res, err;

    // Getters for class members
    matrix::Vector<float, 12> get_x_hat() const { return x_hat; }
    matrix::Matrix<float, 12, 12> get_P_k() const { return P_k; }
    float get_res() const { return res; }
    float get_err() const { return err; }

    void update_previous_data(const matrix::Vector<float, 12>& prev_x, const matrix::Vector<float, 12>& prev_x_cond, const matrix::Vector<float, 4>& prev_u, const matrix::Matrix<float, 12, 12>& prev_p);
    void calculate_priors();
    void calculate_posteriors();
};

#endif // RESILIENT_EKF_H
