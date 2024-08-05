#ifndef RESILIENT_EKF_H
#define RESILIENT_EKF_H

# pragma GCC diagnostic ignored "-Wshadow"

#include <lib/matrix/matrix/math.hpp>


class ResilientEKF {
public:
    ResilientEKF(matrix::Vector<float, 16>& Data, float timeStep, int n_states, int n_control, int n_obs, float Mass, float Ixx, float Iyy, float Izz, float Im, float d, float C_T, float C_M);
    // above code deleted by zyl: matrix::Matrix<float, 7, 12>& obs_matrix, matrix::Matrix<float, 12, 12>& process_noise_matrix, matrix::Matrix<float, 7, 7>& observation_noise_matrix,

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
    static matrix::Matrix<float, 7, 12> H; //obs_matrix


    static matrix::Matrix<float, 12, 12> Q;
    static matrix::Matrix<float, 7, 7> R;

    matrix::Vector<float, 12> x_hat_cond;
    matrix::Vector<float, 7> y;
    matrix::Vector<float, 7> z;
    matrix::Matrix<float, 12, 12> P_cond;
    matrix::Matrix<float, 12, 12> temp1;
    matrix::Matrix<float, 7, 12> temp2;
    matrix::Matrix<float, 12, 7> temp3;
    matrix::Matrix<float, 7, 1> temp4;
    matrix::Matrix<float, 12, 1> temp5;
    matrix::Matrix<float, 7, 7> S;
    matrix::Matrix<float, 12, 7> K;

    matrix::Matrix<float, 12, 12> P_k_support;

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
    matrix::Vector<float, 12> run(bool visual_odom_update_flag);

    template <size_t M, size_t N, size_t P>
    void multiplyMatricesInPlace(const matrix::Matrix<float, M, N> &A, const matrix::Matrix<float, N, P> &B, matrix::Matrix<float, M, P> &C) {
        // Clear the result matrix to ensure it starts with zeros
        C.zero();

        // Perform the matrix multiplication
        for (size_t k = 0; k < N; ++k) {
            for (size_t i = 0; i < M; ++i) {
                float aik = A(i, k);
                for (size_t j = 0; j < P; ++j) {
                    C(i, j) += aik * B(k, j);
                }
            }
        }
    }
};

#endif // RESILIENT_EKF_H
