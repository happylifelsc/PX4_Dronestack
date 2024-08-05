#include "ResilientEKF.hpp"
#include <cmath>


// Static initialization
matrix::Matrix<float, 7, 12> ResilientEKF::H = []() {
    static matrix::Matrix<float, 7, 12> mat;
    mat.zero();
    mat(0, 2) = 1.0f;
    mat(1, 6) = 1.0f;
    mat(2, 7) = 1.0f;
    mat(3, 8) = 1.0f;
    mat(4, 9) = 1.0f;
    mat(5, 10) = 1.0f;
    mat(6, 11) = 1.0f;
    return mat;
}();

matrix::Matrix<float, 12, 12> ResilientEKF::Q = []() {
    matrix::Matrix<float, 12, 12> mat;
    mat.setIdentity();
    return 0.01f * mat;
}();

matrix::Matrix<float, 7, 7> ResilientEKF::R = []() {
    matrix::Matrix<float, 7, 7> mat;
    mat.zero();
    mat(0, 0) = 0.01f;
    for (int i = 1; i < 4; ++i) {
        mat(i, i) = 0.01f;
    }
    for (int i = 4; i < 7; ++i) {
        mat(i, i) = 0.01f;
    }
    return mat;
}();

ResilientEKF::ResilientEKF(matrix::Vector<float, 16>& Data, float timeStep, int n_states, int n_control, int n_obs, float Mass, float Ixx, float Iyy, float Izz, float Im, float d, float C_T, float C_M) {

    // Initialize coefficients
    this->Mass = Mass;
    this->Ixx = Ixx;
    this->Iyy = Iyy;
    this->Izz = Izz;
    this->Im = Im;
    this->d = d;
    this->C_T = C_T;
    this->C_M = C_M;

    // Set other member variables from parameters
    this->data = Data;
    this->dt = timeStep;
    this->n = n_states;
    this->m = n_control;
    this->p = n_obs;
    // this->H = obs_matrix;
    // this->Q = process_noise_matrix;
    // this->R = observation_noise_matrix;


    // // Example of initializing other member variables based on the constructor parameters
    // this->x_hat_prev = matrix::Vector<float, 12>();
    // this->x_cond_prev = matrix::Vector<float, 12>();
    // this->U_prev = matrix::Vector<float, 4>();
    // this->P_prev = matrix::Matrix<float, 12, 12>();

    // // Initialize the state transition matrix F, if it's static or depends on parameters that don't change frequently
    // this->F = matrix::Matrix<float, 12, 12>(); // You might need to update this in your predict or update methods

    // Control input U
    this->U = matrix::Vector<float, 4>();
    this->U(0) = Data(12);
    this->U(1) = Data(13);
    this->U(2) = Data(14);
    this->U(3) = Data(15); // Assuming data contains control inputs in the last four positions

    // // Initial state estimate, if it's based on `data` or should be zero-initialized
    // this->x_hat_cond = matrix::Vector<float, 12>();


    // Initialize y
    this->y = matrix::Vector<float, 7>();
    this->y(0) = Data(2);
    this->y(1) = Data(6);
    this->y(2) = Data(7);
    this->y(3) = Data(8);
    this->y(4) = Data(9);
    this->y(5) = Data(10);
    this->y(6) = Data(11);
}

// delete H(other.H), Q(other.Q), R(other.R) by zyl and gpt
ResilientEKF::ResilientEKF(const ResilientEKF& other)
    : data(other.data), Mass(other.Mass), Ixx(other.Ixx), Iyy(other.Iyy),
      Izz(other.Izz), Im(other.Im), d(other.d), C_T(other.C_T), C_M(other.C_M),
      n(other.n), m(other.m), p(other.p), dt(other.dt),
      x_hat_prev(other.x_hat_prev), x_cond_prev(other.x_cond_prev), U_prev(other.U_prev), P_prev(other.P_prev),
      U(other.U), F(other.F),
      x_hat_cond(other.x_hat_cond), y(other.y), z(other.z),
      P_cond(other.P_cond), S(other.S), K(other.K),
      x_hat(other.x_hat), P_k(other.P_k), res(other.res), err(other.err) {}

ResilientEKF& ResilientEKF::operator=(const ResilientEKF& other) {
    if (this != &other) {
        data = other.data;
        Mass = other.Mass;
        Ixx = other.Ixx;
        Iyy = other.Iyy;
        Izz = other.Izz;
        Im = other.Im;
        d = other.d;
        C_T = other.C_T;
        C_M = other.C_M;
        n = other.n;
        m = other.m;
        p = other.p;
        dt = other.dt;
        x_hat_prev = other.x_hat_prev;
        x_cond_prev = other.x_cond_prev;
        U_prev = other.U_prev;
        P_prev = other.P_prev;
        U = other.U;
        F = other.F;
        H = other.H;
        Q = other.Q;
        R = other.R;
        x_hat_cond = other.x_hat_cond;
        y = other.y;
        z = other.z;
        P_cond = other.P_cond;
        S = other.S;
        K = other.K;
        x_hat = other.x_hat;
        P_k = other.P_k;
        res = other.res;
        err = other.err;
    }
    return *this;
}

// ResilientEKF::~ResilientEKF() {
//     // Clean up if necessary
//     // If no dynamic memory allocation inside, this can be empty
// }

void ResilientEKF::update_previous_data(const matrix::Vector<float, 12>& prev_x, const matrix::Vector<float, 12>& prev_x_cond, const matrix::Vector<float, 4>& prev_u, const matrix::Matrix<float, 12, 12>& prev_p) {
    // Update the previous state estimate, control input, and covariance estimate
    x_hat_prev = prev_x;
    x_cond_prev = prev_x_cond;
    U_prev = prev_u;
    P_prev = prev_p;
}

void ResilientEKF::calculate_priors() {
    float M = Mass;
    float g = 9.8f;
    float a1 = (Iyy - Izz) / Ixx;
    float a2 = (Izz - Ixx) / Iyy;
    float a3 = (Ixx - Iyy) / Izz;
    float b1 = 1 / Ixx;
    float b2 = 1 / Iyy;
    float b3 = 1 / Izz;

    x_hat_cond(0) = x_hat_prev(0) + dt * (x_hat_prev(3));
    x_hat_cond(1) = x_hat_prev(1) + dt * (x_hat_prev(4));
    x_hat_cond(2) = x_hat_prev(2) + dt * (x_hat_prev(5));

    x_hat_cond(3) = x_hat_prev(3) + dt * (a1 * x_hat_prev(4) * x_hat_prev(5) - Im * 10 * x_hat_prev(4) + b1 * U_prev(0));
    x_hat_cond(4) = x_hat_prev(4) + dt * (a2 * x_hat_prev(3) * x_hat_prev(5) + Im * 10 * x_hat_prev(3) + b2 * U_prev(1));
    x_hat_cond(5) = x_hat_prev(5) + dt * (a3 * x_hat_prev(3) * x_hat_prev(4) + b3 * U_prev(2));

    x_hat_cond(6) = x_hat_prev(6) + dt * (x_hat_prev(9));
    x_hat_cond(7) = x_hat_prev(7) + dt * (x_hat_prev(10));
    x_hat_cond(8) = x_hat_prev(8) + dt * (x_hat_prev(11));

    x_hat_cond(9) = x_hat_prev(9) + dt * (1 / M) * U_prev(3) * (
        std::cos(x_hat_prev(0)) * std::sin(x_hat_prev(1)) *
        std::cos(x_hat_prev(2)) + std::sin(x_hat_prev(0)) * std::sin(x_hat_prev(2)));
    x_hat_cond(10) = x_hat_prev(10) + dt * (1 / M) * U_prev(3) * (
        std::cos(x_hat_prev(0)) * std::sin(x_hat_prev(1)) *
        std::sin(x_hat_prev(2)) - std::sin(x_hat_prev(0)) * std::cos(x_hat_prev(2)));
    x_hat_cond(11) = x_hat_prev(11) + dt * (
        -g + (1 / M) * std::cos(x_hat_prev(0)) * std::cos(x_hat_prev(1)) * U_prev(3));

    float z_34 = dt * (a1 * x_hat_prev(5) - Im);
    float z_35 = dt * (a1 * x_hat_prev(4));

    float z_43 = dt * (a2 * x_hat_prev(5) + Im);
    float z_45 = dt * (a2 * x_hat_prev(3));

    float z_53 = dt * (a3 * x_hat_prev(4));
    float z_54 = dt * (a3 * x_hat_prev(3));

    float z_90 = dt * (-std::sin(x_hat_prev(0)) * std::sin(x_hat_prev(1)) * std::cos(x_hat_prev(2)) + std::cos(x_hat_prev(0)) * std::sin(x_hat_prev(2))) * (1 / M) * U_prev(3);
    float z_91 = dt * (std::cos(x_hat_prev(0)) * std::cos(x_hat_prev(1)) * std::cos(x_hat_prev(2))) * (1 / M) * U_prev(3);
    float z_92 = dt * (-std::cos(x_hat_prev(0)) * std::sin(x_hat_prev(1)) * std::sin(x_hat_prev(2)) + std::sin(x_hat_prev(0)) * std::cos(x_hat_prev(2))) * (1 / M) * U_prev(3);

    float z_100 = dt * (-std::sin(x_hat_prev(0)) * std::sin(x_hat_prev(1)) * std::sin(x_hat_prev(2)) - std::cos(x_hat_prev(0)) * std::cos(x_hat_prev(2))) * (1 / M) * U_prev(3);
    float z_101 = dt * (std::cos(x_hat_prev(0)) * std::cos(x_hat_prev(1)) * std::sin(x_hat_prev(2))) * (1 / M) * U_prev(3);
    float z_102 = dt * (std::cos(x_hat_prev(0)) * std::sin(x_hat_prev(1)) * std::cos(x_hat_prev(2)) + std::sin(x_hat_prev(0)) * std::sin(x_hat_prev(2))) * (1 / M) * U_prev(3);

    float z_110 = -dt * (1 / M) * std::sin(x_hat_prev(0)) * std::cos(x_hat_prev(1)) * U_prev(3);
    float z_111 = -dt * (1 / M) * std::cos(x_hat_prev(0)) * std::sin(x_hat_prev(1)) * U_prev(3);
    // F << 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0,
    //     0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 1, z_34, z_35, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, z_43, 1, z_45, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, z_53, z_54, 1, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 0,
    //     0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0,
    //     0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt,
    //     z_90, z_91, z_92, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    //     z_100, z_101, z_102, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    //     z_110, z_111, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

    F.setIdentity();
    F(0, 3) = dt;
    F(1, 4) = dt;
    F(2, 5) = dt;
    F(6, 9) = dt;
    F(7, 10) = dt;
    F(8, 11) = dt;
    F(3, 4) = z_34;
    F(3, 5) = z_35;
    F(4, 3) = z_43;
    F(4, 5) = z_45;
    F(5, 3) = z_53;
    F(5, 4) = z_54;
    F(9, 0) = z_90;
    F(9, 1) = z_91;
    F(9, 2) = z_92;
    F(10, 0) = z_100;
    F(10, 1) = z_101;
    F(10, 2) = z_102;
    F(11, 0) = z_110;
    F(11, 1) = z_111;

    multiplyMatricesInPlace(F, P_prev, temp1);
    multiplyMatricesInPlace(temp1, F.transpose(), P_cond);
    P_cond += Q;

    multiplyMatricesInPlace(H, P_cond, temp2);
    multiplyMatricesInPlace(temp2, H.transpose(), S);
    S += R;

}


void ResilientEKF::calculate_posteriors() {
    // Calculate the pseudoinverse of S
    matrix::Matrix<float, 7, 7> S_inv;
    if (!matrix::geninv(S, S_inv)) {
        // Handle the case where the pseudoinverse could not be computed
        // For example, you can set res and err to some error values or handle it as needed
        res = -1;
        err = -1;
        return;
    }

    // Calculate the Kalman gain, updated state estimate, and updated covariance estimate
    // K = P_cond * H.transpose() * S_inv;
    // matrix::Matrix<float, 12, 12> P_k_support;
    // P_k_support.setIdentity();
    // P_k = (P_k_support- K * H) * P_cond;
    // z = y - H * x_hat_cond;
    // x_hat = x_hat_cond + K * z;

    multiplyMatricesInPlace(P_cond, H.transpose(), temp3);
    multiplyMatricesInPlace(temp3, S_inv, K);
    P_k_support.setIdentity();
    multiplyMatricesInPlace(K, H, temp1);
    multiplyMatricesInPlace(P_k_support - temp1, P_cond, P_k);
    multiplyMatricesInPlace(H, x_hat_cond, temp4);
    z = y - temp4;
    multiplyMatricesInPlace(K, z, temp5);
    x_hat = x_hat_cond + temp5;


}


matrix::Vector<float, 12> ResilientEKF::run(bool visual_odom_update_flag) {

    calculate_priors();
    // return x_hat_cond;

    // Determine whether to do estimation or not
	// GPS is updated, calculate posteriors
	if (visual_odom_update_flag){
	    calculate_posteriors();
		// x_output = x_hat;
		x_hat_prev = x_hat;
		P_prev = P_k;
	}
	//// GPS is not updated, calculate priors only
	else if (!visual_odom_update_flag){
		// x_output = x_hat_cond;
	    x_hat_prev = x_hat_cond;
		P_prev = P_cond;
	}

    // Update next previous control input and prediction
	x_cond_prev = x_hat_cond;
	U_prev = U;

    if(visual_odom_update_flag) return x_hat;
    else return x_hat_cond;


}

// template <size_t M, size_t N, size_t P>
// void multiplyMatricesInPlace(const matrix::Matrix<float, M, N> &A, const matrix::Matrix<float, N, P> &B, matrix::Matrix<float, M, P> &C) {
//     // Clear the result matrix to ensure it starts with zeros
//     C.zero();

//     // Perform the matrix multiplication
//     for (size_t k = 0; k < N; ++k) {
//         for (size_t i = 0; i < M; ++i) {
//             float aik = A(i, k);
//             for (size_t j = 0; j < P; ++j) {
//                 C(i, j) += aik * B(k, j);
//             }
//         }
//     }
// }
