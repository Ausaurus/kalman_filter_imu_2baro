#pragma once

// 1. Detect if we are in Arduino/PlatformIO
#ifdef ARDUINO
// Use the Arduino-compatible wrapper
// This file automatically undefines 'B1' and 'F' so they don't crash Eigen
#include <ArduinoEigenDense.h> 
#else
// We are on PC (Linux/Windows), use standard Eigen
#include <Eigen/Dense>
#endif
namespace MyKalman {

  // S = State Dimensions (e.g., 4 for Position+Velocity x/y)
  // M = Measurement Dimensions (e.g., 2 for GPS x/y)
  template <int S, int M>
    class KalmanFilter {
      public:
        // Define fixed-size types to avoid malloc!
        using StateVector = Eigen::Matrix<float, S, 1>;
        using MeasurementVector = Eigen::Matrix<float, M, 1>;
        using StateMatrix = Eigen::Matrix<float, S, S>;
        using MeasurementMatrix = Eigen::Matrix<float, M, S>;
        using KalmanGainMatrix = Eigen::Matrix<float, S, M>;

        KalmanFilter() {
          // Initialize identity matrices, etc.
          P.setIdentity();
          // ...
        }

        void predict(const StateMatrix& F, const StateMatrix& Q) {
          x = F * x;
          P = F * P * F.transpose() + Q;
        }

        void update(const MeasurementVector& z, const MeasurementMatrix& H, 
            const Eigen::Matrix<float, M, M>& R) {

          // This math is heavy! Ensure optimization (-O2 or -O3) is on.
          Eigen::Matrix<float, M, M> S_cov = H * P * H.transpose() + R;

          // Use LLT or LDLT for inversion on embedded (faster/more stable than .inverse())
          KalmanGainMatrix K = P * H.transpose() * S_cov.llt().solve(Eigen::Matrix<float, M, M>::Identity());

          x = x + K * (z - H * x);
          P = (StateMatrix::Identity() - K * H) * P;
        }

        // Allow the user to reset the filter to a specific state
        void init(const StateVector& x0, const StateMatrix& P0) {
          x = x0;
          P = P0;
        }

        StateVector get() {
          return x;
        }

      private:
        StateVector x; // State estimate
        StateMatrix P; // Error covariance
    };

} // namespace
