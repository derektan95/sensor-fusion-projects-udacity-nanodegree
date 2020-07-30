#include "ukf.h"
#include "Eigen/Dense"
#include<iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  // Initialize class variables
  n_x_ = 5;
  n_aug_ = n_x_ + 2;
  lambda_ = 3 - n_x_;
  weights_ = VectorXd(2*n_aug_ + 1);

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  // If first reading...
  if (!is_initialized_)
  {
    if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      // Initialize State Vector x_
      double x_lidar = meas_package.raw_measurements_(0,0);
      double y_lidar = meas_package.raw_measurements_(1,0);
      x_ << x_lidar, y_lidar, 0, 0, 0;

      P_ = MatrixXd::Identity(5, 5);

      time_us_ = meas_package.timestamp_;

      is_initialized_ = true;
      std::cout << "INTIIALIZED X STATE VECTOR USING LIDAR MEASUREMENTS" << std::endl;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      // Initialize State Vector x_
      double rho = meas_package.raw_measurements_(0,0);
      double phi = meas_package.raw_measurements_(1,0);
      x_ << rho, phi, 0, 0, 0;

      P_ = MatrixXd::Identity(5, 5);

      time_us_ = meas_package.timestamp_;
      
      is_initialized_ = true;
    }
    else
    {
      std::cout << "UNIDENTIFIED SENSOR TYPE" << std::endl;
    }    
  }

  // Prediction Step
  float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  Prediction(dt);

  // Update Step
  if (meas_package.sensor_type_ == MeasurementPackage::LASER)
  {
    UpdateLidar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
    UpdateRadar(meas_package);
  }
  else
  {
    std::cout << "UNIDENTIFIED SENSOR TYPE" << std::endl;
  }

  // Record timestamp to calculate dt for next time step
  time_us_ = meas_package.timestamp_;
  
  
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  // Generate Sigma Points
  MatrixXd Xsig_aug = GenerateAugmentedSigmaPoints();

  // Convert Sigma Points through nonlinear mapping function
  PredictNewSigmaPoints(Xsig_aug, delta_t);

  // Calculate new Predicted Mean and Covariance using new Sigma Points
  // Update x_ and P_ with predicted values temporarily for Update use.
  CalculatePredictedStateAndCovariance();

  // cout << "Completed Prediction Step" << endl;
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  


  // cout << "Completed Lidar Update Step" << endl;
  
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  // Predict New Sigma Points for radar via nonlinear h(x)
  VectorXd z_predicted;
  MatrixXd Z_sigma_pts;
  MatrixXd S_output;
  UpdateNewSigmaPointsRadar(Z_sigma_pts, z_predicted, S_output);

  VectorXd z_meas = meas_package.raw_measurements_;
  CompleteUpdateStepRadar(Z_sigma_pts, z_predicted, S_output, z_meas);

  // cout << "Completed Radar Update Step" << endl;  
}


// --- HELPER FUNCTION --- //


// --- PREDICTION STAGE 

MatrixXd UKF::GenerateAugmentedSigmaPoints() 
{

  // create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  // create augmented state covariance
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i < n_aug_; ++i) {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  return Xsig_aug;

}

// Predict new Sigma Points by applying each augmented sigma point to nonlinear CTRV mapping function
void UKF::PredictNewSigmaPoints(const MatrixXd &Xsig_aug, double delta_t) 
{

  // create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // predict sigma points
  for (int i = 0; i< 2*n_aug_+1; ++i) {
    
    // extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    } 
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // add Process Noise component
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // write predicted sigma point into right column
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;

  }

  // Update within ukf class
  Xsig_pred_ = Xsig_pred;

}


void UKF::CalculatePredictedStateAndCovariance()
{

  // create vector for predicted state
  VectorXd x = VectorXd(n_x_);

  // create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);
  
  // set weights - CAN SET DURING INITIALIZATION TOO
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; ++i) {  // 2n+1 weights
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }

  // predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
    x = x + weights_(i) * Xsig_pred_.col(i);
  }

  // predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P = P + weights_(i) * x_diff * x_diff.transpose() ;
  }

  // Update State Vector X_ and Covariance P_
  x_ = x;
  P_ = P;
  
}



// --- UPDATE STAGE 

void UKF::UpdateNewSigmaPointsRadar(MatrixXd &Z_sigma_pts, VectorXd &z_pred_mean, MatrixXd &S_output)
{

  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  
  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  
  // set weights - CAN SET DURING INITIALIZATION TOO
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; ++i) {  // 2n+1 weights
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // extract values for better readability
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                       // r
    Zsig(1,i) = atan2(p_y,p_x);                                // phi
    Zsig(2,i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   // r_dot
  }

  // mean measurement
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; ++i) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<  std_radr_*std_radr_, 0, 0,
        0, std_radphi_*std_radphi_, 0,
        0, 0,std_radrd_*std_radrd_;
  S = S + R;

  // Pass resulting measurement vector and error matrix as outut reference
  Z_sigma_pts = Zsig;
  z_pred_mean = z_pred;
  S_output = S;

}



void UKF::CompleteUpdateStepRadar(const MatrixXd &Zsig, const VectorXd &z_pred, const MatrixXd &S, const VectorXd &z_meas)
{

  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = z_meas - z_pred;

  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();


}
