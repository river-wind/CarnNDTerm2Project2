#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

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
  //CEL changed to 0.5 based on tip note about reasonable acceleration rates
  std_a_ = 1.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  //CEL: changed to 1.0 based on tip note about reasonable turn rad
  std_yawdd_ = 0.54;

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
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  n_x_ = x_.size(); //augmented state
  n_aug_ = n_x_ + 2; //augmented state size + 2 sigma points
  lambda_ = 3 - n_aug_;  //distance of sigma points from mean
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1); //predicted sigma points
  weights_ = VectorXd(2 * n_aug_ + 1); //sigma point weights
  is_initialized_ = false;
}

UKF::~UKF() {}


double UKF::NormalizeAngle(double& angle){
  double norm_angle = angle;  //copy angle
  while (norm_angle > M_PI)  norm_angle = norm_angle-(2.0*M_PI); //reduce angle if too small 
  while (norm_angle < -M_PI)  norm_angle = norm_angle+(2.0*M_PI); //increase angle if too large
  return norm_angle;  //prevent angles from getting twoo high.  Very rough
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  //Initialize the KF
  if (!is_initialized_) {
    // first measurement
    cout << "UKF: " << endl;
    // CTRV: x_ =[px, py, v, angle, angle_rate]
    //CEL: let's start with an identity matrix for P
    P_ << 1,0,0,0,0,
          0,1,0,0,0,
          0,0,1,0,0,
          0,0,0,1,0, 
          0,0,0,0,1; 

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      //CEL: calculate the x,y position based on angle (phi) and range (rho)
      //CEL: px is adjacent to phi and might be calculated as rho * cos(phi)
      //CEL: py is opposed to phi and might be calculated as rho * sin(phi)
      float px = meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]);
      float py = meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]);
      float v = sqrt(pow(meas_package.raw_measurements_[2] * cos(meas_package.raw_measurements_[1]),2) + pow(meas_package.raw_measurements_[2] * sin(meas_package.raw_measurements_[1]),2));
      x_  << px, py, v, 0, 0;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      //cout<<"initializing for laser"<<endl;
      //CEL: get initial measurements and set ekf_.x_, as in lesson 5.12
      if  (fabs(meas_package.raw_measurements_[0]) < .0001 and fabs(meas_package.raw_measurements_[1])< .0001){
      x_ << .0001, .0001, 0,0,0;
      }
      else {
        x_ << meas_package.raw_measurements_[0],meas_package.raw_measurements_[1], 0, 0, 0;
      }  
    }
    //Finish setup
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  //CEL: calc timestamp
  float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
  time_us_ = meas_package.timestamp_;
  //CEL: call predict
  Prediction(dt);
  if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && (use_laser_)){
    UpdateLidar(meas_package);
  }
  else if  ((meas_package.sensor_type_ == MeasurementPackage::RADAR) && (use_radar_)) {
    UpdateRadar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  VectorXd x_aug = VectorXd(n_aug_);
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  //CEL: code from Lesson 18:, Augmented Assignment 2	
  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }


  //Sigma point prediction, from Lesson 21, Sigma Point Prediction Assignment 2
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
        py_p = p_y + v/yawd * (cos(yaw) - cos(yaw + yawd * delta_t) );
    }
    else {
        px_p = p_x + v * delta_t * cos(yaw);
        py_p = p_y + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;

    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  //CEL: Lesson 24: Predicted Mean and Covariance
  // set weights
  double weight_0 = lambda_/(lambda_ + n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<2 * n_aug_ + 1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug_ + lambda_);
    weights_(i) = weight;
  }

  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    UKF::NormalizeAngle(x_diff(3));

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
//cout<<"test 5.5 - init UpdateLidar";
  VectorXd z = meas_package.raw_measurements_;
  MatrixXd Z_sig = MatrixXd(2, 2 * n_aug_ +1);
  Z_sig.fill(0.0);
  MatrixXd S = MatrixXd(2, 2);
  S.fill(0.0);
  VectorXd z_pred = VectorXd(2);
  z_pred.fill(0.0);
 //cout<<"test6 - inside UpdateLidar"; 
  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);
    Z_sig(0,i) = px;                   
    Z_sig(1,i) = py;                                 
  }
//cout<<"test7 ";
  z_pred = Z_sig * weights_; 
//cout<<"test7.1 ";
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  
    VectorXd z_diff = Z_sig.col(i) - z_pred;
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
//cout<<"test7.2 ";
  //noise
  MatrixXd R = MatrixXd(2,2);
  R <<    std_laspx_ * std_laspx_, 0,
          0, std_laspy_ * std_laspy_;
//cout<<"test7.3 ";
  S = S + R;
//cout<<"test8";	
  //cross correlation
  MatrixXd Tc = MatrixXd(n_x_, 2);
  Tc.fill(0.0);
//cout<<"test9";	
  //calculate cross correlation
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  
    VectorXd z_diff = Z_sig.col(i) - z_pred;
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
//cout<<"test10";
  //Kalman gain
  MatrixXd K = Tc * S.inverse();
  VectorXd z_diff = z - z_pred;
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
//cout<<"test11";
  //NIS Lidar
  NIS_lidar_ = z_diff.transpose() * S.inverse() * z_diff; 

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  //CEL: predict Sgima points  Lesson 27
  VectorXd z = meas_package.raw_measurements_;
  MatrixXd Z_sig = MatrixXd(3, 2 * n_aug_ + 1);
  Z_sig.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    if (fabs(p_x) < .0001) p_x = .0001;
    if (fabs(p_y) < .0001) p_y = .0001;

    // measurement model
    Z_sig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Z_sig(1,i) = atan2(p_y,p_x);                                 //phi
    Z_sig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(3);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Z_sig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(3,3);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Z_sig.col(i) - z_pred;

    //angle normalization
    //while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    //while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    UKF::NormalizeAngle(z_diff(1));
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(3,3);
  R <<    std_radr_ * std_radr_, 0, 0,
          0, std_radphi_ * std_radphi_, 0,
          0, 0,std_radrd_ * std_radrd_;
  S = S + R;
  
  //calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, 3);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Z_sig.col(i) - z_pred;
    //angle normalization
    UKF::NormalizeAngle(z_diff(1));
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    UKF::NormalizeAngle(x_diff(3));
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  UKF::NormalizeAngle(z_diff(1));
  //while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
  //while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;
  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();  
  //NIS Update
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
}
