#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializing Unscented Kalman filter parameters
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // set it initially to false
  is_initialized_ = false;
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // time stamp initialization
  time_us_ = 0;

  // Process noise standard deviation longitudinal acceleration in m/s^2 (This value is tunable)
  std_a_ = 1;
  // Process noise standard deviation yaw acceleration in rad/s^2 (This value is tunable)
  std_yawdd_ = 0.3;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.
  Hint: one or more values initialized above might be wildly off...
  */

  //Initializing number of state vector length
  n_x_ = 5;
  //Initializing number of augmented state vector length
  n_aug_ = 7;
  // Initializing spreading parameter lambda
  lambda_ = 3 - n_aug_;

  // Declaring state vector and augmented state vector
  x_ = VectorXd::Zero(5);
  x_aug = VectorXd::Zero(7);
  // initial covariance matrix and augmented covariance matrix
  P_ = MatrixXd::Zero(5, 5);
  P_aug = MatrixXd::Zero(7, 7);
  
  // Calculating Sigma points weights
  weights_ = VectorXd::Zero(2*n_aug_+1);
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;

  for (int i=1; i<2*n_aug_+1; i++)
  {  //2n+1 weights
   double weight = 0.5/(n_aug_+lambda_);
   weights_(i) = weight;
  }

  // Current NIS for radar
  NIS_radar_ = 0.0;
  // Current NIS for laser
  NIS_laser_ = 0.0;

}

UKF::~UKF() {}

// Creating UKF instance to use it in the Initialization, Prediction and Updating
//UKF ukf_;
/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package){
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  // skip processing if the both sensors are ignored
 if ((meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) ||
     (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_))
 {

    double delta_t;
    // Updating timestamp
    delta_t = (meas_package.timestamp_ - time_us_)/1000000.0;
    time_us_ = meas_package.timestamp_;

    if (is_initialized_ != true)
    {

      // Initlaizing delta_t
      delta_t = 0.05;

      // State vector initialization
      if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
      {
        double rho = meas_package.raw_measurements_(0);
        double phi = meas_package.raw_measurements_(1);
        x_ << rho*cos(phi),
              rho*sin(phi),
              1,
              1,
              0.1;

        // skip initialization step in the next times
        is_initialized_ = true;
      }

      else if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
      {
        x_ << meas_package.raw_measurements_(0),
              meas_package.raw_measurements_(1),
              1,
              1,
              0.1;

        // skip initialization step in the next times
        is_initialized_ = true;
      }


      // Process covariance initialization
      P_ << 0.15,0,0,0,0,
            0,0.15,0,0,0,
            0,0,1,0,0,
            0,0,0,1,0,
            0,0,0,0,1;

      return;
    }

    ///* For normal case (Not initializing case)
    // Prediction step
    Prediction(delta_t);

    // State update using Laser sensor readings
    if(use_laser_ == true && meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      UpdateLidar(meas_package);
    }

    //// State update using Radar sensor readings
    if(use_radar_ == true && meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      UpdateRadar(meas_package);
    }
 }

 else
 {
   return;
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
  VectorXd state_update = VectorXd::Zero(5);
  MatrixXd P_root;
  //double delta_t;
  double px;
  double py;
  double vel;
  double yaw;
  double yawrate;
  double nu_a;
  double nu_yawacc;
  double px_update;
  double py_update;
  double vel_update;
  double yaw_update;
  double yawrate_update;

  //1- calculating augmented mean state
  x_aug.fill(0.0);
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //1- calculating augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;

  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;


  // Calculating the square root of the process noise
  P_root = P_aug.llt().matrixL();

  // Generate the sigma points for the initialized state
  Xsig_aug = MatrixXd::Zero(7, 2*n_aug_+1);
  Xsig_aug.col(0) =  x_aug;

  for (int i=0; i<n_aug_; i++)
  {
    Xsig_aug.col(i+1) = (x_aug + (sqrt(lambda_+n_aug_) * P_root.col(i)));
    Xsig_aug.col(i+1 + n_aug_) = (x_aug - (sqrt(lambda_+n_aug_) * P_root.col(i)));
  }

  // Plugging the generated Sigma points to the model to predict the new mean state & its covariance
  Xsig_pred_ = MatrixXd::Zero(5, 2*n_aug_+1);

  for(int i =0; i < (2*n_aug_ +1); i++)
  {
    px = Xsig_aug(0, i);
    py = Xsig_aug(1, i);
    vel = Xsig_aug(2, i);
    yaw = Xsig_aug(3, i);
    yawrate = Xsig_aug(4, i);
    nu_a = Xsig_aug(5, i);
    nu_yawacc = Xsig_aug(6, i);

    //avoid division by zero
    if (fabs(yawrate) > 0.001) {
       px_update =   px + (vel/yawrate)*(sin(yaw+yawrate*delta_t) - sin(yaw)) + (0.5*(delta_t*delta_t)*cos(yaw)*nu_a);
       py_update =   py + (vel/yawrate)*(-cos(yaw+yawrate*delta_t) + cos(yaw)) + (0.5*(delta_t*delta_t)*sin(yaw)*nu_a);
     }
  else {
       px_update =   px + (vel*cos(yaw)*delta_t) + (0.5*(delta_t*delta_t)*cos(yaw)*nu_a);
       py_update =   py + (vel*sin(yaw)*delta_t) + (0.5*(delta_t*delta_t)*sin(yaw)*nu_a);
     }


  vel_update = vel + (delta_t*nu_a);
  yaw_update = yaw + (yawrate*delta_t) + (0.5*(delta_t*delta_t)*nu_yawacc);
  yawrate_update = yawrate + delta_t*nu_yawacc;

  // Predicting new state sigma points
  Xsig_pred_.col(i) << px_update,
                       py_update,
                       vel_update,
                       yaw_update,
                       yawrate_update;
  }

  // Predict state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)  //iterate over sigma points
  {
    x_ = x_+ weights_(i) * Xsig_pred_.col(i);
  }

  // Predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)   //iterate over sigma points
  {
    // State difference
    VectorXd x_diff = VectorXd::Zero(3);
    x_diff = Xsig_pred_.col(i) - x_;

    // Phi angle normalization
    if(x_diff(3) > M_PI) x_diff(3)-=2.*M_PI;
    if(x_diff(3) < -M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }

 }


void UKF::UpdateLidar(MeasurementPackage meas_package) {
 /**
 TODO:

 Complete this function! Use lidar data to update the belief about the object's
 position. Modify the state vector, x_, and covariance, P_.

 You'll also need to calculate the lidar NIS.
 */
 /**
 TODO:
   * update the state by using Kalman Filter equations
 */
 double px_meas, py_meas, px, py;
 px_meas = meas_package.raw_measurements_(0);
 py_meas = meas_package.raw_measurements_(1);

 VectorXd lidar_meas = VectorXd::Zero(2);
 lidar_meas << px_meas,
               py_meas;

 MatrixXd Zsig = MatrixXd::Zero(2, 2*n_aug_+1);

 // Using the Predicted Sigma points from the state space to fill the measurement space sigma points.
 for (int i=0; i<(2*n_aug_ +1); i++)
 {

   px = Xsig_pred_(0, i);
   py = Xsig_pred_(1, i);

   Zsig(0, i) = px;
   Zsig(1, i) = py;
 }

 // Calculate Measurement mean and covariance
 VectorXd lidarmeas_mean = VectorXd::Zero(2);
 MatrixXd lidarmeas_covariance = MatrixXd::Zero(2,2);
 VectorXd Z_diff = VectorXd::Zero(2);

 // Cross correlation betweem sigma points in both state and measurement spaces
 MatrixXd Tsig_cross = MatrixXd::Zero(5,2);

 // Laser Measurement Mean
 for (int i=0; i<(2*n_aug_ +1); i++)
 {
   lidarmeas_mean = lidarmeas_mean + (weights_(i) * Zsig.col(i));
 }

 // Measurement Covariance
 for (int i=0; i<(2*n_aug_ +1); i++)
 {
   Z_diff = Zsig.col(i) - lidarmeas_mean;
   lidarmeas_covariance = lidarmeas_covariance + (weights_(i) * (Z_diff) * (Z_diff.transpose()));
 }

 // Adding the radar measurement linear noise
 MatrixXd laser_noise_cov(2,2);
 laser_noise_cov << std_laspx_*std_laspx_, 0,
             0, std_laspy_*std_laspy_;

 lidarmeas_covariance = lidarmeas_covariance + laser_noise_cov;

 // Cross-correlation between measurement & state spaces sigma points
 Z_diff.fill(0.0);
 for (int i=0; i<(2*n_aug_ +1); i++)
 {
   Z_diff = Zsig.col(i) - lidarmeas_mean;
   Tsig_cross = Tsig_cross + (weights_(i) * (Xsig_pred_.col(i) - x_) * (Z_diff.transpose()));
 }

 // State mean & covariance update due to new Radar measurements
 // 1- Kalman Gain calculation
 MatrixXd KG = MatrixXd::Zero(5,2);
 KG = Tsig_cross * lidarmeas_covariance.inverse();

 // Measurement Difference
 VectorXd meas_diff = VectorXd::Zero(3);
 meas_diff = lidar_meas - lidarmeas_mean;

 // LIDAR NIS
 NIS_laser_ = meas_diff.transpose() * lidarmeas_covariance.inverse() * meas_diff;

 // State Mean Update
 x_ = x_ + KG * (meas_diff);

 // State Covariance Update
 P_ = P_ - (KG * lidarmeas_covariance * KG.transpose());

}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  // Update radar_meas with the new measurements
  double rho_meas = meas_package.raw_measurements_(0);
  double phi_meas = meas_package.raw_measurements_(1);
  double rhorate_meas = meas_package.raw_measurements_(2);

  VectorXd radar_meas = VectorXd::Zero(3);
  radar_meas << rho_meas,
                phi_meas,
                rhorate_meas;
 // Calculate rho, phi, rhorate as per the predicted Sigma points in order to compare them with the new measurement
 //to calculate the measurement covariance matrix then.
 double px, py, vel, yaw;
 double rho, phi, rho_rate;
 MatrixXd Zsig = MatrixXd::Zero(3, 2*n_aug_+1);

 // Using the Predicted Sigma points from the state space to fill the measurement space sigma points.
 for (int i=0; i<(2*n_aug_ +1); i++)
 {

   px  = Xsig_pred_(0, i);
   py  = Xsig_pred_(1, i);
   vel = Xsig_pred_(2, i);
   yaw = Xsig_pred_(3, i);

   rho = sqrt((px*px) + (py*py));
   phi = atan2(py,px);
   rho_rate = (((px*cos(yaw)*vel)+(py*sin(yaw)*vel))/(sqrt((px*px) + (py*py))));

   Zsig(0, i) = rho;
   Zsig(1, i) = phi;
   Zsig(2, i) = rho_rate;

 }

  // Calculate Measurement mean and covariance
  VectorXd radarmeas_mean = VectorXd::Zero(3);
  MatrixXd radarmeas_covariance = MatrixXd::Zero(3,3);
  VectorXd Z_diff = VectorXd::Zero(3);

  // Cross correlation betweem sigma points in both state and measurement spaces
  MatrixXd Tsig_cross = MatrixXd::Zero(5,3);

  // Measurement Mean
  for (int i=0; i<(2*n_aug_ +1); i++)
  {
    radarmeas_mean = radarmeas_mean + (weights_(i) * Zsig.col(i));
  }

  // Measurement Covariance
  for (int i=0; i<(2*n_aug_ +1); i++)
  {
    Z_diff = Zsig.col(i) - radarmeas_mean;

    // Phi angle normalization
    if(Z_diff(1) > M_PI) Z_diff(1) -= 2.*M_PI;
    if(Z_diff(1) < -M_PI) Z_diff(1) += 2.*M_PI;

    radarmeas_covariance = radarmeas_covariance + (weights_(i) * (Z_diff) * (Z_diff.transpose()));
  }

  // Measurement noise covariance matrix
  MatrixXd radar_noise_cov(3, 3);
  radar_noise_cov << std_radr_*std_radr_, 0, 0,
                     0, std_radphi_*std_radphi_, 0,
                     0, 0, std_radrd_*std_radrd_;
  // Adding the radar measurement linear noise
  radarmeas_covariance = radarmeas_covariance + radar_noise_cov;

  // Cross-correlation between measurement & state spaces sigma points
  for (int i=0; i<(2*n_aug_ +1); i++)
  {
    Z_diff = Zsig.col(i) - radarmeas_mean;

    // Phi angle normalization
    if(Z_diff(1) > M_PI) Z_diff(1) -= 2.*M_PI;
    if(Z_diff(1) < -M_PI) Z_diff(1) += 2.*M_PI;

    Tsig_cross = Tsig_cross + (weights_(i) * (Xsig_pred_.col(i) - x_) * (Z_diff.transpose()));
  }

  // State mean & covariance update due to new Radar measurements
  // 1- Kalman Gain calculation
  MatrixXd KG = MatrixXd::Zero(5,3);
  KG = Tsig_cross * radarmeas_covariance.inverse();

  // Measurement state delta
  VectorXd meas_diff = VectorXd::Zero(3);
  meas_diff = radar_meas - radarmeas_mean;

  // Phi angle normalization
  if(meas_diff(1) > M_PI) meas_diff(1) -= 2.*M_PI;
  if(meas_diff(1) < -M_PI) meas_diff(1) += 2.*M_PI;

  // Radar NIS
  NIS_radar_ = meas_diff.transpose() * radarmeas_covariance.inverse() * meas_diff;

  // State Mean Update
  x_ = x_ + KG * (meas_diff);

  // State covariance update
  P_ = P_ - (KG * radarmeas_covariance * KG.transpose());

}
