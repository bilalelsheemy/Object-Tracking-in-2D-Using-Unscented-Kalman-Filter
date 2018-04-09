#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // set it initially to false
  is_initialized_ = false;
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = false;

  //
  time_us_ = 0;
  //
  //time_us_ = 0;

  // initial state vector and augmented state vector
  x_ = VectorXd::Zero(5);
  x_aug = VectorXd::Zero(7);
  // initial covariance matrix and augmented covariance matrix
  P_ = MatrixXd::Zero(5, 5);
  P_aug = MatrixXd::Zero(7, 7);
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;
  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;
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

  // Calculating Sigma points weights
  weights_ = VectorXd::Zero(2*n_aug_+1);
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;

  for (int i=1; i<2*n_aug_+1; i++)
  {  //2n+1 weights
   double weight = 0.5/(n_aug_+lambda_);
   weights_(i) = weight;
  }

}

UKF::~UKF() {}

// Creating UKF instance to use it in the Initialization, Prediction and Updating
UKF ukf_;
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
  float delta_t;;
  // Updating timestamp
  delta_t = (meas_package.timestamp_ - ukf_.time_us_)/1000000.0;

  if (ukf_.is_initialized_ != true)
  {
    //
    ukf_.is_initialized_ = true;

    // Initlaizing delta_t
    delta_t = 0.05;
    // State vector initialization
    ukf_.x_ << meas_package.raw_measurements_(0),
              meas_package.raw_measurements_(1),
              5,
              1.5,
              0.5;

    // Process Noise initialization
    ukf_.P_ << 1,0,0,0,0,
              0,1,0,0,0,
              0,0,1,0,0,
              0,0,0,1,0,
              0,0,0,0,1;

  }

///* For normal case (Not initializing case)

// 5- Prediction step
Prediction(delta_t, meas_package);

// 6- Laser sensor readings update
if(use_laser_ == true && meas_package.sensor_type_ == MeasurementPackage::LASER)
{
  std::cout<<"Hello 1.71"<<std::endl;
  UpdateLidar(meas_package);
}

//// 7- Radar sensor readings update
if(use_radar_ == true && meas_package.sensor_type_ == MeasurementPackage::RADAR)
{
  //std::cout<<"Hello 1.72"<<std::endl;
  UpdateRadar(meas_package);
}

}
/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t, MeasurementPackage meas_package) {
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


  //1- create augmented mean state
  //std::cout<<"Hello 1.1"<<std::endl;
  ukf_.x_aug.fill(0.0);
  ukf_.x_aug.head(5) = ukf_.x_;
  ukf_.x_aug(5) = 0;
  ukf_.x_aug(6) = 0;
  //std::cout<<"Hello 1.11"<<std::endl;
  //1- create augmented covariance matrix
  ukf_.P_aug.fill(0.0);
  ukf_.P_aug.topLeftCorner(5,5) = ukf_.P_;
  ukf_.P_aug(5,5) = ukf_.std_a_*ukf_.std_a_;
  ukf_.P_aug(6,6) = ukf_.std_yawdd_*ukf_.std_yawdd_;


  //std::cout<<"Hello 1.6"<<std::endl;
  //std::cout<<"Hello 1.61"<<std::endl;
  //std::cout<<"Hello 1.62"<<std::endl;
  //std::cout<<"weights_ = "<<ukf_.weights_<<std::endl;

  // Calculating the square root of the process noise
  P_root = ukf_.P_aug.llt().matrixL();
  //std::cout<<"Hello 1.3"<<std::endl;
  // 2- Generate the sigma points for the initialized state
  ukf_.Xsig_aug = MatrixXd::Zero(7, 2*ukf_.n_aug_+1);
  ukf_.Xsig_aug.col(0) =  ukf_.x_aug;

  //std::cout<<"Hello 1.4"<<std::endl;
  for (int i=1; i<ukf_.n_aug_; i++)
  {
    ukf_.Xsig_aug.col(i) = (ukf_.x_aug + (sqrt(ukf_.lambda_+ukf_.n_aug_) * P_root.col(i-1)));
    //std::cout<<"Hello 1.41"<<std::endl;
    ukf_.Xsig_aug.col(i + ukf_.n_aug_) = (ukf_.x_aug - (sqrt(ukf_.lambda_+ukf_.n_aug_) * P_root.col(i-1)));
    //std::cout<<"Hello 1.42"<<std::endl;
  }
  //std::cout<<"Hello 1.5"<<std::endl;



  //std::cout<<"P_root"<<endl<< P_root <<std::endl;

  // 3- Plugging the generated Sigma points to the model to predict the new mean state & its covariance
  ukf_.Xsig_pred_ = MatrixXd::Zero(5, 2*ukf_.n_aug_+1);

  for(int i =0; i < (2*ukf_.n_aug_ +1); i++)
  {
    px = ukf_.Xsig_aug(0, i);
    py = ukf_.Xsig_aug(1, i);
    vel = ukf_.Xsig_aug(2, i);
    yaw = ukf_.Xsig_aug(3, i);
    yawrate = ukf_.Xsig_aug(4, i);
    nu_a = ukf_.Xsig_aug(5, i);
    nu_yawacc = ukf_.Xsig_aug(6, i);
    //std::cout<<"vel = "<<vel <<std::endl;
    //std::cout<<"yaw = "<<yaw <<std::endl;
    //std::cout<<"yawrate = "<<yawrate <<std::endl;
    //std::cout<<"nu_a = "<<nu_a <<std::endl;
    //std::cout<<"nu_yawacc"<<nu_yawacc <<std::endl;
    //std::cout<<"Hello 1.51"<<std::endl;
    //Normalizing yaw
    if(yaw > M_PI) yaw-=2.*M_PI;
    //std::cout<<"Hello 1.634"<<std::endl;
    if (yaw<-M_PI) yaw+=2.*M_PI;
    //std::cout<<"Hello 1.632"<<std::endl;
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

  state_update << px_update,
              py_update,
              vel_update,
              yaw_update,
              yawrate_update;
  //std::cout<<"state_update"<<endl<<state_update <<std::endl;
  //std::cout<<"Hello 1.52"<<std::endl;
  // 4- Predicting new state sigma points
  //ukf_.Xsig_pred_.col(i) = ukf_.Xsig_aug.col(i).head(ukf_.n_x_) + state_update;
  ukf_.Xsig_pred_.col(i) << px_update,
                            py_update,
                            vel_update,
                            yaw_update,
                            yawrate_update;
  }

  //std::cout<<"P_aug"<<endl<< ukf_.P_aug <<std::endl;
  //std::cout<<"x_aug"<<endl<< ukf_.x_aug <<std::endl;
  //std::cout<<"Xsig_aug"<<endl<< ukf_.Xsig_aug <<std::endl;
  //std::cout<<"Xsig_pred_"<<endl<< ukf_.Xsig_pred_ <<std::endl;


   //predicted state mean
  ukf_.x_.fill(0.0);
  for (int i = 0; i < 2 * ukf_.n_aug_ + 1; i++)  //iterate over sigma points
  {
    ukf_.x_ = ukf_.x_+ ukf_.weights_(i) * ukf_.Xsig_pred_.col(i);
  }
  ukf_.time_us_ = meas_package.timestamp_;
  std::cout<<"x_"<<endl<< ukf_.x_ <<std::endl;
  //std::cout<<"Hello 1.63"<<std::endl;
  //predicted state covariance matrix
  ukf_.P_.fill(0.0);
  for (int i = 0; i < 2 * ukf_.n_aug_ + 1; i++)   //iterate over sigma points
  {
    // state difference
    VectorXd x_diff = VectorXd::Zero(3);
    x_diff = ukf_.Xsig_pred_.col(i) - ukf_.x_;
    //std::cout<<"Hello 1.631"<<std::endl;
    //std::cout<<"x_diff(3) = "<<x_diff(3)<<std::endl;
    //std::cout<<"x_diff(1) = "<<ukf_.Xsig_pred_(3, i)<<std::endl;
    //std::cout<<"x_diff(2) = "<<ukf_.x_(3)<<std::endl;
    //angle normalization
    if(x_diff(3) > M_PI) x_diff(3)-=2.*M_PI;
    //std::cout<<"Hello 1.634"<<std::endl;
    if (x_diff(3)< -M_PI) x_diff(3)+=2.*M_PI;
    //std::cout<<"Hello 1.632"<<std::endl;

    ukf_.P_ = ukf_.P_ + ukf_.weights_(i) * x_diff * x_diff.transpose() ;
    //std::cout<<"Hello 1.633"<<std::endl;
  }
  std::cout<<"P_"<<endl<< ukf_.P_ <<std::endl;
  //std::cout<<"Hello 1.64"<<std::endl;
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
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  float px, py;
  px = meas_package.raw_measurements_(0);
  px = meas_package.raw_measurements_(1);

  VectorXd lidar_meas(2);
  lidar_meas << px,
                py;
  //std::cout<<"Hello 1.72"<<std::endl;
  // Laser measurement model
  MatrixXd H_laser(2,5);
  H_laser << 1, 0, 0, 0, 0,
                  0, 1, 0, 0, 0;
  MatrixXd R_laser(2,2);
  R_laser << ukf_.std_laspx_*ukf_.std_laspx_, 0,
              0, ukf_.std_laspy_*ukf_.std_laspy_;

  VectorXd meas_err = lidar_meas - H_laser*ukf_.x_;
  //std::cout<<"Hello 1.73"<<std::endl;
  if(meas_err(1) > M_PI) meas_err(1) -= 2.*M_PI;
  if(meas_err(1) < -M_PI) meas_err(1) += 2.*M_PI;

  // Lidar Measurement Covariance
  MatrixXd S = H_laser*ukf_.P_*H_laser.transpose() + R_laser;
  //std::cout<<"Hello 1.731"<<std::endl;
  MatrixXd Ka_gain = ukf_.P_*H_laser.transpose()*S.inverse();
  //std::cout<<"Hello 1.732"<<std::endl;

  // Update the state as per the new measurements
  ukf_.x_ = ukf_.x_+ (Ka_gain * meas_err);
  ukf_.time_us_ = meas_package.timestamp_;
  MatrixXd I = MatrixXd::Identity(5, 5);
  ukf_.P_ = (I - (Ka_gain * H_laser))*ukf_.P_;
  //std::cout<<"Hello 1.74"<<std::endl;
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
  // Update radar_meas with the new measurements
  //std::cout<<"Xsig_pred_"<< ukf_.Xsig_pred_ <<std::endl;
  double rho_meas = meas_package.raw_measurements_(0);
  double phi_meas = meas_package.raw_measurements_(1);
  double rhorate_meas = meas_package.raw_measurements_(2);

  // Normalizing phi_meas
  //phi_meas = atan2(sin(phi_meas),cos(phi_meas));
  //std::cout<<"Hello 1.73"<<std::endl;
  VectorXd radar_meas = VectorXd::Zero(3);
  radar_meas << rho_meas,
              phi_meas,
              rhorate_meas;
//std::cout<<"Hello 1.74"<<std::endl;
 // Calculate rho, phi, rhorate as per the predicted Sigma points in order to compare them with the new measurement to calculate the measurement covariance matrix then
 double px, py, vel, yaw;
 double rho, phi, rho_rate;
 MatrixXd Zsig_pred = MatrixXd::Zero(3, 2*ukf_.n_aug_+1);

 // Plugging Predicted Sigma points into Non linear measurement model to calculat the predicted Sigma points in the measurements space.
 for (int i=0; i<(2*ukf_.n_aug_ +1); i++)
 {
   //std::cout<<"Hello 1.741"<<std::endl;
   //std::cout<<"Xsig_pred_"<< ukf_.Xsig_pred_ <<std::endl;
   px = ukf_.Xsig_pred_(0, i);
   py = ukf_.Xsig_pred_(1, i);
   vel = ukf_.Xsig_pred_(2, i);
   yaw = ukf_.Xsig_pred_(3, i);

   rho = sqrt((px*px) + (py*py));
   phi = atan2(py,px);
   rho_rate = (((px*cos(yaw)*vel)+(py*sin(yaw)*vel))/(sqrt((px*px) + (py*py))));

   Zsig_pred(0, i) = rho;
   Zsig_pred(1, i) = phi;
   Zsig_pred(2, i) = rho_rate;

 }

  // Calculate Measurement mean and covariance
  VectorXd radarmeas_mean = VectorXd::Zero(3);
  MatrixXd radarmeas_covariance = MatrixXd::Zero(3,3);
  VectorXd Z_diff = VectorXd::Zero(3);
  // Measurement noise covariance
  MatrixXd noise_cov = MatrixXd::Zero(3,3);
  noise_cov << ukf_.std_radr_*ukf_.std_radr_, 0, 0,
              0, ukf_.std_radphi_*ukf_.std_radphi_, 0,
              0, 0, ukf_.std_radrd_*ukf_.std_radrd_;

  // Cross correlation betweem sigma points in both state and measurement spaces
  MatrixXd Tsig_cross = MatrixXd::Zero(5,3);

  // Measurement Mean
  for (int i=0; i<(2*ukf_.n_aug_ +1); i++)
  {
    radarmeas_mean = radarmeas_mean + (ukf_.weights_(i) * Zsig_pred.col(i));
  }

  // Measurement Covariance
  for (int i=0; i<(2*ukf_.n_aug_ +1); i++)
  {
    Z_diff = Zsig_pred.col(i) - radarmeas_mean;

    // Phi angle normalization
    if(Z_diff(1) > M_PI) Z_diff(1) -= 2.*M_PI;
    if(Z_diff(1) < -M_PI) Z_diff(1) += 2.*M_PI;

    radarmeas_covariance = radarmeas_covariance + (ukf_.weights_(i) * (Z_diff) * (Z_diff.transpose()));
  }

  // Adding the radar measurement linear noise
  radarmeas_covariance = radarmeas_covariance + noise_cov;

  // Cross-correlation between measurement & state spaces sigma points
  Z_diff.fill(0.0);
  for (int i=0; i<(2*ukf_.n_aug_ +1); i++)
  {
    Z_diff = Zsig_pred.col(i) - radarmeas_mean;

    // Phi angle normalization
    if(Z_diff(1) > M_PI) Z_diff(1) -= 2.*M_PI;
    if(Z_diff(1) < -M_PI) Z_diff(1) += 2.*M_PI;

    Tsig_cross = Tsig_cross + (ukf_.weights_(i) * (ukf_.Xsig_pred_.col(i) - ukf_.x_) * (Z_diff.transpose()));
  }
  //std::cout<<"Hello 1.75"<<std::endl;

  // State mean & covariance update due to new Radar measurements
  // 1- Kalman Gain calculation
  MatrixXd KG = MatrixXd::Zero(5,3);
  KG = Tsig_cross * radarmeas_covariance.inverse();

  // New state mean
  VectorXd meas_diff = VectorXd::Zero(3);
  meas_diff = radar_meas - radarmeas_mean;

  // Phi angle normalization
  if(meas_diff(1) > M_PI) meas_diff(1) -= 2.*M_PI;
  if(meas_diff(1) < -M_PI) meas_diff(1) += 2.*M_PI;

  ukf_.x_ = ukf_.x_ + KG * (meas_diff);
  ukf_.time_us_ = meas_package.timestamp_;
  // New state covariance
  ukf_.P_ = ukf_.P_ - (KG * radarmeas_covariance * KG.transpose());
  //std::cout<<"Hello 1.76"<<std::endl;

}
