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
    std_a_ = 6;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 6;

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
    //State dimension
    n_x_=5;

    n_aug_=7;

    lambda_=3 - n_aug_;

    is_initialized_=false;

    Xsig_pred_=MatrixXd(n_x_, 2 * n_aug_ + 1);

    time_us_=0;

    weights_ = VectorXd(2*n_aug_+1);

}

UKF::~UKF() {}

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
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/


    if (!is_initialized_) {
        /**
      TODO:
        * Initialize the state ukf_.x_ with the first measurement.
        * Create the covariance matrix.
        * Remember: you'll need to convert radar from polar to cartesian
      coordinates.
      */
        // first measurement
        cout << "UKF: " << endl;

        //Initialize covariance matrix
        P_ <<   0.1,   0.0,    0.0,   0.0,   0.0,
                0.0,   0.1,    0.0,   0.0,   0.0,
                0.0,   0.0,    0.5,   0.0,   0.0,
                0.0,   0.0,    0.0,   0.5,   0.0,
                0.0,   0.0,    0.0,   0.0,   0.5;

        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            /**
        Convert radar from polar to cartesian coordinates and initialize state.
        */
            /**
        Initialize state.
        */

            Eigen::VectorXd x_in_raw = meas_package.raw_measurements_;
            x_(0) = x_in_raw(0) * cos(x_in_raw(1));
            x_(1) = x_in_raw(0) * sin(x_in_raw(1));
            x_(2) = x_in_raw(2);
            x_(3) = 0;
            x_(4) = 0;


        } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            /**
        Initialize state.
        */
            Eigen::VectorXd x_in_raw = meas_package.raw_measurements_;
            x_(0)= x_in_raw (0);
            x_(1)= x_in_raw (1);
            x_(2) = 0;
            x_(3) = 0;
            x_(4) = 0;

        }
        is_initialized_ = true;
        cout << x_<< endl;
        cout << "Initialization done: " << endl;

    }else {
        double current_time_us=meas_package.timestamp_;
        double delta_t=current_time_us-time_us_;

        Prediction(delta_t/1000000);
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
            UpdateRadar(meas_package);
        }else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
            UpdateLidar(meas_package);
        }
    }
    time_us_ = meas_package.timestamp_;
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
    //augmented mean vector
    VectorXd x_aug= VectorXd(7);

    //augmented state covariance
    MatrixXd P_aug= MatrixXd(7, 7);

    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

    //Update Augmented values
    //create augmented mean state
    x_aug.fill(0);
    x_aug.head(5)=x_;

    //create augmented covariance matrix
    P_aug.fill(0);
    P_aug.topLeftCorner(n_x_, n_x_)=P_;
    P_aug(5,5)=std_a_*std_a_;
    P_aug(6,6)=std_yawdd_*std_yawdd_;
    //std::cout << "P_aug = " << std::endl << P_aug << std::endl;

    //create square root matrix
    MatrixXd A= P_aug.llt().matrixL();

    //create augmented sigma points
    Xsig_aug.col(0)=x_aug;
    for (int i=0; i<n_aug_; i++){
        Xsig_aug.col(i+1)=x_aug+sqrt(lambda_+n_aug_)*A.col(i);
        Xsig_aug.col(i+1+n_aug_)=x_aug-sqrt(lambda_+n_aug_)*A.col(i);
    }
    //predict sigma points

    //extract values for better readability
    for (int i=0; i< 2 * n_aug_ + 1; i++ ){
        double p_x = Xsig_aug(0,i);
        double p_y = Xsig_aug(1,i);
        double v = Xsig_aug(2,i);
        double yaw = Xsig_aug(3,i);
        double yawd = Xsig_aug(4,i);
        double nu_a = Xsig_aug(5,i);
        double nu_yawdd = Xsig_aug(6,i);

        double p_x_pred=p_x;
        double p_y_pred=p_y;
        double v_pred=v;
        double yaw_pred=yaw;
        double yawd_pred=yawd;

        //avoid division by zero
        if (yawd!=0){
            p_x_pred+=v/yawd*(sin(yaw+delta_t*yawd)-sin(yaw));
            p_y_pred+=v/yawd*(-cos(yaw+delta_t*yawd)+cos(yaw));

        }else{
            p_x_pred+=v*cos(yaw)*delta_t;
            p_y_pred+=v*sin(yaw)*delta_t;
        }
        yaw_pred+=yawd*delta_t;

        //adding noise
        p_x_pred+=0.5*delta_t*delta_t*cos(yaw)*nu_a;
        p_y_pred+=0.5*delta_t*delta_t*sin(yaw)*nu_a;
        v_pred+=delta_t*nu_a;
        yaw_pred+=0.5*delta_t*delta_t*nu_yawdd;
        yawd_pred+=delta_t*nu_yawdd;

        //write predicted sigma points into right column
        Xsig_pred_(0, i)= p_x_pred;
        Xsig_pred_(1, i)= p_y_pred;
        Xsig_pred_(2, i)= v_pred;
        Xsig_pred_(3, i)= yaw_pred;
        Xsig_pred_(4, i)= yawd_pred;

    }

    //-----------------
    //set weights
    weights_(0)=lambda_/(lambda_+n_aug_);
    for (int i=1; i<2*n_aug_+1; i++ ){
        weights_(i)=0.5/(n_aug_+lambda_);
    }
    //predict state mean
    x_.fill(0);
    for (int i=0; i<2*n_aug_+1; i++ ){
        x_=x_+weights_(i)*Xsig_pred_.col(i);
    }
    P_.fill(0);
    //predict state covariance matrix
    for (int i=0; i<2*n_aug_+1; i++ ){
        VectorXd v=Xsig_pred_.col(i)-x_;
        //angle normalization
        while (v(3)> M_PI)
            v(3)-=2.*M_PI;

        while (v(3)<-M_PI)
            v(3)+=2.*M_PI;

       P_ = P_ + weights_(i) * v * v.transpose() ;

    }

    cout << x_<< endl;
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
    //set measurement dimension, radar can measure r, phi, and r_dot
    int n_z = 2;

    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);

    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z,n_z);

    //transform sigma points into measurement space
    for (int i=0; i< 2 * n_aug_ + 1; i++){
        double px=Xsig_pred_(0, i);
        double py=Xsig_pred_(1, i);

        Zsig(0,i)=px;
        Zsig(1,i)=py;

    }
    //calculate mean predicted measurement
    z_pred.fill(0.0);
    for (int i=0; i < 2*n_aug_+1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    //calculate measurement covariance matrix S
    //measurement covariance matrix S
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    //add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z,n_z);
    R <<    std_laspx_*std_laspx_, 0,
            0, std_laspy_*std_laspy_;
    S = S + R;

    // ---------------------------------------update
    //calculate cross correlation matrix

    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);

    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }
    //calculate Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    //update state mean and covariance matrix
    //residual
    VectorXd z = VectorXd(n_z);
    z(0)= meas_package.raw_measurements_(0);
    z(1)= meas_package.raw_measurements_(1);

    VectorXd z_diff = z - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();
    cout << "UpdateLidar done " << endl;
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
    //set measurement dimension, radar can measure r, phi, and r_dot
    int n_z = 3;

    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);

    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z,n_z);

    //transform sigma points into measurement space
    for (int i=0; i< 2 * n_aug_ + 1; i++){
        double px=Xsig_pred_(0, i);
        double py=Xsig_pred_(1, i);
        double v=Xsig_pred_(2, i);
        double yaw=Xsig_pred_(3, i);

        double r= sqrt(px*px+py*py);
        double phi= atan2(py,px);
        double r_dot=(px*cos(yaw)*v+py*sin(yaw)*v)/r;

        Zsig(0,i)=r;
        Zsig(1,i)=phi;
        Zsig(2,i)=r_dot;
    }
    //calculate mean predicted measurement
    z_pred.fill(0.0);
    for (int i=0; i < 2*n_aug_+1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    //calculate measurement covariance matrix S
    //measurement covariance matrix S
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    //add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z,n_z);
    R <<    std_radr_*std_radr_, 0, 0,
            0, std_radphi_*std_radphi_, 0,
            0, 0,std_radrd_*std_radrd_;
    S = S + R;

    // ---------------------------------------update
    //calculate cross correlation matrix

    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);

    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }
    //calculate Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    //update state mean and covariance matrix
    //residual
    VectorXd z = VectorXd(n_z);
    z(0)= meas_package.raw_measurements_(0);
    z(1)= meas_package.raw_measurements_(1);
    z(2)= meas_package.raw_measurements_(2);

    VectorXd z_diff = z - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();
}
