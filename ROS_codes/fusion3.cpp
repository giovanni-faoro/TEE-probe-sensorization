#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/QuaternionStamped.h"    //to read quaternion
#include "geometry_msgs/Vector3Stamped.h"       //to read position, free-acceleration, angular velocity
#include "geometry_msgs/Pose.h"
//to use linear algebra
#include "Eigen/Dense"
#include <math.h>

#define PI 3.14159265

using namespace std;

/**
 * This node manage to receive the upsampled data at 100 Hz defining the state vector as x = (qw,qx,qy,qz,wx,wy,wz,px,py,pz,vx,vy,vz,ax,ay,az)
 * An Extended Kalman Filter is implemented in order to fuse the information coming from the XSens and Aurora sensor in order to provide
 * an improved estimate on catheter tip pose (position + orientation)
 * Also the case in which Aurora data become unreliable (i.e. sensor visibility to false because outside of the field or strong EM interferences) is taken into account
 * by exploiting only IMU data in the EKF algorithm. As soon as the Aurora data become again reliable, the fusion is resumed.
 */

//First define XSens, Aurora and Mine class to hold the acquired data
class XSens
{
  float q_w, q_x, q_y, q_z, w_x, w_y, w_z, acc_x, acc_y, acc_z, mag_x, mag_y, mag_z;
  bool initialized = false;         //just tells if we have received data, or we've just created an un-initialized object
public:
  void set_quat(float a, float b, float c, float d)
  {
    q_w = a;
    q_x = b;
    q_y = c;
    q_z = d;
    initialized = true;
  }
  geometry_msgs::QuaternionStamped get_quat()
  {
    geometry_msgs::QuaternionStamped msg;
    msg.quaternion.w = q_w;
    msg.quaternion.x = q_x;
    msg.quaternion.y = q_y;
    msg.quaternion.z = q_z;
    return msg;
  }
  void set_ang_vel(float a, float b, float c)
  {
    w_x = a;
    w_y = b;
    w_z = c;
    initialized = true;
  }
  geometry_msgs::Vector3Stamped get_ang_vel()
  {
    geometry_msgs::Vector3Stamped msg;
    msg.vector.x = w_x;
    msg.vector.y = w_y;
    msg.vector.z = w_z;
    return msg;
  }
  void set_acc(float a, float b, float c)
  {
    acc_x = a;
    acc_y = b;
    acc_z = c;
    initialized = true;
  }
  geometry_msgs::Vector3Stamped get_acc()
  {
    geometry_msgs::Vector3Stamped msg;
    msg.vector.x = acc_x;
    msg.vector.y = acc_y;
    msg.vector.z = acc_z;
    return msg;
  }
  void set_mag(float a, float b, float c)
  {
    mag_x = a;
    mag_y = b;
    mag_z = c;
    initialized = true;
  }
  geometry_msgs::Vector3Stamped get_mag()
  {
    geometry_msgs::Vector3Stamped msg;
    msg.vector.x = mag_x;
    msg.vector.y = mag_y;
    msg.vector.z = mag_z;
    return msg;
  }
  bool is_initialized()
  {
      return initialized;
  }
};
class Aurora
{
  float q_w, q_x, q_y, q_z, p_x, p_y, p_z;
  bool initialized = false;         //just tells if we have received data, or we've just created an un-initialized object
  bool visible = false;
public:
  void set_quat(float a, float b, float c, float d)
  {
    q_w = a;
    q_x = b;
    q_y = c;
    q_z = d;
    initialized = true;
  }
  geometry_msgs::QuaternionStamped get_quat()
  {
    geometry_msgs::QuaternionStamped msg;
    msg.quaternion.w = q_w;
    msg.quaternion.x = q_x;
    msg.quaternion.y = q_y;
    msg.quaternion.z = q_z;
    return msg;
  }
  void set_pos(float a, float b, float c)
  {
    p_x = a;
    p_y = b;
    p_z = c;
    initialized = true;
  }

  geometry_msgs::Vector3Stamped get_pos()
  {
    geometry_msgs::Vector3Stamped msg;
    msg.vector.x = p_x;
    msg.vector.y = p_y;
    msg.vector.z = p_z;
    return msg;
  }
  void set_vis(bool a)
  {
      visible = a;
  }
  bool is_initialized()
  {
      return initialized;
  }
  bool is_visible()
  {
      return visible;
  }
};
class Mine
{
  float q_w, q_x, q_y, q_z;
  bool initialized = false;         //just tells if we have received data, or we've just created an un-initialized object
public:
  void set_quat(float a, float b, float c, float d)
  {
    q_w = a;
    q_x = b;
    q_y = c;
    q_z = d;
    initialized = true;
  }
  geometry_msgs::QuaternionStamped get_quat()
  {
    geometry_msgs::QuaternionStamped msg;
    msg.quaternion.w = q_w;
    msg.quaternion.x = q_x;
    msg.quaternion.y = q_y;
    msg.quaternion.z = q_z;
    return msg;
  }
  bool is_initialized()
  {
      return initialized;
  }
};

//Create a CatheterTip object to update the measurements
XSens inertial;
Aurora electromagnetic;
Mine custom;

//Callback functions to read from topics
//XSens
void quaternionXSensCallback(const geometry_msgs::QuaternionStamped& msg)
{
    inertial.set_quat(msg.quaternion.w,msg.quaternion.x,msg.quaternion.y,msg.quaternion.z);
}
void accelerationXSensCallback(const geometry_msgs::Vector3Stamped& msg)
{
    inertial.set_acc(msg.vector.x,msg.vector.y,msg.vector.z);
}
void angularVelocityXSensCallback(const geometry_msgs::Vector3Stamped& msg)
{
    inertial.set_ang_vel(msg.vector.x,msg.vector.y,msg.vector.z);
}
void magnetometerXSensCallback(const geometry_msgs::Vector3Stamped& msg)
{
    inertial.set_mag(msg.vector.x,msg.vector.y,msg.vector.z);
}
//Mine
void quaternionXSensModCallback(const geometry_msgs::QuaternionStamped& msg)
{
    custom.set_quat(msg.quaternion.w,msg.quaternion.x,msg.quaternion.y,msg.quaternion.z);
}
//Aurora
void quaternionAuroraCallback(const geometry_msgs::QuaternionStamped& msg)
{
    electromagnetic.set_quat(msg.quaternion.w,msg.quaternion.x,msg.quaternion.y,msg.quaternion.z);
}
void positionAuroraCallback(const geometry_msgs::Vector3Stamped& msg)
{
    electromagnetic.set_pos(msg.vector.x,msg.vector.y,msg.vector.z);
}
void visibilityAuroraCallback(const std_msgs::Bool& msg)
{
    electromagnetic.set_vis(msg.data);
}

Eigen::Vector4d quaternionProduct(Eigen::Vector4d q, Eigen::Vector4d p)
{
    //This function implements the quaternion product q*p
    //quaternions are in the form x,y,z,w
    Eigen::Vector4d product = Eigen::Vector4d::Zero();
    product(0) = q(3)*p(0)+p(3)*q(0)+q(1)*p(2)-q(2)*p(1);
    product(1) = q(3)*p(1)+p(3)*q(1)+q(2)*p(0)-q(0)*p(2);
    product(2) = q(3)*p(2)+q(2)*p(3)+q(0)*p(1)-q(1)*p(0);
    product(3) = q(3)*p(3)-q(0)*p(0)-q(1)*p(1)-q(2)*p(2);
    return product;
}
Eigen::Vector4d quaternionInverse(Eigen::Vector4d q)
{
    //This function compute the quaternion inverse q^-1
    Eigen::Vector4d inverse = Eigen::Vector4d::Zero();
    inverse << -q(0),-q(1),-q(2),q(3);
    if(inverse.norm() != 0)
    {
        inverse.normalize();
    }
    return inverse;
}
double quaternionNorm(Eigen::Vector4d q)
{
    //This function compute the norm of a 4D vector
    double norm = sqrt(q(0)*q(0)+q(1)*q(1)+q(2)*q(2)+q(3)*q(3));
    return norm;
}
Eigen::Vector3d quaternionRotation(Eigen::Vector4d q, Eigen::Vector3d v)
{
    //This function implements the rotation of 3D vector v by quaternion q in global reference
    Eigen::Vector4d v_q = Eigen::Vector4d::Zero();
    v_q << v(0),v(1),v(2),0;
    Eigen::Vector4d product = Eigen::Vector4d::Zero();
    product = quaternionProduct(q,quaternionProduct(v_q,quaternionInverse(q)));
    Eigen::Vector3d v_rot = Eigen::Vector3d::Zero();
    v_rot << product(0),product(1),product(2);
    return v_rot;
}
Eigen::Vector3d quaternionRotationLocal(Eigen::Vector4d q, Eigen::Vector3d v)
{
    //This function implements the rotation of 3D vector v by quaternion q in local reference
    Eigen::Vector4d v_q = Eigen::Vector4d::Zero();
    v_q << v(0),v(1),v(2),0;
    Eigen::Vector4d product = Eigen::Vector4d::Zero();
    product = quaternionProduct(quaternionInverse(q),quaternionProduct(v_q,q));
    Eigen::Vector3d v_rot = Eigen::Vector3d::Zero();
    v_rot << product(0),product(1),product(2);
    return v_rot;
}
Eigen::Matrix4d matrixR(Eigen::Vector3d q)
{
    //This function compute the matrix R for a 3D vector (which in quaternion representation is a quaternion with final element w=0)
    Eigen::Matrix4d result = Eigen::Matrix4d::Zero();
    result(0,0) = 0;
    result(0,1) = -q(2);
    result(0,2) = q(1);
    result(0,3) = q(0);
    result(1,0) = q(2);
    result(1,1) = 0;
    result(1,2) = -q(0);
    result(1,3) = q(1);
    result(2,0) = -q(1);
    result(2,1) = q(0);
    result(2,2) = 0;
    result(2,3) = q(2);
    result(3,0) = -q(0);
    result(3,1) = -q(1);
    result(3,2) = -q(2);
    result(3,3) = 0;
    return result;
}
Eigen::Matrix4d matrixL(Eigen::Vector3d q)
{
    //This function compute the matrix L for a 3D vector (which in quaternion representation is a quaternion with final element w=0)
    Eigen::Matrix4d result = Eigen::Matrix4d::Zero();
    result(0,0) = 0;
    result(0,1) = q(2);
    result(0,2) = -q(1);
    result(0,3) = q(0);
    result(1,0) = -q(2);
    result(1,1) = 0;
    result(1,2) = q(0);
    result(1,3) = q(1);
    result(2,0) = q(1);
    result(2,1) = -q(0);
    result(2,2) = 0;
    result(2,3) = q(2);
    result(3,0) = -q(0);
    result(3,1) = -q(1);
    result(3,2) = -q(2);
    result(3,3) = 0;
    return result;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "fusion");
    ros::NodeHandle n;
    bool first = true;

    //Listen to upsample node topics to update the catheter tip pose
    ros::Subscriber acc_sub = n.subscribe("/XSens/acceleration", 1000, accelerationXSensCallback);
    ros::Subscriber quat_sub = n.subscribe("/XSens/quaternion", 1000, quaternionXSensCallback);
    ros::Subscriber my_quat_sub = n.subscribe("/XSensMod/quaternion", 1000, quaternionXSensModCallback);
    ros::Subscriber ang_vel_sub = n.subscribe("/XSens/angular_velocity", 1000, angularVelocityXSensCallback);
    ros::Subscriber mag_sub = n.subscribe("/XSens/magnetometer", 1000, magnetometerXSensCallback);
    ros::Subscriber em_pos_sub = n.subscribe("/Aurora/position", 1000, positionAuroraCallback);
    ros::Subscriber em_quat_sub = n.subscribe("/Aurora/quaternion", 1000, quaternionAuroraCallback);
    ros::Subscriber em_bool_sub = n.subscribe("/Aurora/visibility", 1000, visibilityAuroraCallback);


    //Send out the new estimate for catheter tip pose
    ros::Publisher quat_pub = n.advertise<geometry_msgs::QuaternionStamped>("/EKF/quaternion", 1000);
    ros::Publisher pos_pub = n.advertise<geometry_msgs::Vector3Stamped>("/EKF/position", 1000);
    ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("/EKF/pose", 1000);

    //Check on the variables for magnetic interference detection
    ros::Publisher Mag_pub = n.advertise<geometry_msgs::Vector3Stamped>("/EKF/mag_acc_norm", 1000);

    //Parameter Initialization for LKF
    int freq = 100;                                             //sampling rate Hz
    double dt = (double) 1/freq;                                //delta t for acquisition frequency
    //Initialization of state vector
    Eigen::VectorXd state = Eigen::VectorXd::Zero(16);          //The state is in the form x = (q,w,p,v,a)
    Eigen::VectorXd quaternion = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd angular_vel = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd position = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd velocity = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd acceleration = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd magnetometer = Eigen::VectorXd::Zero(3);
    double mag_norm_dev;
    //Initialization of prediction vector
    Eigen::VectorXd predict_state = Eigen::VectorXd::Zero(16);  //The state is in the form x = (q,w,p,v,a)
    Eigen::VectorXd predict_quat = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd predict_ang_vel = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd predict_pos = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd predict_vel = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd predict_acc = Eigen::VectorXd::Zero(3);
    //Initialization of measurement vector
    geometry_msgs::QuaternionStamped quat;
    geometry_msgs::QuaternionStamped quat_imu;
    geometry_msgs::Vector3Stamped acc;
    geometry_msgs::Vector3Stamped ang_vel;
    geometry_msgs::Vector3Stamped mag;
    geometry_msgs::Vector3Stamped pos;
    Eigen::VectorXd measurement = Eigen::VectorXd::Zero(17);            //The measurement is in the form z = (q_em,p_em,q_imu,w_imu,a_imu)
    Eigen::VectorXd measurement_c = Eigen::VectorXd::Zero(10);          //The corrupetd measurement is in the form z = (q_imu,w_imu,a_imu)
    Eigen::VectorXd meas_quat_em = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd meas_quat_imu = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd meas_ang_vel_imu = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd meas_pos_em = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd meas_acc_imu = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd meas_mag = Eigen::VectorXd::Zero(3);
    //Vector and matrix declaration
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(16,16);           //Process Model matrix F
    Eigen::MatrixXd F_quat = Eigen::MatrixXd::Zero(4,4);        //Process Model matrix F for the quaternion
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(17,16);           //Measurement Model matrix H
    Eigen::MatrixXd H_hat = Eigen::MatrixXd::Zero(17, 16);      //Measurement Model matrix H for the incremental filter
    Eigen::MatrixXd H_c = Eigen::MatrixXd::Zero(10,16);         //Measurement Model matrix H for the corrupted case
    Eigen::MatrixXd H_c_hat = Eigen::MatrixXd::Zero(10, 16);    //Measurement Model matrix H for the corrupted case for incremental filter
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(16,17);           //Kalman Gain matrix K
    Eigen::MatrixXd K_c = Eigen::MatrixXd::Zero(16,10);         //Kalman Gain matrix K for the corrupted case
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(17,17);           //Innovation Covariance matrix S
    Eigen::MatrixXd S_c = Eigen::MatrixXd::Zero(10,10);         //Innovation Covariance matrix S for the corrupted case
    Eigen::MatrixXd I16 = Eigen::MatrixXd::Identity(16,16);     //16x16 Identity matrix
    Eigen::MatrixXd I4 = Eigen::MatrixXd::Identity(4,4);        //4x4 Identity matrix
    Eigen::MatrixXd I3 = Eigen::MatrixXd::Identity(3,3);        //3x3 Identity matrix
    Eigen::VectorXd innovation = Eigen::VectorXd::Zero(17);     //innovation vector
    Eigen::VectorXd innovation_c = Eigen::VectorXd::Zero(10);   //innovation vector for the corrupted case
    //Definition of Model Covariance matrix P
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(16,16);       //Assume to know the initial state with small variance
    double var_p = 0.0001;
    P = var_p*P;
    //Definition of Model Error Covariance matrix Q
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(16,16);
    //considering values from "Research on Intraoperative Organ Tracking"
    double var = 0.00003;
    Q = var*Q;
    //Definition of Measurament Error Covariance Matrix R
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(17,17);
    Eigen::MatrixXd R_c = Eigen::MatrixXd::Zero(10,10);
    double noise_density = 0;
    //Quaternion variance (Aurora)
    double ang_RMS = 0.30;                                                  //°
    ang_RMS = ang_RMS*(PI/180);                                             //rad
    double var_eul_AU = ang_RMS*ang_RMS;
    //Position variance (Aurora)
    double pos_RMS = 0.48;                                                  //mm
    double var_pos = pos_RMS*pos_RMS;
    R.block(4,4,3,3) = var_pos*I3;
    //Quaternion variance (XSens)
    //std on roll pitch and yaw are 0.5° 0.5° 1.5°
    double var_roll_IMU = pow(0.5,2)*pow((PI/180),2);
    double var_pitch_IMU = pow(0.5,2)*pow((PI/180),2);
    double var_yaw_IMU = pow(1.5,2)*pow((PI/180),2);
    //Angular velocity variance (XSens)
    noise_density = 0.007;                                                  //°/s/sqrt(Hz)
    double var_ang_vel = (double)noise_density*sqrt(freq)*(PI/180);         //rad/s
    var_ang_vel = pow(var_ang_vel,2);
    R.block(11,11,3,3) = var_ang_vel*I3;
    //Accelerometer variance (XSens)
    noise_density = 120;                                                    //microg/sqrt(Hz)
    double var_acc = (double)noise_density*pow(10,-3)*9.81*sqrt(freq);      //mm/s^2
    var_acc = pow(var_acc,2);
    R.block(14,14,3,3) = var_acc*I3;

    //registration quaternion
    Eigen::VectorXd ENU_q_FG = Eigen::VectorXd::Zero(4);
    //ENU_q_FG << -0.0015,-0.6717,0.1441,0.7267;                            //Pisa WetLab experiments
    //ENU_q_FG << 0.8124,0.5783,-0.0740,0.0098;                             //KUL experiments
    ENU_q_FG << -0.6674,-0.3003,0.6005,0.3221;                               //Pisa experiments
    Eigen::VectorXd IMU_q_COIL = Eigen::VectorXd::Zero(4);
    //IMU_q_COIL << -0.3780,0.6298,0.5920,0.3317;
    //IMU_q_COIL << -0.3781,0.5560,0.5398,0.5065;
    IMU_q_COIL << -0.3258,0.5190,0.7005,0.3657;
    //composite quaternions for registration
    Eigen::VectorXd quat_COIL = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd quat_ENU = Eigen::VectorXd::Zero(4);

    double yaw, pitch, roll = 0;
    double sinr_cosp;
    double cosr_cosp;
    double sinp;
    double siny_cosp;
    double cosy_cosp;
    double var_qx, var_qy, var_qz, var_qw;
    double a, b;


    //variables for magnetic interference detection
    int window_size = 50;                                                   //sample size of the moving window
    Eigen::MatrixXd MagFieldMagnitude = Eigen::MatrixXd::Zero(2,50);         //vector of magnetic field norm
    Eigen::MatrixXd measurement_buffer = Eigen::MatrixXd::Zero(17,50);      //matrix of old measurement, the measurement is in the form z = (q_em,p_em,q_imu,w_imu,a_imu)
    Eigen::VectorXd measurement_lastgood = Eigen::VectorXd::Zero(17);       //last good measurement, the measurement is in the form z = (q_em,p_em,q_imu,w_imu,a_imu)
    Eigen::VectorXd measurement_increment = Eigen::VectorXd::Zero(17);      //incremental measurement, the measurement is in the form z = (q_em,p_em,q_imu,w_imu,a_imu)
    double acc_threshold = 1500;                                               //mm/s^2, threshold on acceleration norm deviation
    double mag_threshold = 2;                                                  //threshold on magnetic field norm slope
    Eigen::VectorXd mag_slope = Eigen::VectorXd::Zero(2);                   //slope of magnetic field norm
    int counts = 0;                                                         //counts of magnetic field peaks
    bool good = false;                                                      //the EM data are good to be used
    bool increment = false;                                                 //the EM data should be used as increments
    Eigen::VectorXd bad = Eigen::VectorXd::Zero(2);                         //the EM data should be discarded
    Eigen::VectorXd time = Eigen::VectorXd::Zero(50);                       //time vector
    time << 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49;
    time = time*dt;
    int iteration = 0;
    Eigen::VectorXd ones = Eigen::VectorXd::Ones(50);
    int loop_id = 0;
    double sum1 = 0;
    double sum2 = 0;
    double sum3 = 0;
    double sum4 = 0;
    double sum5 = 0;
    bool reduced = false;                                                   //check value for complete or reduced filter
    int index = 0;                                                          //index to select data from measurement buffer when EM is not visible
    double acc_x = 0;                                                       //acceleration on x measured from EM
    double acc_y = 0;                                                       //acceleration on y measured from EM
    double acc_z = 0;                                                       //acceleration on z measured from EM
    double acc_deviation = 0;                                               //acceleration deviation from EM to IMU
    int flag = 0;


    //Set the rate of the while loop, which coincide with the rate of the output data from this synchronizer node
    ros::Rate loop_rate(100);                                   //provide output data at 100Hz
    while (ros::ok())
    {
        //Sensor fusion is carried out considering an Extended Kalman Filter
        //A new artificial sensor providing a complete measurement vector with redundant quaternion estimate is used in order to fuse Aurora and XSens data

        //Before to start fusing data we wait for both the sensor objects to be initialized
        if(electromagnetic.is_initialized() && inertial.is_initialized())
        {

            //Before to start fusion we still need to wait for moving window initialization
            if(loop_id<window_size)
            {
                //stack measurement in the moving window
                quat = electromagnetic.get_quat();
                meas_quat_em << quat.quaternion.x, quat.quaternion.y, quat.quaternion.z, quat.quaternion.w;
                pos = electromagnetic.get_pos();
                meas_pos_em << pos.vector.x, pos.vector.y, pos.vector.z;
                quat_imu = inertial.get_quat();
                //register IMU quaternion
                meas_quat_imu << quat_imu.quaternion.x, quat_imu.quaternion.y, quat_imu.quaternion.z, quat_imu.quaternion.w;
                meas_quat_imu = quaternionProduct(ENU_q_FG,quaternionProduct(meas_quat_imu,IMU_q_COIL));
                //register angular velocity and free acceleration
                ang_vel = inertial.get_ang_vel();
                meas_ang_vel_imu << ang_vel.vector.x, ang_vel.vector.y, ang_vel.vector.z;
                //compute the composite quaternion from IMU to FG, passing through COIL RF
                quat_COIL = quaternionProduct(IMU_q_COIL,quaternionInverse(quaternion));
                meas_ang_vel_imu = quaternionRotationLocal(IMU_q_COIL,meas_ang_vel_imu);
                acc = inertial.get_acc();
                meas_acc_imu << acc.vector.x, acc.vector.y, acc.vector.z;
                meas_acc_imu = quaternionRotation(quaternionInverse(ENU_q_FG),meas_acc_imu);
                measurement << meas_quat_em, meas_pos_em, meas_quat_imu, meas_ang_vel_imu, meas_acc_imu;
                measurement_buffer.block(0,loop_id,17,1) = measurement;
                //stack magnitude norm data in the moving window
                mag = inertial.get_mag();
                meas_mag << mag.vector.x, mag.vector.y, mag.vector.z;
                MagFieldMagnitude(0,loop_id) = sqrt(pow(meas_mag(0),2)+pow(meas_mag(1),2)+pow(meas_mag(2),2));
                loop_id = loop_id+1;
            }
            else
            {
                if(loop_id==window_size)
                {
                    time = time + dt*iteration*ones;
                    //before to move on, let's compute the first mag slope
                    for(int i = 0; i<window_size; i++)
                    {
                        sum1 = sum1 + (double)time(i)*MagFieldMagnitude(0,i);
                        sum2 = sum2 + (double)time(i);
                        sum3 = sum3 + (double)MagFieldMagnitude(0,i);
                        sum4 = sum4 + pow((double)time(i),2);
                    }
                    mag_slope(0) = mag_slope(1);
                    mag_slope(1) = (sum1 - ((sum2*sum3)/window_size))/(sum4 - (pow(sum2,2)/window_size));
                    sum1 = 0;
                    sum2 = 0;
                    sum3 = 0;
                    sum4 = 0;
                    loop_id = loop_id+1;
                    iteration = iteration +1;
                    measurement_increment = measurement_buffer.block(0,1,17,1)-measurement_buffer.block(0,0,17,1);
                }
                //The Kalman filter implemented here is an Extended Kalman Filter, since the process model equations are not linear in the state (to update quaternion we use products
                //between quaternion and angular velocity, which are both elements of the state
                //For an appropriate sensor fusion the rotation matrix from inertial sensor reference to electromagnetic one should be computed and used in order to map all the measurements in
                //the same reference. The rotation matrix should be use to map the quaternion but also the acceleration and angular velocity vectors.
                if((first)&&(inertial.is_initialized())&&(electromagnetic.is_initialized()))
                {
                    //define the initial state x_0 (should be a ground truth value and not data retrieved from sensors)
                    quat = electromagnetic.get_quat();
                    quaternion << quat.quaternion.x, quat.quaternion.y, quat.quaternion.z, quat.quaternion.w;
                    ang_vel = inertial.get_ang_vel();
                    angular_vel << ang_vel.vector.x, ang_vel.vector.y, ang_vel.vector.z;
                    //register angular velocity
                    //compute the composite quaternion from IMU to FG, passing through COIL RF
                    quat_COIL = quaternionProduct(IMU_q_COIL,quaternionInverse(quaternion));
                    //we should use the local RF angular velocity, so juts IMU_q_COIL
                    angular_vel = quaternionRotationLocal(IMU_q_COIL,angular_vel);
                    pos = electromagnetic.get_pos();
                    position << pos.vector.x, pos.vector.y, pos.vector.z;
                    velocity << 0, 0, 0;
                    acc = inertial.get_acc();
                    acceleration << acc.vector.x, acc.vector.y, acc.vector.z;       //m/s^2
                    acceleration = acceleration*pow(10,3);                          //mm/s^2
                    //register free acceleration
                    acceleration = quaternionRotation(quaternionInverse(ENU_q_FG),acceleration);
                    state << quaternion, angular_vel, position, velocity, acceleration;
                    first = false;
                }
                //---Prediction Step---
                //Definition of Process Model matrix F
                //Quaternion update
                double ang_vel_norm = angular_vel.norm();
                if(ang_vel_norm > pow(10,-1))
                {
                    F_quat = cos((ang_vel_norm*dt)/2)*I4 + (1/ang_vel_norm)*sin((ang_vel_norm*dt)/2)*matrixL(angular_vel);
                }
                else
                {
                    //if the norm of angular velocity tends to zero, then we fall into the case cos(0) = 1 and lim_x->0 sin(ax)/x = a so
                    F_quat = I4 + (dt/2)*matrixL(angular_vel);
                }
                predict_quat = F_quat*quaternion;
                //Angular velocity update
                predict_ang_vel = I3*angular_vel;            //By using an identity matrix we are basically exploiting the Kalman filter for low-pass filtering
                //Position update
                predict_pos = position + dt*I3*velocity + (pow(dt,2)/2)*I3*acceleration;
                //Velocity update
                predict_vel = velocity + dt*I3*acceleration;
                //Acceleration update
                predict_acc = I3*acceleration;              //By using an identity matrix we are basically exploiting the Kalman filter for low-pass filtering
                predict_state << predict_quat, predict_ang_vel, predict_pos, predict_vel, predict_acc;
                //Since we are using an EKF we need to define the Jacobian matrix of f(x,u)
                //Let's just consider the case of small angular velocity (this should be our target case)
                //quaternion part
                if (ang_vel_norm > pow(10, -1))
                {
                    //derivative component of cosine part
                    a = -(1/ang_vel_norm)*sin((ang_vel_norm*dt)/2)*(dt/2);
                    //derivative component of sinc part
                    b = (cos(ang_vel_norm*dt/2)*(ang_vel_norm*dt)/2 - sin(ang_vel_norm*dt/2))*(1/pow(ang_vel_norm,3));
                    //Jacobian matrix elements
                    F(0, 0) = cos(ang_vel_norm*dt/2);
                    F(0, 1) = angular_vel(2)*sin(ang_vel_norm*dt/2)/ang_vel_norm;
                    F(0, 2) = -angular_vel(1)*sin(ang_vel_norm*dt/2)/ang_vel_norm;
                    F(0, 3) = angular_vel(0)*sin(ang_vel_norm*dt/2)/ang_vel_norm;
                    F(0, 4) = a*quaternion(0)*angular_vel(0) + b*angular_vel(0)*(angular_vel(2)*quaternion(1)-angular_vel(1)*quaternion(2)+angular_vel(0)*quaternion(3)) + sin(ang_vel_norm*dt/2)*(1/ang_vel_norm)*quaternion(3);
                    F(0, 5) = a*quaternion(0)*angular_vel(1) + b*angular_vel(1)*(angular_vel(2)*quaternion(1)-angular_vel(1)*quaternion(2)+angular_vel(0)*quaternion(3)) - sin(ang_vel_norm*dt/2)*(1/ang_vel_norm)*quaternion(2);
                    F(0, 6) = a*quaternion(0)*angular_vel(2) + b*angular_vel(2)*(angular_vel(2)*quaternion(1)-angular_vel(1)*quaternion(2)+angular_vel(0)*quaternion(3)) + sin(ang_vel_norm*dt/2)*(1/ang_vel_norm)*quaternion(1);
                    F(1, 0) = -angular_vel(2)*sin(ang_vel_norm*dt/2)/ang_vel_norm;
                    F(1, 1) = cos(ang_vel_norm*dt/2);
                    F(1, 2) = angular_vel(0)*sin(ang_vel_norm*dt/2)/ang_vel_norm;
                    F(1, 3) = angular_vel(1)*sin(ang_vel_norm*dt/2)/ang_vel_norm;
                    F(1, 4) = a*quaternion(1)*angular_vel(0) + b*angular_vel(0)*(-angular_vel(2)*quaternion(0)+angular_vel(0)*quaternion(2)+angular_vel(1)*quaternion(3)) + sin(ang_vel_norm*dt/2)*(1/ang_vel_norm)*quaternion(2);
                    F(1, 5) = a*quaternion(1)*angular_vel(1) + b*angular_vel(1)*(-angular_vel(2)*quaternion(0)+angular_vel(0)*quaternion(2)+angular_vel(1)*quaternion(3)) + sin(ang_vel_norm*dt/2)*(1/ang_vel_norm)*quaternion(3);
                    F(1, 6) = a*quaternion(1)*angular_vel(2) + b*angular_vel(2)*(-angular_vel(2)*quaternion(0)+angular_vel(0)*quaternion(2)+angular_vel(1)*quaternion(3)) - sin(ang_vel_norm*dt/2)*(1/ang_vel_norm)*quaternion(0);
                    F(2, 0) = angular_vel(1)*sin(ang_vel_norm*dt/2)/ang_vel_norm;
                    F(2, 1) = -angular_vel(0)*sin(ang_vel_norm*dt/2)/ang_vel_norm;
                    F(2, 2) = cos(ang_vel_norm*dt/2);
                    F(2, 3) = angular_vel(2)*sin(ang_vel_norm*dt/2)/ang_vel_norm;
                    F(2, 4) = a*quaternion(2)*angular_vel(0) + b*angular_vel(0)*(angular_vel(1)*quaternion(0)-angular_vel(0)*quaternion(1)+angular_vel(2)*quaternion(3)) - sin(ang_vel_norm*dt/2)*(1/ang_vel_norm)*quaternion(1);
                    F(2, 5) = a*quaternion(2)*angular_vel(1) + b*angular_vel(1)*(angular_vel(1)*quaternion(0)-angular_vel(0)*quaternion(1)+angular_vel(2)*quaternion(3)) + sin(ang_vel_norm*dt/2)*(1/ang_vel_norm)*quaternion(0);
                    F(2, 6) = a*quaternion(2)*angular_vel(2) + b*angular_vel(2)*(angular_vel(1)*quaternion(0)-angular_vel(0)*quaternion(1)+angular_vel(2)*quaternion(3)) + sin(ang_vel_norm*dt/2)*(1/ang_vel_norm)*quaternion(3);
                    F(3, 0) = -angular_vel(0)*sin(ang_vel_norm*dt/2)/ang_vel_norm;
                    F(3, 1) = -angular_vel(1)*sin(ang_vel_norm*dt/2)/ang_vel_norm;
                    F(3, 2) = -angular_vel(2)*sin(ang_vel_norm*dt/2)/ang_vel_norm;
                    F(3, 3) = cos(ang_vel_norm*dt/2);
                    F(3, 4) = a*quaternion(3)*angular_vel(0) + b*angular_vel(0)*(-angular_vel(0)*quaternion(0)-angular_vel(1)*quaternion(1)-angular_vel(2)*quaternion(2)) - sin(ang_vel_norm*dt/2)*(1/ang_vel_norm)*quaternion(0);
                    F(3, 5) = a*quaternion(3)*angular_vel(1) + b*angular_vel(1)*(-angular_vel(0)*quaternion(0)-angular_vel(1)*quaternion(1)-angular_vel(2)*quaternion(2)) - sin(ang_vel_norm*dt/2)*(1/ang_vel_norm)*quaternion(1);
                    F(3, 6) = a*quaternion(3)*angular_vel(2) + b*angular_vel(2)*(-angular_vel(0)*quaternion(0)-angular_vel(1)*quaternion(1)-angular_vel(2)*quaternion(2)) - sin(ang_vel_norm*dt/2)*(1/ang_vel_norm)*quaternion(2);
                }
                else
                {
                    //if the norm of angular velocity tends to zero, then we fall into the case cos(0) = 1 and lim_x->0 sin(ax)/x = a
                    //additionally if the norm is zero it means that all the elements are zero
                    //Jacobian matrix elements
                    F(0, 0) = 1;
                    F(0, 1) = 0;
                    F(0, 2) = 0;
                    F(0, 3) = 0;
                    F(0, 4) = (dt/2)*quaternion(3);
                    F(0, 5) = -(dt/2)*quaternion(2);
                    F(0, 6) = (dt/2)*quaternion(1);
                    F(1, 0) = 0;
                    F(1, 1) = 1;
                    F(1, 2) = 0;
                    F(1, 3) = 0;
                    F(1, 4) = (dt/2)*quaternion(2);
                    F(1, 5) = (dt/2)*quaternion(3);
                    F(1, 6) = -(dt/2)*quaternion(0);
                    F(2, 0) = 0;
                    F(2, 1) = 0;
                    F(2, 2) = 1;
                    F(2, 3) = 0;
                    F(2, 4) = -(dt/2)*quaternion(1);
                    F(2, 5) = (dt/2)*quaternion(0);
                    F(2, 6) = (dt/2)*quaternion(3);
                    F(3, 0) = 0;
                    F(3, 1) = 0;
                    F(3, 2) = 0;
                    F(3, 3) = 1;
                    F(3, 4) = -(dt/2)*quaternion(0);
                    F(3, 5) = -(dt/2)*quaternion(1);
                    F(3, 6) = -(dt/2)*quaternion(2);
                }
                //angular velocity part
                F.block(4,4,3,3) = I3;
                //position part
                F.block(7,7,3,3) = I3;
                F.block(7,10,3,3) = dt*I3;
                F.block(7,13,3,3) = (pow(dt,2)/2)*I3;
                //velocity part
                F.block(10,10,3,3) = I3;
                F.block(10,13,3,3) = dt*I3;
                //acceleration part
                F.block(13,13,3,3) = I3;
                //---Measurement Step---
                //retrieve measurement data
                //For an appropriate sensor fusion the rotation matrix from inertial sensor reference to electromagnetic one should be computed and used in order to map all the measurements in
                //the same reference. The rotation matrix should be use to map the quaternion but also the acceleration and angular velocity vectors.
                //update and register magnetometer to use as check for interferences and decide whether to use EM data or not
                mag = inertial.get_mag();
                magnetometer << mag.vector.x, mag.vector.y, mag.vector.z;
                magnetometer = quaternionRotationLocal(quat_COIL,magnetometer);
                mag_norm_dev = abs(1-magnetometer.norm());
                //we can try changing the EM position and orientation variances as original*(1+mag_norm_dev), in this way if the deviation is increasing but the sensor is still
                //reported as visible, then the measurement covariances are increased and EKF should rely less on Aurora data
                //the magnetometer check should be based on the deviation of mag norm with respect to reference
                time = time + dt*iteration*ones;
                if(electromagnetic.is_visible())
                {
                    index = 0;
                    quat = electromagnetic.get_quat();
                    meas_quat_em << quat.quaternion.x, quat.quaternion.y, quat.quaternion.z, quat.quaternion.w;
                    pos = electromagnetic.get_pos();
                    meas_pos_em << pos.vector.x, pos.vector.y, pos.vector.z;
                    quat_imu = inertial.get_quat();
                    //register IMU quaternion
                    meas_quat_imu << quat_imu.quaternion.x, quat_imu.quaternion.y, quat_imu.quaternion.z, quat_imu.quaternion.w;
                    meas_quat_imu = quaternionProduct(ENU_q_FG,quaternionProduct(meas_quat_imu,IMU_q_COIL));
                    //register angular velocity and free acceleration
                    ang_vel = inertial.get_ang_vel();
                    meas_ang_vel_imu << ang_vel.vector.x, ang_vel.vector.y, ang_vel.vector.z;
                    //compute the composite quaternion from IMU to FG, passing through COIL RF
                    quat_COIL = quaternionProduct(IMU_q_COIL,quaternionInverse(quaternion));
                    meas_ang_vel_imu = quaternionRotationLocal(IMU_q_COIL,meas_ang_vel_imu);
                    acc = inertial.get_acc();
                    meas_acc_imu << acc.vector.x, acc.vector.y, acc.vector.z;
                    meas_acc_imu = quaternionRotation(quaternionInverse(ENU_q_FG),meas_acc_imu);
                    measurement << meas_quat_em, meas_pos_em, meas_quat_imu, meas_ang_vel_imu, meas_acc_imu;

                    //here we should performe the buffer check to evaluate for magnetic interferences and decide what to do
                    //move data in buffers back of one column
                    measurement_increment = measurement_buffer.block(0,1,17,1)-measurement_buffer.block(0,0,17,1);
                    //measurement_buffer.block(0,0,17,window_size-2) = measurement_buffer.block(0,1,17,window_size-2);
                    //MagFieldMagnitude.block(0,0,1,window_size-2) = MagFieldMagnitude.block(0,1,1,window_size-2);
                    for(int i = 0; i<window_size-1; i++)
                    {
                        measurement_buffer(0,i) = measurement_buffer(0,i+1);
                        measurement_buffer(1,i) = measurement_buffer(1,i+1);
                        measurement_buffer(2,i) = measurement_buffer(2,i+1);
                        measurement_buffer(3,i) = measurement_buffer(3,i+1);
                        measurement_buffer(4,i) = measurement_buffer(4,i+1);
                        measurement_buffer(5,i) = measurement_buffer(5,i+1);
                        measurement_buffer(6,i) = measurement_buffer(6,i+1);
                        measurement_buffer(7,i) = measurement_buffer(7,i+1);
                        measurement_buffer(8,i) = measurement_buffer(8,i+1);
                        measurement_buffer(9,i) = measurement_buffer(9,i+1);
                        measurement_buffer(10,i) = measurement_buffer(10,i+1);
                        measurement_buffer(11,i) = measurement_buffer(11,i+1);
                        measurement_buffer(12,i) = measurement_buffer(12,i+1);
                        measurement_buffer(13,i) = measurement_buffer(13,i+1);
                        measurement_buffer(14,i) = measurement_buffer(14,i+1);
                        measurement_buffer(15,i) = measurement_buffer(15,i+1);
                        measurement_buffer(16,i) = measurement_buffer(16,i+1);
                        MagFieldMagnitude(0,i) = MagFieldMagnitude(0,i+1);
                    }
                    //insert new element in buffer
                    measurement_buffer.block(0,window_size-1,17,1) = measurement;
                    //stack magnitude norm data in the moving window
                    mag = inertial.get_mag();
                    meas_mag << mag.vector.x, mag.vector.y, mag.vector.z;
                    MagFieldMagnitude(0,window_size-1) = sqrt(pow(meas_mag(0),2)+pow(meas_mag(1),2)+pow(meas_mag(2),2));
                    //now we can compute the new magnetic norm slope
                    for(int i = 0; i<window_size; i++)
                    {
                        sum1 = sum1 + (double)time(i)*MagFieldMagnitude(0,i);
                        sum2 = sum2 + (double)time(i);
                        sum3 = sum3 + (double)MagFieldMagnitude(0,i);
                        sum4 = sum4 + pow((double)time(i),2);
                    }
                    mag_slope(0) = mag_slope(1);
                    mag_slope(1) = (sum1 - ((sum2*sum3)/window_size))/(sum4 - (pow(sum2,2)/window_size));
                    sum1 = 0;
                    sum2 = 0;
                    sum3 = 0;
                    sum4 = 0;
                    //at this point the check routine can start
                    if(((mag_slope(1)>mag_threshold)&&(mag_slope(0)<mag_threshold))||((mag_slope(1)<mag_threshold)&&(mag_slope(0)>mag_threshold))||((mag_slope(1)<-mag_threshold)&&(mag_slope(0)>-mag_threshold))||((mag_slope(1)>-mag_threshold)&&(mag_slope(0)<-mag_threshold)))
                    {
                        //we detect the start/end or a raising/falling edge
                        counts = counts+1;
                    }
                    if((counts%2)==0)
                    {
                        //we have even peaks, we need to check magnitude and acceleration norm deviation
                        if((MagFieldMagnitude(0,0)<1.25)&&(MagFieldMagnitude(0,0)>0.75))
                        {
                            //compute expected accelerations from EM data
                            acc_x = (double)(measurement_buffer(4,window_size-3)-2*measurement_buffer(4,window_size-2)+measurement_buffer(4,window_size-1))/pow(dt,2);
                            acc_y = (double)(measurement_buffer(5,window_size-3)-2*measurement_buffer(5,window_size-2)+measurement_buffer(5,window_size-1))/pow(dt,2);
                            acc_z = (double)(measurement_buffer(6,window_size-3)-2*measurement_buffer(6,window_size-2)+measurement_buffer(6,window_size-1))/pow(dt,2);
                            acc_deviation = (double)sqrt(pow(acc_x,2)+pow(acc_y,2)+pow(acc_z,2))-sqrt(pow(meas_acc_imu(0),2)+pow(meas_acc_imu(1),2)+pow(meas_acc_imu(2),2));
                            if(abs(acc_deviation)>acc_threshold)
                            {
                                //we are still inside a transition phase of magnetic disturbance
                                reduced = true;
                                bad(0) = bad(1);
                                bad(1) = time(0);                      //we save the time at which bad data occured
                                measurement_c << measurement_buffer(7,0),measurement_buffer(8,0),measurement_buffer(9,0),measurement_buffer(10,0),measurement_buffer(11,0),measurement_buffer(12,0),measurement_buffer(13,0),measurement_buffer(14,0),measurement_buffer(15,0),measurement_buffer(16,0);
                                //The measurement process is linear, so we can simply define the H matrix
                                H_c.block(0,0,4,4) = I4;
                                H_c.block(4,4,3,3) = I3;
                                H_c.block(7,13,3,3) = I3;
                                innovation_c = measurement_c - H_c*predict_state;
                                flag = 1;
                            }
                            else
                            {
                                if(bad(1)==0)
                                {
                                    //a bad sample never occured, so we are sure that we are in a safe condition and we can use the raw EM measurement
                                    measurement << measurement_buffer(0,0),measurement_buffer(1,0),measurement_buffer(2,0),measurement_buffer(3,0),measurement_buffer(4,0),measurement_buffer(5,0),measurement_buffer(6,0),measurement_buffer(7,0),measurement_buffer(8,0),measurement_buffer(9,0),measurement_buffer(10,0),measurement_buffer(11,0),measurement_buffer(12,0),measurement_buffer(13,0),measurement_buffer(14,0),measurement_buffer(15,0),measurement_buffer(16,0);
                                    measurement_lastgood = measurement;
                                    //The measurement process is linear, so we can simply define the H matrix
                                    H.block(0,0,4,4) = I4;
                                    H.block(4,7,3,3) = I3;
                                    H.block(7,0,4,4) = I4;
                                    H.block(11,4,3,3) = I3;
                                    H.block(13,13,3,3) = I3;
                                    innovation = measurement - H*predict_state;
                                    increment = false;
                                }
                                else
                                {
                                    if(abs(time(0)-bad(1))<2)
                                    {
                                        //the last bad sample occured less than 2 seconds ago so consider this as bad (safe margin)
                                        reduced = true;
                                        //bad(0) = bad(1);
                                        //bad(1) = time(0);                      //we save the time at which bad data occured
                                        measurement_c << measurement_buffer(7,0),measurement_buffer(8,0),measurement_buffer(9,0),measurement_buffer(10,0),measurement_buffer(11,0),measurement_buffer(12,0),measurement_buffer(13,0),measurement_buffer(14,0),measurement_buffer(15,0),measurement_buffer(16,0);
                                        //The measurement process is linear, so we can simply define the H matrix
                                        H_c.block(0,0,4,4) = I4;
                                        H_c.block(4,4,3,3) = I3;
                                        H_c.block(7,13,3,3) = I3;
                                        innovation_c = measurement_c - H_c*predict_state;
                                        flag = 3;
                                    }
                                    else
                                    {
                                        //we are in a condition in which no magnetic field nor acceleration deviation occured, but in the past
                                        //a magnetic interference already occured, thus we cannot distinguish if we are really in a case of undistorted data
                                        //or steady state of distortion, so we should use EM data increments
                                        reduced = false;
                                        measurement = measurement_lastgood + measurement_increment;
                                        //The measurement process is linear, so we can simply define the H matrix
                                        H.block(0,0,4,4) = I4;
                                        H.block(4,7,3,3) = I3;
                                        H.block(7,0,4,4) = I4;
                                        H.block(11,4,3,3) = I3;
                                        H.block(13,13,3,3) = I3;
                                        innovation = measurement - H*predict_state;
                                        increment = true;
                                    }
                                }
                            }
                        }
                        else
                        {
                            //use incremental measure
                            reduced = false;
                            measurement = measurement_lastgood + measurement_increment;
                            //The measurement process is linear, so we can simply define the H matrix
                            H.block(0,0,4,4) = I4;
                            H.block(4,7,3,3) = I3;
                            H.block(7,0,4,4) = I4;
                            H.block(11,4,3,3) = I3;
                            H.block(13,13,3,3) = I3;
                            innovation = measurement - H*predict_state;
                            increment = true;
                        }
                    }
                    else
                    {
                        //we are still inside a transition phase of magnetic disturbance
                        reduced = true;
                        bad(0) = bad(1);
                        bad(1) = time(0);                      //we save the time at which bad data occured
                        measurement_c << measurement_buffer(7,0),measurement_buffer(8,0),measurement_buffer(9,0),measurement_buffer(10,0),measurement_buffer(11,0),measurement_buffer(12,0),measurement_buffer(13,0),measurement_buffer(14,0),measurement_buffer(15,0),measurement_buffer(16,0);
                        //The measurement process is linear, so we can simply define the H matrix
                        H_c.block(0,0,4,4) = I4;
                        H_c.block(4,4,3,3) = I3;
                        H_c.block(7,13,3,3) = I3;
                        innovation_c = measurement_c - H_c*predict_state;
                        flag = 2;
                    }
                }
                else
                {
                    reduced = true;
                    bad(0) = bad(1);
                    bad(1) = time(0);                      //we save the time at which bad data occured
                    measurement_c << measurement_buffer(7,index),measurement_buffer(8,index),measurement_buffer(9,index),measurement_buffer(10,index),measurement_buffer(11,index),measurement_buffer(12,index),measurement_buffer(13,index),measurement_buffer(14,index),measurement_buffer(15,index),measurement_buffer(16,index);
                    //The measurement process is linear, so we can simply define the H matrix
                    H_c.block(0,0,4,4) = I4;
                    H_c.block(4,4,3,3) = I3;
                    H_c.block(7,13,3,3) = I3;
                    innovation_c = measurement_c - H_c*predict_state;
                    index = index + 1;
                }
                //We need to propagate the Euler angle variances to the Quaternion configuration in order to update the R matrix
                //First retrieve roll, pitch and yaw values from measurements and then use variance propagation formula V_new = Jac*V_old*JacT
                //Conversion from quaternion to euler for electromagnetic measurements
                sinr_cosp = 2*(meas_quat_em(3)*meas_quat_em(0) + meas_quat_em(1)*meas_quat_em(2));
                cosr_cosp = 1 - 2*(meas_quat_em(0)*meas_quat_em(0) + meas_quat_em(1)*meas_quat_em(1));
                roll = atan2(sinr_cosp, cosr_cosp);
                sinp = 2*(meas_quat_em(3)*meas_quat_em(1) - meas_quat_em(2)*meas_quat_em(0));
                if(abs(sinp) >= 1)
                {
                    pitch = copysign(PI/2, sinp);
                }
                else
                {
                    pitch = asin(sinp);

                }
                siny_cosp = 2*(meas_quat_em(3)*meas_quat_em(2) + meas_quat_em(0)*meas_quat_em(1));
                cosy_cosp = 1 - 2*(meas_quat_em(1)*meas_quat_em(1) + meas_quat_em(2)*meas_quat_em(2));
                yaw = atan2(siny_cosp, cosy_cosp);
                //propagation of variance
                var_qw = pow(-0.5*sin(roll/2)*cos(pitch/2)*cos(yaw/2)+0.5*cos(roll/2)*sin(pitch/2)*sin(yaw/2),2)*var_eul_AU + pow(-0.5*cos(roll/2)*sin(pitch/2)*cos(yaw/2)+0.5*sin(roll/2)*cos(pitch/2)*sin(yaw/2),2)*var_eul_AU + pow(-0.5*cos(roll/2)*cos(pitch/2)*sin(yaw/2)+0.5*sin(roll/2)*sin(pitch/2)*cos(yaw/2),2)*var_eul_AU;
                var_qx = pow(0.5*cos(roll/2)*cos(pitch/2)*cos(yaw/2)+0.5*sin(roll/2)*sin(pitch/2)*sin(yaw/2),2)*var_eul_AU + pow(-0.5*sin(roll/2)*sin(pitch/2)*cos(yaw/2)-0.5*cos(roll/2)*cos(pitch/2)*sin(yaw/2),2)*var_eul_AU + pow(-0.5*sin(roll/2)*cos(pitch/2)*sin(yaw/2)-0.5*cos(roll/2)*sin(pitch/2)*cos(yaw/2),2)*var_eul_AU;
                var_qy = pow(-0.5*sin(roll/2)*sin(pitch/2)*cos(yaw/2)+0.5*cos(roll/2)*cos(pitch/2)*sin(yaw/2),2)*var_eul_AU + pow(0.5*cos(roll/2)*cos(pitch/2)*cos(yaw/2)-0.5*sin(roll/2)*sin(pitch/2)*sin(yaw/2),2)*var_eul_AU + pow(-0.5*cos(roll/2)*sin(pitch/2)*sin(yaw/2)+0.5*sin(roll/2)*cos(pitch/2)*cos(yaw/2),2)*var_eul_AU;
                var_qz = pow(-0.5*sin(roll/2)*cos(pitch/2)*sin(yaw/2)-0.5*cos(roll/2)*sin(pitch/2)*cos(yaw/2),2)*var_eul_AU + pow(-0.5*cos(roll/2)*sin(pitch/2)*sin(yaw/2)-0.5*sin(roll/2)*cos(pitch/2)*cos(yaw/2),2)*var_eul_AU + pow(0.5*cos(roll/2)*cos(pitch/2)*cos(yaw/2)+0.5*sin(roll/2)*sin(pitch/2)*sin(yaw/2),2)*var_eul_AU;
                R(0,0) = var_qx;
                R(1,1) = var_qy;
                R(2,2) = var_qz;
                R(3,3) = var_qw;
                //Conversion from quaternion to euler for inertial measurements
                sinr_cosp = 2*(meas_quat_imu(3)*meas_quat_imu(0) + meas_quat_imu(1)*meas_quat_imu(2));
                cosr_cosp = 1 - 2*(meas_quat_imu(0)*meas_quat_imu(0) + meas_quat_imu(1)*meas_quat_imu(1));
                roll = atan2(sinr_cosp, cosr_cosp);
                sinp = 2*(meas_quat_imu(3)*meas_quat_imu(1) - meas_quat_imu(2)*meas_quat_imu(0));
                if(abs(sinp) >= 1)
                {
                    pitch = copysign(PI/2, sinp);
                }
                else
                {
                    pitch = asin(sinp);

                }
                siny_cosp = 2*(meas_quat_imu(3)*meas_quat_imu(2) + meas_quat_imu(0)*meas_quat_imu(1));
                cosy_cosp = 1 - 2*(meas_quat_imu(1)*meas_quat_imu(1) + meas_quat_imu(2)*meas_quat_imu(2));
                yaw = atan2(siny_cosp, cosy_cosp);
                //propagation of variance
                var_qw = pow(-0.5*sin(roll/2)*cos(pitch/2)*cos(yaw/2)+0.5*cos(roll/2)*sin(pitch/2)*sin(yaw/2),2)*var_roll_IMU + pow(-0.5*cos(roll/2)*sin(pitch/2)*cos(yaw/2)+0.5*sin(roll/2)*cos(pitch/2)*sin(yaw/2),2)*var_pitch_IMU + pow(-0.5*cos(roll/2)*cos(pitch/2)*sin(yaw/2)+0.5*sin(roll/2)*sin(pitch/2)*cos(yaw/2),2)*var_yaw_IMU;
                var_qx = pow(0.5*cos(roll/2)*cos(pitch/2)*cos(yaw/2)+0.5*sin(roll/2)*sin(pitch/2)*sin(yaw/2),2)*var_roll_IMU + pow(-0.5*sin(roll/2)*sin(pitch/2)*cos(yaw/2)-0.5*cos(roll/2)*cos(pitch/2)*sin(yaw/2),2)*var_pitch_IMU + pow(-0.5*sin(roll/2)*cos(pitch/2)*sin(yaw/2)-0.5*cos(roll/2)*sin(pitch/2)*cos(yaw/2),2)*var_yaw_IMU;
                var_qy = pow(-0.5*sin(roll/2)*sin(pitch/2)*cos(yaw/2)+0.5*cos(roll/2)*cos(pitch/2)*sin(yaw/2),2)*var_roll_IMU + pow(0.5*cos(roll/2)*cos(pitch/2)*cos(yaw/2)-0.5*sin(roll/2)*sin(pitch/2)*sin(yaw/2),2)*var_pitch_IMU + pow(-0.5*cos(roll/2)*sin(pitch/2)*sin(yaw/2)+0.5*sin(roll/2)*cos(pitch/2)*cos(yaw/2),2)*var_yaw_IMU;
                var_qz = pow(-0.5*sin(roll/2)*cos(pitch/2)*sin(yaw/2)-0.5*cos(roll/2)*sin(pitch/2)*cos(yaw/2),2)*var_roll_IMU + pow(-0.5*cos(roll/2)*sin(pitch/2)*sin(yaw/2)-0.5*sin(roll/2)*cos(pitch/2)*cos(yaw/2),2)*var_pitch_IMU + pow(0.5*cos(roll/2)*cos(pitch/2)*cos(yaw/2)+0.5*sin(roll/2)*sin(pitch/2)*sin(yaw/2),2)*var_yaw_IMU;
                R(7,7) = var_qx;
                R(8,8) = var_qy;
                R(9,9) = var_qz;
                R(10,10) = var_qw;
                R_c = R.block(7,7,10,10);           //R matrix for the corrupted case in which no EM data are available
                //---EKF Update---
                if(!reduced)
                {
                    if (!increment)
                    {
                        //Standard EKF formulation
                        P = F*P*F.transpose() + Q;
                        S = H*P*H.transpose() + R;
                        K = P*H.transpose()* S.inverse();
                        state = predict_state + K*innovation;
                        P = (I16 - K*H)*P;
                    }
                    else
                    {
                        //Incremental EKF formulation
                        P = F*P*F.transpose() + Q;
                        //The measurement model is linear and H_k = H_k-1
                        H_hat = H*(F - I16);
                        S = H_hat*P*H_hat.transpose() + H*Q*H.transpose() 2*R;
                        K = F*P*H_hat.transpose()*S.inverse();
                        state = predict_state + K*innovation;
                        P = (I16 - K * H) * P;
                    }
                    //update last reference point with estimated values
                    measurement_lastgood << state(0),state(1),state(2),state(3),state(7),state(8),state(9),state(0),state(1),state(2),state(3),state(4),state(5),state(6),state(13),state(14),state(15);
                }
                else
                {
                    P = F*P*F.transpose() + Q;
                    S_c = H_c*P*H_c.transpose() + R_c;
                    K_c = P*H_c.transpose()*S_c.inverse();
                    state = predict_state + K_c*innovation_c;
                    P = (I16 - K_c*H_c)*P;
                    //update last reference point with estimated values
                    measurement_lastgood << state(0),state(1),state(2),state(3),state(7),state(8),state(9),state(0),state(1),state(2),state(3),state(4),state(5),state(6),state(13),state(14),state(15);
                }

                //retrieve the components from the state
                quaternion << state(0), state(1), state(2), state(3);
                if(quaternion.norm() != 0)
                {
                    quaternion.normalize();
                }
                angular_vel << state(4), state(5), state(6);
                position << state(7), state(8), state(9);
                velocity << state(10), state(11), state(12);
                acceleration << state(13), state(14), state(15);


                //Publish my pose estimate
                geometry_msgs::QuaternionStamped my_quat;
                my_quat.quaternion.w = quaternion(3);
                my_quat.quaternion.x = quaternion(0);
                my_quat.quaternion.y = quaternion(1);
                my_quat.quaternion.z = quaternion(2);
                quat_pub.publish(my_quat);
                geometry_msgs::Vector3Stamped my_pos;
                my_pos.vector.x = position(0);
                my_pos.vector.y = position(1);
                my_pos.vector.z = position(2);
                pos_pub.publish(my_pos);

                geometry_msgs::Pose my_pose;
                my_pose.position.x = position(0);
                my_pose.position.y = position(1);
                my_pose.position.z = position(2);
                my_pose.orientation.w = quaternion(3);
                my_pose.orientation.x = quaternion(0);
                my_pose.orientation.y = quaternion(1);
                my_pose.orientation.z = quaternion(2);
                pose_pub.publish(my_pose);

                geometry_msgs::Vector3Stamped my_mag_acc;
                my_mag_acc.vector.x = flag;
                my_mag_acc.vector.y = bad(1);
                my_mag_acc.vector.z = counts;
                Mag_pub.publish(my_mag_acc);
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}