#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
//to read data from XSens node
#include "geometry_msgs/QuaternionStamped.h"    //to read quaternion, quaternion increments from XSens
#include "geometry_msgs/Vector3Stamped.h"       //to read free-acceleration, angular velocity, velocity increments, magnetic field from XSens
//to read data from Aurora node                 //from run.sh, Orcos is needed
#include <ndi_aurora_msgs/AuroraData.h>
#include <ndi_aurora_msgs/AuroraDataVector.h>
//to use linear algebra
#include "Eigen/Dense"
#include <math.h>

#define PI 3.14159265

using namespace std;

/**
 * This node manage to receive the acquired data from XSens inertial sensor and Aurora EM sensor synchronizing the two (upsampling EM to achieve 100Hz from 40Hz).
 * XSens sensor should provide data on orientation (quaternion), free-acceleration, angular velocity, turn increment (deltaq), velocity increment (deltav), and magnetometer data at 100Hz
 * Aurora EM sensor should provide information on position (x,y,z) and orientation (quaternion) at 40Hz.
 * The XSens quaternion estimate is further manipulated in order to limit deviations and drift due to unmodelled magnetic field changes.
 * In order to synchronize the data output a synchronization message can be delivered to Xsens (to indicate the start of sampling, ideally, in this first example instead no synchronization on
 * initial sampling instant is considered) and the same acquisition rate should be kept for the two.
 * Rather than diminuishing the acquisition rate of Xsens from 100Hz to 40 Hz, the Aurora data can be upsampled to achieve 100Hz (strategy similar to sample-and-hold).
 * Once the data are synchronized this node publish them on topics that will be read by the sensor fusion node.
 */

//First define a XSens and Aurora class to hold the acquired data
class XSens
{
  float q_w, q_x, q_y, q_z, dq_w, dq_x, dq_y, dq_z, acc_x, acc_y, acc_z, accg_x, accg_y, accg_z, dv_x, dv_y, dv_z, mag_x, mag_y, mag_z, w_x, w_y, w_z;
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
  void set_quat_incr(float a, float b, float c, float d)
  {
    dq_w = a;
    dq_x = b;
    dq_y = c;
    dq_z = d;
    initialized = true;
  }
  geometry_msgs::QuaternionStamped get_quat_incr()
  {
    geometry_msgs::QuaternionStamped msg;
    msg.quaternion.w = dq_w;
    msg.quaternion.x = dq_x;
    msg.quaternion.y = dq_y;
    msg.quaternion.z = dq_z;
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
  void set_accg(float a, float b, float c)
  {
    accg_x = a;
    accg_y = b;
    accg_z = c;
    initialized = true;
  }

  geometry_msgs::Vector3Stamped get_accg()
  {
    geometry_msgs::Vector3Stamped msg;
    msg.vector.x = accg_x;
    msg.vector.y = accg_y;
    msg.vector.z = accg_z;
    return msg;
  }
  void set_vel_incr(float a, float b, float c)
  {
    dv_x = a;
    dv_y = b;
    dv_z = c;
    initialized = true;
  }
  geometry_msgs::Vector3Stamped get_vel_incr()
  {
    geometry_msgs::Vector3Stamped msg;
    msg.vector.x = dv_x;
    msg.vector.y = dv_y;
    msg.vector.z = dv_z;
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
  bool is_initialized()
  {
      return initialized;
  }
};
class Aurora
{
  float q_w, q_x, q_y, q_z, p_x, p_y, p_z,q_w_prev, q_x_prev, q_y_prev, q_z_prev, p_x_prev, p_y_prev, p_z_prev;
  bool initialized = false;         //just tells if we have received data, or we've just created an un-initialized object
  bool visible = false;             //if 0 then the sensor is outside the workspace, so no useful data to send --> stop the communication on aurora topics
public:
  void set_EM_data(float a, float b, float c, float d, float e, float f, float g, bool h)
  {
    q_w = a;
    q_x = b;
    q_y = c;
    q_z = d;
    p_x = e;
    p_y = f;
    p_z = g;
    visible = h;
    initialized = true;
  }
  void set_EM_data_prev()
  {
    q_w_prev = q_w;
    q_x_prev = q_x;
    q_y_prev = q_y;
    q_z_prev = q_z;
    p_x_prev = p_x;
    p_y_prev = p_y;
    p_z_prev = p_z;
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
  geometry_msgs::Vector3Stamped get_pos()
  {
    geometry_msgs::Vector3Stamped msg;
    msg.vector.x = p_x;
    msg.vector.y = p_y;
    msg.vector.z = p_z;
    return msg;
  }
  geometry_msgs::QuaternionStamped get_quat_prev()
  {
    geometry_msgs::QuaternionStamped msg;
    msg.quaternion.w = q_w_prev;
    msg.quaternion.x = q_x_prev;
    msg.quaternion.y = q_y_prev;
    msg.quaternion.z = q_z_prev;
    return msg;
  }
  geometry_msgs::Vector3Stamped get_pos_prev()
  {
    geometry_msgs::Vector3Stamped msg;
    msg.vector.x = p_x_prev;
    msg.vector.y = p_y_prev;
    msg.vector.z = p_z_prev;
    return msg;
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

//Create global sensor objects
XSens inertial;
Aurora electromagnetic;

//Callback functions to read from topics
//XSens
void quaternionXSensCallback(const geometry_msgs::QuaternionStamped& msg)
{
    inertial.set_quat(msg.quaternion.w,msg.quaternion.x,msg.quaternion.y,msg.quaternion.z);
}
void quaternionIncrXSensCallback(const geometry_msgs::QuaternionStamped& msg)
{
    inertial.set_quat_incr(msg.quaternion.w,msg.quaternion.x,msg.quaternion.y,msg.quaternion.z);
}
void accelerationXSensCallback(const geometry_msgs::Vector3Stamped& msg)
{
    inertial.set_acc(msg.vector.x,msg.vector.y,msg.vector.z);
}
void accelerationgXSensCallback(const geometry_msgs::Vector3Stamped& msg)
{
    inertial.set_accg(msg.vector.x,msg.vector.y,msg.vector.z);
}
void velocityIncrXSensCallback(const geometry_msgs::Vector3Stamped& msg)
{
    inertial.set_vel_incr(msg.vector.x,msg.vector.y,msg.vector.z);
}
void magnetometerXSensCallback(const geometry_msgs::Vector3Stamped& msg)
{
    inertial.set_mag(msg.vector.x,msg.vector.y,msg.vector.z);
}
void angularVelocityXSensCallback(const geometry_msgs::Vector3Stamped& msg)
{
    inertial.set_ang_vel(msg.vector.x,msg.vector.y,msg.vector.z);
}
//Aurora
void dataAuroraCallback(const ndi_aurora_msgs::AuroraDataVector& msg)
{
    if((electromagnetic.is_initialized())&&(electromagnetic.is_visible()))
    {
        electromagnetic.set_EM_data_prev();
    }
    electromagnetic.set_EM_data(msg.data[0].orientation.w,msg.data[0].orientation.x,msg.data[0].orientation.y,msg.data[0].orientation.z,msg.data[0].position.x,msg.data[0].position.y,msg.data[0].position.z,msg.data[0].visible);
}

double wrapAngle(double angle)
{
    angle = fmod(angle,(2.0*PI));
    if(angle <= -PI)
    {
        angle += (2.0*PI);
    }
    else if(angle > PI)
    {
        angle -= (2.0*PI);
    }
    return angle;
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
Eigen::Vector4d quaternionConj(Eigen::Vector4d q)
{
    //This function compute the quaternion conjugate
    Eigen::Vector4d inverse = Eigen::Vector4d::Zero();
    inverse << -q(0),-q(1),-q(2),q(3);
    return inverse;
}

Eigen::MatrixXd quaternion2R(Eigen::Vector4d q)
{
    //This function compute the rotation matrix R from the input quaternion q
    //q is in the form (qx,qy,qz,qw)
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(4,4);
    R(0,0) = 2*(pow(q(3),2)+pow(q(0),2))-1;
    R(0,1) = 2*(q(0)*q(1)-q(3)*q(2));
    R(0,2) = 2*(q(0)*q(2)+q(3)*q(1));
    return R;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "upsample");
    ros::NodeHandle n;

    //Listen to XSens node topics to update the data of the XSens inertial object
    ros::Subscriber acc_sub = n.subscribe("/filter/free_acceleration", 1000, accelerationXSensCallback);
    ros::Subscriber quat_sub = n.subscribe("/filter/quaternion", 1000, quaternionXSensCallback);
    ros::Subscriber dq_sub = n.subscribe("/imu/dq", 1000, quaternionIncrXSensCallback);
    ros::Subscriber dv_sub = n.subscribe("/imu/dv", 1000, velocityIncrXSensCallback);
    ros::Subscriber mag_sub = n.subscribe("/imu/mag", 1000, magnetometerXSensCallback);
    ros::Subscriber acc_g_sub = n.subscribe("/imu/acceleration", 1000, accelerationgXSensCallback);
    ros::Subscriber ang_vel_sub = n.subscribe("/imu/angular_velocity", 1000, angularVelocityXSensCallback);
    //Listen to Aurora node topics to update the data of the Aurora electromagnetic object
    ros::Subscriber em_sub = n.subscribe("/aurora_data", 1000, dataAuroraCallback);

    //Send out the inertial data on topics (should be subscribed by the analysis node)
    ros::Publisher acc_pub = n.advertise<geometry_msgs::Vector3Stamped>("/XSens/acceleration", 1000);
    ros::Publisher quat_pub = n.advertise<geometry_msgs::QuaternionStamped>("/XSens/quaternion", 1000);
    ros::Publisher dq_pub = n.advertise<geometry_msgs::QuaternionStamped>("/XSens/quaternion_incr", 1000);
    ros::Publisher dv_pub = n.advertise<geometry_msgs::Vector3Stamped>("/XSens/velocity_incr", 1000);
    ros::Publisher mag_pub = n.advertise<geometry_msgs::Vector3Stamped>("/XSens/magnetometer", 1000);
    ros::Publisher ang_vel_pub = n.advertise<geometry_msgs::Vector3Stamped>("/XSens/angular_velocity", 1000);
    //Send out the electromagnetic data on topics (should be subscribed by the analysis node)
    ros::Publisher quat_pub_em = n.advertise<geometry_msgs::QuaternionStamped>("/Aurora/quaternion", 1000);
    ros::Publisher pos_pub_em = n.advertise<geometry_msgs::Vector3Stamped>("/Aurora/position", 1000);
    ros::Publisher bool_em = n.advertise<std_msgs::Bool>("/Aurora/visibility", 1000);

    //Send out my quaternion estimation
    ros::Publisher my_quat_pub = n.advertise<geometry_msgs::QuaternionStamped>("/XSensMod/quaternion", 1000);

    //Initialization of quaternion vector
    Eigen::VectorXd quaternion = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd rot_quaternion = Eigen::VectorXd::Zero(4);
    rot_quaternion << -0.0015,-0.6717,0.1441,0.7267;
    Eigen::VectorXd rot_quaternion2 = Eigen::VectorXd::Zero(4);
    rot_quaternion2 << -0.3780,0.6298,0.5920,0.3317;
    Eigen::VectorXd quaternion_em = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd quaternion_imu = Eigen::VectorXd::Zero(4);

    std_msgs::Bool visible_em;
    geometry_msgs::QuaternionStamped quat_em;

    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    double prev_roll = 0;
    double roll_dev = 0;
    double prev_pitch = 0;
    double pitch_dev = 0;
    double prev_yaw = 0;
    double yaw_dev = 0;
    double sinr_cosp;
    double cosr_cosp;
    double sinp;
    double siny_cosp;
    double cosy_cosp;
    double qx, qy, qz, qw;
    bool first = true;

    //Set the rate of the while loop, which coincide with the rate of the output data from this synchronizer node
    ros::Rate loop_rate(100);                                   //provide output data at 100Hz
    while (ros::ok())
    {
        //Synchronization is simply carried out reading the inertial and aurora object values and sending them out on
        //appropriate topics at the desired rate. Since we've selected 100Hz as data rate, inertial data are coming at
        //nominal 100Hz, aurora data at nominal 40Hz, the output data should be always updated for the inertial sensor
        //and copied as long as new data are not available for aurora sensor (developing something similar to a sample-and-hold)

        //Before to start publishing data we wait for both the sensor objects to be initialized
        //if(electromagnetic.is_initialized() && inertial.is_initialized())
        {
            //Publish XSens data
            geometry_msgs::Vector3Stamped acc = inertial.get_acc();
            acc_pub.publish(acc);
            geometry_msgs::QuaternionStamped quat = inertial.get_quat();
            quat_pub.publish(quat);
            geometry_msgs::QuaternionStamped quat_incr = inertial.get_quat_incr();
            dq_pub.publish(quat_incr);
            geometry_msgs::Vector3Stamped vel_incr = inertial.get_vel_incr();
            dv_pub.publish(vel_incr);
            geometry_msgs::Vector3Stamped mag = inertial.get_mag();
            mag_pub.publish(mag);
            geometry_msgs::Vector3Stamped ang_vel = inertial.get_ang_vel();
            ang_vel_pub.publish(ang_vel);

            //Publish Aurora data only if they are reliable --> we are inside field generator workspace
            if(electromagnetic.is_visible())
            {
                geometry_msgs::Vector3Stamped pos = electromagnetic.get_pos();
                pos_pub_em.publish(pos);
                quat_em = electromagnetic.get_quat();
                quat_pub_em.publish(quat_em);
                visible_em.data = electromagnetic.is_visible();
                bool_em.publish(visible_em);
            }
            else
            {
                //if Aurora data are not visible, just send out the old ones and the fusion node will take care (or maybe send a signal to change the variances, to be CHECKED)
                geometry_msgs::Vector3Stamped pos = electromagnetic.get_pos_prev();
                pos_pub_em.publish(pos);
                quat_em = electromagnetic.get_quat_prev();
                quat_pub_em.publish(quat_em);
                visible_em.data = electromagnetic.is_visible();
                bool_em.publish(visible_em);
            }


            if((first)&&(inertial.is_initialized()))
            {
                quaternion << quat.quaternion.x, quat.quaternion.y, quat.quaternion.z, quat.quaternion.w;
                sinr_cosp = 2*(quaternion(3)*quaternion(0) + quaternion(1)*quaternion(2));
                cosr_cosp = 1 - 2*(quaternion(0)*quaternion(0) + quaternion(1)*quaternion(1));
                prev_roll = atan2(sinr_cosp, cosr_cosp);
                sinp = 2*(quaternion(3)*quaternion(1) - quaternion(2)*quaternion(0));
                if(abs(sinp) >= 1)
                {
                    prev_pitch = copysign(PI/2, sinp);
                }
                else
                {
                    prev_pitch = asin(sinp);

                }
                siny_cosp = 2*(quaternion(3)*quaternion(2) + quaternion(0)*quaternion(1));
                cosy_cosp = 1 - 2*(quaternion(1)*quaternion(1) + quaternion(2)*quaternion(2));
                prev_yaw = atan2(siny_cosp, cosy_cosp);
                first = false;
            }

            //The XSens estimation is accurate but a drift error can be seen in the yaw estimate when magnetic disturbances are present
            //This is a simple code to get rid off of this drift
            quaternion << quat.quaternion.x, quat.quaternion.y, quat.quaternion.z, quat.quaternion.w;
            quaternion_imu = quaternion;
            //Conversion from quaternion to euler
            //for what concern the estimated angle, we want to update them only if the corresponding w component is different from zero (greater then a threshold, as 0.01 rad/s -> 0,5°/sec)
            sinr_cosp = 2*(quaternion(3)*quaternion(0) + quaternion(1)*quaternion(2));
            cosr_cosp = 1 - 2*(quaternion(0)*quaternion(0) + quaternion(1)*quaternion(1));
            if(abs(ang_vel.vector.x) > pow(10,-2))
            {
                roll = wrapAngle(atan2(sinr_cosp, cosr_cosp) - roll_dev);
            }
            else
            {
                //the sensor is not really moving
                roll_dev = wrapAngle(atan2(sinr_cosp, cosr_cosp) - prev_roll);
                roll = prev_roll;

            }
            sinp = 2*(quaternion(3)*quaternion(1) - quaternion(2)*quaternion(0));
            if(abs(ang_vel.vector.y) > pow(10,-2))
            {
                if(abs(sinp) >= 1)
                {
                    pitch = copysign(PI/2, sinp);
                }
                else
                {
                    pitch = asin(sinp);

                }
                pitch = wrapAngle(pitch - pitch_dev);
            }
            else
            {
                //the sensor is not really moving
                if(abs(sinp) >= 1)
                {
                    pitch = copysign(PI/2, sinp);
                }
                else
                {
                    pitch = asin(sinp);

                }
                pitch_dev = wrapAngle(pitch - prev_pitch);
                pitch = prev_pitch;

            }
            siny_cosp = 2*(quaternion(3)*quaternion(2) + quaternion(0)*quaternion(1));
            cosy_cosp = 1 - 2*(quaternion(1)*quaternion(1) + quaternion(2)*quaternion(2));
            if(abs(ang_vel.vector.z) > pow(10,-2))
            {
                yaw = wrapAngle(atan2(siny_cosp, cosy_cosp) - yaw_dev);
            }
            else
            {
                //the sensor is not really moving
                yaw_dev = wrapAngle(atan2(siny_cosp, cosy_cosp) - prev_yaw);
                yaw = prev_yaw;

            }
            prev_roll = roll;
            prev_pitch = pitch;
            prev_yaw = yaw;
            //Go back to quaternion
            qw = cos(roll/2)*cos(pitch/2)*cos(yaw/2)+sin(roll/2)*sin(pitch/2)*sin(yaw/2);
            qx = sin(roll/2)*cos(pitch/2)*cos(yaw/2)-cos(roll/2)*sin(pitch/2)*sin(yaw/2);
            qy = cos(roll/2)*sin(pitch/2)*cos(yaw/2)+sin(roll/2)*cos(pitch/2)*sin(yaw/2);
            qz = cos(roll/2)*cos(pitch/2)*sin(yaw/2)-sin(roll/2)*sin(pitch/2)*cos(yaw/2);
            quaternion << qx,qy,qz,qw;


            //IMU to EM registration
            //quaternion registration
            //quaternion_imu = quaternion;
            //quaternion = quaternionProduct(rot_quaternion,quaternionProduct(quaternion_imu,rot_quaternion2));
            /*
            //angular velocity registration
            Eigen::VectorXd ang_vel_quat = Eigen::VectorXd::Zero(4);
            ang_vel_quat << ang_vel.vector.x,ang_vel.vector.y,ang_vel.vector.z,0;
            ang_vel_quat = quaternionProduct(rot_quaternion,quaternionProduct(ang_vel_quat,quaternionConj(rot_quaternion)));
            geometry_msgs::Vector3Stamped ang_vel2;
            ang_vel2.vector.x = ang_vel_quat(0);
            ang_vel2.vector.y = ang_vel_quat(1);
            ang_vel2.vector.z = ang_vel_quat(2);
            */

            //Publish my quaternion estimate
            geometry_msgs::QuaternionStamped my_quat;
            my_quat.quaternion.w = quaternion(3);
            my_quat.quaternion.x = quaternion(0);
            my_quat.quaternion.y = quaternion(1);
            my_quat.quaternion.z = quaternion(2);
            my_quat_pub.publish(my_quat);

        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}




/*
 * Code for the Kalman Filter attempt
 * //The XSens quaternion estimation is based on fusion of gyroscope, inertial and magnetic data, relying more on gyroscope when altered conditions are sensed.
            //If the altered conditions are mantained in time, then a drift in angles can be seen (due to integration error of gyroscope, which is no more corrected).
            //Our estimation is still based on the fusion between gyroscope, inertial and magnetic data through a LKF, but a vector selection strategy is adopted for yaw estimation.
            //The sensed magnetic field is compared to the expected one in order to understand if disturbances are superimposed.
            //If no disturbances are sensed, then yaw is directly estimated from magnetometer.
            //If disturbances are detected, then the yaw estimated from magnetometer is not correct so we should rely more on the gyroscope.
            //On the other hand, if the disturbances are mantained in time, then the yaw estimate from gyroscope is affected by drift errors.
            //If the disturbances are mantained and stable in time, then the data from magnetometer can be used to compute yaw increments in order to update the previous value.
            //This increment are precise and reliable only if the magnetic field is stable in time (even if perturbed) if a new perturbation is sensed, then the perturbed magnetic
            //field reference should be updated and the istantaneous yaw value can be computed from gyroscope (since we are not integrating angular velocities for a long time, then no
            //drift error occurs).

    //Parameter Initialization for LKF
    int freq = 100;                                             //sampling rate Hz
    double dt = (double) 1/freq;                                         //delta t for acquisition frequency
    //criterion thresholds for vector selection (to be found)
    double expected_norm_acc = 9.81;                            //m/s^2
    double expected_norm_mag = 47.172;                          //mT
    double th_a1 = 0.1*expected_norm_acc;
    double th_a2 = 0.1*expected_norm_acc;
    double th_m1 = 0.05*expected_norm_mag;
    double th_m2 = 0.05*expected_norm_mag;
    //Initialization of quaternion vector
    Eigen::VectorXd quaternion = Eigen::VectorXd::Zero(4);
    //Definition of Model Covariance matrix P
    Eigen::Matrix4d P = Eigen::Matrix4d::Zero();                //assume to know perfectly the initial state
    double var_p = 0.00001;
    P(0,0) = var_p;
    P(1,1) = var_p;
    P(2,2) = var_p;
    P(3,3) = var_p;
    //Definition of Model Error Covariance matrix Q
    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    //considering values from "Research on Intraoperative Organ Tracking"
    double var = 0.00003;
    Q(0,0) = var;
    Q(1,1) = var;
    Q(2,2) = var;
    Q(3,3) = var;
    //Definition of Measurament Error Covariance Matrix R
    //Accelerometer variance
    double noise_density = 120;                                 //microg/sqrt(Hz)
    double var_acc = (double)noise_density*pow(10,-6)*9.81*sqrt(freq);       //m/s^2
    var_acc = pow(var_acc,2);
    double RMS_mag = 0.5;                                       //mG
    //1T = 10000G, hence 1mT = 10000mG
    RMS_mag = 0.5*pow(10,-4);                                        //mT
    double var_mag = RMS_mag*RMS_mag;
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(4,4);
    //Vector and matrix declaration
    Eigen::VectorXd angular_vel = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd F = Eigen::Matrix4d::Zero();                //Process Model matrix F
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4,4);             //Measurement Model matrix H
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(4,4);             //Kalman Gain matrix K
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(4,4);             //Innovation Covariance matrix S
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(4,4);         //4x4 Identity matrix
    Eigen::VectorXd predict_quat = Eigen::VectorXd::Zero(4);    //quaternion prediction
    Eigen::VectorXd measurement = Eigen::VectorXd::Zero(4);     //measurement vector
    Eigen::VectorXd innovation = Eigen::VectorXd::Zero(4);      //innovation vector
    Eigen::Vector4d global_g = Eigen::Vector4d::Zero();         //gravity in global reference
    global_g(2) = 9.81;                                         //m/s^2
    Eigen::Vector4d global_mag = Eigen::Vector4d::Zero();       //Earth magnetic field in global reference
    global_mag << 23.576,1.396,40.834,0;                        //magnetic field vector in Pontedera 16/11/2021 in mT (can be computed with calculators online)
    Eigen::Vector4d predict_acc = Eigen::Vector4d::Zero();
    Eigen::Vector4d predict_mag = Eigen::Vector4d::Zero();

    Eigen::VectorXd predict_quat_in = Eigen::VectorXd::Zero(4);
    Eigen::Vector4d meas_acc = Eigen::Vector4d::Zero();
    Eigen::Vector4d meas_acc_free = Eigen::Vector4d::Zero();
    Eigen::Vector4d meas_acc_prev = Eigen::Vector4d::Zero();

    double yaw_mag = 0;
    double prev_yaw_mag = 0;
    double altered_mag_norm = -1;
    bool first_mag_alt = true;
    double magx = 0;
    double magy = 0;

 * //My Kalman Filter
            if((first)&&(inertial.is_initialized()))
            {
                quaternion << quat.quaternion.x, quat.quaternion.y, quat.quaternion.z, quat.quaternion.w;
                double siny_cosp = 2*(quaternion(3)*quaternion(2) + quaternion(0)*quaternion(1));
                double cosy_cosp = 1 - 2*(quaternion(1)*quaternion(1) + quaternion(2)*quaternion(2));
                prev_yaw = atan2(siny_cosp, cosy_cosp);
                first = false;
            }
            angular_vel << ang_vel.vector.x,ang_vel.vector.y,ang_vel.vector.z,0;        //the angular velocity from topics is already in rad/sec
            //---Prediction Step---
            //Definition of Process Model matrix F
            double ang_vel_norm = angular_vel.norm();
            if(ang_vel_norm > pow(10,-1))
            {
                F = cos((ang_vel_norm*dt)/2)*I + (1/ang_vel_norm)*sin((ang_vel_norm*dt)/2)*matrixL(angular_vel);
            }
            else
            {
                //if the norm of angular velocity tends to zero, then we fall into the case cos(0) = 1 and lim_x->0 sin(ax)/x = a so
                F = I + (dt/2)*matrixL(angular_vel);
            }
            predict_quat = F*quaternion;
            //---Measurement Step---
            //Let's assume that we can compute directly quaternion from acc and mag and propagate the uncertainty
            geometry_msgs::Vector3Stamped acc_g = inertial.get_accg();
            meas_acc_prev = meas_acc;
            meas_acc << acc_g.vector.x, acc_g.vector.y, acc_g.vector.z, 0;
            meas_acc_free << acc.vector.x, acc.vector.y, acc.vector.z, 0;
            meas_acc = meas_acc - meas_acc_free;                                //this should be just the gravity component
            double conv_factor = 0.47/10;                                       //conversion factor from AU to Gauss units and then to T
            conv_factor = conv_factor*1000;                                     //from T to mT
            Eigen::Vector4d meas_mag = Eigen::Vector4d::Zero();
            meas_mag << mag.vector.x*conv_factor,mag.vector.y*conv_factor,mag.vector.z*conv_factor,0;
            //Vector selection substep
            //Inspired by "Quaternion-Based Kalman Filter With Vector Selection for Accurate Orientation Tracking" a vector selection step is added in the measurement model in order to cut off possible
            //errors in quaternion measurement deriving from altered inertial or magnetic conditions. If altered inertial or magnetic conditions are sensed the measurement model relies on gyroscope
            //measurements for that steps
            ///*predict_acc = quaternionProduct(quaternionInverse(quaternion),quaternionProduct(global_g, quaternion));
            predict_mag = quaternionProduct(quaternionInverse(quaternion),quaternionProduct(global_mag, quaternion));
            if(abs(global_g.norm()-meas_acc.norm())>th_a1 || abs(predict_acc.norm()-meas_acc.norm())>th_a2)
            {
                meas_acc = predict_acc;
            }
            if(abs(global_mag.norm()-meas_mag.norm())>th_m1 || abs(predict_mag.norm()-meas_mag.norm())>th_m2)
            {
                meas_mag = predict_mag;
            }
            //the pitch and roll estimates are quite stable with respect to measured acceleration, so we are confident in directly compute them from meas_acc
            roll = atan2(meas_acc(1),sqrt(meas_acc(0)*meas_acc(0)+meas_acc(2)*meas_acc(2)));
            pitch = atan2(-meas_acc(0),sqrt(meas_acc(1)*meas_acc(1)+meas_acc(2)*meas_acc(2)));
            //for what concern the yaw angle, we want to update it only if the w_z is different from zero (greater then a threshold, as 0.01 rad/s -> 0,5°/sec)
            //keep track of previous yaw value
            if(abs(angular_vel(2)) > pow(10,-2))
            {
                //we are really moving with the sensor
                predict_mag = quaternionProduct(quaternionInverse(quaternion),quaternionProduct(global_mag, quaternion));
                if(abs(global_mag.norm()-meas_mag.norm())>th_m1)         //we evaluate the difference in magnitude between the measured magnetic field and the Earth one and the magnitude of the difference between the predicted one (based on previous orientation) and the measured one (we assume the small changes in orientation occur from one time step to the other)
                {
                    //magnetic field alteration case
                    if(altered_mag_norm == -1 || abs(altered_mag_norm-meas_mag.norm())>th_m1)
                    {
                        //We are in the first magnetic field alteration or a new superimposed one, so update the reference
                        altered_mag_norm = meas_mag.norm();
                        //in this case the yaw increments cannot be computed from magnetometer, but for yaw estimation we need to rely only on gyro
                        double siny_cosp = 2*(predict_quat(3)*predict_quat(2) + predict_quat(0)*predict_quat(1));
                        double cosy_cosp = 1 - 2*(predict_quat(1)*predict_quat(1) + predict_quat(2)*predict_quat(2));
                        yaw = atan2(siny_cosp, cosy_cosp);
                        //update the yaw_mag reference
                        magx =meas_mag(0)*cos(pitch)+meas_mag(1)*sin(roll)*sin(pitch)+meas_mag(2)*cos(roll)*sin(pitch);
                        magy = meas_mag(1)*cos(roll)-meas_mag(2)*sin(roll);
                        prev_yaw_mag = atan2(-magy,magx);
                        flag = 0;                   //we sensed a new mag field alteration and the update is based on gyroscope
                    }
                    else
                    {
                        //The magnetic field is still altered, but in a constant way (i.e., no new superimposed interferences are sensed)
                        //so the magnetic field information can be used to estimate the yaw increments and update the yaw previous value
                        //compute the new altered yaw value
                        magx =meas_mag(0)*cos(pitch)+meas_mag(1)*sin(roll)*sin(pitch)+meas_mag(2)*cos(roll)*sin(pitch);
                        magy = meas_mag(1)*cos(roll)-meas_mag(2)*sin(roll);
                        yaw_mag = atan2(-magy,magx);
                        //update the yaw value with mag retrieved increments
                        if(((yaw_mag-prev_yaw_mag)>pow(10,-2)) && ((yaw_mag-prev_yaw_mag)<pow(10,1)))
                        {
                            //we are just moving with the sensor, so the yaw can be updated with the change in magnetic field
                            yaw = wrapAngle(prev_yaw + (yaw_mag-prev_yaw_mag));             //use wrapAngle to keep yaw in [-pi,pi]
                            flag = 1;                       //gyro sense movements and we use magnetic field
                        }
                        else
                        {
                            //both the sensor and magnetic disturbane source are moving in such a way that the sensed magnetic field is constant
                            //so to update the yaw angle we need to use the gyroscope measurements
                            double siny_cosp = 2*(predict_quat(3)*predict_quat(2) + predict_quat(0)*predict_quat(1));
                            double cosy_cosp = 1 - 2*(predict_quat(1)*predict_quat(1) + predict_quat(2)*predict_quat(2));
                            yaw = atan2(siny_cosp, cosy_cosp);
                            flag = 2;                       //gyro sense movements and we use gyro
                        }
                        //update the yaw_mag reference
                        prev_yaw_mag = yaw_mag;
                    }
                }
                else
                {
                    //the sensor is moving and we can trust the magnetometer to update yaw
                    magx =meas_mag(0)*cos(pitch)+meas_mag(1)*sin(roll)*sin(pitch)+meas_mag(2)*cos(roll)*sin(pitch);
                    magy = meas_mag(1)*cos(roll)-meas_mag(2)*sin(roll);
                    yaw = atan2(-magy,magx);
                    altered_mag_norm = -1;
                    flag = 3;                                   //no mag alteration
                }
            }
            else
            {
                //the sensor is not really moving
                yaw = prev_yaw;
                flag = 4;                           //no gyro movements;
            }
            prev_yaw = yaw;
            double qw = cos(roll/2)*cos(pitch/2)*cos(yaw/2)+sin(roll/2)*sin(pitch/2)*sin(yaw/2);
            double qx = sin(roll/2)*cos(pitch/2)*cos(yaw/2)-cos(roll/2)*sin(pitch/2)*sin(yaw/2);
            double qy = cos(roll/2)*sin(pitch/2)*cos(yaw/2)+sin(roll/2)*cos(pitch/2)*sin(yaw/2);
            double qz = cos(roll/2)*cos(pitch/2)*sin(yaw/2)-sin(roll/2)*sin(pitch/2)*cos(yaw/2);
            measurement << qx,qy,qz,qw;
            H = I;
            innovation = measurement - H*predict_quat;
            //propagation of uncertainty
            double c = sqrt(meas_acc(0)*meas_acc(0)+meas_acc(2)*meas_acc(2));
            double c1 = sqrt(meas_acc(1)*meas_acc(1)+meas_acc(2)*meas_acc(2));
            double a = pow(magx/(pow(magx,2)+pow(magy,2)),2);
            double a1 = pow(magy/(pow(magx,2)+pow(magy,2)),2);
            double var_roll = (pow((c/pow(meas_acc.norm(),2)),2) + (pow(meas_acc(1)*meas_acc(0),2)+pow(meas_acc(1)*meas_acc(2),2))/pow(pow(meas_acc.norm(),2)*c,2))*var_acc;
            double var_pitch = (pow((c1/pow(meas_acc.norm(),2)),2) + (pow(meas_acc(0)*meas_acc(1),2)+pow(meas_acc(0)*meas_acc(2),2))/pow(pow(meas_acc.norm(),2)*c1,2))*var_acc;
            double var_magx = (pow(cos(pitch),2)+pow(sin(roll)*sin(pitch),2)+pow(cos(roll)*sin(pitch),2))*var_mag + pow(meas_mag(1)*cos(roll)*sin(pitch)-meas_mag(2)*sin(roll)*sin(pitch),2)*var_roll + pow(-meas_mag(0)*sin(pitch)+meas_mag(1)*sin(roll)*cos(pitch)+meas_mag(2)*cos(roll)*cos(pitch),2)*var_pitch;
            double var_magy = (pow(cos(roll),2)+pow(sin(roll),2))*var_mag + pow(-meas_mag(1)*sin(roll)-meas_mag(2)*cos(roll),2)*var_roll;
            double var_yaw = pow(magx/(pow(magx,2)+pow(magy,2)),2)*var_magy + pow(magy/(pow(magx,2)+pow(magy,2)),2)*var_magx;
            double var_qw = pow(-0.5*sin(roll/2)*cos(pitch/2)*cos(yaw/2)+0.5*cos(roll/2)*sin(pitch/2)*sin(yaw/2),2)*var_roll + pow(-0.5*cos(roll/2)*sin(pitch/2)*cos(yaw/2)+0.5*sin(roll/2)*cos(pitch/2)*sin(yaw/2),2)*var_pitch + pow(-0.5*cos(roll/2)*cos(pitch/2)*sin(yaw/2)+0.5*sin(roll/2)*sin(pitch/2)*cos(yaw/2),2)*var_yaw;
            double var_qx = pow(0.5*cos(roll/2)*cos(pitch/2)*cos(yaw/2)+0.5*sin(roll/2)*sin(pitch/2)*sin(yaw/2),2)*var_roll + pow(-0.5*sin(roll/2)*sin(pitch/2)*cos(yaw/2)-0.5*cos(roll/2)*cos(pitch/2)*sin(yaw/2),2)*var_pitch + pow(-0.5*sin(roll/2)*cos(pitch/2)*sin(yaw/2)-0.5*cos(roll/2)*sin(pitch/2)*cos(yaw/2),2)*var_yaw;
            double var_qy = pow(-0.5*sin(roll/2)*sin(pitch/2)*cos(yaw/2)+0.5*cos(roll/2)*cos(pitch/2)*sin(yaw/2),2)*var_roll + pow(0.5*cos(roll/2)*cos(pitch/2)*cos(yaw/2)-0.5*sin(roll/2)*sin(pitch/2)*sin(yaw/2),2)*var_pitch + pow(-0.5*cos(roll/2)*sin(pitch/2)*sin(yaw/2)+0.5*sin(roll/2)*cos(pitch/2)*cos(yaw/2),2)*var_yaw;
            double var_qz = pow(-0.5*sin(roll/2)*cos(pitch/2)*sin(yaw/2)-0.5*cos(roll/2)*sin(pitch/2)*cos(yaw/2),2)*var_roll + pow(-0.5*cos(roll/2)*sin(pitch/2)*sin(yaw/2)-0.5*sin(roll/2)*cos(pitch/2)*cos(yaw/2),2)*var_pitch + pow(0.5*cos(roll/2)*cos(pitch/2)*cos(yaw/2)+0.5*sin(roll/2)*sin(pitch/2)*sin(yaw/2),2)*var_yaw;
            //There seems to be issues in variance propagation if no vector selection is performed (probably we get too smal values, I don't know), so let's fix the variances to constants
            R(0,0) = 5*var;
            R(1,1) = 5*var;
            R(2,2) = 5*var;
            R(3,3) = 5*var;
            //---LKF Update Step---
            P = F*P*F.transpose() + Q;
            S = H*P*H.transpose() + R;
            K = P*H.transpose()*S.inverse();
            quaternion = predict_quat + K*innovation;
            P = (I - K*H)*P;
            if(quaternion.norm() != 0)
            {
                quaternion.normalize();
            }

            geometry_msgs::Vector3Stamped meas_m = inertial.get_mag();
            meas_m.vector.x = th_m1;
            meas_m.vector.y = angular_vel(2);
            meas_m.vector.z = flag;
            mag_pub.publish(meas_m);





        Eigen::Vector4d quaternionProduct(Eigen::Vector4d q, Eigen::Vector4d p)
        {
            //This function implements the quaternion product q*p
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
            return inverse;
        }
        double quaternionNorm(Eigen::Vector4d q)
        {
            //This function compute the norm of a 4D vector
            double norm = sqrt(q(0)*q(0)+q(1)*q(1)+q(2)*q(2)+q(3)*q(3));
            return norm;
        }
        Eigen::Matrix4d matrixR(Eigen::Vector4d q)
        {
            //This function compute the matrix R for a 4D vector with final element 0
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
        Eigen::Matrix4d matrixL(Eigen::Vector4d q)
        {
            //This function compute the matrix R for a 4D vector with final element 0
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
*/
