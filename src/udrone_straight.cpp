#include <ros/ros.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>

#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include <iostream>
#define SINE_TRAJECTORY 0
#define NEGATIVE_SINE_TRAJECTORY 0
// #define STRAIGHT_TRAJECTORY 0
// #define LEADER 0


//global variables for pose and twist values
Eigen::Vector3d current_position(0, 0, 0), desired_position(0, 0, 0);
Eigen::Quaterniond current_orientation(1, 0, 0, 0), desired_orientation(1, 0, 0, 0);
Eigen::Vector3d current_linear_velocity(0, 0, 0), desired_linear_velocity(0, 0, 0);
Eigen::Vector3d current_angular_velocity(0, 0, 0), desired_angular_velocity(0, 0, 0);

//pose for leader to send to follower
ros::Publisher pub_reef_follow_setpoint;
geometry_msgs::PoseStamped target_pose;
static int seq = 1;

//gobal variables for Integration control
int error_buffer = 100;
int integration_terms = 2;
std::vector<Eigen::Vector3d> position_error_history;


//desired accleration
Eigen::Vector3d desired_acceleration(0, 0, 0);

//desired attitude + thrust
mavros_msgs::AttitudeTarget desired_attitude_thrust;

//desired roll angle as it does not affect the position. (i.e. pitch and yaw do affect the position)
double desired_roll = 0;

//properties
double mass = 1.4; //kgs
double g = 9.8; //m per s2

//thrust normalization factor
double thrust_normalization_factor = 10/27.468; // using formula mg*n = 0.5

//surface area of hippocampus
double surface_area = 1; //m2
//density of water
double rho = 1; //kg/m3 of
//coffecient of drag
double cd = 0.1;

bool leader = 0;

//starting depth
// double start_depth = -2.0;

void mavrosGlobalPositionCallback(const geometry_msgs::PoseStampedPtr msg) {
    current_position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

    current_orientation.x() = msg->pose.orientation.x;
    current_orientation.y() = msg->pose.orientation.y;
    current_orientation.z() = msg->pose.orientation.z;
    current_orientation.w() = msg->pose.orientation.w;
}

void mavrosOdomCallback(nav_msgs::OdometryPtr msg){
    current_linear_velocity << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
    current_angular_velocity << msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z;
}


void desiredPoseCallback(geometry_msgs::PoseStampedPtr msg){
    desired_position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    desired_orientation.x() = 0;
    desired_orientation.y() = 0;
    desired_orientation.z() = 0;
    desired_orientation.w() = 1;

}

void desiredTwistCallback(nav_msgs::OdometryPtr msg){
    desired_linear_velocity << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
    desired_angular_velocity << msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z;

}

void sonarCallback(sensor_msgs::Range msg){
    if(leader){
        target_pose.header.seq = seq++;

        target_pose.pose.position.x = current_position[0];
        target_pose.pose.position.y = current_position[1];
        if(msg.range == msg.max_range) target_pose.pose.position.z = current_position[2];
        else target_pose.pose.position.z = (current_position[2]-msg.range+1.0);

        pub_reef_follow_setpoint.publish(target_pose);
        std::cout<<target_pose<<std::endl;
    }
    else std::cout<<msg.range<<std::endl;
}

void initializeGlobalVariables() {
    current_position << 0, 0, 0;

    current_orientation.x() = 0;
    current_orientation.y() = 0;
    current_orientation.z() = 0;
    current_orientation.w() = 1;

    current_linear_velocity << 0, 0, 0;
    current_angular_velocity << 0, 0, 0;

    desired_position << 0, 0, 0 ;
    desired_orientation.x() = 0;
    desired_orientation.y() = 0;
    desired_orientation.z() = 0;
    desired_orientation.w() = 1;

    target_pose.pose.orientation.x = 0;
    target_pose.pose.orientation.y = 0;
    target_pose.pose.orientation.z = 0;
    target_pose.pose.orientation.w = 1;

    desired_linear_velocity << 0, 0, 0;
    desired_angular_velocity << 0, 0, 0;

    //desired attitude thrust message initialization

    desired_attitude_thrust.orientation.x = 0;
    desired_attitude_thrust.orientation.y = 0;
    desired_attitude_thrust.orientation.z = 0;
    desired_attitude_thrust.orientation.w = 1;

    desired_attitude_thrust.thrust = 0;


    //desired accceleration
    desired_acceleration << 0, 0, 0;

    //set size of error buffer
    position_error_history.resize(error_buffer, Eigen::Vector3d::Zero());

   
}

//debug helper
template <class T>
void print( T a){
    std::cout<<a<<std::endl;
}

void getSineWaveDesiredPoint(){
    static int i = 0;
    double Ts = 0.005;
    double omega = 1;
    double t = Ts*i;
    desired_position[0] = omega*t;
    desired_position[1] = 0;
    desired_position[2] = sin(desired_position[0]);
    i++;
}
void getNegativeSineWaveDesiredPoint(){
    static int i = 0;
    double Ts = 0.005;
    double omega = 1;
    double t = Ts*i;
    desired_position[0] = -omega*t;
    desired_position[1] = 0;
    desired_position[2] = -sin(desired_position[0]);
    i++;
}
void getStraightDesiredPoint(){
    static int i = 0;
    double Ts = 0.005;
    double omega = 1;
    double t = Ts*i;
    desired_position[0] = omega*t;
    desired_position[1] = 0;
    desired_position[2] = 0;
    i++;
}

void setDesiredAttitudeThrust() {
    //pd control steps
    //gains
    double kp = 0.5;
    Eigen::Matrix3d kp_mat;
    kp_mat << kp, 0, 0,
              0, kp, 0,
              0, 0, kp;

    double kd = 0.05;
    Eigen::Matrix3d kd_mat;
    kd_mat << kd, 0, 0,
              0, kd, 0,
              0, 0, kd;
    Eigen::Vector3d e3(0, 0, 1);

    double ki = 0.005;
    Eigen::Matrix3d ki_mat;
    ki_mat << ki, 0, 0,
              0, ki, 0,
              0, 0, ki;

    double buoyant_force = mass*g; // For now asssuming that the density of AUV is same as that of water.
    if(SINE_TRAJECTORY) getSineWaveDesiredPoint();
    if(NEGATIVE_SINE_TRAJECTORY) getNegativeSineWaveDesiredPoint();
    if(leader) getStraightDesiredPoint();

    // errors
    Eigen::Vector3d position_error = desired_position - current_position;
    Eigen::Vector3d velocity_error = desired_linear_velocity - current_linear_velocity;
    Eigen::Vector3d integral_error = Eigen::Vector3d::Zero();
    for(int i = 0; i < integration_terms; ++i){
        integral_error += position_error_history.at(i);
    }
    Eigen::Vector3d drag_force;
    drag_force <<  0.5*rho*surface_area*cd*current_linear_velocity[0]*current_linear_velocity[0],
                   0.5*rho*surface_area*cd*current_linear_velocity[1]*current_linear_velocity[1],
                   0.5*rho*surface_area*cd*current_linear_velocity[2]*current_linear_velocity[2];

    Eigen::Vector3d force = kp_mat*position_error + kd_mat*velocity_error + ki_mat*integral_error + mass*g*e3 - buoyant_force*e3 + desired_acceleration; //TODO(ALG): Add the buoyancy and drag
    //print(force);
    //print(force.norm());
    //get rotation matrices
    Eigen::Matrix3d orientation_mat = current_orientation.normalized().toRotationMatrix();
    Eigen::Vector3d thrust_vector = orientation_mat.transpose()*force;

    //assign thrust values
    desired_attitude_thrust.thrust = thrust_vector[0]*thrust_normalization_factor + 0.5; //because only body frame x-axis force matters
    //std::cout<< desired_attitude_thrust.thrust <<std::endl;
    //std::cout<<force.norm()<<std::endl;

    ///desired orientation calculation
    Eigen::Vector3d b1, b2, b3;
    if(force.norm() > 0.2) {
        if(thrust_vector[0] > 0) b1 = force / force.norm();
        else b1 = -force/force.norm();

        //std::cout << b1 << std::endl;
        //convert desired orientation
        b2 << 0, cos(desired_roll), sin(desired_roll);

        Eigen::Vector3d b1xb2 = b1.cross(b2);
        //std::cout << b1xb2 << std::endl;
        b3 = b1xb2 / (b1xb2.norm());
        //std::cout << b3 << " " << b3.norm() << std::endl;

        b2 = b3.cross(b1);
    }

    /*else {

        b2 << 0, cos(desired_roll), sin(desired_roll);
        b3 << 0, -sin(desired_roll), cos(desired_roll);
        Eigen::Vector3d b2xb3 =  b2.cross(b3);
        b1 = b2xb3/b2xb3.norm();
    }*/

    Eigen::Matrix3d desired_rotation_matrix;
    desired_rotation_matrix << b1, b2, b3;
    //std::cout << desired_rotation_matrix << std::endl;

    //assign desired orientation values to target attitude thrust
    Eigen::Quaterniond desired_quaternion(desired_rotation_matrix);
    desired_attitude_thrust.orientation.x = desired_quaternion.x();
    desired_attitude_thrust.orientation.y = desired_quaternion.y();
    desired_attitude_thrust.orientation.z = desired_quaternion.z();
    desired_attitude_thrust.orientation.w = desired_quaternion.w();

    //update position error buffer
    position_error_history.insert(position_error_history.begin(),position_error);
    position_error_history.resize(error_buffer, Eigen::Vector3d::Zero());

}

int main(int argc, char *argv[]){

    ros::init(argc, argv, "hippocampus_pid_control");
    std::string param;
    ros::NodeHandle nh("~");
    std::string check;
    nh.getParam("param", check);
    if(check.compare("leader") == 0) {
        leader = 1;
        std::cout<<"Leader mode"<<std::endl;
    }

    // Publisher and Subsriber stuff
    ros::Publisher pub_mavros_setpoint_attitude = nh.advertise<mavros_msgs::AttitudeTarget>("/hippocampus/setpoint_raw/attitude", 1000);

    // Publisher for leader drone to publishe target terrain follow setpoints
    pub_reef_follow_setpoint = nh.advertise<geometry_msgs::PoseStamped>("/hippocampus/desired_pose", 10);

    ros::Subscriber sub_sonar = nh.subscribe("/sensor/sonar0", 10, sonarCallback);
    
    //get current pose and twist
    // current pose
    ros::Subscriber sub_current_pose = nh.subscribe("/mavros/local_position/pose", 1000, mavrosGlobalPositionCallback);
    //desired_pose
    ros::Subscriber sub_desired_pose = nh.subscribe("/hippocampus/desired_pose", 1, desiredPoseCallback);

    //current twist
    ros::Subscriber sub_current_twist = nh.subscribe("/mavros/local_position/odom", 1, mavrosOdomCallback);
    //desired twist
    ros::Subscriber sub_desired_twist = nh.subscribe("/hippocampus/desired_twist", 1, desiredTwistCallback);

    ros::Rate rate(50);

    initializeGlobalVariables();

    //rosparam for desired position
    //nh.param("desired")


    while(ros::ok()){

        setDesiredAttitudeThrust();
        pub_mavros_setpoint_attitude.publish(desired_attitude_thrust);
        ros::spinOnce();
        rate.sleep();
    }

}