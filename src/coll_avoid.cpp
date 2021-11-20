#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "collision_avoidance/Pose.h"


/* p_i(x,y) = posa ostacolo
t (x,y) = posa robot

t-p_i = direzione forza risultante
modulo forza risultante: 1/norm(t, p_i) 

\sum_i fi + cmd_vel*/

/* --------VARIABILI GLOBALI---------*/

///Velocità lineare
float linear_speed;
///Velocità angolare
float angular_speed;
//Distanza di threshold a 0.2
const float distanceThreshold = 0.2;
//Variabile che identifica se l'ostacolo è stato rilevato
bool obstacle_detected;
//Per pubblicare e velocità
geometry_msgs::Twist velocities;

geometry_msgs::Pose position;

float prevLinearVelocity, prevAnguarVelocity;

//Prende in input da terminale il valore della velocità angolare e lineare
void setVelocity() {
  std::cout << "Inserisci un comando di velocità: \nlinear_speed = " ;
  std::cin >> linear_speed;
  std::cout << "angular_speed = ";
  std::cin >> angular_speed;
}

//Restituisce il valore booleano corrispondente a obstacle_detected
bool getObstacleDetected() {
  return obstacle_detected;
}

//Se l'ostacolo è stato rilevato stampa un messaggio di avviso 
bool checkObstacle() {
  /// Check if obstacle is ahead
  if (getObstacleDetected()) {
    ROS_WARN_STREAM("Obstacle detected!");
    return true;
  }
  return false;
}

//Setta la variabile obstacle_detected al valore corrispondente, 
//se l'ostacolo è stato rilevato o meno
void setObstacleDetected(bool obstacle) {
    obstacle_detected = obstacle;
  }

/// Il LaserScan legge la distanza dall'ostacolo e la confronta con quella di threshold
void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& sensorData) {
  ///# range data [m] (Note: values < range_min or > range_max should be discarded) 
  for (const float &range : sensorData->ranges) {
    if (range < distanceThreshold) {
      setObstacleDetected(true);
      return;
    }
  }
  setObstacleDetected(false);
}

/// Linear and angular change simultaneously
/// Check if both the velocities have changed
bool checkVelocityChanged() {
  if (velocities.linear.x != prevLinearVelocity and \
      velocities.angular.z != prevAnguarVelocity) {
    ROS_DEBUG_STREAM("Velocity of the robot changed");
    /// Update previous velocities
    velocities.linear.x = prevLinearVelocity;
    velocities.angular.z = prevAnguarVelocity;
    return true;
  }

  return false;
}

void update_position(collision_avoidance::Pose& pose_msg, const float& linear_speed, const float& angular_speed){

    float dx;
    float dy;
    float dtheta;
    float R; //raggio di curvatura

    if (angular_speed == 0) {
        dx = linear_speed;
        dy = 0;
        dtheta = 0;
    }

    else {
        R = linear_speed / angular_speed;
        dtheta= angular_speed;         //(arco lungo il raggio)
        dx = R*sin(dtheta);
        dy = R*(1-cos(dtheta));

    }

    pose_msg.x += cos(pose_msg.theta) * dx - sin(pose_msg.theta) * dy;
    pose_msg.y += sin(pose_msg.theta)  * dx + cos(pose_msg.theta) * dy;
    pose_msg.theta += dtheta;


}


int main (int argc, char **argv ) {

    ros::init(argc,argv,"collision_avoidance");
   
    ros::NodeHandle n;

    ROS_INFO_STREAM("Setting up the robot config for obstacle avoidance...");
    /// Initialize previous with the current value of velocities

    bool setupVelocity = false;
    while (!setupVelocity) {
      
      setVelocity();

      if (linear_speed !=0 || angular_speed!=0) {
    
        ROS_INFO("linear_speed: [%f]", linear_speed);
        ROS_INFO("angular_speed: [%f]", angular_speed);
        setupVelocity = true;
      }

      else {
        ROS_INFO("La velocità deve essere diversa da zero: ");
      }
    }
    

      prevLinearVelocity = linear_speed;
      prevAnguarVelocity = angular_speed;
      /// Initialize obstacle detected value with false
      obstacle_detected = false;
      /// Publish the velocities to the robot on the navigation topic
      ros::Publisher publishVelocity;
    /// Define a subscriber object with topic name and buffer size of messages
    /// Make sure you have subscribed to the correct topic
      ros::Subscriber subscibeSensor;
      ros::Subscriber subscribePose;

      publishVelocity = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
      /// Subscribe for data from the laser sensor on the scan topic
      subscibeSensor = n.subscribe<sensor_msgs::LaserScan>("/base_scan", 500, sensorCallback);
      
      ROS_INFO_STREAM("Set up complete");
    
    collision_avoidance::Pose pos_msg;

    pos_msg.x = 0;
    pos_msg.y = 0;
    pos_msg.theta = 0;

    ros::Rate loop_rate(2);
    while (ros::ok()) { 
    
    update_position(pos_msg, linear_speed,angular_speed);
    ROS_INFO("Position x: [%f]", pos_msg.x);
    ROS_INFO("Position y: [%f]", pos_msg.y);

      if (checkObstacle()) {
      /// Start turning the robot to avoid obstacles
      velocities.linear.x = 0.0;
      velocities.angular.z = angular_speed;
      /// Check if velocities have changed
      checkVelocityChanged();
    } else {
        /// Start moving the robot once obstacle is avoided
        velocities.angular.z = 0.0;
        velocities.linear.x = linear_speed;
        /// Check if velocities have changed
        checkVelocityChanged();
    }

    /// Publish the velocities
    publishVelocity.publish(velocities);
      ros::spinOnce();

      loop_rate.sleep();
    }
    return 0;
}