#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"


/* p_i(x,y) = posa ostacolo
t (x,y) = posa robot

t-p_i = direzione forza risultante
modulo forza risultante: 1/norm(t, p_i) 

\sum_i fi + cmd_vel*/

float linear_speed;

float angular_speed;

void setVelocity() {

  
    std::cout << "Inserisci un comando di velocità: \nlinear_speed = " ;
      
    std::cin >> linear_speed;

    
    std::cout << "angular_speed = ";
    std::cin >> angular_speed;
    

}

/// Inizializzo la velocita a 0.2m/s
const float linearVelocity = 0.2;
/// Inizializzo la velocita angolare with 30degrees/s
const float anguarVelocity = 0.52;
/// Imposto il valore di threshold a 0.2
const float distanceThreshold = 0.2;
/// Initialize publishing rate
const int rate = 2;
/// Define variable to store if obstacle was detected
bool obstacleDetected;
/// Define variables to store previous velocities
float prevLinearVelocity, prevAnguarVelocity;
/// Define twist object to publish velocities
geometry_msgs::Twist velocities;

//Update position
/*void update_position(collision_avoidance::Pose& pose_msg, const float& linear_speed, const float& angular_speed){

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
        dtheta= angular_speed*T;         //(arco lungo il raggio)
        dx = R*sin(dtheta);
        dy = R*(1-cos(dtheta));
    }

    pose_msg.x += cos(pose_msg.theta) * dx - sin(pose_msg.theta) * dy;
    pose_msg.y += sin(pose_msg.theta)  * dx + cos(pose_msg.theta) * dy;
    pose_msg.theta += dtheta;


}*/

bool getObstacleDetected() {
    return obstacleDetected;
  }


bool checkObstacle() {
  /// Check if obstacle is ahead
  if (getObstacleDetected()) {
    ROS_WARN_STREAM("Obstacle ahead!");
    return true;
  }

  return false;
}

void setObstacleDetected(bool obstacle) {
    obstacleDetected = obstacle;
  }


void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& sensorData) {
  /// Read sensor data to get obstacle distances with respect to the robot
  for (const float &range : sensorData->ranges) {
    if (range < distanceThreshold) {
      setObstacleDetected(true);
      
      return;
    }
  }

  setObstacleDetected(false);
}

bool checkVelocityChanged() {
  /// Linear and angular change simultaneously
  /// Check if both the velocities have changed
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
    }
    

      prevLinearVelocity = linearVelocity;
      prevAnguarVelocity = anguarVelocity;
      /// Initialize obstacle detected value with false
      obstacleDetected = false;
      /// Publish the velocities to the robot on the navigation topic
      ros::Publisher publishVelocity;
    /// Define a subscriber object with topic name and buffer size of messages
    /// Make sure you have subscribed to the correct topic
      ros::Subscriber subscibeSensor;
      
      publishVelocity = n.advertise<geometry_msgs::Twist>\
                  ("/cmd_vel", 1000);
      /// Subscribe for data from the laser sensor on the scan topic
      subscibeSensor = n.subscribe<sensor_msgs::LaserScan>("/base_scan", 500, sensorCallback);
      
      ROS_INFO_STREAM("Set up complete");
      //ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
      //ros::Subscriber sub_laser= n.subscribe("scan",1000, scanCallback);
    

    ros::Rate loop_rate(2);
   while (ros::ok()) { 
       
       if (checkObstacle()) {
      /// Start turning the robot to avoid obstacles
      velocities.linear.x = 0.0;
      velocities.angular.z = anguarVelocity;
      /// Check if velocities have changed
      checkVelocityChanged();
    } else {
        /// Start moving the robot once obstacle is avoided
        velocities.angular.z = 0.0;
        velocities.linear.x = linearVelocity;
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