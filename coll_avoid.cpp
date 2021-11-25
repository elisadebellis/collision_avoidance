#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "collision_avoidance/Pose.h"
#include <complex.h>

using namespace std;

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


//Prende in input da terminale il valore della velocità angolare e lineare
void setVelocity() {
  std::cout << "Inserisci un comando di velocità: \n linear_speed = " ;
  std::cin >> linear_speed;
  std::cout << "angular_speed = ";
  std::cin >> angular_speed;
}


//Se l'ostacolo è stato rilevato stampa un messaggio di avviso 
bool checkObstacle() {
  
  if (obstacle_detected) {
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

int main (int argc, char **argv ) {

    ros::init(argc,argv,"collision_avoidance");
   
    ros::NodeHandle n;

    ROS_INFO_STREAM("Setting up the robot configuration for collision avoidance...");
    
    
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
  
      /// Inizializzo obstacle_detected al valore false
      obstacle_detected = false;
      
      /// Definisco il Publisher
      ros::Publisher publishVelocity;
    
      ///Definisco il subsciber
      ros::Subscriber subscibeSensor;
      
      /// Pubblish - velocità del robot sul topic /cmd_vel
      publishVelocity = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
      /// Subscribe - dati del laser dal topic /base_scan
      subscibeSensor = n.subscribe<sensor_msgs::LaserScan>("/base_scan", 500, sensorCallback);
      
      ROS_INFO_STREAM("Set up complete");
    

      ros::Rate loop_rate(2);
      
      while (ros::ok()) { 
        /// Se l'ostacolo è stato rilevato
        if (checkObstacle()) {
        /// Il robot di ferma e inizia a ruotare
        velocities.linear.x = 0.0;
        velocities.angular.z = angular_speed;
        
        } 
        else {
          ///Si muove se l'ostacolo è stato evitato
          velocities.angular.z = 0.0;
          velocities.linear.x = linear_speed;
          
        }
    

      /// Publico le velocità
      publishVelocity.publish(velocities);
      
      ros::spinOnce();

      loop_rate.sleep();
    }
    return 0;
}