#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "collision_avoidance/Pose.h"
#include <complex.h>
#include <string.h>
#include <iostream>

using namespace std;

/* --------VARIABILI GLOBALI---------*/

///Velocità lineare
float linear_speed;
///Velocità angolare
float angular_speed;
//Distanza di threshold a 0.3
const float distanceThreshold = 0.4;
//Variabile che identifica se l'ostacolo è stato rilevato
bool obstacle_detected;
//Per pubblicare le velocità
geometry_msgs::Twist velocities;
//Velocita limite
#define MAX_SPEED 1.0


//Prende in input da terminale il valore della velocità angolare e lineare
void setVelocity() {

  char* check_vel = (char*) malloc(sizeof(char)*20);

  cout << "Enter a speed command (0 < v < 1): \nlinear_speed = " ;
  cin >> check_vel;

  //atof restituisce il corrispondente valore in double 
  if (atof(check_vel) > MAX_SPEED ) {
    linear_speed = MAX_SPEED;
  }
  else {
    linear_speed = atof(check_vel);
  }

  cout << "angular_speed = ";
  cin >> check_vel;

  if (angular_speed > MAX_SPEED ) {
    angular_speed = MAX_SPEED;
  }
  else {
    angular_speed = atof(check_vel);
  }
 
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
  
  float min_range = 0;
  for (const float &range : sensorData->ranges) {
    
    if (range < distanceThreshold && range > min_range) {
     
      min_range = range;

    }
  }

  for (const float &range : sensorData->ranges) {
    
    if (range == min_range) {
      setObstacleDetected(true);
      
      return;
    } 
  }
  setObstacleDetected(false);
}

int main (int argc, char **argv ) {

    ros::init(argc,argv,"collision_avoidance");
   
    ros::NodeHandle n;

    cout << "Setting up the robot configuration for collision avoidance..." << endl;
    
    
    bool setupVelocity = false;
    while (!setupVelocity) {
      
      setVelocity();

      if (linear_speed > 0 && angular_speed > 0) {
        
        cout << "Speed set correctly: " << endl;
        cout << "linear_speed: " << linear_speed << endl;
        cout << "angular_speed: " << angular_speed << endl;
        setupVelocity = true;
      }

      else {
        cout << "\nThe velocity must be a number greater than zero: " << endl;
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
      
      cout << "Set up complete" << endl;
    

      ros::Rate loop_rate(2);
      
      while (ros::ok()) { 
        /// Se l'ostacolo è stato rilevato
        if (checkObstacle()) {
        /// Il robot si ferma e inizia a ruotare
        velocities.linear.x = 0.0;
        velocities.angular.z = angular_speed;
        
        } 
        else {
          ///Si muove se l'ostacolo non è stato evitato
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