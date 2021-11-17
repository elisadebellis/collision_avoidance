#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

/// Initialize linear velocity with 0.2m/s
const float linearVelocity = 0.2;
/// Initialize angular velocity with 30degrees/s
const float anguarVelocity = 0.52;
/// Initalize safe distance as 1.2m
const float distanceThreshold = 1.2;
/// Initialize publishing rate
const int rate = 2;
/* p_i(x,y) = posa ostacolo
t (x,y) = posa robot

t-p_i = direzione forza risultante
modulo forza risultante: 1/norm(t, p_i) 

\sum_i fi + cmd_vel*/


// 1. Trasformo il laser in un punto 



/*void CmdCallBack(const simple_sim::Cmd& cmd_msg){
    ROS_INFO("command linear velocity: [%f]", cmd_msg.linear_speed);
    ROS_INFO("command angular velocity: [%f]", cmd_msg.angular_speed);

    v = cmd_msg.linear_speed;
    omega = cmd_msg.angular_speed;
}

//Update position
void update_position( simple_sim::Pose& pose_msg, const float& linear_speed, const float& angular_speed){

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


int main (int argc, char **argv ) {

    ros::init(argc,argv,"collision_avoidance");
   
    ros::NodeHandle n;


  //ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
  //ros::Subscriber sub_laser= n.subscribe("scan",1000, scanCallback);

  ros::Rate loop_rate(10);

    

   while (ros::ok()) { 
        
      ros::spinOnce();

      loop_rate.sleep();
    }
    return 0;
}