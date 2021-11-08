#include "ros/ros.h"
#include "std_msgs/String.h"
#include "simple_sim/Cmd.h"
#include "simple_sim/Pose.h"

#include <sstream>

float T =1;
float v = 0;
float omega = 0;

void CmdCallBack(const simple_sim::Cmd& cmd_msg){
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


}

int main (int argc, char **argv ) {

    ros::init(argc,argv, "simple_sim_node");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<simple_sim::Pose>("SimpleSimPoseTopic",1000);

    ros::Rate loop_rate(T);

    //sottoscrizione ad un messaggio di tipo Cmd

    ros::Subscriber sub = n.subscribe("Cmd_topic", 1000, CmdCallBack);
    ros::ServiceServer service = n.advertiseService("SetTimeInterval", set_interval);

    //valori di pose settati a zero
    simple_sim::Pose pos_msg;

    pos_msg.x = 0;
    pos_msg.y = 0;
    pos_msg.theta = 0;

    //contatore di messaggi inviati
    int count=0;
    while (ros::ok()) {

        update_position(pos_msg, v, omega);
        

        ROS_INFO("Il valore di x è %f",pos_msg.x);
        ROS_INFO("Il valore di y è %f",pos_msg.y);
        ROS_INFO("Il valore di theta è %f",pos_msg.theta);
        ROS_INFO("Il valore di T è [%f]", T);

        chatter_pub.publish(pos_msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;

    }
    return 0;
}