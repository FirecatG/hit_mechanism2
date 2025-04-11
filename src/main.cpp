#include "cam.hpp"
#include <iostream>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"



int main(int argc, char** argv) {
    Cam cam(55, 130, 30, 135, 65, 70,
         25, 30, 10, 10, 
         moveType::SIN_ACCELERATION, moveType::COS_ACCELERATION);
    
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("joint_state_publisher");
    auto publisher = node->create_publisher<sensor_msgs::msg::JointState>("joint_states",1000);
    rclcpp::Rate rate(30);

    
    auto result = cam.calculate();
    int i=0;
    while (rclcpp::ok()) {
        i%=360;
        auto joint_state = sensor_msgs::msg::JointState();
        joint_state.name = {"joint1", "joint2", "joint3"};
        joint_state.header.stamp = node->now();
        joint_state.header.frame_id = "base_link";
        joint_state.position.resize(3);
        double position1 = -2*i*M_PI/360;
        position1-=1.1; //initial position
        if (position1>M_PI) {
            position1-=2*M_PI;
        }
        if (position1<-1*M_PI) {
            position1+=2*M_PI;
        }
        joint_state.position = {position1,result.s_points[i++]/1000,0};
        publisher->publish(joint_state);
        RCLCPP_INFO(node->get_logger(), "Publishing joint state phi = %d , s = %.2f", i, result.s_points[i]);
        rate.sleep();
    }

    

    return 0;
}