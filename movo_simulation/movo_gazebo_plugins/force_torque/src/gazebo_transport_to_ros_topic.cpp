#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

#include <iostream>

ros::Publisher pub_0;
ros::Publisher pub_1;
double ros_rate=20;

void torquesCb(const ConstWrenchStampedPtr &_msg)
{
  // std::cout << "Received msg: " << std::endl;
  // std::cout << _msg->DebugString() << std::endl;
  geometry_msgs::WrenchStamped msgWrenchedStamped;
  // try WrenchStamped msgWrenchedStamped;
  msgWrenchedStamped.header.stamp = ros::Time::now();
  msgWrenchedStamped.wrench.force.x = _msg->wrench().force().x();
  msgWrenchedStamped.wrench.force.y = _msg->wrench().force().y();
  msgWrenchedStamped.wrench.force.z = _msg->wrench().force().z();
  msgWrenchedStamped.wrench.torque.x = _msg->wrench().torque().x();
  msgWrenchedStamped.wrench.torque.y = _msg->wrench().torque().y();
  msgWrenchedStamped.wrench.torque.z = _msg->wrench().torque().z();
  pub_0.publish(msgWrenchedStamped);
 
}

void torquesCb_1(const ConstWrenchStampedPtr &_msg)
{
  // std::cout << "Received msg: " << std::endl;
  // std::cout << _msg->DebugString() << std::endl;
  geometry_msgs::WrenchStamped msgWrenchedStamped;
  // try WrenchStamped msgWrenchedStamped;
  msgWrenchedStamped.header.stamp = ros::Time::now();
  msgWrenchedStamped.wrench.force.x = _msg->wrench().force().x();
  msgWrenchedStamped.wrench.force.y = _msg->wrench().force().y();
  msgWrenchedStamped.wrench.force.z = _msg->wrench().force().z();
  msgWrenchedStamped.wrench.torque.x = _msg->wrench().torque().x();
  msgWrenchedStamped.wrench.torque.y = _msg->wrench().torque().y();
  msgWrenchedStamped.wrench.torque.z = _msg->wrench().torque().z();
  pub_1.publish(msgWrenchedStamped);
 
}

int main(int argc, char **argv)
{
    
  ROS_INFO("Starting gazebo_transport_to_ros_topic node");

  // Load Gazebo
  gazebo::client::setup(argc, argv);
  ROS_INFO("Starting gazebo");

  // Load ROS
  ros::init(argc, argv, "gazebo_transport_to_ros_topic");
  
  // Create Gazebo node and init
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Create ROS node and init
  ros::NodeHandle n;

  // Get ROS params
  std::string gazebo_transport_topic_to_sub_0= "/gazebo/default/movo/right_wrist_3_joint/joint_6/wrench";
  std::string ros_topic_to_pub_0="/sim/right_arm/cartesianforce";

  std::string gazebo_transport_topic_to_sub_1= "/gazebo/default/movo/left_wrist_3_joint/joint_6/wrench";
  std::string ros_topic_to_pub_1="/sim/left_arm/cartesianforce";
  
  //double ros_rate;
  //ros::param::get("~gazebo_transport_topic_to_sub", gazebo_transport_topic_to_sub);
  //ros::param::get("~ros_rate", ros_rate);

  pub_0 = n.advertise<geometry_msgs::WrenchStamped>(ros_topic_to_pub_0, 100);
  pub_1 = n.advertise<geometry_msgs::WrenchStamped>(ros_topic_to_pub_1, 100);
  
  ROS_INFO("Starting Publisher");
  

  // Listen to Gazebo force_torque sensor topic
  gazebo::transport::SubscriberPtr sub_0 = node->Subscribe(gazebo_transport_topic_to_sub_0 , torquesCb);
  gazebo::transport::SubscriberPtr sub_1 = node->Subscribe(gazebo_transport_topic_to_sub_1 , torquesCb_1);

  ros::Rate loop_rate(ros_rate); 
  ROS_INFO("Starting Subscriber ");
  
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  gazebo::shutdown();
  return 0;
}
