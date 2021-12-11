#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

//publishes base_sensor--->base_link transform

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  //0.25 forward w.r.t center of base
  //0.35 upward w.r.t base
  
  while(n.ok()){
    broadcaster.sendTransform( 
        tf::StampedTransform(
            tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.25, 0.0, 0.35)),
            ros::Time::now(),"base_link", "base_sensor"));
    r.sleep();
  }
}