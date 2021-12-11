#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

//listener  object automatically subscribes to the transform message topic over ROS and manages all transform data coming in over the wire. 

void transformPoint(const tf::TransformListener& listener){
  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped sensor_point;
  sensor_point.header.frame_id = "base_sensor";

  //we'll just use the most recent transform available for our simple example
  sensor_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  sensor_point.point.x = 1.0;
  sensor_point.point.y = 0.2;
  sensor_point.point.z = 0.0;

  try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("base_link", sensor_point, base_point);

    ROS_INFO("base_sensor: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        sensor_point.point.x, sensor_point.point.y, sensor_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"base_sensor\" to \"base_link\": %s", ex.what());
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  ros::spin();

}