#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <gazebo_msgs/ModelState.h>

class DummyPoint {
  private:
    ros::NodeHandle ros_node;
    ros::Publisher mark_pose_puber;
    ros::Subscriber tag_detections;
    tf::StampedTransform  tf_car_pose;
    tf::StampedTransform  tf_tag_pose_a;
    tf::StampedTransform  tf_tag_pose_b;
    tf::TransformListener tf_car_pose_listenser;
    tf::TransformListener tf_tag_pose_listenser_a;
    tf::TransformListener tf_tag_pose_listenser_b;
    tf::Point dummy_mark_pose;
    ros::Publisher puber_docking_direction, puber_x, puber_y, puber_z;
  public:
    DummyPoint(ros::NodeHandle node) {
      ros_node = node;
      tf_car_pose_listenser.waitForTransform("map", "agv_car/base_link", ros::Time(0), ros::Duration(3.0));
      tag_detections = node.subscribe("/tag_detections", 60, &DummyPoint::move_direction_mark, this);
    };

    void move_direction_mark(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
      if (msg->detections.size() < 2) {
        return;
      }

      try{
        tf_tag_pose_listenser_a.waitForTransform("map", "A", ros::Time(0), ros::Duration(3.0));
        tf_tag_pose_listenser_b.waitForTransform("map", "B", ros::Time(0), ros::Duration(3.0));
        tf_car_pose_listenser.lookupTransform("map", "agv_car/base_link", ros::Time(0), tf_car_pose);
        tf_tag_pose_listenser_a.lookupTransform("map", "A", ros::Time(0),  tf_tag_pose_a);
        tf_tag_pose_listenser_b.lookupTransform("map", "B", ros::Time(0),  tf_tag_pose_b);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
        ros::Duration(1,0).sleep();
      }
      
      tf::Point tag_pose_a = tf_tag_pose_a.getOrigin();
      tf::Point tag_pose_b = tf_tag_pose_b.getOrigin();
      dummy_mark_pose = (tag_pose_a +tag_pose_b)/2;

      // ROS_INFO_STREAM("-------------");
      // ROS_INFO("tag a [x: %10.6f, y: %10.6f, z: %10.6f]", tag_pose_a.x(), tag_pose_a.y(), tag_pose_a.z());
      // ROS_INFO("tag b [x: %10.6f, y: %10.6f, z: %10.6f]", tag_pose_b.x(), tag_pose_b.y(), tag_pose_b.z());
      // ROS_INFO("tag c [x: %10.6f, y: %10.6f, z: %10.6f]", dummy_mark_pose.x(), dummy_mark_pose.y(), dummy_mark_pose.z());


      // tf::Vector3 x_axis(1,0,0);
      tf::Vector3 y_axis(0,1,0);
      tf::Vector3 z_axis(0,0,1);
      tf::Vector3 a_to_b = tag_pose_b - tag_pose_a;
      a_to_b.setZ(0);
      // ROS_INFO("a_to_b [x: %10.6f, y: %10.6f, z: %10.6f]", a_to_b.x(), a_to_b.y(), a_to_b.z());
      float angle = a_to_b.angle(y_axis);
      // ROS_INFO("angle:  %7.4f", angle);
      tf::Quaternion q(0,0,0,1);
      q.setRotation(z_axis, angle*-1);


      gazebo_msgs::ModelState mark_state;
      mark_state.model_name = "direction";
      mark_state.pose.position.x = dummy_mark_pose.x();
      mark_state.pose.position.y = dummy_mark_pose.y();
      mark_state.pose.position.z = dummy_mark_pose.z()+0.1;
      mark_state.pose.orientation.x = q.x();
      mark_state.pose.orientation.y = q.y();
      mark_state.pose.orientation.z = q.z();
      mark_state.pose.orientation.w = q.w();
      mark_pose_puber = ros_node.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
      mark_pose_puber.publish(mark_state);


      geometry_msgs::PoseStamped docking_direction_topic;
      docking_direction_topic.header.frame_id = "map";
      docking_direction_topic.pose.position.x = dummy_mark_pose.x();
      docking_direction_topic.pose.position.y = dummy_mark_pose.y();
      docking_direction_topic.pose.position.z = dummy_mark_pose.z();
      docking_direction_topic.pose.orientation.x = q.x();
      docking_direction_topic.pose.orientation.y = q.y();
      docking_direction_topic.pose.orientation.z = q.z();
      docking_direction_topic.pose.orientation.w = q.w();
      puber_docking_direction = ros_node.advertise<geometry_msgs::PoseStamped>("/docking_direction", 1);
      puber_docking_direction.publish(docking_direction_topic);
      // ROS_INFO("docking_direction_topic pubed");


      // axis();
      ros::spinOnce();

      // tag_detections.shutdown();
    };

    void axis() {
      geometry_msgs::PoseStamped x_topic;
      x_topic.header.frame_id = "map";
      x_topic.pose.position.x = dummy_mark_pose.x();
      x_topic.pose.position.y = dummy_mark_pose.y();
      x_topic.pose.position.z = dummy_mark_pose.z();
      x_topic.pose.orientation.x = 0;
      x_topic.pose.orientation.y = 0;
      x_topic.pose.orientation.z = 0;
      x_topic.pose.orientation.w = 1;
      puber_x = ros_node.advertise<geometry_msgs::PoseStamped>("/pose/x", 1);
      puber_x.publish(x_topic);
      // ROS_INFO("pub x");

      geometry_msgs::PoseStamped y_topic;
      y_topic.header.frame_id = "map";
      y_topic.pose.position.x = dummy_mark_pose.x();
      y_topic.pose.position.y = dummy_mark_pose.y();
      y_topic.pose.position.z = dummy_mark_pose.z();
      y_topic.pose.orientation.x = 0;
      y_topic.pose.orientation.y = 0;
      y_topic.pose.orientation.z = 0.707;
      y_topic.pose.orientation.w = 0.707;
      puber_y = ros_node.advertise<geometry_msgs::PoseStamped>("/pose/y", 1);
      puber_y.publish(y_topic);
      // ROS_INFO("pub y");

      geometry_msgs::PoseStamped z_topic;
      z_topic.header.frame_id = "map";
      z_topic.pose.position.x = dummy_mark_pose.x();
      z_topic.pose.position.y = dummy_mark_pose.y();
      z_topic.pose.position.z = dummy_mark_pose.z();
      z_topic.pose.orientation.x = 0;
      z_topic.pose.orientation.y = -0.707;
      z_topic.pose.orientation.z = 0;
      z_topic.pose.orientation.w = 0.707;
      puber_z = ros_node.advertise<geometry_msgs::PoseStamped>("/pose/z", 1);
      puber_z.publish(z_topic);
      // ROS_INFO("pub z");
      // ROS_INFO("x axis [x: %10.6f, y: %10.6f, z: %10.6f, w: %10.6f]", x_axis.x(), x_axis.y(), x_axis.z(), x_axis.w());
      // ROS_INFO("y axis [x: %10.6f, y: %10.6f, z: %10.6f, w: %10.6f]", y_axis.x(), y_axis.y(), y_axis.z(), y_axis.w());
      // ROS_INFO("z axis [x: %10.6f, y: %10.6f, z: %10.6f, w: %10.6f]", z_axis.x(), z_axis.y(), z_axis.z(), z_axis.w());

    };
};
