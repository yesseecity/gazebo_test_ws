#include <math.h>
#include <thread>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <gazebo_msgs/ModelState.h>

class DummyPoint {
  private:
    ros::NodeHandle ros_node;
    ros::Publisher mark_pose_puber, cmd_vel_puber;
    ros::Subscriber tag_detections, odom;
    tf::StampedTransform  tf_car_pose;
    tf::StampedTransform  tf_tag_pose_a;
    tf::StampedTransform  tf_tag_pose_b;
    tf::TransformListener tf_car_pose_listenser;
    tf::TransformListener tf_tag_pose_listenser_a;
    tf::TransformListener tf_tag_pose_listenser_b;
    tf::Point dummy_mark_pose;
    ros::Publisher puber_docking_direction, puber_x, puber_y, puber_z;
    tf::Quaternion dock_direction = tf::Quaternion(0,0,0,1);
    // tf::Vector3 x_axis = tf::Vector3(1,0,0);
    tf::Vector3 y_axis = tf::Vector3(0,1,0);
    tf::Vector3 z_axis = tf::Vector3(0,0,1);
    double car_yaw, dock_yaw;
  public:
    DummyPoint(ros::NodeHandle node) {
      ros_node = node;
    };
    void start() {
      tag_detections = ros_node.subscribe("/tag_detections", 60, &DummyPoint::move_direction_mark, this);
      tf_car_pose_listenser.waitForTransform("map", "agv_car/base_link", ros::Time(0), ros::Duration(3.0));
      odom = ros_node.subscribe("agv_car/odom", 1, &DummyPoint::get_car_pose, this);
    }

    void move_direction_mark(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
      if (msg->detections.size() < 2) {
        return;
      }

      try{
        tf_tag_pose_listenser_a.waitForTransform("map", "A", ros::Time(0), ros::Duration(3.0));
        tf_tag_pose_listenser_b.waitForTransform("map", "B", ros::Time(0), ros::Duration(3.0));
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

      // ROS_INFO_STREAM("======================");
      // ROS_INFO("tag a [x: %10.6f, y: %10.6f, z: %10.6f]", tag_pose_a.x(), tag_pose_a.y(), tag_pose_a.z());
      // ROS_INFO("tag b [x: %10.6f, y: %10.6f, z: %10.6f]", tag_pose_b.x(), tag_pose_b.y(), tag_pose_b.z());
      // ROS_INFO("tag c [x: %10.6f, y: %10.6f, z: %10.6f]", dummy_mark_pose.x(), dummy_mark_pose.y(), dummy_mark_pose.z());


      tf::Vector3 a_to_b = tag_pose_b - tag_pose_a;
      a_to_b.normalize();
      a_to_b.setZ(0);
      float theta  = atan2(a_to_b.y(), a_to_b.x()) ;
      dock_yaw =  theta - M_PI_2;
      if (dock_yaw < -1*M_PI) dock_yaw += M_PI*2;

      // ROS_INFO("AB [x: %10.6f, y: %10.6f, z: %10.6f]", a_to_b.x(), a_to_b.y(), a_to_b.z());
      // ROS_INFO("theta {RAD: %10.6f, DEG: %10.6f}", theta, theta*180/M_PI);
      // ROS_INFO("d_yaw: %10.6f", dock_yaw);
      dock_direction.setRPY(0, 0, dock_yaw);
		
      gazebo_msgs::ModelState mark_state;
      mark_state.model_name = "direction";
      mark_state.pose.position.x = dummy_mark_pose.x();
      mark_state.pose.position.y = dummy_mark_pose.y();
      mark_state.pose.position.z = dummy_mark_pose.z()+0.1;
      mark_state.pose.orientation.x = dock_direction.x();
      mark_state.pose.orientation.y = dock_direction.y();
      mark_state.pose.orientation.z = dock_direction.z();
      mark_state.pose.orientation.w = dock_direction.w();
      mark_pose_puber = ros_node.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
      mark_pose_puber.publish(mark_state);


      geometry_msgs::PoseStamped docking_direction_topic;
      docking_direction_topic.header.frame_id = "map";
      docking_direction_topic.pose.position.x = dummy_mark_pose.x();
      docking_direction_topic.pose.position.y = dummy_mark_pose.y();
      docking_direction_topic.pose.position.z = dummy_mark_pose.z();
      docking_direction_topic.pose.orientation.x = dock_direction.x();
      docking_direction_topic.pose.orientation.y = dock_direction.y();
      docking_direction_topic.pose.orientation.z = dock_direction.z();
      docking_direction_topic.pose.orientation.w = dock_direction.w();
      puber_docking_direction = ros_node.advertise<geometry_msgs::PoseStamped>("/docking_direction", 1);
      puber_docking_direction.publish(docking_direction_topic);
      // ROS_INFO("docking_direction_topic pubed");

      rotate_car();

      // axis();
      ros::spinOnce();

      // tag_detections.shutdown();
    };
    
    void get_car_pose(const nav_msgs::Odometry::ConstPtr& msg) {
      tf_car_pose_listenser.lookupTransform("map", "agv_car/base_link", ros::Time(0), tf_car_pose);

      double car_roll, car_pitch;
      tf::Matrix3x3 car_matrix( tf_car_pose.getRotation());
      car_matrix.getRPY(car_roll, car_pitch, car_yaw);
    }

    void rotate_car(){
      tf::Quaternion init_direction = tf::Quaternion(0,0,0,1);
      // tf_car_pose.getOrigin();
      tf::Quaternion car_direction = tf_car_pose.getRotation();
      // * Quaternion.angle 永遠只有正的值
     
     
      float angle_diffance = dock_yaw - car_yaw;

      float abs_angle_diffance = fabs(angle_diffance);
      ROS_INFO_STREAM("-------------");

      ROS_INFO("car_yaw   : %10.6f", car_yaw);
      ROS_INFO("dock_yaw  : %10.6f", dock_yaw);
      ROS_INFO("angle_diffance     : %10.6f", angle_diffance);


      geometry_msgs::Twist cmd_vel;
      float M_PI_8 = 0.392699082;
      if (abs_angle_diffance < 0.01) {
        cmd_vel.angular.z = 0.0;
        cmd_vel.linear.x = 0.0;
      } else if (abs_angle_diffance < M_PI_4) {
        float angular_z = M_PI_8/2;
        // cmd_vel.linear.x = -0.1;
        if (angle_diffance > 0) {
          cmd_vel.angular.z = angular_z;
        } else {
          cmd_vel.angular.z = angular_z * -1;
        }
      } else if (abs_angle_diffance < M_PI_2) {
        // cmd_vel.linear.x = -0.3;
        if (angle_diffance > 0) {
          cmd_vel.angular.z = M_PI_8;
        } else {
          cmd_vel.angular.z = M_PI_8 * -1;
        }
      } else if (abs_angle_diffance >= M_PI_2) {
        // cmd_vel.linear.x = -0.4;
        if (angle_diffance > 0) {
          cmd_vel.angular.z = M_PI_4;
        } else {
          cmd_vel.angular.z = M_PI_4 * -1;
        }
      }
      ROS_INFO("angular.z : %10.6f", cmd_vel.angular.z);
      cmd_vel_puber = ros_node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
      cmd_vel_puber.publish(cmd_vel);
    }

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
