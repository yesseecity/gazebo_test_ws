#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

class ApriltagDockingPath {
  public:
    ros::NodeHandle ros_node;
    ros::Publisher  pub_docking_path;
    ros::Subscriber tag_detections;
    tf::StampedTransform  tf_car_pose;
    tf::StampedTransform  tf_tag_pose;
    tf::TransformListener tf_car_pose_listenser;
    tf::TransformListener tf_tag_pose_listenser;
    ApriltagDockingPath(ros::NodeHandle node) {
      ros_node = node;
      tag_detections = node.subscribe("/tag_detections", 60, &ApriltagDockingPath::create_bezier, this);
    };

    void create_bezier(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg){
      if (msg->detections.size() == 0) {
        return;
      }

      try{
        tf_car_pose_listenser.lookupTransform("/map", "/agv_car/base_footprint", ros::Time(0), tf_car_pose);
        tf_tag_pose_listenser.lookupTransform("/map", "A", ros::Time(0),  tf_tag_pose);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
        ros::Duration(1,0).sleep();
      }
      
      nav_msgs::Path docking_path;
      tf::Vector3 z_axis(0,0,1);
      tf::Vector3 bezier_vector0(0.4, 0, 0), bezier_vector1(0.4+0.3, 0, 0), bezier_vector2(-0.3, 0, 0);
      tf::Point bezier_point0, bezier_point1, bezier_point2, bezier_point3;
      tf::Point tag_pose = tf_tag_pose.getOrigin();
      bezier_point3= tf_car_pose.getOrigin();


      double dist;
      dist = bezier_point0.distance(bezier_point3);
      if (dist == 0) {
        return;
      }
      if (dist < 0.15) {
        bezier_vector1.setValue(0.4+0.15, 0, 0);
        bezier_vector2.setValue(-0.15, 0, 0);
      }

      double car_roll, car_pitch, car_yaw;
      double tag_roll, tag_pitch, tag_yaw;
      
      tf::Matrix3x3 car_matrix( tf_car_pose.getRotation());
      car_matrix.getRPY(car_roll, car_pitch, car_yaw);
      // tf::Matrix3x3 tag_matrix( tf_tag_pose.getRotation());
      // tag_matrix.getRPY(tag_roll, tag_pitch, tag_yaw);
      
      std::cout<<"dist: "<<dist<<std::endl;
      printf("position: \t(     x,      y,      z) ");
      printf("rotate: (tag_roll, tag_pitch, tag_yaw) \n");
      printf("point0    \t(%6.3f, %6.3f, %6.3f) ", bezier_point0.x(), bezier_point0.y(), bezier_point0.z());
      printf("        (%8.3f, %8.3f, %8.5f) \n", tag_roll, tag_pitch, tag_yaw-M_PI_2);
      // printf("position: \t(     x,      y,      z) ");
      // printf("rotate: (car_roll, car_pitch, car_yaw) \n");
      // printf("point3    \t(%6.3f, %6.3f, %6.3f) ", bezier_point3.x(), bezier_point3.y(), bezier_point3.z());
      // printf("        (%8.3f, %8.3f, %8.3f) \n", car_roll, car_pitch, car_yaw);

      printf("====================================================\n");

      bezier_point0 =  tag_pose + bezier_vector0.rotate(z_axis, 0); //tag_yaw-M_PI_2
      bezier_point1 =  tag_pose + bezier_vector1.rotate(z_axis, 0); //tag_yaw-M_PI_2
      bezier_vector2 = bezier_vector2.rotate(z_axis, car_yaw);
      bezier_point2 = bezier_point3 + bezier_vector2;

      double xa, ya, xb, yb, xc, yc, bx, by, xm, ym, xn, yn;
      for( double i = 0 ; i < 1 ; i += 0.05 ) {
        xa = getPt( bezier_point0.x() , bezier_point1.x() , i );
        ya = getPt( bezier_point0.y() , bezier_point1.x() , i );
        xb = getPt( bezier_point1.x() , bezier_point2.x() , i );
        yb = getPt( bezier_point1.y() , bezier_point2.y() , i );
        xc = getPt( bezier_point2.x() , bezier_point3.x() , i );
        yc = getPt( bezier_point2.y() , bezier_point3.y() , i );

        xm = getPt( xa , xb , i );
        ym = getPt( ya , yb , i );
        xn = getPt( xb , xc , i );
        yn = getPt( yb , yc , i );

        bx = getPt( xm , xn , i );
        by = getPt( ym , yn , i );

        geometry_msgs::PoseStamped ros_pose;
        ros_pose.pose.position.x = bx;
        ros_pose.pose.position.y = by;
        ros_pose.header.frame_id = "path";
        docking_path.poses.push_back(ros_pose);
      }

      docking_path.header.frame_id = "path";

      std::cout<<"path poses size:"<<docking_path.poses.size()<<std::endl;
      pub_docking_path = ros_node.advertise<nav_msgs::Path>("bezier", 1);
      pub_docking_path.publish(docking_path);
      ros::spinOnce();

      // tag_detections.shutdown();
    };
    double getPt( double n1 , double n2 , double perc ){
      double diff = n2 - n1;
      return n1 + ( diff * perc );
    }    

};
