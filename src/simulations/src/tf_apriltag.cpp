#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <ros/console.h>
#include <apriltag_ros/AprilTagDetectionArray.h>


void apriltagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
  if (msg->detections.size() > 0) {
    int detection_size = msg->detections.size();
    printf("size of detections: %i \n", detection_size);
    // std::cout<<  msg->detections.size() <<std::endl;

    float px, py, pz;
    px = msg->detections[0].pose.pose.pose.position.x;
    py = msg->detections[0].pose.pose.pose.position.y;
    pz = msg->detections[0].pose.pose.pose.position.z;
    // std::cout<< "x: "<< px <<"\t y:  " << py <<"\t z:  " << pz <<std::endl;
    printf("position: \t(     x,      y,      z) \n");
    printf("          \t(%6.3f, %6.3f, %6.3f) \n", px, py, pz);
    float ox, oy, oz, ow;
    ox = msg->detections[0].pose.pose.pose.orientation.x;
    oy = msg->detections[0].pose.pose.pose.orientation.y;
    oz = msg->detections[0].pose.pose.pose.orientation.z;
    ow = msg->detections[0].pose.pose.pose.orientation.w;
    printf("orientation:\t(    ox,     oy,     oz,     ow) \n");
    printf("            \t(%6.3f, %6.3f, %6.3f, %6.3f) \n\n", ox, oy, oz, ow);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    // transform.setOrigin( tf::Vector3(-0.33, 0, 0.13) );
    transform.setOrigin( tf::Vector3( py-0.33, px+0, pz+0.13) );

    // -0.33 0 0.13 0 0 3.1416
    tf::Quaternion q;
    q.setRPY(0, 0, 3.1416);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/agv_car/base_footprint", "A"));
  }

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "tf_apriltag");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/tag_detections", 10, &apriltagCallback);
  ros::spin();
  return 0;
};