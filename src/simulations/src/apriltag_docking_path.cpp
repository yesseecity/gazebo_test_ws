#include <apriltag_docking_path.h>
#include <dummy_point.h>

int main(int argc, char** argv) {
  
  ros::init(argc, argv, "apriltag_docking");
  ros::NodeHandle node;
  float offset=0.4;
  // ApriltagDockingPath AdockingPath(node, offset);
  DummyPoint dummyPoint(node);
  ros::spin();

  return 0;
};

//
//  roslaunch simulations start.launch 
//  rosrun simulations apriltag_docking_path
//  rviz -d ./src/simulations/rviz/config.rviz 
//
//
//  rosrun tf tf_echo map /agv_car/base_footprint
//  rosrun tf tf_echo /agv_car/base_footprint A
//  rosrun  tf tf_echo map A  
//  rosrun tf view_frames 