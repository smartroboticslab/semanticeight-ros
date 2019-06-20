#include "supereight_ros/supereight_ros.hpp"

int main(int argc, char **argv) {
  using namespace se;
  // initialize ROS nod
  ros::init(argc, argv, "supereight_ros_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

//#ifdef MAP_OM
  SupereightNode<OFusion> node(nh, nh_private);
  ROS_INFO_STREAM("Occupancy map");
//#elif MAP_SDF
//  SupereightNode<SDF> node(nh, nh_private);
//  ROS_INFO_STREAM("SDF map");
//#endif

  std::cout << "FINISHED" << std::endl;

  ros::spin();

//  std::string filename = "/home/anna/Data/timings.txt";
  std::ofstream timing;
  timing.open("/home/anna/Data/timing.txt", std::ofstream::app);
  Stats.print_all_data(timing, true);
  timing.close();
  // delete pipeline;
  return 0;
}
