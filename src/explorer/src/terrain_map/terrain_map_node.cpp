#include <terrain_map/terrain_map.h>

int main(int argc, char *argv[])
{
  ros::init(argc,argv,"terrain_map_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
  explorer_ns::TerrainAnalysis terrain_map(nh, nh_private);

  ros::spin(); 
  return 0;
}