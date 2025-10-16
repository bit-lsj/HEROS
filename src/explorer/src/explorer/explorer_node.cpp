#include <explorer/explorer.h>

int main(int argc, char *argv[])
{
  ros::init(argc,argv,"explorer_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
  explorer_ns::Explorer explorer(nh, nh_private);

  ros::spin(); 
  return 0;
}