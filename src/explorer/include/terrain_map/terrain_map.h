#ifndef TERRAIN_MAP_H
#define TERRAIN_MAP_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

namespace explorer_ns {
    class TerrainAnalysis {
    public:
        TerrainAnalysis(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
        ~TerrainAnalysis();
        
    private:
        // ros
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
            
        message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_sub_;
        message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicy;
        typedef message_filters::Synchronizer<SyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_point_cloud_odom_;

        ros::Publisher terrain_map_pub_;
        ros::Timer pub_timer_;

        // init
        int no_data_init_;
        double buffer_dist;

        // tf2
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        
        // terrain map
        nav_msgs::OccupancyGrid terrain_map_2d_;

        // position
        geometry_msgs::Pose current_sensor_pose_;
        geometry_msgs::Pose start_pose_;

        // pcl
        pcl::VoxelGrid<pcl::PointXYZI> pcl_voxel_filter_;
        std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>>> grid_cloud_;
        std::vector<std::vector<double>> elevation_grid_;

        // parameters
        int grid_width_num; 
        int grid_height_num;
        double pcl_voxel_size_;
        double terrain_map_width_;
        double terrain_map_height_;
        double map_resolution_;
        double elevation_thre_;
        std::string point_cloud_topic_;
        std::string odom_topic_;
        std::string world_frame_;
        std::string odom_frame_;
        std::string cloud_frame_;

        void pointCloudOdomCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud, 
                            const nav_msgs::Odometry::ConstPtr &odom);
        bool initialize();
        bool readParameters();
        void initGridMap();
        void updateGridMap(const pcl::PointCloud<pcl::PointXYZI> &cloud);
        Eigen::Vector2i getGridIndex(const pcl::PointXYZI &point);

    };
} // namespace explorer_ns
#endif // TERRAIN_MAP_H