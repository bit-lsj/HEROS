#ifndef EXPLORER_H
#define EXPLORER_H

#include <mutex>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <ufo/map/key.h>
#include <ufo/map/code.h>
#include <ufo/map/occupancy_map.h>
#include <ufo/map/occupancy_map_color.h>
#include <ufomap_msgs/UFOMapStamped.h>

#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <graph/graph.h>
#include <viewpoint/viewpoint_manager.h>
// #include <subregion/subregion_manager.h>
#include <subregion_global/subregion_global.h>

namespace explorer_ns{
    // std::unordered_set 是 C++ 标准库中的一个容器，它是一个基于哈希表的集合容器，用于存储唯一的元素，即集合中的每个元素都是不同的；
    // 与 std::set 不同，std::unordered_set 不会根据元素的顺序进行排序，而是通过元素的哈希值来快速访问和查找元素；
    // std::unordered_set 中的元素是唯一的，这意味着不允许有重复的元素存在;
    // 如果尝试插入一个已经存在的元素，它不会被添加到集合中。

    // ufomap中使用了莫顿码以快速遍历八叉树，莫顿码通常用于空间索引、空间分割或者表示多维数据的方法，它将多维数据映射到一维空间中，常用于优化空间查询和数据结构；
    // 莫顿码主要用于多维空间数据的编码和压缩，使得多维数据在一维空间中有序分布；而哈希函数则用于生成固定长度的哈希值，以实现快速查找和存储。
    
    // ufo::map::Code::Hash 是ufomap中的哈希函数对象，用于指导 std::unordered_set 如何对 ufo::map::Code 对象进行哈希；
    typedef std::unordered_set<ufo::map::Code, ufo::map::Code::Hash> CodeUnorderSet;

    class Explorer{
    public:
        Explorer(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
        ~Explorer() {};

        void pointCloudOdomCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud, 
                                    const nav_msgs::Odometry::ConstPtr &odom);
        void terrainMapCallback(const nav_msgs::OccupancyGridPtr &terrain_map);
        void ufomapPublishCallback(const ros::TimerEvent &event);
        void exploreCallback(const ros::TimerEvent &event);
        void frontierSearch();
        void frontierDownsampling();
        void findLocalFrontier();
        void updateGlobalFrontier();
        void visualizeFrontier();
        void visualize();
        void getUpdateBound();
        void waypointPlanning(std::vector<NodePtr>excuted_path);
        void frozenCheck();
        void terminate();
        bool inSensorRange(const ufo::map::Point3 &point);
        bool inExplorationArea(const ufo::map::Point3 &point);
        bool isFrontier(const ufo::map::Code &frontier);
        bool initialize();
        bool readParameters();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Subscriber terrain_map_sub_;
        message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_sub_;
        message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicy;
        typedef message_filters::Synchronizer<SyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_point_cloud_odom_;
        

        ros::Publisher map_pub_;
        ros::Publisher ufo_cloud_pub_;
        ros::Publisher frontier_pub_;
        ros::Publisher sample_horizen_pub_;
        ros::Publisher subregion_pub_;
        ros::Publisher tour_pub_;

        // tmp
        ros::Publisher edges_pub_;
        ros::Publisher graph_pub_;
        ros::Publisher sample_pub_;
        ros::Publisher path_pub_;
        ros::Publisher viewpoint_pub_;
        ros::Publisher viewpoint_line_pub_;
        ros::Publisher map_change_pub_;
        ros::Publisher centroid_pub_;
        ros::Publisher local_path_pub_;
        ros::Publisher excuted_path_pub_;
        ros::Publisher waypoint_pub_;
        ros::Publisher sphere_pub_;
        ros::Publisher runtime_pub_;

        // tf2
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        // time
        ros::Timer pub_timer_;
        ros::Timer explore_timer_;
        ros::Duration transform_timeout_;
        // frozen check
        bool is_frozen_;
        ros::Time last_check_time_;
        nav_msgs::OdometryConstPtr last_odom_;
        // map
        ufo::map::OccupancyMapColor map_;
        nav_msgs::OccupancyGridPtr terrain_map_;
        Box map_update_bound_;
        // position
        Eigen::Vector3d sensor_pose_;
        // pcl
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
        pcl::VoxelGrid<pcl::PointXYZ> frontier_filter_;
        // use code to index frontier points
        CodeUnorderSet changed_cell_codes_;  
        CodeUnorderSet history_frontier_cells_;
        CodeUnorderSet global_frontier_codes_;
        CodeUnorderSet local_frontier_codes_;
        // frontier tree
        PointRtree frontier_tree_;
        // graph
        Graph graph_;
        // subregion
        SubregionManager subregion_manager_;
        // view point
        ViewpointManager viewpoint_manager_;
        
        // param
        int depth_levels_;
        int execute_frequency_;  
        int insert_depth_;
        int early_stopping_;
        int clearing_depth_;
        int frontier_depth_;
        double update_max_range_;
        double vehicle_width_;
        double vehicle_length_;
        double vehicle_height;
        double sensor_height_;
        double search_radius_min_;
        double sensor_vertical_fov_;
        double sensor_orientation_;
        double bound_x_min_;
        double bound_x_max_;
        double bound_y_min_;
        double bound_y_max_;
        double check_frozen_intervel = 2.0;
        double check_frozen_distance_thres = 0.5;
        float scan_voxel_size_;
        float frontier_voxel_size_;
        float resolution_;
        bool simple_ray_casting_;
        std::string odom_topic_;
        std::string point_cloud_topic_;
        std::string robot_frame_;
        std::string odom_frame_;
    };
} // namespace explorer_ns
#endif // EXPLORER_H

