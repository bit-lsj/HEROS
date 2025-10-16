#include <terrain_map/terrain_map.h>

namespace explorer_ns {
    TerrainAnalysis::TerrainAnalysis(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
        : nh_(nh), nh_private_(nh_private), tf_listener_(tf_buffer_) {
        if (initialize()) {
            ROS_INFO("Terrain analysis initialized");
        }
        else {
            ROS_ERROR("Failed to initialize Terrain analysis");
        }
    }

    TerrainAnalysis::~TerrainAnalysis() {}

    bool TerrainAnalysis::readParameters() {
        if (!nh_private_.getParam("point_cloud_topic", point_cloud_topic_)) {
            ROS_WARN("Failed to read parameter point_cloud_topic.");
            return false;
        }

        if (!nh_private_.getParam("odom_topic", odom_topic_)) {
            ROS_WARN("Failed to read parameter odom_topic.");
            return false;
        }

        if (!nh_private_.getParam("terrain_map_width", terrain_map_width_)) {
            ROS_WARN("Failed to read parameter terrain_map_width.");
            return false;
        }

        if (!nh_private_.getParam("terrain_map_height", terrain_map_height_)) {
            ROS_WARN("Failed to read parameter terrain_map_height.");
            return false;
        }

        if (!nh_private_.getParam("map_resolution", map_resolution_)) {
            ROS_WARN("Failed to read parameter map_resolution.");
            return false;
        }

        if (!nh_private_.getParam("elevation_thre", elevation_thre_)) {
            ROS_WARN("Failed to read parameter elevation_thre.");
            return false;
        }


        if (!nh_private_.getParam("world_frame", world_frame_)) {
            ROS_WARN("Failed to read parameter base_frame.");
            return false;
        }

        return true;
    }

    bool TerrainAnalysis::initialize() {
        if (!readParameters()) {
            return false;
        }

        no_data_init_ = 0;
        buffer_dist = 4.0;
        pcl_voxel_size_ = 0.05;
        
        // subscriber
        point_cloud_sub_.subscribe(nh_, point_cloud_topic_, 1);
        odom_sub_.subscribe(nh_, odom_topic_, 1);
        sync_point_cloud_odom_.reset(new Sync(SyncPolicy(10), point_cloud_sub_, odom_sub_));
        sync_point_cloud_odom_->registerCallback(boost::bind(&TerrainAnalysis::pointCloudOdomCallback, this, _1, _2));

        // publisher
        terrain_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("terrain_map_2d", 1);

        // pcl 
        pcl_voxel_filter_.setLeafSize(pcl_voxel_size_, pcl_voxel_size_, pcl_voxel_size_);

        // terrain map
        grid_width_num = int(terrain_map_width_ / map_resolution_); 
        grid_height_num = int(terrain_map_height_ / map_resolution_);
        
        grid_cloud_ = std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>>>(grid_width_num, std::vector<pcl::PointCloud<pcl::PointXYZI>>(grid_height_num));
        elevation_grid_ = std::vector<std::vector<double>>(grid_width_num, std::vector<double>(grid_height_num, 0));
        
        initGridMap();

        return true;
    }

    void TerrainAnalysis::pointCloudOdomCallback(const sensor_msgs::PointCloud2::ConstPtr &terrain_cloud, 
                                                const nav_msgs::Odometry::ConstPtr &odom) {
        current_sensor_pose_ = odom->pose.pose;

        if (no_data_init_ == 0) {
            start_pose_ = current_sensor_pose_;
            no_data_init_ = 1;
        }

        if (no_data_init_ == 1) {
            double distance = sqrt(pow(current_sensor_pose_.position.x - start_pose_.position.x, 2) + 
                                   pow(current_sensor_pose_.position.y - start_pose_.position.y, 2));
            if (distance >= buffer_dist) {
                no_data_init_ = 2;
            }
        }
        // 更新地形图的原点
        terrain_map_2d_.info.origin.position.x = current_sensor_pose_.position.x - terrain_map_width_ / 2;
        terrain_map_2d_.info.origin.position.y = current_sensor_pose_.position.y - terrain_map_height_ / 2;

        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::fromROSMsg(*terrain_cloud, *pcl_cloud);       

        pcl_voxel_filter_.setInputCloud(pcl_cloud);
        pcl::PointCloud<pcl::PointXYZI> pcl_cloud_ds;
        pcl_voxel_filter_.filter(pcl_cloud_ds);

        pcl::PointCloud<pcl::PointXYZI> local_cloud;
        for (auto const point:pcl_cloud_ds.points) {
            // 取在terrain map范围内的点云
            if (fabs(point.x - current_sensor_pose_.position.x) <= terrain_map_width_  / 2  && 
                fabs(point.y - current_sensor_pose_.position.y) <= terrain_map_height_ / 2 ) {
                // 取z轴小于1m高的点云
                if (point.z < 1.0) {
                    local_cloud.push_back(point);
                }
            }
        }

        updateGridMap(local_cloud);
    }

    void TerrainAnalysis::initGridMap() {
        // 初始化grid map
        terrain_map_2d_.header.frame_id = world_frame_;
        terrain_map_2d_.header.stamp = ros::Time::now();

        terrain_map_2d_.info.resolution = map_resolution_;
        terrain_map_2d_.info.width = grid_width_num;
        terrain_map_2d_.info.height = grid_height_num;

        terrain_map_2d_.info.origin.position.x = -terrain_map_width_ / 2;
        terrain_map_2d_.info.origin.position.y = -terrain_map_height_ / 2;

        terrain_map_2d_.data.resize(terrain_map_2d_.info.width * terrain_map_2d_.info.height, -1);
    }
    
    void TerrainAnalysis::updateGridMap(const pcl::PointCloud<pcl::PointXYZI> &cloud) {
        auto start_time = ros::WallTime::now();

        std::vector<std::vector<double>> tmp_elevation_grid;
        tmp_elevation_grid = std::vector<std::vector<double>>(grid_width_num, std::vector<double>(grid_height_num, 0.0));

        // 重置栅格
        for (int i = 0; i < grid_width_num; i++) {
            for (int j = 0; j < grid_height_num; j++) {
                grid_cloud_[i][j].clear();
                elevation_grid_[i][j] = 0.0;
            }
        }

        if (no_data_init_ == 1) {
            // 在初始buffer区域内设置栅格
            int x_num, y_num;
            x_num = y_num = buffer_dist / map_resolution_ / 2;
            for (int i = -x_num; i <= x_num; i++) {
                for (int j = -y_num; j <= y_num; j++) {
                    pcl::PointXYZI point;
                    point.x = start_pose_.position.x + i * map_resolution_;
                    point.y = start_pose_.position.y + j * map_resolution_;
                    Eigen::Vector2i idx = getGridIndex(point);
                    // 对于buffer区域内的栅格高度做补偿
                    elevation_grid_[idx[0]][idx[1]] = 0.1;
                }
            }
        }

        // 将三维点云压缩在二维栅格中，每个二维栅格内可能存有不止一个点
        for (auto const point:cloud.points) {
            Eigen::Vector2i idx = getGridIndex(point);
            grid_cloud_[idx[0]][idx[1]].push_back(point);
        }

        // 对于每个栅格，找到栅格内点云高度最大的点和最小的点，作差，存入强度栅格中
        for (int i = 0; i < grid_width_num; i++) {
            for (int j = 0; j < grid_height_num; j++) {
               if (grid_cloud_[i][j].size() > 0) {
                    double heigtest_point = 0.0;
                    double lowest_point = 1e6;
                   for (auto const point: grid_cloud_[i][j].points) {
                       if (point.z > heigtest_point) {
                            heigtest_point = point.z;
                        }
                        if (point.z < lowest_point) {
                            lowest_point = point.z;
                        }
                    }
                    elevation_grid_[i][j] = heigtest_point - lowest_point;
                }
               if (grid_cloud_[i][j].size() == 0) {
                    // 搜索该栅格索引附近的八邻域，取平均值
                    double tmp_total = 0.0;
                    int tmp_num = 0;
                    for (int m = -1; m <= 1; m++) {
                        for (int n = -1; n <= 1; n++) {
                            if (m == 0 && n == 0) {
                                continue;
                            }

                            if (i + m >= 0 && i + m < grid_width_num && j + n >= 0 && j + n < grid_height_num) {
                                if (elevation_grid_[i + m][j + n] > 0) {
                                tmp_total += elevation_grid_[i + m][j + n];
                                tmp_num++;
                                }
                            }
                        }
                    }
                    if (tmp_num > 0) {
                        tmp_elevation_grid[i][j] = (tmp_total / tmp_num);
                    }
                }
            }
        }

        // 更新强度栅格
        for (int i = 0; i < grid_width_num; i++) {
            for (int j = 0; j < grid_height_num; j++) {
                if (tmp_elevation_grid[i][j] > 0) {
                    elevation_grid_[i][j] = tmp_elevation_grid[i][j];
                }
            }
        }

        // 将强度栅格转换为地形图
        for (int i = 0; i < grid_width_num; i++) {
            for (int j = 0; j < grid_height_num; j++) {
                if (elevation_grid_[i][j] >= elevation_thre_) {
                    terrain_map_2d_.data[i + grid_width_num * j] = 100.0;
                }
                else if (elevation_grid_[i][j] > 0.0 && elevation_grid_[i][j] < elevation_thre_) {
                    terrain_map_2d_.data[i + grid_width_num * j] = 0.0;
                }
                else{
                    terrain_map_2d_.data[i + grid_width_num * j] = -1;
                }
            }
        }

        terrain_map_pub_.publish(terrain_map_2d_);

        auto end_time = (ros::WallTime::now() - start_time).toSec();
        // ROS_INFO("Update terrain map cost: %f ms", end_time * 1000);
    }

    Eigen::Vector2i TerrainAnalysis::getGridIndex(const pcl::PointXYZI &point) {
        Eigen::Vector2i idx;
        idx[0] = fmin(fmax(int((point.x - terrain_map_2d_.info.origin.position.x) / map_resolution_), 0), grid_width_num - 1);
        idx[1] = fmin(fmax(int((point.y - terrain_map_2d_.info.origin.position.y) / map_resolution_), 0), grid_height_num - 1);
        return idx;
    }
}