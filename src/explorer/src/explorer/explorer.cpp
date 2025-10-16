#include <explorer/explorer.h>


namespace explorer_ns {
    using namespace std;
    
    Explorer::Explorer(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) :
        nh_(nh), nh_private_(nh_private), tf_listener_(tf_buffer_), 
        map_(
            nh_private_.param("resolution", 0.3), 
            nh_private_.param("depth_levels", 16), true, 
            nh_private_.param("occupied_thres", 0.5), 
            nh_private_.param("free_thres", 0.5)),
        graph_(
            nh_private_.param("max_neighbor_num", 3), 
            nh_private_.param("max_neighbor_distance", 1.5),
            nh_private_.param("min_neighbor_distance", 0.75), 
            nh_private_.param("max_edge_distance", 2.0),
            nh_private_.param("sample_density", 1.0), 
            nh_private_.param("sensor_height_", 0.75),
            nh_private_.param("vehicle_width", 0.6), 
            nh_private_.param("vehicle_length", 0.6),
            nh_private_.param("inflate_radius", 0.2)),
        subregion_manager_(
            nh_private_.param("bound_x_min", -10.0),
            nh_private_.param("bound_x_max", 10.0),
            nh_private_.param("bound_y_min", -10.0),
            nh_private_.param("bound_y_max", 10.0),
            nh_private_.param("subregion_max_width", 6.0),
            nh_private_.param("subregion_max_length", 6.0),
            nh_private_.param("subregion_min_width", 6.0),
            nh_private_.param("subregion_min_length", 6.0),
            nh_private_.param("known_voxel_thre", 0.2)),
        viewpoint_manager_(
            nh_private_.param("visible_radius", 8.0),
            nh_private_.param("filter_thres", 4))
    {
        if (initialize()) {
            ROS_INFO("Explorer initialized");
        }
        else {
            ROS_ERROR("Failed to initialize Explorer");
        }
    }

    bool Explorer::readParameters() {
        if (!nh_private_.getParam("odom_topic", odom_topic_)) {
            ROS_WARN("Failed to read parameter odom_topic.");
            return false;
        }
        if (!nh_private_.getParam("point_cloud_topic", point_cloud_topic_)) {
            ROS_WARN("Failed to read parameter point_cloud_topic.");
            return false;
        }
        if (!nh_private_.getParam("vehicle_width", vehicle_width_)) {
            ROS_WARN("Failed to read parameter vehicle_width.");
            return false;
        }
        if (!nh_private_.getParam("vehicle_length", vehicle_length_)) {
            ROS_WARN("Failed to read parameter vehicle_length.");
            return false;
        }
        if (!nh_private_.getParam("sensor_height", sensor_height_)) {
            ROS_WARN("Failed to read parameter sensor_height.");
            return false;
        }
        if (!nh_private_.getParam("search_radius_min", search_radius_min_)) {
            ROS_WARN("Failed to read parameter search_radius_min.");
            return false;
        }
        if (!nh_private_.getParam("sensor_vertical_fov", sensor_vertical_fov_)) {
            ROS_WARN("Failed to read parameter sensor_vertical_fov.");
            return false;
        }
        if (!nh_private_.getParam("bound_x_min", bound_x_min_)) {
            ROS_WARN("Failed to read parameter bound_x_min.");
            return false;
        }
        if (!nh_private_.getParam("bound_x_max", bound_x_max_)) {
            ROS_WARN("Failed to read parameter bound_x_max.");
            return false;
        }
        if (!nh_private_.getParam("bound_y_min", bound_y_min_)) {
            ROS_WARN("Failed to read parameter bound_y_min.");
            return false;
        }
        if (!nh_private_.getParam("bound_y_max", bound_y_max_)) {
            ROS_WARN("Failed to read parameter bound_y_max.");
            return false;
        }
        if (!nh_private_.getParam("map_update_max_range", update_max_range_)) {
            ROS_WARN("Failed to read parameter map_update_max_range.");
            return false;
        }

        if (!nh_private_.getParam("frontier_voxel_size", frontier_voxel_size_)) {
            ROS_WARN("Failed to read parameter frontier_voxel_size.");
            return false;
        }

        return true;
    }

    bool Explorer::initialize() {
        if (!readParameters()) {
            return false;
        }

        // 相关参数
        scan_voxel_size_ = 0.1;
        simple_ray_casting_ = false;
        insert_depth_   = 0;
        early_stopping_ = 0;
        clearing_depth_ = 0;
        frontier_depth_ = 0;

        // subscriber 
        terrain_map_sub_ = nh_.subscribe("terrain_map_2d", 1, &Explorer::terrainMapCallback, this);
        odom_sub_.subscribe(nh_, odom_topic_, 1);
        point_cloud_sub_.subscribe(nh_, point_cloud_topic_, 1);
        sync_point_cloud_odom_.reset(new Sync(SyncPolicy(10), point_cloud_sub_, odom_sub_));
        sync_point_cloud_odom_->registerCallback(boost::bind(&Explorer::pointCloudOdomCallback, this, _1, _2));
        
        // publisher
        ufo_cloud_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("ufo_cloud", 1);
        map_pub_ = nh_private_.advertise<ufomap_msgs::UFOMapStamped>("ufomap", 1);
        frontier_pub_ = nh_private_.advertise<visualization_msgs::Marker>("frontiers", 1);
        edges_pub_ = nh_private_.advertise<visualization_msgs::Marker>("edges", 1);
        graph_pub_ = nh_private_.advertise<visualization_msgs::Marker>("graph", 1);
        path_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("path", 1);
        sample_pub_ = nh_private_.advertise<visualization_msgs::Marker>("samples", 1);
        viewpoint_pub_ = nh_private_.advertise<visualization_msgs::Marker>("view_points", 1);
        viewpoint_line_pub_ = nh_private_.advertise<visualization_msgs::Marker>("view_point_lines", 1);
        sample_horizen_pub_ = nh_private_.advertise<visualization_msgs::Marker>("sample_horizen", 1);
        subregion_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("subregion", 1);
        map_change_pub_ = nh_private_.advertise<visualization_msgs::Marker>("map_change", 1);
        centroid_pub_ = nh_private_.advertise<visualization_msgs::Marker>("centroid", 1);
        tour_pub_ = nh_private_.advertise<visualization_msgs::Marker>("tour", 1);
        local_path_pub_ = nh_private_.advertise<visualization_msgs::Marker>("local_path", 1);
        excuted_path_pub_ = nh_private_.advertise<visualization_msgs::Marker>("excuted_path", 1);
        sphere_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("sphere", 1);
        runtime_pub_ = nh_.advertise<std_msgs::Float32>("runtime", 1);
        waypoint_pub_ = nh_.advertise<geometry_msgs::PointStamped>("way_point", 10);

        pub_timer_ = nh_private_.createTimer(ros::Rate(10), &Explorer::ufomapPublishCallback, this);
        explore_timer_ = nh_private_.createTimer(ros::Rate(10), &Explorer::exploreCallback, this);
        // ufomap and pcl_filter
        map_.enableChangeDetection(true);
        map_.enableMinMaxChangeDetection(true);
        voxel_filter_.setLeafSize(scan_voxel_size_, scan_voxel_size_, scan_voxel_size_);
        frontier_filter_.setLeafSize(frontier_voxel_size_, frontier_voxel_size_, frontier_voxel_size_);

        return true;
    }

    void Explorer::terrainMapCallback(const nav_msgs::OccupancyGridPtr &terrain_map) {
        terrain_map_ = terrain_map;
    }

    void Explorer::pointCloudOdomCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud, 
                                                 const nav_msgs::Odometry::ConstPtr &odom) {
        // 机器人冻结检测
        if (last_check_time_.is_zero() || last_odom_ == nullptr) {
            last_check_time_ = ros::Time::now();
            last_odom_ = odom;
        }
        if (ros::Time::now() - last_check_time_ > ros::Duration(check_frozen_intervel)) {
            double dx = odom->pose.pose.position.x - last_odom_->pose.pose.position.x;
            double dy = odom->pose.pose.position.y - last_odom_->pose.pose.position.y;
            double dz = odom->pose.pose.position.z - last_odom_->pose.pose.position.z;
            double distance = sqrt(dx * dx + dy * dy + dz * dz);
            if (distance < check_frozen_distance_thres) {
                is_frozen_ = true;
            }

            last_check_time_ = ros::Time::now();
            last_odom_ = odom;
        }

        sensor_pose_.x() = odom->pose.pose.position.x;
        sensor_pose_.y() = odom->pose.pose.position.y;
        sensor_pose_.z() = odom->pose.pose.position.z;

        // 计算传感器朝向
        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);
        sensor_orientation_ = yaw;
    
        // 使用boost::make_shared创建一个指向 pcl::PointCloud<pcl::PointXYZ> 类型对象的共享指针（shared pointer）
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*cloud, *pcl_cloud);

        // 对点云降采样
        voxel_filter_.setInputCloud(pcl_cloud);
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud_ds;
        voxel_filter_.filter(pcl_cloud_ds);

        // 获取雷达坐标系和地图坐标系的相对位置关系
        ufo::math::Pose6 transform;
        try {
            transform =
                ufomap_ros::rosToUfo(tf_buffer_
                                        .lookupTransform("map", cloud->header.frame_id,
                                                        cloud->header.stamp, ros::Duration(0.1))
                                        .transform);
        } catch (tf2::TransformException &ex) {
            ROS_WARN_THROTTLE(1, "%s", ex.what());
            return;
        }

        // 将pcl点云转存至ufo点云
        ufo::map::PointCloud ufo_cloud_ds;
        for (auto &point:pcl_cloud_ds) {
            // 滤除地面上的点云
            if (point.z > -sensor_height_ + 0.1) {
                ufo_cloud_ds.push_back(ufo::map::Point3(point.x, point.y, point.z));
            }
        }
        ufo_cloud_ds.transform(transform, true);

        // 更新地图
        map_.insertPointCloud(transform.translation(), ufo_cloud_ds, update_max_range_, insert_depth_,
                                simple_ray_casting_, early_stopping_, false);

        // 清除机器人体积
        ufo::map::Point3 robot_bbx_min(sensor_pose_.x() - 0.5 * vehicle_width_,
                                       sensor_pose_.y() - 0.5 * vehicle_length_,
                                        0.0);
        ufo::map::Point3 robot_bbx_max(sensor_pose_.x() + 0.5 * vehicle_width_,
                                       sensor_pose_.y() + 0.5 * vehicle_length_,
                                       sensor_pose_.z());
        ufo::geometry::AABB aabb(robot_bbx_min, robot_bbx_max);  
        map_.setValueVolume(aabb, map_.getClampingThresMin(), clearing_depth_);
    }

    void Explorer::ufomapPublishCallback(const ros::TimerEvent &event) {
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "map";
        
        // 发布ufo map
        ufomap_msgs::UFOMapStamped msg;
        ufomap_msgs::ufoToMsg(map_, msg.map, false);
        msg.header = header;
        map_pub_.publish(msg);
        // 仅有订阅者时才发布ufo cloud
        if (0 < ufo_cloud_pub_.getNumSubscribers() || ufo_cloud_pub_.isLatched()) {
            ufo::map::PointCloud ufo_cloud;
            for (auto it = map_.beginLeaves(true, false, false, false, 0),
                         it_end = map_.endLeaves();
                 it != it_end; ++it) {
                ufo_cloud.push_back(it.getCenter());
            }
            sensor_msgs::PointCloud2 cloud_msg;
            ufomap_ros::ufoToRos(ufo_cloud, cloud_msg);
            cloud_msg.header = header;
            ufo_cloud_pub_.publish(cloud_msg);
        }
    }

    void Explorer::exploreCallback(const ros::TimerEvent &event) {
        std::vector<NodePtr>excuted_path;
        if (map_.validMinMaxChange() && terrain_map_ != nullptr) {
            auto start = ros::WallTime::now();

            // 搜索前沿
            frontierSearch();
            getUpdateBound();
             // 更新图
            graph_.updateGraph(map_, frontier_tree_, map_update_bound_); 
            graph_.extendGraph(map_, frontier_tree_, terrain_map_, map_update_bound_);

            // 视点更新
            viewpoint_manager_.generateViewPoint(map_, frontier_tree_, graph_.graph_);
            if (viewpoint_manager_.viewpoint_tree_.empty()) {
                terminate();
                return;
            }
            // 划分子区域
            subregion_manager_.updateSubregion(map_, viewpoint_manager_.viewpoint_tree_, graph_.graph_, map_update_bound_, sensor_pose_.head<2>());
            subregion_manager_.getGlobalTour(sensor_orientation_, sensor_pose_.head<2>(), map_, graph_.graph_);
            // 获取视点访问顺序
            excuted_path = viewpoint_manager_.getLocalPath(sensor_orientation_, sensor_pose_.head<2>(), subregion_manager_.global_tour_, map_, graph_.graph_);
            // excuted_path = viewpoint_manager_.getGoalViewPoint(sensor_orientation_, sensor_pose_.head<2>(), subregion_manager_.global_tour_, map_, graph_.graph_);
            // 发布路点
            waypointPlanning(excuted_path);
            map_.resetMinMaxChangeDetection();

            std_msgs::Float32 runtime;
            runtime.data = (ros::WallTime::now() - start).toSec();
            runtime_pub_.publish(runtime);
        }
        visualize();
    }

    void Explorer::waypointPlanning(std::vector<NodePtr>excuted_path) {
        // waypoint
        geometry_msgs::PointStamped waypoint;
        waypoint.header.frame_id = "map";
        waypoint.header.stamp = ros::Time::now();

        // goal point
        Eigen::Vector2d goal;
        bool get_goal = false;

        for (size_t i = excuted_path.size() - 1; i > 0; i--) {
            ufo::map::Point3 start_point(excuted_path[i]->position.x(), excuted_path[i]->position.y(), sensor_height_);
            ufo::map::Point3 end_point(sensor_pose_.x(), sensor_pose_.y(), sensor_height_);
            if (map_.isCollisionFree(start_point, end_point, true) && (end_point - start_point).norm() > 0.5) {
                goal = excuted_path[i]->position.head<2>();
                get_goal = true;
                break;
            }
        }
        if (!get_goal || is_frozen_) {
            if (is_frozen_) {
                ROS_WARN("Robot is frozen!");
                is_frozen_ = false;
            }
            // todo 找到最近自由的栅格作为目标点
            goal.x() = sensor_pose_.x() + 1.0 * cos(sensor_orientation_);
            goal.y() = sensor_pose_.y() + 1.0 * sin(sensor_orientation_);
        }
        waypoint.point.x = goal.x();
        waypoint.point.y = goal.y();
        waypoint.point.z = sensor_height_;
        waypoint_pub_.publish(waypoint);
    }

    void Explorer::terminate() {
        // 没有视点时表示探索完毕
        ROS_INFO("\033[1;32mExploration finished!\033[0m");
        ros::shutdown();
    }

    void Explorer::frontierSearch() {
        changed_cell_codes_.clear();

        for (auto it = map_.changesBegin(); it != map_.changesEnd(); it++) {
            changed_cell_codes_.insert(*it);
        }

        map_.resetChangeDetection();

        findLocalFrontier();
        updateGlobalFrontier();
        frontierDownsampling();
    }

    void Explorer::findLocalFrontier() {
        // 每次清除检测的local frontiers
        local_frontier_codes_.clear();
        // 遍历地图中变化的cell，从变化的cell中找到新的前沿
        // 因为新的前沿只会存在于变化的cell中
        for (const auto &changed_cell: changed_cell_codes_) {
            if (frontier_depth_ == changed_cell.getDepth() && history_frontier_cells_.count(changed_cell) == 0) {  
                ufo::map::Point3 point = map_.toCoord(changed_cell.toKey());
                if (point.z() > (sensor_height_ - 1 * map_.getResolution()) && point.z() < (sensor_height_ + 1.5 * map_.getResolution())) {
                    if (inExplorationArea(point) && inSensorRange(point)) {
                        if (isFrontier(changed_cell)) {
                            local_frontier_codes_.insert(changed_cell);
                            history_frontier_cells_.insert(changed_cell);
                        }
                    } 
                }  
            }
        }
    }

    bool Explorer::isFrontier(const ufo::map::Code &frontier) {
        if (map_.isFree(frontier)) {
            const double resolution = map_.getResolution();
            bool xPositive = false, xNegative = false, yPositive = false, yNegative = false;
            ufo::map::Point3 frontier_coord = map_.toCoord(frontier.toKey(), frontier_depth_);
            ufo::map::Point3 neighbor0_coord, neighbor1_coord;
            //  xPositive
            neighbor0_coord = frontier_coord + ufo::map::Point3(resolution, 0, 0);
            neighbor1_coord = frontier_coord + ufo::map::Point3(2 * resolution, 0, 0);
            if (map_.isUnknown(neighbor0_coord, frontier_depth_) && map_.isUnknown(neighbor1_coord, frontier_depth_)) {
                xPositive = true;
            }
            //  xNegative
            neighbor0_coord = frontier_coord - ufo::map::Point3(resolution, 0, 0);
            neighbor1_coord = frontier_coord - ufo::map::Point3(2 * resolution, 0, 0);
            if (map_.isUnknown(neighbor0_coord, frontier_depth_) && map_.isUnknown(neighbor1_coord, frontier_depth_)) {
                xNegative = true;
            }
            //  yPositive
            neighbor0_coord = frontier_coord + ufo::map::Point3(0, resolution, 0);
            neighbor1_coord = frontier_coord + ufo::map::Point3(0, 2 * resolution, 0);
            if (map_.isUnknown(neighbor0_coord, frontier_depth_) && map_.isUnknown(neighbor1_coord, frontier_depth_)) {
                yPositive = true;
            }
            //  yNegative
            neighbor0_coord = frontier_coord - ufo::map::Point3(0, resolution, 0);
            neighbor1_coord = frontier_coord - ufo::map::Point3(0, 2 * resolution, 0);
            if (map_.isUnknown(neighbor0_coord, frontier_depth_) && map_.isUnknown(neighbor1_coord, frontier_depth_)) {
                yNegative = true;
            }
            // 判断是否为前沿
            if (xPositive || xNegative || yPositive || yNegative) {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    void Explorer::updateGlobalFrontier() {
        for (const auto &local_frontier_code: local_frontier_codes_) {
            global_frontier_codes_.insert(local_frontier_code);
        }
        
        CodeUnorderSet global_frontiers_tmp;
        // 对全局前沿进行检查
        if (!global_frontier_codes_.empty()) {
            for (const auto &global_frontier_code: global_frontier_codes_) {
                ufo::map::Point3 cell_coord = map_.toCoord(global_frontier_code.toKey());
                if (inSensorRange(cell_coord)) {
                    if (isFrontier(global_frontier_code)) {  
                        global_frontiers_tmp.insert(global_frontier_code);
                    }
                }
            }
            global_frontier_codes_ = global_frontiers_tmp;
        }
    }
    
    void Explorer::frontierDownsampling() {
        frontier_tree_.clear();

        // 使用pcl库对前沿点进行降采样
        pcl::PointCloud<pcl::PointXYZ>::Ptr frontier_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        if (!global_frontier_codes_.empty()) {
            for(const auto &global_frontier_code: global_frontier_codes_) {
                ufo::map::Point3 cell_coord = map_.toCoord(global_frontier_code.toKey());
                pcl::PointXYZ point;
                point.x = cell_coord.x();
                point.y = cell_coord.y();
                point.z = cell_coord.z();
                frontier_cloud->push_back(point);
            }
            pcl::PointCloud<pcl::PointXYZ> frontier_cloud_ds;
            frontier_filter_.setInputCloud(frontier_cloud);
            frontier_filter_.filter(frontier_cloud_ds);
            
            // 将降采样后的前沿点存入frontier_tree
            for (const auto &point: frontier_cloud_ds) {
                Point frontier_point;
                frontier_point.set<0>(point.x);
                frontier_point.set<1>(point.y);
                frontier_point.set<2>(point.z);
                frontier_tree_.insert(frontier_point);
            }
        }
    }

    bool Explorer::inSensorRange(const ufo::map::Point3 &cell_coord) {
        ufo::map::Point3 direction;
        direction.x() = cell_coord.x() - sensor_pose_.x();
        direction.y() = cell_coord.y() - sensor_pose_.y();
        direction.z() = cell_coord.z() - sensor_pose_.z();

        double distance = direction.norm();
        if (distance < search_radius_min_) {
            return false;
        }

        bool in_sensor_range = false;
        if (fabs(direction.z()) < sqrt(direction.x() * direction.x() + direction.y() * direction.y()) * tan(2 * M_PI * sensor_vertical_fov_ / 360)) {
            in_sensor_range = true;
        }

        if (in_sensor_range) {
            return true;
        } else {
            return false;
        }
    }

    bool Explorer::inExplorationArea(const ufo::map::Point3 &cell_coord) {
        if (cell_coord.x() < bound_x_min_ || cell_coord.x() > bound_x_max_ ||
            cell_coord.y() < bound_y_min_ || cell_coord.y() > bound_y_max_) {
                return false;
            } else {
                return true;
            }
    }

    void Explorer::visualizeFrontier() {
        double size = map_.getResolution();
        visualization_msgs::Marker global_frontier_markers;
        global_frontier_markers.header.frame_id = "map";
        global_frontier_markers.ns = "global_frontiers";
        global_frontier_markers.type = visualization_msgs::Marker::SPHERE_LIST;
        global_frontier_markers.action = visualization_msgs::Marker::ADD;
        global_frontier_markers.scale.x = size;
        global_frontier_markers.scale.y = size;
        global_frontier_markers.scale.z = size;
        global_frontier_markers.color.a = 0.5;
        global_frontier_markers.color.r = 0;
        global_frontier_markers.color.g = 1;
        global_frontier_markers.color.b = 1;
        global_frontier_markers.pose.orientation.w = 1;

        // 把frontier tree中的前沿点转换成marker消息发布
        for (auto it = frontier_tree_.begin(); it != frontier_tree_.end(); it++) {
            geometry_msgs::Point center;
            center.x = it->get<0>();
            center.y = it->get<1>();
            center.z = it->get<2>();
            global_frontier_markers.points.push_back(center);
        }

        frontier_pub_.publish(global_frontier_markers);
    }

    void Explorer::visualize() {
        // 可视化前沿
        visualizeFrontier();

        // 可视化子区域
        visualization_msgs::MarkerArray subregion_markers = subregion_manager_.visualizeSubregion();
        subregion_pub_.publish(subregion_markers);
        visualization_msgs::Marker centroid_marker = subregion_manager_.visualizeCentroid();
        centroid_pub_.publish(centroid_marker);
        visualization_msgs::Marker global_tour_marker = subregion_manager_.visualizeGlobalTour(sensor_pose_.head<2>());
        tour_pub_.publish(global_tour_marker);
        // visualization_msgs::Marker local_bound_marker = subregion_manager_.visualizeLocalBound();
        // local_bound_pub_.publish(local_bound_marker);

        // 可视化图
        visualization_msgs::Marker edges_marker = graph_.visualizeEdge();
        edges_pub_.publish(edges_marker);
        visualization_msgs::Marker graph_marker = graph_.visualizeGraph();
        graph_pub_.publish(graph_marker);

        // 可视化视点
        visualization_msgs::Marker viewpoint_marker = viewpoint_manager_.visualizeViewPoint();
        viewpoint_pub_.publish(viewpoint_marker);
        visualization_msgs::Marker viewpoint_line_marker = viewpoint_manager_.visualizeViewPointLine();
        viewpoint_line_pub_.publish(viewpoint_line_marker);
        visualization_msgs::Marker local_path_marker = viewpoint_manager_.visualizeLocalPath(sensor_pose_.head<2>());
        local_path_pub_.publish(local_path_marker);
        visualization_msgs::MarkerArray sphere_marker = viewpoint_manager_.visualizeSphere();
        sphere_pub_.publish(sphere_marker);

        // 可视化路径
        visualization_msgs::Marker excuted_path_marker = viewpoint_manager_.visualizeExcutedPath();
        excuted_path_pub_.publish(excuted_path_marker);

        visualization_msgs::MarkerArray path_marker = subregion_manager_.visualizeViewPaths();
        path_pub_.publish(path_marker);
        // 可视化采样空间
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        
        geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;
        p1.x = graph_.sample_bound_min_.x(); p1.y = graph_.sample_bound_min_.y(); p1.z = 0.0;
        p2.x = graph_.sample_bound_min_.x(); p2.y = graph_.sample_bound_max_.y(); p2.z = 0.0;

        p3.x = graph_.sample_bound_max_.x(); p3.y = graph_.sample_bound_max_.y(); p3.z = 0.0;
        p4.x = graph_.sample_bound_max_.x(); p4.y = graph_.sample_bound_min_.y(); p4.z = 0.0;

        p5.x = graph_.sample_bound_min_.x(); p5.y = graph_.sample_bound_min_.y(); p5.z = 2.0;
        p6.x = graph_.sample_bound_min_.x(); p6.y = graph_.sample_bound_max_.y(); p6.z = 2.0;

        p7.x = graph_.sample_bound_max_.x(); p7.y = graph_.sample_bound_max_.y(); p7.z = 2.0;
        p8.x = graph_.sample_bound_max_.x(); p8.y = graph_.sample_bound_min_.y(); p8.z = 2.0;
        
        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker.points.push_back(p2);
        marker.points.push_back(p3);
        marker.points.push_back(p3);
        marker.points.push_back(p4);
        marker.points.push_back(p4);
        marker.points.push_back(p1);
        marker.points.push_back(p5);
        marker.points.push_back(p6);
        marker.points.push_back(p6);
        marker.points.push_back(p7);
        marker.points.push_back(p7);
        marker.points.push_back(p8);
        marker.points.push_back(p8);
        marker.points.push_back(p5);
        marker.points.push_back(p1);
        marker.points.push_back(p5);
        marker.points.push_back(p6);
        marker.points.push_back(p2);
        marker.points.push_back(p7);
        marker.points.push_back(p3);
        marker.points.push_back(p8);
        marker.points.push_back(p4);

        // 发布立方体的marker消息
        sample_horizen_pub_.publish(marker);

        // 发布map_update_bound_，发布2d的矩形框
        visualization_msgs::Marker marker_box;
        marker_box.header.frame_id = "map";
        marker_box.header.stamp = ros::Time::now();
        marker_box.type = visualization_msgs::Marker::LINE_LIST;
        marker_box.action = visualization_msgs::Marker::ADD;
        marker_box.pose.orientation.w = 1.0;
        marker_box.scale.x = 0.1;
        marker_box.scale.y = 0.1;
        marker_box.scale.z = 0.1;
        marker_box.color.r = 1.0;
        marker_box.color.g = 0.5;
        marker_box.color.b = 0.5;
        marker_box.color.a = 1.0;

        geometry_msgs::Point p9, p10, p11, p12;
        p9.x = map_update_bound_.min_corner().get<0>(); 
        p9.y = map_update_bound_.min_corner().get<1>(); 
        p9.z = 0.0;

        p10.x = map_update_bound_.min_corner().get<0>();
        p10.y = map_update_bound_.max_corner().get<1>();
        p10.z = 0.0;

        p11.x = map_update_bound_.max_corner().get<0>();
        p11.y = map_update_bound_.max_corner().get<1>();
        p11.z = 0.0;

        p12.x = map_update_bound_.max_corner().get<0>();
        p12.y = map_update_bound_.min_corner().get<1>();
        p12.z = 0.0;

        marker_box.points.push_back(p9);
        marker_box.points.push_back(p10);
        marker_box.points.push_back(p10);
        marker_box.points.push_back(p11);
        marker_box.points.push_back(p11);
        marker_box.points.push_back(p12);
        marker_box.points.push_back(p12);
        marker_box.points.push_back(p9);

        map_change_pub_.publish(marker_box);
    }

    void Explorer::getUpdateBound() {
        double inflate = 0.5;
        Eigen::Vector2d change_bound_min_ = {map_.minChange()[0] - inflate, map_.minChange()[1] - inflate};
        Eigen::Vector2d change_bound_max_ = {map_.maxChange()[0] + inflate, map_.maxChange()[1] + inflate};

        map_update_bound_ = Box(Point(change_bound_min_.x(), change_bound_min_.y(), 0), Point(change_bound_max_.x(), change_bound_max_.y(), 2.0));
    }
}
