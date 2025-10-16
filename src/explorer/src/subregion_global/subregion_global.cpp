#include <subregion_global/subregion_global.h>

using namespace std;
namespace explorer_ns {
    SubregionManager::SubregionManager(const double &bound_x_min, const double &bound_x_max, const double &bound_y_min, const double &bound_y_max,
                                       const double &subregion_max_width, const double &subregion_max_length, const double &subregion_min_width, const double &subregion_min_length,
                                       const double &known_voxel_thre):
        known_voxel_thre_(known_voxel_thre) {
        
        // 初始化子区域的高度
        subregion_height_ = 2.0;
        // 初始化探测区域
        Box explorer_area(Point(bound_x_min, bound_y_min, 0.0), Point(bound_x_max, bound_y_max, subregion_height_));
        // 对区域进行初始化划分
        initSubregion(explorer_area, subregion_max_width, subregion_max_length, subregion_min_width, subregion_min_length);
    }

    std::vector<SubregionPtr> SubregionManager::divideSubregion(const Eigen::Vector2d &sensor_pose, const SubregionPtr &subregion, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &viewpoint_tree, const NodeRtree &graph) {
        // 先把该子区域分成四份
        std::vector<SubregionPtr> divided_cells;
        // 对于每个子区域，如果内部前沿点大于阈值，计算其前沿点，然后计算其质心
        for (size_t i = 0; i < 4; i++) {
            // 分割子区域
            Box sub_aabb = divideBox(subregion->aabb, i);
            SubregionPtr sub_cell = generateSubregion(sub_aabb.min_corner(), sub_aabb.max_corner(), subregion->level + 1);
            
            // 如果子区域内的前沿点数量大于阈值，计算相关信息
            getViewPointInSubregion(sub_cell, viewpoint_tree);
            calculateViewPointCentroid(sensor_pose, sub_cell, ufo_map, graph);
            calculateKnownVoxelRatio(sub_cell, ufo_map);
            
            if (sub_cell->view_points.empty()) {
                subregions_invalid_.push_back(sub_cell);
            }
            else {
                divided_cells.push_back(sub_cell);
            }
        }

        return divided_cells;
    }

    bool SubregionManager::isOverlap(const Box &aabb1, const Box &aabb2) {
        // 判断两个子区域是否重叠
        return boost::geometry::intersects(aabb1, aabb2);
    }

    void SubregionManager::calculateMaxLevel(const double &subregion_max_width, const double &subregion_max_length, const double &subregion_min_width, const double &subregion_min_length) {
        max_level_ = std::max(std::log2(subregion_max_width / subregion_min_width), std::log2(subregion_max_length / subregion_min_length)) + 1;

        ROS_INFO("Max level: %d", max_level_);
    }

    void SubregionManager::initSubregion(const Box &explorer_area, const double &subregion_max_width, const double &subregion_max_length, const double &subregion_min_width, const double &subregion_min_length) {
        subregions_.clear();

        calculateMaxLevel(subregion_max_width, subregion_max_length, subregion_min_width, subregion_min_length);

        double explorer_area_length = explorer_area.max_corner().get<0>() - explorer_area.min_corner().get<0>();
        double explorer_area_width = explorer_area.max_corner().get<1>() - explorer_area.min_corner().get<1>();
        size_t x_num = std::ceil(explorer_area_length / subregion_max_length);
        size_t y_num = std::ceil(explorer_area_width / subregion_max_width);

        double subregion_length = explorer_area_length / x_num;
        double subregion_width = explorer_area_width / y_num;
        for (size_t i = 0; i < x_num; i++) {
            for (size_t j = 0; j < y_num; j++) {
                Point min(explorer_area.min_corner().get<0>() + i * subregion_length, 
                          explorer_area.min_corner().get<1>() + j * subregion_width, 0.0);
                Point max(explorer_area.min_corner().get<0>() + (i + 1) * subregion_length, 
                          explorer_area.min_corner().get<1>() + (j + 1) * subregion_width, subregion_height_);
                Box sub_aabb(min, max);
                SubregionPtr sub_cell = generateSubregion(sub_aabb.min_corner(), sub_aabb.max_corner(), 1);
                subregions_.push_back(sub_cell);
            }
        }

        ROS_INFO("Subregion initialization completed!");
    }

    void SubregionManager::updateSubregion(const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &viewpoint_tree, 
                                           const NodeRtree &graph, const Box &map_update_bound, const Eigen::Vector2d &sensor_pose) {
        if (viewpoint_tree.empty()) {
            return;
        }

        already_searched_path_.clear();
        robot2subregion_.clear();
        
        // 1. 检查失效的子区域能不能加入到有效的子区域中
        for (auto it = subregions_invalid_.begin(); it != subregions_invalid_.end();) {
            if (isOverlap((*it)->aabb, map_update_bound)) {
                updateInfo(sensor_pose, *it, ufo_map, viewpoint_tree, graph);
            }

            if (!(*it)->view_points.empty()){
                // 加入到有效的子区域中
                subregions_.push_back(*it);
                it = subregions_invalid_.erase(it);
            }
            else {
                ++it;
            }
        }

        // 2. 再检查有效的子区域是否已经失效
        for (auto it = subregions_.begin(); it != subregions_.end();) {
            if (isOverlap((*it)->aabb, map_update_bound)) {
                updateInfo(sensor_pose, *it, ufo_map, viewpoint_tree, graph);
            }

            if ((*it)->view_points.empty()) {
                // 加入到失效的子区域中
                subregions_invalid_.push_back(*it);
                it = subregions_.erase(it);
            }
            else {
                ++it;
            }
        }
        
        // 3. 对剩余有效的子区域进行划分
        for (auto it = subregions_.begin(); it != subregions_.end();) {
            if ((*it)->level >= max_level_ || (*it)->known_voxel_ratio <= known_voxel_thre_) {
                // 只有与当前更新区域交叠的区域才需要更新距离
                if (isOverlap((*it)->aabb, map_update_bound) || (*it)->distances.size() < subregions_.size() - 1) {
                    updateDistance(*it, ufo_map, graph);
                }
                ++it;
            }
            else {
                std::deque<SubregionPtr> queue;
                queue.push_back(*it);
                it = subregions_.erase(it);
                // 尝试往下划分
                while (!queue.empty()) {
                    SubregionPtr current = queue.front();
                    queue.pop_front();

                    if (current->level < max_level_ && current->known_voxel_ratio > known_voxel_thre_) {
                        // 划分子区域
                        std::vector<SubregionPtr> divided_subregions = divideSubregion(sensor_pose, current, ufo_map, viewpoint_tree, graph);
                        // 将新的子区域加入到队列中
                        queue.insert(queue.end(), divided_subregions.begin(), divided_subregions.end());
                    }
                    else {
                        updateDistance(current, ufo_map, graph);
                        subregions_.push_back(current);
                    }
                }
            }
        }
       
        // todo
        for (const auto &subregion: subregions_) {
            deleteInvaildDistance(subregion);
        }

        checkDistance();
    }

    void SubregionManager::updateInfo(const Eigen::Vector2d &sensor_pose, const SubregionPtr &subregion, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &viewpoint_tree, const NodeRtree &graph) {
        // 更新子区域的信息
        getViewPointInSubregion(subregion, viewpoint_tree);
        calculateViewPointCentroid(sensor_pose, subregion, ufo_map, graph);
        calculateKnownVoxelRatio(subregion, ufo_map);
    }

    void SubregionManager::updateDistance(const SubregionPtr &subregion, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph) {
        // 更新子区域与其他子区域的距离
        // 也更新其他子区域与该子区域的距离
        for (const auto &other: subregions_) {
            if (subregion != other && 
                // 记录已经搜索过的路径，避免重复计算
                already_searched_path_.find(std::make_pair(other, subregion)) == already_searched_path_.end() && 
                already_searched_path_.find(std::make_pair(subregion, other)) == already_searched_path_.end()) {
                getSubregionDistance(ufo_map, graph, subregion, other);
            }
        }
    }

    void SubregionManager::deleteInvaildDistance(const SubregionPtr &subregion) {
        // 删除失效的路径
        for (auto dist = subregion->distances.begin(); dist != subregion->distances.end();) {
            bool find = false;
            for (const auto subregion: subregions_) {
                if (subregion == dist->first) {
                    find = true;
                    break;
                }
            }
            if (!find) {
                dist = subregion->distances.erase(dist);
            }
            else {
                ++dist;
            }
        }

        // for test
        for (auto path = subregion->paths.begin(); path != subregion->paths.end();) {
            bool find_path = false;
            for (const auto subregion: subregions_) {
                if (subregion == path->first) {
                    find_path = true;
                    break;
                }
            }
            if (!find_path) {
                path = subregion->paths.erase(path);
            }
            else {
                ++path;
            }
        }
    }

    void SubregionManager::checkDistance() {
        // for check or test
        for (const auto subregion: subregions_) {
            if (subregion->distances.size() != (subregions_.size() - 1)) {
                ROS_ERROR("Distance error!");
            }
        }
    }
    
    void SubregionManager::getGlobalTour(const double &sensor_orient, const Eigen::Vector2d &sensor_pose, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph) {
        // 使用TSP求解全局路径
        DataModel data;
        data.dist_matrix = getDistMatrix(sensor_orient, sensor_pose, ufo_map, graph);
        TSPSolver tsp_solver(data);
        tsp_solver.solve();

        // 得到TSP路径索引
        std::vector<int> tour = tsp_solver.getSolutionIndex();

        // 根据TSP路径索引得到子区域访问顺序
        global_tour_.clear();
        for (const auto &index: tour) {
            if (index == 0) {
                continue;
            }
            global_tour_.push_back(subregion_index_[index]);
        }

        // tsp_solver.printSolution();
    }

    std::vector<NodePtr> SubregionManager::getShortestPath(const Eigen::Vector2d &position, const SubregionPtr &subregion, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph) {
        std::vector<NodePtr> path;

        // 基于可见性判断能否直接到达
        if (isVisible(ufo_map, position, subregion->centroid)) {
            // 直接计算机器人到子区域的距离
            path = {std::make_shared<Node>(position), std::make_shared<Node>(subregion->centroid)};
            // for test
            robot2subregion_.push_back(path);

            return path;
        }
        
        // 计算两个点之间的距离
        NodePtr start_node = getNearestNodeVisible(position, ufo_map, graph);
        NodePtr end_node = getNearestNodeVisible(subregion->centroid, ufo_map, graph);
        if (start_node == nullptr) {
            start_node = getNearestNode(position, graph);
        }
        else if (end_node == nullptr) {
            end_node = getNearestNode(subregion->centroid, graph);
        }

        if (start_node == nullptr || end_node == nullptr) {
            ROS_ERROR("Can't find the nearest node!");
            return {std::make_shared<Node>(position), std::make_shared<Node>(subregion->centroid)};
        }
        path = aStar(start_node, end_node);

        // 补偿由于寻找最近点而引入的误差
        NodePtr init_start_node = std::make_shared<Node>(position);
        NodePtr init_end_node = std::make_shared<Node>(subregion->centroid);
        path.insert(path.begin(), init_start_node);
        path.push_back(init_end_node);

        // for test
        robot2subregion_.push_back(path);

        return path;
    }

    double SubregionManager::getPathDistance(const Eigen::Vector2d &sensor_pos, const SubregionPtr &subregion, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph) {
        double distance = 0.0;
        std::vector<NodePtr> path = getShortestPath(sensor_pos, subregion, ufo_map, graph);

        for (size_t i = 0; i < path.size() - 1; i++) {
            distance += (path[i]->position - path[i + 1]->position).norm();
        }

        return distance;
    }

    NodePtr SubregionManager::getNearestNode(const Eigen::Vector2d &centroid, const NodeRtree &graph) {
        std::vector<Value> result;
        Point query_point(centroid.x(), centroid.y(), 0.0);
        graph.query(boost::geometry::index::nearest(query_point, 1), std::back_inserter(result));

        if (result.size() != 1) {
            return nullptr;
        } else {
            return result[0].second;
        }
    }

    NodePtr SubregionManager::getNearestNodeVisible(const Eigen::Vector2d &centroid, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph) {
        std::vector<Value> result;
        Point query_point(centroid.x(), centroid.y(), 0.0);

        auto pred = boost::geometry::index::satisfies([&](const Value &v) {
                        return isVisible(ufo_map, v.second->position, centroid);
                    });
        graph.query(pred, std::back_inserter(result));
        std::sort(result.begin(), result.end(), [&](const Value &v1, const Value &v2) {
            return boost::geometry::distance(query_point, v1.first) < boost::geometry::distance(query_point, v2.first);
        });

        if (result.empty()) {
            return nullptr;
        }

        return result.front().second;
    }

    SubregionPtr SubregionManager::generateSubregion(const Point &min, const Point &max, const size_t &level) {
        // 生成子区域
        Box aabb(min, max);
        Eigen::Vector2d center = getSubregionCenter(aabb);
        
        return std::make_shared<Subregion>(aabb, center, level);
    }

    void SubregionManager::getViewPointInSubregion(const SubregionPtr &subregion, const NodeRtree &viewpoint_tree) {
        // 获取子区域内的前沿点
        std::vector<Value> results;
        viewpoint_tree.query(boost::geometry::index::intersects(subregion->aabb), 
                            std::back_inserter(results));

        std::vector<NodePtr> view_points;
        for (const auto &result: results) {
            view_points.push_back(result.second);
        }

        subregion->view_points = view_points;
    }

    std::vector<std::vector<int>> SubregionManager::getDistMatrix(const double &sensor_orient, const Eigen::Vector2d &sensor_pose, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph) {
        std::vector<std::vector<int>> dist_matrix(subregions_.size() + 1, std::vector<int>(subregions_.size() + 1, 0));
        subregion_index_.clear();

        // 先在矩阵的第一行和第一列填充机器人到各子区域的距离
        auto it = subregions_.begin();
        for (size_t i = 0; i < subregions_.size(); ++i, ++it) {
            dist_matrix[0][i + 1] = static_cast<int>(getPathDistance(sensor_pose, *it, ufo_map, graph) * 10);
        }
        
        // 再填充子区域之间的距离
        auto it1 = subregions_.begin();
        for (size_t i = 0; i < subregions_.size(); ++i, ++it1) {
            // 索引0是机器人自身位置
            subregion_index_.insert(std::make_pair(i + 1, *it1));
            auto it2 = subregions_.begin();
            for (size_t j = 0; j < subregions_.size(); ++j, ++it2) {
                if (*it1 != *it2) {
                    dist_matrix[i + 1][j + 1] = static_cast<int>((*it1)->distances[*it2] * 10);
                }
            }
        }

        // 打印矩阵，并格式化输出
        // for (size_t i = 0; i < dist_matrix.size(); ++i) {
        //     for (size_t j = 0; j < dist_matrix[i].size(); ++j) {
        //         std::cout << std::setw(5) << dist_matrix[i][j] << " ";
        //     }
        //     std::cout << std::endl;
        // }

        return dist_matrix;
    }

    Box SubregionManager::divideBox(const Box &aabb, const size_t &index) {
        // 将子区域分割成四份
        Point min, max;
        min = aabb.min_corner();
        max = aabb.max_corner();

        double box_width = max.get<0>() - min.get<0>();
        double box_length = max.get<1>() - min.get<1>();
        double half_width = box_width / 2;
        double half_length = box_length / 2;

        min = Point(min.get<0>() + half_width * (index % 2), min.get<1>() + half_length * (index / 2), 0.0);
        max = Point(max.get<0>() - half_width * (1 - index % 2), max.get<1>() - half_length * (1 - index / 2), subregion_height_);
        
        return Box(min, max);
    }

    void SubregionManager::calculateViewPointCentroid(const Eigen::Vector2d &sensor_pose ,const SubregionPtr &subregion, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph) {
        // 计算前沿点的质心
        if (subregion->view_points.empty()) {
            subregion->centroid = subregion->center;
        }
        else {
            double x = 0.0;
            double y = 0.0;
            for (const auto &view_point: subregion->view_points) {
                x += view_point->position.x();
                y += view_point->position.y();
            }
            x /= subregion->view_points.size();
            y /= subregion->view_points.size();

            Eigen::Vector2d centroid = Eigen::Vector2d(x, y);
            ufo::map::Point3 centroid_point(centroid.x(), centroid.y(), buffer_height_);
            if (ufo_map.isOccupied(centroid_point)) {
                // 以子区域内距离质心最近的视点作为质心
                for (size_t i = 0; i < subregion->view_points.size(); i++) {
                    double min_distance = std::numeric_limits<double>::max();
                    double distance = (subregion->view_points[i]->position - centroid).norm();
                    if (distance < min_distance) {
                        centroid = subregion->view_points[i]->position;
                        min_distance = distance;
                    }
                }
            }
            else {
                subregion->centroid = centroid;
            }
        }
    }

    Eigen::Vector2d SubregionManager::getSubregionCenter(const Box &box) {
        // 计算子区域的中心
        double x = (box.min_corner().get<0>() + box.max_corner().get<0>()) / 2;
        double y = (box.min_corner().get<1>() + box.max_corner().get<1>()) / 2;

        return Eigen::Vector2d(x, y);
    }

    void SubregionManager::calculateKnownVoxelRatio(const SubregionPtr &subregion, const ufo::map::OccupancyMapColor &ufo_map) {
        Eigen::Vector2d min(subregion->aabb.min_corner().get<0>(), subregion->aabb.min_corner().get<1>());
        Eigen::Vector2d max(subregion->aabb.max_corner().get<0>(), subregion->aabb.max_corner().get<1>());
        
        double res = ufo_map.getResolution();
        // 直接设定z的高度，避免计算量过大
        double z = 0.6;

        double known_voxel = 0.0;
        double unknown_voxel = 0.0;

        for (double x = min.x(); x < max.x(); x += res) {
            for (double y = min.y(); y < max.y(); y += res) {
                ufo::map::Point3 point(x, y, z);
                if (ufo_map.isUnknown(point)) {
                    unknown_voxel += 1;
                }
                else {
                   known_voxel += 1;
                }
            }
        }

        subregion->known_voxel_ratio = known_voxel / (known_voxel + unknown_voxel);
    }

    void SubregionManager::getSubregionDistance(const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph, const SubregionPtr &subregion1, const SubregionPtr &subregion2) {
        double distance = 0.0;
        std::vector<NodePtr> path = getShortestPath(subregion1->centroid, subregion2, ufo_map, graph);

        for (size_t i = 0; i < path.size() - 1; i++) {
            distance += (path[i]->position - path[i + 1]->position).norm();
        }

        subregion1->distances[subregion2] = distance;
        subregion2->distances[subregion1] = distance;

        // 记录已经更新的子区域之间的距离
        already_searched_path_.insert(std::make_pair(subregion1, subregion2));
        already_searched_path_.insert(std::make_pair(subregion2, subregion1));

        // for test
        subregion1->paths[subregion2] = path;
        subregion2->paths[subregion1] = path;
    }

    bool SubregionManager::isVisible(const ufo::map::OccupancyMapColor &ufo_map, const Eigen::Vector2d &position, const Eigen::Vector2d &centroid) {
        // 判断前沿点是否可见
        ufo::map::Point3 start(position.x(), position.y(), buffer_height_);
        ufo::map::Point3 end(centroid.x(), centroid.y(), buffer_height_);
        
        return  (end - start).norm() < 5.0 && ufo_map.isCollisionFree(start, end, true);
    }

    std::vector<NodePtr> SubregionManager::aStar(NodePtr start, NodePtr end) {
        std::vector<NodePtr> path;
        // std::unordered_map 存储的是键值对，每个元素都是由键和值组成的，键是唯一的，而值则可以重复
        // std::unordered_set 存储的是唯一的元素集合，每个元素都是唯一的，不能重复
        // std::unordered_map 适用于需要将键和值关联起来，并且需要根据键来快速查找值的情况
        // std::unordered_set 适用于需要维护一组唯一元素的情况，常用于去重或者判断某个元素是否存在的场景
        std::multimap<double, NodePtr> open_list;
        std::unordered_set<NodePtr> closed_list;
        std::unordered_map<NodePtr, NodePtr> parent;
        std::unordered_map<NodePtr, double> g_score;
        std::unordered_map<NodePtr, double> f_score;

        // f = g + h
        g_score[start] = 0.0;
        f_score[start] = g_score[start] + (start->position - end->position).norm();
        open_list.insert(std::make_pair(f_score[start], start));

        while (!open_list.empty()) {
            auto current = open_list.begin()->second;
            if (current == end) {
                while (current != start) {
                    path.push_back(current);
                    current = parent[current];
                }
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                // return path;
                return {start, end};
            }

            open_list.erase(open_list.begin());
            closed_list.insert(current);

            for (const auto &neighbor: current->neighbors) {
                if (closed_list.find(neighbor) != closed_list.end()) {
                    continue;
                }

                double tentative_g_score = g_score[current] + (current->position - neighbor->position).norm();
                if (g_score.count(neighbor) == 0 || tentative_g_score < g_score[neighbor]) {
                    parent[neighbor] = current;
                    g_score[neighbor] = tentative_g_score;
                    f_score[neighbor] = g_score[neighbor] + (neighbor->position - end->position).norm();;
                    open_list.insert(std::make_pair(f_score[neighbor], neighbor));
                }
            }
        }
        // 没找到路径
        path = {start, end};
        // ROS_WARN("No path found in subregion!");

        return path;
    }

    visualization_msgs::MarkerArray SubregionManager::visualizeSubregion() {
        // 使用CUBE类型的MarkerArray可视化子区域
        visualization_msgs::MarkerArray markers;

        // 创建一个删除所有Marker的Marker，并将其添加到MarkerArray中
        visualization_msgs::Marker delete_all;
        delete_all.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(delete_all);

        int id = 0;
        for (const auto &subregion : subregions_) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "subregions";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = subregion->center.x();
            marker.pose.position.y = subregion->center.y();
            marker.pose.position.z = subregion_height_ / 2;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = subregion->aabb.max_corner().get<0>() - subregion->aabb.min_corner().get<0>();
            marker.scale.y = subregion->aabb.max_corner().get<1>() - subregion->aabb.min_corner().get<1>();
            marker.scale.z = subregion_height_;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 0.8;
            marker.color.a = 0.4;
            markers.markers.push_back(marker);
        }
        return markers;
    }

    visualization_msgs::Marker SubregionManager::visualizeCentroid() {
        // 使用SPHERE类型的MarkerArray可视化子区域的质心
        visualization_msgs::Marker markers;

        markers.header.frame_id = "map";
        markers.header.stamp = ros::Time::now();
        markers.ns = "subregions";
        markers.id = 0;
        markers.type = visualization_msgs::Marker::SPHERE_LIST;
        markers.action = visualization_msgs::Marker::ADD;
        markers.pose.orientation.w = 1.0;
        markers.scale.x = 0.5;
        markers.scale.y = 0.5;
        markers.scale.z = 0.5;
        markers.color.r = 1.0;
        markers.color.g = 0.8;
        markers.color.b = 1.0;
        markers.color.a = 1.0;

        for (const auto &subregion : subregions_) {
            geometry_msgs::Point point;
            point.x = subregion->centroid.x();
            point.y = subregion->centroid.y();
            point.z = subregion_height_ / 2;
            markers.points.push_back(point);
        }

        return markers;
    }

    visualization_msgs::MarkerArray SubregionManager::visualizeViewPaths() {
        // 使用LINE_STRIP类型的MarkerArray可视化子区域之间的路径
        visualization_msgs::MarkerArray markers;
        std::vector<std_msgs::ColorRGBA> colors;
        // 定义10种颜色
        std_msgs::ColorRGBA rgba1;
        rgba1.r = 1.0;
        rgba1.g = 0.0;
        rgba1.b = 0.0;
        colors.push_back(rgba1);

        std_msgs::ColorRGBA rgba2;
        rgba2.r = 0.0;
        rgba2.g = 1.0;
        rgba2.b = 0.0;
        colors.push_back(rgba2);

        std_msgs::ColorRGBA rgba3;
        rgba3.r = 0.0;
        rgba3.g = 0.0;
        rgba3.b = 1.0;
        colors.push_back(rgba3);

        std_msgs::ColorRGBA rgba4;
        rgba4.r = 1.0;
        rgba4.g = 1.0;
        rgba4.b = 0.0;
        colors.push_back(rgba4);

        std_msgs::ColorRGBA rgba5;
        rgba5.r = 1.0;
        rgba5.g = 0.0;
        rgba5.b = 1.0;
        colors.push_back(rgba5);

        std_msgs::ColorRGBA rgba6;
        rgba6.r = 0.0;
        rgba6.g = 1.0;
        rgba6.b = 1.0;
        colors.push_back(rgba6);

        std_msgs::ColorRGBA rgba7;
        rgba7.r = 0.5;
        rgba7.g = 0.0;
        rgba7.b = 0.8;
        colors.push_back(rgba7);

        // 创建一个删除所有Marker的Marker，并将其添加到MarkerArray中
        visualization_msgs::Marker delete_all;
        delete_all.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(delete_all);

        int id = 0;
        for (const auto &subregion : subregions_) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "paths";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::LINE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.1;
            marker.color = colors[id % 7];
            marker.color.a = 0.5;
            for (const auto &path : subregion->paths) {
                for (size_t i = 1; i < path.second.size(); i++) {
                    geometry_msgs::Point p1;
                    p1.x = path.second[i-1]->position.x();
                    p1.y = path.second[i-1]->position.y();
                    p1.z = id * 0.5;
                    marker.points.push_back(p1);

                    geometry_msgs::Point p2;
                    p2.x = path.second[i]->position.x();
                    p2.y = path.second[i]->position.y();
                    p2.z = id * 0.5;
                    marker.points.push_back(p2);
                }
            }
            markers.markers.push_back(marker);
        }

        for (const auto &path: robot2subregion_) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "paths";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::LINE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.1;
            marker.color.b = 0.0;
            marker.color.g = 0.0;
            marker.color.r = 0.0;
            marker.color.a = 0.5;
            for (size_t i = 1; i < path.size(); i++) {
                geometry_msgs::Point p1;
                p1.x = path[i-1]->position.x();
                p1.y = path[i-1]->position.y();
                p1.z = id * 0.5;
                marker.points.push_back(p1);

                geometry_msgs::Point p2;
                p2.x = path[i]->position.x();
                p2.y = path[i]->position.y();
                p2.z = id * 0.5;
                marker.points.push_back(p2);
            }
            markers.markers.push_back(marker);
        }
        // ROS_INFO("-------------------------------");

        return markers;
    }

    visualization_msgs::Marker SubregionManager::visualizeGlobalTour(const Eigen::Vector2d &sensor_pose) {
        // 使用line list，根据global tour，把子区域的质心连起来，形成一条路径
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "global_tour";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.3;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        geometry_msgs::Point p;
        p.x = sensor_pose.x();
        p.y = sensor_pose.y();
        p.z = subregion_height_ / 2;
        marker.points.push_back(p);

        if (!global_tour_.empty()) {
            p.x = global_tour_[0]->centroid.x();
            p.y = global_tour_[0]->centroid.y();
            p.z = subregion_height_ / 2;
            marker.points.push_back(p);

        }

        for (size_t i = 1; i < global_tour_.size(); i++) {
            geometry_msgs::Point p1;
            p1.x = global_tour_[i - 1]->centroid.x();
            p1.y = global_tour_[i - 1]->centroid.y();
            p1.z = subregion_height_ / 2;
            marker.points.push_back(p1);

            geometry_msgs::Point p2;
            p2.x = global_tour_[i]->centroid.x();
            p2.y = global_tour_[i]->centroid.y();
            p2.z = subregion_height_ / 2;
            marker.points.push_back(p2);
        }

        return marker;
    }

    // visualization_msgs::Marker SubregionManager::visualizeLocalBound() {
    //     // 使用CUBE类型的MarkerArray可视化局部边界
    //     // 使用LINE LIST发布2d局部边界
    //     visualization_msgs::Marker marker;
    //     marker.header.frame_id = "map";
    //     marker.header.stamp = ros::Time::now();
    //     marker.ns = "local_bound";
    //     marker.id = 0;
    //     marker.type = visualization_msgs::Marker::LINE_LIST;
    //     marker.action = visualization_msgs::Marker::ADD;
    //     marker.pose.orientation.w = 1.0;
    //     marker.scale.x = 0.1;
    //     marker.scale.y = 0.1;
    //     marker.color.r = 0.0;
    //     marker.color.g = 1.0;
    //     marker.color.b = 0.0;
    //     marker.color.a = 1.0;

    //     geometry_msgs::Point p1;
    //     p1.x = local_bound_.min_corner().get<0>();
    //     p1.y = local_bound_.min_corner().get<1>();
    //     p1.z = 0.0;

    //     geometry_msgs::Point p2;
    //     p2.x = local_bound_.max_corner().get<0>();
    //     p2.y = local_bound_.min_corner().get<1>();
    //     p2.z = 0.0;

    //     geometry_msgs::Point p3;
    //     p3.x = local_bound_.max_corner().get<0>();
    //     p3.y = local_bound_.max_corner().get<1>();
    //     p3.z = 0.0;

    //     geometry_msgs::Point p4;
    //     p4.x = local_bound_.min_corner().get<0>();
    //     p4.y = local_bound_.max_corner().get<1>();
    //     p4.z = 0.0;

    //     marker.points.push_back(p1);
    //     marker.points.push_back(p2);
    //     marker.points.push_back(p2);
    //     marker.points.push_back(p3);
    //     marker.points.push_back(p3);
    //     marker.points.push_back(p4);
    //     marker.points.push_back(p4);
    //     marker.points.push_back(p1);

    //     return marker;
    // }

} // namespace explorer_ns