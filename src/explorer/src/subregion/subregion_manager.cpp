#include <subregion/subregion_manager.h>

using namespace std;
namespace explorer_ns {
    SubregionManager::SubregionManager(const double &subregion_width, const double &subregion_length, const double known_thre):
        subregion_width_(subregion_width), subregion_length_(subregion_length), known_thre_(known_thre) {
        // 初始化子区域的高度
        subregion_height_ = 2.0;
    }

    std::vector<SubregionPtr> SubregionManager::divideSubregion(const SubregionPtr &subregion, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &viewpoint_tree) {
        // 先把该子区域分成四份
        std::vector<SubregionPtr> divided_cells;
        // 对于每个子区域，如果内部前沿点大于阈值，计算其前沿点，然后计算其质心
        for (size_t i = 0; i < 4; i++) {
            // 分割子区域
            Box sub_aabb = divideBox(subregion->aabb, i);
            SubregionPtr sub_cell = generateSubregion(sub_aabb.min_corner(), sub_aabb.max_corner(), subregion->level + 1);
            
            // 如果子区域内的前沿点数量大于阈值，计算相关信息
            sub_cell->view_points  = getViewPoint(sub_aabb, viewpoint_tree);
            if (!sub_cell->view_points.empty()) {
                calculateViewPointCentroid(sub_cell);
                calculateKnownVoxelRatio(sub_cell, ufo_map);
                divided_cells.push_back(sub_cell);
            }
            else {
                subregions_invalid_.push_back(sub_cell);
            }
        }

        return divided_cells;
    }

    bool SubregionManager::isBoundChanged(const ufo::map::OccupancyMapColor &ufo_map) {
        map_bound_ = getMapBound(ufo_map);

        double epsilon = 1e-6;
        // 根据坐标位置判断边界是否发生变化
        if (std::abs(map_bound_.min_corner().get<0>() - bound_x_min_) > epsilon ||
            std::abs(map_bound_.max_corner().get<0>() - bound_x_max_) > epsilon ||
            std::abs(map_bound_.min_corner().get<1>() - bound_y_min_) > epsilon ||
            std::abs(map_bound_.max_corner().get<1>() - bound_y_max_) > epsilon) {
            bound_x_min_ = map_bound_.min_corner().get<0>();
            bound_x_max_ = map_bound_.max_corner().get<0>();
            bound_y_min_ = map_bound_.min_corner().get<1>();
            bound_y_max_ = map_bound_.max_corner().get<1>();
            return true;
        }

        return false;
    }

    bool SubregionManager::isOverlap(const Box &aabb1, const Box &aabb2) {
        // 判断两个子区域是否重叠
        return boost::geometry::intersects(aabb1, aabb2);
    }

    void SubregionManager::initSubregion(const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &viewpoint_tree, const NodeRtree &graph) {
        subregions_.clear();
        subregions_invalid_.clear();

        // 计算地图边界可以划分成的最大等级
        double bound_length = bound_y_max_ - bound_y_min_;
        max_level_ = std::log2(bound_length / subregion_length_);
        // 以地图当前边界，生成初始的子区域
        SubregionPtr init_subregion = generateSubregion(map_bound_.min_corner(), map_bound_.max_corner(), 0);
        updateInfo(init_subregion, ufo_map, viewpoint_tree);

        // 划分子区域
        std::deque<SubregionPtr> queue;
        queue.push_back(init_subregion);
        while (!queue.empty()) {
            SubregionPtr current = queue.front();
            queue.pop_front();

            if (current->level < max_level_ && (current->known_ratio > known_thre_ || current->level == 0)) {
                // 划分子区域
                std::vector<SubregionPtr> divided_subregions = divideSubregion(current, ufo_map, viewpoint_tree);
                // 将新的子区域加入到队列中
                queue.insert(queue.end(), divided_subregions.begin(), divided_subregions.end());
            } 
            else {
                // 如果不需要进一步划分，将子区域添加到 subregions_ 中
                subregions_.push_back(current);
            }
        }

        // 对每个子区域，计算其到其他子区域的最短距离
        for (const auto &subregion: subregions_) {
            updateDistance(subregion, graph);
        }
    }

    void SubregionManager::updateSubregion(const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &viewpoint_tree, const NodeRtree &graph, const Box &map_update_bound) {
        updated_dist.clear();
        if (isBoundChanged(ufo_map)) {
            initSubregion(ufo_map, viewpoint_tree, graph);
        }
        else {
            // 1. 检查失效的子区域能不能加入到有效的子区域中
            for (auto it = subregions_invalid_.begin(); it != subregions_invalid_.end();) {
                if (isOverlap((*it)->aabb, map_update_bound)) {
                    updateInfo(*it, ufo_map, viewpoint_tree);
                }

                if (!(*it)->view_points.empty() && (!isOverlap((*it)->aabb, map_update_bound) || (*it)->level <= 1)) {
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
                    updateInfo(*it, ufo_map, viewpoint_tree);
                }

                if ((*it)->view_points.empty() || (isOverlap((*it)->aabb, map_update_bound) && (*it)->level > 1)) {
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
                if ((*it)->level >= max_level_ || (*it)->known_ratio <= known_thre_) {
                    // 只有与当前更新区域交叠的区域才需要更新距离
                    if (isOverlap((*it)->aabb, map_update_bound)) {
                        updateDistance(*it, graph);
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

                        if (current->level < max_level_ && current->known_ratio > known_thre_) {
                            // 划分子区域
                            std::vector<SubregionPtr> divided_subregions = divideSubregion(current, ufo_map, viewpoint_tree);
                            // 将新的子区域加入到队列中
                            queue.insert(queue.end(), divided_subregions.begin(), divided_subregions.end());
                        }
                        else {
                            updateDistance(current, graph);
                            subregions_.push_back(current);
                        }
                    }
                }
            }

            // todo
            for (const auto &subregion: subregions_) {
                deleteInvaildDistance(subregion);
            }
        }

        checkDistance();
    }

    void SubregionManager::updateInfo(const SubregionPtr &subregion, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &viewpoint_tree) {
        // 更新子区域的信息
        subregion->view_points = getViewPoint(subregion->aabb, viewpoint_tree);
        calculateViewPointCentroid(subregion);

        if (!subregion->view_points.empty()) {
            calculateKnownVoxelRatio(subregion, ufo_map);
        }
    }

    void SubregionManager::updateDistance(const SubregionPtr &subregion, const NodeRtree &graph) {
        // 更新子区域与其他子区域的距离
        // 也更新其他子区域与该子区域的距离
        for (const auto &other: subregions_) {
            if (subregion != other && 
                // 避免重复计算
                updated_dist.find(std::make_pair(other, subregion)) == updated_dist.end() && 
                updated_dist.find(std::make_pair(subregion, other)) == updated_dist.end()) {
                getSubregionDistance(graph, subregion, other);
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
    
    void SubregionManager::getGlobalTour(const Eigen::Vector2d &sensor_pose, const NodeRtree &graph) {
        // 使用TSP求解全局路径
        DataModel data;
        data.dist_matrix = getDistMatrix(sensor_pose, graph);
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

    double SubregionManager::getPathDistance(const Eigen::Vector2d &start, const Eigen::Vector2d &end, const NodeRtree &graph) {
        // 计算两个点之间的距离
        NodePtr start_node = getNearestNode(start, graph);
        NodePtr end_node = getNearestNode(end, graph);

        std::vector<NodePtr> path = aStar(start_node, end_node);
        double distance = 0.0;
        for (size_t i = 0; i < path.size() - 1; i++) {
            distance += (path[i]->position - path[i + 1]->position).norm();
        }

        return distance;
    }

    SubregionPtr SubregionManager::generateSubregion(const Point &min, const Point &max, const size_t &level) {
        // 生成子区域
        Box aabb(min, max);
        Eigen::Vector2d center = getSubregionCenter(aabb);
        
        return std::make_shared<Subregion>(aabb, center, level);
    }

    std::vector<NodePtr> SubregionManager::getViewPoint(const Box &box, const NodeRtree &viewpoint_tree) {
        // 获取子区域内的前沿点
        std::vector<Value> results;
        viewpoint_tree.query(boost::geometry::index::intersects(box), 
                            std::back_inserter(results));

        std::vector<NodePtr> view_points;
        for (const auto &result: results) {
            view_points.push_back(result.second);
        }

        return view_points;
    }

    std::vector<std::vector<int>> SubregionManager::getDistMatrix(const Eigen::Vector2d &sensor_pose, const NodeRtree &graph) {
        std::vector<std::vector<int>> dist_matrix(subregions_.size() + 1, std::vector<int>(subregions_.size() + 1, 0));
        subregion_index_.clear();

        // 先在矩阵的第一行和第一列填充机器人到各子区域的距离
        auto it = subregions_.begin();
        for (size_t i = 0; i < subregions_.size(); ++i, ++it) {
            dist_matrix[0][i + 1] = static_cast<int>(getPathDistance(sensor_pose, (*it)->centroid, graph) * 10);
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

    Box SubregionManager::divideBoxInHorizontal(const Box &aabb, const size_t &index) {
        // 水平划分子区域
        double x_min = aabb.min_corner().get<0>();
        double x_max = aabb.max_corner().get<0>();
        double y_min = aabb.min_corner().get<1>();
        double y_max = aabb.max_corner().get<1>();
        double x_mid = (x_min + x_max) / 2;
        double y_mid = (y_min + y_max) / 2;

        if (index == 0) {
            return Box(Point(x_min, y_min, 0.0), Point(x_mid, y_max, subregion_height_));
        }
        else {
            return Box(Point(x_mid, y_min, 0.0), Point(x_max, y_max, subregion_height_));
        }
    }

    Box SubregionManager::divideBoxInVertical(const Box &aabb, const size_t &index) {
        // 垂直划分子区域
        double x_min = aabb.min_corner().get<0>();
        double x_max = aabb.max_corner().get<0>();
        double y_min = aabb.min_corner().get<1>();
        double y_max = aabb.max_corner().get<1>();
        double x_mid = (x_min + x_max) / 2;
        double y_mid = (y_min + y_max) / 2;

        if (index == 0) {
            return Box(Point(x_min, y_min, 0.0), Point(x_max, y_mid, subregion_height_));
        }
        else {
            return Box(Point(x_min, y_mid, 0.0), Point(x_max, y_max, subregion_height_));
        }
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

    Box SubregionManager::getMapBound(const ufo::map::OccupancyMapColor &ufo_map) {
        // 获取地图的边界
        ufo::geometry::AABB known_bbx = ufo_map.getKnownBBX();
        ufo::map::Point3 min, max;
        min = known_bbx.getMin();
        max = known_bbx.getMax();

        // 取最大边长作为边界
        double max_length = std::max(max.x() - min.x(), max.y() - min.y());
        double x_min = min.x();
        double y_min = min.y();
        double x_max = x_min + max_length;
        double y_max = y_min + max_length;

        return Box(Point(x_min, y_min, 0.0), Point(x_max, y_max, subregion_height_));
    }

    void SubregionManager::calculateViewPointCentroid(const SubregionPtr &subregion) {
        Eigen::Vector2d centroid_last = subregion->centroid;
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

            subregion->centroid = Eigen::Vector2d(x, y);
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

        subregion->known_ratio = known_voxel / (known_voxel + unknown_voxel);
    }

    void SubregionManager::getSubregionDistance(const NodeRtree &graph, const SubregionPtr &subregion1, const SubregionPtr &subregion2) {
        // A-star 计算两个子区域的距离
        NodePtr start = getNearestNode(subregion1->centroid, graph);
        NodePtr end = getNearestNode(subregion2->centroid, graph);

        std::vector<NodePtr> path = aStar(start, end);
        double distance = 0.0;
        for (size_t i = 0; i < path.size() - 1; i++) {
            distance += (path[i]->position - path[i + 1]->position).norm();
        }

        subregion1->distances[subregion2] = distance;
        subregion2->distances[subregion1] = distance;

        // 记录已经更新的子区域之间的距离
        updated_dist.insert(std::make_pair(subregion1, subregion2));
        updated_dist.insert(std::make_pair(subregion2, subregion1));

        // for test
        subregion1->paths[subregion2] = path;
        subregion2->paths[subregion1] = path;
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
            // ROS_INFO("path_size: %zu", subregion->paths.size());
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

} // namespace explorer_ns