#include <graph/graph.h>

using namespace std;

namespace explorer_ns {
    Graph::Graph(const int max_neighbor_num, const double max_neighbor_distance, const double min_edge_distance,
                 const double max_edge_distance, const double sample_density, const double sensor_height,
                 const double vehicle_width, const double vehicle_length, const double inflate_radius) {
        // 初始化图
        max_neighbor_num_ = max_neighbor_num;
        max_neighbor_distance_ = max_neighbor_distance;
        min_edge_distance_ = min_edge_distance;
        max_edge_distance_ = max_edge_distance;
        sample_density_ = sample_density;
        sensor_height_ = sensor_height;
        vehicle_width_ = vehicle_width;
        vehicle_length_ = vehicle_length;
        inflate_radius_ = inflate_radius;

        graph_.clear();
    }

    void Graph::extendGraph(const ufo::map::OccupancyMapColor &ufo_map, const PointRtree &frontier_tree, 
                            const nav_msgs::OccupancyGridPtr &terrain_map, const Box &map_update_bound) {
        getSampleWindow(terrain_map, map_update_bound);
        size_t sample_num = (sample_bound_max_.x() - sample_bound_min_.x()) * (sample_bound_max_.y() - sample_bound_min_.y()) * sample_density_;
        
        for (size_t i = 0; i < sample_num; i++) {
            // 随机采样新的节点
            Eigen::Vector2d sample = biasSampleNode(frontier_tree, map_update_bound);
            samples_.push_back(sample);
            if (!graph_.empty()) {
                // 查询图中距离采样点最近的节点
                NodePtr closest_node = getCloestNodeInGraph(sample);
                if (closest_node) {
                    // 限制采样点与最近节点之间的距离
                    sample = AdjustDistToGraph(sample, closest_node);
                }
            }
            
            // 利用地形图检查采样点是否在可通行空间
            if (!inTraversableSpace(sample, terrain_map)) {
                continue;;
            }
            
            std::vector<NodePtr> neighbors;
            if (!graph_.empty()) {
                bool is_valid = true;
                // 检查采样点附近图中节点的数量，防止图中节点过于密集
                neighbors = getNeighborsInGraph(sample, max_neighbor_distance_);
                if (neighbors.size() >= max_neighbor_num_) {
                    continue;
                }

                // 检查采样点与邻居的距离，防止采样点过于密集
                for (const auto &neighbor: neighbors) {
                    double dist = (sample - neighbor->position).norm();
                    if (dist < min_edge_distance_) {
                        is_valid = false;
                        continue;
                    }
                }
                if (!is_valid) {
                    continue;
                }
            }

            // 添加新节点
            NodePtr new_node = std::make_shared<Node>(sample);
            // 设置节点的邻居
            neighbors = getNeighborsInGraph(sample, max_edge_distance_);
            for (const auto &neighbor: neighbors) {
                // 先进行碰撞检测
                ufo::map::Point3 start(new_node->position.x(), new_node->position.y(), 0.0);
                ufo::map::Point3 end(neighbor->position.x(), neighbor->position.y(), 0.0);
                if (isCollisionFree(ufo_map, start, end)) {
                    // 如果邻居的邻居的数量超过最大值，检查更新
                    if (neighbor->neighbors.size() >= max_neighbor_num_) {
                        // 根据邻居的邻居与邻居之间的距离进行排序，找出距离最远的邻居的邻居
                        std::vector<std::pair<double, NodePtr>> neighbor_neighbors;
                        for (const auto &neighbor_neighbor: neighbor->neighbors) {
                            double dist = (neighbor_neighbor->position - neighbor->position).squaredNorm();
                            neighbor_neighbors.push_back(std::make_pair(dist, neighbor_neighbor));
                        }
                        std::sort(neighbor_neighbors.begin(), neighbor_neighbors.end(), 
                                    [](const std::pair<double, NodePtr> &a, const std::pair<double, NodePtr> &b) {
                                        return a.first > b.first;});
                        // 将距离最远的邻居与当前节点的距离进行比较
                        if ((new_node->position - neighbor->position).squaredNorm() < neighbor_neighbors[0].first) {
                            // 删除最远的邻居
                            neighbor->deleteNeighbor(neighbor_neighbors[0].second);
                            neighbor_neighbors[0].second->deleteNeighbor(neighbor);
                            edges_.erase(makeEdge(neighbor, neighbor_neighbors[0].second));
                            // 添加邻居
                            new_node->addNeighbor(neighbor);
                            neighbor->addNeighbor(new_node);
                            edges_.insert(makeEdge(new_node, neighbor));
                        }
                    }
                    else {
                        // 添加邻居
                        new_node->addNeighbor(neighbor);
                        neighbor->addNeighbor(new_node);
                        edges_.insert(makeEdge(new_node, neighbor));
                    }
                }
                else {
                    deleted_edges_.insert(makeEdge(new_node, neighbor));
                }
            }
            
            if (new_node->neighbors.empty() && !graph_.empty()) {
                // 新节点没有邻居，
                continue;
            }
            
            // 将新节点添加到图中
            addNode(new_node);
        }
    }

    void Graph::updateGraph(const ufo::map::OccupancyMapColor &ufo_map, const PointRtree &frontier_tree, const Box &map_update_bound) {
        std::vector<Value> nodes = getGraphNodeInBox<NodeRtree>(graph_, map_update_bound);
        
        std::vector<NodePtr> affected_nodes;
        for (const auto node:nodes) {
            affected_nodes.push_back(node.second);
        }
        // 维护一个已经检测过的边的集合
        std::set<Edge> checked_edges;
        
        // 检查受影响的节点的邻居是否仍然是可通行的,或添加新的边
        for (const auto &node: affected_nodes) {
            std::vector<NodePtr> neighbors_tmp(node->neighbors.begin(),
                                               node->neighbors.end());
            for (const auto &neighbor: neighbors_tmp) {
                // 如果边已经检查过，跳过
                if (checked_edges.find(makeEdge(node, neighbor)) != checked_edges.end()) {
                    continue;
                }
                ufo::map::Point3 start(node->position.x(), node->position.y(), 0.0);
                ufo::map::Point3 end(neighbor->position.x(), neighbor->position.y(), 0.0);
                // 边没有检查过，添加到集合中
                checked_edges.insert(makeEdge(node, neighbor));

                if (!isCollisionFree(ufo_map, start, end)) {
                    // 删除边
                    neighbor->deleteNeighbor(node);
                    node->deleteNeighbor(neighbor);
                    edges_.erase(makeEdge(node, neighbor));
                    deleted_edges_.insert(makeEdge(node, neighbor));
                }

                if (neighbor->neighbors.empty()) {
                    // 删除节点
                    removeNode(neighbor);
                }
            }
            if (node->neighbors.empty()) {
                // 删除节点
                removeNode(node);
            }

            checked_edges.clear();
            if (node->neighbors.size() < max_neighbor_num_) {
                std::vector<NodePtr> neighbors_in_graph = getNeighborsInGraph(node->position, max_edge_distance_);
                for (const auto &neighbor: neighbors_in_graph) {
                    // 已经是邻居，跳过
                    if (edges_.find(makeEdge(node, neighbor)) != edges_.end()) {
                        continue;
                    }
                    // 如果边已经检查过，跳过
                    if (checked_edges.find(makeEdge(node, neighbor)) != checked_edges.end()) {
                        continue;
                    }
                    if (neighbor->neighbors.size() < max_neighbor_num_) {
                        ufo::map::Point3 start(node->position.x(), node->position.y(), 0.0);
                        ufo::map::Point3 end(neighbor->position.x(), neighbor->position.y(), 0.0);
                        checked_edges.insert(makeEdge(node, neighbor));

                        if (isCollisionFree(ufo_map, start, end)){
                            // 添加边
                            node->addNeighbor(neighbor);
                            neighbor->addNeighbor(node);
                            edges_.insert(makeEdge(node, neighbor));
                        }
                        else {
                            deleted_edges_.insert(makeEdge(node, neighbor));
                        }
                    }
                }
            }
        }
    }

    void Graph::getSampleWindow(const nav_msgs::OccupancyGridPtr &terrain_map, const Box &map_update_bound) {
        Eigen::Vector2d terrain_map_bound_min_ = {terrain_map->info.origin.position.x, terrain_map->info.origin.position.y};
        Eigen::Vector2d terrain_map_bound_max_ = {terrain_map->info.origin.position.x + terrain_map->info.width * terrain_map->info.resolution,
                                                  terrain_map->info.origin.position.y + terrain_map->info.height * terrain_map->info.resolution};

        sample_bound_min_.x() = std::max(map_update_bound.min_corner().get<0>(), terrain_map_bound_min_.x());
        sample_bound_min_.y() = std::max(map_update_bound.min_corner().get<1>(), terrain_map_bound_min_.y()); 
        sample_bound_max_.x() = std::min(map_update_bound.max_corner().get<0>(), terrain_map_bound_max_.x());
        sample_bound_max_.y() = std::min(map_update_bound.max_corner().get<1>(), terrain_map_bound_max_.y());
        sample_bound_center_ = (sample_bound_min_ + sample_bound_max_) / 2;
    }

    Eigen::Vector2d Graph::biasSampleNode(const PointRtree &frontier_tree, const Box &map_update_bound) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dis(0.0, 1.0);
        Eigen::Vector2d sample;
        double total_prob = 1.0;

        // 各区域内前沿点的数量
        std::vector<size_t> region_frontiers(4, 0);
        // 采样概率
        std::vector<double> region_probs(region_frontiers.size(), 0.0);
        // 在采样窗口内的前沿点
        std::vector<Point> range_frontiers;
        range_frontiers = getFrontiersInRange(frontier_tree, map_update_bound);
        
        // 统计各区域内前沿点的数量
        for (const auto &frontier: range_frontiers) {
            // 计算前沿点与区域中心连线和x轴的夹角
            double theta = atan2(frontier.get<1>() - sample_bound_center_.y(), frontier.get<0>() - sample_bound_center_.x());
            if (theta > 0 && theta <= M_PI_2) {
                region_frontiers[0]++;
            } else if (theta > M_PI_2 && theta <= M_PI) {
                region_frontiers[1]++;
            } else if (theta > -M_PI && theta <= -M_PI_2) {
                region_frontiers[2]++;
            } else if (theta > -M_PI_2 && theta <= 0) {
                region_frontiers[3]++;
            }
        }

        double total = region_frontiers[0] + region_frontiers[1] + region_frontiers[2] + region_frontiers[3];
        if (total == 0.0) {
            // 如果没有前沿点，直接在采样窗口内随机采样
            double x = dis(gen) * (sample_bound_max_.x() - sample_bound_min_.x()) + sample_bound_min_.x();
            double y = dis(gen) * (sample_bound_max_.y() - sample_bound_min_.y()) + sample_bound_min_.y();
            sample = Eigen::Vector2d(x, y);

            return sample;
        }
        else {
            // 执行偏执采样
            for (int i = 0; i < region_frontiers.size(); i++) {
                if (region_frontiers[i] == 0) {
                    region_probs[i] = 0.1;
                    total_prob -= 0.1;
                }
            }

            for (int i = 0; i < region_frontiers.size(); i++) {
                if (region_frontiers[i] != 0) {
                    region_probs[i] = region_frontiers[i] / total * total_prob;
                }
            }

            // 调整概率，确保每个区域的概率至少为0.1
            for (int i = 0; i < region_probs.size(); i++) {
                if (region_probs[i] < 0.1) {
                    // 找出概率最大的区域
                    auto max_iter = std::max_element(region_probs.begin(), region_probs.end());
                    // 从概率最大的区域中取出一些概率
                    *max_iter -= (0.1 - region_probs[i]);
                    // 将当前区域的概率设置为0.1
                    region_probs[i] = 0.1;
                }
            }
    
            // 使用上述概率，执行采样
            double bias_prob = dis(gen);
            // 在偏置采样的基础上，根据采样概率采样
            if (bias_prob <= region_probs[0]) {
                // 采样右上侧区域
                double x = sample_bound_center_.x() + (sample_bound_max_.x() - sample_bound_min_.x()) / 2 * dis(gen);
                double y = sample_bound_center_.y() + (sample_bound_max_.y() - sample_bound_min_.y()) / 2 * dis(gen);
                sample = Eigen::Vector2d(x, y);
            } 
            else if (bias_prob <= region_probs[0] + region_probs[1]) {
                // 采样左上方区域
                double x = sample_bound_center_.x() - (sample_bound_max_.x() - sample_bound_min_.x()) / 2 * dis(gen);
                double y = sample_bound_center_.y() + (sample_bound_max_.y() - sample_bound_min_.y()) / 2 * dis(gen);
                sample = Eigen::Vector2d(x, y);
            } 
            else if (bias_prob <= region_probs[0] + region_probs[1] + region_probs[2]) {
                // 采样左下侧区域
                double x = sample_bound_center_.x() - (sample_bound_max_.x() - sample_bound_min_.x()) / 2 * dis(gen);
                double y = sample_bound_center_.y() - (sample_bound_max_.y() - sample_bound_min_.y()) / 2 * dis(gen);
                sample = Eigen::Vector2d(x, y);
            } 
            else if (bias_prob <= 1.0){
                // 采样右下方区域
                double x = sample_bound_center_.x() + (sample_bound_max_.x() - sample_bound_min_.x()) / 2 * dis(gen);
                double y = sample_bound_center_.y() - (sample_bound_max_.y() - sample_bound_min_.y()) / 2 * dis(gen);
                sample = Eigen::Vector2d(x, y);
            }

            return sample;
        }
    }

    std::vector<Point> Graph::getFrontiersInRange(const PointRtree &frontier_tree, const Box &map_update_bound) {
        // 获取范围内的所有前沿点
        std::vector<Point> range_frontiers = getGraphNodeInBox<PointRtree>(frontier_tree, map_update_bound);

        return range_frontiers;
    }
    
    template <typename Rtree>
    std::vector<typename Rtree::value_type> Graph::getGraphNodeInBox(const Rtree &rtree, const Box &box) {
        std::vector<typename Rtree::value_type> result;
        rtree.query(boost::geometry::index::intersects(box), std::back_inserter(result));
        return result;
    }

    NodePtr Graph::getCloestNodeInGraph(const Eigen::Vector2d &sample) {
        std::vector<Value> result;
        Point query_point(sample[0], sample[1], 0.0);
        graph_.query(boost::geometry::index::nearest(query_point, 1), std::back_inserter(result));

        if (result.size() != 1) {
            return nullptr;
        } else {
            return result[0].second;
        }
    }

    Eigen::Vector2d Graph::AdjustDistToGraph(const Eigen::Vector2d &sample, const NodePtr &node) {
        Eigen::Vector2d closer_sample;
        Eigen::Vector2d direction = sample - node->position;

        double distance = direction.norm();
        if (distance > max_edge_distance_) {
            closer_sample = node->position + direction / distance * max_edge_distance_;
        } 
        else if (distance < min_edge_distance_) {
            closer_sample = node->position + direction / distance * min_edge_distance_ ;
        }
        else {
            closer_sample = sample;
        }

        return closer_sample;
    }

    std::vector<NodePtr> Graph::getNeighborsInGraph(const Eigen::Vector2d &sample, const double &radius) {
        // 在graph中查找距离sample小于radius的节点
        std::vector<Value> result;
        Point query_point(sample[0], sample[1], 0.0);
        graph_.query(boost::geometry::index::satisfies([&](const Value &value) {
            return boost::geometry::distance(value.first, query_point) <= radius;
        }), std::back_inserter(result));

        std::vector<NodePtr> neighbors;
        for (const auto &node_value: result) {
            if (node_value.second->position == sample || (node_value.second->position - sample).norm() < 1e-6){
                continue;
            }
            neighbors.push_back(node_value.second);
        }

        return neighbors;
    }

    bool Graph::inTraversableSpace(const Eigen::Vector2d &sample, const nav_msgs::OccupancyGridPtr &terrain_map) {
        int ind_x = int((sample.x() - terrain_map->info.origin.position.x) / terrain_map->info.resolution);
        int ind_y = int((sample.y() - terrain_map->info.origin.position.y) / terrain_map->info.resolution);

        if (ind_x < 0 || ind_x >= terrain_map->info.width || ind_y < 0 || ind_y >= terrain_map->info.height) {
            return false;
        }

        int x_num = std::ceil((vehicle_length_ + inflate_radius_)/ 2 / terrain_map->info.resolution);
        int y_num = std::ceil((vehicle_width_ + inflate_radius_) / 2 / terrain_map->info.resolution);
        int total = (2 * x_num + 1) * (2 * y_num + 1);

        // 判断邻域内未知或占用栅格的数量
        int unknown_count = 0;
        int occupied_count = 0;
        int free_count = 0;
        for (int m = -x_num; m <= x_num; m++) {
            for (int n = -y_num; n <= y_num; n++) {
                if (m == 0 && n == 0) {
                    continue;
                }

                int tmp_ind_x = ind_x + m;
                int tmp_ind_y = ind_y + n;
                if (tmp_ind_x > 0 && tmp_ind_x < terrain_map->info.width && 
                    tmp_ind_y > 0 && tmp_ind_y < terrain_map->info.height) {
                    double tmp_grid_data = terrain_map->data[tmp_ind_x + tmp_ind_y * terrain_map->info.width];
                    if (tmp_grid_data == 100) {
                        occupied_count++;
                    }
                    else if (tmp_grid_data == -1) {
                        unknown_count++;
                    }
                    else if (tmp_grid_data == 0) {
                        free_count++;
                    }
                }
            }
        }
        if (occupied_count != 0 || unknown_count >= int(0.4 * total) || free_count <= int(0.2 * total)) {
            return false;
        } else {return true;}
    }

    bool Graph::isFrontierVisible(const ufo::map::OccupancyMapColor &ufo_map, const PointRtree &frontier_tree, const Eigen::Vector2d &sample) {
        // 在树中搜索距离sample最近的前沿点
        std::vector<Point> result;
        double visible_raius_ = 8.0;
        Point query_point(sample[0], sample[1], 0.0);
        frontier_tree.query(boost::geometry::index::satisfies([&](const Point &point) {
            return boost::geometry::distance(point, query_point) < visible_raius_;
        }), std::back_inserter(result));
        
        if (!result.empty()) {
            // 检查采样点是否能观察到一定数量的前沿点
            int count = 0;
            for (const auto &frontier: result) {
                    ufo::map::Point3 start(sample.x(), sample.y(), sensor_height_);
                    ufo::map::Point3 end(frontier.get<0>(), frontier.get<1>(), frontier.get<2>());
                    if (ufo_map.isCollisionFree(start, end, true)) {
                        count++;
                    }
                }
            if (count >= 3) {
                return true;
            } else {return false;}
        } else {return false;}
    }

    bool Graph::isCollisionFree(const ufo::map::OccupancyMapColor &ufo_map, const ufo::map::Point3 &start, const ufo::map::Point3 &end) {
        ufo::map::Point3 direction = end - start;
        ufo::map::Point3 center = start + (direction / 2.0);
        double distance = direction.norm();
        direction /= distance;

        double yaw = -atan2(direction.y(), direction.x());
        double pitch = 0;
        double roll = 0;

        ufo::geometry::OBB obb(center, ufo::math::Vector3(distance / 2.0, inflate_radius_, sensor_height_), ufo::math::Quaternion(roll, pitch, yaw));
        for (auto it = ufo_map.beginLeaves(obb, true, false, false, 0), it_end = ufo_map.endLeaves(); it != it_end; ++it) {
            return false;
        }

        return true;
    }

    void Graph::addNode(const NodePtr &node) {
        Value node_value = std::make_pair(Point(node->position.x(), node->position.y(), 0.0), node);
        graph_.insert(node_value);
    }

    void Graph::removeNode(const NodePtr &node) {
        Value node_value = std::make_pair(Point(node->position.x(), node->position.y(), 0.0), node);
        graph_.remove(node_value);
    }

    visualization_msgs::Marker Graph::visualizeGraph() {
        // visualization_msgs::MarkerArray markers;
        // int nodeCount = 0;
        // for (const auto &node_value : graph_) {
        //     visualization_msgs::Marker marker;
        //     marker.header.frame_id = "map";
        //     marker.header.stamp = ros::Time::now();
        //     marker.ns = "graph";
        //     marker.id = nodeCount;
        //     marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        //     marker.action = visualization_msgs::Marker::ADD;
        //     marker.pose.position.x = node_value.second->position.x();
        //     marker.pose.position.y = node_value.second->position.y();
        //     marker.pose.position.z = 0.0;
        //     marker.pose.orientation.w = 1.0;
        //     marker.scale.z = 0.5;
        //     marker.color.r = 1.0;
        //     marker.color.g = 1.0;
        //     marker.color.b = 1.0;
        //     marker.color.a = 1.0;

        //     marker.text = std::to_string(++nodeCount);
        //     markers.markers.push_back(marker);
        // }
        // 可视化图
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "graph";
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 0.5;
        marker.color.g = 0.7;
        marker.color.b = 0.7;
        marker.color.a = 0.6;

        // 对graph中的节点可视化
        for (const auto &node_value: graph_) {
            geometry_msgs::Point p;
            p.x = node_value.first.get<0>();
            p.y = node_value.first.get<1>();
            p.z = 0.0;
            marker.points.push_back(p);
        }
        return marker;
    }

    visualization_msgs::Marker Graph::visualizeEdge() {
        // 将边可视化
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "graph";
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 0.6;

        // 对graph中的边可视化
        for (const auto &edge: edges_) {
            geometry_msgs::Point p1;
            p1.x = edge.first->position.x();
            p1.y = edge.first->position.y();
            p1.z = 0.0;
            marker.points.push_back(p1);

            geometry_msgs::Point p2;
            p2.x = edge.second->position.x();
            p2.y = edge.second->position.y();
            p2.z = 0.0;
            marker.points.push_back(p2);
        }

        return marker;
    }

    visualization_msgs::Marker Graph::visualizeSamples() {
        // 可视化采样点
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "graph";
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.g = 1.0;
        marker.color.a = 1.0;

        // 对采样点可视化
        for (const auto &sample: samples_) {
            geometry_msgs::Point p;
            p.x = sample.x();
            p.y = sample.y();
            p.z = 0.0;
            marker.points.push_back(p);
        }

        return marker;
    }

    Edge Graph::makeEdge(const NodePtr &node1, const NodePtr &node2) {
        Edge edge;
        if (node1->position.squaredNorm() <= node2->position.squaredNorm()) {
            edge = std::make_pair(node1, node2);
        } else {
            edge = std::make_pair(node2, node1);
        }

        return edge;
    }

    // 用BFS检查图是否是连通的
    bool Graph::isConnected() {
        if (graph_.empty()) {
            return true;
        }

        std::unordered_set<NodePtr> visited;
        std::queue<NodePtr> queue;

        queue.push(graph_.begin()->second);
        visited.insert(graph_.begin()->second);

        while (!queue.empty()) {
            NodePtr current = queue.front();
            queue.pop();

            for (const auto &neighbor: current->neighbors) {
                if (visited.find(neighbor) == visited.end()) {
                    queue.push(neighbor);
                    visited.insert(neighbor);
                }
            }
        }

        return visited.size() == graph_.size();
    }
}


