#include <viewpoint/viewpoint_manager.h>

using namespace std;

namespace explorer_ns {
    void ViewpointManager::generateViewPoint(const ufo::map::OccupancyMapColor &ufo_map, const PointRtree &frontier_tree, const NodeRtree &graph) {
        viewpoint_frontiers_.clear();
        
        generateSphere(ufo_map, frontier_tree);
        // 根据sphere大小顺序，依次寻找视点
        for (const auto &sphere: sphere_list) {
            // 应当过滤一些球，例如球内的前沿点数量小于阈值
            if (frontiers_in_sphere_list[sphere.second].size() <= filter_thres_) {
                continue;
            }
            Point centroid = getFrontierCentroid(frontiers_in_sphere_list[sphere.second]);
            // 寻找距离球心最近的节点
            NodePtr nearest_node = getNearestNodeVisible(ufo_map, sphere.second, graph);
            if (nearest_node == nullptr) {
                continue;
            }
            viewpoint_frontiers_[nearest_node].insert(viewpoint_frontiers_[nearest_node].end(), 
                                                      frontiers_in_sphere_list[sphere.second].begin(), 
                                                      frontiers_in_sphere_list[sphere.second].end());
        }

        viewpoint_tree_.clear();
        for (const auto &pair: viewpoint_frontiers_) {
            Value tmp = std::make_pair(Point(pair.first->position.x(), pair.first->position.y(), 0.0), pair.first);
            viewpoint_tree_.insert(tmp);
        }
    }

    Point ViewpointManager::getFrontierCentroid(const std::vector<Point> &frontiers) {
        double x = 0.0, y = 0.0, z = 0.0;
        for (const auto &frontier: frontiers) {
            x += frontier.get<0>();
            y += frontier.get<1>();
            z += frontier.get<2>();
        }

        return Point(x / frontiers.size(), y / frontiers.size(), z / frontiers.size());
    }

    void ViewpointManager::generateSphere(const ufo::map::OccupancyMapColor &ufo_map, const PointRtree &frontier_tree) {
        sphere_list.clear();
        frontiers_in_sphere_list.clear();

        // 遍历所有的前沿点中心，找到最大半径的球
        Point max_sphere_center;
        PointRtree frontier_tree_tmp = frontier_tree;
        while (!frontier_tree_tmp.empty()) {
            double max_radius = 0.0;
            // cout<<"frontier_tree_tmp.size():"<<frontier_tree_tmp.size()<<endl;
            for (const auto &frontier_point: frontier_tree_tmp) {
                // 对每一个前沿点，找到最大半径的球（不接触到障碍物）
                ufo::map::Point3 center(frontier_point.get<0>(), frontier_point.get<1>(), frontier_point.get<2>());
                // 找到对于center来说距离最近的障碍物体素
                for (auto it = ufo_map.beginNNLeaves(center, true, false, false, false, 0), it_end = ufo_map.endNNLeaves(); it != it_end; ++it) {
                    double radius = std::sqrt((it.getX() - center.x()) * (it.getX() - center.x()) + 
                                                (it.getY() - center.y()) * (it.getY() - center.y()) +
                                                (it.getZ() - center.z()) * (it.getZ() - center.z()));
                    if (radius > max_radius) {
                        max_sphere_center = Point(center.x(), center.y(), center.z());
                        if (radius > visible_radius_ / 2) {
                            max_radius = visible_radius_ / 2;
                        } else {
                            max_radius = radius;
                        }
                    }

                    break;
                }
            }

            // 已经在前沿点树中，找到了半径最大得球，现在我们需要找到这个球中的所有前沿点
            std::vector<Point> values;
            auto pred = boost::geometry::index::satisfies([&](const Point& point){
                return boost::geometry::distance(point, max_sphere_center) < max_radius;
            });
            frontier_tree_tmp.query(pred, std::back_inserter(values));
            // 存储该球的半径和中心
            sphere_list.insert(std::make_pair(max_radius, max_sphere_center));
            // 将这些前沿点放入到frontiers_in_sphere_list中
            frontiers_in_sphere_list[max_sphere_center] = values;
            // 从前沿点树中删除这些前沿点
            for (const auto &point: values) {
                frontier_tree_tmp.remove(point);
            }
        }
    }

    bool ViewpointManager::isVisible(const ufo::map::OccupancyMapColor &ufo_map, const NodePtr &node, const Point &frontier) {
        ufo::map::Point3 start(node->position.x(), node->position.y(), buffer_height_);
        ufo::map::Point3 end(frontier.get<0>(), frontier.get<1>(), frontier.get<2>());
        
        return ((end - start).norm() < visible_radius_ && ufo_map.isCollisionFree(start, end, true));
    }

    std::vector<NodePtr> ViewpointManager::getLocalPath(const double &sensor_orient, const Eigen::Vector2d &sensor_pose, const std::vector<SubregionPtr> &global_tour, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph) {
        if (global_tour.empty()) {
            return {};
        }

        // 先把global tour中第一个子区域的视点加入到viewpoints中
        std::vector<NodePtr> viewpoints;
        viewpoints.insert(viewpoints.end(), global_tour[0]->view_points.begin(), global_tour[0]->view_points.end());
        size_t subregion_tour_num = 1;

        // 把剩余所有与机器人位置距离小于local_path_generate_radius_的视点都加入到viewpoints中
        for (size_t i = 1; i < global_tour.size(); i++) {
            if ((global_tour[i]->centroid - sensor_pose).norm() < add_global_tour_to_local_plan_radius_) {
                viewpoints.insert(viewpoints.end(), global_tour[i]->view_points.begin(), global_tour[i]->view_points.end());
            }
            else {
                subregion_tour_num = i;
                break;
            }
        }

        DataModel data;
        data.dist_matrix = getViewpointDistMatrix(sensor_orient, sensor_pose, viewpoints, ufo_map, graph);
        TSPSolver tsp_solver(data);

        if (subregion_tour_num > 1) {
            // 根据globaltour中的子区域访问顺序，设置约束条件
            // for (size_t i = 1; i <= global_tour[0]->view_points.size(); i++) {
            //     tsp_solver.setConstraint(0, i);
            // }
            size_t offset = 1;
            for (size_t i = 0; i < subregion_tour_num - 1; i++) {
                offset += global_tour[i]->view_points.size();
                for (size_t j = 0; j < global_tour[i]->view_points.size(); j++) {
                    for (size_t k = 0; k < global_tour[i+1]->view_points.size(); k++) {
                        tsp_solver.setConstraint(j + (offset - global_tour[i]->view_points.size()), k + offset);
                    }
                }
            }
        }
        tsp_solver.solve();
        // tsp_solver.printSolution();
        
        std::vector<int> solution_index = tsp_solver.getSolutionIndex();
        local_path_.clear();
        for (size_t i = 0; i < solution_index.size(); i++) {
            if (solution_index[i] == 0) {
                continue;
            }

            local_path_.push_back(viewpoints[solution_index[i] - 1]);
        }

        getExcutedPath(sensor_orient, sensor_pose, ufo_map, graph);

        return excuted_path_;
    }

    std::vector<NodePtr> ViewpointManager::getGoalViewPoint(const double &sensor_orient, const Eigen::Vector2d &sensor_pose, const std::vector<SubregionPtr> &global_tour, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph) {  
        // if (global_tour.empty()) {
        //     return {};
        // }
        
        NodePtr candidate_viewpoint;
        std::vector<NodePtr> candidate_viewpoints;
        for (const auto &pair: viewpoint_frontiers_) {
            candidate_viewpoints.push_back(pair.first);
        }

        cout<<"candidate_viewpoints.size():"<<candidate_viewpoints.size()<<endl;

        double min_revenue = std::numeric_limits<double>::max();
        for (const auto &viewpoint: candidate_viewpoints) {
            double dist = getPathDist(std::make_shared<Node>(sensor_pose), viewpoint, ufo_map, graph);
            double revenue = dist;
            if (revenue < min_revenue) {
                min_revenue = revenue;
                candidate_viewpoint = viewpoint;
            }
        }

        excuted_path_ = getPath(std::make_shared<Node>(sensor_pose), candidate_viewpoint, ufo_map, graph);

        // if (previous_viewpoint_goal != nullptr && previous_viewpoint_exist) {
        //     if (calculateTheta(sensor_orient, sensor_pose, candidate_viewpoint) < calculateTheta(sensor_orient, sensor_pose, previous_viewpoint_goal)) {
        //         excuted_path_ = getPath(std::make_shared<Node>(sensor_pose), candidate_viewpoint, ufo_map, graph);
        //         previous_viewpoint_goal = candidate_viewpoint;
        //     }
        //     else {
        //         excuted_path_ = getPath(std::make_shared<Node>(sensor_pose), previous_viewpoint_goal, ufo_map, graph);
        //     }
        // }
        // else {
        //     excuted_path_ = getPath(std::make_shared<Node>(sensor_pose), candidate_viewpoint, ufo_map, graph);
        //     previous_viewpoint_goal = candidate_viewpoint;
        // }
        
        return excuted_path_;
    }

    double ViewpointManager::calculateTheta(const double &sensor_orient, const Eigen::Vector2d &sensor_pose, const NodePtr &viewpoint) {
        // 计算机器人当前朝向与子区域中心连线的夹角
        Eigen::Vector2d direction = viewpoint->position - sensor_pose;
        double beta = std::atan2(direction.y(), direction.x());

        // beta的角度范围是[-pi, pi]
        // sensor_orient的角度范围是[-pi, pi]
        double orientation_diff = std::abs(sensor_orient - beta);
        if (orientation_diff > M_PI) {
            orientation_diff -= M_PI;
        }

        return orientation_diff;
    }

    std::vector<std::vector<int>> ViewpointManager::getViewpointDistMatrix(const double &sensor_orient, const Eigen::Vector2d &sensor_pose, const std::vector<NodePtr> &viewpoints, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph) {
        std::vector<std::vector<int>> dist_matrix(viewpoints.size() + 1, std::vector<int>(viewpoints.size() + 1, 0));
        robot_to_viewpoint_path_.clear();
        // 先在矩阵的第一行和第一列填充机器人到视点的距离
        for (size_t i = 0; i < viewpoints.size(); i++) {
            NodePtr robot_node = std::make_shared<Node>(sensor_pose);
            std::vector<NodePtr> path = getPath(robot_node, viewpoints[i], ufo_map, graph);
            robot_to_viewpoint_path_[viewpoints[i]] = path;
            // dist_matrix[0][i + 1] = static_cast<int>((getPathDist(path) + 3 * calculateTheta(sensor_orient, sensor_pose, viewpoints[i])) * 10);
            dist_matrix[0][i + 1] = static_cast<int>(((sensor_pose - viewpoints[i]->position).norm() + 3 * calculateTheta(sensor_orient, sensor_pose, viewpoints[i])) * 10);
        }

        // 再填充视点之间的距离
        for (size_t i = 0; i < viewpoints.size(); i++) {
            for (size_t j = 0; j < viewpoints.size(); j++) {
                if (i != j) {
                    // dist_matrix[i+1][j+1] = static_cast<int>(getPathDist(viewpoints[i], viewpoints[j], ufo_map, graph) * 10);
                    dist_matrix[i+1][j+1] = static_cast<int>(((viewpoints[i]->position - viewpoints[j]->position).norm()) * 10);
                }
            }
        }

        return dist_matrix;
    }

    std::vector<NodePtr> ViewpointManager::getPath(const NodePtr &start, const NodePtr &end, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph) {
        NodePtr start_node = getNearestNode(start->position, graph);
        NodePtr end_node = getNearestNode(end->position, graph);
        
        if (start_node == nullptr || end_node == nullptr) {
            return {std::make_shared<Node>(start->position), std::make_shared<Node>(end->position)};
        }
        std::vector<NodePtr> path = aStar(start_node, end_node);

        return path;
    }

    double ViewpointManager::getPathDist(const NodePtr &start, const NodePtr &end, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph) {
        std::vector<NodePtr> path = getPath(start, end, ufo_map, graph);
        double distance = 0.0;
        for (size_t i = 0; i < path.size() - 1; i++) {
            distance += (path[i]->position - path[i + 1]->position).norm();
        }

        return distance;
    }

    double ViewpointManager::getPathDist(const std::vector<NodePtr> &path) {
        double distance = 0.0;
        for (size_t i = 0; i < path.size() - 1; i++) {
            distance += (path[i]->position - path[i + 1]->position).norm();
        }

        return distance;
    }

    NodePtr ViewpointManager::getNearestNode(const Eigen::Vector2d &position, const NodeRtree &graph) {
        std::vector<Value> result;
        Point query_point(position.x(), position.y(), 0.0);
        graph.query(boost::geometry::index::nearest(query_point, 1), std::back_inserter(result));

        if (result.empty()) {
            return nullptr;
        }

        return result[0].second;
    }

    NodePtr ViewpointManager::getNearestNodeVisible(const ufo::map::OccupancyMapColor &ufo_map, const Point &frontier, const NodeRtree &graph) {
        std::vector<Value> result;
        auto pred = boost::geometry::index::satisfies([&](const Value& value){
                    return isVisible(ufo_map, value.second, frontier);
                 });         
        graph.query(pred, std::back_inserter(result));

        std::sort(result.begin(), result.end(), [&](const Value& a, const Value& b){
            return boost::geometry::distance(a.first, frontier) < boost::geometry::distance(b.first, frontier);
        });

        if (result.empty()) {
            return nullptr;
        }

        return result[0].second;
    }

    std::vector<NodePtr> ViewpointManager::getKNearestViewpoint(const Eigen::Vector2d &position) {
        std::vector<Value> result;
        viewpoint_tree_.query(boost::geometry::index::satisfies([&](const Value& value){
                    return (value.second->position - position).norm() < search_viewpoint_radius_;
                 }), std::back_inserter(result));

        std::vector<NodePtr> k_nearest_nodes;
        for (const auto &value: result) {
            k_nearest_nodes.push_back(value.second);
        }

        return k_nearest_nodes;
    }

    std::vector<NodePtr> ViewpointManager::getExcutedPath(const double &sensor_orient, const Eigen::Vector2d &sensor_pose, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph) {
        std::vector<NodePtr> path;
        // 根据local_path_中的路径片段 从使用A*算法得到的路径中提取出整个路径
        NodePtr target_node = local_path_[0];
        NodePtr robot_node = std::make_shared<Node>(sensor_pose);

        if (robot_to_viewpoint_path_.find(target_node)!=robot_to_viewpoint_path_.end()) {
            path = robot_to_viewpoint_path_[target_node];
        }
        else {
            path = getPath(robot_node, target_node, ufo_map, graph);
            robot_to_viewpoint_path_[target_node] = path;
        }
        double dist = getPathDist(path);

        // if (dist > check_path_dist_thres_) {
        //     // 路径偏长，需要做进一步检查
        //     std::vector<NodePtr> k_nearest_nodes = getKNearestViewpoint(sensor_pose);
        //     std::multimap<double, NodePtr> dist_map;
        //     for (const auto &viewpoint: k_nearest_nodes) {
        //         if (robot_to_viewpoint_path_.find(viewpoint)!= robot_to_viewpoint_path_.end()) {
        //             dist_map.insert(std::make_pair(getPathDist(robot_to_viewpoint_path_[viewpoint]), viewpoint));
        //         }
        //         else {
        //             std::vector<NodePtr> path_tmp = getPath(robot_node, viewpoint, ufo_map, graph);
        //             robot_to_viewpoint_path_[viewpoint] = path_tmp;
        //             dist_map.insert(std::make_pair(getPathDist(path_tmp), viewpoint));
        //         }
        //     }

        //     if (!dist_map.empty()) {
        //         NodePtr closest_viewpoint = dist_map.begin()->second;
        //         double dist_to_closest_viewpoint = dist_map.begin()->first;
        //         if (dist / dist_to_closest_viewpoint > dist_ratio_thre_) {
        //             // 更换目标节点
        //             ROS_INFO("Change target node!");
        //             target_node = closest_viewpoint;
        //             path = robot_to_viewpoint_path_[target_node];
        //         }
        //     }
        // }

        // 主要用于防止A*路径跳变
        // 如果目标节点几乎没有变化，而且新路径的朝向反而更差了（导致机器人调头），那么我们就不更新路径
        if (previous_target_node != nullptr && (previous_target_node->position - target_node->position).norm() < 0.1) {
             if (calculateTheta(sensor_orient, sensor_pose, path[1]) < calculateTheta(sensor_orient, sensor_pose, excuted_path_[1])) {
                excuted_path_ = path;
                previous_target_node = target_node;
             }
             else {
                previous_target_node = target_node;
             }
        }
        else {
            excuted_path_ = path;
            previous_target_node = target_node;
        }

        return excuted_path_;
    }

    std::vector<NodePtr> ViewpointManager::aStar(const NodePtr &start, const NodePtr &end) {
        std::vector<NodePtr> path;
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
                return path;
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
        // ROS_WARN("No path found in viewpoint!");

        return path;
    }

    visualization_msgs::Marker ViewpointManager::visualizeViewPoint() {
        // 将viewpoint_frontiers中的视点转换成marker消息发布
        visualization_msgs::Marker view_point_markers;
        view_point_markers.header.frame_id = "map";
        view_point_markers.ns = "view_points";
        view_point_markers.type = visualization_msgs::Marker::SPHERE_LIST;
        view_point_markers.action = visualization_msgs::Marker::ADD;
        view_point_markers.scale.x = 0.4;
        view_point_markers.scale.y = 0.4;
        view_point_markers.scale.z = 0.4;
        view_point_markers.color.a = 1.0;
        view_point_markers.color.r = 1.0;
        view_point_markers.color.g = 0;
        view_point_markers.color.b = 0;
        view_point_markers.pose.orientation.w = 1;

        for (const auto &pair: viewpoint_frontiers_) {
            geometry_msgs::Point center;
            center.x = pair.first->position.x();
            center.y = pair.first->position.y();
            center.z = 0;
            view_point_markers.points.push_back(center);
        }

        return view_point_markers;
    }

    visualization_msgs::Marker ViewpointManager::visualizeViewPointLine() {
        // 将frontier_viewpoint_pairs_中视点和前沿的连线转换成marker消息发布
        visualization_msgs::Marker view_point_line_markers;
        view_point_line_markers.header.frame_id = "map";
        view_point_line_markers.ns = "view_points";
        view_point_line_markers.type = visualization_msgs::Marker::LINE_LIST;
        view_point_line_markers.action = visualization_msgs::Marker::ADD;
        view_point_line_markers.scale.x = 0.02;
        view_point_line_markers.color.a = 1.0;
        view_point_line_markers.color.r = 0.5;
        view_point_line_markers.color.g = 0.5;
        view_point_line_markers.color.b = 0.5;
        view_point_line_markers.pose.orientation.w = 1;

        // viewpoint_frontiers_
        for (auto it = viewpoint_frontiers_.begin(); it != viewpoint_frontiers_.end(); it++) {
            NodePtr viewpoint = it->first;
            std::vector<Point> frontiers = it->second;
            for (const auto &frontier: frontiers) {
                geometry_msgs::Point p1, p2;
                p1.x = viewpoint->position.x();
                p1.y = viewpoint->position.y();
                p1.z = 0.0;
                p2.x = frontier.get<0>();
                p2.y = frontier.get<1>();
                p2.z = frontier.get<2>();
                view_point_line_markers.points.push_back(p1);
                view_point_line_markers.points.push_back(p2);
            }
        }

        return view_point_line_markers;
    }

    visualization_msgs::Marker ViewpointManager::visualizeLocalPath(const Eigen::Vector2d &sensor_pose) {
        // 使用line list和marker，将local_path中的路径片段转换成marker消息发布
        // 并且应该把两个片段连接起来
        visualization_msgs::Marker local_path_markers;
        local_path_markers.header.frame_id = "map";
        local_path_markers.ns = "local_path";
        local_path_markers.type = visualization_msgs::Marker::LINE_LIST;
        local_path_markers.action = visualization_msgs::Marker::ADD;
        local_path_markers.scale.x = 0.2;
        local_path_markers.color.a = 1.0;
        local_path_markers.color.r = 0.0;
        local_path_markers.color.g = 1.0;
        local_path_markers.color.b = 1.0;
        local_path_markers.pose.orientation.w = 1;
        
        // 起点是机器人的位置
        geometry_msgs::Point p;
        p.x = sensor_pose.x();
        p.y = sensor_pose.y();
        p.z = 0.0;
        local_path_markers.points.push_back(p);

        if (!local_path_.empty()) {
            p.x = local_path_[0]->position.x();
            p.y = local_path_[0]->position.y();
            p.z = 0.0;
            local_path_markers.points.push_back(p);
        }

        for (size_t i = 1; i < local_path_.size(); i++) {
            geometry_msgs::Point p1, p2;
            p1.x = local_path_[i-1]->position.x();
            p1.y = local_path_[i-1]->position.y();
            p1.z = 0.0;
            p2.x = local_path_[i]->position.x();
            p2.y = local_path_[i]->position.y();
            p2.z = 0.0;
            local_path_markers.points.push_back(p1);
            local_path_markers.points.push_back(p2);
        }

        return local_path_markers;
    }

    visualization_msgs::Marker ViewpointManager::visualizeExcutedPath() {
        // 使用line list和marker，将excuted_path中的路径转换成marker消息发布
        visualization_msgs::Marker excuted_path_markers;
        excuted_path_markers.header.frame_id = "map";
        excuted_path_markers.ns = "excuted_path";
        excuted_path_markers.type = visualization_msgs::Marker::LINE_LIST;
        excuted_path_markers.action = visualization_msgs::Marker::ADD;
        excuted_path_markers.scale.x = 0.2;
        excuted_path_markers.color.a = 1.0;
        excuted_path_markers.color.r = 0.0;
        excuted_path_markers.color.g = 1.0;
        excuted_path_markers.color.b = 0.0;
        excuted_path_markers.pose.orientation.w = 1;

        for (size_t i = 1; i < excuted_path_.size(); i++) {
            geometry_msgs::Point p1, p2;
            p1.x = excuted_path_[i-1]->position.x();
            p1.y = excuted_path_[i-1]->position.y();
            p1.z = 0.0;
            p2.x = excuted_path_[i]->position.x();
            p2.y = excuted_path_[i]->position.y();
            p2.z = 0.0;
            excuted_path_markers.points.push_back(p1);
            excuted_path_markers.points.push_back(p2);
        }

        return excuted_path_markers;
    }

    visualization_msgs::MarkerArray ViewpointManager::visualizeSphere() {
        // 使用sphere list和marker，将sphere_list中的球转换成marker消息发布
        visualization_msgs::MarkerArray sphere_markers;
        visualization_msgs::Marker delete_all;
        delete_all.action = visualization_msgs::Marker::DELETEALL;
        sphere_markers.markers.push_back(delete_all);
        int id = 0;
        for (const auto &sphere: sphere_list) {
            visualization_msgs::Marker sphere_marker;
            sphere_marker.header.frame_id = "map";
            sphere_marker.ns = "sphere";
            sphere_marker.id = id++;
            sphere_marker.type = visualization_msgs::Marker::SPHERE;
            sphere_marker.action = visualization_msgs::Marker::ADD;
            sphere_marker.scale.x = sphere.first * 2;
            sphere_marker.scale.y = sphere.first * 2;
            sphere_marker.scale.z = sphere.first * 2;
            sphere_marker.color.a = 0.4;
            sphere_marker.color.r = 0.0;
            sphere_marker.color.g = 1.0;
            sphere_marker.color.b = 0.0;
            sphere_marker.pose.position.x = sphere.second.get<0>();
            sphere_marker.pose.position.y = sphere.second.get<1>();
            sphere_marker.pose.position.z = sphere.second.get<2>();
            sphere_marker.pose.orientation.w = 1;
            sphere_markers.markers.push_back(sphere_marker);
        }

        return sphere_markers;
    }
}