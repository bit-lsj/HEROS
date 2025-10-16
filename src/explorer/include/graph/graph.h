#ifndef GRAPH_H
#define GRAPH_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ufo/map/occupancy_map_color.h>
#include <Eigen/Core>
// STL
#include <memory>
#include <unordered_set>
#include <queue>
#include <unordered_map>
#include <random>

// Boost
#include <boost/functional/hash.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace explorer_ns{
    struct Node;
    typedef std::shared_ptr<Node> NodePtr;
    typedef std::pair<NodePtr, NodePtr> Edge;

    // Rtree相关
    typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> Point;
    typedef boost::geometry::model::box<Point> Box;
    typedef std::pair<Point, NodePtr> Value;
    typedef boost::geometry::index::rtree<Point, boost::geometry::index::quadratic<16>> PointRtree;
    typedef boost::geometry::index::rtree<Value, boost::geometry::index::quadratic<16>> NodeRtree;

    // 为Point类定义哈希函数和相等函数
    struct PointHash {
        std::size_t operator()(const Point& p) const {
            // 使用 std::hash 来计算每个坐标的哈希值，然后将它们组合起来
            std::size_t h1 = std::hash<double>()(p.get<0>());
            std::size_t h2 = std::hash<double>()(p.get<1>());
            std::size_t h3 = std::hash<double>()(p.get<2>());
            return h1 ^ h2 ^ h3;
        }
    };
    struct PointEqual {
        bool operator()(const Point& a, const Point& b) const {
            // 比较两个 Point 对象的每个坐标是否相等
            return a.get<0>() == b.get<0>() && a.get<1>() == b.get<1>() && a.get<2>() == b.get<2>();
        }
    };

    struct Node {
        Eigen::Vector2d position;
        std::unordered_set<NodePtr> neighbors;

        Node(const Eigen::Vector2d &pos): position(pos) {}

        void addNeighbor(const NodePtr &neighbor){
            if (neighbor) {
                neighbors.insert(neighbor);
            }
            else{
                ROS_ERROR("Neighbor is nullptr");
            }
        }

        void deleteNeighbor(const NodePtr &neighbor){
            if (neighbor) {
                neighbors.erase(neighbor);
            }
        }
    };

    class Graph {
    public:
        Graph(const int max_neighbor_num, const double max_neighbor_distance, const double min_edge_distance,
              const double max_edge_distance, const double sample_density, const double sensor_height,
              const double vehicle_width, const double vehicle_length, const double inflate_radius);
        ~Graph() {};
        void extendGraph(const ufo::map::OccupancyMapColor &ufo_map, const PointRtree &frontier_tree, const nav_msgs::OccupancyGridPtr &terrain_map, const Box &map_update_bound);
        void updateGraph(const ufo::map::OccupancyMapColor &ufo_map, const PointRtree &frontier_tree, const Box &map_update_bound);
        visualization_msgs::Marker visualizeGraph();
        visualization_msgs::Marker visualizeEdge();
        visualization_msgs::Marker visualizeSamples();

        // 参数
        int max_neighbor_num_;
        double max_neighbor_distance_;
        double min_edge_distance_;
        double max_edge_distance_;
        double sample_density_;
        double vehicle_width_;
        double vehicle_length_;
        double sensor_height_;
        double inflate_radius_;

        // 采样窗口
        Eigen::Vector2d sample_bound_min_;
        Eigen::Vector2d sample_bound_max_;
        Eigen::Vector2d sample_bound_center_;

        NodeRtree graph_;
        std::set<Edge> edges_;
        std::vector<Eigen::Vector2d> samples_;
        std::set<Edge> deleted_edges_;
        
        void addNode(const NodePtr &node);
        void removeNode(const NodePtr &node);
        void getSampleWindow(const nav_msgs::OccupancyGridPtr &terrain_map, const Box &map_update_bound);
        bool inTraversableSpace(const Eigen::Vector2d &sample, const nav_msgs::OccupancyGridPtr &terrain_map);
        bool isFrontierVisible(const ufo::map::OccupancyMapColor &ufo_map, const PointRtree &frontier_tree, const Eigen::Vector2d &sample);
        bool isCollisionFree(const ufo::map::OccupancyMapColor &ufo_map, const ufo::map::Point3 &start, const ufo::map::Point3 &end);
        bool isConnected();
        std::vector<Point> getFrontiersInRange(const PointRtree &frontier_tree, const Box &map_update_bound);
        template <typename Rtree>
        std::vector<typename Rtree::value_type> getGraphNodeInBox(const Rtree &rtree, const Box &box);
        std::vector<NodePtr> getNeighborsInGraph(const Eigen::Vector2d &sample, const double &radius);
        
        Eigen::Vector2d AdjustDistToGraph(const Eigen::Vector2d &sample, const NodePtr &node);
        Eigen::Vector2d biasSampleNode(const PointRtree &frontier_tree, const Box &map_update_bound);
        NodePtr getCloestNodeInGraph(const Eigen::Vector2d &sample);
        Edge makeEdge(const NodePtr &node1, const NodePtr &node2);
    };

} // namespace explorer_ns

#endif // GRAPH_H