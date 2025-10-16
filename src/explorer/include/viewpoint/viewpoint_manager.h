#include <graph/graph.h>
// #include <subregion/subregion_manager.h>
#include <subregion_global/subregion_global.h>

namespace explorer_ns {
    class ViewpointManager {
    public:
        ViewpointManager(const double visible_radius, const int filter_thres): visible_radius_(visible_radius), filter_thres_(filter_thres) {};
        ~ViewpointManager() {};

        void generateViewPoint(const ufo::map::OccupancyMapColor &ufo_map, const PointRtree &frontier_tree, const NodeRtree &graph);
        std::vector<NodePtr> getLocalPath(const double &sensor_orient, const Eigen::Vector2d &sensor_pose, const std::vector<SubregionPtr> &global_tour, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph);
        std::vector<NodePtr> getGoalViewPoint(const double &sensor_orient, const Eigen::Vector2d &sensor_pose, const std::vector<SubregionPtr> &global_tour, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph);
        visualization_msgs::Marker visualizeViewPoint();
        visualization_msgs::Marker visualizeViewPointLine();
        visualization_msgs::Marker visualizeLocalPath(const Eigen::Vector2d &sensor_pose);
        visualization_msgs::Marker visualizeExcutedPath();
        visualization_msgs::MarkerArray visualizeSphere();

        NodeRtree viewpoint_tree_;

    private:
        int filter_thres_;
        double visible_radius_;
        double buffer_height_ = 0.3;
        double check_path_dist_thres_ = 20.0;
        double search_viewpoint_radius_ = 13.0;
        double add_global_tour_to_local_plan_radius_ = 15.0;
        double dist_ratio_thre_ = 2;
        NodePtr previous_target_node = nullptr;
        NodePtr previous_viewpoint_goal = nullptr;
        std::multimap<double, Point> sphere_list;
        std::unordered_map<Point, std::vector<Point>, PointHash, PointEqual> frontiers_in_sphere_list;
        std::map<NodePtr, std::vector<Point>> viewpoint_frontiers_; 
        std::map<NodePtr, std::vector<NodePtr>> robot_to_viewpoint_path_;
        std::vector<NodePtr> local_path_;
        std::vector<NodePtr> excuted_path_;

        void generateSphere(const ufo::map::OccupancyMapColor &ufo_map, const PointRtree &frontier_tree);
        bool isVisible(const ufo::map::OccupancyMapColor &ufo_map, const NodePtr &node, const Point &frontier);
        double calculateTheta(const double &sensor_orient, const Eigen::Vector2d &sensor_pose, const NodePtr &viewpoint);
        double getPathDist(const NodePtr &start, const NodePtr &end, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph);
        double getPathDist(const std::vector<NodePtr> &path);
        Point getFrontierCentroid(const std::vector<Point> &frontiers);
        NodePtr getNearestNode(const Eigen::Vector2d &position, const NodeRtree &graph);
        NodePtr getNearestNodeVisible(const ufo::map::OccupancyMapColor &ufo_map, const Point &frontier, const NodeRtree &graph);
        std::vector<std::vector<int>> getViewpointDistMatrix(const double &sensor_orient, const Eigen::Vector2d &sensor_pose, const std::vector<NodePtr>&viewpoints, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph);
        std::vector<NodePtr> getKNearestViewpoint(const Eigen::Vector2d &position);
        std::vector<NodePtr> aStar(const NodePtr &start, const NodePtr &end);
        std::vector<NodePtr> getPath(const NodePtr &start, const NodePtr &end, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph);
        std::vector<NodePtr> getExcutedPath(const double &sensor_orient, const Eigen::Vector2d &sensor_pose, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph);
    };
}