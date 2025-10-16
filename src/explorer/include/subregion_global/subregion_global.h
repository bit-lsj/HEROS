#ifndef SUBREGION_MANAGER_H
#define SUBREGION_MANAGER_H

#include <graph/graph.h>
#include <tsp_solver/tsp_solver.h>

namespace explorer_ns{
    struct Subregion;
    typedef std::shared_ptr<Subregion> SubregionPtr;
    
    struct Subregion {
        double known_voxel_ratio; // 已知体素的比例
        Eigen::Vector2d centroid; // 视点质心
        Eigen::Vector2d center; // 中心坐标
        Box aabb; // 轴对齐边框AABB
        size_t level; // 层级
        std::vector<NodePtr> view_points; // 视点
        std::unordered_map<SubregionPtr, double> distances; // 与其他所有子区域的距离
        std::map<SubregionPtr,std::vector<NodePtr>> paths; // for test

        Subregion(Box ab, Eigen::Vector2d ctr, size_t lv): 
                  aabb(ab), centroid(ctr), center(ctr), level(lv) {};
    };

    class SubregionManager {
    public:
        SubregionManager(const double &bound_x_min, const double &bound_x_max, const double &bound_y_min, const double &bound_y_max,
                         const double &subregion_max_width, const double &subregion_max_length, const double &subregion_min_width, const double &subregion_min_length,
                         const double &known_voxel_thre);
        ~SubregionManager() {};

    void updateSubregion(const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &viewpoint_tree, const NodeRtree &graph, const Box &map_update_bound, const Eigen::Vector2d &sensor_pose);
    void getGlobalTour(const double &sensor_orient, const Eigen::Vector2d &sensor_pose, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph);
    visualization_msgs::MarkerArray visualizeSubregion();
    visualization_msgs::Marker visualizeCentroid();
    visualization_msgs::MarkerArray visualizeViewPaths();
    visualization_msgs::Marker visualizeGlobalTour(const Eigen::Vector2d &sensor_pose);
    visualization_msgs::Marker visualizeLocalBound();

    std::vector<SubregionPtr> global_tour_; // 全局路径

    private:
        double known_voxel_thre_;
        double buffer_height_ = 0.3;
        double subregion_height_;
        size_t max_level_; // 子区域的最大层级
        std::list<SubregionPtr> subregions_; // 子区域集合
        std::list<SubregionPtr> subregions_invalid_; // 当前失效的子区域集合
        std::set<std::pair<SubregionPtr, SubregionPtr>> already_searched_path_; // 已经更新的子区域之间的距离
        std::unordered_map<int, SubregionPtr> subregion_index_; // 子区域索引, 用于tsp

        std::vector<std::vector<NodePtr>> robot2subregion_; // 机器人到子区域的路径

        void calculateMaxLevel(const double &subregion_max_width, const double &subregion_max_length, const double &subregion_min_width, const double &subregion_min_length);
        void initSubregion(const Box &explorer_area, const double &subregion_max_width, const double &subregion_max_length, const double &subregion_min_width, const double &subregion_min_length);
        void calculateKnownVoxelRatio(const SubregionPtr &subregion, const ufo::map::OccupancyMapColor &ufo_map);
        void updateInfo(const Eigen::Vector2d &sensor_pose, const SubregionPtr &subregion, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &viewpoint_tree, const NodeRtree &graph);
        void updateDistance(const SubregionPtr &subregion, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph);
        void deleteInvaildDistance(const SubregionPtr &subregion);
        void getSubregionDistance(const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph, const SubregionPtr &subregion1, const SubregionPtr &subregion2);
        void getViewPointInSubregion(const SubregionPtr &subregion, const NodeRtree &viewpoint_tree);
        // void getRobotInSubregion(const Eigen::Vector2d &sensor_pose);
        void calculateViewPointCentroid(const Eigen::Vector2d &sensor_pose ,const SubregionPtr &subregion, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph);
        void checkDistance();
        double getPathDistance(const Eigen::Vector2d &sensor_pos, const SubregionPtr &subregion, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph);
        bool isOverlap(const Box &aabb1, const Box &aabb2);
        bool isVisible(const ufo::map::OccupancyMapColor &ufo_map, const Eigen::Vector2d &position, const Eigen::Vector2d &centroid);
        SubregionPtr generateSubregion(const Point &min, const Point &max, const size_t &level);
        NodePtr getNearestNode(const Eigen::Vector2d &centroid, const NodeRtree &graph);
        NodePtr getNearestNodeVisible(const Eigen::Vector2d &centroid, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph);
        std::vector<std::vector<int>> getDistMatrix(const double &sensor_orient, const Eigen::Vector2d &sensor_pose, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph);
        std::vector<SubregionPtr> divideSubregion(const Eigen::Vector2d &sensor_pose, const SubregionPtr &subregion, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &viewpoint_tree, const NodeRtree &graph);
        std::vector<NodePtr> aStar(NodePtr start, NodePtr end);
        std::vector<NodePtr> getShortestPath(const Eigen::Vector2d &position, const SubregionPtr &subregion, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &graph);
        Eigen::Vector2d getSubregionCenter(const Box &box);
        Box divideBox(const Box &aabb, const size_t &index);
        Box getMapBound(const ufo::map::OccupancyMapColor &ufo_map);
    };

} // namespace explorer_ns

#endif // SUBREGION_MANAGER_H