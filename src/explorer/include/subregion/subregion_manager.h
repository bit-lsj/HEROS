#ifndef SUBREGION_MANAGER_H
#define SUBREGION_MANAGER_H

#include <graph/graph.h>
#include <tsp_solver/tsp_solver.h>

namespace explorer_ns{
    struct Subregion;
    typedef std::shared_ptr<Subregion> SubregionPtr;
    
    struct Subregion {
        double known_ratio; // 已知体素的比例
        Eigen::Vector2d centroid; // 视点质心
        Eigen::Vector2d center; // 中心坐标
        Box aabb; // 轴对齐边框AABB
        size_t level; // 层级
        std::vector<NodePtr> view_points; // 视点
        std::unordered_map<SubregionPtr, double> distances; // 与其他所有子区域的距离
        std::map<SubregionPtr,std::vector<NodePtr>> paths; // for test

        Subregion(Box ab, Eigen::Vector2d ctr, size_t lv): 
                  aabb(ab), center(ctr), level(lv) {};
    };

    class SubregionManager {
    public:
        SubregionManager(const double &subregion_width, const double &subregion_length, const double known_thre);
        ~SubregionManager() {};

    void updateSubregion(const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &viewpoint_tree, const NodeRtree &graph, const Box &map_update_bound);
    void getGlobalTour(const Eigen::Vector2d &sensor_pose, const NodeRtree &graph);
    visualization_msgs::MarkerArray visualizeSubregion();
    visualization_msgs::Marker visualizeCentroid();
    visualization_msgs::MarkerArray visualizeViewPaths();
    visualization_msgs::Marker visualizeGlobalTour(Eigen::Vector2d &sensor_pose);

    std::vector<SubregionPtr> global_tour_; // 全局路径

    private:
        double sensor_height_;
        double known_thre_;
        double bound_x_min_; //地图边界
        double bound_x_max_;
        double bound_y_min_;
        double bound_y_max_;
        double subregion_width_; // 子区域的最小宽高
        double subregion_length_;
        double subregion_height_;
        size_t max_level_; // 子区域的最大层级
        std::list<SubregionPtr> subregions_; // 子区域集合
        std::list<SubregionPtr> subregions_invalid_; // 待更新的子区域集合
        std::set<std::pair<SubregionPtr, SubregionPtr>> updated_dist; // 已经更新的子区域之间的距离
        std::unordered_map<int, SubregionPtr> subregion_index_; // 子区域索引, 用于tsp
        Box map_bound_; // 地图边界

        void initSubregion(const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &viewpoint_tree, const NodeRtree &graph);
        void calculateKnownVoxelRatio(const SubregionPtr &subregion, const ufo::map::OccupancyMapColor &ufo_map);
        void updateInfo(const SubregionPtr &subregion, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &viewpoint_tree);
        void updateDistance(const SubregionPtr &subregion, const NodeRtree &graph);
        void deleteInvaildDistance(const SubregionPtr &subregion);
        void getSubregionDistance(const NodeRtree &graph, const SubregionPtr &subregion1, const SubregionPtr &subregion2);
        void calculateViewPointCentroid(const SubregionPtr &subregion);
        void checkDistance();
        double getPathDistance(const Eigen::Vector2d &start, const Eigen::Vector2d &end, const NodeRtree &graph);
        bool isBoundChanged(const ufo::map::OccupancyMapColor &ufo_map);
        bool isOverlap(const Box &aabb1, const Box &aabb2);
        SubregionPtr generateSubregion(const Point &min, const Point &max, const size_t &level);
        NodePtr getNearestNode(const Eigen::Vector2d &centroid, const NodeRtree &graph);
        std::vector<std::vector<int>> getDistMatrix(const Eigen::Vector2d &sensor_pose, const NodeRtree &graph);
        std::vector<SubregionPtr> divideSubregion(const SubregionPtr &subregion, const ufo::map::OccupancyMapColor &ufo_map, const NodeRtree &viewpoint_tree);
        std::vector<NodePtr> aStar(NodePtr start, NodePtr end);
        std::vector<NodePtr> getViewPoint(const Box &box, const NodeRtree &viewpoint_tree);
        Eigen::Vector2d getSubregionCenter(const Box &box);
        Box divideBoxInHorizontal(const Box &aabb, const size_t &index);
        Box divideBoxInVertical(const Box &aabb, const size_t &index);
        Box divideBox(const Box &aabb, const size_t &index);
        Box getMapBound(const ufo::map::OccupancyMapColor &ufo_map);
    };

} // namespace explorer_ns

#endif // SUBREGION_MANAGER_H