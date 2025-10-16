#ifndef TSP_SOLVER_H
#define TSP_SOLVER_H

#include "ortools/constraint_solver/constraint_solver.h"
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

using namespace operations_research;

namespace explorer_ns{
    struct DataModel {
        std::vector<std::vector<int>> dist_matrix;
        const int num_vehicles = 1;
        const RoutingIndexManager::NodeIndex depot{0};
    };
    
    class TSPSolver {
    public:
        TSPSolver(DataModel data);
        ~TSPSolver() {};

        void setConstraint(const int &index_1, const int &index_2);
        void solve();
        void printSolution();
        std::vector<int> getSolutionIndex();

    private:
        DataModel data_;
        std::unique_ptr<RoutingModel> routing_;
        std::unique_ptr<RoutingIndexManager> manager_;
        const Assignment *solution_;
    };
} // namespace explorer_ns

#endif // TSP_SOLVER_H