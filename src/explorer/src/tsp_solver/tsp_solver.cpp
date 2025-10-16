#include <tsp_solver/tsp_solver.h>

// This implement is referenced from https://developers.google.com/optimization/routing/tsp

namespace explorer_ns {
    TSPSolver::TSPSolver(DataModel data): data_(data) {
        manager_ = std::make_unique<RoutingIndexManager>(data_.dist_matrix.size(), data_.num_vehicles, data_.depot);
        routing_ = std::make_unique<RoutingModel>(*manager_);
    }

    void TSPSolver::setConstraint(const int &index_1, const int &index_2) {
        auto constraint = routing_->solver()->MakeLessOrEqual(routing_->NextVar(index_1), routing_->NextVar(index_2));
        routing_->solver()->AddConstraint(constraint);
    }

    void TSPSolver::solve() {
        const int transit_callback_index = routing_->RegisterTransitCallback(
            [this](int64_t from_index, int64_t to_index) -> int64_t {
                const int from_node = manager_->IndexToNode(from_index).value();
                const int to_node = manager_->IndexToNode(to_index).value();
                return data_.dist_matrix[from_node][to_node];
            }
        );

        routing_->SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

        RoutingSearchParameters search_parameters = DefaultRoutingSearchParameters();
        search_parameters.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);
    
        solution_ = routing_->SolveWithParameters(search_parameters);
    }

    void TSPSolver::printSolution() {
        LOG(INFO) << "Objective: " << (solution_->ObjectiveValue() / 10.0) << "meters";
        // Inspect solution.
        int64_t index = routing_->Start(0);
        LOG(INFO) << "Route:";
        int64_t distance{0};
        std::stringstream route;
        while (!routing_->IsEnd(index)) {
            route << manager_->IndexToNode(index).value() << " -> ";
            const int64_t previous_index = index;
            index = solution_->Value(routing_->NextVar(index));
            distance += routing_->GetArcCostForVehicle(previous_index, index, int64_t{0});
        }
        LOG(INFO) << route.str() << manager_->IndexToNode(index).value();
        LOG(INFO) << "Route distance: " << distance / 10.0 << "meters";
        LOG(INFO) << "Problem solved in " << routing_->solver()->wall_time() << "ms"; 
    }

    std::vector<int> TSPSolver::getSolutionIndex() {
        std::vector<int> tour;
        int index = routing_->Start(0);
        while (!routing_->IsEnd(index)) {
            tour.push_back(manager_->IndexToNode(index).value());
            index = solution_->Value(routing_->NextVar(index));
        }

        return tour;
    }
}