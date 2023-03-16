#include "dwa_ext_local_planner/traversability_cost_function.h"

#include <time.h>
#include <cstdlib>


namespace dwa_ext_local_planner {

    TraversabilityCostFunction::TraversabilityCostFunction(){
        std::srand((unsigned)time(NULL));
    }
    TraversabilityCostFunction::~TraversabilityCostFunction(){}

    bool TraversabilityCostFunction::prepare() {
        return true;
    }

    double TraversabilityCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj) {
        double cost = (double) rand()/RAND_MAX;
        return cost;
    }
}
