#ifndef TRAVERSABILITY_COST_FUNCTION_H_
#define TRAVERSABILITY_COST_FUNCTION_H_

#include <base_local_planner/trajectory_cost_function.h>
// #include <Eigen/Core>

namespace dwa_ext_local_planner {
    /**
     * @class TraversabilityCostFunction
     * @brief Uses camera images to extract the robot footprint and assign a
     * cost to trajectories depending on the terrain traversability
     * 
     */
    class TraversabilityCostFunction: public base_local_planner::TrajectoryCostFunction {

        public:

            /**
             * @brief Construct a new Traversability Cost Function object
             * 
             */
            TraversabilityCostFunction();

            /**
             * @brief Destroy the Traversability Cost Function object
             * 
             */
            virtual ~TraversabilityCostFunction();
            
            /**
             * @brief General updating of context values
             * 
             * @return true 
             * @return false 
             */
            virtual bool prepare();

            /**
             * @brief Return a score for trajectory traj
             * 
             * @param traj The trajectory to assign a cost
             * @return double The score of the trajectory
             */
            virtual double scoreTrajectory(base_local_planner::Trajectory &traj);
    };
}

#endif
