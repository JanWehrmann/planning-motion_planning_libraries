#ifndef _MOTION_PLANNING_LIBRARIES_SBPL_HPP_
#define _MOTION_PLANNING_LIBRARIES_SBPL_HPP_

#include <vector>

#include <boost/shared_ptr.hpp>

#include <motion_planning_libraries/AbstractMotionPlanningLibrary.hpp>

#include <sbpl/utils/utils.h>
#include <sbpl/config.h> // here #define DEBUG 0, causes a lot of trouble
#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/discrete_space_information/environment_nav2D.h>
#include <sbpl/discrete_space_information/environment_navxythetamlevlat.h>
#include <sbpl/planners/planner.h>
#undef DEBUG

namespace motion_planning_libraries
{
    
/**
 * Finds the path with minimal cost from start to goal using a traversability map. 
 */
class Sbpl : public AbstractMotionPlanningLibrary
{      
 protected:
    // Driveability 0.0 to 1.0 will be mapped to SBPL_MAX_COST + 1 to 1 
    // with obstacle threshold of SBPL_MAX_COST + 1.
    static const unsigned char SBPL_MAX_COST = 20;
    
    boost::shared_ptr<DiscreteSpaceInformation> mpSBPLEnv;
    boost::shared_ptr<SBPLPlanner> mpSBPLPlanner;
    std::vector<int> mSBPLWaypointIDs;
    unsigned char* mpSBPLMapData;
    size_t mSBPLNumElementsMap;
    int mLastSolutionCost;
    // Discrete start and goal state(x,y,theta), can be used to check 
    // - after planning have failed - whether the states intersect with an obstacle.
    Eigen::Vector3i mStartGrid, mGoalGrid;
    double mEpsilon;
        
 public: 
    Sbpl(Config config = Config());
    
    /**
     * Clears the waypoint-id-list and replans.
     */
    virtual bool solve(double time);    
   
    /**
     * Converts the trav map to a sbpl map using the driveability value.
     * Driveability 0.0 to 1.0 is mapped to costs SBPL_MAX_COST + 1  to 1 with obstacle threshold SBPL_MAX_COST + 1.
     * (+1 because costs of 0 should be avoided).
     */
    void createSBPLMap(maps::grid::TraversabilityGrid* trav_grid);
    
    /**
     * The footprint has to be defined in meter.
     */
    std::vector<sbpl_2Dpt_t> createFootprint(double robot_width, double robot_height);
    
    /**
     * In SBPL the optimal solution has been found if the internal epsilon reaches 1.0
     * (starting at a higher number).
     */
    bool foundFinalSolution();
    
    unsigned char driveability2sbpl_cost(double driveability);
};
    
} // end namespace motion_planning_libraries

#endif // _MOTION_PLANNING_LIBRARIES_SBPL_HPP_
