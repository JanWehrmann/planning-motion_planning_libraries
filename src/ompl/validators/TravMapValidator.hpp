#ifndef _TRAV_MAP_VALIDATOR_HPP_
#define _TRAV_MAP_VALIDATOR_HPP_

#include <vector>
#include <map>

#include <base/samples/RigidBodyState.hpp>
#include <base/Waypoint.hpp>

#include <maps/grid/TraversabilityGrid.hpp>
#include <maps/tools/SimpleTraversability.hpp>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <motion_planning_libraries/Config.hpp>
#include <motion_planning_libraries/Helpers.hpp>

namespace envire {
class TraversabilityGrid;
}

namespace motion_planning_libraries
{
class TravMapValidator:  public ompl::base::StateValidityChecker {
 
 private:
    ompl::base::SpaceInformationPtr mpSpaceInformation;
    maps::grid::TraversabilityGrid* mpTravGrid; // To request the driveability values.
    Config mConfig;
    mutable GridCalculations mGridCalc;
    
 public:
    TravMapValidator(const ompl::base::SpaceInformationPtr& si,
            Config config);
 
    TravMapValidator(const ompl::base::SpaceInformationPtr& si,
            maps::grid::TraversabilityGrid* trav_grid,
            Config config);
    
    ~TravMapValidator();
    
    void setTravGrid(maps::grid::TraversabilityGrid* trav_grid);
    
    bool isValid(const ompl::base::State* state) const;
};

} // end namespace motion_planning_libraries

#endif
