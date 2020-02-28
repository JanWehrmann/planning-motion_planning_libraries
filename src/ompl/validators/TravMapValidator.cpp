#include "TravMapValidator.hpp"

#include <ompl/base/SpaceInformation.h>

#include <base-logging/Logging.hpp>

#include <motion_planning_libraries/ompl/spaces/SherpaStateSpace.hpp>
#include <motion_planning_libraries/State.hpp>

namespace motion_planning_libraries
{

TravMapValidator::TravMapValidator(const ompl::base::SpaceInformationPtr& si,
            Config config) : 
            ompl::base::StateValidityChecker(si),
            mpSpaceInformation(si),
            mConfig(config), 
            mGridCalc() {
}

TravMapValidator::TravMapValidator(const ompl::base::SpaceInformationPtr& si,
            maps::grid::TraversabilityGrid* trav_grid,
            Config config) : 
            ompl::base::StateValidityChecker(si),
            mpSpaceInformation(si),
            mpTravGrid(trav_grid),
            mConfig(config), 
            mGridCalc() {
    mGridCalc.setTravGrid(trav_grid);
}

TravMapValidator::~TravMapValidator() {
}

void TravMapValidator::setTravGrid(maps::grid::TraversabilityGrid* trav_grid) {
    mpTravGrid = trav_grid;
    mGridCalc.setTravGrid(trav_grid);
}
    
bool TravMapValidator::isValid(const ompl::base::State* state) const
{  
    if(mpTravGrid == NULL) {
        throw std::runtime_error("TravMapValidator: No traversability grid available");
    }

    switch(mConfig.mEnvType) {
        case ENV_XY: {
            int x_grid = 0;
            int y_grid = 0;

            const ompl::base::RealVectorStateSpace::StateType* state_rv = 
                    state->as<ompl::base::RealVectorStateSpace::StateType>();
            x_grid = (int)state_rv->values[0];
            y_grid = (int)state_rv->values[1];

            // Check borders.
            if(     x_grid < 0 || x_grid >= static_cast<int>(mpTravGrid->getNumCells().x()) ||
                    y_grid < 0 || y_grid >= static_cast<int>(mpTravGrid->getNumCells().y())) {
                LOG_DEBUG("State (%d,%d) is invalid (not within the grid)", x_grid, y_grid);
                return false;
            }   

            // Check obstacle.
            float driveability = mpTravGrid->getTraversability(x_grid, y_grid).getDrivability();
                
            if(driveability == 0.0) {
                LOG_DEBUG("State (%d,%d) is invalid (lies on an obstacle)", x_grid, y_grid);
                return false;
            }

            return true;
        }
        case ENV_XYTHETA: {
            const ompl::base::SE2StateSpace::StateType* state_se2 = 
                    state->as<ompl::base::SE2StateSpace::StateType>();
                
            // Get current state XYTHETA.    
            double x_grid = state_se2->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0];
            double y_grid = state_se2->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1];
            double yaw_grid = state_se2->as<ompl::base::SO2StateSpace::StateType>(1)->value;
            
            double max_fp = std::max(mConfig.mFootprintRadiusMinMax.first, mConfig.mFootprintRadiusMinMax.second);
            // We use the smaller scale value to check a larger area (actually they should be the same).
            double min_scale = std::min(mpTravGrid->getResolution().x(), mpTravGrid->getResolution().y());
            
            mGridCalc.setFootprintCircleInGrid(0);
            mGridCalc.setFootprintCircleInGrid((int)std::ceil(max_fp / min_scale));
            mGridCalc.setFootprintPoseInGrid(x_grid, y_grid, yaw_grid);
                   
            return mGridCalc.isValid();
        }
        case ENV_SHERPA: {
            const SherpaStateSpace::StateType* state_sherpa = state->as<SherpaStateSpace::StateType>();
                
            // Get current state XYTHETA.    
            double x_grid = state_sherpa->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0];
            double y_grid = state_sherpa->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1];
            double yaw_grid = state_sherpa->as<ompl::base::SO2StateSpace::StateType>(1)->value;
            int fp_class = state_sherpa->getFootprintClass();            
            
            // Use method in State to calculate the radius.
            State state;
            state.setFootprintRadius(mConfig.mFootprintRadiusMinMax.first,
                mConfig.mFootprintRadiusMinMax.second,
                mConfig.mNumFootprintClasses,
                fp_class);
            // Used to calculate the number of grids.
            double min_scale = std::min(mpTravGrid->getResolution().x(), mpTravGrid->getResolution().x());
            
            mGridCalc.setFootprintCircleInGrid(std::ceil(state.getFootprintRadius()/min_scale), false);
            mGridCalc.setFootprintPoseInGrid(x_grid, y_grid, yaw_grid);
            return mGridCalc.isValid();
        }
        default: {
            throw std::runtime_error("TravMapValidator received an unknown environment");
        }
    }
   
}

} // end namespace motion_planning_libraries

