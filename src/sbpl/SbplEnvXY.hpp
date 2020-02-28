#ifndef _MOTION_PLANNING_LIBRARIES_SBPL_ENVXY_HPP_
#define _MOTION_PLANNING_LIBRARIES_SBPL_ENVXY_HPP_

#include "Sbpl.hpp"

namespace motion_planning_libraries
{
    
class SbplEnvXY : public Sbpl
{      
   
 public: 
    SbplEnvXY(Config config = Config());
 
    /**
     * 
     */
    virtual bool initialize(maps::grid::TraversabilityGrid* trav_grid);
    
    virtual bool partialMapUpdate(std::vector<CellUpdate>& cell_updates);
    
    /**
     * 
     */
    virtual bool setStartGoal(struct State start_state, struct State goal_state);
      
    /**
     * 
     */       
    virtual bool solve(double time);   
        
    /**
     * 
     */
    virtual bool fillPath(std::vector<struct State>& path, bool& pos_defined_in_local_grid);       
    
    enum MplErrors isStartGoalValid();
};
    
} // end namespace motion_planning_libraries

#endif // _MOTION_PLANNING_LIBRARIES_SBPL_ENVXY_HPP_
