#include "AbstractMotionPlanningLibrary.hpp"

namespace motion_planning_libraries
{

// PUBLIC
AbstractMotionPlanningLibrary::AbstractMotionPlanningLibrary(Config config) : 
        mConfig(config),
        mPathCost(nan(""))
{
}

AbstractMotionPlanningLibrary::~AbstractMotionPlanningLibrary() {
}

bool AbstractMotionPlanningLibrary::initialize(maps::grid::TraversabilityGrid* travGrid) {
    LOG_WARN("Abstract initialization is used");
    return false;
}

bool AbstractMotionPlanningLibrary::partialMapUpdate(std::vector<CellUpdate>& cell_updates) {
    LOG_WARN("Abstract partialMapUpdate() is used");
    return false;
}
        
bool AbstractMotionPlanningLibrary::initialize_arm() {
    LOG_WARN("Abstract arm initialization is used");
    return false;
}

} // namespace motion_planning_libraries
