#include <stdlib.h>
#include <stdio.h>

#include <motion_planning_libraries/MotionPlanningLibraries.hpp>
#include <motion_planning_libraries/Helpers.hpp>
#include <motion_planning_libraries/sbpl/SbplMotionPrimitives.hpp>

#include <envire/core/Environment.hpp>
#include <maps/grid/TraversabilityGrid.hpp>

int main(int argc, char** argv)
{
    using namespace motion_planning_libraries;

    Config conf;
    conf.mPlanningLibType = motion_planning_libraries::PlanningLibraryType::LIB_SBPL;
    conf.mEnvType = motion_planning_libraries::ENV_XYTHETA;
    conf.mPlanner = motion_planning_libraries::ANYTIME_DSTAR;
    conf.mSearchUntilFirstSolution = false;
    conf.mMaxAllowedSampleDist = 1.0;
    conf.mEscapeTrajRadiusFactor = 1.4;
    conf.mMobility.mSpeed = 0.8;
    conf.mMobility.mTurningSpeed = 0.5;
    conf.mMobility.mMultiplierForward = 1;
    conf.mMobility.mMultiplierBackward = 2;
    conf.mMobility.mMultiplierBackwardTurn = 4;
    conf.mMobility.mMultiplierLateral = 0;
    conf.mMobility.mMultiplierForwardTurn = 3;
    conf.mMobility.mMultiplierPointTurn = 3;
    conf.mMobility.mMinTurningRadius = 1.0;
    conf.mReplanning.mReplanDuringEachUpdate = false;
    conf.mReplanning.mReplanOnNewStartPose = false;
    conf.mReplanning.mReplanOnNewGoalPose = true;
    conf.mReplanning.mReplanOnNewMap = false;
    conf.mReplanning.mReplanMinDistStartGoal = 1.0;
    conf.mFootprintLengthMinMax.first = 1.27;
    conf.mFootprintLengthMinMax.second = 1.27;
    conf.mFootprintWidthMinMax.first = 0.87;
    conf.mFootprintWidthMinMax.second = 0.87;
    conf.mNumFootprintClasses = 10;
    conf.mTimeToAdaptFootprint = 40;
    conf.mAdaptFootprintPenalty = 20;
    conf.mSBPLEnvFile = "";
    conf.mSBPLMotionPrimitivesFile = "";
    conf.mSBPLForwardSearch = true;
    conf.mNumIntermediatePoints = 4;
    conf.mNumPrimPartition = 4;
    conf.mPrimAccuracy = 0.15;
    conf.mJointBorders.clear();

    // Create the trav map.
    maps::grid::TraversabilityCell defaultCell = maps::grid::TraversabilityCell();
    maps::grid::TraversabilityGrid* trav = new maps::grid::TraversabilityGrid(maps::grid::Vector2ui(100, 100),
                                                                              maps::grid::Vector2d(0.1, 0.1),
                                                                              defaultCell);
    trav->setTraversabilityClass(0, maps::grid::TraversabilityClass(0.5)); // driveability of unknown
    trav->setTraversabilityClass(1, maps::grid::TraversabilityClass(0.0)); // driveability of obstacles
 
    // Create start and goal
    base::samples::RigidBodyState rbs_start;
    rbs_start.setPose(base::Pose(base::Position(1,1,0), base::Orientation::Identity()));
    base::samples::RigidBodyState rbs_goal;
    rbs_goal.setPose(base::Pose(base::Position(1.8,1,0), base::Orientation::Identity()));
    
    // Draw a rectangle in the center 
    GridCalculations calc;
    calc.setTravGrid(trav);
    
    // x,y,theta,width,length
    calc.setFootprintRectangleInGrid(10, 10); // length, width
    calc.setFootprintPoseInGrid(50, 50, 0); // x, y, theta
    calc.setValue(1); // obstacle
    

    calc.setFootprintPoseInGrid(50, 50, 0);
    std::cout << "Footprint 1 " << (calc.isValid() ? "valid" : "not valid") << std::endl;
    
    calc.setFootprintPoseInGrid(55, 55, 0);
    std::cout << "Footprint 2 " << (calc.isValid() ? "valid" : "not valid") << std::endl;
    
    calc.setFootprintPoseInGrid(60, 60, 0);
    std::cout << "Footprint 3 " << (calc.isValid() ? "valid" : "not valid") << std::endl;
    
    MotionPlanningLibraries sbpl(conf);
    sbpl.setTravGrid(trav);
    sbpl.setStartState(State(rbs_start));
    sbpl.setGoalState(State(rbs_goal));

    double cost = 0;
    if(sbpl.plan(10, cost)) {
        std::cout << "SBPL problem solved" << std::endl;
        sbpl.printPathInWorld();
    } else {
        std::cout << "SBPL problem could not be solved" << std::endl;
    }

    std::vector<base::Trajectory> vec_traj =
            sbpl.getTrajectoryInWorld();
     
    return 0;
}
