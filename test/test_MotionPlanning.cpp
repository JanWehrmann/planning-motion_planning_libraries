#include <boost/test/unit_test.hpp>

#include <stdlib.h>
#include <stdio.h>

#include <motion_planning_libraries/MotionPlanningLibraries.hpp>
#include <motion_planning_libraries/Helpers.hpp>
#include <motion_planning_libraries/sbpl/SbplMotionPrimitives.hpp>

#include <maps/grid/TraversabilityGrid.hpp>

using namespace motion_planning_libraries;

struct Fixture {
    Fixture(){
        maps::grid::TraversabilityCell defaultCell = maps::grid::TraversabilityCell();
        trav = new maps::grid::TraversabilityGrid(maps::grid::Vector2ui(100, 100), maps::grid::Vector2d(0.1, 0.1), defaultCell);
        trav->setTraversabilityClass(0, maps::grid::TraversabilityClass(0.5)); // driveability of unknown
        trav->setTraversabilityClass(1, maps::grid::TraversabilityClass(0.0)); // driveability of obstacles

        // Set start and goal
        rbs_start.setPose(base::Pose(base::Position(1,1,0), base::Orientation::Identity()));
        rbs_goal.setPose(base::Pose(base::Position(9,9,0), base::Orientation::Identity()));
        
        conf.mSearchUntilFirstSolution = true;
        
    }
    
    ~Fixture() {
    }
    double cost = 0;
    maps::grid::TraversabilityGrid* trav;
    Config conf;
    base::samples::RigidBodyState rbs_start;
    base::samples::RigidBodyState rbs_goal;
};

BOOST_FIXTURE_TEST_SUITE( s, Fixture )

BOOST_AUTO_TEST_CASE(sbpl_mprims)
{
    struct MotionPrimitivesConfig config;
    config.mMobility.mSpeed = 1.0;
    config.mMobility.mTurningSpeed = 0.1;
    config.mMobility.mMinTurningRadius = 1.0;
    
    config.mMobility.mMultiplierForward = 1;
    config.mMobility.mMultiplierBackward = 2;
    config.mMobility.mMultiplierLateral = 3;
    config.mMobility.mMultiplierForwardTurn = 4;
    config.mMobility.mMultiplierPointTurn = 5;
    
    config.mNumPrimPartition = 2;
    config.mNumPosesPerPrim = 10;
    config.mNumAngles = 16;
    
    config.mMapWidth = 100;
    config.mMapHeight = 100;
    config.mGridSize = 0.1;
    
    SbplMotionPrimitives mprims(config);
    mprims.createPrimitives();
    mprims.storeToFile("test.mprim");
}
    
#if 0

BOOST_AUTO_TEST_CASE(helper_rectangle)
{
    // Draw a rectangle in the center 
    GridCalculations calc;
    calc.setTravGrid(trav, trav_data);
    calc.setFootprintRectangleInGrid(10, 10); // length, width
    calc.setFootprintPoseInGrid(50, 50, 0); // x, y, theta
    calc.setValue(1); // obstacle
    BOOST_CHECK(calc.isValid() == false);
}

// BOOST_FIXTURE_TEST_CASE resets the fixture
BOOST_AUTO_TEST_CASE(helper_rectangle_valid_test)
{
    std::cout << std::endl << "HELPER RECTANGLES TESTS" << std::endl;
    GridCalculations calc;
    calc.setTravGrid(trav, trav_data);
    calc.setFootprintRectangleInGrid(10, 10); // length, width
    calc.setFootprintPoseInGrid(50, 50, 0); // x, y, theta
    calc.setValue(1); // obstacle
    
    // x,y,theta,width,length
    calc.setFootprintRectangleInGrid(10, 10); 
    calc.setFootprintPoseInGrid(50, 50, 0);
    BOOST_CHECK(calc.isValid() == false);
    
    calc.setFootprintPoseInGrid(55, 55, 0);
    BOOST_CHECK(calc.isValid() == false);
    
    calc.setFootprintPoseInGrid(60, 60, 0);
    BOOST_CHECK(calc.isValid() == true);
}

BOOST_AUTO_TEST_CASE(sbpl_xy_planning)
{
    // SBPL
    std::cout << std::endl << "SBPL XY PLANNING" << std::endl;
    conf.mPlanningLibType = LIB_SBPL;
    conf.mEnvType = ENV_XY;
    
    GridCalculations calc;
    calc.setTravGrid(trav, trav_data);
    calc.setFootprintRectangleInGrid(10, 10); // length, width
    calc.setFootprintPoseInGrid(50, 50, 0); // x, y, theta
    calc.setValue(1); // obstacle
    
    MotionPlanningLibraries sbpl(conf);
    sbpl.setTravGrid(env, "/trav_map");
    sbpl.setStartState(State(rbs_start));
    sbpl.setGoalState(State(rbs_goal));

    if(sbpl.plan(10)) {
        std::cout << "SBPL problem solved" << std::endl;
        sbpl.printPathInWorld();
    } else {
        std::cout << "SBPL problem could not be solved" << std::endl;
    } 
}

BOOST_AUTO_TEST_CASE(sbpl_xytheta_planning)
{
    std::string path_primitives(getenv ("AUTOPROJ_CURRENT_ROOT"));
    path_primitives += "/external/sbpl/matlab/mprim/pr2_10cm.mprim";
    conf.mSBPLMotionPrimitivesFile = path_primitives;
    
    GridCalculations calc;
    calc.setTravGrid(trav, trav_data);
    calc.setFootprintRectangleInGrid(10, 10); // length, width
    calc.setFootprintPoseInGrid(50, 50, 0); // x, y, theta
    calc.setValue(1); // obstacle    
    
    // SBPL
    std::cout << std::endl << "SBPL XYTHETA PLANNING" << std::endl;
    conf.mPlanningLibType = LIB_SBPL;
    conf.mEnvType = ENV_XYTHETA;
    
    MotionPlanningLibraries sbpl(conf);
    sbpl.setTravGrid(env, "/trav_map");
    sbpl.setStartState(State(rbs_start));
    sbpl.setGoalState(State(rbs_goal));

    if(sbpl.plan(10)) {
        std::cout << "SBPL problem solved" << std::endl;
        sbpl.printPathInWorld();
    } else {
        std::cout << "SBPL problem could not be solved" << std::endl;
    }
}

BOOST_AUTO_TEST_CASE(ompl_xy_planning)
{
    std::cout << std::endl << "OMPL XY PLANNING" << std::endl;
    conf.mPlanningLibType = LIB_OMPL;
    conf.mEnvType = ENV_XY;
    
    GridCalculations calc;
    calc.setTravGrid(trav, trav_data);
    calc.setFootprintRectangleInGrid(10, 10); // length, width
    calc.setFootprintPoseInGrid(50, 50, 0); // x, y, theta
    calc.setValue(1); // obstacle
    
    MotionPlanningLibraries ompl(conf);
    ompl.setTravGrid(env, "/trav_map");
    ompl.setStartState(rbs_start);
    ompl.setGoalState(rbs_goal);
    
    if(ompl.plan(10)) {
        std::cout << "OMPL problem solved" << std::endl;
        ompl.printPathInWorld();
    } else {
        std::cout << "OMPL problem could not be solved" << std::endl;
    }
    
    std::cout << std::endl << "OMPL XY PLANNING IMPROVE" << std::endl;
    if(ompl.plan(10)) {
        std::cout << "OMPL problem improved" << std::endl;
        ompl.printPathInWorld();
    } else {
        std::cout << "OMPL problem could not be improved" << std::endl;
    }
}

BOOST_AUTO_TEST_CASE(omBOOST_AUTO_TEST_CASEpl_xytheta_planning)
{
    std::cout << std::endl << "OMPL XYTHETA PLANNING" << std::endl;
    conf.mPlanningLibType = LIB_OMPL;
    conf.mEnvType = ENV_XYTHETA;
    
    GridCalculations calc;
    calc.setTravGrid(trav, trav_data);
    calc.setFootprintRectangleInGrid(10, 10); // length, width
    calc.setFootprintPoseInGrid(50, 50, 0); // x, y, theta
    calc.setValue(1); // obstacle
    
    MotionPlanningLibraries ompl(conf);
    ompl.setTravGrid(env, "/trav_map");
    ompl.setStartState(rbs_start);
    ompl.setGoalState(rbs_goal);
    
    if(ompl.plan(10)) {
        std::cout << "OMPL problem solved" << std::endl;
        ompl.printPathInWorld();
    } else {
        std::cout << "OMPL problem could not be solved" << std::endl;
    }
    
    std::cout << std::endl << "OMPL XY PLANNING IMPROVE" << std::endl;
    if(ompl.plan(10)) {
        std::cout << "OMPL problem improved" << std::endl;
        ompl.printPathInWorld();
    } else {
        std::cout << "OMPL problem could not be improved" << std::endl;
    }   
}

#endif

BOOST_AUTO_TEST_SUITE_END()