#include "SbplEnvXYTHETA.hpp"

#include <exception>

#include <sbpl/headers.h>
#include <sbpl/sbpl_exception.h>

namespace motion_planning_libraries
{

// PUBLIC
SbplEnvXYTHETA::SbplEnvXYTHETA(Config config) : Sbpl(config), 
        mSBPLScaleX(0), mSBPLScaleY(0), mPrims(NULL), mGoalLocal() {
    LOG_DEBUG("SbplEnvXYTHETA constructor");
}

bool SbplEnvXYTHETA::initialize(maps::grid::TraversabilityGrid* trav_grid) {
    
    LOG_DEBUG("SBPLEnvXYTHETA initialize");
    
    mPrims = NULL;
    
    size_t grid_width = trav_grid->getNumCells().x();
    size_t grid_height = trav_grid->getNumCells().y();
    double scale_x = trav_grid->getResolution().x();
    double scale_y = trav_grid->getResolution().y();
    
    mSBPLScaleX = scale_x;
    mSBPLScaleY = scale_y;
    
    assert (scale_x == scale_y);
    
    if(scale_x != 0.1) {
        LOG_WARN("SBPL uses a cell size of 0.1m, other values will probably produce strange results.");
    }
       
    // Create and fill SBPL environment.
    mpSBPLEnv = boost::shared_ptr<EnvironmentNAVXYTHETAMLEVLAT>(
            new EnvironmentNAVXYTHETAMLEVLAT());


    // Use the sbpl-env file if path is given.
    if(!mConfig.mSBPLEnvFile.empty()) {
        LOG_INFO("Load SBPL environment '%s'", mConfig.mSBPLEnvFile.c_str());
        
        try {
            mpSBPLEnv->InitializeEnv(mConfig.mSBPLEnvFile.c_str());
        
            // Request loaded cellsize / scale for environment SBPL_XYTHETA.
            boost::shared_ptr<EnvironmentNAVXYTHETAMLEVLAT> env_xytheta =
                    boost::dynamic_pointer_cast<EnvironmentNAVXYTHETAMLEVLAT>(mpSBPLEnv);
            mSBPLScaleX = mSBPLScaleY = env_xytheta->GetEnvNavConfig()->cellsize_m;
        } catch (SBPL_Exception* e) {
            LOG_ERROR("EnvironmentNAVXYTHETAMLEVLAT could not be initialized using %s (%s)", mConfig.mSBPLEnvFile.c_str(), e->what());
            return false;
        }        
    // Create an sbpl-environment.
    } else {
        
        std::string mprim_file = mConfig.mSBPLMotionPrimitivesFile;
        
        // If no mprim file is specified it will be generated using the available
        // speed informations.
        if(mprim_file.empty()) {
            LOG_INFO("No sbpl mprim file specified, it will be generated");
            assert(scale_x == scale_y);
            mprim_file = "sbpl_motion_primitives.mprim";
            MotionPrimitivesConfig mprim_config(mConfig, grid_width, grid_height, scale_x);
            mPrims = new struct SbplMotionPrimitives(mprim_config);
            mPrims->createPrimitives();
            mPrims->storeToFile(mprim_file);
        }
        createSBPLMap(trav_grid);
        LOG_INFO("Create SBPL EnvironmentNAVXYTHETAMLEVLAT environment");
        boost::shared_ptr<EnvironmentNAVXYTHETAMLEVLAT> env_xytheta =
                boost::dynamic_pointer_cast<EnvironmentNAVXYTHETAMLEVLAT>(mpSBPLEnv);
        try {
            // SBPL does not allow the definition of forward AND backward velocity.
            double speed = fabs(mConfig.mMobility.mSpeed);
            
            if(speed == 0.0) {
                LOG_WARN("Speed of zero is not allowed, abort");
                return false;
            }
          
            // The average turning speed (forward-turn and point-turn) is used for the
            // cost calculation.
            double turning_speed = mConfig.mMobility.mTurningSpeed;
            if(turning_speed == 0) {
                LOG_WARN("Rotational velocity of zero is not allowed, abort");
                return false;
            }
            
            // SBPL: time in sec for a 45° turn  
            double time_to_turn_45_degree = fabs((M_PI / 4.0) / turning_speed);
            // Dynamic footprints are not supported yet, so the max defined width and length are used.
            double robot_width = std::max(mConfig.mFootprintWidthMinMax.first, mConfig.mFootprintWidthMinMax.second);
            double robot_length = std::max(mConfig.mFootprintLengthMinMax.first, mConfig.mFootprintLengthMinMax.second);
            
            if(robot_width == 0 || robot_length == 0) {
                robot_width = robot_length = std::max(mConfig.mFootprintRadiusMinMax.first, mConfig.mFootprintRadiusMinMax.second);
                LOG_WARN("No rectangle footprint has been defined, using footprint radius instead %4.2f", robot_width);
            }
            if(robot_width == 0 || robot_length == 0) {
                LOG_ERROR("No footprint has been specified");
                return false;
            }

            LOG_INFO("SBPL does not support variable footprints, using max width,length (%4.2f, %4.2f)", 
                    robot_width, robot_length);
            std::vector<sbpl_2Dpt_t> fp_vec = createFootprint(robot_width, robot_length);
            base::Time start_t = base::Time::now();
            env_xytheta->InitializeEnv(grid_width, grid_height, 
                mpSBPLMapData, // initial map
                0,0,0, //mStartGrid.position.x(), mStartGrid.position.y(), mStartGrid.getYaw(), 
                0,0,0, //mGoalGrid.position.x(), mGoalGrid.position.y(), mGoalGrid.getYaw(),
                0.1, 0.1, 0.1, // tolerance x,y,yaw, ignored
                fp_vec, 
                scale_x,  // Size of a cell in meter => in SBPL cells have to be quadrats
                speed, 
                time_to_turn_45_degree, 
                SBPL_MAX_COST, // cost threshold
                mprim_file.c_str()); // motion primitives file
            LOG_INFO("SBPL environment initialized within %4.2f sec", 
                    (base::Time::now() - start_t).toSeconds());
        } catch (SBPL_Exception* e) {
            LOG_ERROR("EnvironmentNAVXYTHETAMLEVLAT could not be created using the motion primitive file %s (%s)", 
                    mprim_file.c_str(),
                    e->what());
            return false;
        } catch ( ... ) {
             LOG_ERROR("EnvironmentNAVXYTHETAMLEVLAT initialization: catched a exception");
             return false;
        }
    }
 
    // Create planner.
    switch(mConfig.mPlanner) {
        case UNDEFINED_PLANNER: {
        }
        case ANYTIME_DSTAR: {
            mpSBPLPlanner = boost::shared_ptr<SBPLPlanner>(new ADPlanner(mpSBPLEnv.get(), 
                    mConfig.mSBPLForwardSearch));
            break;
        }
        case ANYTIME_NONPARAMETRIC_ASTAR: {
            mpSBPLPlanner = boost::shared_ptr<SBPLPlanner>(new anaPlanner(mpSBPLEnv.get(), 
                    mConfig.mSBPLForwardSearch));
            break;
        }
        case ANYTIME_ASTAR: {
            mpSBPLPlanner = boost::shared_ptr<SBPLPlanner>(new ARAPlanner(mpSBPLEnv.get(), 
                    mConfig.mSBPLForwardSearch));
        }
        default: {
            LOG_ERROR("Planner %d is not available for this environment", (int)mConfig.mPlanner);
            return false;
        }
    }
    mpSBPLPlanner->set_search_mode(mConfig.mSearchUntilFirstSolution); 
    
    // If available use the start and goal defined in the SBPL environment.
    if(!mConfig.mSBPLEnvFile.empty()) {
        LOG_INFO("Use start/goal of the loaded SBPL environment");
        MDPConfig mdp_cfg;
        
        if (! mpSBPLEnv->InitializeMDPCfg(&mdp_cfg)) {
            LOG_ERROR("InitializeMDPCfg failed, start and goal id cannot be requested yet");
            return false;
        }
           
        if (mpSBPLPlanner->set_start(mdp_cfg.startstateid) == 0) {
            LOG_ERROR("Failed to set start state");
            return false;
        }

        if (mpSBPLPlanner->set_goal(mdp_cfg.goalstateid) == 0) {
            LOG_ERROR("Failed to set goal state");
            return false;
        }
    }

    // Print primitive informations.
    //std::cout << "Primitives: " << std::endl << mPrims->toString() << std::endl;
    LOG_INFO("Primitives:\n%s", mPrims->toString().c_str());

    return true;
}

bool SbplEnvXYTHETA::partialMapUpdate(std::vector<CellUpdate>& cell_updates) {
    if(cell_updates.size() == 0) {
        return true;
    }
    
    boost::shared_ptr<EnvironmentNAVXYTHETAMLEVLAT> env_xytheta =
        boost::dynamic_pointer_cast<EnvironmentNAVXYTHETAMLEVLAT>(mpSBPLEnv);
    
    // Runs through all the cell updates and uses the stored driveability for the 
    // SBPL cost calculation.
    std::vector<CellUpdate>::iterator it = cell_updates.begin();
    for(; it != cell_updates.end(); it++) {
        if(!env_xytheta->UpdateCost(it->x, it->y, driveability2sbpl_cost(it->driveability))) {
            LOG_WARN("SBPL cell (%d, %d) could not be updated", it->x, it->y);
            return false;
        }
    }
    return true;
}

bool SbplEnvXYTHETA::setStartGoal(struct State start_state, struct State goal_state) {
    
    LOG_DEBUG("SBPL setStartGoal");
    
    int start_id = 0;
    int goal_id = 0;
    
    // Start/goal have to be defined in meters (grid_local).
    boost::shared_ptr<EnvironmentNAVXYTHETAMLEVLAT> env_xytheta =
            boost::dynamic_pointer_cast<EnvironmentNAVXYTHETAMLEVLAT>(mpSBPLEnv);
    
    double start_x = start_state.getPose().position[0] * mSBPLScaleX;
    double start_y = start_state.getPose().position[1] * mSBPLScaleY;
    double start_yaw = start_state.getPose().getYaw();
    double goal_x = goal_state.getPose().position[0] * mSBPLScaleX;
    double goal_y = goal_state.getPose().position[1] * mSBPLScaleY;
    double goal_yaw = goal_state.getPose().getYaw();
    
    LOG_INFO("Change start/goal within SBPL env and planner to (%4.2f, %4.2f, %4.2f), (%4.2f, %4.2f, %4.2f)",
        start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw);
    
    start_id = env_xytheta->SetStart(start_x, start_y, start_yaw);
    goal_id = env_xytheta->SetGoal(goal_x, goal_y, goal_yaw);

    if (mpSBPLPlanner->set_start(start_id) == 0) {
        LOG_ERROR("Failed to set start state");
        return false;
    }

    if (mpSBPLPlanner->set_goal(goal_id) == 0) {
        LOG_ERROR("Failed to set goal state");
        return false;
    }
    
    // Used to add it at the end of the intermediate path.
    mGoalLocal[0] = goal_x;
    mGoalLocal[1] = goal_y;
    mGoalLocal[2] = goal_yaw;
    
    // Stores discrete start and goal pose to check for validity.
    mStartGrid[0] = start_state.getPose().position[0];
    mStartGrid[1] = start_state.getPose().position[1];
    mStartGrid[2] = mPrims->calcDiscreteEndOrientation(start_state.getPose().getYaw());
    mGoalGrid[0] = goal_state.getPose().position[0];
    mGoalGrid[1] = goal_state.getPose().position[1];
    mGoalGrid[2] = mPrims->calcDiscreteEndOrientation(goal_state.getPose().getYaw());
      
    return true;
}
    
bool SbplEnvXYTHETA::solve(double time) {
    return Sbpl::solve(time);
}
    
bool SbplEnvXYTHETA::fillPath(std::vector<struct State>& path, bool& pos_defined_in_local_grid) {
    
    LOG_DEBUG("SBPL fillPath");
    
    struct State state;
    
    // GetCoordFromState returns the discrete position and angle!!
    int x_discrete = 0, y_discrete = 0, theta_discrete = 0; 
    
    std::vector<int> path_ids;
    std::vector<sbpl_xy_theta_pt_t> path_xytheta;
    
    // Just fill the path with the motion primitive poses (in grid coordinates).
    std::vector<int>::iterator it = mSBPLWaypointIDs.begin();
    for(; it != mSBPLWaypointIDs.end(); it++) {
        // Fill path with the found solution.
        boost::shared_ptr<EnvironmentNAVXYTHETAMLEVLAT> env_xytheta =
                boost::dynamic_pointer_cast<EnvironmentNAVXYTHETAMLEVLAT>(mpSBPLEnv);
        // Provides grid coordinates, not grid local.
        env_xytheta->GetCoordFromState(*it, x_discrete, y_discrete, theta_discrete);

        // MotionPlanningLibraries expects grid coordinates, but a real angle in rad,
        // not the discrete one! (0-15), adapts to OMPL angles with (-PI,PI]
        state.mPose.position = base::Vector3d((double)x_discrete, (double)y_discrete, 0);
        double theta_rad = DiscTheta2Cont(theta_discrete, NAVXYTHETALAT_THETADIRS);
        // Converts [0,2*M_PI) to (-PI,PI].
        if(theta_rad > 180) {
            theta_rad -= 2*M_PI;
        }
        state.mPose.orientation =  Eigen::AngleAxis<double>(theta_rad, base::Vector3d(0,0,1));
        
        state.mSBPLPrimId = *it;
        
        if(mConfig.mNumIntermediatePoints == 0) {
            path.push_back(state);
        }
        // Store the path ids to request the intermediate poses and the action list.
        path_ids.push_back(*it);
    }
    
    boost::shared_ptr<EnvironmentNAVXYTHETAMLEVLAT> env_xytheta =
        boost::dynamic_pointer_cast<EnvironmentNAVXYTHETAMLEVLAT>(mpSBPLEnv);
    
    // Use ConvertStateIDPathintoXYThetaPath to create the path in the local grid frame
    // using the intermediate points. In this case 'pos_defined_in_local_grid' has to be set to true.
    // The intermediate poses already contain start and goal and their orientations
    // are already adapted to (-PI,PI].
    if(mConfig.mNumIntermediatePoints > 0) {         
        // The returned path is already transformed to grid-local.   
        env_xytheta->ConvertStateIDPathintoXYThetaPath(&path_ids, &path_xytheta);
        pos_defined_in_local_grid = true;
        
        sbpl_xy_theta_pt_t xyt_m_rad;
        for(unsigned int i=0; i<path_xytheta.size(); ++i) {
            xyt_m_rad = path_xytheta[i];
            // The grid local path is shifted to the center of the cells.
            // This is not required here (will be done in MotionPlanningLibraries
            // regarding to the goal pose.
            xyt_m_rad.x -= mSBPLScaleX / 2.0;
            xyt_m_rad.y -= mSBPLScaleY / 2.0;
            state.mPose.position = base::Vector3d(xyt_m_rad.x, xyt_m_rad.y, 0);
            state.mPose.orientation =  Eigen::AngleAxis<double>(xyt_m_rad.theta, base::Vector3d(0,0,1));
            path.push_back(state);
        }
        // The goal pose is not part of the received path, so we add it manually.
        state.mPose.position = base::Vector3d(mGoalLocal[0], mGoalLocal[1], 0.0);
        state.mPose.orientation =  Eigen::AngleAxis<double>(mGoalLocal[2], base::Vector3d(0,0,1));
        path.push_back(state);
    }
    
    // Request and assign prim id and speed values.
    // The prim ids and the speed values are just assigned
    // to the first starting state of each primitive.
    std::vector<EnvNAVXYTHETALATAction_t> action_list;
    env_xytheta->GetActionsFromStateIDPath(&path_ids, &action_list);
    std::vector<EnvNAVXYTHETALATAction_t>::iterator it_action = action_list.begin();
    std::vector<struct State>::iterator it_state = path.begin();
    unsigned int prim_id = 0;
    
    LOG_INFO("Path consist of %d poses / %d primitives (each primitive contains %d poses)", 
            path.size(), action_list.size(), mPrims->mConfig.mNumPosesPerPrim);
    
    
    // Runs through all states and assigns prim id, speed values and movement type to 
    // state of each primitive. If a mprim file is used mPrims may be NULL!
    for(;it_action < action_list.end(); it_action++) {
        prim_id = it_action->aind;
        double speed;
        enum MovementType mov_type;
        
        if(mPrims != NULL) {
            if(mPrims->getSpeed(prim_id, speed)) {
                LOG_INFO("Assigns to prim id %d the speed %4.2f", prim_id, speed);
            } else {
                LOG_ERROR("No speed has been stored for prim id %d", prim_id);
                return false;
            }
            
            if(mPrims->getMovementType(prim_id, mov_type)) {
                LOG_INFO("Assigns to prim id %d the movement type %s", prim_id, MovementTypesString[(int)mov_type].c_str());
            } else {
                LOG_ERROR("No movement type has been stored for prim id %d", prim_id);
                return false;
            }
        }
           
        for (int ipind = 0; ipind < ((int)it_action->intermptV.size()) - 1; ipind++) {
            if(it_state == path.end())
            {
                throw std::runtime_error("Error, path to action mismatch");
            }
            it_state->mSBPLPrimId = prim_id;
            it_state->mSpeed = speed;
            it_state->mMovType = mov_type;
            it_state++;
        }
    }
    
    return true;
}

enum MplErrors SbplEnvXYTHETA::isStartGoalValid() {
    boost::shared_ptr<EnvironmentNAVXYTHETAMLEVLAT> env_xytheta =
        boost::dynamic_pointer_cast<EnvironmentNAVXYTHETAMLEVLAT>(mpSBPLEnv);
    
    LOG_INFO("Check discrete start (%d, %d, %d) and goal pose (%d, %d, %d) for validity",
            mStartGrid[0], mStartGrid[1], mStartGrid[2], mGoalGrid[0], mGoalGrid[1], mGoalGrid[2]);
        
    int err = (int)MPL_ERR_NONE;
    
    if(!env_xytheta->IsValidConfiguration(mStartGrid[0], mStartGrid[1], mStartGrid[2])) {
        LOG_WARN("Start lies on an obstacle");
        err += (int)MPL_ERR_START_ON_OBSTACLE;
    }
    
    if(!env_xytheta->IsValidConfiguration(mGoalGrid[0], mGoalGrid[1], mGoalGrid[2])) {
        LOG_WARN("Goal lies on an obstacle");
        err += (int)MPL_ERR_GOAL_ON_OBSTACLE;
    }
    
    return (enum MplErrors)err;
}

} // namespace motion_planning_libraries
