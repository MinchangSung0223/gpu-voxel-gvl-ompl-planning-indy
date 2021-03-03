// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the GPU Voxels Software Library.
//
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE.txt in the top
// directory of the source code.
//
// © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Andreas Hermann
 * \date    2018-01-07
 *
 */
//----------------------------------------------------------------------/*
#include <iostream>
using namespace std;
#include <signal.h>

#include <gpu_voxels/logging/logging_gpu_voxels.h>

#define IC_PERFORMANCE_MONITOR
#include <icl_core_performance_monitor/PerformanceMonitor.h>
#include <unistd.h> 
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/fmt/FMT.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <sensor_msgs/JointState.h>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames_io.hpp>

#include <ompl/geometric/PathSimplifier.h>
#include "indy/IndyDCPConnector.h"
#include "gvl_ompl_planner_helper.h"
#include <stdlib.h>
#include <ros/ros.h>
#include <thread>
#include <memory>
#include "gripper/dynamixel_sdk.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

#define PI 3.141592
#define D2R 3.141592/180.0
#define R2D 180.0/3.141592
using namespace KDL;
using namespace std;
using namespace NRMKIndy::Service::DCP;
IndyDCPConnector connector("192.168.0.7", ROBOT_INDY7);
std::shared_ptr<GvlOmplPlannerHelper> my_class_ptr;
enum States { READY, FIND, GRASP, MOVE1,MOVE2,UNGRASP };
void WaitFinish(IndyDCPConnector& connector) {
    // Wait for motion finishing
    bool fin = false;
    do {
            sleep(0.5);
        	connector.isMoveFinished(fin); // check if motion finished
    } while (!fin);
}
void doTaskPlanning(double* goal_values,IndyDCPConnector& connector){


    PERF_MON_INITIALIZE(100, 1000);
    PERF_MON_ENABLE("planning");

    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(6));
    //We then set the bounds for the R3 component of this state space:
    ob::RealVectorBounds bounds(6);
    bounds.setLow(-3.14159265);
    bounds.setHigh(3.14159265);
 
    space->setBounds(bounds);
    //Create an instance of ompl::base::SpaceInformation for the state space
    auto si(std::make_shared<ob::SpaceInformation>(space));
    std::shared_ptr<GvlOmplPlannerHelper> my_class_ptr(std::make_shared<GvlOmplPlannerHelper>(si));

    //Set the state validity checker
       std::cout << "-------------------------------------------------" << std::endl;
    og::PathSimplifier simp(si);

    si->setStateValidityChecker(my_class_ptr->getptr());
    si->setMotionValidator(my_class_ptr->getptr());
    si->setup();
    KDL::Tree my_tree;
    KDL::Chain my_chain;


   if (!kdl_parser::treeFromFile("indy7_ros.urdf", my_tree)){
      ROS_ERROR("Failed to construct kdl tree");
   }

    LOGGING_INFO(Gpu_voxels, "\n\nKDL Number of Joints : "<<my_tree.getNrOfJoints() <<"\n"<< endl);
    LOGGING_INFO(Gpu_voxels, "\n\nKDL Chain load : "<<my_tree.getChain("base_link","tcp",my_chain) <<"\n"<< endl);
  
    std::cout<<my_chain.getNrOfJoints()<<std::endl;


    KDL::JntArray q1(my_chain.getNrOfJoints());
    KDL::JntArray q_init(my_chain.getNrOfJoints());
  
    std::cout<<my_chain.getNrOfJoints()<<std::endl;
    double start_values[6];
    connector.getJointPosition(start_values);
    q_init(0) = start_values[0]*D2R;//0;
    q_init(1) = start_values[1]*D2R;// -0.7850857777;
    q_init(2) =  start_values[2]*D2R;//0;
    q_init(3) =  start_values[3]*D2R;//-2.3555949;
    q_init(4) =  start_values[4]*D2R;//0;
    q_init(5) =  start_values[5]*D2R;//1.57091693296;

    KDL::JntArray q_min(my_chain.getNrOfJoints()),q_max(my_chain.getNrOfJoints());
    q_min(0) = -3.05432619099;
    q_min(1) =  -3.05432619099;
    q_min(2) =  -3.05432619099;
    q_min(3) =  -3.05432619099;
    q_min(4) =  -3.05432619099;
    q_min(5) =  -3.75245789179;

    q_max(0) = 3.05432619099;
    q_max(1) = 3.05432619099;
    q_max(2) = 3.05432619099;
    q_max(3) = 3.05432619099;
    q_max(4) = 3.05432619099;
    q_max(5) = 3.75245789179;

    KDL::Frame cart_pos;
    KDL::ChainFkSolverPos_recursive fk_solver = KDL::ChainFkSolverPos_recursive(my_chain);
    fk_solver.JntToCart(q_init, cart_pos);
    cout<<cart_pos<<endl;
    KDL::ChainIkSolverVel_pinv iksolver1v(my_chain);
    KDL::ChainIkSolverPos_NR_JL iksolver1(my_chain,q_min,q_max,fk_solver,iksolver1v,2000,0.01);
    //KDL::ChainIkSolverPos_NR  iksolver1(my_chain,fk_solver,iksolver1v,200,1e-6);
  	KDL::Vector pos = KDL::Vector(0.05,-0.1865,1.20);
  	
    KDL::Frame goal_pose( KDL::Rotation::RPY(goal_values[0],goal_values[1],goal_values[2]),KDL::Vector(goal_values[3],goal_values[4],goal_values[5]));

    bool ret = iksolver1.CartToJnt(q_init,goal_pose,q1);
    std::cout<<"ik ret : "<<ret<<std::endl;
    std::cout<<"ik q : "<<q1(0)<<","<<q1(1)<<","<<q1(2)<<","<<q1(3)<<","<<q1(4)<<","<<q1(5)<<std::endl;
    
    ob::ScopedState<> start(space);


    start[0] = double(q_init(0));
    start[1] = double(q_init(1));
    start[2] = double(q_init(2));
    start[3] = double(q_init(3));
    start[4] = double(q_init(4));
    start[5] = double(q_init(5));


   ob::ScopedState<> goal(space);
    goal[0] = double(q1(0));
    goal[1] =  double(q1(1));
    goal[2] =  double(q1(2));
    goal[3] = double(q1(3));
    goal[4] =  double(q1(4));
    goal[5] = double(q1(5));


	my_class_ptr->insertStartAndGoal(start, goal);
    my_class_ptr->doVis();

    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    pdef->setStartAndGoalStates(start, goal);
    auto planner(std::make_shared<og::LBKPIECE1>(si));
    planner->setProblemDefinition(pdef);
    planner->setup();

    int succs = 0;

    std::cout << "Waiting for Viz. Press Key if ready!" << std::endl;
    ob::PathPtr path ;
     for(int i = 0;i<2;i++)
    {
        my_class_ptr->moveObstacle();
        planner->clear();
        ob::PlannerStatus solved = planner->ob::Planner::solve(200.0);
        PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("planner", "Planning time", "planning");


        if (solved)
        {
            ++succs;
            path = pdef->getSolutionPath();
            std::cout << "Found solution:" << std::endl;
            path->print(std::cout);
            simp.simplifyMax(*(path->as<og::PathGeometric>()));

        }else{
            std::cout << "No solution could be found" << std::endl;
        }

        PERF_MON_SUMMARY_PREFIX_INFO("planning");
        std::cout << "END OMPL" << std::endl;
        my_class_ptr->doVis();


    }
    PERF_MON_ADD_STATIC_DATA_P("Number of Planning Successes", succs, "planning");

    PERF_MON_SUMMARY_PREFIX_INFO("planning");

    // keep the visualization running:
    og::PathGeometric* solution= path->as<og::PathGeometric>();
    solution->interpolate();

    int step_count = solution->getStateCount();
    for(int i=0;i<step_count;i++){
    
     const double *values = solution->getState(i)->as<ob::RealVectorStateSpace::StateType>()->values;
    std::cout<<"step "<<i<<" joint :"<< values[0]<<","<<values[1]<<","<<values[2]<<","<<values[3]<<","<<values[4]<<","<<values[5]<<std::endl;
		double qvec[6]  = {values[0]*R2D,values[1]*R2D,values[2]*R2D,values[3]*R2D,values[4]*R2D,values[5]*R2D};       
    std::cout<<"qvec step "<<i<<" joint :"<< qvec[0]<<","<<qvec[1]<<","<<qvec[2]<<","<<qvec[3]<<","<<qvec[4]<<","<<qvec[5] <<std::endl;

    	my_class_ptr->visualizeRobot(values);
       	connector.moveJointTo(qvec);
   		WaitFinish(connector);	
     }



}

void jointStateCallback()
{
  //std::cout << "Got JointStateMessage" << std::endl;
	signal(SIGINT, ctrlchandler);
  signal(SIGTERM, killhandler);
  while(1){

    double values[6];
    connector.getJointPosition(values);
    //std::cout<<"\njoint values : "<<values[0]<<","<<values[1]<<","<<values[2]<<","<<values[3]<<","<<values[4]<<","<<values[5]<<"\n"<<endl;
	robot::JointValueMap state_joint_values;
	state_joint_values["joint0"] = values[0]*D2R;
	state_joint_values["joint1"] = values[1]*D2R;
	state_joint_values["joint2"] = values[2]*D2R;
	state_joint_values["joint3"] = values[3]*D2R;
	state_joint_values["joint4"] = values[4]*D2R;
	state_joint_values["joint5"] = values[5]*D2R;
  // update the robot joints:
  gvl->setRobotConfiguration("myUrdfRobot", state_joint_values);
  // insert the robot into the map:
  gvl->insertRobotIntoMap("myUrdfRobot", "myRobotMap",(eBVM_OCCUPIED));
      gvl->visualizeMap("countingVoxelList");


  sleep(0.1);
}
}

/* ROWS */
#define ROW_MODE_CURRENT        6
#define ROW_MODE_POSITION       7

#define ROW_TORQUE_ON_OFF       10

#define ROW_CTRL_OPEN           13
#define ROW_CTRL_CLOSE          14
#define ROW_CTRL_REPEAT         15
#define ROW_CTRL_GOAL_POSITION  16

#define ROW_GOAL_PWM            19
#define ROW_GOAL_CURRENT        20
#define ROW_GOAL_VELOCITY       21
#define ROW_GOAL_POSITION       22

/* COLS */
#define COL_CHECK               5
#define COL_VALUE               23

/* CONTROL TABLE ADDRESS */
#define ADDR_OPERATING_MODE     11
#define ADDR_TORQUE_ENABLE      512
#define ADDR_GOAL_PWM           548
#define ADDR_GOAL_CURRENT       550
#define ADDR_GOAL_VELOCITY      552
#define ADDR_GOAL_POSITION      564
#define ADDR_MOVING             570

/* VALUE LIMIT */
#define MIN_POSITION            0
#define MAX_POSITION            1150

#define MIN_PWM                 0
#define MAX_PWM                 2009

#define MIN_VELOCITY            0
#define MAX_VELOCITY            2970

#define MIN_CURRENT             0
#define MAX_CURRENT             1984


#define PROTOCOL_VERSION        2.0

#define GRIPPER_ID              1
#define BAUDRATE                2000000

#if defined(__linux__)
#define DEVICE_NAME             "/dev/ttyUSB0"
#elif defined(_WIN32) || defined(_WIN64)
#define DEVICE_NAME             "COM4"
#endif

enum MODE {
  MODE_CURRENT_CTRL = 0,
  MODE_POSITION_CTRL = 5
};

enum CONTROL {
  CTRL_NONE,
  CTRL_REPEAT,
  CTRL_OPEN,
  CTRL_CLOSE,
  CTRL_POSITION
};


int g_curr_row            = ROW_MODE_POSITION;
int g_curr_col            = COL_CHECK;

MODE g_curr_mode          = MODE_POSITION_CTRL;
bool g_is_torque_on       = false;
CONTROL g_curr_control    = CTRL_NONE;


bool g_flag_goal_position = false;
bool g_flag_auto_repeat   = false;

bool g_flag_repeat_thread = false;

int g_goal_position       = 740;
int g_goal_velocity       = 2970;
int g_goal_pwm            = 2009;
int g_goal_current        = 350;
dynamixel::PacketHandler  *g_packet_handler = NULL;
dynamixel::PortHandler    *g_port_handler   = NULL;

void closeGripper(){

	   g_packet_handler->write2ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_GOAL_CURRENT, 500);

}
void openGripper(){
		g_packet_handler->write2ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_GOAL_CURRENT, -500);

}
int gripperSetup(){
	    g_packet_handler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
 	 char *devName = (char*)DEVICE_NAME;

	  g_port_handler = dynamixel::PortHandler::getPortHandler(devName);
if (g_port_handler->openPort())
  {
    printf("Succeeded to open port.\n");

    if (g_port_handler->setBaudRate(BAUDRATE))
    {
      printf("Succeeded to change the baudrate.\n");
      printf(" - Device Name : %s\n", devName);
      printf(" - Baudrate    : %d\n\n", g_port_handler->getBaudRate());
    }
    else
    {
      printf("Failed to change the baudrate.\n");
      printf("Press any key to terminate...\n");
      return 0;
    }
  }
  else
  {
    printf("Failed to open port.\n");
    printf("Press any key to terminate...\n");
    return 0;
  }

  if (g_packet_handler->ping(g_port_handler, GRIPPER_ID) != COMM_SUCCESS)
  {
    printf("Failed to connect the gripper (ID:%d).\n", GRIPPER_ID);
    printf("Press any key to terminate...\n");
    return 0;
  }

  uint8_t _mode;
  g_packet_handler->read1ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_OPERATING_MODE, &_mode);
  g_curr_mode = (MODE)_mode;

  if (g_curr_mode == MODE_POSITION_CTRL)
    g_packet_handler->write2ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_GOAL_CURRENT, g_goal_current);
  
  g_packet_handler->write1ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_TORQUE_ENABLE, 1);
}
void doVis(){

	my_class_ptr->doVis();
	my_class_ptr->moveObstacle();
}
int main(int argc, char **argv)
{
signal(SIGINT, ctrlchandler);
  signal(SIGTERM, killhandler);

    icl_core::logging::initialize(argc, argv);
	//---------------------Indy setup-----------------------------------------//
    cout << "Connecting to the robot" << endl;
	connector.connect();
	cout << "--------------------------Indy7 OK-----------------------------" << endl;
	bool ready;
    connector.isRobotReady(ready);


	if (ready) {
	    connector.setJointBoundaryLevel(5);
        cout << "---------------------Indy7 Robot is ready-------------------" << endl;
        bool ishome;
        connector.isHome(ishome);
        if (!ishome){
        	connector.moveJointHome();
       		WaitFinish(connector);	

        } 

    }
	//--------------------------------------------------------------------------//



    //----------------------------gripper--------------------------------------//
    gripperSetup();

   	openGripper();




    PERF_MON_INITIALIZE(100, 1000);
    PERF_MON_ENABLE("planning");

    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(6));
    //We then set the bounds for the R3 component of this state space:
    ob::RealVectorBounds bounds(6);
    bounds.setLow(-3.14159265);
    bounds.setHigh(3.14159265);
 
    space->setBounds(bounds);
    //Create an instance of ompl::base::SpaceInformation for the state space
    auto si(std::make_shared<ob::SpaceInformation>(space));
    //Set the state validity checker
    std::shared_ptr<GvlOmplPlannerHelper> my_class_ptr(std::make_shared<GvlOmplPlannerHelper>(si));
    my_class_ptr->doVis();
    thread t1{&GvlOmplPlannerHelper::rosIter ,my_class_ptr};    
    thread t2(jointStateCallback);
	thread t3(doVis);


    sleep(10);

   States state = READY;
   int toggle = 1;


	double task_goal_values00[6] ={0.0, PI, 0,0.454,-0.06,0.415};
    doTaskPlanning(task_goal_values00, connector);
	double task_goal_values01[6] ={0.0, PI, 0,0.23,0.3,0.47};
    doTaskPlanning(task_goal_values01, connector);
	double task_goal_values02[6] ={0.0, PI, 0,0.23,0.3,0.18};
    doTaskPlanning(task_goal_values02, connector);
	closeGripper();

	sleep(1);
    doTaskPlanning(task_goal_values01, connector);
    doTaskPlanning(task_goal_values00, connector);
	if (ready) {
        cout << "---------------------Indy7 Robot is ready-------------------" << endl;
        bool ishome;
        connector.isHome(ishome);
        if (!ishome){
        	connector.moveJointHome();
       		WaitFinish(connector);	

        } 

    }
   	double task_goal_values[6] ={PI, PI/2, PI,0.50,-0.13,0.415};
	doTaskPlanning(task_goal_values, connector);
	double task_goal_values05[6] ={PI, PI/2, PI,0.55,0.0,0.415};
	doTaskPlanning(task_goal_values05, connector);
	cout<<"END first Task"<<endl;
	double task_goal_values2[6] ={PI, PI/2, PI,0.65,0.1965,0.41558};
	doTaskPlanning(task_goal_values2, connector);
	double task_goal_values4[6] ={PI, PI/2, PI,0.8,0.1965,0.41558};
	doTaskPlanning(task_goal_values4, connector);
   	openGripper();
    sleep(1);
	double task_goal_values3[6] ={PI, PI/2, PI,0.65,0.1965,0.41558};
	doTaskPlanning(task_goal_values3, connector);
	double task_goal_values7[6] ={PI, PI/2, PI,0.65,0.0,0.41558};
	doTaskPlanning(task_goal_values7, connector);
	doTaskPlanning(task_goal_values, connector);


	cout<<"END second Task"<<endl;

//---------------Indy disconnect---------------------//
	if (ready) {
        cout << "---------------------Indy7 Robot is ready-------------------" << endl;
        bool ishome;
        connector.isHome(ishome);
        if (!ishome){
        	connector.moveJointHome();
       		WaitFinish(connector);	

        } 

    }
   	openGripper();
   	sleep(1);
    connector.disconnect();
//----------------------------------------------------//
    t1.join();
    t2.join();
    t3.join();
    return 1;
}


