#include <iostream>
#include <fstream>
#include <cmath>
#include <functional>
#include <memory>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include "CollisionChecking.h"
#include "RTP.h"

//Including namespaces
namespace ob = ompl::base;
namespace og = ompl::geometric;

//State Validity Checker for checking if point robot is in collision
bool isValidStatePoint(const ob::State *state, const std::vector<Rectangle> & obstacles)
{
    auto r2state = state->as<ob::RealVectorStateSpace::StateType>();

    double x = r2state->values[0];
    double y = r2state->values[1];

    return isValidPoint(x, y, obstacles);
}

//State Validity Checker for checking if square robot is in collision
bool isValidStateSquare(const ob::State *state, const std::vector<Rectangle> &obstacles, const ompl::base::RealVectorBounds &bounds)
{
    auto cstate = state->as<ob::SE2StateSpace::StateType>();

    double x = cstate->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
    double y = cstate->as<ob::RealVectorStateSpace::StateType>(0)->values[1];
    double theta = cstate->as<ob::SO2StateSpace::StateType>(1)->value;
    double sideLength = 0.1;
    double hbound = bounds.high[0];
    double lbound = bounds.low[0];
    
    return isValidSquare(x, y, theta, sideLength, obstacles, hbound, lbound);
}

void planPoint(const std::vector<Rectangle> & obstacles )
{
    // TODO: Plan for a point robot in the plane.
    std::cout<<"Number of obstacles present "<<obstacles.size()<<std::endl;

    // Construct the state space
    auto r2 = std::make_shared<ompl::base::RealVectorStateSpace>(2);
    // Set the bounds of the space
    ob:: RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(4);

    r2->setBounds(bounds);
    
    // Simple Setup initializes many default settings
    og::SimpleSetup space(r2);

    

    // Set state validity checking for this space
    space.setStateValidityChecker([=](const ompl::base::State* state) {
        return isValidStatePoint(state, obstacles);
    });

    // Create a start state
    ob::ScopedState<> start(r2);
    start[0] = 0.2;
    start[1] = 0.2;

    // Create a goal state
    ob::ScopedState<> goal(r2);
    goal[0] = 3.5;
    goal[1] = 0.5;

    // Set the start and goal states in the pdef instance
    space.setStartAndGoalStates(start,goal);

    // Create a planner
    auto planner = std::make_shared<og::RTP>(space.getSpaceInformation());

    // Set the problem to be solved in the planner
    space.setPlanner(planner);

    //Setup the Space
    space.setup();
    space.print();

    // Attempt to solve the problem within time limits
    ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

    if (solved)
    {
        /* get the goal representation from the problem definition (not the same as the goal state)
           and inquire about the found path*/
        std::cout << "Found solution:" << std::endl;
        og::PathGeometric &path = space.getSolutionPath();
        path.interpolate(50);

        // print the path to screen
        path.printAsMatrix(std::cout);

        
        /*CHANGE THIS NAME TO "path2.txt" IF USING Environment 2 and path1.txt IF USING ENVIRONMENT 1*/
        std::ofstream output("path1.txt");
        output << "R2" << std::endl;
        path.printAsMatrix(output);
        output.close();
    }
    else
        std::cout << "No solution found" << std::endl;
}

void planBox(const std::vector<Rectangle> &  obstacles)
{
    // TODO: Plan for a square_box that rotates and translates in the plane.
    
    // Construct the state space
    auto se2 = std::make_shared<ob::SE2StateSpace>();
    // Set the bounds of the space
    ob:: RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(4);

    se2->setBounds(bounds);
    og::SimpleSetup space(se2);

    // Set state validity checking for this space
    space.setStateValidityChecker([=](const ompl::base::State* state) {
        return isValidStateSquare(state, obstacles, bounds);
    });

    // Create a start state
    ob::ScopedState<> start(se2);
    start[0] = 0.2;
    start[1] = 0.2;
    start[2] = 0.0;

    // Create a goal state
    ob::ScopedState<> goal(se2);
    goal[0] = 3.5;
    goal[1] = 0.5;
    goal[2] = 0.0;

    // Set the start and goal states in the pdef instance
    space.setStartAndGoalStates(start,goal);

    // Create a planner
    auto planner = std::make_shared<og::RTP>(space.getSpaceInformation());

    // Setup the planner
    space.setPlanner(planner);


    
    // Attempt to solve the problem within time limits
    ob::PlannerStatus solved = space.solve(1.0);

    if (solved)
    {
        /* get the goal representation from the problem definition (not the same as the goal state)
           and inquire about the found path*/
        std::cout << "Found solution:" << std::endl;
        og::PathGeometric &path = space.getSolutionPath();
        path.interpolate(50);
        
        // print the path to screen
        path.printAsMatrix(std::cout);

        
        /*CHANGE THIS NAME TO "path4.txt" IF USING Environment 2 and path3.txt IF USING ENVIRONMENT 1*/
        std::ofstream output("path4.txt");
        output << "SE2" << " " << 0.2 << std::endl;
        path.printAsMatrix(output);
        output.close();
    }
    else
        std::cout << "No solution found" << std::endl;
}

void makeEnvironment1(std::vector<Rectangle> &  obstacles )
{

    // TODO: Fill in the vector of rectangles with your second environment.
    Rectangle obstacle1;
	obstacle1.x = 1;
	obstacle1.y = 0;
	obstacle1.width = 0.5;
	obstacle1.height = 3;
	obstacles.push_back(obstacle1);

	Rectangle obstacle2;
	obstacle2.x = 2;
	obstacle2.y = 0;
	obstacle2.width = 0.5;
	obstacle2.height = 2;
	obstacles.push_back(obstacle2);

    std::cout<<"After making environment "<<obstacles.size()<<std::endl;

    /*Store in a file called obstacles1.txt*/
    std::ofstream output("obstacles1.txt");
    for (const auto& obstacle : obstacles) {
        output << obstacle.x << " " 
                   << obstacle.y << " " 
                   << obstacle.width << " " 
                   << obstacle.height << std::endl;
    }
    output.close();
}

void makeEnvironment2(std::vector<Rectangle> &  obstacles )
{
    // TODO: Fill in the vector of rectangles with your second environment.
    Rectangle obstacle1;
	obstacle1.x = 0.5;
	obstacle1.y = 0;
	obstacle1.width = 0.5;
	obstacle1.height = 1;
	obstacles.push_back(obstacle1);

	Rectangle obstacle2;
	obstacle2.x = 1.5;
	obstacle2.y = 0;
	obstacle2.width = 0.5;
	obstacle2.height = 2;
	obstacles.push_back(obstacle2);

    Rectangle obstacle3;
	obstacle3.x = 2.5;
	obstacle3.y = 2.5;
	obstacle3.width = 0.5;
	obstacle3.height = 0.5;
	obstacles.push_back(obstacle3);

	Rectangle obstacle4;
	obstacle4.x = 0.5;
	obstacle4.y = 2.5;
	obstacle4.width = 0.5;
	obstacle4.height = 0.5;
	obstacles.push_back(obstacle4);

    Rectangle obstacle5;
	obstacle5.x = 2.5;
	obstacle5.y = 0;
	obstacle5.width = 0.5;
	obstacle5.height = 1;
	obstacles.push_back(obstacle5);

    /*Store in a file called obstacles2.txt*/
    std::ofstream output("obstacles2.txt");
    for (const auto& obstacle : obstacles) {
        output << obstacle.x << " " 
                   << obstacle.y << " " 
                   << obstacle.width << " " 
                   << obstacle.height << std::endl;
    }
    output.close();

}

int main(int /* argc */, char ** /* argv */)
{
    int robot, choice;
    std::vector<Rectangle> obstacles;

    do
    {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) A point in 2D" << std::endl;
        std::cout << " (2) A rigid box in 2D" << std::endl;

        std::cin >> robot;
    } while (robot < 1 || robot > 2);

    do
    {
        std::cout << "In Environment: " << std::endl;
        std::cout << " (1) TODO" << std::endl;
        std::cout << " (2) TODO" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    switch (choice)
    {
        case 1:
            makeEnvironment1(obstacles);
            break;
        case 2:
            makeEnvironment2(obstacles);
            break;
        default:
            std::cerr << "Invalid Environment Number!" << std::endl;
            break;
    }

    switch (robot)
    {
        case 1:
            planPoint(obstacles);
            break;
        case 2:
            planBox(obstacles);
            break;
        default:
            std::cerr << "Invalid Robot Type!" << std::endl;
            break;
    }

    return 0;
}