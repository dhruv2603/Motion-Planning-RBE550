///////////////////////////////////////
// RBE 550
// Project 4
// Authors: Dhruv, Shreyas
//////////////////////////////////////

#include <iostream>
#include <fstream>

#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

#include <ompl/tools/benchmark/Benchmark.h>


// The collision checker produced in project 2
#include "CollisionChecking.h"

// Your implementation of RG-RRT
#include "RG-RRT.h"

// Your projection for the car
class CarProjection : public ompl::base::ProjectionEvaluator
{
public:
    CarProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the car
        return 2; //projecting to 2D - the position of car in cartesian space
    }

    void project(const ompl::base::State *state /* state */, Eigen::Ref<Eigen::VectorXd> projection/* projection */) const override
    {
        // TODO: Your projection for the car
        const auto *carState = state->as<ompl::base::CompoundState>();
        const auto *carSE2   = carState->as<ompl::base::SE2StateSpace::StateType>(0);
        // const auto *r2State  = carSE2->as<ompl::base::RealVectorStateSpace::StateType>(0);
        // Set the projection to the angle
        projection(0) = carSE2->getX();//r2State->values[0];
        projection(1) = carSE2->getY();//r2State->values[1];
    }
};

//State Validity Checker for checking if square robot is in collision
bool isValidStateSquare(const ompl::base::State *state, const std::vector<Rectangle> &obstacles, const ompl::control::SpaceInformationPtr si)
{
    const auto *cpace = si->getStateSpace()->as<ompl::base::CompoundStateSpace>();
    const auto *space = cpace->as<ompl::base::SE2StateSpace>(0);

    auto cstate = state->as<ompl::base::CompoundStateSpace::StateType>();
    auto se2 = cstate->as<ompl::base::SE2StateSpace::StateType>(0);
    auto v = cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);

    double x = se2->getX();
    double y = se2->getY();
    double theta = se2->getYaw();
    double sideLength = 0.1;

    double hbound = space->getBounds().high[0];
    double lbound = space->getBounds().low[0];
    double velbound = 3.0;

    if(v->values[0] > velbound)
    {
        return false;

    }
    else if(v->values[0] < -velbound)
    {  
        return false;
    }

    if (x < lbound || x > hbound || y < lbound || y > hbound)
    {
        return false;
    }   
    bool is_ok  = isValidSquare(x, y, theta, sideLength, obstacles);//isValidPoint(x, y, obstacles);

    return is_ok;//isValidSquare(x, y, theta, sideLength, obstacles);
}

void postPropagate(const ompl::base::State* state, const ompl::control::Control* control, const double duration, ompl::base::State* result)
{
    ompl::base::SO2StateSpace SO2;
 
    // Ensure that the car's resulting orientation lies between 0 and 2*pi.
    ompl::base::CompoundStateSpace::StateType* s = result->as<ompl::base::CompoundStateSpace::StateType>();
    SO2.enforceBounds (s->as<ompl::base::SE2StateSpace::StateType>(0)->as<ompl::base::SO2StateSpace::StateType>(1));
}

void carODE(const ompl::control::ODESolver::StateType &q /* q */, const ompl::control::Control *c /* control */,
            ompl::control::ODESolver::StateType &qdot /* qdot */)
{
    // TODO: Fill in the ODE for the car's dynamics
    const double *u  = c->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double w   = u[0];
    const double acc = u[1];

    const double theta = q[2];
    const double v     = q[3];

    qdot.resize(q.size(),0);

    qdot[0] = v*cos(theta);
    qdot[1] = v*sin(theta);
    qdot[2] = u[0];//w;
    qdot[3] = u[1];//acc;
}

void makeStreet(std::vector<Rectangle> & obstacles/* obstacles */)
{
    // TODO: Fill in the vector of rectangles with your street environment.
    Rectangle obstacle1;
	obstacle1.x = 0;
	obstacle1.y = 3;
	obstacle1.width = 2;
	obstacle1.height = 1;
	obstacles.push_back(obstacle1);

	Rectangle obstacle2;
	obstacle2.x = 1;
	obstacle2.y = 5;
	obstacle2.width = 1;
	obstacle2.height = 2;
	obstacles.push_back(obstacle2);

    Rectangle obstacle3;
	obstacle3.x = 1;
	obstacle3.y = 0;
	obstacle3.width = 1;
	obstacle3.height = 2;
	obstacles.push_back(obstacle3);

	Rectangle obstacle4;
	obstacle4.x = 3;
	obstacle4.y = 2;
	obstacle4.width = 1;
	obstacle4.height = 3;
	obstacles.push_back(obstacle4);

    Rectangle obstacle5;
	obstacle5.x = 5;
	obstacle5.y = 4;
	obstacle5.width = 1;
	obstacle5.height = 3;
	obstacles.push_back(obstacle5);

    Rectangle obstacle6;
	obstacle6.x = 5;
	obstacle6.y = 0;
	obstacle6.width = 1;
	obstacle6.height = 3;
	obstacles.push_back(obstacle6);

    std::ofstream output("obstacles.txt");
    for (const auto& obstacle : obstacles) {
        output << obstacle.x << " " 
                   << obstacle.y << " " 
                   << obstacle.width << " " 
                   << obstacle.height << std::endl;
    }
    output.close();
}

ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> &obstacles /* obstacles */)
{
    // TODO: Create and setup the car's state space, control space, validity checker, everything you need for planning.
    auto space = std::make_shared<ompl::base::CompoundStateSpace>();
    auto se2   = std::make_shared<ompl::base::SE2StateSpace>();
    auto v     = std::make_shared<ompl::base::RealVectorStateSpace>(1);
    
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(7);
    se2->setBounds(bounds);

    ompl::base::RealVectorBounds vbounds(1);
    vbounds.setLow(-3);
    vbounds.setHigh(3);
    v->setBounds(vbounds);

    space->addSubspace(se2,1.0);  
    space->addSubspace(v,1.0);

    auto con_space = std::make_shared<ompl::control::RealVectorControlSpace>(space,2);
    
    ompl::base::RealVectorBounds cbounds(2); // Create bounds for 2D space
    cbounds.setLow(0,-M_PI);  // Set lower bounds for both dimensions
    cbounds.setHigh(0,M_PI);   // Set upper bounds for both dimensions
    cbounds.setLow(1,-0.5);  // Set lower bounds for both dimensions
    cbounds.setHigh(1,0.5);   // Set upper bounds for both dimensions
    con_space->setBounds(cbounds);

    ompl::control::SimpleSetupPtr ss = std::make_shared<ompl::control::SimpleSetup>(con_space);
    ompl::control::ODESolverPtr odeSolver (new ompl::control::ODEBasicSolver<> (ss->getSpaceInformation(), &carODE));
    
    // set the state propagation routine
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver, postPropagate));

    space->registerProjection("myProjection", ompl::base::ProjectionEvaluatorPtr(new CarProjection(space.get())));
    // Set state validity checking for this space
    ss->setStateValidityChecker([=](const ompl::base::State* state) {
        return isValidStateSquare(state, obstacles, ss->getSpaceInformation());
        });
    
    // create a start state
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> start(space);
    start[0] = 0.5;
    start[1] = 4.5;
    start[2] = 0.0;
    start[3] = 1.5;
  
    // create a  goal state; use the hard way to set the elements
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> goal(space);
    goal[0] = 6.5;
    goal[1] = 1.0;
    goal[2] = 0.0;
    goal[3] = 1.5;

  
    // set the start and goal states
    ss->setStartAndGoalStates(start,goal,0.05);
    return ss;
    // return nullptr;
}

void planCar(ompl::control::SimpleSetupPtr &ss/* ss */, int choice/* choice */)
{
    // TODO: Do some motion planning for the car
    // choice is what planner to use.
    ompl::base::PlannerPtr planner;
    switch (choice)
    {
    case 1/* constant-expression */:
        /* RRT */
        /* code */
        //Create a planner
        // Set the problem to be solved in the planner
        ss->setPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
        break;
    
    case 2/* constant-expression */:
        /* KPIECE1 */
        /* code */
        planner = std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation());
        planner->as<ompl::control::KPIECE1>()->setProjectionEvaluator("myProjection");
        ss->setPlanner(planner);
        break;
    
    case 3/* constant-expression */:
        /* RG-RRT */
        /* code */
        planner = std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation());
        ss->setPlanner(planner);
        break;
    
    default:
        break;
    }

    ss->getSpaceInformation()->setPropagationStepSize(0.2);
    ss->setup();
    ompl::base::PlannerStatus solved = ss->solve(30.0);
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ompl::control::PathControl &path = ss->getSolutionPath();
        path.interpolate();
        path.asGeometric().printAsMatrix(std::cout);


        std::ofstream output("path.txt");
        output << "SE2" << " " << 0.1 << std::endl;
        path.asGeometric().printAsMatrix(output);
        output.close();
    }
    else
        std::cout << "No solution found" << std::endl;
}

void benchmarkCar(ompl::control::SimpleSetupPtr &ss/* ss */)
{
    // TODO: Do some benchmarking for the car
    double runtime_limit = 60, memory_limit = 1024;
    int run_count = 20;
    // ss->getSpaceInformation()->setPropagationStepSize(0.5);
    ss->setup();
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.5);
    ompl::tools::Benchmark b(*ss.get(), "Car_Benchmarking");

    // b.addPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
    // auto kpiecePlanner = std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation());
    // kpiecePlanner->setProjectionEvaluator("myProjection");
    // b.addPlanner(kpiecePlanner);
    b.addPlanner(std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation()));

    // Run the benchmark
    b.benchmark(request);

    // Save results to a file
    b.saveResultsToFile("benchmark_car_nc3.log");
}

int main(int /* argc */, char ** /* argv */)
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);
    ompl::control::SimpleSetupPtr ss = createCar(obstacles);

    // Planning
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planCar(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkCar(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
