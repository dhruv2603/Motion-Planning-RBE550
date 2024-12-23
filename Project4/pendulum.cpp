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
#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

#include <ompl/tools/benchmark/Benchmark.h>

// Your implementation of RG-RRT
#include "RG-RRT.h"

// Your projection for the pendulum
class PendulumProjection : public ompl::base::ProjectionEvaluator
{
public:
    PendulumProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the pendulum
        return 1; //projecting to 1D - just the angle
    }

    void project(const ompl::base::State *state/* state */, Eigen::Ref<Eigen::VectorXd> projection/* projection */) const override
    {
        // TODO: Your projection for the pendulum
        // Cast the state to the appropriate type
        const auto *pendulum = state->as<ompl::base::CompoundStateSpace::StateType>();
        const auto *theta      = pendulum->as<ompl::base::SO2StateSpace::StateType>(0);

        // Set the projection to the angle
        projection(0) = theta->value;
    }
};

bool isStateValid(const ompl::control::SpaceInformation *si, const ompl::base::State *state)
 {
    //    ob::ScopedState<ob::SE2StateSpace>
    // cast the abstract state type to the type we expect
    const auto *pendulum = state->as<ompl::base::CompoundStateSpace::StateType>();
  
    // extract the first component of the state and cast it to what we expect
    const auto *theta = pendulum->as<ompl::base::SO2StateSpace::StateType>(0);
  
    // extract the second component of the state and cast it to what we expect
    const auto *w = pendulum->as<ompl::base::RealVectorStateSpace::StateType>(1);
  
    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return si->satisfiesBounds(state) && (const void*)theta != (const void*)w;
 }

 void propagate(const ompl::base::State *start, const ompl::control::Control *control, const double duration, ompl::base::State *result)
 {
    const auto *pendulum = start->as<ompl::base::CompoundStateSpace::StateType>();
    const double theta = pendulum->as<ompl::base::SO2StateSpace::StateType>(0)->value; // Get theta
    const double w = pendulum->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0]; // Get omega

    const double g = 9.81;

    const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double torque = u[0];

    const double thetadot = w;
    const double wdot = -g*cos(theta) + torque;

    auto *pendulumResult = result->as<ompl::base::CompoundStateSpace::StateType>();

    pendulumResult->as<ompl::base::SO2StateSpace::StateType>(0)->value = theta + thetadot * duration; // Update theta
    pendulumResult->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0] = w + wdot * duration; // Update omega
 }

void postPropagate(const ompl::base::State* state, const ompl::control::Control* control, const double duration, ompl::base::State* result)
{
    ompl::base::SO2StateSpace SO2;
 
    // Ensure that the pendulum's resulting orientation lies between 0 and 2*pi.
    ompl::base::CompoundStateSpace::StateType& s = *result->as<ompl::base::CompoundStateSpace::StateType>();
    SO2.enforceBounds(s[0]);
}

void pendulumODE(const ompl::control::ODESolver::StateType &q/* q */, const ompl::control::Control *c/* control */,
                 ompl::control::ODESolver::StateType &qdot/* qdot */)
{
    // TODO: Fill in the ODE for the pendulum's dynamics
    const double *u  = c->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double torque   = u[0];

    const double theta = q[0];
    const double w     = q[1];
    const double g     = 9.81;

    qdot.resize(q.size(),0);

    qdot[0] = w;
    qdot[1] = -g*cos(theta) + torque;
}

ompl::control::SimpleSetupPtr createPendulum(double torque/* torque */)
{
    // TODO: Create and setup the pendulum's state space, control space, validity checker, everything you need for
    // planning.
    auto space = std::make_shared<ompl::base::CompoundStateSpace>();

    auto theta = std::make_shared<ompl::base::SO2StateSpace>();
    auto w     = std::make_shared<ompl::base::RealVectorStateSpace>(1);
    
    ompl::base::RealVectorBounds bounds(1);
    bounds.setLow(-10);
    bounds.setHigh(10);
    w->setBounds(bounds);

    space->addSubspace(theta,1.0);  
    space->addSubspace(w,1.0);

    auto con_space = std::make_shared<ompl::control::RealVectorControlSpace>(space,1);
    ompl::base::RealVectorBounds cbounds(1);
    cbounds.setLow(-torque);
    cbounds.setHigh(torque);
    con_space->setBounds(cbounds);

    ompl::control::SimpleSetupPtr ss = std::make_shared<ompl::control::SimpleSetup>(con_space);
    ompl::control::ODESolverPtr odeSolver (new ompl::control::ODEBasicSolver<> (ss->getSpaceInformation(), &pendulumODE));
    
    // set the state propagation routine
    // ss->setStatePropagator(propagate);
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));
    // ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver, &postPropagate));

    // ompl::base::StateSpace stateSpace = &ss->getStateSpace();
    // stateSpace->registerProjection("myProjection", std::make_shared<PendulumProjection>(ss->getStateSpace()));
    space->registerProjection("myProjection", ompl::base::ProjectionEvaluatorPtr(new PendulumProjection(space.get())));
    // set state validity checking for this space
    ss->setStateValidityChecker(
        [&ss](const ompl::base::State *state) { return isStateValid(ss->getSpaceInformation().get(), state); });
    
    // create a start state
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> start(space);
    start[0] = -M_PI/2;
    start[1] = 0.0;
  
    // create a  goal state; use the hard way to set the elements
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> goal(space);
    goal[0] = M_PI/2;
    goal[1] = 0.0;
  
  
    // set the start and goal states
    // ss->setStartState(start);
    // ss->setGoalState(goal);
    ss->setStartAndGoalStates(start,goal,0.001);

    return ss;
}

void planPendulum(ompl::control::SimpleSetupPtr &ss/* ss */, int choice/* choice */)
{
    // TODO: Do some motion planning for the pendulum
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

    ss->getSpaceInformation()->setMinMaxControlDuration(1,10);
    ss->getSpaceInformation()->setPropagationStepSize(0.05);
    ompl::base::PlannerStatus solved = ss->solve(30.0);
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ompl::control::PathControl &path = ss->getSolutionPath();
        path.interpolate();
        path.asGeometric().printAsMatrix(std::cout);


        std::ofstream output("path.txt");
        output << "R2" << std::endl;
        path.asGeometric().printAsMatrix(output);
        output.close();
    }
    else
        std::cout << "No solution found" << std::endl;
}


void benchmarkPendulum(ompl::control::SimpleSetupPtr &ss/* ss */)
{
    // TODO: Do some benchmarking for the pendulum
    double runtime_limit = 60, memory_limit = 1024;
    int run_count = 20;
    ss->setup();
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.5);
    ompl::tools::Benchmark b(*ss.get(), "Pendulum_Benchmarking");

    b.addPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
    auto kpiecePlanner = std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation());
    kpiecePlanner->setProjectionEvaluator("myProjection");
    b.addPlanner(kpiecePlanner);
    b.addPlanner(std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation()));

    // Run the benchmark
    b.benchmark(request);

    // Save results to a file
    b.saveResultsToFile("benchmark_pendulum.log");
}

int main(int /* argc */, char ** /* argv */)
{
    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    int which;
    do
    {
        std::cout << "Torque? " << std::endl;
        std::cout << " (1)  3" << std::endl;
        std::cout << " (2)  5" << std::endl;
        std::cout << " (3) 10" << std::endl;

        std::cin >> which;
    } while (which < 1 || which > 3);

    double torques[] = {3., 5., 10.};
    double torque = torques[which - 1];

    ompl::control::SimpleSetupPtr ss = createPendulum(torque);

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

        planPendulum(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkPendulum(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
