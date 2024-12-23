#include "KinematicChain.h"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/BridgeTestValidStateSampler.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
///////////////////////////////////////////////////////////////////////////////////////////
class ClearanceObjective : public ompl::base::StateCostIntegralObjective
{
public:
    ClearanceObjective(const ompl::base::SpaceInformationPtr& si) :
        ompl::base::StateCostIntegralObjective(si, true)
    {
    }
 
    ompl::base::Cost stateCost(const ompl::base::State* s) const
    {
        return ompl::base::Cost(1 / si_->getStateValidityChecker()->clearance(s));
    }
};

ompl::base::OptimizationObjectivePtr getClearanceObjective(const ompl::base::SpaceInformationPtr &si)
{
    return ompl::base::OptimizationObjectivePtr(std::make_shared<ClearanceObjective>(si));
}


ompl::base::ValidStateSamplerPtr getUniformSampler(const ompl::base::SpaceInformation *si)
{
    return std::make_shared<ompl::base::UniformValidStateSampler>(si);
}
ompl::base::ValidStateSamplerPtr getGaussianValidStateSampler(const ompl::base::SpaceInformation* si)
{
    return std::make_shared<ompl::base::GaussianValidStateSampler>(si);
}
ompl::base::ValidStateSamplerPtr getBridgeTestValidStateSampler(const ompl::base::SpaceInformation* si)
{
    return std::make_shared<ompl::base::BridgeTestValidStateSampler>(si);
}
ompl::base::ValidStateSamplerPtr getObstacleBasedValidStateSampler(const ompl::base::SpaceInformation* si)
{
    return std::make_shared<ompl::base::ObstacleBasedValidStateSampler>(si);
}


class ValidityChecker : public ompl::base::StateValidityChecker
{
public:
    ValidityChecker(const ompl::base::SpaceInformationPtr &si, const Environment &env) : ompl::base::StateValidityChecker(si), env_(env)
    {
    }
    bool isValid(const ompl::base::State *state) const override
    {
        const auto cstate = state->as<ompl::base::CompoundStateSpace::StateType>();
        const auto r = cstate->as<ompl::base::SE2StateSpace::StateType>(0);
        const auto s = cstate->as<KinematicChainSpace::StateType>(1);
        // Environment envi = env_;
        return isValidSetup(env_, s, r);
    }

    /* PLEASE UNCOMMENT THIS TO RUN Q3 clearance function!!!!!!!! 
    AND COMMENT THE BELOW clearance function*/
    ////////////////////////////////////////

    //  // Only calculates distances from robot base and NOT from chain links
    // double clearance(const ompl::base::State* state) const override
    // {   
    //     const auto cstate = state->as<ompl::base::CompoundStateSpace::StateType>();
    //     const auto r = cstate->as<ompl::base::SE2StateSpace::StateType>(0);
    //     const auto s = cstate->as<KinematicChainSpace::StateType>(1);

    //     double robotX = r->getX();
    //     double robotY = r->getY();
    //     double minClearance = std::numeric_limits<double>::max();

    //     // Calculate clearance for the robot base
    //     for (const auto& obstacle : env_)
    //     {
    //         double distanceToObstacle = computeDistanceToCorner(robotX, robotY, obstacle);
    //         minClearance = std::min(minClearance, distanceToObstacle);
    //     }
        
    //     // std::cout<<"clearance : "<<std::max(minClearance, 0.0)<<std::endl;
    //     return std::max(minClearance, 0.0);
    // }
    /////////////////////////////////////////////

    /* PLEASE UNCOMMENT THIS TO RUN BONUS clearance function!!!!!!!! 
    AND COMMENT THE ABOVE clearance function*/
    /////////////////////////////////////////////
    double clearance(const ompl::base::State* state) const
    {
        const auto cstate = state->as<ompl::base::CompoundStateSpace::StateType>();
        const auto r = cstate->as<ompl::base::SE2StateSpace::StateType>(0);
        const auto s = cstate->as<KinematicChainSpace::StateType>(1);

        double robotX = r->getX();
        double robotY = r->getY();
        double xN, yN;
        double robotTheta = r->getYaw();
        double minClearance = std::numeric_limits<double>::max();
        double distanceToObstacle;
        unsigned int n = 4; // Adjust based on the number of links
        double linkLength = 1;
        double theta = r->getYaw(), x = robotX, y = robotY;

        for (const auto& obstacle : env_)
            {
                distanceToObstacle = computeDistanceToSegment((cos(robotTheta) * (-1/2.0) - sin(robotTheta) * (-1/2.0) + robotX), (sin(robotTheta) * (-1/2.0) + cos(robotTheta) * (-1/2.0) + robotY), obstacle);
                minClearance = std::min(minClearance, distanceToObstacle);
                distanceToObstacle = computeDistanceToSegment((cos(robotTheta) * (1/2.0) - sin(robotTheta) * (-1/2.0) + robotX), (sin(robotTheta) * (1/2.0) + cos(robotTheta) * (-1/2.0) + robotY), obstacle);
                minClearance = std::min(minClearance, distanceToObstacle);
                distanceToObstacle = computeDistanceToSegment((cos(robotTheta) * (1/2.0) - sin(robotTheta) * (1/2.0) + robotX), (sin(robotTheta) * (1/2.0) + cos(robotTheta) * (1/2.0) + robotY), obstacle);
                minClearance = std::min(minClearance, distanceToObstacle);
                distanceToObstacle = computeDistanceToSegment((cos(robotTheta) * (-1/2.0) - sin(robotTheta) * (1/2.0) + robotX), (sin(robotTheta) * (-1/2.0) + cos(robotTheta) * (1/2.0) + robotY), obstacle);
                minClearance = std::min(minClearance, distanceToObstacle);
            }

        for (unsigned int i = 0; i < n; ++i)
        {
            theta += s->values[i];
            xN = x + cos(theta) * linkLength;
            yN = y + sin(theta) * linkLength;

            // Check distance to obstacle for the current link endpoint
            for (const auto& obstacle : env_)
            {
                distanceToObstacle = computeDistanceToSegment(xN, yN, obstacle);
                minClearance = std::min(minClearance, distanceToObstacle);
            }

            x = xN;
            y = yN;
        }

        return std::max(minClearance, 0.0); 

    }
    /////////////////////////////////////////////
private:
    const Environment& env_; // Store the environment as a member variable
    
    // return true iff segment s0 intersects segment s1
    bool intersectionTest(const Segment &s0, const Segment &s1) const
    {
            // adopted from:
            // http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect/1201356#1201356
            double s10_x = s0.x1 - s0.x0;
            double s10_y = s0.y1 - s0.y0;
            double s32_x = s1.x1 - s1.x0;
            double s32_y = s1.y1 - s1.y0;
            double denom = s10_x * s32_y - s32_x * s10_y;
            if (fabs(denom) < std::numeric_limits<double>::epsilon())
                return false;  // Collinear
            bool denomPositive = denom > 0;

            double s02_x = s0.x0 - s1.x0;
            double s02_y = s0.y0 - s1.y0;
            double s_numer = s10_x * s02_y - s10_y * s02_x;
            if ((s_numer < std::numeric_limits<float>::epsilon()) == denomPositive)
                return false;  // No collision
            double t_numer = s32_x * s02_y - s32_y * s02_x;
            if ((t_numer < std::numeric_limits<float>::epsilon()) == denomPositive)
                return false;  // No collision
            if (((s_numer - denom > -std::numeric_limits<float>::epsilon()) == denomPositive) ||
                ((t_numer - denom > std::numeric_limits<float>::epsilon()) == denomPositive))
                return false;  // No collision
            return true;
    }
    
    // return true iff env does *not* include a pair of intersecting segments
    bool selfIntersectionTest(const Environment &env) const
    {
            for (unsigned int i = 0; i < env.size(); ++i)
                for (unsigned int j = i + 1; j < env.size(); ++j)
                    if (intersectionTest(env[i], env[j]))
                        return false;
            return true;
    }
    
    // return true iff no segment in env0 intersects any segment in env1
    bool environmentIntersectionTest(const Environment &env0, const Environment &env1) const
    {               
            for (const auto &i : env0)
                for (const auto &j : env1)
                    if (intersectionTest(i, j))
                        return false;
            return true;
    }

    bool boxIntersectionTest(const Environment &env0, const Environment &env1) const
    {
            for (const auto &i : env0)
                    for (unsigned int j = 1; j < env1.size(); j++)
                        if (intersectionTest(i, env1[j]))
                            return false;
            return true;
    }

    bool isValidSetup(const Environment &env, const KinematicChainSpace::StateType *s,const ompl::base::SE2StateSpace::StateType *r) const
    {

        unsigned int n = 4; //4 link chain
        Environment segments;
        double linkLength = si_->getStateSpace()->as<ompl::base::CompoundStateSpace>()->as<KinematicChainSpace>(1)->linkLength();
        double theta = r->getYaw(), x = r->getX(), y =r->getY(), xN, yN;
        ///////////////////////////////////////
        
        Environment square_segments;
        double sq_len = 1;
        double robo_x = x,robo_y = y;
        double robo_theta =  r->getYaw();
        double rx1, rx2, rx3, rx4, ry1, ry2, ry3, ry4;

        ///////////////////////////////////////

        segments.reserve(n + 1);
        for (unsigned int i = 0; i < n; ++i)
        {
            theta += s->values[i];
            xN = x + cos(theta) * linkLength;
            yN = y + sin(theta) * linkLength;
            segments.emplace_back(x, y, xN, yN);
            x = xN;
            y = yN;
        }
        xN = x + cos(theta) * 0.001;
        yN = y + sin(theta) * 0.001;
        segments.emplace_back(x, y, xN, yN);
        /////////////////////////////////////

        // Multiply points by transformation matrix
        rx1 = (cos(robo_theta) * (-sq_len/2.0) - sin(robo_theta) * (-sq_len/2.0) + robo_x);
        ry1 = (sin(robo_theta) * (-sq_len/2.0) + cos(robo_theta) * (-sq_len/2.0) + robo_y); 

        rx2 = (cos(robo_theta) * (sq_len/2.0) - sin(robo_theta) * (-sq_len/2.0) + robo_x);
        ry2 = (sin(robo_theta) * (sq_len/2.0) + cos(robo_theta) * (-sq_len/2.0) + robo_y);

        rx3 = (cos(robo_theta) * (sq_len/2.0) - sin(robo_theta) * (sq_len/2.0) + robo_x);
        ry3 = (sin(robo_theta) * (sq_len/2.0) + cos(robo_theta) * (sq_len/2.0) + robo_y);

        rx4 = (cos(robo_theta) * (-sq_len/2.0) - sin(robo_theta) * (sq_len/2.0) + robo_x);
        ry4 = (sin(robo_theta) * (-sq_len/2.0) + cos(robo_theta) * (sq_len/2.0) + robo_y);  

        square_segments.reserve(4);
        square_segments.emplace_back(rx1, ry1, rx2, ry2);
        square_segments.emplace_back(rx2, ry2, rx3, ry3);
        square_segments.emplace_back(rx3, ry3, rx4, ry4);
        square_segments.emplace_back(rx4, ry4, rx1, ry1);
        
        return (selfIntersectionTest(segments) && environmentIntersectionTest(segments, env) && boxIntersectionTest(square_segments,segments) && environmentIntersectionTest(square_segments, env));
        // return environmentIntersectionTest(segments, env);

    }
    

    double computeDistanceToSegment(double x, double y, const Segment& segment) const
    {
        double x0 = segment.x0, y0 = segment.y0;
        double x1 = segment.x1, y1 = segment.y1;

        double ABx = x1 - x0;
        double ABy = y1 - y0;

        double APx = x - x0;
        double APy = y - y0;

        double t = (APx * ABx + APy * ABy) / (ABx * ABx + ABy * ABy);
        t = std::max(0.0, std::min(1.0, t));

        double closestX = x0 + t * ABx;
        double closestY = y0 + t * ABy;

        double distX = x - closestX;
        double distY = y - closestY;
        return sqrt(distX * distX + distY * distY);
    }

    double computeDistanceToCorner(double x, double y, const Segment& segment) const
    {
        double dx = segment.x0 - x, dy = segment.y0 - y;
        return sqrt(dx * dx + dy * dy);
    }
};



//////////////////////////////////////////////////////////////////////////////////////////

void makeScenario1(Environment &env, std::vector<double> &start, std::vector<double> &goal)
{

    start.reserve(7);
    start.assign(7, 0.0);

    goal.reserve(7);
    goal.assign(7, 0.0);

    start[0] = -3;
    start[1] = -3;
    goal[0] = 2 ; 
    goal[1] = 2 ; 
    goal[2] = 0; 
    goal[4] = -0.5*M_PI;

    //Obstacle 1
    env.emplace_back(2, -1, 2.8, -1);
    env.emplace_back(2.8, -1, 2.8, 0.5);
    env.emplace_back(2.8, 0.5, 2, 0.5);
    env.emplace_back(2, 0.5, 2, -1);

    //Obstacle 2
    env.emplace_back(3.2, -1, 4, -1);
    env.emplace_back(4, -1, 4, 0.5);
    env.emplace_back(4, 0.5, 3.2, 0.5);
    env.emplace_back(3.2, 0.5, 3.2, -1);
}

void makeScenario2(Environment &env, std::vector<double> &start, std::vector<double> &goal)
{
    start.reserve(7);
    start.assign(7, 0.0);

    goal.reserve(7);
    goal.assign(7, 0.0);

    start[0] = -4;
    start[1] = -4;
    start[2] = 0;
    goal[0] = 3; 
    goal[1] = 3; 
    goal[2] = 0; 

    //Obstacle 1
    env.emplace_back(-1, -1, 1, -1);
    env.emplace_back(1, -1, 1, 1);
    env.emplace_back(1, 1, -1, 1);
    env.emplace_back(-1, 1, -1, -1);
}

void planScenario1(ompl::geometric::SimpleSetup &ss)
{
    // TODO: Plan for chain_box in the plane, and store the path in path1.txt.
    //Create a planner
    auto planner = std::make_shared<ompl::geometric::PRM>(ss.getSpaceInformation());
    // Set the problem to be solved in the planner
    ss.setPlanner(planner);

    // Attempt to solve the problem within time limits
    ompl::base::PlannerStatus solved = ss.solve(10.0);//planner->ompl::base::Planner::solve(10.0);
    if (solved)
    {
        /* get the goal representation from the problem definition (not the same as the goal state)
           and inquire about the found path*/
        std::cout << "Found solution:" << std::endl;
        ompl::geometric::PathGeometric &path = ss.getSolutionPath();
        path.interpolate(100);

        // print the path to screen
        path.printAsMatrix(std::cout);
        /*CHANGE THIS NAME TO "path2.txt" IF USING Environment 2 and path1.txt IF USING ENVIRONMENT 1*/
        std::ofstream output("narrow_path.txt");
        path.printAsMatrix(output);
        output.close();
    }
    else
        std::cout << "No solution found" << std::endl;
}

void benchScenario1(ompl::geometric::SimpleSetup &ss)
{
    //TODO: Benchmark PRM with uniform, bridge, gaussian, and obstacle-based Sampling. Do 20 trials with 20 seconds each 
    double runtime_limit = 20, memory_limit = 1024;
    int run_count = 20;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.5);
    ompl::tools::Benchmark b(ss, "ChainBox_Narrow");

    auto planner = std::make_shared<ompl::geometric::PRM>(ss.getSpaceInformation());
    ss.getSpaceInformation()->setValidStateSamplerAllocator(getUniformSampler);
    b.addPlanner(planner);
    // Run the benchmark
    b.benchmark(request);
    // Save results to a file
    b.addExperimentParameter("sampler", "STRING", "Uniform_Sampling");
    b.saveResultsToFile("ChainBox_Narrow_uniform.log");

    ss.getSpaceInformation()->setValidStateSamplerAllocator(getGaussianValidStateSampler);
    // Run the benchmark
    b.benchmark(request);
    // Save results to a file
    b.addExperimentParameter("sampler", "STRING", "Gaussian_Sampling");
    b.saveResultsToFile("ChainBox_Narrow_gaussian.log");

    ss.getSpaceInformation()->setValidStateSamplerAllocator(getBridgeTestValidStateSampler);
    // Run the benchmark
    b.benchmark(request);
    // Save results to a file
    b.addExperimentParameter("sampler", "STRING", "Bridge_Test_Sampling");
    b.saveResultsToFile("ChainBox_Narrow_bridge.log");

    ss.getSpaceInformation()->setValidStateSamplerAllocator(getObstacleBasedValidStateSampler);
    // Run the benchmark
    b.benchmark(request);
    // Save results to a file
    b.addExperimentParameter("sampler", "STRING", "Obstacle_Based_Sampling");
    b.saveResultsToFile("ChainBox_Narrow_obstaclebased.log");
    
}

void planScenario2(ompl::geometric::SimpleSetup &ss)
{
    // TODO: Plan for chain_box in the plane, with a clearance optimization objective, with an Asymptoticallly optimal planner of your choice and store the path in path2.txt


//  NEED TO CHOOSE A PLANNER FOR BONUS QUESTION
    //Create a planner
    
    /*UNCOMMENT THIS BELOW PLANNER FOR Q3
    AND COMMENT THE PLANNER MENTIONED FOR BONUS QUESTION*/
    ////////////////////////////
    auto planner = std::make_shared<ompl::geometric::RRTsharp>(ss.getSpaceInformation());
    ///////////////////////////

    /*UNCOMMENT THIS BELOW PLANNER FOR BONUS QUESTION
    AND COMMENT THE PLANNER MENTIONED FOR Q3*/
    ////////////////////////////
    // ss.getSpaceInformation()->setValidStateSamplerAllocator(getUniformSampler);
    // auto planner = std::make_shared<ompl::geometric::PRMstar>(ss.getSpaceInformation());
    ///////////////////////////


    // Set the problem to be solved in the planner
    ss.setPlanner(planner);

    
    ss.setOptimizationObjective(getClearanceObjective(ss.getSpaceInformation()));

    // Attempt to solve the problem within time limits
    ompl::base::PlannerStatus solved = ss.solve(15.0);
    if (solved)
    {
        /* get the goal representation from the problem definition (not the same as the goal state)
           and inquire about the found path*/
        std::cout << "Found solution:" << std::endl;
        ompl::geometric::PathGeometric &path = ss.getSolutionPath();
        path.interpolate(100);

        // print the path to screen
        path.printAsMatrix(std::cout);

        
        /* path2.txt is needed to run clearance executable file */
        std::ofstream output("path2.txt");
        path.printAsMatrix(output);
        output.close();
    }
    else
        std::cout << "No solution found" << std::endl;
}

void benchScenario2(ompl::geometric::SimpleSetup &ss)
{
    //TODO: Benchmark RRT*, PRM*, RRT# for 10 trials with 60 secounds timeout.
    double runtime_limit = 60, memory_limit = 1024;
    int run_count = 10;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.5);
    ompl::tools::Benchmark b(ss, "ChainBox_Clearance");

    b.addPlanner(std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::PRMstar>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::RRTsharp>(ss.getSpaceInformation()));

    // Run the benchmark
    b.benchmark(request);

    // Save results to a file
    b.saveResultsToFile("clear_benchmark.log");
}



std::shared_ptr<ompl::base::CompoundStateSpace> createChainBoxSpace()
{   //TODO Create the Chainbox ConfigurationSpace
    auto space = std::make_shared<ompl::base::CompoundStateSpace>();

    auto box_se2 = std::make_shared<ompl::base::SE2StateSpace>();
    
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-5);
    bounds.setHigh(5);
    box_se2->setBounds(bounds);
    auto kchain4 = std::make_shared<KinematicChainSpace>(4,1);

    space->addSubspace(box_se2,1.0);//(box_so2, 1.0);  
    space->addSubspace(kchain4,1.0);//(chain_r4, 1.0);  

    //SE2
    //Kinmatic
    //Subspace
    return space;
}

void setupCollisionChecker(ompl::geometric::SimpleSetup &ss, Environment &env)
{   //TODO Setup the stateValidity Checker
    // ValidityChecker checker(ss.getSpaceInformation(),env);
    // ss.setStateValidityChecker([=](const ompl::base::State* state) {
    //     return checker.isValid(state);
    // //     return isValidCheck(state,env);
    // });
    // ss.setStateValidityChecker(std::bind(isValidCheck, std::placeholders::_1, env));
    ss.setStateValidityChecker(std::make_shared<ValidityChecker>(ss.getSpaceInformation(),env));
}

   
int main(int argc, char **argv)
{

    int scenario; 
    Environment env;
    std::vector<double> startVec;
    std::vector<double> goalVec;
    do
    {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) Robot Reaching Task" << std::endl;
        std::cout << " (2) Robot Avoiding Task" << std::endl;

        std::cin >> scenario;
    } while (scenario < 1 || scenario > 3);

    switch (scenario)
    {
        case 1:
            makeScenario1(env, startVec, goalVec);
            break;
        case 2:
            makeScenario2(env, startVec, goalVec);
            break;
        default:
            std::cerr << "Invalid Scenario Number!" << std::endl;
    }

    auto space = createChainBoxSpace();
    ompl::geometric::SimpleSetup ss(space);

    setupCollisionChecker(ss, env);

    //setup Start and Goal
    ompl::base::ScopedState<> start(space), goal(space);
    space->setup();
    space->copyFromReals(start.get(), startVec);
    space->copyFromReals(goal.get(), goalVec);
    ss.setStartAndGoalStates(start, goal);

    switch (scenario)
    {
        case 1:
            planScenario1(ss);
            benchScenario1(ss);
            break;
        case 2:
            planScenario2(ss);
            benchScenario2(ss);
            break;
        default:
            std::cerr << "Invalid Scenario Number!" << std::endl;
    }

}
