#include "RTP.h"
#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
 
ompl::geometric::RTP::RTP(const base::SpaceInformationPtr &si, bool addIntermediateStates)
  : base::Planner(si, addIntermediateStates ? "RTPintermediate" : "RTP")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;
 
    Planner::declareParam<double>("range", this, &RTP::setRange, &RTP::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RTP::setGoalBias, &RTP::getGoalBias, "0.:.05:1.");

}
 
ompl::geometric::RTP::~RTP()
{
    freeMemory();
}
 
void ompl::geometric::RTP::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();

    allMotions.clear();

    lastGoalMotion_ = nullptr;
}
 
void ompl::geometric::RTP::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);
 
}
 
void ompl::geometric::RTP::freeMemory()
{
    allMotions.clear();
}
 
ompl::base::PlannerStatus ompl::geometric::RTP::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);
 
    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        allMotions.push_back(motion);
    }


    if (allMotions.size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return ompl::base::PlannerStatus::INVALID_START;
    }
    
 
    if (!sampler_)
        sampler_ = si_->allocStateSampler();
 
   
    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), allMotions.size());

 
    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    ompl::base::State *rstate = rmotion->state;
    
    
    while (!ptc)
    {
        /*We sample a random state (with goal biasing) */
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);
 
        /*We select a random node from the tree*/ 
        int randI = std::rand() % allMotions.size();
        Motion *nmotion = allMotions[randI];

 
        /*We check if there is a valid stright-line path between the sampled node and the tree node*/
        if (si_->checkMotion(nmotion->state, rstate))
        {
        
            auto *motion = new Motion(si_);
                si_->copyState(motion->state, rstate);
                motion->parent = nmotion;
                allMotions.push_back(motion);
 
                nmotion = motion;
                ////////
 
            double dist = 0.0;
            bool sat = goal->isSatisfied(nmotion->state, &dist);
            /*We check if the new node added to the tree is the goal node*/
            if (sat)
            {
                approxdif = dist;
                solution = nmotion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = nmotion;
            }
        }
    }
 
    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }
 
    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;
 
        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }
 
        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }
 

    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;
 

    OMPL_INFORM("%s: Created %u states", getName().c_str(), allMotions.size());


    return {solved, approximate};
}
 
void ompl::geometric::RTP::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);
 
    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));
 
    for (auto &motion : allMotions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}