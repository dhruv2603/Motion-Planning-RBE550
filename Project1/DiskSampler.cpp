/* Author: Ali Golestaneh and Constantinos Chamzas */
#include <ompl/util/RandomNumbers.h>
#include "DiskSampler.h"
#include <cmath>


bool isStateValid(const ob::State *state) {

    // cast the abstract state type to the type we expect
    const auto *r2state = state->as<ob::RealVectorStateSpace::StateType>();
    double x = r2state->values[0];
    double y = r2state->values[1];
    // A square obstacle with and edge of size 2*sqrt(2) is located in location [-3,-2,] and rotated pi/4 degrees around its center.
    // Fill out this function that returns False when the state is inside/or the obstacle and True otherwise// 

    // ******* START OF YOUR CODE HERE *******//
    
    double X, Y, theta, s, xb, yb, xc, yc;
    X     = -3;
    Y     = -2;
    theta = M_PI/4;
    s     = 2*sqrt(2); 
    
    xb = x - X;
    yb = y - Y;

    //Implement this
    xc = xb*cos(theta) - yb*sin(theta);
    yc = xb*sin(theta) + yb*cos(theta);

    if (xc <= s/2 and xc >= -s/2)
    {
        if (yc <= s/2 and yc >= -s/2)
        {
            return false;
        }
        
    }
    
    return true;

    // ******* END OF YOUR CODE HERE *******//
}


bool DiskSampler::sampleNaive(ob::State *state) 
{
    // ******* START OF YOUR CODE HERE *******//

    ompl::RNG rng_;
    const auto *pstate = state->as<ob::RealVectorStateSpace::StateType>();
    double r = rng_.uniformReal(0,10);
    double theta = rng_.uniformReal(0,2*M_PI);

    pstate->values[0] = r*cos(theta);
    pstate->values[1] = r*sin(theta);
    

    // ******* END OF YOUR CODE HERE *******//
    
    //The valid state sampler must return false if the state is in-collision
    return isStateValid(state);
}

bool DiskSampler::sampleCorrect(ob::State *state)
{
    // ******* START OF YOUR CODE HERE *******//
    ompl::RNG rng_;
    const auto *pstate = state->as<ob::RealVectorStateSpace::StateType>();
    double r = sqrt(rng_.uniformReal(0,10*10));
    double theta = rng_.uniformReal(0,2*M_PI);

    pstate->values[0] = r*cos(theta);
    pstate->values[1] = r*sin(theta);

    // ******* END OF YOUR CODE HERE *******//


    //The valid state sampler must return false if the state is in-collision
    return isStateValid(state);
}
