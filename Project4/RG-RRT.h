///////////////////////////////////////
// RBE 550
// Project 4
// Authors: Dhruv, Shreyas
//////////////////////////////////////

#ifndef RGRRT_H
#define RGRRT_H

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/datastructures/NearestNeighborsLinear.h"
#include <vector>
#include <limits>
#include <cmath>
#include <memory>

namespace ompl
{
    namespace control
    {
        class RGRRT : public base::Planner
        {
        public:
            /** @brief Constructor */
            RGRRT(const SpaceInformationPtr &si);

            /** @brief Destructor */
            ~RGRRT() override;

            /** @brief Continue solving for some amount of time. Return true if a solution was found.
             * @param ptc: condition for planner to stop solving
             * @return planner status
             */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            /** @brief Clear the planners RRTGT motion-tree*/
            void clear() override;

            /** @brief Set the goal bias for selecting the goal state
             * @param goalBias: an elipsion region around goal
             */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** @brief Getter function for the current goal bias
             * @return goal
             */
            double getGoalBias() const
            {
                return goalBias_;
            }

            /** @brief Specify whether to add intermediate states in the tree
             * @param addIntermediateStates: boolean value true for enable adding intermediat states
             */
            void setIntermediateStates(bool addIntermediateStates)
            {
                addIntermediateStates_ = addIntermediateStates;
            }

            /** @brief Check whether intermediate states are added to the tree 
             * @return true: if added
            */
            bool getIntermediateStates() const
            {
                return addIntermediateStates_;
            }

            /** @brief Set the control sampling size 
             * @param controlSampleSize: no of control samples
             */
            void setControlSampleSize(int controlSampleSize)
            {
                number_of_control_inputs_ = controlSampleSize;
            }

            /** @brief Getter function for the  control sample size
             * @return number_of_control_inputs_
             */
            double getControlSampleSize() const
            {
                return number_of_control_inputs_;
            }

            /** @brief Specify whether to use sampling on first control state
             * @param useFirstControlSample: boolean value true for enabling sampling on the first control state
             * @brief If set true then it will only consider u[0] sampling as per project
             * @brief If set false then it will consider sampling on all states
             */
            void setFirstControlSample(bool useFirstControlSample)
            {
                use_first_control_ = useFirstControlSample;
            }

            /** @brief Check whether we are sampling on first control state or on all control state
             * @return true: if single control state sampling
             * @return false: if all control state sampling
            */
            bool getFirstControlSample() const
            {
                return use_first_control_;
            }

            /** @brief Retrieve planner data */
            void getPlannerData(base::PlannerData &data) const override;

            /** @brief Set a different nearest neighbors data structure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                clear();
                nn_ = std::make_shared<NN<std::shared_ptr<Motion>>>();
                setup();
            }

            /** @brief Setup the planner */
            void setup() override;

        protected:
            /** @brief Representation of a motion */
            class Motion
            {
            public:
                Motion() = default;

                Motion(const SpaceInformation *si)
                    : state(si->allocState()), control(si->allocControl())
                {
                }

                ~Motion() = default;

                /** @brief State of the motion */
                base::State *state{nullptr};

                /** @brief Control applied in the motion */
                Control *control{nullptr};

                /** @brief Number of control steps */
                unsigned int steps{0};

                /** @brief Parent motion in the tree */
                std::weak_ptr<Motion> parent;

                /** @brief Reachable states from this motion */
                std::vector<std::shared_ptr<Motion>> reachable;
            };

            /** @brief Free the memory allocated by the planner */
            void freeMemory();

            /** @brief Calculate the distance between two motions */
            double distanceFunction(const std::shared_ptr<Motion>& a, const std::shared_ptr<Motion>& b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** @brief State sampler */
            base::StateSamplerPtr sampler_;

            /** @brief Control sampler */
            DirectedControlSamplerPtr controlSampler_;

            /** @brief Cast of SpaceInformation for convenience */
            const SpaceInformation *siC_;

            /** @brief Nearest-neighbors data structure */
            // std::shared_ptr<NearestNeighbors<Motion *>> nn_;
            std::shared_ptr<NearestNeighbors<std::shared_ptr<Motion>>> nn_;

            /** @brief Probability of expanding towards the goal */
            double goalBias_{0.05};

            /** @brief Flag for adding intermediate states */
            bool addIntermediateStates_{false};

            /** @brief Random number generator */
            RNG rng_;

            /** @brief Most recent goal motion */
            // Motion *lastGoalMotion_{nullptr};
            std::shared_ptr<Motion> lastGoalMotion_;

            /** @brief Setup the reachable set for a motion */
            // void setupReachableSet(Motion *m);
            void setupReachableSet(std::shared_ptr<Motion> m);

            /** @brief Select a motion from the reachable set */
            // int selectReachableMotion(const Motion *qnear, const Motion *qrand);
            int selectReachableMotion(const std::shared_ptr<Motion> &qnear, const std::shared_ptr<Motion> &qrand);

            /** @brief Control sampling  size */
            int number_of_control_inputs_{10};
            
            /** @brief Control sampling  step for all control inputs*/
            std::vector<double> control_step_;

            // /** @brief Enable single control sample*/
            bool use_first_control_{true};
        };

    } // namespace control
} // namespace ompl

#endif
