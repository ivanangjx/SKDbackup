/**
 * Copyright 2017
 *
 * This file is part of On-line POMDP Planning Toolkit (OPPT).
 * OPPT is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License published by the Free Software Foundation,
 * either version 2 of the License, or (at your option) any later version.
 *
 * OPPT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with OPPT.
 * If not, see http://www.gnu.org/licenses/.
 */
#ifndef _KAMIKAZE_TRAJ_GEN_HEURISTIC_PLUGIN_HPP_
#define _KAMIKAZE_TRAJ_GEN_HEURISTIC_PLUGIN_HPP_
#include "oppt/plugin/Plugin.hpp"
#include "../SKDDefines/KamikazeTrajGenGeneralUtils.hpp"

namespace oppt
{
class KamikazeTrajGenHeuristicPlugin: public HeuristicPlugin
{
public:
    KamikazeTrajGenHeuristicPlugin():
        HeuristicPlugin() {

    }

    virtual ~KamikazeTrajGenHeuristicPlugin() {

    }

    virtual bool load(const std::string& optionsFile) override {        
        optionsFile_ = optionsFile;
        parseOptions_<KamikazeTrajGenGeneralOptions>(optionsFile);
        generalOptions_ = static_cast<KamikazeTrajGenGeneralOptions*>(options_.get());

        return true;
    }

    virtual FloatType getHeuristicValue(const HeuristicInfo* heuristicInfo) const override {   
        // Constant for problem
        const FloatType MAX_SPEED = 2.5;
       // Retrieve information from the state
        VectorFloat stateVec = heuristicInfo->currentState->as<VectorState>()->asVector();
        // User data associated with the state
        auto nextStateUserData = 
            static_cast<SKDUserData*>(heuristicInfo->currentState->getUserData().get());

        FloatType currentVisitIndex = nextStateUserData->visitIndex;

        float constraint = heuristicInfo->constraint;

        // cout << "currentVisitIndex: " << currentVisitIndex << " constraint: " << constraint << endl;

        
        // cout << constraint << endl;

         // Retrieve safe trajectory and step time
        TrajData pedSafeTraj = nextStateUserData->safeTrajectory;
        TrajPoint currentSafeTrajPoint = pedSafeTraj[currentVisitIndex];
        FloatType stepTime_ = generalOptions_->fixedStepTime;
        FloatType brakingDeceleration = generalOptions_->brakingDeceleration;

        // cout << pedSafeTraj.size() << endl;

        // Further information from general options
        FloatType stepPenalty_ = generalOptions_->stepPenalty;
        VectorFloat carDimensions_ = generalOptions_->carDimensions;
        VectorFloat pedDimensions_ = generalOptions_->pedDimensions; 

        // Extract location information from state
        VectorFloat pedLocation{stateVec[STATE_INFO::PED_LONGIT], stateVec[STATE_INFO::PED_HORIZONTAL]};
        VectorFloat carLocation{stateVec[STATE_INFO::CAR_LONGIT], stateVec[STATE_INFO::CAR_HORIZONTAL]};
        float carSpeed = stateVec[STATE_INFO::CAR_SPEED];

        // calculate where car stops
        FloatType stoppingTime = std::abs(carSpeed / brakingDeceleration);
        FloatType carDisplacement =  stateVec[STATE_INFO::CAR_SPEED] * stoppingTime + (0.5 * stoppingTime * stoppingTime * brakingDeceleration);
        FloatType carFinalX = stateVec[STATE_INFO::CAR_LONGIT] + carDisplacement + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;

        // cout << "pedLocation: " << stateVec[STATE_INFO::PED_LONGIT] << " " << stateVec[STATE_INFO::PED_HORIZONTAL] <<  " carSpeed: " << carSpeed << " carFinalX: " << carFinalX << endl;

        // Calculate relative distance on both dimensions
        FloatType transverseDist = stateVec[STATE_INFO::PED_HORIZONTAL] - (stateVec[STATE_INFO::CAR_HORIZONTAL]) ; 

        // Calculate distance between ped and horizontal line
        FloatType distanceFromSafetLongit = std::abs(stateVec[STATE_INFO::PED_LONGIT] - currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT]);
        FloatType distanceFromSafeHoz =     std::abs(stateVec[STATE_INFO::PED_HORIZONTAL] - currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ]);
        
      
        //Approximated steps to the vertical center of the car
        FloatType approximateCollisionTime =  (stateVec[STATE_INFO::PED_LONGIT] - stateVec[STATE_INFO::CAR_LONGIT])
                                                            / (MAX_SPEED + stateVec[STATE_INFO::CAR_SPEED]);

        FloatType approximatedStepsX = std::abs(approximateCollisionTime) / stepTime_;
        // Approximated steps till hitting the car from left side
        FloatType approximatedStepsY = std::abs(transverseDist) / (MAX_SPEED * stepTime_);

        // cout << "approximatedStepsX: " << std::ceil(approximatedStepsX) << " approximatedStepsY: " << std::ceil(approximatedStepsY) << endl;

        bool collide = true;
        long numStepsY = 0;
        long numStepsX = 0;
        TrajPoint tempSafeTrajPoint;
        //loop starting from current index
        for (int i=0; i <= std::ceil(approximatedStepsY) ; i++) {

            if (currentVisitIndex + i >= pedSafeTraj.size() -1) {
                tempSafeTrajPoint = pedSafeTraj[pedSafeTraj.size() -1]; //use last index if exceed
            } else {
                tempSafeTrajPoint = pedSafeTraj[currentVisitIndex + i];
            }

            
            // cout << "   tempSafeTrajIndex: " << currentVisitIndex + i << endl;
            // cout << "   safeLocation: " << tempSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT] << " " << tempSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ] << endl;           
            // cout << "   pedLocation: " << stateVec[STATE_INFO::PED_LONGIT] << " " << stateVec[STATE_INFO::PED_HORIZONTAL] - (MAX_SPEED * stepTime_ *  i) << endl;

            float distanceY = std::abs( stateVec[STATE_INFO::PED_HORIZONTAL] - (MAX_SPEED * stepTime_ * i) - tempSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ] );

            if (distanceY <= constraint) {
                // cout << "   distY to safe: " << distanceY << " still within" << endl;
                numStepsY += 1;
            } else {
                // cout << "   distY to safe: " << distanceY << " outside" << endl;
                collide = false;
                break;
            }
        }
        // cout << "numStepsY: " << numStepsY << endl;

        if (!collide) {
             FloatType failVal = - stepPenalty_ * (approximatedStepsY / std::ceil(approximatedStepsY) * numStepsY); // doesnt account for terminalPenalty
             // cout << "failVal: " << failVal << endl;
             return failVal;
        }

        // cout << "test1"  << endl;

        for (int i=0; i <= std::ceil(approximatedStepsX) ; i++) {

            if (currentVisitIndex + numStepsY + i >= pedSafeTraj.size() -1) {
                tempSafeTrajPoint = pedSafeTraj[pedSafeTraj.size() -1]; //use last index if exceed
            } else {
                tempSafeTrajPoint = pedSafeTraj[currentVisitIndex + numStepsY + i];
            }
            // cout << "test2"  << endl;

            float distanceX = std::abs( stateVec[STATE_INFO::PED_LONGIT] - (MAX_SPEED * stepTime_ * i) - tempSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT] );
            FloatType pedLeftEdgeX = distanceX - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];

            if (distanceX <= constraint) {
                // cout << "   distX to safe: " << distanceX << " still within" << endl;
                if (1>0)
                    numStepsX += 1;
                if (pedLeftEdgeX <= carFinalX) {
                    // cout << "COLLIDED within constraint" << endl;
                    collide = true;
                    break;
                }

            } else {
                // cout << "   distX to safe: " << distanceX << " outside" << endl;
                collide = false;
                break;
            }

        }

        // cout << "test3"  << endl;

        if (!collide) {
            FloatType failVal = - stepPenalty_ * ((approximatedStepsX + approximatedStepsY) / std::ceil(approximatedStepsX + approximatedStepsY) * (numStepsX + numStepsY)); // doesnt account for terminalPenalty
            // cout << "failVal: " << failVal << endl;
            return failVal;
         } else {
            FloatType successVal = generalOptions_->goalReward - ( ((approximatedStepsX + approximatedStepsY) / std::ceil(approximatedStepsX + approximatedStepsY) * (numStepsX + numStepsY)) * stepPenalty_);
            // cout << "SUCCESSVal: " << successVal << endl;
            return successVal;
         }

        // Deviation
        // FloatType longitDeviation = distanceFromSafetLongit / (MAX_SPEED * stepTime_);
        // FloatType hozDeviation = distanceFromSafeHoz / (MAX_SPEED * stepTime_);

        // // Approximated values is approximated steps until goal and approximated future penalties incurred from current state
        // FloatType heuristicVal = generalOptions_->goalReward - 
        //        ( (approximatedStepsX  + approximatedStepsY) * stepPenalty_);


        // return heuristicVal;
    }

    // virtual void getConstraint(){
    //     // cout << "test: " << endl;
    // }

  

private:
    // Pointer to structure with parsed options values
    KamikazeTrajGenGeneralOptions* generalOptions_;
    std::string optionsFile_;


};

OPPT_REGISTER_HEURISTIC_PLUGIN(KamikazeTrajGenHeuristicPlugin)

}

#endif
