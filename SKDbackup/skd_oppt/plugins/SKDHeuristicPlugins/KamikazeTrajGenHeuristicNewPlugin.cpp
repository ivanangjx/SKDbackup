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
using namespace std;

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
        fixedVelocity_ = generalOptions_->fixedVelocity;
        stepTime_ = generalOptions_->fixedStepTime;
        brakingDeceleration_ = generalOptions_->brakingDeceleration;
        controllerMultiplier_ = generalOptions_->controllerMultiplier;

        carDimensions_ = generalOptions_->carDimensions;   
        pedDimensions_ = generalOptions_->pedDimensions;    

        // Compute threshold distance at which the controller will decide to break
        FloatType stoppingTime = std::abs(fixedVelocity_ / brakingDeceleration_);

        // This is the base (multiplier == 1) desired stopping distance described in # Car_BEHAVIOUR.md
        baseStoppingDist_ = 
            // Stopping distance for car
            (fixedVelocity_ * stoppingTime) + (0.5 * brakingDeceleration_ * stoppingTime * stoppingTime) 
            // Added padding to avoid collision in the absence of control error
            + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2 + (stepTime_ * (fixedVelocity_ / 2));

        // cout << "Heuristic: baseStoppingDist_: " << baseStoppingDist_ << endl;  


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

         // Retrieve safe trajectory and step time
        TrajData pedSafeTraj = nextStateUserData->safeTrajectory;
        TrajPoint currentSafeTrajPoint = pedSafeTraj[currentVisitIndex];



        // Further information from general options
        FloatType stepPenalty_ = generalOptions_->stepPenalty;


        // Extract location information from state
        VectorFloat pedLocation{stateVec[STATE_INFO::PED_LONGIT], stateVec[STATE_INFO::PED_HORIZONTAL]};
        VectorFloat carLocation{stateVec[STATE_INFO::CAR_LONGIT], stateVec[STATE_INFO::CAR_HORIZONTAL]};

        FloatType pedInitialX = stateVec[STATE_INFO::PED_LONGIT];
        FloatType pedInitialY = stateVec[STATE_INFO::PED_HORIZONTAL];
        
        FloatType carInitialV = stateVec[STATE_INFO::CAR_SPEED];

        // cout << "pedLocationX: " << pedInitialX << " pedLocationY: " << pedInitialY << " carInitialV: " << carInitialV  << endl;



        FloatType carUpper = stateVec[STATE_INFO::CAR_HORIZONTAL] + carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
        FloatType carLower = stateVec[STATE_INFO::CAR_HORIZONTAL] - carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;

        // cout << "carLocation: " << stateVec[STATE_INFO::CAR_LONGIT] << " " << stateVec[STATE_INFO::CAR_HORIZONTAL] << " upper: " << carUpper << " lower: " << carLower   << endl;

        FloatType STIntersectPointX = std::numeric_limits<FloatType>::infinity();
        FloatType STIntersectPointY = std::numeric_limits<FloatType>::infinity();
        long STIntersectIndex;

        // for (int i = 0; i <= pedSafeTraj.size() -1 ; i++) {
        //     TrajPoint tempSafeTrajPoint = pedSafeTraj[i];
        //     // cout << tempSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT] << " " << tempSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ] << endl;

        //     FloatType pedUpper = tempSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ] + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
        //     FloatType pedLower = tempSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ] - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];

        //     if ((carLower <= pedUpper) && (pedLower <= carUpper)) {
        //         // cout << i << endl;

        //         if (tempSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT] < STIntersectPointX) {    // use leftmost coordinates
        //             STIntersectPointY = tempSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];
        //             STIntersectPointX = tempSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
        //             STIntersectIndex = i;
        //         }
        //     }
        // }



        for (int i = 0; i <= pedSafeTraj.size() -1 ; i++) {
            TrajPoint tempSafeTrajPoint = pedSafeTraj[i];
            // cout << tempSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT] << " " << tempSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ] << endl;

            FloatType pedUpper = tempSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ] + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
            FloatType pedLower = tempSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ] - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];

            if ((carLower <= pedUpper) && (pedLower <= carUpper)) {
                // cout << i << endl;

                if (tempSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ] >= stateVec[STATE_INFO::CAR_HORIZONTAL]) {    // more or equal than y axis center of car
                    STIntersectPointY = tempSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];
                    STIntersectPointX = tempSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
                    STIntersectIndex = i;
                }
            }
        }

        // cout << "index: " << STIntersectIndex << " STIntersectPointX: " << STIntersectPointX << " STIntersectPointY: " << STIntersectPointY << endl;

        FloatType distToIntersectPoint = sqrt( pow(STIntersectPointX - pedInitialX, 2) + pow(STIntersectPointY - pedInitialY, 2) );
        // cout << distToIntersectPoint << endl;

        FloatType numStepsToIntersectPoint = distToIntersectPoint/ (MAX_SPEED * stepTime_);

        // cout << "numStepsToIntersectPoint : " << numStepsToIntersectPoint << endl;
        // cout << round(numStepsToIntersectPoint) << endl;
        //----------------------------------------------------

        FloatType incrementX = abs(pedInitialX - STIntersectPointX) / round(numStepsToIntersectPoint);
        FloatType incrementY = abs(pedInitialY - STIntersectPointY) / round(numStepsToIntersectPoint);

        // cout << "increment: X: " << incrementX << " Y: " << incrementY << endl;

        FloatType pedX = pedInitialX;
        FloatType carX = stateVec[STATE_INFO::CAR_LONGIT];
        FloatType distCarPed = abs(pedX - carX);
        FloatType distCarTravel = 0;
        FloatType carV = carInitialV;

        for (int i = 0; i <= round(numStepsToIntersectPoint) ; i++) {

            if (pedInitialX < STIntersectPointX) {
                pedX = pedInitialX + (i * incrementX);
            } else {
                pedX = pedInitialX - (i * incrementX);
            }
      
            carX = stateVec[STATE_INFO::CAR_LONGIT] + distCarTravel;
            distCarPed = abs(pedX - carX);

            // cout << "carX: " << carX << " pedX: " << pedX << " distCarPed: " << distCarPed << " distCarTravel: " << distCarTravel << " carV: " << carV << endl;

            if (distCarPed > baseStoppingDist_) { //more dist so not braking yet /a=0
                distCarTravel += (carV * stepTime_);
            } else {
                distCarTravel += ( (carV * stepTime_) + (0.5 * brakingDeceleration_ * stepTime_ * stepTime_));
                carV = carV + (brakingDeceleration_ * stepTime_);
            }
        }

        FloatType carXPartOne = stateVec[STATE_INFO::CAR_LONGIT] + distCarTravel;

        // cout << "carXPartOne : " << carXPartOne << endl;

        //-----------------------------------
        FloatType carRight = carXPartOne + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;
        FloatType pedLeft = pedX - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];

        FloatType numStepsToCollideY = 0;

        while (carRight < pedLeft) {

            // cout << "carRight : " << carRight << "pedLeft : " << pedLeft << " carV : " << carV  << endl;

            pedLeft = pedLeft - (MAX_SPEED * stepTime_); //assume collision is left

            FloatType carOneStepTravelY = 0;

            if (carV > 0) {
                if (distCarPed > baseStoppingDist_) { //more dist so not braking yet /a=0
                    carOneStepTravelY = (carV * stepTime_);
                } else {
                    carOneStepTravelY = ( (carV * stepTime_) + (0.5 * brakingDeceleration_ * stepTime_ * stepTime_));
                    carV = carV + (brakingDeceleration_ * stepTime_);
                }
                carRight = carRight + carOneStepTravelY;               
            }



            numStepsToCollideY += 1;
        }

        // cout << "numStepsToCollideY: " << numStepsToCollideY << endl;

        FloatType totalSteps = numStepsToIntersectPoint + numStepsToCollideY;

        long totalStepsRounded = round(totalSteps);

        // cout << "totalSteps: " << totalSteps << endl;

        // cout << "totalStepsRounded: " << totalStepsRounded << endl;

        FloatType heuristicVal = generalOptions_->goalReward - 
               ( totalSteps * stepPenalty_);

        // cout << "heuristicVal: " << heuristicVal << endl;


        // take into account constraint used ------------------------------

        if (totalStepsRounded > pedSafeTraj.size()-1) {
            totalStepsRounded = pedSafeTraj.size()-1;
        }

        TrajPoint collideSafeTrajPoint = pedSafeTraj[totalStepsRounded];

        FloatType STCollidePointY = collideSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];
        FloatType STCollidePointX = collideSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];

        FloatType KamiCollidePedX = pedLeft + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
        FloatType KamiCollidePedY = STIntersectPointY;

        // cout << "STCollidePointX: " << STCollidePointX << "STCollidePointY: " << STCollidePointY << endl;
        // cout << "KamiCollidePedX: " << KamiCollidePedX << "KamiCollidePedY: " << KamiCollidePedY << endl;

        FloatType distSafeToKamiCollide = sqrt( pow(STCollidePointX - KamiCollidePedX, 2) + pow(STCollidePointY - KamiCollidePedY, 2) );
        // cout << "distSafeToKamiCollide: " << distSafeToKamiCollide << endl;

        float constraint = heuristicInfo->constraint;

        if (distSafeToKamiCollide > constraint) {
            return -1;
        }

        // cout << endl;


        // jimy part old ----------------------------------------------------

        // Calculate relative distance on both dimensions
        // FloatType transverseDist = stateVec[STATE_INFO::PED_HORIZONTAL] - (stateVec[STATE_INFO::CAR_HORIZONTAL]) ; 

        // // Calculate distance between ped and horizontal line
        // FloatType distanceFromSafetLongit = std::abs(stateVec[STATE_INFO::PED_LONGIT] - currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT]);
        // FloatType distanceFromSafeHoz = 
        //     std::abs(stateVec[STATE_INFO::PED_HORIZONTAL] - currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ]);
        
      
        // //Approximated steps to the vertical center of the car
        // FloatType approximateCollisionTime =  (stateVec[STATE_INFO::PED_LONGIT] - stateVec[STATE_INFO::CAR_LONGIT])
        //                                                     / (MAX_SPEED + stateVec[STATE_INFO::CAR_SPEED]);

        // FloatType approximatedStepsX = std::abs(approximateCollisionTime) / stepTime_;

        

        // // Approximated steps till hitting the car from left side
        // FloatType approximatedStepsY = std::abs(transverseDist) / (MAX_SPEED * stepTime_);

        // // cout << approximatedStepsY << endl;

        // // Deviation
        // FloatType longitDeviation = distanceFromSafetLongit / (MAX_SPEED * stepTime_);
        // FloatType hozDeviation = distanceFromSafeHoz / (MAX_SPEED * stepTime_);

        // // Approximated values is approximated steps until goal and approximated future penalties incurred from current state
        // FloatType heuristicVal = generalOptions_->goalReward - 
        //        ( (approximatedStepsX  + approximatedStepsY) * stepPenalty_);


        return heuristicVal;
    }

  

private:
    // Pointer to structure with parsed options values
    KamikazeTrajGenGeneralOptions* generalOptions_;
    std::string optionsFile_;

    FloatType baseStoppingDist_ = 0.0;
    FloatType fixedVelocity_ = 0.0;
    FloatType stepTime_ = 0.0;

    FloatType brakingDeceleration_ = 0.0;

    FloatType controllerMultiplier_ = 1.0;


    VectorFloat carDimensions_;
    VectorFloat pedDimensions_;

};

OPPT_REGISTER_HEURISTIC_PLUGIN(KamikazeTrajGenHeuristicPlugin)

}

#endif
