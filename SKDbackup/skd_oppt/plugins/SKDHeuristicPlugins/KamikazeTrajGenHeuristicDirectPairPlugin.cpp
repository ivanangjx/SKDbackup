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

#include <cmath>

// C++ implementation of the approach
#include <bits/stdc++.h>
// To store the point
#define Point pair<double, double>
#define F first
#define S second

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

        pedRadius = pedDimensions_[PED_DIMENSIONS::PED_RADIUS];

        discountFactor = generalOptions_->discountFactor;    

        accelerationError = generalOptions_->accelerationError;
        velocityError = generalOptions_->velocityError;

        // Compute threshold distance at which the controller will decide to break
        FloatType stoppingTime = std::abs(fixedVelocity_ / brakingDeceleration_);

        // This is the base (multiplier == 1) desired stopping distance described in # Car_BEHAVIOUR.md
        baseStoppingDist_ = 
            // Stopping distance for car
            (fixedVelocity_ * stoppingTime) + (0.5 * brakingDeceleration_ * stoppingTime * stoppingTime) 
            // Added padding to avoid collision in the absence of control error
            + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2 + (stepTime_ * (fixedVelocity_ / 2));

        // baseStoppingDist_ = 100;

        baseStoppingDist_ = baseStoppingDist_ * controllerMultiplier_;

        cout << "Heuristic: baseStoppingDist_: " << baseStoppingDist_ << endl;  


        return true;
    }

    virtual void parseAllActions(const std::vector<ActionSharedPtr> allActions_) {
        // cout << "PARSE ALL ACTIONS" << endl;
        allActionsVector.clear();
        for(auto actions : allActions_){
            // actions->print(cout);
            // std::cout << std::endl;
            VectorFloat actionApplied = actions->as<VectorAction>()->asVector();
            // cout << actionApplied[0] << endl;
            // cout << actionApplied[1] << endl;

            std::pair<FloatType, FloatType> tempPair;
            tempPair.first = actionApplied[0];
            tempPair.second = actionApplied[1];
            allActionsVector.push_back(tempPair);
        }
    }

    FloatType leastDistanceY;
    long intersectIndexMin;
    long intersectIndexMax;
    long firstIndexMin;
    long lastIndexMin;

    virtual FloatType getMinConstraintUsingST(const HeuristicInfo* heuristicInfo) {
        fixedVelocity_ = generalOptions_->fixedVelocity;
        brakingDeceleration_ = generalOptions_->brakingDeceleration;
        cout << "----------------------------------------------------------------------------------" << endl;
        cout << "GETTING MIN CONSTRAINT DIST (CAR TRAVEL MORE): " << endl;
        // Constant for problem
        const FloatType MAX_SPEED = 2.5;
        // Retrieve information from the state
        VectorFloat stateVec = heuristicInfo->currentState->as<VectorState>()->asVector();
        // User data associated with the state
        auto nextStateUserData = static_cast<SKDUserData*>(heuristicInfo->currentState->getUserData().get());
        TrajData pedSafeTraj = nextStateUserData->safeTrajectory;

        VectorFloat carStartPos = generalOptions_->carStartPos;

        FloatType currentVisitIndex = 0;
        //initial Ped Position
        TrajPoint currentSafeTrajPoint = pedSafeTraj[currentVisitIndex];
        FloatType STPointX = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
        FloatType STPointY = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];
        // initial car position
        FloatType carX = carStartPos[0];
        FloatType carY = carStartPos[1];
        FloatType carV = stateVec[STATE_INFO::CAR_SPEED];

        //ped and car dimensions
        FloatType carLower = carY - carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
        FloatType carUpper = carY + carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;        
        FloatType carLeft = carX - carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;
        FloatType carRight = carX + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;

        FloatType pedLower = STPointY - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
        FloatType pedUpper = STPointY + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
        FloatType pedLeft = STPointX - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
        FloatType pedRight = STPointX + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];

        //get FIRST INDEX point -> lowest intersect before passing car
        
        // FloatType tempLeftMostX = std::numeric_limits<FloatType>::infinity();

        for (int i=0; i<= pedSafeTraj.size() -1; i++) {

            TrajPoint tempCurrentSafeTrajPoint = pedSafeTraj[i];

            FloatType tempSTPointX = tempCurrentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
            FloatType tempSTPointY = tempCurrentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];

            FloatType tempPedLower = tempSTPointY - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
            FloatType tempPedUpper = tempSTPointY + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];

            // cout << tempSTPointX << " " << tempSTPointY << endl;

            if (tempPedUpper < carUpper) {
                break;
            }

            firstIndexMin = i;
        }

        // cout << "firstIndex: " << firstIndex << endl;

        //get stopping point -> lowest intersect before passing car
        
        // FloatType tempLeftMostX = std::numeric_limits<FloatType>::infinity();

        for (int i=0; i<= pedSafeTraj.size() -1; i++) {

            TrajPoint tempCurrentSafeTrajPoint = pedSafeTraj[i];

            FloatType tempSTPointX = tempCurrentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
            FloatType tempSTPointY = tempCurrentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];

            FloatType tempPedLower = tempSTPointY - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
            FloatType tempPedUpper = tempSTPointY + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];

            // cout << tempSTPointX << " " << tempSTPointY << endl;

            if (tempPedUpper < carLower) {
                break;
            }

            lastIndexMin = i;
        }

        // cout << "lastIndex: " << lastIndex << endl;

        FloatType distCarPedX;

        FloatType leastDistance = std::numeric_limits<FloatType>::infinity();
        
        FloatType leastDistanceForIndex = std::numeric_limits<FloatType>::infinity();

        //DIDNT TAKE INTO ACCOUNT POSSIBILITY OF COLLISION SINCE IT IS SAFE TRAJECTORY

        // for (int i=0; i<= pedSafeTraj.size() -1 ; i++) {
        for (int i=0; i<= lastIndexMin ; i++) {

            // cout << "t= " << i << endl;

            currentSafeTrajPoint = pedSafeTraj[i];
            STPointX = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
            STPointY = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];

            //Should be here, i+1 and car move after prev iteration - SEE TRANSITION PLUGIN dist to know coast or brake?
            distCarPedX = abs(STPointX - carX);

            //get car Right line
            Point A = make_pair(carRight, carUpper);
            Point B = make_pair(carRight, carLower);

            Point topLeft = make_pair(carLeft, carUpper);
            Point topRight = make_pair(carRight, carUpper);
            Point bottomLeft = make_pair(carLeft, carLower);
            Point bottomRight = make_pair(carRight, carLower);

            FloatType tempBrakingDeceleration_ = brakingDeceleration_;

            for (int j=i; j<= i ; j++) {
                // cout << "   compare: " << j << endl;
                //get ST index
                TrajPoint tempCurrentSafeTrajPoint = pedSafeTraj[j];
                FloatType tempSTPointX = tempCurrentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
                FloatType tempSTPointY = tempCurrentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];
                //get ped point
                Point E = make_pair(tempSTPointX, tempSTPointY);

                //calc dist
                double minDistToCar = min({minDistance(topLeft, topRight, E) , minDistance(bottomLeft, bottomRight, E) , minDistance(topLeft, bottomLeft, E) , minDistance(topRight, bottomRight, E)});
                // cout << "       R: " << minDistance(topRight, bottomRight, E) << "       D: " << minDistance(bottomLeft, bottomRight, E) << endl;
                // double minDistToCar = minDistance(A, B, E);

                // // DIST SHOULD BE ST POINT TO INTERSECT POINT FROM ST- NOT MINIMUM!!!
                // FloatType distToCarTopRight = sqrt( pow(carRight - tempSTPointX, 2) + pow(carUpper - tempSTPointY, 2) );
                // FloatType distToCarBottomRight = sqrt( pow(carRight - tempSTPointX, 2) + pow(carLower - tempSTPointY, 2) );
                // FloatType distToCarMidRight = sqrt( pow(carRight - tempSTPointX, 2) + pow(carY - tempSTPointY, 2) );
                // double distToCar = max(distToCarTopRight, distToCarBottomRight);
                // // cout << "   minDistToCar: " << minDistToCar << "   distToCarTopRight: " << distToCarTopRight << "   distToCarBottomRight: " << distToCarBottomRight << "   distToCarMidRight: " << distToCarMidRight  << "   dist: " << distToCar << endl;

                //minDisttoCar  means i need x more steps to reach car, so the constraint dist should count from current ST index plus the X number of steps
                // minimum ped steps needed to achive this distance
                FloatType pedDist = 0;
                int numSteps = 0;
                while (pedDist < minDistToCar) {
                    pedDist += (MAX_SPEED * stepTime_);
                    numSteps += 1;
                }
                // cout << "pedDist: " << pedDist << endl;
                // cout << "   numSteps: " << numSteps << endl;

                //get ST index for extra
                FloatType extraStep = j + numSteps;
                // cout << "   totalSteps: " << extraStep << endl;
                if (extraStep > pedSafeTraj.size()-1) {
                    extraStep = pedSafeTraj.size()-1;
                }
                TrajPoint extraCurrentSafeTrajPoint = pedSafeTraj[extraStep];
                FloatType extraSTPointX = extraCurrentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
                FloatType extraSTPointY = extraCurrentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];

                Point F = make_pair(extraSTPointX, extraSTPointY);
                double collideDistToCar = min({minDistance(topLeft, topRight, F) , minDistance(bottomLeft, bottomRight, F) , minDistance(topLeft, bottomLeft, F) , minDistance(topRight, bottomRight, F)});

                if (collideDistToCar < leastDistance) {
                    leastDistance = collideDistToCar;
                    leastDistanceY = tempSTPointY;
                    intersectIndexMin = i;
                }
                

                // // smallest dist within index range to get index
                // if ((i <= lastIndex) && (i >= firstIndex)) {
                //     if (minDistToCar < leastDistanceForIndex) {
                //         // cout << "INDEXED" << endl;
                //         leastDistanceForIndex = minDistToCar;
                //         intersectIndex = i;
                //     }
                // }

                // cout << "       minDistToCar: " << minDistToCar << "       comparing: " << std::setw(10) << tempSTPointX << " " << std::setw(10) << tempSTPointY << " car: " << std::setw(10) << carX << " " << std::setw(10) << carY << " " << std::setw(10) << carRight<< endl;
                // cout << "   collideDistToCar: " << collideDistToCar << endl;
            }

            
            // cout << "distCarPedX: " << distCarPedX << " baseStoppingDist_: " << baseStoppingDist_ << endl;
            // cout << "comparing: " << STPointX << " " << STPointY << " " << carX << " " << carY << endl;

            //CAR MAX MOVEMENT - CLOSEST TO PED (SO LESS PED DIST TO ST)
            if (carV <= 0) {
                carX = carX;
                carV = 0;
            }
            else if (distCarPedX > baseStoppingDist_) { //more dist so not braking yet /a=0
                // cout << "coast v:" << carV << endl;
                carX += (carV * stepTime_); //car v is clamped max to fixed V- so assume as long as costing- always max V
            } else {
                // cout << "brake" << endl;
                tempBrakingDeceleration_ = tempBrakingDeceleration_ - (tempBrakingDeceleration_ * accelerationError); //less deceleration, car travel more, closer to ped
                // cout << "brake: " << tempBrakingDeceleration_ << endl;
                FloatType carTravelDist = ( (carV * stepTime_) + (0.5 * tempBrakingDeceleration_ * stepTime_ * stepTime_));
                // cout << "carTravelDist: " << carTravelDist << endl;
                carX += carTravelDist;
                carV = carV + (tempBrakingDeceleration_ * stepTime_);
                // cout << "carV: " << carV << endl;
            }
            // cout << "carV: " << carV << endl;
            carLower = carY - carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
            carUpper = carY + carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
            carRight = carX + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;
        }

        long numDecimals = 100;
        cout << "RAW VALUE CONSTRAINT DIST: " << leastDistance << endl;
        // leastDistance = leastDistance +0.35;
        leastDistance = std::ceil(leastDistance * numDecimals) / numDecimals;

        cout << "MIN CONSTRAINT DIST: " << leastDistance << " ST Y: " << leastDistanceY << "intersectIndexMin: " << intersectIndexMin << endl;

        cout << "firstIndexMin: " << firstIndexMin << endl;
        cout << "lastIndexMin: " << lastIndexMin << endl;
        cout << "intersectIndexMin: " << intersectIndexMin << endl;

        return leastDistance;

        // minimum ped steps needed to achive this distance
        // FloatType pedDist = 0;
        // while (pedDist < leastDistance) {
        //     pedDist += (MAX_SPEED * stepTime_);
        // }
        // cout << "pedDist: " << pedDist << endl;        
        

        // return pedDist;

    }


    long firstIndexMax;
    long lastIndexMax;

    virtual FloatType getMaxConstraintUsingST(const HeuristicInfo* heuristicInfo) {
        fixedVelocity_ = generalOptions_->fixedVelocity;
        brakingDeceleration_ = generalOptions_->brakingDeceleration;
        cout << "----------------------------------------------------------------------------------" << endl;
        cout << "GETTING MAX CONSTRAINT DIST (CAR TRAVEL LESS): " << endl;
        // Constant for problem
        const FloatType MAX_SPEED = 2.5;
        // Retrieve information from the state
        VectorFloat stateVec = heuristicInfo->currentState->as<VectorState>()->asVector();
        // User data associated with the state
        auto nextStateUserData = static_cast<SKDUserData*>(heuristicInfo->currentState->getUserData().get());
        TrajData pedSafeTraj = nextStateUserData->safeTrajectory;

        VectorFloat carStartPos = generalOptions_->carStartPos;

        FloatType currentVisitIndex = 0;
        //initial Ped Position
        TrajPoint currentSafeTrajPoint = pedSafeTraj[currentVisitIndex];
        FloatType STPointX = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
        FloatType STPointY = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];
        // initial car position
        FloatType carX = carStartPos[0];
        FloatType carY = carStartPos[1];
        FloatType carV = stateVec[STATE_INFO::CAR_SPEED];

        //ped and car dimensions
        FloatType carLower = carY - carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
        FloatType carUpper = carY + carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;        
        FloatType carLeft = carX - carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;
        FloatType carRight = carX + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;

        FloatType pedLower = STPointY - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
        FloatType pedUpper = STPointY + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
        FloatType pedLeft = STPointX - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
        FloatType pedRight = STPointX + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];

        //get FIRST INDEX point -> lowest intersect before passing car

        // FloatType tempLeftMostX = std::numeric_limits<FloatType>::infinity();

        for (int i=0; i<= pedSafeTraj.size() -1; i++) {

            TrajPoint tempCurrentSafeTrajPoint = pedSafeTraj[i];

            FloatType tempSTPointX = tempCurrentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
            FloatType tempSTPointY = tempCurrentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];

            FloatType tempPedLower = tempSTPointY - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
            FloatType tempPedUpper = tempSTPointY + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];

            // cout << tempSTPointX << " " << tempSTPointY << endl;

            if (tempPedUpper < carUpper) {
                break;
            }

            firstIndexMax = i;
        }

        // cout << "firstIndex: " << firstIndex << endl;

        //get stopping point -> lowest intersect before passing car

        // FloatType tempLeftMostX = std::numeric_limits<FloatType>::infinity();

        for (int i=0; i<= pedSafeTraj.size() -1; i++) {

            TrajPoint tempCurrentSafeTrajPoint = pedSafeTraj[i];

            FloatType tempSTPointX = tempCurrentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
            FloatType tempSTPointY = tempCurrentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];

            FloatType tempPedLower = tempSTPointY - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
            FloatType tempPedUpper = tempSTPointY + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];

            // cout << tempSTPointX << " " << tempSTPointY << endl;

            if (tempPedUpper < carLower) {
                break;
            }

            lastIndexMax = i;
        }

        // cout << "lastIndex: " << lastIndex << endl;

        FloatType distCarPedX;

        FloatType leastDistance = std::numeric_limits<FloatType>::infinity();
        
        FloatType leastDistanceForIndex = std::numeric_limits<FloatType>::infinity();

        //DIDNT TAKE INTO ACCOUNT POSSIBILITY OF COLLISION SINCE IT IS SAFE TRAJECTORY

        // for (int i=0; i<= pedSafeTraj.size() -1 ; i++) {
        for (int i=0; i<= lastIndexMax ; i++) {

            // cout << "t= " << i << endl;

            currentSafeTrajPoint = pedSafeTraj[i];
            STPointX = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
            STPointY = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];

            //Should be here, i+1 and car move after prev iteration - SEE TRANSITION PLUGIN dist to know coast or brake?
            distCarPedX = abs(STPointX - carX);

            //get car Right line
            Point A = make_pair(carRight, carUpper);
            Point B = make_pair(carRight, carLower);

            Point topLeft = make_pair(carLeft, carUpper);
            Point topRight = make_pair(carRight, carUpper);
            Point bottomLeft = make_pair(carLeft, carLower);
            Point bottomRight = make_pair(carRight, carLower);

            for (int j=i; j<= i ; j++) {
                // cout << "   compare: " << j << endl;
                //get ST index
                TrajPoint tempCurrentSafeTrajPoint = pedSafeTraj[j];
                FloatType tempSTPointX = tempCurrentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
                FloatType tempSTPointY = tempCurrentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];
                //get ped point
                Point E = make_pair(tempSTPointX, tempSTPointY);

                //calc dist
                // double minDistToCar = minDistance(A, B, E);
                double minDistToCar = min({minDistance(topLeft, topRight, E) , minDistance(bottomLeft, bottomRight, E) , minDistance(topLeft, bottomLeft, E) , minDistance(topRight, bottomRight, E)});
                // cout << "       R: " << minDistance(topRight, bottomRight, E) << "       D: " << minDistance(bottomLeft, bottomRight, E) << endl;

                // // DIST SHOULD BE ST POINT TO INTERSECT POINT FROM ST- NOT MINIMUM!!!
                // FloatType distToCarTopRight = sqrt( pow(carRight - tempSTPointX, 2) + pow(carUpper - tempSTPointY, 2) );
                // FloatType distToCarBottomRight = sqrt( pow(carRight - tempSTPointX, 2) + pow(carLower - tempSTPointY, 2) );
                // FloatType distToCarMidRight = sqrt( pow(carRight - tempSTPointX, 2) + pow(carY - tempSTPointY, 2) );
                // double distToCar = max(distToCarTopRight, distToCarBottomRight);
                // cout << "   minDistToCar: " << minDistToCar << "   distToCarTopRight: " << distToCarTopRight << "   distToCarBottomRight: " << distToCarBottomRight << "   distToCarMidRight: " << distToCarMidRight  << "   dist: " << distToCar << endl;


                //minDisttoCar  means i need x more steps to reach car, so the constraint dist should count from current ST index plus the X number of steps
                // minimum ped steps needed to achive this distance
                FloatType pedDist = 0;
                int numSteps = 0;
                while (pedDist < minDistToCar) {
                    pedDist += (MAX_SPEED * stepTime_);
                    numSteps += 1;
                }
                // cout << "pedDist: " << pedDist << endl;
                // cout << "   numSteps: " << numSteps << endl;

                //get ST index for extra
                FloatType extraStep = j + numSteps;
                // cout << "   totalSteps: " << extraStep << endl;
                if (extraStep > pedSafeTraj.size()-1) {
                    extraStep = pedSafeTraj.size()-1;
                }
                TrajPoint extraCurrentSafeTrajPoint = pedSafeTraj[extraStep];
                FloatType extraSTPointX = extraCurrentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
                FloatType extraSTPointY = extraCurrentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];

                Point F = make_pair(extraSTPointX, extraSTPointY);
                double collideDistToCar = min({minDistance(topLeft, topRight, F) , minDistance(bottomLeft, bottomRight, F) , minDistance(topLeft, bottomLeft, F) , minDistance(topRight, bottomRight, F)});

                if (collideDistToCar < leastDistance) {
                    leastDistance = collideDistToCar;
                    leastDistanceY = tempSTPointY;
                    intersectIndexMax = i;
                }
                

                // // smallest dist within index range to get index
                // if ((i <= lastIndex) && (i >= firstIndex)) {
                //     if (minDistToCar < leastDistanceForIndex) {
                //         // cout << "INDEXED" << endl;
                //         leastDistanceForIndex = minDistToCar;
                //         intersectIndex = i;
                //     }
                // }

                // cout << "       minDistToCar: " << minDistToCar << "       comparing: " << std::setw(10) << tempSTPointX << " " << std::setw(10) << tempSTPointY << " car: " << std::setw(10) << carX << " " << std::setw(10) << carY << " " << std::setw(10) << carRight<< endl;
                // cout << "   collideDistToCar: " << collideDistToCar << endl;
            }

            // cout << "distCarPedX: " << distCarPedX << " baseStoppingDist_: " << baseStoppingDist_ << endl;
            // cout << "comparing: " << STPointX << " " << STPointY << " " << carX << " " << carY << endl;

            //CAR MAX MOVEMENT - CLOSEST TO PED
            if (carV <= 0) {
                carX = carX;
                carV = 0;
            }
            else if (distCarPedX > baseStoppingDist_) { //more dist so not braking yet /a=0                
                FloatType speedError = fixedVelocity_ * velocityError;
                carV = fixedVelocity_ - speedError;
                // cout << "coast v:" << (carV - speedError) << endl;
                carX += ( carV * stepTime_); //car v is always decreasing with error from previous value
            } else {
                // brakingDeceleration_ = brakingDeceleration_ + (brakingDeceleration_ * accelerationError); //less deceleration, car travel more, closer to ped
                // cout << "brake: " << brakingDeceleration_ << endl;
                FloatType carTravelDist = ( (carV * stepTime_) + (0.5 * brakingDeceleration_ * stepTime_ * stepTime_));
                // cout << "carTravelDist: " << carTravelDist << endl;
                carX += carTravelDist;
                carV = carV + (brakingDeceleration_ * stepTime_);
                // cout << "carV: " << carV << endl;
            }
            // cout << "carV: " << carV << endl;
            carLower = carY - carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
            carUpper = carY + carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
            carRight = carX + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;
        }

        long numDecimals = 100;
        cout << "RAW VALUE CONSTRAINT DIST: " << leastDistance << endl;
        // leastDistance = leastDistance +0.35;
        leastDistance = std::ceil(leastDistance * numDecimals) / numDecimals;

        cout << "MIN CONSTRAINT DIST: " << leastDistance << " ST Y: " << leastDistanceY << "intersectIndexMax: " << intersectIndexMax << endl;

        cout << "firstIndexMax: " << firstIndexMax << endl;
        cout << "lastIndexMax: " << lastIndexMax << endl;
        cout << "intersectIndexMax: " << intersectIndexMax << endl;

        return leastDistance;

        // // minimum ped steps needed to achive this distance
        // FloatType pedDist = 0;
        // while (pedDist < leastDistance) {
        //     pedDist += (MAX_SPEED * stepTime_);
        // }
        // cout << "pedDist: " << pedDist << endl;        
        

        // return pedDist;

    }


    virtual FloatType getHeuristicValue(const HeuristicInfo* heuristicInfo) const override {   

        FloatType brakingDeceleration_ = generalOptions_->brakingDeceleration;
        // Constant for problem
        const FloatType MAX_SPEED = 2.5;
        // Retrieve information from the state
        VectorFloat stateVec = heuristicInfo->currentState->as<VectorState>()->asVector();
        // User data associated with the state
        auto nextStateUserData = 
            static_cast<SKDUserData*>(heuristicInfo->currentState->getUserData().get());

        FloatType currentVisitIndex = nextStateUserData->visitIndex;
        // cout << "currentVisitIndex: " << currentVisitIndex << endl;

        //Use max? get better estimates
        FloatType stepsToIntersect = intersectIndexMax -currentVisitIndex;

         // Retrieve safe trajectory and step time
        TrajData pedSafeTraj = nextStateUserData->safeTrajectory;
        TrajPoint currentSafeTrajPoint = pedSafeTraj[currentVisitIndex];

        FloatType STPointX = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
        FloatType STPointY = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];   

        // Further information from general options
        FloatType stepPenalty_ = generalOptions_->stepPenalty;

        FloatType constraint = heuristicInfo->constraint;

        // cout << endl; cout << "Constraint dist: " << constraint << " currentVisitIndex: " << currentVisitIndex << " intersectIndex: " << intersectIndex << " constraint: " << constraint << endl;

        // Extract location information from state
        VectorFloat pedLocation{stateVec[STATE_INFO::PED_LONGIT], stateVec[STATE_INFO::PED_HORIZONTAL]};
        VectorFloat carLocation{stateVec[STATE_INFO::CAR_LONGIT], stateVec[STATE_INFO::CAR_HORIZONTAL]};

        FloatType pedInitialX = stateVec[STATE_INFO::PED_LONGIT];
        FloatType pedInitialY = stateVec[STATE_INFO::PED_HORIZONTAL];
        
        FloatType carInitialX = stateVec[STATE_INFO::CAR_LONGIT];
        FloatType carInitialY = stateVec[STATE_INFO::CAR_HORIZONTAL];

        FloatType carInitialV = stateVec[STATE_INFO::CAR_SPEED];

        // cout << "pedLocationX: " << pedInitialX << " pedLocationY: " << pedInitialY << " carInitialV: " << carInitialV  << endl;


        FloatType carLower = carInitialY - carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
        FloatType carUpper = carInitialY + carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;        
        FloatType carLeft = carInitialX - carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;
        FloatType carRight = carInitialX + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;

        FloatType pedLower = pedInitialY - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
        FloatType pedUpper = pedInitialY + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
        FloatType pedLeft = pedInitialX - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
        FloatType pedRight = pedInitialX + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];


        // cout << "carLocation: " << stateVec[STATE_INFO::CAR_LONGIT] << " " << stateVec[STATE_INFO::CAR_HORIZONTAL] << " upper: " << carUpper << " lower: " << carLower   << endl;

        //==================================================================================================================
        FloatType pedX = pedInitialX;
        FloatType pedY = pedInitialY;
        FloatType carX = carInitialX;
        FloatType carY = carInitialY;
        FloatType carV = carInitialV;

        float numStepsTaken = 0;
        FloatType distToSafe = 0;

        //figure out intersect index stuffs
        TrajPoint intersectSafeTrajPoint = pedSafeTraj[intersectIndexMax];
        FloatType STIntersectPointX = intersectSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
        FloatType STIntersectPointY = intersectSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];
        FloatType distToIntersectPoint = sqrt( pow(STIntersectPointX - pedInitialX, 2) + pow(STIntersectPointY - pedInitialY, 2) );
        FloatType lengthX = abs(STIntersectPointX - pedInitialX);
        FloatType lengthY = abs(STIntersectPointY - pedInitialY);
        FloatType numStepsToIntersectPoint = distToIntersectPoint/ (MAX_SPEED * stepTime_);

        FloatType theta;
        FloatType xStep;
        FloatType yStep;

        FloatType hypoStep = (MAX_SPEED * stepTime_);
        theta = asin(lengthX/lengthY);
        xStep = hypoStep * sin(theta);
        yStep = hypoStep * cos(theta);

        // cout << "theta: " << theta << " xStep: " << xStep << " yStep: " << yStep << endl;
        // cout << " Int X: " << STIntersectPointX << " Int Y: " << STIntersectPointY << " numStepsToIntersectPoint: " << numStepsToIntersectPoint << endl;
        
        bool violateConstraint = false;
        bool collide = false;

        // for (int i =0; i < floor(numStepsToIntersectPoint); i++) {
        // for (int i =0; i < ceil(numStepsToIntersectPoint); i++) {
        for (int i =0; i < numStepsToIntersectPoint; i++) {

            //CALCULATE THIS BEFORE PROPOGATE PEDESTRIAN- SEE TRANSITION PLUGIN dist to know coast or brake?
            FloatType distCarPedX = abs(STPointX - carX);

            //get next ST coordinates
            if (currentVisitIndex > pedSafeTraj.size()-1)
                currentVisitIndex = pedSafeTraj.size()-1;
            FloatType nextIndex = currentVisitIndex + 1;
            if (nextIndex > pedSafeTraj.size()-1)
                nextIndex = pedSafeTraj.size()-1;
            TrajPoint nextSafeTrajPoint = pedSafeTraj[nextIndex]; 
            FloatType nextSTPointX = nextSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
            FloatType nextSTPointY = nextSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];
            // cout << "   nextST: "<<  nextSTPointX << " " << nextSTPointY << endl;
            //for All actions
            int bestActionIndex;
            FloatType bestDistToNextST = std::numeric_limits<FloatType>::infinity();
             //testActions
            for (int a = 0; a < allActionsVector.size(); a++) {
                FloatType xMove = allActionsVector[a].first;
                FloatType yMove = allActionsVector[a].second;
                // cout << "   a: "<<  xMove << " " << yMove << endl;

                FloatType nextX = pedX + (xMove * stepTime_);
                FloatType nextY = pedY + (yMove * stepTime_);
                // cout << "   next state: "<<  nextX << " " << nextY << endl;

                FloatType nextDistToSafe = sqrt( pow(nextSTPointX - nextX, 2) + pow(nextSTPointY - nextY, 2) );
                // cout << "       nextDistToSafe: " << nextDistToSafe << endl;
                if (nextDistToSafe < bestDistToNextST) {
                    bestDistToNextST = nextDistToSafe;
                    bestActionIndex = a;
                }
                // cout << "       bestActionIndex: " << bestActionIndex << endl;            
            }
            // cout << "   best action:" << allActionsVector[bestActionIndex].first << " " << allActionsVector[bestActionIndex].second << endl;

            // execute action
            pedX += (allActionsVector[bestActionIndex].first  * stepTime_);
            pedY += (allActionsVector[bestActionIndex].second * stepTime_);

            // cout << "i"<< i << "   currentVisitIndex: " << currentVisitIndex << endl;

            //car movement
            if (carV <= 0) {
                carX = carX;
                carV = 0;
            }
            else if (distCarPedX > baseStoppingDist_) { //more dist so not braking yet /a=0
                carX += (carV * stepTime_);
            } else {
                carX += ( (carV * stepTime_) + (0.5 * brakingDeceleration_ * stepTime_ * stepTime_));
                carV = carV + (brakingDeceleration_ * stepTime_);
            }

            //UPDATE PED AND CAR BOUNDS
            carLower = carY - carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
            carUpper = carY + carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
            carLeft = carX - carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;
            carRight = carX + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;

            pedLower = pedY - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
            pedUpper = pedY + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
            pedLeft = pedX - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
            pedRight = pedX + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];

            //propagate ST as well before making comparison
            currentVisitIndex += 1;
            numStepsTaken += 1;

            //get ST coordinates
            if (currentVisitIndex > pedSafeTraj.size()-1) {
                currentVisitIndex = pedSafeTraj.size()-1;
            }
            currentSafeTrajPoint = pedSafeTraj[currentVisitIndex];
            STPointX = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
            STPointY = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];

            // point 2 point matching to check
            distToSafe = sqrt( pow(STPointX - pedX, 2) + pow(STPointY - pedY, 2) );

            //shortest dist to ST
            // distToSafe = std::numeric_limits<FloatType>::infinity();
            // Point E = make_pair(pedX, pedY);
            // for (int i = 0; i < pedSafeTraj.size() - 1; i++) {
            //     TrajPoint tmpSafeTrajPoint = pedSafeTraj[i];
            //     double STPointX1 = tmpSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
            //     double STPointY1 = tmpSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];
            //     tmpSafeTrajPoint = pedSafeTraj[i+1];
            //     double STPointX2 = tmpSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
            //     double STPointY2 = tmpSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ]; 
                
            //     Point A = make_pair(STPointX1, STPointY1);
            //     Point B = make_pair(STPointX2, STPointY2);
                
            //     double distToST = minDistance(A, B, E);
            //     // cout << "   distToST: " << distToST << endl; 
            //     if (distToST < distToSafe) {
            //         distToSafe = distToST;
            //     }      
            // }

            // cout << "   car: " << carX << " " << carY  << " ped: " << pedX << " " << pedY << " ST: " << STPointX << " " << STPointY << endl;
            // cout << "   distToSafe1: " << distToSafe << endl;  

            //break if constraint violation             
            if (distToSafe > constraint) {
                // cout << "constraint" << endl;
                violateConstraint = true;
                break;
            }
            //if ped collide with car
            // if ((carLower <= pedUpper) && (pedLower <= carUpper) && (carLeft <= pedRight) && (pedLeft <= carRight) ) {
            if (checkOverlap(pedRadius, pedX, pedY, carLeft, carLower, carRight, carUpper)) {
                // cout << "collide" << endl;
                collide = true;
                break;
            }  
        }
        //==================================================================================================================
        // cout << "par2" << endl;
        if (violateConstraint) {
            FloatType constraintVal = 0;
            for (int i = 1; i <= numStepsTaken; i++) {
                constraintVal += -(stepPenalty_) * pow(discountFactor, i);
            }

            return constraintVal;
        }
        else if (collide) {
            FloatType collideVal = 0;
            for (int i = 1; i < numStepsTaken; i++) {
                collideVal += -(stepPenalty_) * pow(discountFactor, i);
            }
            collideVal += generalOptions_->goalReward * pow(discountFactor, numStepsTaken);
            return collideVal;           
        }

        while (true) {
            //CALCULATE THIS BEFORE PROPOGATE PEDESTRIAN- SEE TRANSITION PLUGIN dist to know coast or brake?
            FloatType distCarPedX = abs(STPointX - carX);

            //PROPAGATE CAR FIRST BECAUSE WANT TO KNOW WHICH PED ACTION IS CLOSEST
            //car movement
            if (carV <= 0) {
                carX = carX;
                carV = 0;
            }
            else if (distCarPedX > baseStoppingDist_) { //more dist so not braking yet /a=0
                carX += (carV * stepTime_);
            } else {
                carX += ( (carV * stepTime_) + (0.5 * brakingDeceleration_ * stepTime_ * stepTime_));
                carV = carV + (brakingDeceleration_ * stepTime_);
            }

            //UPDATE PED AND CAR BOUNDS
            carLower = carY - carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
            carUpper = carY + carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
            carLeft = carX - carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;
            carRight = carX + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;


            //PED ACTION

            //for All actions
            int bestActionIndex;
            FloatType bestDistToCar = std::numeric_limits<FloatType>::infinity();
             //testActions
            for (int a = 0; a < allActionsVector.size(); a++) {
                FloatType xMove = allActionsVector[a].first;
                FloatType yMove = allActionsVector[a].second;
                // cout << "   a: "<<  xMove << " " << yMove << endl;

                FloatType nextX = pedX + (xMove * stepTime_);
                FloatType nextY = pedY + (yMove * stepTime_);
                // cout << "   next state: "<<  nextX << " " << nextY << endl;

                Point pedNextXY = make_pair(nextX, nextY);

                Point topLeft = make_pair(carLeft, carUpper);
                Point topRight = make_pair(carRight, carUpper);
                Point bottomLeft = make_pair(carLeft, carLower);
                Point bottomRight = make_pair(carRight, carLower);

                double nextDistToCar = min({minDistance(topLeft, topRight, pedNextXY) , minDistance(bottomLeft, bottomRight, pedNextXY) , minDistance(topLeft, bottomLeft, pedNextXY) , minDistance(topRight, bottomRight, pedNextXY)});

                // FloatType nextDistToCar = sqrt( pow(nextSTPointX - nextX, 2) + pow(nextSTPointY - nextY, 2) );
                // cout << "       nextDistToSafe: " << nextDistToSafe << endl;
                if (nextDistToCar < bestDistToCar) {
                    bestDistToCar = nextDistToCar;
                    bestActionIndex = a;
                }
                // cout << "       bestActionIndex: " << bestActionIndex << endl;            
            }
            // cout << "   best action:" << allActionsVector[bestActionIndex].first << " " << allActionsVector[bestActionIndex].second << endl;

            // execute action
            pedX += (allActionsVector[bestActionIndex].first  * stepTime_);
            pedY += (allActionsVector[bestActionIndex].second * stepTime_);

            //UPDATE PED BOUNDS
            pedLower = pedY - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
            pedUpper = pedY + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
            pedLeft = pedX - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
            pedRight = pedX + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];

            //propagate ST as well before making comparison
            currentVisitIndex += 1;
            numStepsTaken += 1;

            //get ST coordinates
            if (currentVisitIndex > pedSafeTraj.size()-1) {
                currentVisitIndex = pedSafeTraj.size()-1;
            }
            currentSafeTrajPoint = pedSafeTraj[currentVisitIndex];
            STPointX = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
            STPointY = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];

            // point 2 point matching to check
            distToSafe = sqrt( pow(STPointX - pedX, 2) + pow(STPointY - pedY, 2) );

            // cout << "   car: " << carX << " " << carY << "   car v: " << carV  << " ped: " << pedX << " " << pedY << " ST: " << STPointX << " " << STPointY << endl;
            // cout << "   distToSafe2: " << distToSafe << endl;  

            //after propagation
            // cout << "Step: " << numStepsTaken << " D to ST: " << distToSafe << endl;
            // cout << "car: " << carX << " " << carY << " upper: " << carUpper << " lower: " << carLower  << carY << " left: " << carLeft << " right: " << carRight << endl;
            // cout << "ped: " << pedX << " " << pedY << " upper: " << pedUpper << " lower: " << pedLower  << carY << " left: " << pedLeft << " right: " << pedRight << endl;
            // cout << "car: " << carX << " " << carY  << endl;
            // cout << "ped: " << pedX << " " << pedY << " ST: " << STPointX << " " << STPointY << endl;

            //break if constraint violation 
            
            if (distToSafe > constraint) {
                // cout << "constraint" << endl;
                violateConstraint = true;
                break;
            }
            //if ped collide with car
            // if ((carLower <= pedUpper) && (pedLower <= carUpper) && (carLeft <= pedRight) && (pedLeft <= carRight) ) {  //pedLeft < carRight
            if (checkOverlap(pedRadius, pedX, pedY, carLeft, carLower, carRight, carUpper)) {
                // cout << "collide" << endl;
                collide = true;
                break;
            }  


        }

        if (violateConstraint) {
            FloatType constraintVal = 0;
            for (int i = 1; i <= numStepsTaken; i++) {
                constraintVal += -(stepPenalty_) * pow(discountFactor, i);
            }

            return constraintVal;
        }
        else if (collide) {
            FloatType collideVal = 0;
            for (int i = 1; i < numStepsTaken; i++) {
                collideVal += -(stepPenalty_) * pow(discountFactor, i);
            }
            collideVal += generalOptions_->goalReward * pow(discountFactor, numStepsTaken);
            return collideVal;           
        }

    }



    virtual FloatType getMinConstraintRefine(const HeuristicInfo* heuristicInfo) {   
        // cout << "-------------------------------------------------------------------------------"  << endl;
        // cout << "MIN REFINE: "  << endl;
        // Constant for problem
        const FloatType MAX_SPEED = 2.5;
        // Retrieve information from the state
        VectorFloat stateVec = heuristicInfo->currentState->as<VectorState>()->asVector();
        // User data associated with the state
        auto nextStateUserData = 
            static_cast<SKDUserData*>(heuristicInfo->currentState->getUserData().get());

        FloatType currentVisitIndex = nextStateUserData->visitIndex;
        // cout << "currentVisitIndex: " << currentVisitIndex << endl;

        //Use max? get better estimates
        FloatType stepsToIntersect = intersectIndexMin -currentVisitIndex;

         // Retrieve safe trajectory and step time
        TrajData pedSafeTraj = nextStateUserData->safeTrajectory;
        TrajPoint currentSafeTrajPoint = pedSafeTraj[currentVisitIndex];

        FloatType STPointX = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
        FloatType STPointY = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];   

        // Further information from general options
        FloatType stepPenalty_ = generalOptions_->stepPenalty;

        // Extract location information from state
        VectorFloat pedLocation{stateVec[STATE_INFO::PED_LONGIT], stateVec[STATE_INFO::PED_HORIZONTAL]};
        VectorFloat carLocation{stateVec[STATE_INFO::CAR_LONGIT], stateVec[STATE_INFO::CAR_HORIZONTAL]};

        FloatType pedInitialX = stateVec[STATE_INFO::PED_LONGIT];
        FloatType pedInitialY = stateVec[STATE_INFO::PED_HORIZONTAL];
        
        FloatType carInitialX = stateVec[STATE_INFO::CAR_LONGIT];
        FloatType carInitialY = stateVec[STATE_INFO::CAR_HORIZONTAL];

        FloatType carInitialV = stateVec[STATE_INFO::CAR_SPEED];

        // cout << "pedLocationX: " << pedInitialX << " pedLocationY: " << pedInitialY << " carInitialX: " << carInitialX << " carInitialY: " << carInitialY << " carInitialV: " << carInitialV  << endl;


        FloatType carLower = carInitialY - carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
        FloatType carUpper = carInitialY + carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;        
        FloatType carLeft = carInitialX - carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;
        FloatType carRight = carInitialX + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;

        FloatType pedLower = pedInitialY - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
        FloatType pedUpper = pedInitialY + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
        FloatType pedLeft = pedInitialX - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
        FloatType pedRight = pedInitialX + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];


        // cout << "carLocation: " << stateVec[STATE_INFO::CAR_LONGIT] << " " << stateVec[STATE_INFO::CAR_HORIZONTAL] << " upper: " << carUpper << " lower: " << carLower   << endl;

        //==================================================================================================================
        FloatType pedX = pedInitialX;
        FloatType pedY = pedInitialY;
        FloatType carX = carInitialX;
        FloatType carY = carInitialY;
        FloatType carV = carInitialV;

        float numStepsTaken = 0;
        FloatType distToSafe = 0;

        //figure out intersect index stuffs
        TrajPoint intersectSafeTrajPoint = pedSafeTraj[intersectIndexMin];
        FloatType STIntersectPointX = intersectSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
        FloatType STIntersectPointY = intersectSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];
        FloatType distToIntersectPoint = sqrt( pow(STIntersectPointX - pedInitialX, 2) + pow(STIntersectPointY - pedInitialY, 2) );
        FloatType lengthX = abs(STIntersectPointX - pedInitialX);
        FloatType lengthY = abs(STIntersectPointY - pedInitialY);
        FloatType numStepsToIntersectPoint = distToIntersectPoint/ (MAX_SPEED * stepTime_);

        FloatType theta;
        FloatType xStep;
        FloatType yStep;

        FloatType hypoStep = (MAX_SPEED * stepTime_);
        theta = asin(lengthX/lengthY);
        xStep = hypoStep * sin(theta);
        yStep = hypoStep * cos(theta);

        // cout << "theta: " << theta << " xStep: " << xStep << " yStep: " << yStep << endl;
        // cout << " Int X: " << STIntersectPointX << " Int Y: " << STIntersectPointY << " numStepsToIntersectPoint: " << numStepsToIntersectPoint << endl;
        
        bool violateConstraint = false;
        bool collide = false;

        FloatType minToSafeForMin = std::numeric_limits<FloatType>::infinity(); 

        int stepsTilEndOfST = pedSafeTraj.size()-1 - currentVisitIndex;
        // cout << "stepsTilEndOfST: " << stepsTilEndOfST << endl;

        for (int t =0; t < stepsToIntersect; t++) {

            collide = false;
            FloatType trajMaxDistToST = -std::numeric_limits<FloatType>::infinity();
            // cout << "deviate at t: " << t << endl;

            FloatType carLower = carInitialY - carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
            FloatType carUpper = carInitialY + carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;        
            FloatType carLeft = carInitialX - carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;
            FloatType carRight = carInitialX + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;

            FloatType pedLower = pedInitialY - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
            FloatType pedUpper = pedInitialY + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
            FloatType pedLeft = pedInitialX - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
            FloatType pedRight = pedInitialX + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];

            FloatType pedX = pedInitialX;
            FloatType pedY = pedInitialY;
            FloatType carX = carInitialX;
            FloatType carY = carInitialY;
            FloatType carV = carInitialV;

            brakingDeceleration_ = generalOptions_->brakingDeceleration;
            FloatType currIndex = currentVisitIndex;

            for (int i =0; i < t; i++) {
                
                //CALCULATE THIS BEFORE PROPOGATE PEDESTRIAN- SEE TRANSITION PLUGIN dist to know coast or brake?
                FloatType distCarPedX = abs(STPointX - carX);

                //get next ST coordinates
                if (currIndex > pedSafeTraj.size()-1)
                    currIndex = pedSafeTraj.size()-1;
                FloatType nextIndex = currIndex + 1;
                if (nextIndex > pedSafeTraj.size()-1)
                    nextIndex = pedSafeTraj.size()-1;
                TrajPoint nextSafeTrajPoint = pedSafeTraj[nextIndex];               
                FloatType nextSTPointX = nextSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
                FloatType nextSTPointY = nextSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];
                // cout << "   nextST: "<<  nextSTPointX << " " << nextSTPointY << endl;
                //for All actions
                int bestActionIndex;
                FloatType bestDistToNextST = std::numeric_limits<FloatType>::infinity();
                 //testActions
                for (int a = 0; a < allActionsVector.size(); a++) {
                    FloatType xMove = allActionsVector[a].first;
                    FloatType yMove = allActionsVector[a].second;
                    // cout << "   a: "<<  xMove << " " << yMove << endl;

                    FloatType nextX = pedX + (xMove * stepTime_);
                    FloatType nextY = pedY + (yMove * stepTime_);
                    // cout << "   next state: "<<  nextX << " " << nextY << endl;

                    FloatType nextDistToSafe = sqrt( pow(nextSTPointX - nextX, 2) + pow(nextSTPointY - nextY, 2) );
                    // cout << "       nextDistToSafe: " << nextDistToSafe << endl;
                    if (nextDistToSafe < bestDistToNextST) {
                        bestDistToNextST = nextDistToSafe;
                        bestActionIndex = a;
                    }
                    // cout << "       bestActionIndex: " << bestActionIndex << endl;            
                }
                // cout << "   best action:" << allActionsVector[bestActionIndex].first << " " << allActionsVector[bestActionIndex].second << endl;

                // execute action
                pedX += (allActionsVector[bestActionIndex].first  * stepTime_);
                pedY += (allActionsVector[bestActionIndex].second * stepTime_);

                // cout << "i"<< i << "   currIndex: " << currIndex << endl;

                //CAR MAX MOVEMENT - CLOSEST TO PED
                if (carV <= 0) {
                    carX = carX;
                    carV = 0;
                }
                else if (distCarPedX > baseStoppingDist_) { //more dist so not braking yet /a=0
                    // cout << "coast v:" << carV << endl;
                    carX += (carV * stepTime_); //car v is clamped max to fixed V- so assume as long as costing- always max V
                } else {
                    // cout << "brake" << endl;
                    brakingDeceleration_ = brakingDeceleration_ - (brakingDeceleration_ * accelerationError); //less deceleration, car travel more, closer to ped
                    // cout << "brake: " << brakingDeceleration_ << endl;
                    FloatType carTravelDist = ( (carV * stepTime_) + (0.5 * brakingDeceleration_ * stepTime_ * stepTime_));
                    // cout << "carTravelDist: " << carTravelDist << endl;
                    carX += carTravelDist;
                    carV = carV + (brakingDeceleration_ * stepTime_);
                    // cout << "carV: " << carV << endl;
                }

                // cout << "pedX: " << pedX << " pedY: " << pedY << " carX: " << carX << " carY: " << carY << " carV: " << carV << " carV: " << carV << endl;

                //UPDATE PED AND CAR BOUNDS
                carLower = carY - carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
                carUpper = carY + carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
                carLeft = carX - carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;
                carRight = carX + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;

                pedLower = pedY - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
                pedUpper = pedY + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
                pedLeft = pedX - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
                pedRight = pedX + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];

                //propagate ST as well before making comparison
                currIndex += 1;
                numStepsTaken += 1;

                //get ST coordinates
                if (currIndex > pedSafeTraj.size()-1) {
                    currIndex = pedSafeTraj.size()-1;
                }
                currentSafeTrajPoint = pedSafeTraj[currIndex];
                STPointX = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
                STPointY = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];

                // point 2 point matching to check
                distToSafe = sqrt( pow(STPointX - pedX, 2) + pow(STPointY - pedY, 2) );
                if (distToSafe > trajMaxDistToST){
                    trajMaxDistToST = distToSafe;
                }

                // cout << "   distToSafe1: " << distToSafe << endl; 
                
                //if ped collide with car
                // if ((carLower <= pedUpper) && (pedLower <= carUpper) && (carLeft <= pedRight) && (pedLeft <= carRight) ) {
                if (checkOverlap(pedRadius, pedX, pedY, carLeft, carLower, carRight, carUpper)) {
                    // cout << "collide: distToSafe: " << distToSafe << endl;
                    collide = true;
                    break;
                } 
                // cout << "test1: "  << endl;

                if (carRight >= pedRight) {
                    collide = true;
                    break;
                } 
                // cout << "test2: "  << endl;
            }
            //==================================================================================================================
            // cout << "deviate"  << endl;
            // if (collide) {
            //     cout << "part 1 collide: " << collide << endl;        
            // }
            // cout << "test2: " << t << endl;

            while (true && !collide) {
                //CALCULATE THIS BEFORE PROPOGATE PEDESTRIAN- SEE TRANSITION PLUGIN dist to know coast or brake?
                FloatType distCarPedX = abs(STPointX - carX);

                //PROPAGATE CAR FIRST BECAUSE WANT TO KNOW WHICH PED ACTION IS CLOSEST
                //CAR MAX MOVEMENT - CLOSEST TO PED
                if (carV <= 0) {
                    carX = carX;
                    carV = 0;
                }
                else if (distCarPedX > baseStoppingDist_) { //more dist so not braking yet /a=0
                    // cout << "coast v:" << carV << endl;
                    carX += (carV * stepTime_); //car v is clamped max to fixed V- so assume as long as costing- always max V
                } else {
                    // cout << "brake" << endl;
                    brakingDeceleration_ = brakingDeceleration_ - (brakingDeceleration_ * accelerationError); //less deceleration, car travel more, closer to ped
                    // cout << "brake: " << brakingDeceleration_ << endl;
                    FloatType carTravelDist = ( (carV * stepTime_) + (0.5 * brakingDeceleration_ * stepTime_ * stepTime_));
                    // cout << "carTravelDist: " << carTravelDist << endl;
                    carX += carTravelDist;
                    carV = carV + (brakingDeceleration_ * stepTime_);
                    // cout << "carV: " << carV << endl;
                }

                //UPDATE PED AND CAR BOUNDS
                carLower = carY - carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
                carUpper = carY + carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
                carLeft = carX - carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;
                carRight = carX + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;


                //PED ACTION

                //for All actions
                int bestActionIndex;
                FloatType bestDistToCar = std::numeric_limits<FloatType>::infinity();
                 //testActions
                for (int a = 0; a < allActionsVector.size(); a++) {
                    FloatType xMove = allActionsVector[a].first;
                    FloatType yMove = allActionsVector[a].second;
                    // cout << "   a: "<<  xMove << " " << yMove << endl;

                    FloatType nextX = pedX + (xMove * stepTime_);
                    FloatType nextY = pedY + (yMove * stepTime_);
                    // cout << "   next state: "<<  nextX << " " << nextY << endl;

                    Point pedNextXY = make_pair(nextX, nextY);

                    Point topLeft = make_pair(carLeft, carUpper);
                    Point topRight = make_pair(carRight, carUpper);
                    Point bottomLeft = make_pair(carLeft, carLower);
                    Point bottomRight = make_pair(carRight, carLower);

                    double nextDistToCar = min({minDistance(topLeft, topRight, pedNextXY) , minDistance(bottomLeft, bottomRight, pedNextXY) , minDistance(topLeft, bottomLeft, pedNextXY) , minDistance(topRight, bottomRight, pedNextXY)});

                    // FloatType nextDistToCar = sqrt( pow(nextSTPointX - nextX, 2) + pow(nextSTPointY - nextY, 2) );
                    // cout << "       nextDistToSafe: " << nextDistToSafe << endl;
                    if (nextDistToCar < bestDistToCar) {
                        bestDistToCar = nextDistToCar;
                        bestActionIndex = a;
                    }
                    // cout << "       bestActionIndex: " << bestActionIndex << endl;            
                }
                // cout << "   best action:" << allActionsVector[bestActionIndex].first << " " << allActionsVector[bestActionIndex].second << endl;

                // execute action
                pedX += (allActionsVector[bestActionIndex].first  * stepTime_);
                pedY += (allActionsVector[bestActionIndex].second * stepTime_);

                // cout << "pedX: " << pedX << " pedY: " << pedY << " carX: " << carX << " carY: " << carY << " carV: " << carV << " carV: " << carV << endl;

                //UPDATE PED BOUNDS
                pedLower = pedY - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
                pedUpper = pedY + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
                pedLeft = pedX - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
                pedRight = pedX + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];

                //propagate ST as well before making comparison
                currIndex += 1;
                numStepsTaken += 1;

                //get ST coordinates
                if (currIndex > pedSafeTraj.size()-1) {
                    currIndex = pedSafeTraj.size()-1;
                }
                currentSafeTrajPoint = pedSafeTraj[currIndex];
                STPointX = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
                STPointY = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];

                // point 2 point matching to check
                distToSafe = sqrt( pow(STPointX - pedX, 2) + pow(STPointY - pedY, 2) );

                if (distToSafe > trajMaxDistToST){
                    trajMaxDistToST = distToSafe;
                }
                // cout << "   car: " << carX << " " << carY << "   car v: " << carV  << " ped: " << pedX << " " << pedY << " ST: " << STPointX << " " << STPointY << endl;
                // cout << "   distToSafe2: " << distToSafe << endl;  

                //after propagation
                // cout << "Step: " << numStepsTaken << " D to ST: " << distToSafe << endl;
                // cout << "car: " << carX << " " << carY << " upper: " << carUpper << " lower: " << carLower  << carY << " left: " << carLeft << " right: " << carRight << endl;
                // cout << "ped: " << pedX << " " << pedY << " upper: " << pedUpper << " lower: " << pedLower  << carY << " left: " << pedLeft << " right: " << pedRight << endl;
                // cout << "car: " << carX << " " << carY  << endl;
                // cout << "ped: " << pedX << " " << pedY << " ST: " << STPointX << " " << STPointY << endl;


                //if ped collide with car
                // if ((carLower <= pedUpper) && (pedLower <= carUpper) && (carLeft <= pedRight) && (pedLeft <= carRight) ) {  //pedLeft < carRight
                if (checkOverlap(pedRadius, pedX, pedY, carLeft, carLower, carRight, carUpper)) {
                    // cout << "collide" << endl;
                    collide = true;
                    break;
                }  
            }

            // if (collide) {
            //     break;           
            // }
            
        // cout << "collide: MAX traj distToSafe: " << trajMaxDistToST << endl;
        // cout << "collide: distToSafe: " << distToSafe << endl;

        
        if (trajMaxDistToST < minToSafeForMin){
            minToSafeForMin = trajMaxDistToST;
        }

        }

        // cout << "minToSafeForMin: " << minToSafeForMin << endl;
        return minToSafeForMin;
    }


    virtual FloatType getMaxConstraintRefine(const HeuristicInfo* heuristicInfo) {   

        // cout << "-------------------------------------------------------------------------------"  << endl;
        // cout << "MAX REFINE: "  << endl;

        // Constant for problem
        const FloatType MAX_SPEED = 2.5;
        // Retrieve information from the state
        VectorFloat stateVec = heuristicInfo->currentState->as<VectorState>()->asVector();
        // User data associated with the state
        auto nextStateUserData = 
            static_cast<SKDUserData*>(heuristicInfo->currentState->getUserData().get());

        FloatType currentVisitIndex = nextStateUserData->visitIndex;
        // cout << "currentVisitIndex: " << currentVisitIndex << endl;

        //Use max? get better estimates
        FloatType stepsToIntersect = intersectIndexMax -currentVisitIndex;
        // cout << "stepsToIntersect: " << stepsToIntersect << endl;

         // Retrieve safe trajectory and step time
        TrajData pedSafeTraj = nextStateUserData->safeTrajectory;
        TrajPoint currentSafeTrajPoint = pedSafeTraj[currentVisitIndex];

        FloatType STPointX = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
        FloatType STPointY = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];   

        // Further information from general options
        FloatType stepPenalty_ = generalOptions_->stepPenalty;

        // Extract location information from state
        VectorFloat pedLocation{stateVec[STATE_INFO::PED_LONGIT], stateVec[STATE_INFO::PED_HORIZONTAL]};
        VectorFloat carLocation{stateVec[STATE_INFO::CAR_LONGIT], stateVec[STATE_INFO::CAR_HORIZONTAL]};

        FloatType pedInitialX = stateVec[STATE_INFO::PED_LONGIT];
        FloatType pedInitialY = stateVec[STATE_INFO::PED_HORIZONTAL];
        
        FloatType carInitialX = stateVec[STATE_INFO::CAR_LONGIT];
        FloatType carInitialY = stateVec[STATE_INFO::CAR_HORIZONTAL];

        FloatType carInitialV = stateVec[STATE_INFO::CAR_SPEED];

        // cout << "pedLocationX: " << pedInitialX << " pedLocationY: " << pedInitialY << " carInitialX: " << carInitialX << " carInitialY: " << carInitialY << " carInitialV: " << carInitialV  << endl;


        FloatType carLower = carInitialY - carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
        FloatType carUpper = carInitialY + carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;        
        FloatType carLeft = carInitialX - carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;
        FloatType carRight = carInitialX + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;

        FloatType pedLower = pedInitialY - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
        FloatType pedUpper = pedInitialY + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
        FloatType pedLeft = pedInitialX - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
        FloatType pedRight = pedInitialX + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];


        // cout << "carLocation: " << stateVec[STATE_INFO::CAR_LONGIT] << " " << stateVec[STATE_INFO::CAR_HORIZONTAL] << " upper: " << carUpper << " lower: " << carLower   << endl;

        //==================================================================================================================
        FloatType pedX = pedInitialX;
        FloatType pedY = pedInitialY;
        FloatType carX = carInitialX;
        FloatType carY = carInitialY;
        FloatType carV = carInitialV;

        float numStepsTaken = 0;
        FloatType distToSafe = 0;

        //figure out intersect index stuffs
        TrajPoint intersectSafeTrajPoint = pedSafeTraj[intersectIndexMax];
        FloatType STIntersectPointX = intersectSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
        FloatType STIntersectPointY = intersectSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];
        FloatType distToIntersectPoint = sqrt( pow(STIntersectPointX - pedInitialX, 2) + pow(STIntersectPointY - pedInitialY, 2) );
        FloatType lengthX = abs(STIntersectPointX - pedInitialX);
        FloatType lengthY = abs(STIntersectPointY - pedInitialY);
        FloatType numStepsToIntersectPoint = distToIntersectPoint/ (MAX_SPEED * stepTime_);

        FloatType theta;
        FloatType xStep;
        FloatType yStep;

        FloatType hypoStep = (MAX_SPEED * stepTime_);
        theta = asin(lengthX/lengthY);
        xStep = hypoStep * sin(theta);
        yStep = hypoStep * cos(theta);

        // cout << "theta: " << theta << " xStep: " << xStep << " yStep: " << yStep << endl;
        // cout << " Int X: " << STIntersectPointX << " Int Y: " << STIntersectPointY << " numStepsToIntersectPoint: " << numStepsToIntersectPoint << endl;
        
        bool violateConstraint = false;
        bool collide = false;

        FloatType minToSafeForMax = std::numeric_limits<FloatType>::infinity(); 

        int stepsTilEndOfST = pedSafeTraj.size()-1 - currentVisitIndex;
        // cout << "stepsTilEndOfST: " << stepsTilEndOfST << endl;

        for (int t =0; t < stepsToIntersect; t++) { //how many steps to take (before deviation) til car reach intersectpoint

            collide = false;

            // cout << "t: " << t << endl;
            FloatType trajMaxDistToST = -std::numeric_limits<FloatType>::infinity();

            FloatType carLower = carInitialY - carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
            FloatType carUpper = carInitialY + carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;        
            FloatType carLeft = carInitialX - carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;
            FloatType carRight = carInitialX + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;

            FloatType pedLower = pedInitialY - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
            FloatType pedUpper = pedInitialY + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
            FloatType pedLeft = pedInitialX - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
            FloatType pedRight = pedInitialX + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];

            FloatType pedX = pedInitialX;
            FloatType pedY = pedInitialY;
            FloatType carX = carInitialX;
            FloatType carY = carInitialY;
            FloatType carV = carInitialV;

            brakingDeceleration_ = generalOptions_->brakingDeceleration;
            FloatType currIndex = currentVisitIndex;

            for (int i =0; i < t; i++) {
                // cout << "i"<< i << endl;
                //CALCULATE THIS BEFORE PROPOGATE PEDESTRIAN- SEE TRANSITION PLUGIN dist to know coast or brake?
                FloatType distCarPedX = abs(STPointX - carX);

                //get next ST coordinates
                if (currIndex > pedSafeTraj.size()-1)
                    currIndex = pedSafeTraj.size()-1;
                FloatType nextIndex = currIndex + 1;
                if (nextIndex > pedSafeTraj.size()-1)
                    nextIndex = pedSafeTraj.size()-1;
                TrajPoint nextSafeTrajPoint = pedSafeTraj[nextIndex]; 
                FloatType nextSTPointX = nextSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
                FloatType nextSTPointY = nextSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];
                // cout << "   nextST: "<<  nextSTPointX << " " << nextSTPointY << endl;
                // cout << "test4: "  << endl;
                //for All actions
                int bestActionIndex;
                FloatType bestDistToNextST = std::numeric_limits<FloatType>::infinity();
                 //testActions
                for (int a = 0; a < allActionsVector.size(); a++) {
                    FloatType xMove = allActionsVector[a].first;
                    FloatType yMove = allActionsVector[a].second;
                    // cout << "   a: "<<  xMove << " " << yMove << endl;

                    FloatType nextX = pedX + (xMove * stepTime_);
                    FloatType nextY = pedY + (yMove * stepTime_);
                    // cout << "   next state: "<<  nextX << " " << nextY << endl;

                    FloatType nextDistToSafe = sqrt( pow(nextSTPointX - nextX, 2) + pow(nextSTPointY - nextY, 2) );
                    // cout << "       nextDistToSafe: " << nextDistToSafe << endl;
                    if (nextDistToSafe < bestDistToNextST) {
                        bestDistToNextST = nextDistToSafe;
                        bestActionIndex = a;
                    }
                    // cout << "       bestActionIndex: " << bestActionIndex << endl;            
                }
                // cout << "   best action:" << allActionsVector[bestActionIndex].first << " " << allActionsVector[bestActionIndex].second << endl;
                // execute action
                pedX += (allActionsVector[bestActionIndex].first  * stepTime_);
                pedY += (allActionsVector[bestActionIndex].second * stepTime_);

                // cout << "i"<< i << "   currIndex: " << currIndex << endl;

                //CAR MAX MOVEMENT - CLOSEST TO PED
                if (carV <= 0) {
                    carX = carX;
                    carV = 0;
                }
                else if (distCarPedX > baseStoppingDist_) { //more dist so not braking yet /a=0                
                    FloatType speedError = fixedVelocity_ * velocityError;
                    carV = fixedVelocity_ - speedError;
                    // cout << "coast v:" << (carV - speedError) << endl;
                    carX += ( carV * stepTime_); //car v is always decreasing with error from previous value
                } else {
                    // brakingDeceleration_ = brakingDeceleration_ + (brakingDeceleration_ * accelerationError); //less deceleration, car travel more, closer to ped
                    // cout << "brake: " << brakingDeceleration_ << endl;
                    FloatType carTravelDist = ( (carV * stepTime_) + (0.5 * brakingDeceleration_ * stepTime_ * stepTime_));
                    // cout << "carTravelDist: " << carTravelDist << endl;
                    carX += carTravelDist;
                    carV = carV + (brakingDeceleration_ * stepTime_);
                    // cout << "carV: " << carV << endl;
                }

                // cout << "pedX: " << pedX << " pedY: " << pedY << " carX: " << carX << " carY: " << carY << " carV: " << carV << " carV: " << carV << endl;

                //UPDATE PED AND CAR BOUNDS
                carLower = carY - carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
                carUpper = carY + carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
                carLeft = carX - carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;
                carRight = carX + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;

                pedLower = pedY - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
                pedUpper = pedY + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
                pedLeft = pedX - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
                pedRight = pedX + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];

                //propagate ST as well before making comparison
                currIndex += 1;
                numStepsTaken += 1;

                //get ST coordinates
                if (currIndex > pedSafeTraj.size()-1) {
                    currIndex = pedSafeTraj.size()-1;
                }
                currentSafeTrajPoint = pedSafeTraj[currIndex];
                STPointX = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
                STPointY = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];

                // point 2 point matching to check
                distToSafe = sqrt( pow(STPointX - pedX, 2) + pow(STPointY - pedY, 2) );
                // cout << "distToSafe: " << distToSafe << endl;
                
                if (distToSafe > trajMaxDistToST){
                    trajMaxDistToST = distToSafe;
                }

                //if ped collide with car
                // if ((carLower <= pedUpper) && (pedLower <= carUpper) && (carLeft <= pedRight) && (pedLeft <= carRight) ) {
                if (checkOverlap(pedRadius, pedX, pedY, carLeft, carLower, carRight, carUpper)) {
                    // cout << "collide: distToSafe: " << distToSafe << endl;
                    collide = true;
                    break;
                }
                // cout << "test1: "  << endl;

                if (carRight >= pedRight) {
                    collide = true;
                    break;
                }
                // cout << "test2: "  << endl;
            }
            //==================================================================================================================
            // cout << "part2: " << t << endl;
            // // if (collide) {
            // //     break;          
            // // }
            // cout << "test2: " << t << endl;

            while (true && !collide) {
                //CALCULATE THIS BEFORE PROPOGATE PEDESTRIAN- SEE TRANSITION PLUGIN dist to know coast or brake?
                FloatType distCarPedX = abs(STPointX - carX);

                //PROPAGATE CAR FIRST BECAUSE WANT TO KNOW WHICH PED ACTION IS CLOSEST
                //CAR MAX MOVEMENT - CLOSEST TO PED
                if (carV <= 0) {
                    carX = carX;
                    carV = 0;
                }
                else if (distCarPedX > baseStoppingDist_) { //more dist so not braking yet /a=0                
                    FloatType speedError = fixedVelocity_ * velocityError;
                    carV = fixedVelocity_ - speedError;
                    // cout << "coast v:" << (carV - speedError) << endl;
                    carX += ( carV * stepTime_); //car v is always decreasing with error from previous value
                } else {
                    // brakingDeceleration_ = brakingDeceleration_ + (brakingDeceleration_ * accelerationError); //less deceleration, car travel more, closer to ped
                    // cout << "brake: " << brakingDeceleration_ << endl;
                    FloatType carTravelDist = ( (carV * stepTime_) + (0.5 * brakingDeceleration_ * stepTime_ * stepTime_));
                    // cout << "carTravelDist: " << carTravelDist << endl;
                    carX += carTravelDist;
                    // cout << "carV: " << carV << endl;
                    // cout << "brakingDeceleration_: " << brakingDeceleration_ << endl;
                    // cout << "stepTime_: " << stepTime_ << endl;
                    carV = carV + (brakingDeceleration_ * stepTime_);
                    // cout << "carV: " << carV << endl;
                }

                //UPDATE PED AND CAR BOUNDS
                carLower = carY - carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
                carUpper = carY + carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
                carLeft = carX - carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;
                carRight = carX + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;


                //PED ACTION

                //for All actions
                int bestActionIndex;
                FloatType bestDistToCar = std::numeric_limits<FloatType>::infinity();
                 //testActions
                for (int a = 0; a < allActionsVector.size(); a++) {
                    FloatType xMove = allActionsVector[a].first;
                    FloatType yMove = allActionsVector[a].second;
                    // cout << "   a: "<<  xMove << " " << yMove << endl;

                    FloatType nextX = pedX + (xMove * stepTime_);
                    FloatType nextY = pedY + (yMove * stepTime_);
                    // cout << "   next state: "<<  nextX << " " << nextY << endl;

                    Point pedNextXY = make_pair(nextX, nextY);

                    Point topLeft = make_pair(carLeft, carUpper);
                    Point topRight = make_pair(carRight, carUpper);
                    Point bottomLeft = make_pair(carLeft, carLower);
                    Point bottomRight = make_pair(carRight, carLower);

                    double nextDistToCar = min({minDistance(topLeft, topRight, pedNextXY) , minDistance(bottomLeft, bottomRight, pedNextXY) , minDistance(topLeft, bottomLeft, pedNextXY) , minDistance(topRight, bottomRight, pedNextXY)});

                    // FloatType nextDistToCar = sqrt( pow(nextSTPointX - nextX, 2) + pow(nextSTPointY - nextY, 2) );
                    // cout << "       nextDistToSafe: " << nextDistToSafe << endl;
                    if (nextDistToCar < bestDistToCar) {
                        bestDistToCar = nextDistToCar;
                        bestActionIndex = a;
                    }
                    // cout << "       bestActionIndex: " << bestActionIndex << endl;            
                }
                // cout << "   best action:" << allActionsVector[bestActionIndex].first << " " << allActionsVector[bestActionIndex].second << endl;

                // execute action
                pedX += (allActionsVector[bestActionIndex].first  * stepTime_);
                pedY += (allActionsVector[bestActionIndex].second * stepTime_);

                // cout << "pedX: " << pedX << " pedY: " << pedY << " carX: " << carX << " carY: " << carY << " carV: " << carV << " carV: " << carV << endl;

                //UPDATE PED BOUNDS
                pedLower = pedY - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
                pedUpper = pedY + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
                pedLeft = pedX - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
                pedRight = pedX + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];

                //propagate ST as well before making comparison
                currIndex += 1;
                numStepsTaken += 1;

                //get ST coordinates
                if (currIndex > pedSafeTraj.size()-1) {
                    currIndex = pedSafeTraj.size()-1;
                }
                currentSafeTrajPoint = pedSafeTraj[currIndex];
                STPointX = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
                STPointY = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];

                // point 2 point matching to check
                distToSafe = sqrt( pow(STPointX - pedX, 2) + pow(STPointY - pedY, 2) );

                // cout << "   car: " << carX << " " << carY << "   car v: " << carV  << " ped: " << pedX << " " << pedY << " ST: " << STPointX << " " << STPointY << endl;
                // cout << "   distToSafe2: " << distToSafe << endl;

                if (distToSafe > trajMaxDistToST){
                    trajMaxDistToST = distToSafe;
                }
                //after propagation
                // cout << "Step: " << numStepsTaken << " D to ST: " << distToSafe << endl;
                // cout << "car: " << carX << " " << carY << " upper: " << carUpper << " lower: " << carLower  << carY << " left: " << carLeft << " right: " << carRight << endl;
                // cout << "ped: " << pedX << " " << pedY << " upper: " << pedUpper << " lower: " << pedLower  << carY << " left: " << pedLeft << " right: " << pedRight << endl;
                // cout << "car: " << carX << " " << carY  << endl;
                // cout << "ped: " << pedX << " " << pedY << " ST: " << STPointX << " " << STPointY << endl;


                //if ped collide with car
                // if ((carLower <= pedUpper) && (pedLower <= carUpper) && (carLeft <= pedRight) && (pedLeft <= carRight) ) {  //pedLeft < carRight
                if (checkOverlap(pedRadius, pedX, pedY, carLeft, carLower, carRight, carUpper)) {
                    // cout << "collide" << endl;
                    collide = true;
                    break;
                } 
                if (carRight >= pedRight) {
                    collide = true;
                    break;
                }
            }

            // if (collide) {
            //     break;           
            // }
        // cout << "collide: MAX traj distToSafe: " << trajMaxDistToST << endl;
        // cout << "collide: distToSafe: " << distToSafe << endl;

        if (trajMaxDistToST < minToSafeForMax){
            minToSafeForMax = trajMaxDistToST;
        }

        }

        // cout << "minToSafeForMax: " << minToSafeForMax << endl;
        return minToSafeForMax;
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

    FloatType accelerationError = 0;
    FloatType velocityError = 0;

    VectorFloat carDimensions_;
    VectorFloat pedDimensions_;

    FloatType pedRadius;

    FloatType discountFactor;

    std::vector<std::pair<FloatType, FloatType>> allActionsVector;


    // Clamp a value
    FloatType clamp(const FloatType val, const FloatType bound1, const FloatType bound2) const{
        FloatType min = bound1;
        FloatType max = bound2;

        // Check for max and min bounds
        if(min >= max){
            // Reverse bounds
            min = bound2;
            max = bound1;
        }

        if(val < min){
            return min;
        } else if (val > max){
            return max;
        }

        // Does not exceed bounds
        return val;
    }

    // Function to return the minimum distance
    // between a line segment AB and a point E
    double minDistance(Point A, Point B, Point E) const 
    {
     
        // vector AB
        pair<double, double> AB;
        AB.F = B.F - A.F;
        AB.S = B.S - A.S;
     
        // vector BP
        pair<double, double> BE;
        BE.F = E.F - B.F;
        BE.S = E.S - B.S;
     
        // vector AP
        pair<double, double> AE;
        AE.F = E.F - A.F,
        AE.S = E.S - A.S;
     
        // Variables to store dot product
        double AB_BE, AB_AE;
     
        // Calculating the dot product
        AB_BE = (AB.F * BE.F + AB.S * BE.S);
        AB_AE = (AB.F * AE.F + AB.S * AE.S);
     
        // Minimum distance from
        // point E to the line segment
        double reqAns = 0;
     
        // Case 1
        if (AB_BE > 0) {
     
            // Finding the magnitude
            double y = E.S - B.S;
            double x = E.F - B.F;
            reqAns = sqrt(x * x + y * y);
        }
     
        // Case 2
        else if (AB_AE < 0) {
            double y = E.S - A.S;
            double x = E.F - A.F;
            reqAns = sqrt(x * x + y * y);
        }
     
        // Case 3
        else {
     
            // Finding the perpendicular distance
            double x1 = AB.F;
            double y1 = AB.S;
            double x2 = AE.F;
            double y2 = AE.S;
            double mod = sqrt(x1 * x1 + y1 * y1);
            reqAns = abs(x1 * y2 - y1 * x2) / mod;
        }
        return reqAns;
    }

    // Function to check if any point
    // overlaps the given circle
    // and rectangle
    bool checkOverlap(FloatType R, FloatType Xc, FloatType Yc,
                             FloatType X1, FloatType Y1,
                             FloatType X2, FloatType Y2) const
    {
     
        // Find the nearest point on the
        // rectangle to the center of
        // the circle
        FloatType Xn = max(X1, min(Xc, X2));
        FloatType Yn = max(Y1, min(Yc, Y2));
         
        // Find the distance between the
        // nearest point and the center
        // of the circle
        // Distance between 2 points,
        // (x1, y1) & (x2, y2) in
        // 2D Euclidean space is
        // ((x1-x2)**2 + (y1-y2)**2)**0.5
        FloatType Dx = Xn - Xc;
        FloatType Dy = Yn - Yc;
        return (Dx * Dx + Dy * Dy) <= R * R;
    }

};

OPPT_REGISTER_HEURISTIC_PLUGIN(KamikazeTrajGenHeuristicPlugin)

}

#endif
