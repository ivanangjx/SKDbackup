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

    FloatType leastDistanceY;

    virtual FloatType getMaxConstraintUsingST(const HeuristicInfo* heuristicInfo) {
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

        //get stopping point -> lowest intersect before passing car
        long lastIndex;
        FloatType tempLeftMostX = std::numeric_limits<FloatType>::infinity();

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

            lastIndex = i;
        }

        cout << "lastIndex: " << lastIndex << endl;

        FloatType distCarPedX;

        FloatType leastDistance = std::numeric_limits<FloatType>::infinity();
        

        //DIDNT TAKE INTO ACCOUNT POSSIBILITY OF COLLISION SINCE IT IS SAFE TRAJECTORY

        for (int i=0; i<= lastIndex ; i++) {

            cout << "t= " << i << endl;

            currentSafeTrajPoint = pedSafeTraj[i];
            STPointX = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
            STPointY = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];

            //get car Right line
            Point A = make_pair(carRight, carUpper);
            Point B = make_pair(carRight, carLower);
            

            for (int j=i; j<= lastIndex ; j++) {

                cout << "   compare: " << j << endl;
                //get ST index
                TrajPoint tempCurrentSafeTrajPoint = pedSafeTraj[j];
                FloatType tempSTPointX = tempCurrentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
                FloatType tempSTPointY = tempCurrentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];
                //get ped point
                Point E = make_pair(tempSTPointX, tempSTPointY);

                //calc dist
                double distToCar = minDistance(A, B, E);
                distToCar = distToCar - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];

                if (distToCar < leastDistance) {
                    leastDistance = distToCar;
                    leastDistanceY = tempSTPointY;
                }


                
                cout << "       comparing: " << std::setw(10) << tempSTPointX << " " << std::setw(10) << tempSTPointY << " " << std::setw(10) << carX << " " << std::setw(10) << carY << endl;
                cout << "       distToCar: " << distToCar << endl;
            }

            //dist to know coast or brake?
            distCarPedX = abs(STPointX - carX);
            cout << "distCarPedX: " << distCarPedX << " baseStoppingDist_: " << baseStoppingDist_ << endl;
            // cout << "comparing: " << STPointX << " " << STPointY << " " << carX << " " << carY << endl;

            //car movement
            if (distCarPedX > baseStoppingDist_) { //more dist so not braking yet /a=0
                // cout << "coast" << endl;
                carX += (carV * stepTime_);
            } else {
                // cout << "brake" << endl;
                FloatType carTravelDist = ( (carV * stepTime_) + (0.5 * brakingDeceleration_ * stepTime_ * stepTime_));
                cout << "       carTravelDist: " << carTravelDist << endl;
                carX += carTravelDist;
                carV = carV + (brakingDeceleration_ * stepTime_);
            }
            cout << "       carV: " << carV << endl;
            carLower = carY - carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
            carUpper = carY + carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
            carRight = carX + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;

        }

        long numDecimals = 100;

        leastDistance = std::ceil(leastDistance * numDecimals) / numDecimals;

        cout << "MIN CONSTRAINT DIST: " << leastDistance << " ST Y: " << leastDistanceY << endl;

        return leastDistance;
    }


    // Function to return the minimum distance
    // between a line segment AB and a point E
    double minDistance(Point A, Point B, Point E)
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

    //this method takes the leftmost intersect ST and just uses the dist from car to ST
    // virtual FloatType getMaxConstraintUsingST(const HeuristicInfo* heuristicInfo) {
    //     // Constant for problem
    //     const FloatType MAX_SPEED = 2.5;
    //     // Retrieve information from the state
    //     VectorFloat stateVec = heuristicInfo->currentState->as<VectorState>()->asVector();
    //     // User data associated with the state
    //     auto nextStateUserData = static_cast<SKDUserData*>(heuristicInfo->currentState->getUserData().get());
    //     TrajData pedSafeTraj = nextStateUserData->safeTrajectory;

    //     VectorFloat carStartPos = generalOptions_->carStartPos;

    //     FloatType currentVisitIndex = 0;
    //     //initial Ped Position
    //     TrajPoint currentSafeTrajPoint = pedSafeTraj[currentVisitIndex];
    //     FloatType STPointX = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
    //     FloatType STPointY = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];
    //     // initial car position
    //     FloatType carX = carStartPos[0];
    //     FloatType carY = carStartPos[1];
    //     FloatType carV = stateVec[STATE_INFO::CAR_SPEED];

    //     //ped and car dimensions
    //     FloatType carLower = carY - carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
    //     FloatType carUpper = carY + carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;        
    //     FloatType carLeft = carX - carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;
    //     FloatType carRight = carX + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;

    //     FloatType pedLower = STPointY - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
    //     FloatType pedUpper = STPointY + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
    //     FloatType pedLeft = STPointX - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
    //     FloatType pedRight = STPointX + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];

    //     //get stopping point -> leftmost ped ST within the car height
    //     long bestIndex;
    //     FloatType tempLeftMostX = std::numeric_limits<FloatType>::infinity();

    //     for (int i=0; i<= pedSafeTraj.size() -1; i++) {

    //         TrajPoint tempCurrentSafeTrajPoint = pedSafeTraj[i];

    //         FloatType tempSTPointX = tempCurrentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
    //         FloatType tempSTPointY = tempCurrentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];

    //         FloatType tempPedLower = tempSTPointY - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
    //         FloatType tempPedUpper = tempSTPointY + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];

    //         // cout << tempSTPointX << " " << tempSTPointY << endl;

    //         if ( (carLower <= tempPedUpper) && (tempPedLower <= carUpper) ) {
    //             if (tempSTPointX < tempLeftMostX) {
    //                 tempLeftMostX = tempSTPointX;
    //                 bestIndex = i;
    //             }
    //         }
    //     }

    //     // cout << "bestIndex: " << bestIndex << endl;

    //     FloatType distCarPedX;

    //     //DIDNT TAKE INTO ACCOUNT POSSIBILITY OF COLLISION SINCE IT IS SAFE TRAJECTORY

    //     for (int i=0; i<= bestIndex ; i++) {

    //         //get ST index
    //         currentSafeTrajPoint = pedSafeTraj[i];
    //         STPointX = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
    //         STPointY = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];

    //         distCarPedX = abs(STPointX - carX);
    //         // cout << "comparing: " << STPointX << " " << STPointY << " " << carX << " " << carY << endl;

    //         //car movement
    //         if (distCarPedX > baseStoppingDist_) { //more dist so not braking yet /a=0
    //             // cout << "coast" << endl;
    //             carX += (carV * stepTime_);
    //         } else {
    //             // cout << "brake" << endl;
    //             carX += ( (carV * stepTime_) + (0.5 * brakingDeceleration_ * stepTime_ * stepTime_));
    //             carV = carV + (brakingDeceleration_ * stepTime_);
    //         }

    //     }
    //     //remove dimensions from distance
    //     distCarPedX = distCarPedX - carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2 - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];

    //     long numDecimals = 10;

    //     distCarPedX = std::ceil(distCarPedX * numDecimals) / numDecimals;

    //     cout << "MAX CONSTRAINT DIST: " << distCarPedX << " using ST index "<< bestIndex << endl;

    //     return distCarPedX;
    // }

    virtual FloatType getHeuristicValue(const HeuristicInfo* heuristicInfo) const override {   
        // Constant for problem
        const FloatType MAX_SPEED = 2.5;
        // Retrieve information from the state
        VectorFloat stateVec = heuristicInfo->currentState->as<VectorState>()->asVector();
        // User data associated with the state
        auto nextStateUserData = 
            static_cast<SKDUserData*>(heuristicInfo->currentState->getUserData().get());

        FloatType currentVisitIndex = nextStateUserData->visitIndex;
        // cout << currentVisitIndex << endl;

         // Retrieve safe trajectory and step time
        TrajData pedSafeTraj = nextStateUserData->safeTrajectory;
        TrajPoint currentSafeTrajPoint = pedSafeTraj[currentVisitIndex];

        FloatType STPointX = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
        FloatType STPointY = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];   

        // Further information from general options
        FloatType stepPenalty_ = generalOptions_->stepPenalty;

        FloatType constraint = heuristicInfo->constraint;

        // cout << endl; cout << "Constraint dist: " << constraint << " baseStoppingDist_: " << baseStoppingDist_ << endl;

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

        long numStepsTaken = 0;
        FloatType distToSafe = sqrt( pow(STPointX - pedX, 2) + pow(STPointY - pedY, 2) );

        bool violateConstraint = false;
        bool collide = false;
        // Y AXIS PED MOVEMENT
        while (true) {

            //break if constraint violation             
            if (distToSafe > constraint) {
                // cout << "constraint" << endl;
                violateConstraint = true;
                break;
            }
            //if ped collide with car
            if ((carLower <= pedUpper) && (pedLower <= carUpper) && (carLeft <= pedRight) && (pedLeft <= carRight) ) {
                // cout << "collide" << endl;
                collide = true;
                break;
            }  
            // // if ped reach center of car Y
            // if (pedY <= carY) { //this is wrong?-propagate next step and check
            //     // cout << "reach y center: " << numStepsTaken << endl;

            //     break;
            // }

            // Y ending based on Y axis of ST - calculation of min constraint dist
            if ( (pedY - (MAX_SPEED * stepTime_) ) < leastDistanceY) {
                // cout << "reach y center: " << numStepsTaken << endl;
                break;
            }


            FloatType distCarPedX = abs(pedX - carX);
            //ped movement
            pedY -= (MAX_SPEED * stepTime_);
            //car movement
            if (distCarPedX > baseStoppingDist_) { //more dist so not braking yet /a=0
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

            //get ST coordinates
            if (currentVisitIndex > pedSafeTraj.size()-1) {
                currentVisitIndex = pedSafeTraj.size()-1;
            }
            currentSafeTrajPoint = pedSafeTraj[currentVisitIndex];
            STPointX = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
            STPointY = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];

            distToSafe = sqrt( pow(STPointX - pedX, 2) + pow(STPointY - pedY, 2) );

            currentVisitIndex += 1;
            numStepsTaken += 1;


            //after propagation
            // cout << "Step: " << numStepsTaken << " D to ST: " << distToSafe << endl;
            // cout << "   car: " << carX << " " << carY << " upper: " << carUpper << " lower: " << carLower  << carY << " left: " << carLeft << " right: " << carRight << endl;
            // cout << "   ped: " << pedX << " " << pedY << " upper: " << pedUpper << " lower: " << pedLower  << carY << " left: " << pedLeft << " right: " << pedRight << endl;
            // cout << "car: " << carX << " " << carY  << endl;
            // cout << "ped: " << pedX << " " << pedY << " ST: " << STPointX << " " << STPointY << endl;
        }
        //==================================================================================================================
        //return stuff if collide/constraint
        // cout << "numStepsTaken: " << numStepsTaken << endl;
        if (violateConstraint) {
            FloatType constraintVal = - numStepsTaken * stepPenalty_;
            return constraintVal;
        }
        else if (collide) {
            FloatType collideVal = generalOptions_->goalReward - ( (numStepsTaken) * stepPenalty_);
            return collideVal;           
        }
        //==================================================================================================================
        // X AXIS PED MOVEMENT
        while (true) {

            //break if constraint violation 
            
            if (distToSafe > constraint) {
                // cout << "constraint" << endl;
                violateConstraint = true;
                break;
            }
            //if ped collide with car
            if ((carLower <= pedUpper) && (pedLower <= carUpper) && (carLeft <= pedRight) && (pedLeft <= carRight) ) {  //pedLeft < carRight
                // cout << "collide" << endl;
                collide = true;
                break;
            }  
          
            FloatType distCarPedX = abs(pedX - carX);
            //ped movement
            pedX -= (MAX_SPEED * stepTime_);
            //car movement
            if (distCarPedX > baseStoppingDist_) { //more dist so not braking yet /a=0
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

            //get ST coordinates
            if (currentVisitIndex > pedSafeTraj.size()-1) {
                currentVisitIndex = pedSafeTraj.size()-1;
            }
            currentSafeTrajPoint = pedSafeTraj[currentVisitIndex];
            STPointX = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
            STPointY = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];

            distToSafe = sqrt( pow(STPointX - pedX, 2) + pow(STPointY - pedY, 2) );

            currentVisitIndex += 1;
            numStepsTaken += 1;

            //after propagation
            // cout << "Step: " << numStepsTaken << " D to ST: " << distToSafe << endl;
            // cout << "car: " << carX << " " << carY << " upper: " << carUpper << " lower: " << carLower  << carY << " left: " << carLeft << " right: " << carRight << endl;
            // cout << "ped: " << pedX << " " << pedY << " upper: " << pedUpper << " lower: " << pedLower  << carY << " left: " << pedLeft << " right: " << pedRight << endl;
            // cout << "car: " << carX << " " << carY  << endl;
            // cout << "ped: " << pedX << " " << pedY << " ST: " << STPointX << " " << STPointY << endl;
        }

        // cout << "numStepsTaken: " << numStepsTaken << endl;
        if (violateConstraint) {
            FloatType constraintVal = - numStepsTaken * stepPenalty_;
            return constraintVal;
        }
        else if (collide) {
            FloatType collideVal = generalOptions_->goalReward - ( (numStepsTaken) * stepPenalty_);
            return collideVal;           
        }


        // for (int i = 0; i <= pedSafeTraj.size() -1 ; i++) {
        //     TrajPoint tempSafeTrajPoint = pedSafeTraj[i];
        //     // cout << tempSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT] << " " << tempSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ] << endl;

        //     FloatType pedUpper = tempSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ] + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
        //     FloatType pedLower = tempSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ] - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];

        //     if ((carLower <= pedUpper) && (pedLower <= carUpper)) {
        //         // cout << i << endl;

        //         if (tempSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ] >= stateVec[STATE_INFO::CAR_HORIZONTAL]) {    // more or equal than y axis center of car
        //             STIntersectPointY = tempSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];
        //             STIntersectPointX = tempSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
        //             STIntersectIndex = i;
        //         }
        //     }
        // }

        // // cout << "index: " << STIntersectIndex << " STIntersectPointX: " << STIntersectPointX << " STIntersectPointY: " << STIntersectPointY << endl;

        // FloatType distToIntersectPoint = sqrt( pow(STIntersectPointX - pedInitialX, 2) + pow(STIntersectPointY - pedInitialY, 2) );
        // // cout << distToIntersectPoint << endl;

        // FloatType numStepsToIntersectPoint = distToIntersectPoint/ (MAX_SPEED * stepTime_);

        // // cout << "numStepsToIntersectPoint : " << numStepsToIntersectPoint << endl;
        // // cout << round(numStepsToIntersectPoint) << endl;
        // //----------------------------------------------------

        // FloatType incrementX = abs(pedInitialX - STIntersectPointX) / round(numStepsToIntersectPoint);
        // FloatType incrementY = abs(pedInitialY - STIntersectPointY) / round(numStepsToIntersectPoint);

        // // cout << "increment: X: " << incrementX << " Y: " << incrementY << endl;

        // FloatType pedX = pedInitialX;
        // FloatType carX = stateVec[STATE_INFO::CAR_LONGIT];
        // FloatType distCarPed = abs(pedX - carX);
        // FloatType distCarTravel = 0;
        // FloatType carV = carInitialV;

        // for (int i = 0; i <= round(numStepsToIntersectPoint) ; i++) {

        //     if (pedInitialX < STIntersectPointX) {
        //         pedX = pedInitialX + (i * incrementX);
        //     } else {
        //         pedX = pedInitialX - (i * incrementX);
        //     }
      
        //     carX = stateVec[STATE_INFO::CAR_LONGIT] + distCarTravel;
        //     distCarPed = abs(pedX - carX);

        //     // cout << "carX: " << carX << " pedX: " << pedX << " distCarPed: " << distCarPed << " distCarTravel: " << distCarTravel << " carV: " << carV << endl;

        //     if (distCarPed > baseStoppingDist_) { //more dist so not braking yet /a=0
        //         distCarTravel += (carV * stepTime_);
        //     } else {
        //         distCarTravel += ( (carV * stepTime_) + (0.5 * brakingDeceleration_ * stepTime_ * stepTime_));
        //         carV = carV + (brakingDeceleration_ * stepTime_);
        //     }
        // }

        // FloatType carXPartOne = stateVec[STATE_INFO::CAR_LONGIT] + distCarTravel;

        // // cout << "carXPartOne : " << carXPartOne << endl;

        // //-----------------------------------
        // FloatType carRight = carXPartOne + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;
        // FloatType pedLeft = pedX - pedDimensions_[PED_DIMENSIONS::PED_RADIUS];

        // FloatType numStepsToCollideY = 0;

        // while (carRight < pedLeft) {

        //     // cout << "carRight : " << carRight << "pedLeft : " << pedLeft << " carV : " << carV  << endl;

        //     pedLeft = pedLeft - (MAX_SPEED * stepTime_); //assume collision is left

        //     FloatType carOneStepTravelY = 0;

        //     if (carV > 0) {
        //         if (distCarPed > baseStoppingDist_) { //more dist so not braking yet /a=0
        //             carOneStepTravelY = (carV * stepTime_);
        //         } else {
        //             carOneStepTravelY = ( (carV * stepTime_) + (0.5 * brakingDeceleration_ * stepTime_ * stepTime_));
        //             carV = carV + (brakingDeceleration_ * stepTime_);
        //         }
        //         carRight = carRight + carOneStepTravelY;               
        //     }



        //     numStepsToCollideY += 1;
        // }

        // // cout << "numStepsToCollideY: " << numStepsToCollideY << endl;

        // FloatType totalSteps = numStepsToIntersectPoint + numStepsToCollideY;

        // long totalStepsRounded = round(totalSteps);

        // // cout << "totalSteps: " << totalSteps << endl;

        // // cout << "totalStepsRounded: " << totalStepsRounded << endl;

        // FloatType heuristicVal = generalOptions_->goalReward - 
        //        ( totalSteps * stepPenalty_);

        // // cout << "heuristicVal: " << heuristicVal << endl;


        // // take into account constraint used ------------------------------

        // if (totalStepsRounded > pedSafeTraj.size()-1) {
        //     totalStepsRounded = pedSafeTraj.size()-1;
        // }

        // TrajPoint collideSafeTrajPoint = pedSafeTraj[totalStepsRounded];

        // FloatType STCollidePointY = collideSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];
        // FloatType STCollidePointX = collideSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];

        // FloatType KamiCollidePedX = pedLeft + pedDimensions_[PED_DIMENSIONS::PED_RADIUS];
        // FloatType KamiCollidePedY = STIntersectPointY;

        // // cout << "STCollidePointX: " << STCollidePointX << "STCollidePointY: " << STCollidePointY << endl;
        // // cout << "KamiCollidePedX: " << KamiCollidePedX << "KamiCollidePedY: " << KamiCollidePedY << endl;

        // FloatType distSafeToKamiCollide = sqrt( pow(STCollidePointX - KamiCollidePedX, 2) + pow(STCollidePointY - KamiCollidePedY, 2) );
        // // cout << "distSafeToKamiCollide: " << distSafeToKamiCollide << endl;

        // float constraint = heuristicInfo->constraint;

        // if (distSafeToKamiCollide > constraint) {
        //     return -1;
        // }

        // // cout << endl;


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


        // return heuristicVal;
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
