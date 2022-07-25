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

        discountFactor = generalOptions_->discountFactor;    

        // Compute threshold distance at which the controller will decide to break
        FloatType stoppingTime = std::abs(fixedVelocity_ / brakingDeceleration_);

        // This is the base (multiplier == 1) desired stopping distance described in # Car_BEHAVIOUR.md
        baseStoppingDist_ = 
            // Stopping distance for car
            (fixedVelocity_ * stoppingTime) + (0.5 * brakingDeceleration_ * stoppingTime * stoppingTime) 
            // Added padding to avoid collision in the absence of control error
            + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2 + (stepTime_ * (fixedVelocity_ / 2));

        // baseStoppingDist_ = 100;

        // cout << "Heuristic: baseStoppingDist_: " << baseStoppingDist_ << endl;  


        return true;
    }

    FloatType leastDistanceY;
    long intersectIndex;

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

        //get FIRST INDEX point -> lowest intersect before passing car
        long firstIndex;
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

            firstIndex = i;
        }

        cout << "firstIndex: " << firstIndex << endl;

        //get stopping point -> lowest intersect before passing car
        long lastIndex;
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

            lastIndex = i;
        }

        cout << "lastIndex: " << lastIndex << endl;

        FloatType distCarPedX;

        FloatType leastDistance = std::numeric_limits<FloatType>::infinity();
        
        FloatType leastDistanceForIndex = std::numeric_limits<FloatType>::infinity();

        //DIDNT TAKE INTO ACCOUNT POSSIBILITY OF COLLISION SINCE IT IS SAFE TRAJECTORY

        for (int i=0; i<= pedSafeTraj.size() -1 ; i++) {

            cout << "t= " << i << endl;

            currentSafeTrajPoint = pedSafeTraj[i];
            STPointX = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
            STPointY = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];

            //get car Right line
            Point A = make_pair(carRight, carUpper);
            Point B = make_pair(carRight, carLower);

            //-------------------------------------------------------------------
            //realistic steps to reach car from origin
            TrajPoint originSafeTrajPoint = pedSafeTraj[0];
            FloatType originSTPointX = originSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
            FloatType originSTPointY = originSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];  
            
            Point origin = make_pair(originSTPointX, originSTPointY);
            double originToCar = minDistance(A, B, origin); 
            originToCar = originToCar - pedDimensions_[PED_DIMENSIONS::PED_RADIUS]; //should not minus this?
            cout << "   originToCar: " << originToCar << endl;
            FloatType pedDist = 0;
            int numsteps = 0;
            while (pedDist < originToCar) {
                pedDist += (MAX_SPEED * stepTime_);
                numsteps +=1;
            }
            cout << "   realistic steps: " << numsteps << endl;
            //-------------------------------------------------------------------           

            for (int j=0; j<= pedSafeTraj.size() -1 ; j++) {

                cout << "   compare: " << j << endl;
                //get ST index
                TrajPoint tempCurrentSafeTrajPoint = pedSafeTraj[j];
                FloatType tempSTPointX = tempCurrentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
                FloatType tempSTPointY = tempCurrentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];
                //get ped point
                Point E = make_pair(tempSTPointX, tempSTPointY);

                //calc dist
                double minDistToCar = minDistance(A, B, E);
                // DIST SHOULD BE ST POINT TO INTERSECT POINT FROM ST- NOT MINIMUM!!!
                FloatType distToCarTopRight = sqrt( pow(carRight - tempSTPointX, 2) + pow(carUpper - tempSTPointY, 2) );
                FloatType distToCarBottomRight = sqrt( pow(carRight - tempSTPointX, 2) + pow(carLower - tempSTPointY, 2) );
                FloatType distToCarMidRight = sqrt( pow(carRight - tempSTPointX, 2) + pow(carY - tempSTPointY, 2) );
                double distToCar = max(distToCarTopRight, distToCarBottomRight);
                    cout << "   minDistToCar: " << minDistToCar << "   distToCarTopRight: " << distToCarTopRight << "   distToCarBottomRight: " << distToCarBottomRight << "   distToCarMidRight: " << distToCarMidRight  << "   dist: " << distToCar << endl;

                // if ((i <= lastIndex) && (i >= firstIndex) && j >= numsteps) {
                if ((i <= lastIndex) && (i >= firstIndex) && j >= i) {
                // if (j >= i) {
                // if ( j >= i) {
                    // cout << "   CONSIDERING: " << j << "   minDistToCar: " << minDistToCar << "   distToCarTopRight: " << distToCarTopRight << "   distToCarBottomRight: " << distToCarBottomRight << "   distToCarMidRight: " << distToCarMidRight  << "   dist: " << distToCar << endl;
                    cout << "CONSIDERED" << endl;
                    if (minDistToCar < leastDistance) {
                        leastDistance = minDistToCar;
                        leastDistanceY = tempSTPointY;
                        // intersectIndex = i;
                    }
                }

                // smallest dist within index range to get index
                if ((i <= lastIndex) && (i >= firstIndex)) {
                    if (minDistToCar < leastDistanceForIndex) {
                        cout << "INDEXED" << endl;
                        leastDistanceForIndex = minDistToCar;
                        intersectIndex = i;
                    }
                }

                // if (distToCar < leastDistance) {
                //     leastDistance = distToCar;
                //     leastDistanceY = tempSTPointY;
                // }



                
                cout << "       comparing: " << std::setw(10) << tempSTPointX << " " << std::setw(10) << tempSTPointY << " car: " << std::setw(10) << carX << " " << std::setw(10) << carY << " " << std::setw(10) << carRight<< endl;
                cout << "       distToCar: " << distToCar << endl;
            }

            //dist to know coast or brake?
            distCarPedX = abs(STPointX - carX);
            // cout << "distCarPedX: " << distCarPedX << " baseStoppingDist_: " << baseStoppingDist_ << endl;
            // cout << "comparing: " << STPointX << " " << STPointY << " " << carX << " " << carY << endl;

            //car movement
            if (carV <= 0) {
                carX = carX;
                carV = 0;
            }
            else if (distCarPedX > baseStoppingDist_) { //more dist so not braking yet /a=0
                // cout << "coast" << endl;
                carX += (carV * stepTime_);
            } else {
                // cout << "brake" << endl;
                FloatType carTravelDist = ( (carV * stepTime_) + (0.5 * brakingDeceleration_ * stepTime_ * stepTime_));
                cout << "carTravelDist: " << carTravelDist << endl;
                carX += carTravelDist;
                carV = carV + (brakingDeceleration_ * stepTime_);
            }
            cout << "carV: " << carV << endl;
            carLower = carY - carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
            carUpper = carY + carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
            carRight = carX + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;

        }

        long numDecimals = 100;
        cout << "RAW VALUE CONSTRAINT DIST: " << leastDistance << endl;
        // leastDistance = leastDistance +0.35;
        leastDistance = std::ceil(leastDistance * numDecimals) / numDecimals;

        cout << "MIN CONSTRAINT DIST: " << leastDistance << " ST Y: " << leastDistanceY << "intersectIndex: " << intersectIndex << endl;

        cout << "firstIndex: " << firstIndex << endl;
        cout << "lastIndex: " << lastIndex << endl;
        cout << "intersectIndex: " << intersectIndex << endl;

        // return leastDistance;

        // minimum ped steps needed to achive this distance
        FloatType pedDist = 0;
        while (pedDist < leastDistance) {
            pedDist += (MAX_SPEED * stepTime_);
        }
        cout << "pedDist: " << pedDist << endl;        
        return pedDist;

    }

    virtual std::vector<float> getConstraintVectorUsingST(const HeuristicInfo* heuristicInfo) {

        std::vector<float> returnVector;

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
        long firstIndex;
        FloatType tempLeftMostX = std::numeric_limits<FloatType>::infinity();

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

            firstIndex = i;
        }

        cout << "firstIndex: " << firstIndex << endl;

        //get stopping point -> lowest intersect before passing car
        long lastIndex;
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

            lastIndex = i;
        }

        cout << "lastIndex: " << lastIndex << endl;

        intersectIndex = lastIndex;

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

            //-------------------------------------------------------------------
            //realistic steps to reach car from origin
            TrajPoint originSafeTrajPoint = pedSafeTraj[0];
            FloatType originSTPointX = originSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
            FloatType originSTPointY = originSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];  
            
            Point origin = make_pair(originSTPointX, originSTPointY);
            double originToCar = minDistance(A, B, origin); 
            originToCar = originToCar - pedDimensions_[PED_DIMENSIONS::PED_RADIUS]; //should not minus this?
            cout << "   originToCar: " << originToCar << endl;
            FloatType pedDist = 0;
            int numsteps = 0;
            while (pedDist < originToCar) {
                pedDist += (MAX_SPEED * stepTime_);
                numsteps +=1;
            }
            cout << "   realistic steps: " << numsteps << endl;


            //calculate and add == index
            //get ST index
            // TrajPoint tempCurrentSafeTrajPoint = pedSafeTraj[numsteps];
            // FloatType tempSTPointX = tempCurrentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
            // FloatType tempSTPointY = tempCurrentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];
            // //get ped point
            //  Point E = make_pair(tempSTPointX, tempSTPointY);
            // //calc dist
            // double distToCar = minDistance(A, B, E);
            // distToCar = distToCar - pedDimensions_[PED_DIMENSIONS::PED_RADIUS]; //should not minus this?
            // cout << "   compare: " << numsteps << "   dist: " << distToCar << endl;
            // returnVector.push_back(distToCar*1.05);
            //---------------------------------------------------------------------

          

            for (int j=0; j<= pedSafeTraj.size() -1 ; j++) {

                cout << "   compare: " << j << endl;
                //get ST index
                TrajPoint tempCurrentSafeTrajPoint = pedSafeTraj[j];
                FloatType tempSTPointX = tempCurrentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
                FloatType tempSTPointY = tempCurrentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];
                //get ped point
                Point E = make_pair(tempSTPointX, tempSTPointY);

                //calc dist
                double distToCar = minDistance(A, B, E);
                distToCar = distToCar - pedDimensions_[PED_DIMENSIONS::PED_RADIUS]; //should not minus this?

                // if (distToCar < leastDistance) {
                //     leastDistance = distToCar;
                //     leastDistanceY = tempSTPointY;
                // }

                if ((i == lastIndex) && (j >= numsteps)) {   
                    cout << "CONSIDERING: " << j << "   dist: " << distToCar << endl;
                    returnVector.push_back(distToCar*1.03);
                }

                // if (i >= lastIndex) {   
                //     cout << "   compare: " << j << "   dist: " << distToCar << endl;
                //     returnVector.push_back(distToCar*1.1);



                    // // minimum ped steps needed to achive this distance
                    // FloatType pedDist = 0;

                    // while (pedDist < distToCar) {
                    //     pedDist += (MAX_SPEED * stepTime_);
                    // }
                    // cout << "pedDist: " << pedDist << endl;
                    // returnVector.push_back(pedDist*1.1); // 10% for car spped & acceleration error?

                // }


                
                cout << "       coors: " << std::setw(10) << tempSTPointX << " " << std::setw(10) << tempSTPointY << " car: " << std::setw(10) << carX << " " << std::setw(10) << carY << " " << std::setw(10) << carRight<< endl;
                cout << "       distToCar: " << distToCar << endl;
            }

            //dist to know coast or brake?
            distCarPedX = abs(STPointX - carX);
            // cout << "distCarPedX: " << distCarPedX << " baseStoppingDist_: " << baseStoppingDist_ << endl;
            // cout << "comparing: " << STPointX << " " << STPointY << " " << carX << " " << carY << endl;
            
            //car movement
            if (carV <= 0) {
                carX = carX;
                carV = 0;
            }
            else if (distCarPedX > baseStoppingDist_) { //more dist so not braking yet /a=0
                // cout << "coast" << endl;
                carX += (carV * stepTime_);
            } else {
                // cout << "brake" << endl;
                FloatType carTravelDist = ( (carV * stepTime_) + (0.5 * brakingDeceleration_ * stepTime_ * stepTime_));
                // cout << "       carTravelDist: " << carTravelDist << endl;
                carX += carTravelDist;
                carV = carV + (brakingDeceleration_ * stepTime_);
            }
            // cout << "       carV: " << carV << endl;
            carLower = carY - carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
            carUpper = carY + carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2;
            carRight = carX + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2;

        }

        long numDecimals = 100;

        leastDistance = std::ceil(leastDistance * numDecimals) / numDecimals;

        // cout << "MIN CONSTRAINT DIST: " << leastDistance << " ST Y: " << leastDistanceY << "lastIndex: " << lastIndex << endl;

        // for (int i=0; i< returnVector.size(); i++) {
        //  std::cout << returnVector.at(i) <<std::endl;
        // }

        cout << "firstIndex: " << firstIndex << endl;
        cout << "lastIndex: " << lastIndex << endl;
        cout << "intersectIndex: " << intersectIndex << endl;

        return returnVector;
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

    virtual FloatType getHeuristicValue(const HeuristicInfo* heuristicInfo) const override {   
        // Constant for problem
        const FloatType MAX_SPEED = 2.5;
        // Retrieve information from the state
        VectorFloat stateVec = heuristicInfo->currentState->as<VectorState>()->asVector();
        // User data associated with the state
        auto nextStateUserData = 
            static_cast<SKDUserData*>(heuristicInfo->currentState->getUserData().get());

        FloatType currentVisitIndex = nextStateUserData->visitIndex;
        // cout << "currentVisitIndex: " << currentVisitIndex << endl;

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

        float numStepsTaken = 0;
        FloatType distToSafe = 0;

        //figure out intersect index stuffs
        TrajPoint intersectSafeTrajPoint = pedSafeTraj[intersectIndex];
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
        for (int i =0; i < ceil(numStepsToIntersectPoint); i++) {

            // cout << "i"<< i << "   currentVisitIndex: " << currentVisitIndex << endl;

            //ped movement        
            // if (i == floor(numStepsToIntersectPoint)-1) { //last step ultra long
            if (i == floor(numStepsToIntersectPoint)) { //last step short one
                // cout << "   FINAL "  << endl;
                pedX = STIntersectPointX;
                pedY = STIntersectPointY;
            } else {
                 //propagate ped Y axis
                if (pedInitialY >= STIntersectPointY) {  //going down
                    pedY -= yStep;
                } else {
                    pedY += yStep;
                }

                //propagate ped X axis
                if (pedInitialX >= STIntersectPointX) {  //going left
                    pedX -= xStep;
                } else {
                    pedX += xStep;
                }               
            }

            FloatType distCarPedX = abs(pedX - carX);
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
            if ((carLower <= pedUpper) && (pedLower <= carUpper) && (carLeft <= pedRight) && (pedLeft <= carRight) ) {
                // cout << "collide" << endl;
                collide = true;
                break;
            }  
        }


        //==================================================================================================================
        //return stuff if collide/constraint
        // numStepsTaken = numStepsTaken / ceil(numStepsToIntersectPoint) * numStepsToIntersectPoint;
        // cout << "numStepsTaken: " << numStepsTaken << endl;
        // if (violateConstraint) {
        //     FloatType constraintVal = - numStepsTaken * stepPenalty_;
        //     return constraintVal;
        // }
        // else if (collide) {
        //     FloatType collideVal = generalOptions_->goalReward - ( (numStepsTaken) * stepPenalty_);
        //     return collideVal;           
        // }

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




        //==================================================================================================================
        // X AXIS PED MOVEMENT

        // cout << "PART 2 ------------------------------------------------------------" << endl;

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

            //get ST coordinates
            if (currentVisitIndex > pedSafeTraj.size()-1) {
                currentVisitIndex = pedSafeTraj.size()-1;
            }
            currentSafeTrajPoint = pedSafeTraj[currentVisitIndex];
            STPointX = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT];
            STPointY = currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ];

            // point 2 point matching to check
            distToSafe = sqrt( pow(STPointX - pedX, 2) + pow(STPointY - pedY, 2) );

            // //shortest dist to ST
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

            currentVisitIndex += 1;
            numStepsTaken += 1;

            // cout << "   car: " << carX << " " << carY << "   car v: " << carV  << " ped: " << pedX << " " << pedY << " ST: " << STPointX << " " << STPointY << endl;
            // cout << "   distToSafe2: " << distToSafe << endl;  

            //after propagation
            // cout << "Step: " << numStepsTaken << " D to ST: " << distToSafe << endl;
            // cout << "car: " << carX << " " << carY << " upper: " << carUpper << " lower: " << carLower  << carY << " left: " << carLeft << " right: " << carRight << endl;
            // cout << "ped: " << pedX << " " << pedY << " upper: " << pedUpper << " lower: " << pedLower  << carY << " left: " << pedLeft << " right: " << pedRight << endl;
            // cout << "car: " << carX << " " << carY  << endl;
            // cout << "ped: " << pedX << " " << pedY << " ST: " << STPointX << " " << STPointY << endl;
        }

        // cout << "numStepsTaken2: " << numStepsTaken << endl;
        // if (violateConstraint) {
        //     FloatType constraintVal = - numStepsTaken * stepPenalty_;
        //     return constraintVal;
        // }
        // else if (collide) {
        //     FloatType collideVal = generalOptions_->goalReward - ( (numStepsTaken) * stepPenalty_);
        //     return collideVal;           
        // }

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

    FloatType discountFactor;

};

OPPT_REGISTER_HEURISTIC_PLUGIN(KamikazeTrajGenHeuristicPlugin)

}

#endif
