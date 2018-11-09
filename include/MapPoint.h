/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include<opencv2/core/core.hpp>
#include<mutex>
#include "BoostArchiver.h"

namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class Frame;


class MapPoint
{
    //20181107 song
    enum memType {LONGTERM = 0, SHORTTERM = 1, REMOVED = 2};
    //end
public:
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();

    cv::Mat GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF,size_t idx);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapPoint* pMP);    
    MapPoint* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, KeyFrame*pKF);
    int PredictScale(const float &currentDist, Frame* pF);

    //20181107 song
    //Set the status of the map point
    void setMemStatus(int i)
    {
        if (i == 0) status = LONGTERM;
        else if (i == 1) status = SHORTTERM;
        else status = REMOVED;
    }

    //Get the status of the map point
    int getMemStatus()
    {
        if (status == LONGTERM) return 0;
        else if (status == SHORTTERM) return 1;
        else return 2;
    }

    //reset the countMem to 0
    void resetCount() {countMem = 0;}

    //start the countdown of countMem
    void countDown()
    {
        countMem = 7;
        setMemStatus(1);
    }

    //get the countMem
    uint getCount() {return countMem;}

    //add 1 to the countMem and decide whether it will upgrade to long term
    void addCount()
    {
        if (status == SHORTTERM)
        {
            if (countMem >= 8 || countMem < 0) setMemStatus(2);
            else if (countMem ==7)
            {
                resetCount();
                setMemStatus(0);
            }
            else
            {
                countMem += 1;
                setMemStatus(1);
            }
        }
    }

    //minus from the countMem and decide whether the status will change
    void minusCount()
    {
        if (status == LONGTERM) countDown();
        else if (status == SHORTTERM && countMem!= 0) resetCount();
        else setMemStatus(2); 
    }

    //decide whether two map points are the same
    bool isSame(MapPoint* pMP);

    //add visible to isVisible
    void addVisible(bool s) {isVisible.push_back(s);}

    //predict current status
    bool predictStatus()
    {
        if (isVisible.size() < p) return false;
        double currentStatus = para_P[0];
        for(int i = 0; i < p - 1; i++) currentStatus += para_P[i] * isVisible[isVisible.size()-1-i];
        if (currentStatus >= 0.5) prediction = true;
        else prediction = false;
        //cout << "Predict Finished!" << endl;
        return prediction;
    }

    //get isVisible
    vector<bool> getVisible() {return isVisible;}

    //get lastTime
    double getLastTime() {return lastTime;}

    //set parameters
    void setPara(std::vector<float> para);

    int getP() {return p;}
    //end

public:
    // for serialization
    MapPoint();
private:
    // serialize is recommended to be private
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version);

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;


    static std::mutex mGlobalMutex;

    //20181109 song
    bool visibleStatus;
    //end

protected:    

     // Position in absolute coordinates
     cv::Mat mWorldPos;

     // Keyframes observing the point and associated index in keyframe
     std::map<KeyFrame*,size_t> mObservations;

     // Mean viewing direction
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     KeyFrame* mpRefKF;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     MapPoint* mpReplaced;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;

     //20181107 song
     //The status of this map point
     memType status;

     //The counting of this map point
     uint countMem;

     //Vector for saving visible times
     std::vector<bool> isVisible;

     //arma parameters
     int p;
     std::vector<float> para_P;

     //predict status
     bool prediction;

     //last time when this map point is visible
     double lastTime;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
