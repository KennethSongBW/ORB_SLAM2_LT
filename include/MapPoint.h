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
    //20180929 add by song
    //create a type to save the status of the mappoint
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

    //20180929 add by song
    //get the memory status of this map point
    int getMemStatus()
    {
        if (memStatus == LONGTERM) return 0;
        else if (memStatus == SHORTTERM) return 1;
        else return 2;
    }

    //set the memory status of this map point
    void setMemStaus(int i)
    {
        if (i == 0) memStatus = LONGTERM;
        else if (i == 1) memStatus = SHORTTERM;
        else memStatus = REMOVED;
    }

    //get the count of short memory
    uint getCount() {return countOfShort;}

    //adding one to count of short
    void addCount();

    //reset the count of short
    void resetCount() {countOfShort = 0;}

    //countdown the count of short
    void countDown() {countOfShort = 7;}

    //minus one from count of short
    void minusCount();

    //determine whether two map points are the same
    bool isSame(MapPoint* pMP);

    //add a visible record of this point
    void addVisible(double mTimeStamp);

    //get isVisible
    vector<bool> getVisible() {return isVisible;}

    bool getPrediction() {return predictStatus;}
    void setPrediction(bool status) {predictStatus = status;}
    double getLastTime() {return lastTime;}
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

     //20180929 add by song
     //Saving the memory status of the map point
     memType memStatus;
     //Saving the count of short memory
     uint countOfShort;
     vector<bool> isVisible;
     double lastTime;
     bool predictStatus;
     bool isChanged;
     //end
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
