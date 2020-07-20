
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    // cv::resizeWindow(windowName, 500, 500);
    cv::namedWindow(windowName, 0);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // ...
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // ...
}


// Computing Time to Collsion using Lidar data by evaluating Median Point of point cloud cluster of preceeding vehicle.
void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // auxiliary variables
    double dT = 1 / frameRate;        // time between two measurements in seconds

    // Picking median point to avoid outlier (of point being too close to egocar)
    double medianXPrev = 1e9, medianXCurr = 1e9;
    vector<double> xPrevVec, xCurrVec;
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        // minXPrev = (minXPrev > it->x) ? it->x : minXPrev;
        xPrevVec.push_back(it->x);
    }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        // minXCurr = (minXCurr > it->x)  ? it->x : minXCurr;
        xCurrVec.push_back(it->x);
    }
    sort(xPrevVec.begin(), xPrevVec.end());
    sort(xCurrVec.begin(), xCurrVec.end());

    // compute TTC from both measurements
    double medianDistIdxPrev = floor(xPrevVec.size() / 2.0);
    double medianDistPrev = 0;
    if (xPrevVec.size() % 2 == 0) 
        medianDistPrev = (xPrevVec[medianDistIdxPrev - 1] + xPrevVec[medianDistIdxPrev]) / 2.0;
    else
        medianDistPrev = xPrevVec[medianDistIdxPrev];

    double medianDistIdxCurr = floor(xCurrVec.size() / 2.0);
    double medianDistCurr = 0;
    if (xCurrVec.size() % 2 == 0) 
        medianDistCurr = (xCurrVec[medianDistIdxCurr - 1] + xCurrVec[medianDistIdxCurr]) / 2.0;
    else
        medianDistCurr = xCurrVec[medianDistIdxCurr];


    TTC = medianDistCurr * dT / (medianDistPrev - medianDistCurr);



    // // find closest distance to Lidar points within ego lane
    // double minXPrev = 1e9, minXCurr = 1e9;
    // for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    // {
    //     minXPrev = (minXPrev > it->x) ? it->x : minXPrev;
    // }

    // for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    // {
    //     minXCurr = (minXCurr > it->x)  ? it->x : minXCurr;
    // }

    // // compute TTC from both measurements
    // TTC = minXCurr * dT / (minXPrev - minXCurr);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // 2D vector tracking bounding box matches for each keypoint match - filled with 0s
    // row = prevFrame, col = currFrame
    vector<vector<int>> boundingBoxMatches(prevFrame.boundingBoxes.size(), vector<int>(currFrame.boundingBoxes.size(), 0));

    // iterate through each match
    for (auto &match : matches)
    {
        // queryIdx vs trainIdx: https://stackoverflow.com/questions/13318853/opencv-drawmatches-queryidx-and-trainidx/13320083#13320083
        cv::KeyPoint prevKeypoint = prevFrame.keypoints[match.queryIdx];
        cv::KeyPoint currKeypoint = currFrame.keypoints[match.trainIdx];

        // Consolidate bounding boxes IDs that prev keypoint is in
        vector<int> prevBB_matchesIds;
        for (auto &prevFrameBB : prevFrame.boundingBoxes)
        {

            // check whether point is within current bounding box
            if (prevFrameBB.roi.contains(prevKeypoint.pt))
            {
                prevBB_matchesIds.push_back(prevFrameBB.boxID);
            }
        }

        // Consolidate bounding boxes IDs that current keypoint is in
        vector<int> currBB_matchesIds;
        for (auto &currFrameBB : currFrame.boundingBoxes)
        {

            // check whether point is within current bounding box
            if (currFrameBB.roi.contains(currKeypoint.pt))
            {
                currBB_matchesIds.push_back(currFrameBB.boxID);
            }
        }


        // Update BB_table with keypoint matches
        for (int prevBB_id : prevBB_matchesIds)
        {
            for (int currBB_id : currBB_matchesIds)
            {
                boundingBoxMatches[prevBB_id][currBB_id]++;
            }
        }

    }

    // Select BB matches based on hightest number from each row...
    for (size_t i = 0; i < boundingBoxMatches.size(); i++)
    {
        
        int highestMatches = -1;
        int highestMatchesIdx = -1;
        for (size_t j = 0; j < boundingBoxMatches[i].size(); j++)
        {
            if (boundingBoxMatches[i][j] > highestMatches)
            {
                highestMatches = boundingBoxMatches[i][j];
                highestMatchesIdx = j;
            }
        
        // Insert pair of prev-to-curr Bounding Box ID match (highest)
        bbBestMatches[i] = highestMatchesIdx;
        }
        
    }
}

