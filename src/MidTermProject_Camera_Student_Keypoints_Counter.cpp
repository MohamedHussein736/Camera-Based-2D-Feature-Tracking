/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>

#include <deque> // A double-ended queue

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

#include "csvWriter.h" // csv custom header

using namespace std;

/* This Code allows test to multiple configrations of detector --> descriptor --> matcher 
   By counting the number of keypoints on the preceding vehicle for all 10 images and take note
   of the distribution of their neighborhood size. for all the detectors implemented.
   loop in over each to test the different detectors and save the results in a CSV file */

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;      // no. of images which are held in memory (ring buffer) at the same time
    deque<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time

    bool bVis = false; // visualize results

    //****************************************** variables of csv ************************************************//
    // Location of CSV file
    std::string FileName = "../performance_results/KeyPoints_In_Frames.csv";
    // All detectors supported
    std::vector<std::string> detectorTypeList = {"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};

    // create variables to csv file
    int det_keypoints[detectorTypeList.size()][imgEndIndex + 1];

    // Creating an object of CSVWriter
    CSVWriter writer(FileName);

    // Creating a vector of strings Detector type and image number (csv header)
    std::vector<std::string> header = {"Detector type", "frame0","frame1", "frame2", "frame3", "frame4",
                                       "frame5", "frame6", "frame7", "frame8", "frame9"};

    // Adding vector to CSV File
    writer.addDatainRow(header.begin(), header.end());

    std::vector<std::string> dataList;
    //*************************************** End of variables of csv ************************************************//

    /* MAIN LOOP OVER ALL DETECTORS*/
    for (size_t detIndex = 0; detIndex < detectorTypeList.size(); detIndex++)
    {
        //  of the currunt detector form our list
        string detectorTypeAuto = detectorTypeList[detIndex];

        /* MAIN LOOP OVER ALL IMAGES */
        for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
        {
            /* LOAD IMAGE INTO BUFFER */

            // assemble filenames for current index
            ostringstream imgNumber;
            imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
            string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

            // load image from file and convert to grayscale
            cv::Mat img, imgGray;
            img = cv::imread(imgFullFilename);
            cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

            //// STUDENT ASSIGNMENT
            //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

            // push image into data frame buffer

            // The dataBuffer on line 40 above has been changed from a std::vector to std::deque
            // A double-ended queue (std::deque) has constant time O(1) insertion
            // and deletion on both ends of the collection. -Dylan

            DataFrame frame;
            frame.cameraImg = imgGray;
            dataBuffer.push_back(frame);
            if (dataBuffer.size() > dataBufferSize)
                dataBuffer.pop_front();
            assert(dataBuffer.size() <= dataBufferSize);

            //// EOF STUDENT ASSIGNMENT
            cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

            /* DETECT IMAGE KEYPOINTS */

            // extract 2D keypoints from current image
            vector<cv::KeyPoint> keypoints; // create empty feature list for current image
            // string detectorType = "SHITOMASI";
            string detectorType = detectorTypeAuto;

            //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
            //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

            // start detector timer
            // TASK MP.9
            double det_t = (double)cv::getTickCount();

            // Shi-Tomasi
            if (detectorType.compare("SHITOMASI") == 0)
            {
                detKeypointsShiTomasi(keypoints, imgGray, false);
            }

            // Harris
            else if (detectorType.compare("HARRIS") == 0)
            {
                detKeypointsHarris(keypoints, imgGray, false);
            }

            // Modern detector types, including FAST, BRISK, ORB, AKAZE, and SIFT
            else if (detectorType.compare("FAST") == 0 ||
                     detectorType.compare("BRISK") == 0 ||
                     detectorType.compare("ORB") == 0 ||
                     detectorType.compare("AKAZE") == 0 ||
                     detectorType.compare("SIFT") == 0)
            {
                detKeypointsModern(keypoints, imgGray, detectorType, false);
            }

            // Specified detectorType is unsupported
            else
            {
                throw invalid_argument(detectorType + " is not a valid detectorType");
            }

            // end detector timer
            det_t = ((double)cv::getTickCount() - det_t) / cv::getTickFrequency();

            // Filter with a hardcoded bounding box to keep only keypoints on the preceding vehicle
            bool bFocusOnVehicle = true;
            cv::Rect vehicleRect(535, 180, 180, 150);
            if (bFocusOnVehicle)
            {
                vector<cv::KeyPoint> filteredKeypoints;
                for (auto kp : keypoints)
                {
                    if (vehicleRect.contains(kp.pt))
                        filteredKeypoints.push_back(kp);
                }
                keypoints = filteredKeypoints;
            }

            // Task MP.7 Counting keypoints
            cout << detectorType << ",";
            cout << " Num of Kps " << keypoints.size() << ",";
            if (keypoints.size() > 0)
            {
                auto kp = keypoints[0];
                cout << kp.size << endl;
            }

            // Record keypoints
            det_keypoints[detIndex][imgIndex] = keypoints.size();

            // optional : limit number of keypoints (helpful for debugging and learning)
            bool bLimitKpts = false;
            if (bLimitKpts)
            {
                int maxKeypoints = 50;

                if (detectorType.compare("SHITOMASI") == 0)
                { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                    keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                }
                cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                cout << " NOTE: Keypoints have been limited!" << endl;
            }

            // push keypoints and descriptor for current frame to end of data buffer
            (dataBuffer.end() - 1)->keypoints = keypoints;
            cout << "#2 : DETECT KEYPOINTS done" << endl;

        } // eof loop over all images

    } // eof loop over detectors

    // // Wrote number of detector keypoints to csv file.
    for (size_t detIndex = 0; detIndex < detectorTypeList.size(); detIndex++)
    {
        std::vector<std::string> dataList;
        dataList.push_back(detectorTypeList[detIndex]);
        for (int i = 0; i < 10; i++)
            dataList.push_back(std::to_string(det_keypoints[detIndex][i]));
        // Wrote number of detector keypointss to csv file.
        writer.addDatainRow(dataList.begin(), dataList.end());
    }

    cout << "#3 : SAVE RESULTS INTO CSV " << FileName << " done" << endl;

    return 0;
}
