# SFND 2D Feature Tracking
## [Rubric](https://review.udacity.com/#!/rubrics/2549/view) Points
---


<img src="images/MatchingKeypointsBetweenTwoCameraFrames.png" />

#### 0. General Notes

* Main file submission is  *MidTermProject_Camera_Student.cpp*

* Custom Functions where implemented and inherited from the main file to generate the final performance results 
  check the *MidTermProject_Camera_Student_Keypoints_Counter.cpp*

* Project Folder Description

```   .
    ├── CMakeLists.txt
    ├── images
    │   ├── keypoints.png 
    │   └── KITTI
    │       └── 2011_09_26
    │           └── image_00
    │               └── data  --> *our images*
    │                   ├── 0000000000.png
    │                   ├──    ...    .png
    │                   └── 0000000009.png
    ├── LICENSE
    ├── performance_results --> *CSV results*
    │   ├── Detectors_Descriptors_Matching.csv
    │   ├── Detectors_Descriptors_Time.csv
    │   └── KeyPoints_In_Frames.csv
    ├── Technical_Write_Up.md 
    ├── README.md
    └── src
        ├── bkc.cpp
        ├── csvWriter.h --> *CSV Writer Class*
        ├── dataStructures.h
        ├── matching2D.hpp
        ├── matching2D_Student.cpp --> *Functions Definitions*
        ├── MidTermProject_Camera_Student.cpp
        └── MidTermProject_Camera_Student_Keypoints_Counter.cpp --> *Example of CSV Generator for Performance Evaluation*
```
* Known issus

    1- Non Free Algorithms in the Open CV contrib lib needed to be built and enabled
    
    2- SIFT detector + ORB descriptor will throw 'OutOfMemoryError' error, 
       Solution:
       SIFT descriptors are not extracted with ORB detectors, and AKAZE descriptors worked 
       only with AKAZE detectors. So avoid Combining ORB /SIFT detector/descriptor combination
       and other detectors with AKAZE descriptor when we implement the scripts.

#### 1. Task MP.1 Data Buffer Optimization

 This can be achieved by pushing in new elements on one end and removing elements on the other end. So I implemented double ended queue std::deque offers constant time O(1) insertion and deletion of objects at each end of the queue. Checking the current size of the deque is similarly constant time. New items are added with .push_back() and, once the specified ring buffer size is reached, old items are dropped with .pop_font().

*MidTermProject_Camera_Student.cpp*

```c++
DataFrame frame;
frame.cameraImg = imgGray;
dataBuffer.push_back(frame);
if (dataBuffer.size() > dataBufferSize)
    dataBuffer.pop_front();
assert(dataBuffer.size() <= dataBufferSize);

```
#### 2. Task MP.2 Keypoints Detection

Implement detectors HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT and make them selectable by setting a string accordingly.
*MidTermProject_Camera_Student.cpp*

```c++
string detectorType = "...";  // Uncomment to set detector selection
```

Functions can be found in *matching2D_Student.cpp*

```c++
// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)

// Detect keypoints in an image by adapting the Harris detector developed in a previous exercise
void detKeypointsHarris(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)

// Detect keypoints in an image using more recent methods available in OpenCV
void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)

```
#### 3. MP.3 Keypoint Removal

Remove all keypoints outside of a pre-defined rectangle and only use the keypoints within the rectangle for further processing.

*MidTermProject_Camera_Student.cpp*

```c++
// only keep keypoints on the preceding vehicle
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
```
#### 4. Task MP.4 Keypoints Descriptors
Implement descriptors BRIEF, ORB, FREAK, AKAZE and SIFT and make them selectable by setting a string accordingly.
*MidTermProject_Camera_Student.cpp*

```c++
cv::Mat descriptors;

// string descriptorType = "BRISK";
// string descriptorType = "BRIEF";
string descriptorType = "ORB";
// string descriptorType = "FREAK";
// string descriptorType = "AKAZE";  // Fails with all non-AKAZE detectors
// string descriptorType = "SIFT";  // Fails with ORB detector

descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
```

*matching2D_Student.cpp*
```c++

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // Select appropriate descriptor, using default values for now
    cv::Ptr<cv::DescriptorExtractor> extractor;
    
    if (descriptorType.compare("BRISK") == 0)
    {
        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("BRIEF") == 0)
    {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if (descriptorType.compare("ORB") == 0)
    {
        extractor = cv::ORB::create();
    }
    else if (descriptorType.compare("FREAK") == 0)
    {
        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if (descriptorType.compare("AKAZE") == 0)
    {
        extractor = cv::AKAZE::create();
    }
    else if (descriptorType.compare("SIFT") == 0)
    {
        extractor = cv::xfeatures2d::SIFT::create();
    }
    else
    {
        // Specified descriptorType is unsupported
        throw invalid_argument(descriptorType + " is not a valid descriptorType");
    }

    // perform feature description and log time
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}
```

#### 5. Task MP.5 Descriptor Matching 
Implement FLANN matching as well as k-nearest neighbor selection. Both methods must be selectable using the respective strings in the main function.

The function `matchDescriptors` in `matching2D_Student.cpp` contains sort of decision tree structure, based on the settings of these string parameters:
- `descriptorCategory` either: `DES_BINARY` (binary), `DES_HOG` (histogram of gradients)
- `matcherType` either: `MAT_FLANN` (cv::FlannBasedMatcher), `MAT_BF` (brute force)
- `selectorType` either: `SEL_NN` (nearest neighbors), `SEL_KNN` (k nearest neighbors)

To reduce the complexity and chance of mismatching the descriptor category, I've made `descriptorCategory` conditional on the `descriptorType`. In this exercise, SIFT is the only histogram of gradients (HoG) based descriptor evaluated.


*matching2D_Student.cpp*
```c++
/* For descriptor type, select binary (BINARY) or histogram of gradients (HOG) 
    BINARY descriptors include: BRISK, BRIEF, ORB, FREAK, and (A)KAZE. 
    HOG descriptors include: SIFT (and SURF and GLOH, all patented). */

string descriptorCategory{};

if (0 == descriptorType.compare("SIFT"))
{
    descriptorCategory = "DES_HOG";
}
else
{
    descriptorCategory = "DES_BINARY";
}

```
For the performance benchmarks (Tasks MP.7-9) below, `matcherType` was set to `MAT_BF` and `selectorType` was set to `SEL_KNN`, which implements match filtering based on the descriptor distance ratio.


#### 6. Task MP.6 Descriptor Distance Ratio
 
Use the K-Nearest-Neighbor matching to implement the descriptor distance ratio test, which looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints.

This distance ratio filter compares the distance (SSD) between two candidate matched keypoint descriptors. A threshold of `0.8` is applied and the stronger candidate (minimum distance) is selected as the correct match. This method eliminates many false-positive keypoint matches.

*matching2D_Student.cpp*

```c++
// Perform matching task with nearest neighbor matching (best match)
if (selectorType.compare("SEL_NN") == 0)
{ // nearest neighbor (best match)
    matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
}
// Perform k nearest neighbors
else if (selectorType.compare("SEL_KNN") == 0)
{ // k nearest neighbors (k=2)
    int k = 2;
    vector<vector<cv::DMatch>> knn_matches;
    matcher->knnMatch(descSource, descRef, knn_matches, k);
    
    // Filter matches using descriptor distance ratio test
    double minDescDistRatio = 0.8;
    for (auto it : knn_matches) {
        // The returned knn_matches vector contains some nested vectors with size < 2 !?
        if ( 2 == it.size() && (it[0].distance < minDescDistRatio * it[1].distance) ) {
            matches.push_back(it[0]);
        }
    }
}
```
#### 7. Task MP.7 Performance Evaluation 1

Count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.

I created a class to save the number of keypoints in a CSV file.

The number of keypoints within the bounding box of the preceding vehicle were counted for each detector type.
See the results in: [KeyPoints_In_Frames.csv](/performance_results/KeyPoints_In_Frames.csv)

*csvWriter.h*

```c++
std::string FileName = "/home/workspace/SFND_2D_Feature_Matching/Output.csv";

// A class to create and write data in a csv file.
class CSVWriter
{
	std::string fileName;
	std::string delimeter;
	int linesCount;
 
public:
	CSVWriter(std::string filename, std::string delm = ",") :
			fileName(filename), delimeter(delm), linesCount(0)
	{}
	/*
	 * Member function to store a range as comma separated value
	 */
	template<typename T>
	void addDatainRow(T first, T last);
};

/*
* This Function accepts a range and appends all the elements in the range
* to the last row, separated by delimiter (Default is comma)
*/
template<typename T>
void CSVWriter::addDatainRow(T first, T last)
{
    std::fstream file;
    // Open the file in truncate mode if first line else in Append Mode
    file.open(fileName, std::ios::out | (linesCount ? std::ios::app : std::ios::trunc));
    // Iterate over the range and add each lament to file separated by delimiter.
    for (; first != last; )
    {
        file << *first;
        if (++first != last)
            file << delimeter;
        }
    file << "\n";
    linesCount++;
    // Close the file
    file.close();
}
```
Also Example of the Auto Generator: [Keypoints_Counter.cpp](/src/MidTermProject_Camera_Student_Keypoints_Counter.cpp) 

Below results are shown for 10 test images.

|Detector type|frame0|frame1|frame2|frame3|frame4|frame5|frame6|frame7|frame8|frame9|
|-------------|------|------|------|------|------|------|------|------|------|------|
|SHITOMASI    |125   |118   |123   |120   |120   |113   |114   |123   |111   |112   |
|HARRIS       |17    |14    |19    |22    |26    |47    |18    |33    |27    |35    |
|FAST         |419   |427   |404   |423   |386   |414   |418   |406   |396   |401   |
|BRISK        |264   |282   |282   |277   |297   |279   |289   |272   |266   |254   |
|ORB          |92    |102   |106   |113   |109   |125   |130   |129   |127   |128   |
|AKAZE        |166   |157   |161   |155   |163   |164   |173   |175   |177   |179   |
|SIFT         |138   |132   |124   |137   |134   |140   |137   |148   |159   |137   |


SIFT and ORB detectors have small kepoints number. Other detectors (SHITOMASI, HARRIS, FATS, BRISK, AKAZE) have large kepoints number or separate keypoints.

Harris had the fewest relevant keypoints, while the top three performers in this metric were:
1. FAST (consistently produced ~400 per image)
2. BRISK (254-297 keypoints per image)
3. AKAZE (155-179 keypoints per image)
   
#### 8. Task MP.8 Performance evaluation 2

Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.

The number of matched keypoints were then counted for each valid detector type and descriptor type combination, 35 in total. Note that SIFT descriptors could not be extracted with ORB detectors, and AKAZE descriptors worked only with AKAZE detectors.

I created a another code with CSV writer and nested loops to test all possible combinations of detectors and descriptors and saved the results.

See the results in: [Detectors_Descriptors_Matching.csv](/performance_results/Detectors_Descriptors_Matching.csv)

The FAST detectors with BRIEF, SIFT, and ORB descriptors consistently produced the largest number of matched keypoints (~300 per image).


*MidTermProject_Camera_Student.cpp*

```c++
std::string FileName2 = "../performance_results/Detectors_Descriptors_Matching.csv";

std::vector<std::string> descriptorTypeList = {"BRISK",  "BRIEF", "ORB", "FREAK"};

int match_keypointss[detectorTypeList.size()][descriptorTypeList.size()][imgEndIndex-1];

for (size_t desIndex = 0; desIndex < descriptorTypeList.size(); desIndex++)
{  
    string  descriptorType  = descriptorTypeList[desIndex];
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    ...
}

// Creating an object of CSVWriter
CSVWriter writer2(FileName2);

// Creating a vector of strings
std::vector<std::string> header2 = { "Detector/Descriptor type", "1", "2", "3", "4", "5", "6", "7", "8"};
// Adding vector to CSV File
writer2.addDatainRow(header2.begin(), header2.end());
for (size_t detIndex = 0; detIndex < detectorTypeList.size(); detIndex++)
{ 
    for (size_t desIndex = 0; desIndex < descriptorTypeList.size(); desIndex++)
    { 
        std::vector<std::string> dataList2; 
        std::string det_des_str = detectorTypeList[detIndex] + "/" + descriptorTypeList[desIndex];                     
        dataList2.push_back(det_des_str);
           for (int i = 2; i < 10; i++)
               dataList2.push_back(std::to_string(match_keypointss[detIndex][desIndex][i]));
        // Wrote number of detector keypointss to csv file.
        writer2.addDatainRow(dataList2.begin(), dataList2.end());
    }
}   
```

Below results are shown for all possible combinations of detectors and descriptors over the test images.

|Detector/Descriptor type|1  |2  |3  |4  |5  |6  |7  |8  |
|------------------------|---|---|---|---|---|---|---|---|
|SHITOMASI/BRISK         |95 |82 |88 |80 |90 |82 |79 |85 |
|SHITOMASI/BRIEF         |86 |82 |111|104|101|102|102|100|
|SHITOMASI/ORB           |109|100|102|99 |102|103|97 |98 |
|SHITOMASI/FREAK         |104|97 |90 |86 |88 |86 |80 |81 |
|HARRIS/BRISK            |86 |85 |10 |14 |15 |16 |16 |15 |
|HARRIS/BRIEF            |23 |21 |11 |15 |20 |24 |26 |16 |
|HARRIS/ORB              |24 |23 |12 |15 |18 |24 |20 |15 |
|HARRIS/FREAK            |24 |22 |13 |15 |15 |17 |20 |12 |
|FAST/BRISK              |21 |18 |104|101|98 |85 |107|107|
|FAST/BRIEF              |100|100|130|118|126|108|123|131|
|FAST/ORB                |125|119|123|112|126|106|122|122|
|FAST/FREAK              |123|119|99 |91 |98 |85 |99 |102|
|BRISK/BRISK             |101|105|163|146|173|150|162|182|
|BRISK/BRIEF             |183|161|205|185|179|183|195|207|
|BRISK/ORB               |189|183|199|175|187|179|185|200|
|BRISK/FREAK             |203|178|162|147|170|148|172|173|
|ORB/BRISK               |178|166|46 |57 |61 |56 |68 |64 |
|ORB/BRIEF               |80 |73 |43 |45 |59 |53 |78 |68 |
|ORB/ORB                 |84 |66 |46 |58 |58 |52 |75 |67 |
|ORB/FREAK               |83 |74 |43 |52 |52 |58 |63 |58 |
|AKAZE/BRISK             |75 |69 |121|128|126|126|133|137|
|AKAZE/BRIEF             |144|143|134|131|130|134|146|150|
|AKAZE/ORB               |148|152|134|132|124|139|141|144|
|AKAZE/FREAK             |157|150|126|130|117|126|128|147|
|SIFT/BRISK              |146|133|62 |63 |63 |56 |60 |60 |
|SIFT/BRIEF              |66 |71 |78 |76 |85 |69 |74 |76 |
|SIFT/ORB                |70 |88 |76 |76 |77 |68 |66 |72 |
|SIFT/FREAK              |78 |87 |70 |64 |64 |61 |60 |63 |


#### 9. Task MP.9 Performance evaluation 3

Log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this data, the TOP3 detector / descriptor combinations must be recommended as the best choice for our purpose of detecting keypoints on vehicles.

Smiler to Task 7 and 8 created another code with the CSV writer class.
See the results in: [Detectors_Descriptors_Time.csv](/performance_results/Detectors_Descriptors_Time.csv)

*MidTermProject_Camera_Student.cpp*

```c++
std::string FileName3 = "/home/workspace/SFND_2D_Feature_Matching/Time.csv";

int time_match_keypointss[detectorTypeList.size()][descriptorTypeList.size()][imgEndIndex+1];

time_match_keypointss[detIndex][desIndex][imgIndex] = (t1 + t2) * 1000.0;

// Creating an object of CSVWriter
CSVWriter writer3(FileName3);

// Creating a vector of strings
std::vector<std::string> header3 = { "Detector/Descriptor type", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10"};
// Adding vector to CSV File
writer3.addDatainRow(header3.begin(), header3.end());
for (size_t detIndex = 0; detIndex < detectorTypeList.size(); detIndex++)
{ 
    for (size_t desIndex = 0; desIndex < descriptorTypeList.size(); desIndex++)
    { 
        std::vector<std::string> dataList3; 
        std::string det_des_str = detectorTypeList[detIndex] + "/" + descriptorTypeList[desIndex];                     
        dataList3.push_back(det_des_str);
        for (int i = 0; i < 10; i++)
            dataList3.push_back(std::to_string(time_match_keypointss[detIndex][desIndex][i]));
        // Wrote number of detector keypointss to csv file.
        writer3.addDatainRow(dataList3.begin(), dataList3.end());
    }
}   
```

below the result:

|Detector/Descriptor type|1  |2  |3  |4  |5  |6  |7  |8  |9  |10 |
|------------------------|---|---|---|---|---|---|---|---|---|---|
|SHITOMASI/BRISK         |22 |19 |18 |19 |19 |21 |17 |18 |18 |17 |
|SHITOMASI/BRIEF         |16 |16 |15 |16 |15 |15 |16 |16 |15 |15 |
|SHITOMASI/ORB           |16 |18 |17 |18 |18 |18 |19 |19 |18 |17 |
|SHITOMASI/FREAK         |58 |55 |52 |51 |51 |51 |51 |51 |50 |50 |
|HARRIS/BRISK            |13 |14 |14 |13 |14 |28 |13 |15 |15 |19 |
|HARRIS/BRIEF            |13 |14 |15 |14 |15 |29 |12 |15 |14 |19 |
|HARRIS/ORB              |13 |13 |13 |13 |14 |28 |13 |15 |14 |19 |
|HARRIS/FREAK            |51 |52 |52 |53 |52 |67 |51 |53 |52 |57 |
|FAST/BRISK              |2  |2  |2  |2  |2  |2  |2  |2  |2  |2  |
|FAST/BRIEF              |2  |1  |2  |1  |1  |2  |1  |1  |2  |1  |
|FAST/ORB                |2  |3  |1  |1  |3  |1  |1  |3  |1  |1  |
|FAST/FREAK              |41 |41 |41 |41 |40 |40 |42 |40 |40 |41 |
|BRISK/BRISK             |43 |43 |44 |43 |45 |44 |43 |42 |42 |42 |
|BRISK/BRIEF             |42 |45 |42 |41 |43 |42 |41 |40 |42 |40 |
|BRISK/ORB               |41 |41 |43 |43 |42 |42 |44 |42 |46 |41 |
|BRISK/FREAK             |83 |84 |83 |84 |83 |81 |81 |79 |80 |82 |
|ORB/BRISK               |9  |9  |8  |9  |8  |8  |8  |8  |8  |8  |
|ORB/BRIEF               |8  |7  |7  |7  |7  |7  |7  |7  |8  |7  |
|ORB/ORB                 |8  |8  |8  |8  |7  |8  |9  |8  |8  |9  |
|ORB/FREAK               |49 |47 |47 |47 |47 |49 |48 |47 |49 |47 |
|AKAZE/BRISK             |73 |71 |72 |67 |75 |69 |70 |73 |72 |67 |
|AKAZE/BRIEF             |71 |70 |69 |72 |70 |70 |67 |72 |70 |75 |
|AKAZE/ORB               |69 |85 |69 |69 |70 |66 |70 |67 |63 |70 |
|AKAZE/FREAK             |111|117|121|114|118|110|108|112|108|108|
|SIFT/BRISK              |130|132|135|136|138|134|137|138|136|139|
|SIFT/BRIEF              |136|139|135|129|135|130|130|134|132|134|
|SIFT/ORB                |136|136|135|134|129|132|138|133|135|139|
|SIFT/FREAK              |183|174|180|173|174|175|173|181|183|176|


The TOP3 detector / descriptor combinations as the best choice for our purpose of detecting keypoints on vehicles are:


All of these combinations consistently ran in less than `3 ms` total time. See the results in: _Task_MP8_MP9.csv_

However, processing time must be balanced with the number of keypoints successfully matched in that time. For overall performance relevant to this project, the top three combinations were:

- FAST detectors and ORB descriptors
- FAST detectors and BRIEF descriptors
- FAST detectors and SIFT descriptors


DETECTOR/DESCRIPTOR  | NUMBER OF KEYPOINTS | TIME
-------------------- | --------------------| --------
FAST+BRIEF           | 117 keypoints       | 1,4 ms
FAST+ORB             | 119 keypoints       | 1,7 ms
FAST+BRISK           | 80 keypoints        | 2.0 ms 


There are two disadvantages to SIFT: patent royalties and the worst-case execution time. For the combination, some SIFT runtime slipped to `6 ms` total time. The distribution of runtime for FAST with ORB and BRIEF descriptors was more tightly bound within `3 ms`.