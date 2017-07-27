// Te Tang, UC Berkeley, 2016/09/28
// Mex file to call Aruco functions to get Marker corner info
// Mex command: cv.mex('detectArucoCorner.cpp')
// Matlab command: [ids, Corners, imageWithCorner] = detectArucoCorner(image);
// Matlab might crash if no marker is detected in detectAruco.cpp. Fixed in detectArucoCorner.cpp.

#include <opencv2/matlab/bridge.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include "nxLib.h"

using namespace cv;
using namespace matlab;
using namespace bridge;
using namespace std;

static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}

void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray *prhs[]) {
    
    /* check for proper number of arguments */
    if(nrhs!=1) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nrhs","One output: image.");
    }
    if(nlhs!=3) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nlhs","Three outputs: ids,corners,image.");
    }
    
    // parse the inputs
    ArgumentParser parser("detectArucoCorner");
    parser.addVariant("detectArucoCorner", 1);
    MxArrayVector sorted = parser.parse(MxArrayVector(prhs, prhs+nrhs));
    
    // unpack the arguments
    BridgeVector inputs(sorted.begin(), sorted.end());
    Mat image = inputs[0].toMat();


    try {
        // Aruco Marker dictionary
        int dictionaryId = 0;
        // Show detected fake squares
        bool showRejected = false;
        // Show axis on marker
        bool estimatePose = false;
        float markerLength = 0.0315;
        
        Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
        detectorParams->doCornerRefinement = true; // do corner refinement in markers
        Ptr<aruco::Dictionary> dictionary =
                aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
        
        // read camera calibration matrix
        Mat camMatrix, distCoeffs;
        if (estimatePose) {
            bool readOk = readCameraParameters("calibration_Ensenso.xml", camMatrix, distCoeffs);
            if (!readOk) {
                cerr << "Invalid camera file" << endl;
            }
        }
        
        // Ensenso capture a rgb picture
//         NxLibItem root;
//         NxLibItem camera1 = root[itmCameras][itmBySerialNo][1];
//         NxLibCommand(cmdCapture).execute();
//         NxLibCommand(cmdRectifyImages).execute();
//         // save rgb data to image
//         Mat image;
//         camera1[itmImages][itmRectified].getBinaryData(image, 0);
//         mexPrintf("RGB picture captured.\n");
        
        // detect markers and estimate pose
        double tick = (double)getTickCount();
        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
        vector< Vec3d >   rvecs, tvecs; 
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
        if (estimatePose && ids.size() > 0)
            aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs, tvecs);
        // display tracking time
        double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
        mexPrintf("Detection Time = %f ms \n",currentTime * 1000);
        
        // draw results
        Mat imageCopy;
        image.copyTo(imageCopy);
        if (ids.size() > 0) {
            aruco::drawDetectedMarkers(imageCopy, corners, ids);
            if (estimatePose) {
                for (unsigned int i = 0; i < ids.size(); i++)
                    aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);
            }
        }
        if (showRejected && rejected.size() > 0)
            aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));
        
        // output results
        BridgeVector  outputs(3);
        if (ids.size() > 0) {
            
            outputs[0] = cv::Mat(ids);
            
            cv::Mat allCorners(0, corners[0].size(), cv::DataType<Point2f>::type);
            for (unsigned int i = 0; i < corners.size(); ++i){
                cv::Mat Sample(1, corners[0].size(), cv::DataType<Point2f>::type, corners[i].data());
                allCorners.push_back(Sample);
            } 
            outputs[1] = allCorners;           
            outputs[2] = imageCopy;
        }
        else{
            float notFound = -1;
            outputs[0] = cv::Mat(1, 1, CV_32F, notFound);
            outputs[1] = cv::Mat(1, 1, CV_32F, notFound);
            outputs[2] = imageCopy;
        }
        
        // push the outputs back to matlab
        for (size_t n = 0; n < static_cast<size_t>(std::max(nlhs,1)); ++n) {
            plhs[n] = outputs[n].toMxArray().releaseOwnership();
        }
        
    } catch(Exception& e) {
        error(e.what());
    } catch(...) {
        error("Uncaught exception occurred");
    }
}
