// Wenjie Chen, FANUC Corporation, 2016/05/13
// Mex file to call NXLIB functions to open Ensenso camera set
// Mex command: mex nxOpenCam.cpp nxLib64.lib

#include "mex.h"
#include "nxLib.h"
#include <fstream>
using namespace std;

void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]) {

	try {
    	mexPrintf("\nOpening NxLib and waiting for cameras to be detected...\n");
        
		// Initialize NxLib and enumerate cameras
		nxLibInitialize(true);
		// Open TCP port to inspect tree with NxTreeEdit
		nxLibOpenTcpPort(24001);

		// Reference to cameras node
		// Uses reference by path here
		NxLibItem Cams = NxLibItem("/Cameras/BySerialNo");

		if (Cams.count() < 2) throw "There must be at least 2 cameras";

		// Reference to the first two cameras in the node BySerialNo
		NxLibItem root;
		NxLibItem camera0 = root[itmCameras][itmBySerialNo][0];
		NxLibItem camera1 = root[itmCameras][itmBySerialNo][1];

		// Open both Ensenso cameras, here specifying explicitly which ones to open
		// Per default all available would be opened
		std::string camString;
		camString = "[\""+camera0[itmSerialNumber].asString()+"\",\""+camera1[itmSerialNumber].asString()+"\"]";
        
        NxLibCommand open(cmdOpen);
		open.parameters()[itmCameras].setJson(camString,true);
		open.execute();

        // Load parameter file
        {std::ifstream file("Settings_stereo.json");
         if (file.is_open() && file.rdbuf()) {
             std::stringstream buffer;
             buffer << file.rdbuf();
             std::string const& fileContent = buffer.str();
             camera0[itmParameters].setJson(fileContent, true);
         }
        }
        {std::ifstream file("Settings_mono.json");
         if (file.is_open() && file.rdbuf()) {
             std::stringstream buffer;
             buffer << file.rdbuf();
             std::string const& fileContent = buffer.str();
             camera1[itmParameters].setJson(fileContent, true);
         }
        }
//         camera0[itmParameters][itmCapture][itmAutoExposure] = false;
//         camera0[itmParameters][itmCapture][itmExposure] = 3.08; //5; // for N20: 5; for N35: 3.08
// //         camera0[itmParameters][itmCapture][itmTargetBrightness] = 80;
//         camera1[itmParameters][itmCapture][itmAutoExposure] = false;
//         camera1[itmParameters][itmCapture][itmExposure] = 3.69; //100; // for old 8mm lens: 100; for new 8mm 1/8 lens: 3.69
// //         camera1[itmParameters][itmCapture][itmTargetBrightness] = 128;

    	mexPrintf("Ensenso cameras opened and ready for capture\n\n");

	} catch (NxLibException ex) {
        mexErrMsgIdAndTxt("NxLibException:exception", ex.getErrorText().c_str());
	} catch (char const* e) {
        mexErrMsgIdAndTxt("NxLibException:exception", e);
	}	
}
