// Wenjie Chen, FANUC Corporation, 2016/05/13
// Mex file to call NXLIB functions to close Ensenso camera set
// Mex command: mex nxCloseCam.cpp nxLib64.lib

#include "mex.h"
#include "nxLib.h"

void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]) {

	try {
		// Reference to the first two cameras in the node BySerialNo
		NxLibItem root;
		NxLibItem camera0 = root[itmCameras][itmBySerialNo][0];
		NxLibItem camera1 = root[itmCameras][itmBySerialNo][1];

		// Open both Ensenso cameras, here specifying explicitly which ones to open
		// Per default all available would be opened
		std::string camString;
		camString = "[\""+camera0[itmSerialNumber].asString()+"\",\""+camera1[itmSerialNumber].asString()+"\"]";
        
		// Close both cameras
    	mexPrintf("\nClosing cameras...\n");
		NxLibCommand close(cmdClose);
		close.parameters()[itmCameras].setJson(camString,true);
		close.execute();

		// Close NxLib
       	mexPrintf("Closing NxLib...\n");
		nxLibCloseTcpPort();
		nxLibFinalize();
        
        mexPrintf("Ensenso cameras and NxLib closed\n\n");
	} catch (NxLibException ex) {
        mexErrMsgIdAndTxt("NxLibException:exception", ex.getErrorText().c_str());
	} catch (char const* e) {
        mexErrMsgIdAndTxt("NxLibException:exception", e);
	}	
}
