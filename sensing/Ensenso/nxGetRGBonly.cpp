// Wenjie Chen, FANUC Corporation, 2016/09/02
// Mex file to call NXLIB functions to get only RGB picture data from Ensenso camera set
// Mex command: mex nxGetRGBonly.cpp nxLib64.lib

#include "mex.h"
#include "nxLib.h"

void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]) {

    mwSignedIndex dims[3];

    /* check for proper number of arguments */
    if(nlhs!=1) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nlhs","One output required.");
    }
    
	try {
		// Reference to the first two cameras in the node BySerialNo
		NxLibItem root;
		NxLibItem camera1 = root[itmCameras][itmBySerialNo][1];

		// Open only color camera, here specifying explicitly which ones to open
		std::string camString;
		camString = "[\""+camera1[itmSerialNumber].asString()+"\"]";
        
        // Capture an image from all open cameras
    	mexPrintf("\nCapturing the current scene...\n");
		NxLibCommand (cmdCapture).execute();
        NxLibCommand (cmdRectifyImages).execute();
        
    	// Get info about the computed map and copy it into a std::vector
        /* create the output matrix */
        uint8_T * rgbData  = 0;
    	int rgbDataSize = 0;
        int width, height, channels;
		camera1[itmImages][itmRectified].getBinaryDataInfo(&width, &height, &channels, 0,0,0);
        dims[0] = channels;  dims[1] = width;  dims[2] = height;
        plhs[0] = mxCreateNumericArray(3,dims,mxUINT8_CLASS,mxREAL);
        rgbData = (uint8_T*)mxGetData(plhs[0]);   
        rgbDataSize = channels*width*height*sizeof(rgbData[0]);
        camera1[itmImages][itmRectified].getBinaryData(rgbData, rgbDataSize, 0, 0);
        
        mexPrintf("RGB picture captured.\n\n");
	} catch (NxLibException ex) {
        mexErrMsgIdAndTxt("NxLibException:exception", ex.getErrorText().c_str());
	} catch (char const* e) {
        mexErrMsgIdAndTxt("NxLibException:exception", e);
	}	
}
