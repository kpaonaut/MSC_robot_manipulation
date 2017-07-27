// Wenjie Chen, FANUC Corporation, 2016/05/13
// Mex file to call NXLIB functions to get point clouds data from Ensenso camera set
// Mex command: mex nxGetPtCld.cpp nxLib64.lib

#include "mex.h"
#include "nxLib.h"

void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]) {

    mwSignedIndex dims[3];

    /* check for proper number of arguments */
    if(nlhs!=2) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nlhs","Two outputs required.");
    }
    
	try {
		// Reference to the first two cameras in the node BySerialNo
		NxLibItem root;
		NxLibItem camera0 = root[itmCameras][itmBySerialNo][0];
		NxLibItem camera1 = root[itmCameras][itmBySerialNo][1];

		// Open both Ensenso cameras, here specifying explicitly which ones to open
		// Per default all available would be opened
		std::string camString;
		camString = "[\""+camera0[itmSerialNumber].asString()+"\",\""+camera1[itmSerialNumber].asString()+"\"]";
        
        // Capture an image from all open cameras
    	mexPrintf("\nCapturing the current scene...\n");
		NxLibCommand (cmdCapture).execute();

		// Compute disparity map (this also computes the rectfied images)
		NxLibCommand (cmdComputeDisparityMap).execute();
		// Compute XYZ data for each pixel
		NxLibCommand (cmdComputePointMap).execute();

		// Compute map explicitly using data from both cameras
    	mexPrintf("Rendering the point clouds for the current scene...\n");
		NxLibCommand compRenderPointMap(cmdRenderPointMap);
		compRenderPointMap.parameters()[itmCameras].setJson(camString,true);
		compRenderPointMap.parameters()[itmCamera] = camera1[itmSerialNumber].asString();
		compRenderPointMap.parameters()[itmNear] = 500;
		compRenderPointMap.parameters()[itmFar] = 3000;
		compRenderPointMap.execute();

		// Get info about the computed map and copy it into a std::vector
        /* create the output matrix */
        float * renderPointMap  = 0;
    	int pointMapSize = 0;
        int width, height, channels;
		root[itmImages][itmRenderPointMap].getBinaryDataInfo(&width, &height, &channels, 0,0,0);
        dims[0] = channels;  dims[1] = width;  dims[2] = height;
        plhs[0] = mxCreateNumericArray(3,dims,mxSINGLE_CLASS,mxREAL);
        renderPointMap = (float*)mxGetData(plhs[0]);   
        pointMapSize = channels*width*height*sizeof(renderPointMap[0]);
        root[itmImages][itmRenderPointMap].getBinaryData(renderPointMap, pointMapSize, 0, 0);

        uint8_T* renderPointMapTexture  = 0;
		root[itmImages][itmRenderPointMapTexture].getBinaryDataInfo(&width, &height, &channels, 0,0,0);
        dims[0] = channels;  dims[1] = width;  dims[2] = height;
        plhs[1] = mxCreateNumericArray(3,dims,mxUINT8_CLASS,mxREAL);
        renderPointMapTexture = (uint8_T*)mxGetData(plhs[1]);   
        pointMapSize = channels*width*height*sizeof(renderPointMapTexture[0]);
        root[itmImages][itmRenderPointMapTexture].getBinaryData(renderPointMapTexture, pointMapSize, 0, 0);
        
        mexPrintf("Point clouds computed\n\n");
	} catch (NxLibException ex) {
        mexErrMsgIdAndTxt("NxLibException:exception", ex.getErrorText().c_str());
	} catch (char const* e) {
        mexErrMsgIdAndTxt("NxLibException:exception", e);
	}	
}
