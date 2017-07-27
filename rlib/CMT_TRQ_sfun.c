/*  File    : CMT_TRQ_sfun.c
 *  Abstract:
 *
 *      C MEX S-function wrapper for computing motor torques.
 *  
 *  Usage (mex): mex CMT_TRQ_sfun.c CMT_TRQ.c
 *
 *	Copyright (c) 11/09/2014 by Yu Zhao. All Rights Reserved.
 *  Revised: 06/10/2016, by Wenjie Chen, for TLC implementation of S-Function and with model parameter update options
*/

#define S_FUNCTION_NAME CMT_TRQ_sfun
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"


/*========================================================================*
 *Number of S-function Parameters and marcos to access from the simStruct *
 *========================================================================*/
#define SAMPLE_TIME_IDX     0
#define SAMPLE_TIME_PARAM   ssGetSFcnParam(S,SAMPLE_TIME_IDX)


/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 1);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
	}

	if (!ssSetNumInputPorts(S, 4)) return;
    ssSetInputPortWidth(S, 0, 6);//pos
    ssSetInputPortWidth(S, 1, 6);//vel
    ssSetInputPortWidth(S, 2, 6);//acc_ref
    ssSetInputPortWidth(S, 3, 1);//model_no
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    ssSetInputPortDirectFeedThrough(S, 3, 1);

	if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 6);//torque

    ssSetNumSampleTimes(S, 1);

    // for model referencing, need TLC implementation with code reuse option
    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE | SS_OPTION_WORKS_WITH_CODE_REUSE));
}


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy that we inherit our sample time from the driving block.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    real_T  ts = *mxGetPr(SAMPLE_TIME_PARAM);

    ssSetSampleTime(S, 0, ts);
    ssSetOffsetTime(S, 0, 0.0);
}


// for multi-rate model, need to set support for multiple executions
#define MDL_SET_WORK_WIDTHS   /* Change to #undef to remove function */
#if defined(MDL_SET_WORK_WIDTHS) && defined(MATLAB_MEX_FILE)
/* Function: mdlSetWorkWidths ===============================================
 * Abstract:
 *      Set up multiple executions support
 */
static void mdlSetWorkWidths(SimStruct *S)
{
    ssSupportsMultipleExecInstances(S, true);
}
#endif /* MDL_SET_WORK_WIDTHS */


/* Function: mdlOutputs =======================================================
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
	//*********************************************************************

	real_T            *tau    = ssGetOutputPortRealSignal(S,0);
    InputRealPtrsType qptr    = ssGetInputPortRealSignalPtrs(S,0);
    InputRealPtrsType qdptr   = ssGetInputPortRealSignalPtrs(S,1);
    InputRealPtrsType qddptr  = ssGetInputPortRealSignalPtrs(S,2);
    InputPtrsType     model_no_ptr = ssGetInputPortRealSignalPtrs(S,3);

    double *q, *qd, *qdd;
    int model_no;
    
    q = qptr[0];
	qd = qdptr[0];
	qdd = qddptr[0];    
    model_no = *((InputInt8PtrsType)model_no_ptr);
    cmt_trq(q, qd, qdd, tau, model_no);
}


//================================================================


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S){}


#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
