
#define S_FUNCTION_NAME s_test_control_lib_mat_vect_mult
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
//#include "C:\projects\nw-libraries\lib-lti-systems\src\lti_systems.h"
#include "C:\projects\nw-libraries\lib-lti-systems\src\control_systems.h"

#define U(element) (*uPtrs[element])  /* Pointer to Input Port0 */

#define TS_IDX 0
#define TS_PARAM(S) ssGetSFcnParam(S,TS_IDX)

#define NPARAMS 1

#define NUM_CONT_STATES 0
#define NUM_DISC_STATES 0

#define MAT_H 2
#define MAT_L 3
        
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
    ssSetNumSFcnParams(S, NPARAMS);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    ssSetNumContStates(S, NUM_CONT_STATES);
    ssSetNumDiscStates(S, NUM_DISC_STATES); 
   
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 1); // no. of inputs
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, MAT_H); // no. of outputs
    
    
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    
    /* specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specify the sample time as 1.0
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, *mxGetPr(TS_PARAM(S)));
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S); 
}


#define MDL_INITIALIZE_CONDITIONS
/* Function: mdlInitializeConditions ========================================
 * Abstract:
 *    Initialize both discrete states to one.
 */
static void mdlInitializeConditions(SimStruct *S)
{
                
}



/* Function: mdlOutputs =======================================================
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // definition of the variables
    real_T            *y    = ssGetOutputPortRealSignal(S,0);
    real_T            *x    = ssGetRealDiscStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
    t_system_var test_mat[MAT_H][MAT_L] = {{1,4,3},{2,8,9}};
    t_system_var test_vect[MAT_L] = {2,8,7};
    t_system_var test_res[MAT_H];

    mat_vect_mult(test_res, test_mat, test_vect, MAT_H, MAT_L);
    
    {
        unsigned int i;
        
        for(i=0; i<MAT_H; i++) {
            y[i] = test_res[i];
        }
    }
}



#define MDL_UPDATE
/* Function: mdlUpdate ======================================================
 */
static void mdlUpdate(SimStruct *S, int_T tid)
{

}



/* Function: mdlTerminate =====================================================
 */
static void mdlTerminate(SimStruct *S)
{
    UNUSED_ARG(S); /* unused input argument */
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
