
#define S_FUNCTION_NAME s_test_control_lib_zero_eye
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

// Definition of the functions
// delay
enum {
    order = 4,
    n_u = 2,
    n_y = 2
};
t_state_observer test_observer;

t_system_var eye[order][order];
t_system_var zero[order][n_y+n_u];
        
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
    ssSetOutputPortWidth(S, 0, 1); // no. of outputs
    
    
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

    // Cobs = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]
    eye_matrix_filling((t_system_var*)eye, order);
        
    // Dobs = [0 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0]
    zero_matrix_filling((t_system_var*)zero, order, n_y+n_u);
                
}



/* Function: mdlOutputs =======================================================
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // definition of the variables
    real_T            *y    = ssGetOutputPortRealSignal(S,0);
    real_T            *x    = ssGetRealDiscStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
    int err = 0;
    
    {
        int i, j;
        for(i=0; i<order; i++) {
            for(j=0; j<order; j++) {
                if(i == j) {
                    if(eye[i][j] != (t_system_var)1) {
                        err = 1;
                    }
                } else {
                    if(eye[i][j] != (t_system_var)0) {
                        err = 1;
                    }
                }
            }
        }
    }
    
    {
        int i, j;
        for(i=0; i<order; i++) {
            for(j=0; j<n_u+n_y; j++) {
                if(zero[i][j] != (t_system_var)0) {
                    err = 1;
                }
            }
        }
    }
    y[0] = err;
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
