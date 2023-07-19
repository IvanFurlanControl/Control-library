
#define S_FUNCTION_NAME s_test_control_lib_sf_contr
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
    order_test = 4,
    nu_test = 2
};
t_state_feedback_controller test_state_feedback_controller;
        
t_lti_initialization_errors test_out = dimension_error;

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
    ssSetInputPortWidth(S, 0, 6); // no. of inputs
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 2); // no. of outputs
    
    
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
    {
        t_lti_initialization_errors init_test;
        t_system_var K_test[2][4];
        // K_test = [1 5 6 8;9 3 2 1]
        K_test[0][0] = 1;
        K_test[0][1] = 5;
        K_test[0][2] = 6;
        K_test[0][3] = 8;
        
        K_test[1][0] = 9;
        K_test[1][1] = 3;
        K_test[1][2] = 2;
        K_test[1][3] = 1;
        
        init_test = state_feedback_controller_create(order_test, nu_test, (t_system_var *)K_test, &test_state_feedback_controller);
        //init_test = transfer_function_system_create(order_test, num_test,  den_test, &(test_transfer_function.transfer_function_numerator_fir));
        test_out = init_test;
    }

}



/* Function: mdlOutputs =======================================================
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // definition of the variables
    real_T            *y    = ssGetOutputPortRealSignal(S,0);
    real_T            *x    = ssGetRealDiscStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
    
    t_system_var ref_k[2];
    t_system_var x_k[4];
    t_system_var u_k[2];
    
    ref_k[0] = (t_system_var)U(0);
    ref_k[1] = (t_system_var)U(1);
    
    x_k[0] = (t_system_var)U(2);
    x_k[1] = (t_system_var)U(3);
    x_k[2] = (t_system_var)U(4);
    x_k[3] = (t_system_var)U(5);
    
    state_feedback_controller_output_calc(ref_k, x_k, u_k, &test_state_feedback_controller);
    
    y[0] = (real_T)(u_k[0]);
    y[1] = (real_T)(u_k[1]);
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
