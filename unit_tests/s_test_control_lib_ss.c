
#define S_FUNCTION_NAME s_test_control_lib_ss
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
    n_y = 4
};
t_system_state_space test_ss;
        
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
    ssSetInputPortWidth(S, 0, n_u); // no. of inputs
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, n_y); // no. of outputs
    
    
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
        t_system_var A[order][order];
        t_system_var B[order][n_u];
        t_system_var C[order][order];
        t_system_var D[order][n_u];
         
        // A = [1 2 3 4;5 8 7 2;1 9 4 2;6 5 2 6]
        A[0][0] = (t_system_var)1.0/100*5.4;
        A[0][1] = (t_system_var)2.0/100*5.4;
        A[0][2] = (t_system_var)3.0/100*5.4;
        A[0][3] = (t_system_var)4.0/100*5.4;
        
        A[1][0] = (t_system_var)5.0/100*5.4;
        A[1][1] = (t_system_var)8.0/100*5.4;
        A[1][2] = (t_system_var)7.0/100*5.4;
        A[1][3] = (t_system_var)2.0/100*5.4;

        A[2][0] = (t_system_var)1.0/100*5.4;
        A[2][1] = (t_system_var)9.0/100*5.4;
        A[2][2] = (t_system_var)4.0/100*5.4;
        A[2][3] = (t_system_var)2.0/100*5.4;

        A[3][0] = (t_system_var)6.0/100*5.4;
        A[3][1] = (t_system_var)5.0/100*5.4;
        A[3][2] = (t_system_var)2.0/100*5.4;
        A[3][3] = (t_system_var)6.0/100*5.4;
        
        // B = [3 2;3 2;2 2;9 8]
        B[0][0] = 3;
        B[0][1] = 2;
        
        B[1][0] = 3;
        B[1][1] = 2;
        
        B[2][0] = 2;
        B[2][1] = 2;
        
        B[3][0] = 9;
        B[3][1] = 8;
        
        // C = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]
        C[0][0] = 1;
        C[0][1] = 0;
        C[0][2] = 0;
        C[0][3] = 0;
        
        C[1][0] = 0;
        C[1][1] = 1;
        C[1][2] = 0;
        C[1][3] = 0;

        C[2][0] = 0;
        C[2][1] = 0;
        C[2][2] = 1;
        C[2][3] = 0;

        C[3][0] = 0;
        C[3][1] = 0;
        C[3][2] = 0;
        C[3][3] = 1;
        
        // D = [0 1;1 0;1 0;1 0]
        D[0][0] = 0;
        D[0][1] = 1;
        
        D[1][0] = 1;
        D[1][1] = 0;
        
        D[2][0] = 1;
        D[2][1] = 0;
        
        D[3][0] = 1;
        D[3][1] = 0;
        
        init_test = state_space_system_create(order, n_u, n_y, (t_system_var*)A, (t_system_var*)B, (t_system_var*)C, (t_system_var*)D, &test_ss);
        
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
    
    t_system_var y_k[4];
    t_system_var u_k[2];
   
    {
        unsigned int i;
        for(i=0; i<n_u; i++) {
            u_k[i] = (t_system_var)U(i);
        }
    }
    
    state_space_linear_filter(u_k, y_k, &test_ss);
    {
        unsigned int i;
        for(i=0; i<n_y; i++) {
            y[i] = y_k[i];
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
