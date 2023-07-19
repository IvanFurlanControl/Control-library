/** @file lti-systems.h
 * @brief Linear Time Invariant filtering lib
 * @author Ivan Furlan
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h> 
#include "type_defs_control_system_libs.h"
#include "lti_systems.h"
#include "lin_algebra.h"
#include <string.h>

/**
 * @brief Create the transfer-function "object"
 * @param order System order
 * @param num Transfer-function address pointing to the numerator coefficients
 * @param den Transfer-function address pointing to the denominator coefficients
 * @param system Address pointing to the transfer-function "object"
 * @return Returns the status of the initialization task
 */
t_lti_initialization_errors transfer_function_system_create(int order, t_system_var *num, t_system_var *den, t_system_transfer_funtion *system)
{
    
    t_lti_initialization_errors ret;
    
    // inputs check
    if(order<1) {
        ret = order_zero_or_negative;
        return ret;
    }
        
    // initialization of the system order
    system->system_order = order;
    
    // definition of the memory space for the system variables
    system->in_k = (t_system_var *)malloc((system->system_order+1) * sizeof (t_system_var));
    system->out_k = (t_system_var *)malloc((system->system_order+1) * sizeof (t_system_var));
    
    // memory allocation check
    if (system->in_k==NULL || system->out_k==NULL) {
        ret = memory_space_cannot_be_allocated;
#ifdef DEBUG_MODE
        perror("Error: ");
#endif /*DEBUG_MODE*/
#ifdef CHECK_MEMORY_ALLOCATION_ENABLEED
        return ret;
#endif /*CHECK_MEMORY_ALLOCATION_ENABLEED*/
    }
    
    // initialization of the system variables to 0.0
    {
        int i;
        for(i=0; i<system->system_order+1; i++) {
            system->in_k[i] = 0.0;
            system->out_k[i] = 0.0;
        }
    }

    // definition of the memory space for the system parameters
    system->num = (t_system_var *)malloc((system->system_order+1) * sizeof (t_system_var));
    system->den = (t_system_var *)malloc((system->system_order+1) * sizeof (t_system_var));
    
    // memory allocation check
    if (system->num==NULL || system->den==NULL) {
        ret = memory_space_cannot_be_allocated;
#ifdef DEBUG_MODE
        perror("Error: ");
#endif /*DEBUG_MODE*/
#ifdef CHECK_MEMORY_ALLOCATION_ENABLEED
        return ret;
#endif /*CHECK_MEMORY_ALLOCATION_ENABLEED*/
    }
    
    // initialization of the system parameters
    {
        unsigned int i;
        for (i=0; i<(system->system_order+1); i++) {
            system->num[i] =  num[i];
            system->den[i] =  den[i];
        }
    }
	ret = init_successful;
	return ret;
}

/**
 * @brief Reset a transfer-function "object"
 * @param system Address pointing to the transfer-function "object"
 * @return void
 */
void transfer_function_system_reset(t_system_transfer_funtion *system)
{
    // initialization of the system variables to 0.0
    {
        int i;
        for(i=0; i<system->system_order+1; i++) {
            system->in_k[i] = 0.0;
            system->out_k[i] = 0.0;
        }
    }
}

/**
 * @brief Calculates one iteration of the transfer-function difference equation defined in system
 * @param in_k transfer-function input at the current samplig instant k
 * @param system Address pointing to the transfer-function "object"s
 * @return Returns transfer-fucntion output at the current samplig instant k
 */
#if !defined(USE_INLINE)
t_system_var transfer_function_linear_filter(t_system_var in_k, t_system_transfer_funtion *system)
{
    // only valid for normalized transfer-functions (i.e. coeff of the z^n term present in the den == 1)
    
    int i;
    
    // recursive equation
    system->in_k[0] = in_k;
    system->out_k[0] = system->in_k[0]*system->num[0];   
    for (i=0; i<system->system_order; i++) {
        system->out_k[0] = system->out_k[0]+system->in_k[i+1]*system->num[i+1]-system->out_k[i+1]*system->den[i+1];  
    }
    
    // recursive equation variable update
#if !defined(USE_MEM_OPERATOTS)
    for (i=0; i<system->system_order; i++) {
        system->in_k[system->system_order-i] = system->in_k[system->system_order-i-1];
        system->out_k[system->system_order-i] = system->out_k[system->system_order-i-1];
    }
#else //!defined(USE_MEM_OPERATOTS)
	memmove(&system->in_k[1], &system->in_k[0], system->system_order * sizeof(&system->in_k[0]));
	memmove(&system->out_k[1], &system->out_k[0], system->system_order * sizeof(&system->out_k[0]));
#endif //defined(USE_MEM_OPERATOTS)
    
    return system->out_k[0];
}
#endif // #if !defined(USE_INLINE)

/**
 * @brief Destroy the transfer-function "object"
 * @param system Address pointing to the transfer-function "object"
 * @return void 
 */
void transfer_function_system_destroy(t_system_transfer_funtion *system) {
    
    // deallocation of the memory reserved for defing the state-space lti system
    free(system->in_k);
    free(system->out_k);
    free(system->num);
    free(system->den);
    
}


/**
 * @brief Create the state-space "object"
 * @param order System order
 * @param order n_u number of inputs
 * @param order n_y number of outputs
 * @param A address pointing to the matrix A of the state-space 
 * @param B address pointing to the matrix B of the state-space
 * @param C address pointing to the matrix C of the state-space
 * @param D address pointing to the matrix D of the state-space
 * @param system Address pointing to the state-space "object"
 * @return Returns the status of the initialization task
 */
t_lti_initialization_errors state_space_system_create(int order, int n_u, int n_y, t_system_var *A, t_system_var *B, t_system_var *C, t_system_var *D, t_system_state_space *system)
{
    
    t_lti_initialization_errors ret;
    
    // inputs check
    if(order<1) {
        ret = order_zero_or_negative;
        return ret;
    }
    
    if(n_u<1 || n_y<0) {
        ret = nu_or_ny_negative;
        return ret;
    }
    
    // initialization of the system order
    system->system_order = order;
    
    // definition of system number of input and output
    system->n_u = n_u;
    system->n_y = n_y;

    // definition of the memory space for the states-spave variables
    system->x = (t_system_var *)malloc((system->system_order) * sizeof (t_system_var));
    
    // definition of the memory space for the input variables
    system->u = (t_system_var *)malloc((system->system_order*system->n_u) * sizeof (t_system_var));
    
    // definition of the memory space for the output variables
    system->y = (t_system_var *)malloc((system->n_y*system->system_order) * sizeof (t_system_var));
   
    // memory allocation check
    if (system->x==NULL || system->u==NULL || system->y==NULL) {
        ret = memory_space_cannot_be_allocated;
#ifdef DEBUG_MODE
        perror("Error: ");
#endif /*DEBUG_MODE*/
#ifdef CHECK_MEMORY_ALLOCATION_ENABLEED
        return ret;
#endif /*CHECK_MEMORY_ALLOCATION_ENABLEED*/
    }
    
    // initialization of the state-space variables to 0.0
    {
        int i;
        for(i=0; i<system->system_order; i++) {
            system->x[i] = 0.0;
        }
    }
            
    // definition of the memory space for the state-space matrices
    system->A = (t_system_var *)malloc((system->system_order)*(system->system_order) * sizeof (t_system_var));
    system->B = (t_system_var *)malloc((system->system_order)*(system->n_u) * sizeof (t_system_var));
    system->C = (t_system_var *)malloc((system->n_y)*(system->system_order) * sizeof (t_system_var));
    system->D = (t_system_var *)malloc((system->n_y)*(system->n_u) * sizeof (t_system_var));
    
    // definition of temporary variables
    system->cx_ = (t_system_var *)malloc((system->n_y) * sizeof (t_system_var));
    system->du_ = (t_system_var *)malloc((system->n_y) * sizeof (t_system_var));
    system->ax_ = (t_system_var *)malloc((system->system_order) * sizeof (t_system_var));
    system->bu_ = (t_system_var *)malloc((system->system_order) * sizeof (t_system_var));
    
    // memory allocation check
    if (system->A==NULL || system->B==NULL || system->C==NULL || system->D==NULL ||
        system->cx_==NULL || system->du_==NULL || system->ax_==NULL || system->bu_==NULL
            ) {
        ret = memory_space_cannot_be_allocated;
#ifdef DEBUG_MODE
        perror("Error: ");
#endif /*DEBUG_MODE*/
#ifdef CHECK_MEMORY_ALLOCATION_ENABLEED
        return ret;
#endif /*CHECK_MEMORY_ALLOCATION_ENABLEED*/
    }
    
    {
        int i;
        for(i=0; i<system->n_y; i++) {
            system->cx_[i] = 0.0;
            system->du_[i] = 0.0;
        }
    }
    {
        int i;
        for(i=0; i<system->system_order; i++) {
            system->ax_[i] = 0.0;
            system->bu_[i] = 0.0;
        }
    }   
  
    // initialization of the state_space matrices
    {
        unsigned int i, j, k;
        
        // How the matrices are stored internally? An example: 
        // An 2x2 A matrix, for instance
        // A=[1 2;3 4] 
        // it's stored internally as a row vector 1x4: 
        // system->A = [1 2 3 4] 
        // This holds also for the matrices B, D and D.
        
        // matrix A: 
        for (i=0; i<system->system_order*system->system_order; i++) {
            system->A[i] = A[i];
        }
        
        // matrix B       
        for (i=0; i<system->system_order*system->n_u; i++) {
            system->B[i] = B[i];
        }
        
        // matrix C   
        for (i=0; i<system->n_y*system->system_order; i++) {
            system->C[i] = C[i];
        }
        
        // matrix D
        for (i=0; i<system->n_y*system->n_u; i++) {
            system->D[i] =D [i];
        }
   }
   ret = init_successful;
   return ret;
}

/**
 * @brief Reset a state-space "object"
 * @param system Address pointing to the state-space "object"
 * @return void
 */
void state_space_system_reset(t_system_state_space *system)
{
    // initialization of the state-space variables to 0.0
    {
        int i;
        for(i=0; i<system->system_order; i++) {
            system->x[i] = 0.0;
        }
    }
}

/**
 * @brief Calculates one iteration of the tstate-space difference equation defined in system
 * @param out_k pointer to the state-space output vector
 * @param in_k pointer to the state-space input vector
 * @param system Address pointing to the state-space "object"
 * @return void
 */
#if !defined(USE_INLINE) 
void state_space_linear_filter(t_system_var *in_k, t_system_var *out_k, t_system_state_space *system)
{
    // input initialization:
    system->u = in_k;
    
    // recursive state-space equation
    {
        // scope variables
        int i;
        
        // y = Cx+Du
        mat_vect_mult(system->cx_, system->C, system->x, system->n_y, system->system_order);
        mat_vect_mult(system->du_, system->D, system->u, system->n_y, system->n_u);
        vects_sum(system->y, system->cx_, system->du_, system->system_order);
        
        // x = Ax+Bu
        mat_vect_mult(system->ax_, system->A, system->x, system->system_order, system->system_order);
        mat_vect_mult(system->bu_, system->B, system->u, system->system_order, system->n_u);
        vects_sum(system->x, system->ax_, system->bu_, system->system_order);
       
        // output assignation
#if !defined(USE_MEM_OPERATOTS)
        for (i=0; i<system->n_y; i++) {
            out_k[i] = system->y[i];
        }
#else //!defined(USE_MEM_OPERATOTS)
		memcpy (out_k, &(system->y[0]), system->n_y * sizeof(system->y[0]));        
#endif //defined(USE_MEM_OPERATOTS)

    }
}
#endif // #if !defined(USE_INLINE)

/**
 * @brief Destroy the state-space "object"
 * @param system Address pointing to the state-space "object"
 * @return void 
 */
void state_space_linear_system_destroy(t_system_state_space *system) {

    // deallocation the memory reserved for difing the state-space lti system 
    free(system->x);
    free(system->u);
    free(system->y);
    free(system->A);
    free(system->B);
    free(system->C);
    free(system->D);
    free(system->cx_);
    free(system->du_);
    free(system->ax_);
    free(system->du_);
}
