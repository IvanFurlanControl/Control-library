/** @file lti-systems.h
 * @brief Linear Time Invariant filtering lib
 * @author Ivan Furlan
 */
        
#ifndef _LTI_SYSTEMS_H_
#define _LTI_SYSTEMS_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdbool.h> 
#include "type_defs_control_system_libs.h"
#include "lin_algebra.h"
#include <string.h>

/**
 * \struct t_lti_initialization_errors
 * \brief possible initilization results
 */
typedef enum {
    
	order_zero_or_negative = 0,
    nu_or_ny_negative,
    memory_space_cannot_be_allocated,
	init_successful,
	dimension_error
            
} t_lti_initialization_errors;

/**
 * \struct t_system_transfer_funtion
 * \brief transfer-function object structure
 */
typedef struct system_transfer_funtion{
     int system_order; /**< order of the system */
     t_system_var *num; /**< transfer-function numerator: if system_order == n --> [a_n a_{n-1} ... a_0] */
     t_system_var *den; /**< transfer-function denominator:  if system_order == n --> [1 b_{n-1} ... b_0] */
     t_system_var *in_k; /**< input variables: if system_order == n --> [in_k in_{k-1} ... in_{k-n}] */
     t_system_var *out_k; /**< output variables: if system_order == n --> [out_k out_{k-1} ... out_{k-n}] */
} t_system_transfer_funtion;
    
/**
 * \struct t_system_state_space
 * \brief state-space object structure
 */
typedef struct system_state_space {
     int system_order; /**< order of the system */
     int n_u; /**< number of inputs  */
     int n_y; /**< number of outputs */
     t_system_var *u; /**< pointer to the state-space input variables */
     t_system_var *y; /**< pointer to the state-space output variables */
     t_system_var *A; /**< pointer to the state-space matrix A */
     t_system_var *B; /**< pointer to the state-space matrix B */
     t_system_var *C; /**< pointer to the state-space matrix C */
     t_system_var *D; /**< pointer to the state-space matrix D */
     t_system_var *x; /**< pointer to the state-space state vector */
     t_system_var *cx_; /**< pointer to the variables (internal use only) */
     t_system_var *du_; /**< pointer to the variables (internal use only) */
     t_system_var *ax_; /**< pointer to the variables (internal use only) */
     t_system_var *bu_; /**< pointer to the variables (internal use only) */
} t_system_state_space;

/**
 * @brief Create the transfer-function "object"
 * @param order System order
 * @param num Transfer-function address pointing to the numerator coefficients
 * @param den Transfer-function address pointing to the denominator coefficients
 * @param system Address pointing to the transfer-function "object"
 * @return Returns the status of the initialization task
 */
extern t_lti_initialization_errors transfer_function_system_create(int order, t_system_var *num, t_system_var *den, t_system_transfer_funtion *system);

/**
 * @brief Reset a transfer-function "object"
 * @param system Address pointing to the transfer-function "object"
 * @return void
 */
extern void transfer_function_system_reset(t_system_transfer_funtion *system);

/**
 * @brief Calculates one iteration of the transfer-function difference equation defined in system
 * @param in_k transfer-function input at the current samplig instant k
 * @param system Address pointing to the transfer-function "object"s
 * @return Returns transfer-fucntion output at the current samplig instant k
 */
#if defined(USE_INLINE)
inline t_system_var transfer_function_linear_filter(t_system_var in_k, t_system_transfer_funtion *system)
{
    // only valid for normalized transfer-functions (i.e. coeff of the z^n term present in the den == 1)
    
    int i;
    
    // recursive equation
    system->in_k[0] = in_k;
    system->out_k[0] = system->in_k[0]*system->num[0];   
    for (i=0; i<system->system_order; i++) {
        system->out_k[0] = system->out_k[0]+system->in_k[i+1]*system->num[i+1]-system->out_k[i+1]*system->den[i+1];  
    }
  
#if !defined(USE_MEM_OPERATOTS)
    for (i=0; i<system->system_order; i++) {
        system->in_k[system->system_order-i] = system->in_k[system->system_order-i-1];
        system->out_k[system->system_order-i] = system->out_k[system->system_order-i-1];
    }
#else  //!defined(USE_MEM_OPERATOTS)  
	memmove(&system->in_k[1], &system->in_k[0], system->system_order * sizeof(&system->in_k[0]));
	memmove(&system->out_k[1], &system->out_k[0], system->system_order * sizeof(&system->out_k[0]));
#endif  //defined(USE_MEM_OPERATOTS)  
	
    return system->out_k[0];
}
#else // #if defined(USE_INLINE)
extern t_system_var transfer_function_linear_filter(t_system_var in_k, t_system_transfer_funtion *system);
#endif // #if !defined(USE_INLINE

/**
 * @brief Destroy the transfer-function "object"
 * @param system Address pointing to the transfer-function "object"
 * @return void 
 */
extern void transfer_function_system_destroy(t_system_transfer_funtion *system);

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
extern t_lti_initialization_errors state_space_system_create(int order, int n_u, int n_y, t_system_var *A, t_system_var *B, t_system_var *C, t_system_var *D, t_system_state_space *system);

/**
 * @brief Reset a state-space "object"
 * @param system Address pointing to the state-space "object"
 * @return void
 */
extern void state_space_system_reset(t_system_state_space *system);

/**
 * @brief Calculates one iteration of the tstate-space difference equation defined in system
 * @param out_k pointer to the state-space output vector
 * @param in_k pointer to the state-space input vector
 * @param system Address pointing to the state-space "object"
 * @return void
 */
#if defined(USE_INLINE) 
inline void state_space_linear_filter(t_system_var *in_k, t_system_var *out_k, t_system_state_space *system)
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
#else // #if defined(USE_INLINE)
extern void state_space_linear_filter(t_system_var *in_k, t_system_var *out_k, t_system_state_space *system);
#endif // #if !defined(USE_INLINE)

/**
 * @brief Destroy the state-space "object"
 * @param system Address pointing to the state-space "object"
 * @return void 
 */
extern void state_space_linear_system_destroy(t_system_state_space *system);


#ifdef __cplusplus
}
#endif
 
#endif /* _LTI_SYSTEMS_H_ */
