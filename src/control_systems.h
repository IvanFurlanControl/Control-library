/** @file control-systems.h
 * @brief control system lib
 * @author Ivan Furlan
 */
        
#ifndef _CONTROL_SYSTEMS_H_
#define _CONTROL_SYSTEMS_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdbool.h> 
#include "type_defs_control_system_libs.h"
#include "lti_systems.h"
#include <string.h>

/**
 * \struct t_polynomial_controller_antiwindup_form
 * \brief polynomial controller object structure
 */
typedef struct polynomial_controller{
    
    bool anti_windup;
    
    // for canonical form
    t_system_transfer_funtion transfer_function_poly_ctr;
    
    // for antiwind-up form form
	t_system_transfer_funtion transfer_function_numerator_fir;
	t_system_transfer_funtion transfer_function_denominator_fir;
    t_system_var prev_output;
    
} t_polynomial_controller;

/**
 * \struct t_system_state_feedback_controller
 * \brief state-feedback controller object structure
 */
typedef struct state_feedback_controller{
     int n_u; /** number of the system inputs  */
     int system_order; /** number of states */
     t_system_var *K; /** pointer to the state-feedback matrix gains */
     t_system_var *_u_p; /** support variable */
} t_state_feedback_controller;

/**
 * \struct t_system_state_observer
 * \brief state observer object structure
 */
typedef struct state_observer{
     int n_y; /** number of the observed system outputs */
     int n_u; /** number of the observed system inputs */
     t_system_var *input_observer; /** pointer to the observer input vector */
     t_system_state_space observer; /** observer object */
} t_state_observer;

/**
 * \struct t_state_feedback_controller_wfullobs
 * \brief state-feedback controller with full observer object structure
 */
typedef struct state_feedback_controller_wfullobs{
    t_state_feedback_controller state_feedback_controller;
    t_state_observer state_full_observer;
    t_system_var *hat_x_k;
} t_state_feedback_controller_wfullobs;

/**
 * @brief Saturator block
 * @param signal input signal to the saturator
 * @param upper_aw_limit upper saturation limit
 * @param lower_aw_limit lower saturation limit
 * @return saturated signal
 */
#if defined(USE_INLINE)
inline t_system_var saturator(t_system_var signal, t_system_var upper_aw_limit, t_system_var lower_aw_limit)
{
    signal = (signal > upper_aw_limit) ? upper_aw_limit : signal;
	signal = (signal < lower_aw_limit) ? lower_aw_limit : signal;
	
	return signal;
}
#else // #if defined(USE_INLINE)
extern t_system_var saturator(t_system_var signal, t_system_var upper_aw_limit, t_system_var lower_aw_limit);
#endif // #if !defined(USE_INLINE)

/**
 * @brief Create the polynomial controller in antiwindup form "object" 
 * @param order System order
 * @param anti_windup anti_windup form (sat. feeback) or canonical form (transfer-function)
 * @param num polynomial controller address pointing to the numerator coefficients
 * @param den polynomial controller address pointing to the denominator coefficients
 * @param system Address pointing to the polynomial controller in antiwindup form "object"
 * @return Returns the status of the initialization task
 */
extern t_lti_initialization_errors polynomial_controller_object_create(int order, bool anti_windup,t_system_var *num, t_system_var *den, t_polynomial_controller *system);

/**
 * @brief Reset a polynomial controller in anti-windup "object"
 * @param system Address pointing to the polynomial controller in antiwindup form "object"
 * @return void
 */
extern void polynomial_controller_object_reset(t_polynomial_controller *system);

/**
 * @brief Calculates the output of the polynomial controller difference equation defined in saturation feedback antiwidup form
 * @param in_k polynomial controller input at the current samplig instant k
 * @param upper_aw_limit upper antiwidup limit
 * @param lower_aw_limit lower antiwidup limit
 * @param system Address pointing to the polynomial controller "object"s
 * @return Returns polynomial controller output at the current samplig instant k
 */
#if defined(USE_INLINE)
inline t_system_var polynomial_controller_output_calc(t_system_var in_k, t_system_var upper_aw_limit, t_system_var lower_aw_limit, t_polynomial_controller *system)
{
    t_system_var  out_k = 0;
    
    if(system->anti_windup == true) {
        t_system_var  tmp_k = 0;
        
        tmp_k = transfer_function_linear_filter(in_k, &(system->transfer_function_numerator_fir)); 
        out_k = saturator(tmp_k+system->prev_output, upper_aw_limit, lower_aw_limit);
        system->prev_output = transfer_function_linear_filter(out_k, &(system->transfer_function_denominator_fir)); 
        
    } else {
        
        out_k = transfer_function_linear_filter(in_k, &(system->transfer_function_poly_ctr)); 
        
    }
    
	return out_k;
}
#else // #if defined(USE_INLINE)
extern t_system_var polynomial_controller_output_calc(t_system_var in_k, t_system_var upper_aw_limit, t_system_var lower_aw_limit, t_polynomial_controller *system);
#endif // #if !defined(USE_INLINE)

/**
 * @brief Destroy the polynomial controller in anti-windup form "object"
 * @param system Address pointing to the polynomial controller in anti-windup "object"
 * @return void 
 */
extern void polynomial_controller_object_destroy(t_polynomial_controller *system);

/**
 * @brief Create the state-feedback controller "object"
 * @param order System order
 * @param order n_u number of system inputs
 * @param K address pointing to the state-ffedback matrix gains 
 * @param system Address pointing to the state-feedback "object"
 * @return Returns the status of the initialization task
 */
extern t_lti_initialization_errors state_feedback_controller_create(int order, int n_u, t_system_var *K, t_state_feedback_controller *system);

/**
 * @brief Calculates the output of the state-feedback controller
 * @param ref_k pointer to the state-feedback controller reference vector
 * @param x_k pointer to the state-feedback states vector
 * @param u_k pointer to the state-feedback controller actuation vector
 * @param system Address pointing to the state-feedback controller "object"
 * @return void
 */
#if defined(USE_INLINE)
inline void state_feedback_controller_output_calc(t_system_var *ref_k, t_system_var *x_k, t_system_var *u_k, t_state_feedback_controller *system)
{
    
    // u' = Kx
    mat_vect_mult(system->_u_p, system->K, x_k, system->n_u, system->system_order);
    
    // u = ref-u'
	vects_sub(u_k, ref_k, system->_u_p, system->system_order);
    
}
#else // #if defined(USE_INLINE)
extern void state_feedback_controller_output_calc(t_system_var *ref_k, t_system_var *x_k, t_system_var *u_k, t_state_feedback_controller *system);
#endif // #if !defined(USE_INLINE)

/**
 * @brief Destroy the state-feedback controller "object"
 * @param system Address pointing to the state-fedback controller "object"
 * @return void 
 */
extern void state_feedback_controller_destroy(t_state_feedback_controller *system);

/**
 * @brief Create the state observer "object"
 * @param order system order
 * @param n_u number of system inputs
 * @param n_y number of system outputs
 * @param A address pointing to the matrix A of the state-space observer
 * @param B address pointing to the matrix B of the state-space observer
 * @param C address pointing to the matrix C of the state-space observer
 * @param D address pointing to the matrix D of the state-space observer
 * @param system Address pointing to the observer "object"
 * @return Returns the status of the initialization task
 */
extern t_lti_initialization_errors state_observer_create(int order, int n_u, int n_y, t_system_var *Aobs, t_system_var *Bobs, t_system_var *Cobs, t_system_var *Dobs, t_state_observer *system);

/**
 * @brief Reset an observer "object"
 * @param system Address pointing to the observer "object"
 * @return void
 */
extern void observer_reset(t_state_observer *system);

/**
 * @brief Calculates one iteration of the state-space observer
 * @param hat_x_k pointer to the estimated states vector
 * @param y_k pointer to the observed system output vector
 * @param u_k pointer to the observed system input vector
 * @param system Address pointing to the state-space observer
 * @return void
 */
#if defined(USE_INLINE)
inline void state_space_observer_estimation(t_system_var *hat_x_k, t_system_var *y_k, t_system_var *u_k, t_state_observer *system)
{
   int i = 0;
   
#if !defined(USE_MEM_OPERATOTS)   
   // obesrevr input vector definition
   for(i=0; i<system->n_y; i++) {
    system->input_observer[i] = y_k[i];
   }
#else // !defined(USE_MEM_OPERATOTS)   
   memcpy (&(system->input_observer[0]), y_k, system->n_y * sizeof(y_k[0]));    
#endif // defined(USE_MEM_OPERATOTS)   
   
#if !defined(USE_MEM_OPERATOTS)
   for(i=0; i<system->n_u; i++) {
    system->input_observer[i+system->n_y] = u_k[i];
   }
#else // !defined(USE_MEM_OPERATOTS) 
   memcpy (&(system->input_observer[system->n_y]), u_k, system->n_u * sizeof(u_k[0]));    
#endif // defined(USE_MEM_OPERATOTS) 
   
   // obeserver estimation
   state_space_linear_filter(system->input_observer, hat_x_k, &(system->observer));
}
#else // #if defined(USE_INLINE)
extern void state_space_observer_estimation(t_system_var *hat_x_k, t_system_var *y_k, t_system_var *u_k, t_state_observer *system);
#endif // #if !defined(USE_INLINE)

/**
 * @brief Destroy an observer "object"
 * @param system Address pointing to the observer "object"
 * @return void 
 */
extern void observer_destroy(t_state_observer *system);

/**
 * @brief Create the state-feedback controller with full observer "object"
 * @param order System order
 * @param K address pointing to the state-ffedback matrix gains
 * @param n_u number of system inputs
 * @param n_y number of system outputs
 * @param A address pointing to the matrix A of the state-space observer
 * @param B address pointing to the matrix B of the state-space observer 
 * @param system Address pointing to the state-feedback with full-observer "object"
 * @return Returns the status of the initialization task
 */
extern t_lti_initialization_errors state_feedback_wfullobs_controller_create(int order, t_system_var *K, int n_u, int n_y, t_system_var *Aobs, t_system_var *Bobs, t_state_feedback_controller_wfullobs *system);

/**
 * @brief Calculates the output of the state-feedback controller with full observer
 * @param ref_k pointer to the state-feedback controller reference vector
 * @param y_k pointer to the system output
 * @param u_k pointer to the state-feedback controller actuation vector
 * @param system Address pointing to the state-feedback controller with observer "object"
 * @return void
 */
#if defined(USE_INLINE)
inline void state_feedback_controller_wfullobs_output_calc(t_system_var *ref_k, t_system_var *y_k, t_system_var *u_k, t_state_feedback_controller_wfullobs *system)
{
    state_space_observer_estimation(system->hat_x_k, y_k, u_k, &(system->state_full_observer));
    state_feedback_controller_output_calc(ref_k, system->hat_x_k, u_k, &(system->state_feedback_controller));
}
#else // #if defined(USE_INLINE)
extern void state_feedback_controller_wfullobs_output_calc(t_system_var *ref_k, t_system_var *y_k, t_system_var *u_k, t_state_feedback_controller_wfullobs *system);
#endif // #if !defined(USE_INLINE)

/**
 * @brief Reset of the state-feedback controller with full observer "object"
 * @param system Address pointing to the observer "object"
 * @return void
 */
extern void state_feedback_controller_wfullobs_reset(t_state_feedback_controller_wfullobs *system);

/**
 * @brief Destroy the state-feedback controller with full-observer "object"
 * @param system Address pointing to the state-fedback controller with full-observer "object"
 * @return void 
 */
extern void state_feedback_controller_wfullobs_destroy(t_state_feedback_controller_wfullobs *system);

#ifdef __cplusplus
}
#endif
 
#endif /* _CONTROL_SYSTEMS_H_ */
