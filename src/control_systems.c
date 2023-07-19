/** @file lti-systems.c
 * @brief control system lib
 * @author Ivan Furlan
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h> 
#include "type_defs_control_system_libs.h"
#include "control_systems.h"
#include "lti_systems.h"
#include "lin_algebra.h"
#include <string.h>

/**
 * @brief Create the polynomial controller in antiwindup form "object" 
 * @param order System order
 * @param anti_windup anti_windup form (sat. feeback) or canonical form (transfer-function)
 * @param num polynomial controller address pointing to the numerator coefficients
 * @param den polynomial controller address pointing to the denominator coefficients
 * @param system Address pointing to the polynomial controller in antiwindup form "object"
 * @return Returns the status of the initialization task
 */
t_lti_initialization_errors polynomial_controller_object_create(int order, bool anti_windup, t_system_var *num, t_system_var *den, t_polynomial_controller *system)
{
    t_lti_initialization_errors ret;

	
    // inputs check
    if(order<1) {
        ret = order_zero_or_negative;
        return ret;
    }
   
	
    if(anti_windup == true) {
        
        // creation of the various numerator and denominator needed for the FIR filters present in the stauration feedback antiwindup form     
        unsigned int i = 0;
        
        system->anti_windup = true;
                
        t_system_var *num_first_stage_aw;
        t_system_var *den_first_stage_aw;
        t_system_var *num_second_stage_aw;
        t_system_var *den_second_stage_aw;
    
        num_first_stage_aw = (t_system_var *)malloc((order+1) * sizeof (t_system_var));
        den_first_stage_aw = (t_system_var *)malloc((order+1) * sizeof (t_system_var));
        num_second_stage_aw = (t_system_var *)malloc((order+1) * sizeof (t_system_var));
        den_second_stage_aw = (t_system_var *)malloc((order+1) * sizeof (t_system_var));
        
        // memory allocation check
        if (num_first_stage_aw==NULL || den_first_stage_aw==NULL || num_second_stage_aw==NULL || den_second_stage_aw==NULL) {
            ret = memory_space_cannot_be_allocated;
        #ifdef DEBUG_MODE
            perror("Error: ");
        #endif /*DEBUG_MODE*/
        #ifdef CHECK_MEMORY_ALLOCATION_ENABLEED
            return ret;
        #endif /*CHECK_MEMORY_ALLOCATION_ENABLEED*/
        }

		// clear of the temporary vectors
		for(i=0; i<order+1; i++) {
            num_first_stage_aw[i] = 0.0;
            den_first_stage_aw[i] = 0.0;
			num_second_stage_aw[i] = 0.0;
            den_second_stage_aw[i] = 0.0;
        }
    
		// creation of the antiwindup "first stage" FIR numerator
		for(i=0; i<order+1; i++) {
			num_first_stage_aw[i] = num[i];
		}
		// creation of the antiwindup "first stage" FIR denominator
		den_first_stage_aw[0] = 1;
		
		// creation of the antiwindup "second stage"  FIR numerator (i.e. z*(1-D(z)))
		for(i=0; i<order; i++) {
			num_second_stage_aw[i] = -den[i+1]; //signaed changed becouse 1-D
		}
		// creation of the antiwindup "second stage" FIR denominator
		den_second_stage_aw[0] = 1;
		
        // creation of the antiwidup "first stage" FIR object
        {
            t_lti_initialization_errors init_rct_delay;
            init_rct_delay = transfer_function_system_create(order, num_first_stage_aw,  den_first_stage_aw, &(system->transfer_function_numerator_fir));
            if(init_rct_delay != init_successful) {
                return ret;
            }
        }
	
        // creation of the antiwidup "second stage" FIR object z*(1-D)
        {
            t_lti_initialization_errors init_rct_delay;
            init_rct_delay = transfer_function_system_create(order, num_second_stage_aw,  den_second_stage_aw, &(system->transfer_function_denominator_fir));
            if(init_rct_delay != init_successful) {
                return ret;
            }
        }
        
    } else {
		
        // creation of the polynomial controller transfer-function
        {
            t_lti_initialization_errors init_rct_delay;
            
            system->anti_windup = false;
            
            init_rct_delay = transfer_function_system_create(order, num,  den, &(system->transfer_function_poly_ctr));
            if(init_rct_delay != init_successful) {
                return ret;
            }
        } 
    }
    
	ret = init_successful;
	return ret;
}

/**
 * @brief Reset a polynomial controller in anti-windup "object"
 * @param system Address pointing to the polynomial controller in antiwindup form "object"
 * @return void
 */
void polynomial_controller_object_reset(t_polynomial_controller *system)
{
    if(system->anti_windup == true) {
        transfer_function_system_reset(&(system->transfer_function_numerator_fir));
        transfer_function_system_reset(&(system->transfer_function_denominator_fir));
        system->prev_output = 0;
    } else {
        transfer_function_system_reset(&(system->transfer_function_poly_ctr));
    }
}

/**
 * @brief Calculates the output of the polynomial controller difference equation defined in saturation feedback antiwidup form
 * @param in_k polynomial controller input at the current samplig instant k
 * @param upper_aw_limit upper antiwidup limit
 * @param lower_aw_limit lower antiwidup limit
 * @param system Address pointing to the polynomial controller "object"s
 * @return Returns polynomial controller output at the current samplig instant k
 */
#if !defined(USE_INLINE)
t_system_var polynomial_controller_output_calc(t_system_var in_k, t_system_var upper_aw_limit, t_system_var lower_aw_limit, t_polynomial_controller *system)
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
#endif // #if !defined(USE_INLINE)

/**
 * @brief Saturator block
 * @param signal input signal to the saturator
 * @param upper_aw_limit upper saturation limit
 * @param lower_aw_limit lower saturation limit
 * @return saturated signal
 */
#if !defined(USE_INLINE)
t_system_var saturator(t_system_var signal, t_system_var upper_aw_limit, t_system_var lower_aw_limit)
{
    signal = (signal > upper_aw_limit) ? upper_aw_limit : signal;
	signal = (signal < lower_aw_limit) ? lower_aw_limit : signal;
	
	return signal;
}
#endif // #if !defined(USE_INLINE)

/**
 * @brief Destroy the polynomial controller in anti-windup form "object"
 * @param system Address pointing to the polynomial controller in anti-windup "object"
 * @return void 
 */
void polynomial_controller_object_destroy(t_polynomial_controller *system)
{
    if(system->anti_windup == true) {
        transfer_function_system_destroy(&(system->transfer_function_numerator_fir));
        transfer_function_system_destroy(&(system->transfer_function_denominator_fir));
    } else {
        transfer_function_system_destroy(&(system->transfer_function_poly_ctr));
    }
}

/**
 * @brief Create the state-feedback controller "object"
 * @param order System order
 * @param order n_u number of system inputs
 * @param K address pointing to the state-ffedback matrix gains 
 * @param system Address pointing to the state-feedback "object"
 * @return Returns the status of the initialization task
 */
t_lti_initialization_errors state_feedback_controller_create(int order, int n_u, t_system_var *K, t_state_feedback_controller *system)
{
    
    t_lti_initialization_errors ret;
    
    // inputs check
    if(order<1) {
        ret = order_zero_or_negative;
        return ret;
    }
    
    if(n_u<1) {
        ret = nu_or_ny_negative;
        return ret;
    }
        
    // definition of system number of input and output
    system->n_u = n_u;
    system->system_order = order;
    
    // definition of the memory space for the state-feedback matrix
    system->K = (t_system_var *)malloc((system->n_u)*(system->system_order) * sizeof (t_system_var));
    system->_u_p = (t_system_var *)malloc((system->n_u) * sizeof (t_system_var));
    
    // memory allocation check
    if (system->K==NULL || system->_u_p==NULL) {
        ret = memory_space_cannot_be_allocated;
#ifdef DEBUG_MODE
        perror("Error: ");
#endif /*DEBUG_MODE*/
#ifdef CHECK_MEMORY_ALLOCATION_ENABLEED
        return ret;
#endif /*CHECK_MEMORY_ALLOCATION_ENABLEED*/
    }
    
    // initialization of the state feedback matrix
    {
        unsigned int i = 0;
        
        // matrix K definition: 
        for (i=0; i<(system->n_u)*(system->system_order); i++) {
            system->K[i]=K[i];
        }
    }
    
    ret = init_successful;
    return ret;
}

/**
 * @brief Calculates the output of the state-feedback controller
 * @param ref_k pointer to the state-feedback controller reference vector
 * @param x_k pointer to the state-feedback states vector
 * @param u_k pointer to the state-feedback controller actuation vector
 * @param system Address pointing to the state-feedback controller "object"
 * @return void
 */
#if !defined(USE_INLINE)
void state_feedback_controller_output_calc(t_system_var *ref_k, t_system_var *x_k, t_system_var *u_k, t_state_feedback_controller *system)
{
    
    // u' = Kx
    mat_vect_mult(system->_u_p, system->K, x_k, system->n_u, system->system_order);
    
    // u = ref-u'
	vects_sub(u_k, ref_k, system->_u_p, system->system_order);
    
}
#endif // #if !defined(USE_INLINE)

/**
 * @brief Destroy the state-feedback controller "object"
 * @param system Address pointing to the state-fedback controller "object"
 * @return void 
 */
void state_feedback_controller_destroy(t_state_feedback_controller *system) {

    // deallocation the memory reserved for difing the state-space lti system 
    free(system->K);
    free(system->_u_p);
    
}

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
t_lti_initialization_errors state_observer_create(int order, int n_u, int n_y, t_system_var *Aobs, t_system_var *Bobs, t_system_var *Cobs, t_system_var *Dobs, t_state_observer *system)
{
    
    t_lti_initialization_errors ret;
    int i = 0;
    
    // inputs check
    if(order<1) {
        ret = order_zero_or_negative;
        return ret;
    }
    
    if(n_u<1 || n_y<1) {
        ret = nu_or_ny_negative;
        return ret;
    }
             
    // definiton of the observer object
    ret = state_space_system_create(order, n_u+n_y, order, (t_system_var *)Aobs, (t_system_var *)Bobs, (t_system_var *)Cobs, (t_system_var *)Dobs, &(system->observer));
    if (ret != init_successful) {
		return ret;
	}
	
    // definiton of the system n_y and n_u
    system->n_y = n_y;
    system->n_u = n_u;
            
    // definiton of the observer input vector
    system->input_observer = (t_system_var *)malloc((n_u+n_y) * sizeof (t_system_var));
    
	if (system->input_observer == NULL) {
        ret = memory_space_cannot_be_allocated;
	}		
    
	ret = init_successful;
    return ret;
}

/**
 * @brief Reset an observer "object"
 * @param system Address pointing to the observer "object"
 * @return void
 */
void observer_reset(t_state_observer *system)
{
    state_space_system_reset(&(system->observer));
}

/**
 * @brief Calculates one iteration of the state-space observer
 * @param hat_x_k pointer to the estimated states vector
 * @param y_k pointer to the observed system output vector
 * @param u_k pointer to the observed system input vector
 * @param system Address pointing to the state-space observer
 * @return void
 */
#if !defined(USE_INLINE)
void state_space_observer_estimation(t_system_var *hat_x_k, t_system_var *y_k, t_system_var *u_k, t_state_observer *system)
{
   int i = 0;
    
   // obesrevr input vector definition
#if !defined(USE_MEM_OPERATOTS)
   for(i=0; i<system->n_y; i++) {
    system->input_observer[i] = y_k[i];
   }
#else //!defined(USE_MEM_OPERATOTS)
   memcpy (&(system->input_observer[0]), y_k, system->n_y * sizeof(y_k[0]));    
#endif //defined(USE_MEM_OPERATOTS)
  
#if !defined(USE_MEM_OPERATOTS)  
   for(i=0; i<system->n_u; i++) {
    system->input_observer[i+system->n_y] = u_k[i];
   }
#else //!defined(USE_MEM_OPERATOTS)
   memcpy (&(system->input_observer[system->n_y]), u_k, system->n_u * sizeof(u_k[0]));    
#endif  //defined(USE_MEM_OPERATOTS)
   
   // obeserver estimation
   state_space_linear_filter(system->input_observer, hat_x_k, &(system->observer));
}
#endif // #if !defined(USE_INLINE)

/**
 * @brief Destroy an observer "object"
 * @param system Address pointing to the observer "object"
 * @return void 
 */
void observer_destroy(t_state_observer *system)
{
    free(system->input_observer);
    state_space_linear_system_destroy(&(system->observer));
}

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
t_lti_initialization_errors state_feedback_wfullobs_controller_create(int order, t_system_var *K, int n_u, int n_y, t_system_var *Aobs, t_system_var *Bobs, t_state_feedback_controller_wfullobs *system)
{
    t_lti_initialization_errors ret = init_successful;
    t_lti_initialization_errors ret_sf_ctr;
    t_lti_initialization_errors ret_obs;
    
    t_system_var *Cobs = (t_system_var *)malloc((order)*(order) * sizeof (t_system_var));
    t_system_var *Dobs = (t_system_var *)malloc((order)*(n_y+n_u) * sizeof (t_system_var));
    system->hat_x_k = (t_system_var *)malloc((order)* sizeof (t_system_var));
    if(Cobs == NULL || Dobs == NULL || system->hat_x_k == NULL) {
		ret = memory_space_cannot_be_allocated;
	}
    
	// Cobs = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]
    eye_matrix_filling((t_system_var*)Cobs, order);
        
    // Dobs = [0 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0]
    zero_matrix_filling((t_system_var*)Dobs, order, n_y+n_u);
      
    ret_sf_ctr = state_feedback_controller_create(order, n_u, (t_system_var *)K, &(system->state_feedback_controller));
    ret_obs = state_observer_create(order, n_u, n_y, (t_system_var*)Aobs, (t_system_var*)Bobs, (t_system_var*)Cobs, (t_system_var*)Dobs, &(system->state_full_observer));
    
    if(ret_sf_ctr != init_successful) {
        ret = ret_sf_ctr;
    }
    if(ret_obs != init_successful) {
        ret = ret_obs;
    }
    
    return ret;
}

/**
 * @brief Calculates the output of the state-feedback controller with full observer
 * @param ref_k pointer to the state-feedback controller reference vector
 * @param y_k pointer to the system output
 * @param u_k pointer to the state-feedback controller actuation vector
 * @param system Address pointing to the state-feedback controller with observer "object"
 * @return void
 */
#if !defined(USE_INLINE)
void state_feedback_controller_wfullobs_output_calc(t_system_var *ref_k, t_system_var *y_k, t_system_var *u_k, t_state_feedback_controller_wfullobs *system)
{
    state_space_observer_estimation(system->hat_x_k, y_k, u_k, &(system->state_full_observer));
    state_feedback_controller_output_calc(ref_k, system->hat_x_k, u_k, &(system->state_feedback_controller));
}
#endif // #if !defined(USE_INLINE)

/**
 * @brief Reset of the state-feedback controller with full observer "object"
 * @param system Address pointing to the observer "object"
 * @return void
 */
void state_feedback_controller_wfullobs_reset(t_state_feedback_controller_wfullobs *system)
{
    observer_reset(&(system->state_full_observer));
}

/**
 * @brief Destroy the state-feedback controller with full-observer "object"
 * @param system Address pointing to the state-fedback controller with full-observer "object"
 * @return void 
 */
void state_feedback_controller_wfullobs_destroy(t_state_feedback_controller_wfullobs *system)
{
    observer_destroy(&(system->state_full_observer));
    state_feedback_controller_destroy(&(system->state_feedback_controller));
}
