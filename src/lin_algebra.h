/** @file lin-algebra.h
 * @brief Linear algebra basic operatos
 * @author Ivan Furlan
 */
        
#ifndef _LIN_ALGEBRA_H_
#define _LIN_ALGEBRA_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdbool.h> 
#include "type_defs_control_system_libs.h"
     
/**
 * \struct t_lin_algebra_errors
 * \brief possible lin algebra erros
 */
typedef enum {
    
	vect_mat_dimension_error = 0,
	operation_successfully_executed
            
} t_lin_algebra_errors;

/**
 * @brief Multiplication betwin a matrix and a vector
 * @param vect_out Address pointing to the result vector
 * @param mat Address pointing to the matrix
 * @param vect Address pointing to the vector
 * @param mat_h number of row of the matrix
 * @param mat_l number of column of the matrix
 * @return void
 */
#if defined(USE_INLINE) 
inline t_lin_algebra_errors mat_vect_mult(t_system_var *vect_out, t_system_var *mat, t_system_var *vect, int mat_h, int mat_l)
{
    
    int i,j;
    t_lin_algebra_errors ret;
    
    if(mat_h < 1 || mat_l < 1) {
    	ret = vect_mat_dimension_error;
    	return ret;
    }

    {
        t_system_var tmp_ = 0.0;
        for(j=0; j<mat_h; j++) {
            for(i=0; i<mat_l; i++) {
                tmp_ += mat[i+j*mat_l]*vect[i];
            }
            vect_out[j] = tmp_;
            tmp_ = 0.0;
        }
    }

    ret = operation_successfully_executed;
    return ret;
}
#else // #if defined(USE_INLINE)
extern t_lin_algebra_errors mat_vect_mult(t_system_var *vect_out, t_system_var *mat, t_system_var *vect, int mat_h, int mat_l);
#endif // #if !defined(USE_INLINE)

/**
 * @brief Add two vectors
 * @param vect_out Address pointing to the result vector
 * @param vect_1 Address pointing to the first addend vector
 * @param vect_2 Address pointing to the second addend vector
 * @return void
 */
#if defined(USE_INLINE)
inline t_lin_algebra_errors vects_sum(t_system_var *vect_out, t_system_var *vect_1, t_system_var *vect_2, int vect_h)
{
    
    int i;
    t_lin_algebra_errors ret;

    if(vect_h < 1) {
    	ret = vect_mat_dimension_error;
    	return ret;
    }
    
    for(i=0; i<vect_h; i++) {
        vect_out[i] = vect_1[i]+vect_2[i];
    }

    ret = operation_successfully_executed;
    return ret;
}
#else // #if defined(USE_INLINE)
extern t_lin_algebra_errors vects_sum(t_system_var *vect_out, t_system_var *vect_1, t_system_var *vect_2, int vect_h);
#endif // #if !defined(USE_INLINE)

/**
 * @brief Subtract two vectors
 * @param vect_out Address pointing to the result vector
 * @param vect_1 Address pointing to the first addend vector
 * @param vect_2 Address pointing to the second addend vector
 * @return void
 */
#if defined(USE_INLINE)
inline t_lin_algebra_errors vects_sub(t_system_var *vect_out, t_system_var *vect_1, t_system_var *vect_2, int vect_h)
{
    
    int i;
    t_lin_algebra_errors ret;

    if(vect_h < 1) {
    	ret = vect_mat_dimension_error;
    	return ret;
    }
    
    for(i=0; i<vect_h; i++) {
        vect_out[i] = vect_1[i]-vect_2[i];
    }

    ret = operation_successfully_executed;
    return ret;
}
#else // #if defined(USE_INLINE)
extern t_lin_algebra_errors vects_sub(t_system_var *vect_out, t_system_var *vect_1, t_system_var *vect_2, int vect_h);
#endif // #if !defined(USE_INLINE)

/**
 * @brief Change thej sign of the vector elements
 * @param vect_out Address pointing to the result vector
 * @param vect Address pointing to the input vector
 * @param vect_h Numbner of elemnts of the vector
 * @return void 
 */
#if defined(USE_INLINE)
inline t_lin_algebra_errors vect_el_sign_change(t_system_var *vect_out, t_system_var *vect_in, int vect_h)
{
    
    int i;
    t_lin_algebra_errors ret;

    if(vect_h < 1) {
    	ret = vect_mat_dimension_error;
    	return ret;
    }
    
    for(i=0; i<vect_h; i++) {
        vect_out[i] = -vect_in[i];
    }

    ret = operation_successfully_executed;
    return ret;
}
#else // #if defined(USE_INLINE)
extern t_lin_algebra_errors vect_el_sign_change(t_system_var *vect_out, t_system_var *vect_in, int vect_h);
#endif // #if !defined(USE_INLINE)

/**
 * @brief Create an eye matrix n x n
 * @param mat Address pointing to the eye matrix
 * @param n Dimension to the eye matrix
 * @return void 
 */
extern void eye_matrix_filling(t_system_var *mat, int n);

/**
 * @brief Create an zero matrix n_r x n_c
 * @param mat Address pointing to the zero matrix
 * @param n_r Number of rows of the zero matrix
 * @param n_c Number of columns of the zero matrix
 * @return void 
 */
extern void zero_matrix_filling(t_system_var *mat, int n_r, int n_c);

#ifdef __cplusplus
}
#endif
 
#endif /* _LIN_ALGEBRA_H_ */
