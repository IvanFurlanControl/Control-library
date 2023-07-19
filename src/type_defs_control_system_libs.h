/** @file type-defs-control-system-lib.h
 * @brief Definitions of the types used for the control system lib
 * @author Ivan Furlan
 */
        
#ifndef _TYPE_DEFS_CONTROL_SYSTEM_LIB_H_
#define _TYPE_DEFS_CONTROL_SYSTEM_LIB_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdbool.h> 
 
// definition of the variable type used in the linear system
typedef float t_system_var;
//typedef double t_system_var;

// for using inline for the execution time critical functions
// for using the code in simulink within a s_function use 
// not INLINE functions (i.e. USE_INLINE must be not defined)
#define USE_INLINE 

// no any performance increase has been seen by using memmove or memcpy functions 
// prehaps becouse thiose functions are not inline
// in any case if those function want be used insted of for cycles uncomment
// the below define  
//define USE_MEM_OPERATOTS

// for activating malloc memory check
#define CHECK_MEMORY_ALLOCATION_ENABLEED

#endif /* _TYPE_DEFS_CONTROL_SYSTEM_LIB_H_ */
