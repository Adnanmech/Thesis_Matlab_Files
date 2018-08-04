#ifndef __c2_Controller_ABS_VLC_AYC_ALGO2_h__
#define __c2_Controller_ABS_VLC_AYC_ALGO2_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct
#define typedef_SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c2_sfEvent;
  boolean_T c2_doneDoubleBufferReInit;
  uint8_T c2_is_active_c2_Controller_ABS_VLC_AYC_ALGO2;
  real_T c2_Kc1[120400];
  real_T c2_Kc2[120400];
  real_T c2_u[120400];
  real_T *c2_Vd_f;
  real_T (*c2_K_new)[2];
  real_T *c2_V_f;
  real_T (*c2_b_Kc1)[120400];
  real_T (*c2_b_Kc2)[120400];
} SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct;

#endif                                 /*typedef_SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c2_Controller_ABS_VLC_AYC_ALGO2_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c2_Controller_ABS_VLC_AYC_ALGO2_get_check_sum(mxArray *plhs[]);
extern void c2_Controller_ABS_VLC_AYC_ALGO2_method_dispatcher(SimStruct *S,
  int_T method, void *data);

#endif
