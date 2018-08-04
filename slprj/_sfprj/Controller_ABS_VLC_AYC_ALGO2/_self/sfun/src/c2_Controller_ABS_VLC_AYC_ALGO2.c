/* Include files */

#include "Controller_ABS_VLC_AYC_ALGO2_sfun.h"
#include "c2_Controller_ABS_VLC_AYC_ALGO2.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "Controller_ABS_VLC_AYC_ALGO2_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c_with_debugger(S, sfGlobalDebugInstanceStruct);

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization);
static void chart_debug_initialize_data_addresses(SimStruct *S);
static const mxArray* sf_opaque_get_hover_data_for_msg(void *chartInstance,
  int32_T msgSSID);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c2_debug_family_names[9] = { "V_fint", "Vd_fint", "nargin",
  "nargout", "Vd_f", "V_f", "Kc1", "Kc2", "K_new" };

/* Function Declarations */
static void initialize_c2_Controller_ABS_VLC_AYC_ALGO2
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance);
static void initialize_params_c2_Controller_ABS_VLC_AYC_ALGO2
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance);
static void enable_c2_Controller_ABS_VLC_AYC_ALGO2
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance);
static void disable_c2_Controller_ABS_VLC_AYC_ALGO2
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance);
static void c2_update_debugger_state_c2_Controller_ABS_VLC_AYC_ALGO2
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance);
static const mxArray *get_sim_state_c2_Controller_ABS_VLC_AYC_ALGO2
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance);
static void set_sim_state_c2_Controller_ABS_VLC_AYC_ALGO2
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance, const mxArray
   *c2_st);
static void finalize_c2_Controller_ABS_VLC_AYC_ALGO2
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance);
static void sf_gateway_c2_Controller_ABS_VLC_AYC_ALGO2
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance);
static void mdl_start_c2_Controller_ABS_VLC_AYC_ALGO2
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance);
static void initSimStructsc2_Controller_ABS_VLC_AYC_ALGO2
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber, uint32_T c2_instanceNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static void c2_emlrt_marshallIn(SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *
  chartInstance, const mxArray *c2_b_K_new, const char_T *c2_identifier, real_T
  c2_y[2]);
static void c2_b_emlrt_marshallIn
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance, const mxArray
   *c2_b_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[2]);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_c_emlrt_marshallIn
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance, const mxArray
   *c2_b_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static uint32_T c2_d_emlrt_marshallIn
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance, const mxArray
   *c2_b_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_e_emlrt_marshallIn
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance, const mxArray
   *c2_b_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static uint8_T c2_f_emlrt_marshallIn
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance, const mxArray
   *c2_b_is_active_c2_Controller_ABS_VLC_AYC_ALGO2, const char_T *c2_identifier);
static uint8_T c2_g_emlrt_marshallIn
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance, const mxArray
   *c2_b_u, const emlrtMsgIdentifier *c2_parentId);
static void init_dsm_address_info
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance);
static void init_simulink_io_address
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c2_Controller_ABS_VLC_AYC_ALGO2
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance)
{
  if (sf_is_first_init_cond(chartInstance->S)) {
    initSimStructsc2_Controller_ABS_VLC_AYC_ALGO2(chartInstance);
    chart_debug_initialize_data_addresses(chartInstance->S);
  }

  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c2_is_active_c2_Controller_ABS_VLC_AYC_ALGO2 = 0U;
}

static void initialize_params_c2_Controller_ABS_VLC_AYC_ALGO2
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c2_Controller_ABS_VLC_AYC_ALGO2
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c2_Controller_ABS_VLC_AYC_ALGO2
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c2_update_debugger_state_c2_Controller_ABS_VLC_AYC_ALGO2
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c2_Controller_ABS_VLC_AYC_ALGO2
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  const mxArray *c2_b_y = NULL;
  uint8_T c2_hoistedGlobal;
  uint8_T c2_b_u;
  const mxArray *c2_c_y = NULL;
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellmatrix(2, 1), false);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", *chartInstance->c2_K_new, 0, 0U, 1U,
    0U, 2, 1, 2), false);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_hoistedGlobal = chartInstance->c2_is_active_c2_Controller_ABS_VLC_AYC_ALGO2;
  c2_b_u = c2_hoistedGlobal;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  sf_mex_assign(&c2_st, c2_y, false);
  return c2_st;
}

static void set_sim_state_c2_Controller_ABS_VLC_AYC_ALGO2
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance, const mxArray
   *c2_st)
{
  const mxArray *c2_b_u;
  real_T c2_dv0[2];
  int32_T c2_i0;
  chartInstance->c2_doneDoubleBufferReInit = true;
  c2_b_u = sf_mex_dup(c2_st);
  c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell("K_new", c2_b_u,
    0)), "K_new", c2_dv0);
  for (c2_i0 = 0; c2_i0 < 2; c2_i0++) {
    (*chartInstance->c2_K_new)[c2_i0] = c2_dv0[c2_i0];
  }

  chartInstance->c2_is_active_c2_Controller_ABS_VLC_AYC_ALGO2 =
    c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(
    "is_active_c2_Controller_ABS_VLC_AYC_ALGO2", c2_b_u, 1)),
    "is_active_c2_Controller_ABS_VLC_AYC_ALGO2");
  sf_mex_destroy(&c2_b_u);
  c2_update_debugger_state_c2_Controller_ABS_VLC_AYC_ALGO2(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_Controller_ABS_VLC_AYC_ALGO2
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c2_Controller_ABS_VLC_AYC_ALGO2
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance)
{
  int32_T c2_i1;
  int32_T c2_i2;
  real_T c2_hoistedGlobal;
  real_T c2_b_hoistedGlobal;
  real_T c2_b_Vd_f;
  real_T c2_b_V_f;
  int32_T c2_i3;
  int32_T c2_i4;
  uint32_T c2_debug_family_var_map[9];
  uint32_T c2_V_fint;
  uint32_T c2_Vd_fint;
  real_T c2_nargin = 4.0;
  real_T c2_nargout = 1.0;
  real_T c2_b_K_new[2];
  real_T c2_d0;
  uint32_T c2_u0;
  uint32_T c2_b_u;
  const mxArray *c2_y = NULL;
  real_T c2_d1;
  uint32_T c2_u1;
  uint32_T c2_c_u;
  const mxArray *c2_b_y = NULL;
  int32_T c2_i5;
  int32_T c2_i6;
  int32_T c2_i7;
  const mxArray *c2_c_y = NULL;
  real_T c2_d_u[2];
  const mxArray *c2_d_y = NULL;
  int32_T c2_i8;
  int32_T c2_i9;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  for (c2_i1 = 0; c2_i1 < 120400; c2_i1++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_b_Kc2)[c2_i1], 3U, 1U, 0U,
                          chartInstance->c2_sfEvent, false);
  }

  for (c2_i2 = 0; c2_i2 < 120400; c2_i2++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_b_Kc1)[c2_i2], 2U, 1U, 0U,
                          chartInstance->c2_sfEvent, false);
  }

  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_V_f, 1U, 1U, 0U,
                        chartInstance->c2_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_Vd_f, 0U, 1U, 0U,
                        chartInstance->c2_sfEvent, false);
  chartInstance->c2_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  c2_hoistedGlobal = *chartInstance->c2_Vd_f;
  c2_b_hoistedGlobal = *chartInstance->c2_V_f;
  c2_b_Vd_f = c2_hoistedGlobal;
  c2_b_V_f = c2_b_hoistedGlobal;
  for (c2_i3 = 0; c2_i3 < 120400; c2_i3++) {
    chartInstance->c2_Kc1[c2_i3] = (*chartInstance->c2_b_Kc1)[c2_i3];
  }

  for (c2_i4 = 0; c2_i4 < 120400; c2_i4++) {
    chartInstance->c2_Kc2[c2_i4] = (*chartInstance->c2_b_Kc2)[c2_i4];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 9U, 9U, c2_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_V_fint, 0U, c2_d_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Vd_fint, 1U, c2_d_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 2U, c2_c_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 3U, c2_c_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_Vd_f, 4U, c2_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_V_f, 5U, c2_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(chartInstance->c2_Kc1, 6U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(chartInstance->c2_Kc2, 7U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_K_new, 8U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 4);
  c2_d0 = muDoubleScalarRound(c2_b_V_f * 10.0);
  if (c2_d0 < 4.294967296E+9) {
    if (CV_SATURATION_EVAL(4, 0, 0, 1, c2_d0 >= 0.0)) {
      c2_u0 = (uint32_T)c2_d0;
    } else {
      c2_u0 = 0U;
    }
  } else if (CV_SATURATION_EVAL(4, 0, 0, 0, c2_d0 >= 4.294967296E+9)) {
    c2_u0 = MAX_uint32_T;
  } else {
    c2_u0 = 0U;
  }

  c2_V_fint = c2_u0;
  sf_mex_printf("%s =\\n", "V_fint");
  c2_b_u = c2_V_fint;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_b_u, 7, 0U, 0U, 0U, 0), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "disp", 0U, 1U, 14, c2_y);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 5);
  c2_d1 = muDoubleScalarRound(c2_b_Vd_f * 10.0 + 151.0);
  if (c2_d1 < 4.294967296E+9) {
    if (CV_SATURATION_EVAL(4, 0, 1, 1, c2_d1 >= 0.0)) {
      c2_u1 = (uint32_T)c2_d1;
    } else {
      c2_u1 = 0U;
    }
  } else if (CV_SATURATION_EVAL(4, 0, 1, 0, c2_d1 >= 4.294967296E+9)) {
    c2_u1 = MAX_uint32_T;
  } else {
    c2_u1 = 0U;
  }

  c2_Vd_fint = c2_u1;
  sf_mex_printf("%s =\\n", "Vd_fint");
  c2_c_u = c2_Vd_fint;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_c_u, 7, 0U, 0U, 0U, 0), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "disp", 0U, 1U, 14, c2_b_y);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 6);
  if (CV_EML_IF(0, 1, 0, CV_RELATIONAL_EVAL(4U, 0U, 0, (real_T)c2_V_fint, 0.0,
        -1, 0U, (real_T)c2_V_fint == 0.0))) {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 7);
    for (c2_i5 = 0; c2_i5 < 2; c2_i5++) {
      c2_b_K_new[c2_i5] = 0.0;
    }

    sf_mex_printf("%s =\\n", "K_new");
    for (c2_i7 = 0; c2_i7 < 2; c2_i7++) {
      c2_d_u[c2_i7] = c2_b_K_new[c2_i7];
    }

    c2_d_y = NULL;
    sf_mex_assign(&c2_d_y, sf_mex_create("y", c2_d_u, 0, 0U, 1U, 0U, 2, 1, 2),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "disp", 0U, 1U, 14, c2_d_y);
  } else {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 9);
    c2_b_K_new[0] = chartInstance->c2_Kc1[(sf_eml_array_bounds_check
      (sfGlobalDebugInstanceStruct, chartInstance->S, 1U, 168, 6, MAX_uint32_T,
       (int32_T)(uint32_T)sf_integer_check(chartInstance->S, 1U, 168U, 6U,
      (real_T)c2_V_fint), 1, 400) + 400 * (sf_eml_array_bounds_check
      (sfGlobalDebugInstanceStruct, chartInstance->S, 1U, 175, 7, MAX_uint32_T,
       (int32_T)(uint32_T)sf_integer_check(chartInstance->S, 1U, 175U, 7U,
      (real_T)c2_Vd_fint), 1, 301) - 1)) - 1];
    c2_b_K_new[1] = chartInstance->c2_Kc2[(sf_eml_array_bounds_check
      (sfGlobalDebugInstanceStruct, chartInstance->S, 1U, 188, 6, MAX_uint32_T,
       (int32_T)(uint32_T)sf_integer_check(chartInstance->S, 1U, 188U, 6U,
      (real_T)c2_V_fint), 1, 400) + 400 * (sf_eml_array_bounds_check
      (sfGlobalDebugInstanceStruct, chartInstance->S, 1U, 195, 7, MAX_uint32_T,
       (int32_T)(uint32_T)sf_integer_check(chartInstance->S, 1U, 195U, 7U,
      (real_T)c2_Vd_fint), 1, 301) - 1)) - 1];
    sf_mex_printf("%s =\\n", "K_new");
    for (c2_i6 = 0; c2_i6 < 2; c2_i6++) {
      c2_d_u[c2_i6] = c2_b_K_new[c2_i6];
    }

    c2_c_y = NULL;
    sf_mex_assign(&c2_c_y, sf_mex_create("y", c2_d_u, 0, 0U, 1U, 0U, 2, 1, 2),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "disp", 0U, 1U, 14, c2_c_y);
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -9);
  _SFD_SYMBOL_SCOPE_POP();
  for (c2_i8 = 0; c2_i8 < 2; c2_i8++) {
    (*chartInstance->c2_K_new)[c2_i8] = c2_b_K_new[c2_i8];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_Controller_ABS_VLC_AYC_ALGO2MachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c2_i9 = 0; c2_i9 < 2; c2_i9++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_K_new)[c2_i9], 4U, 1U, 0U,
                          chartInstance->c2_sfEvent, false);
  }
}

static void mdl_start_c2_Controller_ABS_VLC_AYC_ALGO2
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void initSimStructsc2_Controller_ABS_VLC_AYC_ALGO2
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber, uint32_T c2_instanceNumber)
{
  (void)c2_machineNumber;
  (void)c2_chartNumber;
  (void)c2_instanceNumber;
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i10;
  const mxArray *c2_y = NULL;
  real_T c2_b_u[2];
  SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance;
  chartInstance = (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i10 = 0; c2_i10 < 2; c2_i10++) {
    c2_b_u[c2_i10] = (*(real_T (*)[2])c2_inData)[c2_i10];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_b_u, 0, 0U, 1U, 0U, 2, 1, 2), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_emlrt_marshallIn(SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *
  chartInstance, const mxArray *c2_b_K_new, const char_T *c2_identifier, real_T
  c2_y[2])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_K_new), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_K_new);
}

static void c2_b_emlrt_marshallIn
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance, const mxArray
   *c2_b_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[2])
{
  real_T c2_dv1[2];
  int32_T c2_i11;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_b_u), c2_dv1, 1, 0, 0U, 1, 0U, 2, 1,
                2);
  for (c2_i11 = 0; c2_i11 < 2; c2_i11++) {
    c2_y[c2_i11] = c2_dv1[c2_i11];
  }

  sf_mex_destroy(&c2_b_u);
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_K_new;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[2];
  int32_T c2_i12;
  SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance;
  chartInstance = (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *)
    chartInstanceVoid;
  c2_b_K_new = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_K_new), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_K_new);
  for (c2_i12 = 0; c2_i12 < 2; c2_i12++) {
    (*(real_T (*)[2])c2_outData)[c2_i12] = c2_y[c2_i12];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i13;
  int32_T c2_i14;
  const mxArray *c2_y = NULL;
  int32_T c2_i15;
  SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance;
  chartInstance = (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i13 = 0;
  for (c2_i14 = 0; c2_i14 < 301; c2_i14++) {
    for (c2_i15 = 0; c2_i15 < 400; c2_i15++) {
      chartInstance->c2_u[c2_i15 + c2_i13] = (*(real_T (*)[120400])c2_inData)
        [c2_i15 + c2_i13];
    }

    c2_i13 += 400;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", chartInstance->c2_u, 0, 0U, 1U, 0U, 2,
    400, 301), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_b_u;
  const mxArray *c2_y = NULL;
  SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance;
  chartInstance = (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_b_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static real_T c2_c_emlrt_marshallIn
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance, const mxArray
   *c2_b_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d2;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_b_u), &c2_d2, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d2;
  sf_mex_destroy(&c2_b_u);
  return c2_y;
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_nargout;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance;
  chartInstance = (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *)
    chartInstanceVoid;
  c2_nargout = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_nargout), &c2_thisId);
  sf_mex_destroy(&c2_nargout);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  uint32_T c2_b_u;
  const mxArray *c2_y = NULL;
  SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance;
  chartInstance = (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_b_u = *(uint32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_b_u, 7, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static uint32_T c2_d_emlrt_marshallIn
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance, const mxArray
   *c2_b_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint32_T c2_y;
  uint32_T c2_u2;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_b_u), &c2_u2, 1, 7, 0U, 0, 0U, 0);
  c2_y = c2_u2;
  sf_mex_destroy(&c2_b_u);
  return c2_y;
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_Vd_fint;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  uint32_T c2_y;
  SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance;
  chartInstance = (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *)
    chartInstanceVoid;
  c2_Vd_fint = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Vd_fint), &c2_thisId);
  sf_mex_destroy(&c2_Vd_fint);
  *(uint32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

const mxArray
  *sf_c2_Controller_ABS_VLC_AYC_ALGO2_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  sf_mex_assign(&c2_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), false);
  return c2_nameCaptureInfo;
}

static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_b_u;
  const mxArray *c2_y = NULL;
  SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance;
  chartInstance = (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_b_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_b_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static int32_T c2_e_emlrt_marshallIn
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance, const mxArray
   *c2_b_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i16;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_b_u), &c2_i16, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i16;
  sf_mex_destroy(&c2_b_u);
  return c2_y;
}

static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance;
  chartInstance = (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *)
    chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static uint8_T c2_f_emlrt_marshallIn
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance, const mxArray
   *c2_b_is_active_c2_Controller_ABS_VLC_AYC_ALGO2, const char_T *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_Controller_ABS_VLC_AYC_ALGO2), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_Controller_ABS_VLC_AYC_ALGO2);
  return c2_y;
}

static uint8_T c2_g_emlrt_marshallIn
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance, const mxArray
   *c2_b_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u3;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_b_u), &c2_u3, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u3;
  sf_mex_destroy(&c2_b_u);
  return c2_y;
}

static void init_dsm_address_info
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address
  (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance)
{
  chartInstance->c2_Vd_f = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c2_K_new = (real_T (*)[2])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c2_V_f = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c2_b_Kc1 = (real_T (*)[120400])ssGetInputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c2_b_Kc2 = (real_T (*)[120400])ssGetInputPortSignal_wrapper
    (chartInstance->S, 3);
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c2_Controller_ABS_VLC_AYC_ALGO2_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(709403724U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3608688569U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1968579637U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(307788071U);
}

mxArray* sf_c2_Controller_ABS_VLC_AYC_ALGO2_get_post_codegen_info(void);
mxArray *sf_c2_Controller_ABS_VLC_AYC_ALGO2_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("LFKS1UAhgxXTPG2Sd8KzOE");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(400);
      pr[1] = (double)(301);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(400);
      pr[1] = (double)(301);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(2);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo =
      sf_c2_Controller_ABS_VLC_AYC_ALGO2_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c2_Controller_ABS_VLC_AYC_ALGO2_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c2_Controller_ABS_VLC_AYC_ALGO2_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("pre");
  mxArray *fallbackReason = mxCreateString("hasBreakpoints");
  mxArray *hiddenFallbackType = mxCreateString("none");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c2_Controller_ABS_VLC_AYC_ALGO2_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c2_Controller_ABS_VLC_AYC_ALGO2_get_post_codegen_info(void)
{
  const char* fieldNames[] = { "exportedFunctionsUsedByThisChart",
    "exportedFunctionsChecksum" };

  mwSize dims[2] = { 1, 1 };

  mxArray* mxPostCodegenInfo = mxCreateStructArray(2, dims, sizeof(fieldNames)/
    sizeof(fieldNames[0]), fieldNames);

  {
    mxArray* mxExportedFunctionsChecksum = mxCreateString("");
    mwSize exp_dims[2] = { 0, 1 };

    mxArray* mxExportedFunctionsUsedByThisChart = mxCreateCellArray(2, exp_dims);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsUsedByThisChart",
               mxExportedFunctionsUsedByThisChart);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsChecksum",
               mxExportedFunctionsChecksum);
  }

  return mxPostCodegenInfo;
}

static const mxArray *sf_get_sim_state_info_c2_Controller_ABS_VLC_AYC_ALGO2(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"K_new\",},{M[8],M[0],T\"is_active_c2_Controller_ABS_VLC_AYC_ALGO2\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_Controller_ABS_VLC_AYC_ALGO2_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance =
      (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *)
      sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _Controller_ABS_VLC_AYC_ALGO2MachineNumber_,
           2,
           1,
           1,
           0,
           5,
           0,
           0,
           0,
           0,
           0,
           &chartInstance->chartNumber,
           &chartInstance->instanceNumber,
           (void *)S);

        /* Each instance must initialize its own list of scripts */
        init_script_number_translation
          (_Controller_ABS_VLC_AYC_ALGO2MachineNumber_,
           chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,
             _Controller_ABS_VLC_AYC_ALGO2MachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _Controller_ABS_VLC_AYC_ALGO2MachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"Vd_f");
          _SFD_SET_DATA_PROPS(1,1,1,0,"V_f");
          _SFD_SET_DATA_PROPS(2,1,1,0,"Kc1");
          _SFD_SET_DATA_PROPS(3,1,1,0,"Kc2");
          _SFD_SET_DATA_PROPS(4,2,0,1,"K_new");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,0,1,0,2,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,209);
        _SFD_CV_INIT_EML_SATURATION(0,1,0,65,-1,79);
        _SFD_CV_INIT_EML_SATURATION(0,1,1,90,-1,111);
        _SFD_CV_INIT_EML_IF(0,1,0,112,126,145,208);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,0,115,126,-1,0);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[2];
          dimVector[0]= 400U;
          dimVector[1]= 301U;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 400U;
          dimVector[1]= 301U;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1U;
          dimVector[1]= 2U;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)
            c2_sf_marshallIn);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _Controller_ABS_VLC_AYC_ALGO2MachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static void chart_debug_initialize_data_addresses(SimStruct *S)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance =
      (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *)
      sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S)) {
      /* do this only if simulation is starting and after we know the addresses of all data */
      {
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c2_Vd_f);
        _SFD_SET_DATA_VALUE_PTR(4U, *chartInstance->c2_K_new);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c2_V_f);
        _SFD_SET_DATA_VALUE_PTR(2U, *chartInstance->c2_b_Kc1);
        _SFD_SET_DATA_VALUE_PTR(3U, *chartInstance->c2_b_Kc2);
      }
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "sWYHzI49XKxO1v5xE3Rf97F";
}

static void sf_opaque_initialize_c2_Controller_ABS_VLC_AYC_ALGO2(void
  *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c2_Controller_ABS_VLC_AYC_ALGO2
    ((SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct*) chartInstanceVar);
  initialize_c2_Controller_ABS_VLC_AYC_ALGO2
    ((SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c2_Controller_ABS_VLC_AYC_ALGO2(void
  *chartInstanceVar)
{
  enable_c2_Controller_ABS_VLC_AYC_ALGO2
    ((SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c2_Controller_ABS_VLC_AYC_ALGO2(void
  *chartInstanceVar)
{
  disable_c2_Controller_ABS_VLC_AYC_ALGO2
    ((SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c2_Controller_ABS_VLC_AYC_ALGO2(void
  *chartInstanceVar)
{
  sf_gateway_c2_Controller_ABS_VLC_AYC_ALGO2
    ((SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct*) chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c2_Controller_ABS_VLC_AYC_ALGO2
  (SimStruct* S)
{
  return get_sim_state_c2_Controller_ABS_VLC_AYC_ALGO2
    ((SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *)
     sf_get_chart_instance_ptr(S));    /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c2_Controller_ABS_VLC_AYC_ALGO2(SimStruct* S,
  const mxArray *st)
{
  set_sim_state_c2_Controller_ABS_VLC_AYC_ALGO2
    ((SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct*)sf_get_chart_instance_ptr
     (S), st);
}

static void sf_opaque_terminate_c2_Controller_ABS_VLC_AYC_ALGO2(void
  *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct*)
                    chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_Controller_ABS_VLC_AYC_ALGO2_optimization_info();
    }

    finalize_c2_Controller_ABS_VLC_AYC_ALGO2
      ((SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct*) chartInstanceVar);
    utFree(chartInstanceVar);
    if (ssGetUserData(S)!= NULL) {
      sf_free_ChartRunTimeInfo(S);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_Controller_ABS_VLC_AYC_ALGO2
    ((SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_Controller_ABS_VLC_AYC_ALGO2(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_Controller_ABS_VLC_AYC_ALGO2
      ((SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct*)
       sf_get_chart_instance_ptr(S));
  }
}

static void mdlSetWorkWidths_c2_Controller_ABS_VLC_AYC_ALGO2(SimStruct *S)
{
  /* Set overwritable ports for inplace optimization */
  ssMdlUpdateIsEmpty(S, 1);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_Controller_ABS_VLC_AYC_ALGO2_optimization_info
      (sim_mode_is_rtw_gen(S), sim_mode_is_modelref_sim(S), sim_mode_is_external
       (S));
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,1);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,2,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_set_chart_accesses_machine_info(S, sf_get_instance_specialization(),
      infoStruct, 2);
    sf_update_buildInfo(S, sf_get_instance_specialization(),infoStruct,2);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 4; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1487006006U));
  ssSetChecksum1(S,(3182071265U));
  ssSetChecksum2(S,(4199646965U));
  ssSetChecksum3(S,(3967030213U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSetStateSemanticsClassicAndSynchronous(S, true);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c2_Controller_ABS_VLC_AYC_ALGO2(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_Controller_ABS_VLC_AYC_ALGO2(SimStruct *S)
{
  SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *chartInstance;
  chartInstance = (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct *)utMalloc
    (sizeof(SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof
         (SFc2_Controller_ABS_VLC_AYC_ALGO2InstanceStruct));
  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c2_Controller_ABS_VLC_AYC_ALGO2;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c2_Controller_ABS_VLC_AYC_ALGO2;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c2_Controller_ABS_VLC_AYC_ALGO2;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c2_Controller_ABS_VLC_AYC_ALGO2;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c2_Controller_ABS_VLC_AYC_ALGO2;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_Controller_ABS_VLC_AYC_ALGO2;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_Controller_ABS_VLC_AYC_ALGO2;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_Controller_ABS_VLC_AYC_ALGO2;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_Controller_ABS_VLC_AYC_ALGO2;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_Controller_ABS_VLC_AYC_ALGO2;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c2_Controller_ABS_VLC_AYC_ALGO2;
  chartInstance->chartInfo.callGetHoverDataForMsg = NULL;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  sf_init_ChartRunTimeInfo(S, &(chartInstance->chartInfo), false, 0);
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  chart_debug_initialization(S,1);
  mdl_start_c2_Controller_ABS_VLC_AYC_ALGO2(chartInstance);
}

void c2_Controller_ABS_VLC_AYC_ALGO2_method_dispatcher(SimStruct *S, int_T
  method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_Controller_ABS_VLC_AYC_ALGO2(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_Controller_ABS_VLC_AYC_ALGO2(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_Controller_ABS_VLC_AYC_ALGO2(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_Controller_ABS_VLC_AYC_ALGO2_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
