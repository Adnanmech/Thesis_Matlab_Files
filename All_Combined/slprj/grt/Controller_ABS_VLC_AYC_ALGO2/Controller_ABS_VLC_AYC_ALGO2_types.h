/*
 * Controller_ABS_VLC_AYC_ALGO2_types.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Controller_ABS_VLC_AYC_ALGO2".
 *
 * Model version              : 1.288
 * Simulink Coder version : 8.10 (R2016a) 10-Feb-2016
 * C source code generated on : Sat May 06 23:44:30 2017
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_Controller_ABS_VLC_AYC_ALGO2_types_h_
#define RTW_HEADER_Controller_ABS_VLC_AYC_ALGO2_types_h_
#include "rtwtypes.h"
#include "model_reference_types.h"
#include "multiword_types.h"
#ifndef struct_md2de03d95441e5b3cc1e40ff6e95f39e0
#define struct_md2de03d95441e5b3cc1e40ff6e95f39e0

struct md2de03d95441e5b3cc1e40ff6e95f39e0
{
  int32_T S0_isInitialized;
  real_T W0_states[69];
  real_T P0_InitialStates;
  real_T P1_Coefficients[70];
};

#endif                                 /*struct_md2de03d95441e5b3cc1e40ff6e95f39e0*/

#ifndef typedef_dsp_FIRFilter_0_Controller_AB_T
#define typedef_dsp_FIRFilter_0_Controller_AB_T

typedef struct md2de03d95441e5b3cc1e40ff6e95f39e0
  dsp_FIRFilter_0_Controller_AB_T;

#endif                                 /*typedef_dsp_FIRFilter_0_Controller_AB_T*/

#ifndef struct_mdtRzZ7gIX8BqTYbgSGc4hLE
#define struct_mdtRzZ7gIX8BqTYbgSGc4hLE

struct mdtRzZ7gIX8BqTYbgSGc4hLE
{
  int32_T isInitialized;
  dsp_FIRFilter_0_Controller_AB_T cSFunObject;
};

#endif                                 /*struct_mdtRzZ7gIX8BqTYbgSGc4hLE*/

#ifndef typedef_dspcodegen_FIRFilter_Controll_T
#define typedef_dspcodegen_FIRFilter_Controll_T

typedef struct mdtRzZ7gIX8BqTYbgSGc4hLE dspcodegen_FIRFilter_Controll_T;

#endif                                 /*typedef_dspcodegen_FIRFilter_Controll_T*/

#ifndef struct_mdWR8mPXbeWK9sD44gRxHVa
#define struct_mdWR8mPXbeWK9sD44gRxHVa

struct mdWR8mPXbeWK9sD44gRxHVa
{
  int32_T isInitialized;
  real_T pSampleRateInherit;
  real_T NumChannels;
  dspcodegen_FIRFilter_Controll_T *FilterObj;
};

#endif                                 /*struct_mdWR8mPXbeWK9sD44gRxHVa*/

#ifndef typedef_dsp_LowpassFilter_Controller__T
#define typedef_dsp_LowpassFilter_Controller__T

typedef struct mdWR8mPXbeWK9sD44gRxHVa dsp_LowpassFilter_Controller__T;

#endif                                 /*typedef_dsp_LowpassFilter_Controller__T*/

#ifndef typedef_struct_T_Controller_ABS_VLC_A_T
#define typedef_struct_T_Controller_ABS_VLC_A_T

typedef struct {
  char_T f0[7];
} struct_T_Controller_ABS_VLC_A_T;

#endif                                 /*typedef_struct_T_Controller_ABS_VLC_A_T*/

#ifndef typedef_struct_T_Controller_ABS_VLC_c_T
#define typedef_struct_T_Controller_ABS_VLC_c_T

typedef struct {
  char_T f0[6];
} struct_T_Controller_ABS_VLC_c_T;

#endif                                 /*typedef_struct_T_Controller_ABS_VLC_c_T*/

#ifndef typedef_struct_T_Controller_ABS_VL_cb_T
#define typedef_struct_T_Controller_ABS_VL_cb_T

typedef struct {
  char_T f0[6];
  char_T f1[6];
  char_T f2[4];
  char_T f3[8];
} struct_T_Controller_ABS_VL_cb_T;

#endif                                 /*typedef_struct_T_Controller_ABS_VL_cb_T*/

#ifndef typedef_struct_T_Controller_ABS_V_cbt_T
#define typedef_struct_T_Controller_ABS_V_cbt_T

typedef struct {
  char_T f0[6];
  char_T f1[6];
  char_T f2[4];
  char_T f3[8];
  char_T f4[2];
  real_T f5;
} struct_T_Controller_ABS_V_cbt_T;

#endif                                 /*typedef_struct_T_Controller_ABS_V_cbt_T*/

#ifndef typedef_struct_T_Controller_ABS__cbt4_T
#define typedef_struct_T_Controller_ABS__cbt4_T

typedef struct {
  char_T f0[6];
  char_T f1[6];
  char_T f2[4];
  char_T f3[8];
  char_T f4;
  real_T f5;
} struct_T_Controller_ABS__cbt4_T;

#endif                                 /*typedef_struct_T_Controller_ABS__cbt4_T*/

#ifndef struct_mda9d534d69f46e44bd4f94f8f2d954130
#define struct_mda9d534d69f46e44bd4f94f8f2d954130

struct mda9d534d69f46e44bd4f94f8f2d954130
{
  int32_T S0_isInitialized;
  real_T W0_states[25];
  real_T P0_InitialStates;
  real_T P1_Coefficients[26];
};

#endif                                 /*struct_mda9d534d69f46e44bd4f94f8f2d954130*/

#ifndef typedef_dsp_FIRFilter_0_Controller__l_T
#define typedef_dsp_FIRFilter_0_Controller__l_T

typedef struct mda9d534d69f46e44bd4f94f8f2d954130
  dsp_FIRFilter_0_Controller__l_T;

#endif                                 /*typedef_dsp_FIRFilter_0_Controller__l_T*/

#ifndef struct_mdzYjaZyczglywJ4qmQmKUYD
#define struct_mdzYjaZyczglywJ4qmQmKUYD

struct mdzYjaZyczglywJ4qmQmKUYD
{
  int32_T isInitialized;
  dsp_FIRFilter_0_Controller__l_T cSFunObject;
};

#endif                                 /*struct_mdzYjaZyczglywJ4qmQmKUYD*/

#ifndef typedef_dspcodegen_FIRFilter_Contro_p_T
#define typedef_dspcodegen_FIRFilter_Contro_p_T

typedef struct mdzYjaZyczglywJ4qmQmKUYD dspcodegen_FIRFilter_Contro_p_T;

#endif                                 /*typedef_dspcodegen_FIRFilter_Contro_p_T*/

#ifndef struct_mdrHcJfOSjlkTCWl8K5ThuAG
#define struct_mdrHcJfOSjlkTCWl8K5ThuAG

struct mdrHcJfOSjlkTCWl8K5ThuAG
{
  int32_T isInitialized;
  real_T pSampleRateInherit;
  real_T NumChannels;
  dspcodegen_FIRFilter_Contro_p_T *FilterObj;
};

#endif                                 /*struct_mdrHcJfOSjlkTCWl8K5ThuAG*/

#ifndef typedef_dsp_LowpassFilter_Controlle_a_T
#define typedef_dsp_LowpassFilter_Controlle_a_T

typedef struct mdrHcJfOSjlkTCWl8K5ThuAG dsp_LowpassFilter_Controlle_a_T;

#endif                                 /*typedef_dsp_LowpassFilter_Controlle_a_T*/

#ifndef struct_md62ec9d63a416a03de13eedbdf423dac2
#define struct_md62ec9d63a416a03de13eedbdf423dac2

struct md62ec9d63a416a03de13eedbdf423dac2
{
  int32_T S0_isInitialized;
  real_T W0_states[116];
  real_T P0_InitialStates;
  real_T P1_Coefficients[117];
};

#endif                                 /*struct_md62ec9d63a416a03de13eedbdf423dac2*/

#ifndef typedef_dsp_FIRFilter_0_Controller_lu_T
#define typedef_dsp_FIRFilter_0_Controller_lu_T

typedef struct md62ec9d63a416a03de13eedbdf423dac2
  dsp_FIRFilter_0_Controller_lu_T;

#endif                                 /*typedef_dsp_FIRFilter_0_Controller_lu_T*/

#ifndef struct_mdOSc9GfCCRzvJXzRq98IzzB
#define struct_mdOSc9GfCCRzvJXzRq98IzzB

struct mdOSc9GfCCRzvJXzRq98IzzB
{
  int32_T isInitialized;
  dsp_FIRFilter_0_Controller_lu_T cSFunObject;
};

#endif                                 /*struct_mdOSc9GfCCRzvJXzRq98IzzB*/

#ifndef typedef_dspcodegen_FIRFilter_Contr_po_T
#define typedef_dspcodegen_FIRFilter_Contr_po_T

typedef struct mdOSc9GfCCRzvJXzRq98IzzB dspcodegen_FIRFilter_Contr_po_T;

#endif                                 /*typedef_dspcodegen_FIRFilter_Contr_po_T*/

#ifndef struct_mdIjdXfGjc9ClanNKBQcHJaE
#define struct_mdIjdXfGjc9ClanNKBQcHJaE

struct mdIjdXfGjc9ClanNKBQcHJaE
{
  int32_T isInitialized;
  real_T pSampleRateInherit;
  real_T NumChannels;
  dspcodegen_FIRFilter_Contr_po_T *FilterObj;
};

#endif                                 /*struct_mdIjdXfGjc9ClanNKBQcHJaE*/

#ifndef typedef_dsp_LowpassFilter_Controll_ae_T
#define typedef_dsp_LowpassFilter_Controll_ae_T

typedef struct mdIjdXfGjc9ClanNKBQcHJaE dsp_LowpassFilter_Controll_ae_T;

#endif                                 /*typedef_dsp_LowpassFilter_Controll_ae_T*/

#ifndef struct_mdd4a91d4537e9ab80636d67b45ee786e7
#define struct_mdd4a91d4537e9ab80636d67b45ee786e7

struct mdd4a91d4537e9ab80636d67b45ee786e7
{
  int32_T S0_isInitialized;
  real_T W0_states[180];
  real_T P0_InitialStates;
  real_T P1_Coefficients[181];
};

#endif                                 /*struct_mdd4a91d4537e9ab80636d67b45ee786e7*/

#ifndef typedef_dsp_FIRFilter_0_Controlle_lul_T
#define typedef_dsp_FIRFilter_0_Controlle_lul_T

typedef struct mdd4a91d4537e9ab80636d67b45ee786e7
  dsp_FIRFilter_0_Controlle_lul_T;

#endif                                 /*typedef_dsp_FIRFilter_0_Controlle_lul_T*/

#ifndef struct_mdxDF6Fg1dr9dGoXRd2FEiFG
#define struct_mdxDF6Fg1dr9dGoXRd2FEiFG

struct mdxDF6Fg1dr9dGoXRd2FEiFG
{
  int32_T isInitialized;
  dsp_FIRFilter_0_Controlle_lul_T cSFunObject;
};

#endif                                 /*struct_mdxDF6Fg1dr9dGoXRd2FEiFG*/

#ifndef typedef_dspcodegen_FIRFilter_Cont_pow_T
#define typedef_dspcodegen_FIRFilter_Cont_pow_T

typedef struct mdxDF6Fg1dr9dGoXRd2FEiFG dspcodegen_FIRFilter_Cont_pow_T;

#endif                                 /*typedef_dspcodegen_FIRFilter_Cont_pow_T*/

#ifndef struct_mdkNoG7h4bJ1nmpVsBWGPbTB
#define struct_mdkNoG7h4bJ1nmpVsBWGPbTB

struct mdkNoG7h4bJ1nmpVsBWGPbTB
{
  int32_T isInitialized;
  real_T pSampleRateInherit;
  real_T NumChannels;
  dspcodegen_FIRFilter_Cont_pow_T *FilterObj;
};

#endif                                 /*struct_mdkNoG7h4bJ1nmpVsBWGPbTB*/

#ifndef typedef_dsp_LowpassFilter_Control_aen_T
#define typedef_dsp_LowpassFilter_Control_aen_T

typedef struct mdkNoG7h4bJ1nmpVsBWGPbTB dsp_LowpassFilter_Control_aen_T;

#endif                                 /*typedef_dsp_LowpassFilter_Control_aen_T*/

/* Forward declaration for rtModel */
typedef struct tag_RTM_Controller_ABS_VLC_AY_T RT_MODEL_Controller_ABS_VLC_A_T;

#endif                                 /* RTW_HEADER_Controller_ABS_VLC_AYC_ALGO2_types_h_ */
