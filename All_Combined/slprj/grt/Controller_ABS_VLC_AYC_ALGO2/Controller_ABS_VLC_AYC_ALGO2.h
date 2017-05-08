/*
 * Code generation for system model 'Controller_ABS_VLC_AYC_ALGO2'
 * For more details, see corresponding source file Controller_ABS_VLC_AYC_ALGO2.c
 *
 */

#ifndef RTW_HEADER_Controller_ABS_VLC_AYC_ALGO2_h_
#define RTW_HEADER_Controller_ABS_VLC_AYC_ALGO2_h_
#include <math.h>
#include <string.h>
#ifndef Controller_ABS_VLC_AYC_ALGO2_COMMON_INCLUDES_
# define Controller_ABS_VLC_AYC_ALGO2_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* Controller_ABS_VLC_AYC_ALGO2_COMMON_INCLUDES_ */

#include "Controller_ABS_VLC_AYC_ALGO2_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "model_reference_types.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#include "rt_powd_snf.h"

/* Block signals for system '<S4>/Lowpass Filter1' */
typedef struct {
  real_T LowpassFilter1;               /* '<S4>/Lowpass Filter1' */
} B_LowpassFilter1_Controller_A_T;

/* Block states (auto storage) for system '<S4>/Lowpass Filter1' */
typedef struct {
  dspcodegen_FIRFilter_Controll_T gobj_0;/* '<S4>/Lowpass Filter1' */
  dspcodegen_FIRFilter_Controll_T gobj_1;/* '<S4>/Lowpass Filter1' */
  dsp_LowpassFilter_Controller__T obj; /* '<S4>/Lowpass Filter1' */
  void *LowpassFilter1_PWORK;          /* '<S4>/Lowpass Filter1' */
  boolean_T objisempty;                /* '<S4>/Lowpass Filter1' */
  boolean_T isInitialized;             /* '<S4>/Lowpass Filter1' */
} DW_LowpassFilter1_Controller__T;

/* Block signals for model 'Controller_ABS_VLC_AYC_ALGO2' */
typedef struct {
  real_T FLSlipError;                  /* '<S4>/Sum' */
  real_T DigitalClock;                 /* '<S4>/Digital Clock' */
  real_T Switch;                       /* '<S4>/Switch' */
  real_T FLSlipError_k;                /* '<S5>/Sum' */
  real_T DigitalClock_m;               /* '<S5>/Digital Clock' */
  real_T Switch_k;                     /* '<S5>/Switch' */
  real_T DigitalClock_o;               /* '<S6>/Digital Clock' */
  real_T FLSlipError_f;                /* '<S8>/Sum' */
  real_T Switch_o;                     /* '<S8>/Switch' */
  real_T DigitalClock_k;               /* '<S8>/Digital Clock' */
  real_T FLSlipError_n;                /* '<S9>/Sum' */
  real_T Switch_ka;                    /* '<S9>/Switch' */
  real_T DigitalClock_g;               /* '<S9>/Digital Clock' */
  real_T DeltaTorqueFromAYCController; /* '<S10>/Available Torque [Nm]' */
  real_T DigitalClock_l;               /* '<S10>/Digital Clock' */
  real_T LowpassFilter1_o;             /* '<S10>/Lowpass Filter1' */
  real_T LowpassFilter1_k;             /* '<S6>/Lowpass Filter1' */
  B_LowpassFilter1_Controller_A_T LowpassFilter1_pnae;/* '<S9>/Lowpass Filter1' */
  B_LowpassFilter1_Controller_A_T LowpassFilter1_pna;/* '<S8>/Lowpass Filter1' */
  B_LowpassFilter1_Controller_A_T LowpassFilter1_p;/* '<S5>/Lowpass Filter1' */
  B_LowpassFilter1_Controller_A_T LowpassFilter1;/* '<S4>/Lowpass Filter1' */
} B_Controller_ABS_VLC_AYC_AL_c_T;

/* Block states (auto storage) for model 'Controller_ABS_VLC_AYC_ALGO2' */
typedef struct {
  dspcodegen_FIRFilter_Cont_pow_T gobj_0;/* '<S10>/Lowpass Filter1' */
  dspcodegen_FIRFilter_Cont_pow_T gobj_1;/* '<S10>/Lowpass Filter1' */
  dspcodegen_FIRFilter_Contr_po_T gobj_0_j;/* '<S6>/Lowpass Filter1' */
  dspcodegen_FIRFilter_Contr_po_T gobj_1_l;/* '<S6>/Lowpass Filter1' */
  dspcodegen_FIRFilter_Contro_p_T gobj_0_jd;/* '<S1>/Lowpass Filter1' */
  dspcodegen_FIRFilter_Contro_p_T gobj_1_m;/* '<S1>/Lowpass Filter1' */
  dsp_LowpassFilter_Controll_ae_T obj; /* '<S6>/Lowpass Filter1' */
  dsp_LowpassFilter_Controlle_a_T obj_d;/* '<S1>/Lowpass Filter1' */
  dsp_LowpassFilter_Control_aen_T obj_dz;/* '<S10>/Lowpass Filter1' */
  real_T TimeStampA;                   /* '<S1>/Derivative' */
  real_T LastUAtTimeA;                 /* '<S1>/Derivative' */
  real_T TimeStampB;                   /* '<S1>/Derivative' */
  real_T LastUAtTimeB;                 /* '<S1>/Derivative' */
  real_T TimeStampA_n;                 /* '<S4>/Derivative' */
  real_T LastUAtTimeA_o;               /* '<S4>/Derivative' */
  real_T TimeStampB_c;                 /* '<S4>/Derivative' */
  real_T LastUAtTimeB_c;               /* '<S4>/Derivative' */
  real_T TimeStampA_d;                 /* '<S5>/Derivative' */
  real_T LastUAtTimeA_i;               /* '<S5>/Derivative' */
  real_T TimeStampB_o;                 /* '<S5>/Derivative' */
  real_T LastUAtTimeB_h;               /* '<S5>/Derivative' */
  real_T TimeStampA_nm;                /* '<S8>/Derivative' */
  real_T LastUAtTimeA_p;               /* '<S8>/Derivative' */
  real_T TimeStampB_a;                 /* '<S8>/Derivative' */
  real_T LastUAtTimeB_n;               /* '<S8>/Derivative' */
  real_T TimeStampA_m;                 /* '<S9>/Derivative' */
  real_T LastUAtTimeA_a;               /* '<S9>/Derivative' */
  real_T TimeStampB_m;                 /* '<S9>/Derivative' */
  real_T LastUAtTimeB_hf;              /* '<S9>/Derivative' */
  real_T TimeStampA_ml;                /* '<S10>/Derivative' */
  real_T LastUAtTimeA_l;               /* '<S10>/Derivative' */
  real_T TimeStampB_j;                 /* '<S10>/Derivative' */
  real_T LastUAtTimeB_a;               /* '<S10>/Derivative' */
  void *LowpassFilter1_PWORK;          /* '<S10>/Lowpass Filter1' */
  void *LowpassFilter1_PWORK_n;        /* '<S6>/Lowpass Filter1' */
  void *LowpassFilter1_PWORK_i;        /* '<S1>/Lowpass Filter1' */
  boolean_T objisempty;                /* '<S10>/Lowpass Filter1' */
  boolean_T isInitialized;             /* '<S10>/Lowpass Filter1' */
  boolean_T objisempty_l;              /* '<S6>/Lowpass Filter1' */
  boolean_T isInitialized_i;           /* '<S6>/Lowpass Filter1' */
  boolean_T objisempty_d;              /* '<S1>/Lowpass Filter1' */
  boolean_T isInitialized_e;           /* '<S1>/Lowpass Filter1' */
  DW_LowpassFilter1_Controller__T LowpassFilter1_pnae;/* '<S9>/Lowpass Filter1' */
  DW_LowpassFilter1_Controller__T LowpassFilter1_pna;/* '<S8>/Lowpass Filter1' */
  DW_LowpassFilter1_Controller__T LowpassFilter1_p;/* '<S5>/Lowpass Filter1' */
  DW_LowpassFilter1_Controller__T LowpassFilter1;/* '<S4>/Lowpass Filter1' */
} DW_Controller_ABS_VLC_AYC_A_f_T;

/* Real-time Model Data Structure */
struct tag_RTM_Controller_ABS_VLC_AY_T {
  const char_T **errorStatus;
  RTWSolverInfo *solverInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T stepSize0;
    SimTimeStep *simTimeStep;
  } Timing;
};

typedef struct {
  B_Controller_ABS_VLC_AYC_AL_c_T rtb;
  DW_Controller_ABS_VLC_AYC_A_f_T rtdw;
  RT_MODEL_Controller_ABS_VLC_A_T rtm;
} MdlrefDW_Controller_ABS_VLC_A_T;

/* Model reference registration function */
extern void Controller_ABS_VLC_A_initialize(const char_T **rt_errorStatus,
  RTWSolverInfo *rt_solverInfo, const rtTimingBridge *timingBridge, int_T
  mdlref_TID0, int_T mdlref_TID1, RT_MODEL_Controller_ABS_VLC_A_T *const
  Controller_ABS_VLC_AYC_ALGO2_M, B_Controller_ABS_VLC_AYC_AL_c_T *localB,
  DW_Controller_ABS_VLC_AYC_A_f_T *localDW);
extern void Controller__LowpassFilter1_Init(DW_LowpassFilter1_Controller__T
  *localDW);
extern void Controller_LowpassFilter1_Reset(DW_LowpassFilter1_Controller__T
  *localDW);
extern void Controller_LowpassFilter1_Start(DW_LowpassFilter1_Controller__T
  *localDW);
extern void Controller_ABS_V_LowpassFilter1(real_T rtu_0,
  B_LowpassFilter1_Controller_A_T *localB, DW_LowpassFilter1_Controller__T
  *localDW);
extern void Controller_ABS_VLC_AYC_ALG_Init(DW_Controller_ABS_VLC_AYC_A_f_T
  *localDW);
extern void Controller_ABS_VLC_AYC_AL_Reset(DW_Controller_ABS_VLC_AYC_A_f_T
  *localDW);
extern void Controller_ABS_VLC_AYC_AL_Start(DW_Controller_ABS_VLC_AYC_A_f_T
  *localDW);
extern void Controller_ABS_VLC_AYC_A_Update(const real_T *rtu_r_cont, real_T
  *rty_YE, B_Controller_ABS_VLC_AYC_AL_c_T *localB,
  DW_Controller_ABS_VLC_AYC_A_f_T *localDW);
extern void Controller_ABS_VLC_AYC_ALGO2(RT_MODEL_Controller_ABS_VLC_A_T * const
  Controller_ABS_VLC_AYC_ALGO2_M, const real_T *rtu_Vx_cont, const real_T
  *rtu_r_cont, const real_T *rtu_S_Ang, const real_T *rtu_FL_SR, const real_T
  *rtu_FR_SR, const real_T *rtu_RL_SR, const real_T *rtu_RR_SR, const real_T
  *rtu_T_P, const real_T *rtu_FLFzN, const real_T *rtu_FRFzN, const real_T
  *rtu_RLFzN, const real_T *rtu_RRFzN, real_T *rty_TFL, real_T *rty_TFR, real_T *
  rty_TRL, real_T *rty_TRR, real_T *rty_YE, real_T *rty_LAE,
  B_Controller_ABS_VLC_AYC_AL_c_T *localB, DW_Controller_ABS_VLC_AYC_A_f_T
  *localDW);
extern void Controller__LowpassFilter1_Term(DW_LowpassFilter1_Controller__T
  *localDW);
extern void Controller_ABS_VLC_AYC_ALG_Term(DW_Controller_ABS_VLC_AYC_A_f_T
  *localDW);

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'Controller_ABS_VLC_AYC_ALGO2'
 * '<S1>'   : 'Controller_ABS_VLC_AYC_ALGO2/ABS//VLC//AYC Control'
 * '<S2>'   : 'Controller_ABS_VLC_AYC_ALGO2/ABS//VLC//AYC Control/Async Rate Transition '
 * '<S3>'   : 'Controller_ABS_VLC_AYC_ALGO2/ABS//VLC//AYC Control/Async Rate Transition 1'
 * '<S4>'   : 'Controller_ABS_VLC_AYC_ALGO2/ABS//VLC//AYC Control/FL Slip Controller'
 * '<S5>'   : 'Controller_ABS_VLC_AYC_ALGO2/ABS//VLC//AYC Control/FR Slip Controller'
 * '<S6>'   : 'Controller_ABS_VLC_AYC_ALGO2/ABS//VLC//AYC Control/Lateral Acceleration Controller'
 * '<S7>'   : 'Controller_ABS_VLC_AYC_ALGO2/ABS//VLC//AYC Control/Lateral Acceleration Reference//Error Generator'
 * '<S8>'   : 'Controller_ABS_VLC_AYC_ALGO2/ABS//VLC//AYC Control/RL Slip Controller'
 * '<S9>'   : 'Controller_ABS_VLC_AYC_ALGO2/ABS//VLC//AYC Control/RR Slip Controller'
 * '<S10>'  : 'Controller_ABS_VLC_AYC_ALGO2/ABS//VLC//AYC Control/Yaw Controller'
 * '<S11>'  : 'Controller_ABS_VLC_AYC_ALGO2/ABS//VLC//AYC Control/Yaw Reference//Error Generator'
 * '<S12>'  : 'Controller_ABS_VLC_AYC_ALGO2/ABS//VLC//AYC Control/FL Slip Controller/Async Rate Transition '
 * '<S13>'  : 'Controller_ABS_VLC_AYC_ALGO2/ABS//VLC//AYC Control/FL Slip Controller/Async Rate Transition 1'
 * '<S14>'  : 'Controller_ABS_VLC_AYC_ALGO2/ABS//VLC//AYC Control/FR Slip Controller/Async Rate Transition '
 * '<S15>'  : 'Controller_ABS_VLC_AYC_ALGO2/ABS//VLC//AYC Control/FR Slip Controller/Async Rate Transition 1'
 * '<S16>'  : 'Controller_ABS_VLC_AYC_ALGO2/ABS//VLC//AYC Control/Lateral Acceleration Controller/Async Rate Transition '
 * '<S17>'  : 'Controller_ABS_VLC_AYC_ALGO2/ABS//VLC//AYC Control/Lateral Acceleration Controller/Async Rate Transition 1'
 * '<S18>'  : 'Controller_ABS_VLC_AYC_ALGO2/ABS//VLC//AYC Control/RL Slip Controller/Async Rate Transition '
 * '<S19>'  : 'Controller_ABS_VLC_AYC_ALGO2/ABS//VLC//AYC Control/RL Slip Controller/Async Rate Transition 1'
 * '<S20>'  : 'Controller_ABS_VLC_AYC_ALGO2/ABS//VLC//AYC Control/RR Slip Controller/Async Rate Transition '
 * '<S21>'  : 'Controller_ABS_VLC_AYC_ALGO2/ABS//VLC//AYC Control/RR Slip Controller/Async Rate Transition 1'
 * '<S22>'  : 'Controller_ABS_VLC_AYC_ALGO2/ABS//VLC//AYC Control/Yaw Controller/Async Rate Transition '
 * '<S23>'  : 'Controller_ABS_VLC_AYC_ALGO2/ABS//VLC//AYC Control/Yaw Controller/Async Rate Transition 1'
 */
#endif                                 /* RTW_HEADER_Controller_ABS_VLC_AYC_ALGO2_h_ */
