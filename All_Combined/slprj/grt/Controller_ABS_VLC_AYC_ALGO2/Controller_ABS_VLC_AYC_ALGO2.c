/*
 * Code generation for system model 'Controller_ABS_VLC_AYC_ALGO2'
 *
 * Model                      : Controller_ABS_VLC_AYC_ALGO2
 * Model version              : 1.288
 * Simulink Coder version : 8.10 (R2016a) 10-Feb-2016
 * C source code generated on : Sat May 06 23:44:30 2017
 *
 * Note that the functions contained in this file are part of a Simulink
 * model, and are not self-contained algorithms.
 */

#include "Controller_ABS_VLC_AYC_ALGO2.h"
#include "Controller_ABS_VLC_AYC_ALGO2_private.h"

int_T Controller_ABS_VLC_AY_GlobalTID[2];
const rtTimingBridge *Controller_ABS_VLC_A_TimingBrdg;

/*
 * System initialize for atomic system:
 *    'synthesized block'
 *    'synthesized block'
 *    'synthesized block'
 *    'synthesized block'
 */
void Controller__LowpassFilter1_Init(DW_LowpassFilter1_Controller__T *localDW)
{
  real_T tmp;
  int32_T i;

  /* Start for MATLABSystem: '<S4>/Lowpass Filter1' incorporates:
   *  InitializeConditions for MATLABSystem: '<S4>/Lowpass Filter1'
   */
  if ((localDW->obj.isInitialized == 1) && (localDW->
       obj.FilterObj->isInitialized == 1)) {
    /* System object Initialization function: dsp.FIRFilter */
    tmp = localDW->obj.FilterObj->cSFunObject.P0_InitialStates;
    for (i = 0; i < 69; i++) {
      localDW->obj.FilterObj->cSFunObject.W0_states[i] = tmp;
    }
  }

  /* End of Start for MATLABSystem: '<S4>/Lowpass Filter1' */
}

/*
 * System reset for atomic system:
 *    'synthesized block'
 *    'synthesized block'
 *    'synthesized block'
 *    'synthesized block'
 */
void Controller_LowpassFilter1_Reset(DW_LowpassFilter1_Controller__T *localDW)
{
  real_T tmp;
  int32_T i;

  /* Start for MATLABSystem: '<S4>/Lowpass Filter1' incorporates:
   *  InitializeConditions for MATLABSystem: '<S4>/Lowpass Filter1'
   */
  if ((localDW->obj.isInitialized == 1) && (localDW->
       obj.FilterObj->isInitialized == 1)) {
    /* System object Initialization function: dsp.FIRFilter */
    tmp = localDW->obj.FilterObj->cSFunObject.P0_InitialStates;
    for (i = 0; i < 69; i++) {
      localDW->obj.FilterObj->cSFunObject.W0_states[i] = tmp;
    }
  }

  /* End of Start for MATLABSystem: '<S4>/Lowpass Filter1' */
}

/*
 * Start for atomic system:
 *    'synthesized block'
 *    'synthesized block'
 *    'synthesized block'
 *    'synthesized block'
 */
void Controller_LowpassFilter1_Start(DW_LowpassFilter1_Controller__T *localDW)
{
  dspcodegen_FIRFilter_Controll_T *iobj_0;
  int32_T i;
  static const real_T tmp[70] = { -0.00020947958686481327,
    -0.00050938708003221364, -0.00083730038415124307, -0.0009324609591446311,
    -0.000527774325420364, 0.00045614652325841071, 0.0017324774168993458,
    0.0026529392283882337, 0.0024855300326423351, 0.0008864669336692691,
    -0.0016934342472111731, -0.0039872553244412762, -0.004443333455144886,
    -0.00214908325154883, 0.0023451875696967805, 0.0068931886477229809,
    0.0086267036100513353, 0.0055771120826348776, -0.001787820196672016,
    -0.010132770422839962, -0.014522967902406335, -0.011062913749590578,
    0.0003745812186515215, 0.014994823123493695, 0.024620024339669207,
    0.02168692152184374, 0.0039663605348216916, -0.022523249594437071,
    -0.044430578490265743, -0.0461020999760876, -0.016850072697670436,
    0.042889271059264288, 0.11972622096192787, 0.19122575483518353,
    0.23425916909699984, 0.23425916909699984, 0.19122575483518353,
    0.11972622096192787, 0.042889271059264288, -0.016850072697670436,
    -0.0461020999760876, -0.044430578490265743, -0.022523249594437071,
    0.0039663605348216916, 0.02168692152184374, 0.024620024339669207,
    0.014994823123493695, 0.0003745812186515215, -0.011062913749590578,
    -0.014522967902406335, -0.010132770422839962, -0.001787820196672016,
    0.0055771120826348776, 0.0086267036100513353, 0.0068931886477229809,
    0.0023451875696967805, -0.00214908325154883, -0.004443333455144886,
    -0.0039872553244412762, -0.0016934342472111731, 0.0008864669336692691,
    0.0024855300326423351, 0.0026529392283882337, 0.0017324774168993458,
    0.00045614652325841071, -0.000527774325420364, -0.0009324609591446311,
    -0.00083730038415124307, -0.00050938708003221364, -0.00020947958686481327 };

  static const real_T tmp_0[70] = { -0.00020947958686481327,
    -0.00050938708003221364, -0.00083730038415124307, -0.0009324609591446311,
    -0.000527774325420364, 0.00045614652325841071, 0.0017324774168993458,
    0.0026529392283882337, 0.0024855300326423351, 0.0008864669336692691,
    -0.0016934342472111731, -0.0039872553244412762, -0.004443333455144886,
    -0.00214908325154883, 0.0023451875696967805, 0.0068931886477229809,
    0.0086267036100513353, 0.0055771120826348776, -0.001787820196672016,
    -0.010132770422839962, -0.014522967902406335, -0.011062913749590578,
    0.0003745812186515215, 0.014994823123493695, 0.024620024339669207,
    0.02168692152184374, 0.0039663605348216916, -0.022523249594437071,
    -0.044430578490265743, -0.0461020999760876, -0.016850072697670436,
    0.042889271059264288, 0.11972622096192787, 0.19122575483518353,
    0.23425916909699984, 0.23425916909699984, 0.19122575483518353,
    0.11972622096192787, 0.042889271059264288, -0.016850072697670436,
    -0.0461020999760876, -0.044430578490265743, -0.022523249594437071,
    0.0039663605348216916, 0.02168692152184374, 0.024620024339669207,
    0.014994823123493695, 0.0003745812186515215, -0.011062913749590578,
    -0.014522967902406335, -0.010132770422839962, -0.001787820196672016,
    0.0055771120826348776, 0.0086267036100513353, 0.0068931886477229809,
    0.0023451875696967805, -0.00214908325154883, -0.004443333455144886,
    -0.0039872553244412762, -0.0016934342472111731, 0.0008864669336692691,
    0.0024855300326423351, 0.0026529392283882337, 0.0017324774168993458,
    0.00045614652325841071, -0.000527774325420364, -0.0009324609591446311,
    -0.00083730038415124307, -0.00050938708003221364, -0.00020947958686481327 };

  /* Start for MATLABSystem: '<S4>/Lowpass Filter1' */
  localDW->obj.pSampleRateInherit = -1.0;
  localDW->obj.isInitialized = 0;
  localDW->obj.NumChannels = -1.0;
  localDW->objisempty = true;
  iobj_0 = &localDW->gobj_0;
  localDW->obj.isInitialized = 1;
  localDW->obj.pSampleRateInherit = 1000.0;
  localDW->gobj_0.isInitialized = 0;

  /* System object Constructor function: dsp.FIRFilter */
  iobj_0->cSFunObject.P0_InitialStates = 0.0;
  for (i = 0; i < 70; i++) {
    iobj_0->cSFunObject.P1_Coefficients[i] = tmp_0[i];
  }

  for (i = 0; i < 70; i++) {
    iobj_0->cSFunObject.P1_Coefficients[i] = tmp[i];
  }

  localDW->obj.FilterObj = &localDW->gobj_0;
  localDW->obj.NumChannels = 1.0;

  /* End of Start for MATLABSystem: '<S4>/Lowpass Filter1' */
}

/*
 * Output and update for atomic system:
 *    'synthesized block'
 *    'synthesized block'
 *    'synthesized block'
 *    'synthesized block'
 */
void Controller_ABS_V_LowpassFilter1(real_T rtu_0,
  B_LowpassFilter1_Controller_A_T *localB, DW_LowpassFilter1_Controller__T
  *localDW)
{
  dsp_FIRFilter_0_Controller_AB_T *obj;
  real_T acc1;
  real_T acc2;
  int32_T j;

  /* Start for MATLABSystem: '<S4>/Lowpass Filter1' incorporates:
   *  MATLABSystem: '<S4>/Lowpass Filter1'
   */
  if (localDW->obj.FilterObj->isInitialized != 1) {
    localDW->obj.FilterObj->isInitialized = 1;

    /* System object Initialization function: dsp.FIRFilter */
    acc1 = localDW->obj.FilterObj->cSFunObject.P0_InitialStates;
    for (j = 0; j < 69; j++) {
      localDW->obj.FilterObj->cSFunObject.W0_states[j] = acc1;
    }
  }

  obj = &localDW->obj.FilterObj->cSFunObject;

  /* System object Outputs function: dsp.FIRFilter */
  /* Consume delay line and beginning of input samples */
  acc1 = 0.0;
  j = 0;
  while (j < 1) {
    acc2 = rtu_0 * obj->P1_Coefficients[0];
    acc1 += acc2;
    j = 1;
  }

  for (j = 0; j < 69; j++) {
    acc2 = obj->P1_Coefficients[1 + j] * obj->W0_states[j];
    acc1 += acc2;
  }

  /* Update delay line for next frame */
  for (j = 67; j >= 0; j--) {
    obj->W0_states[1 + j] = obj->W0_states[j];
  }

  localDW->obj.FilterObj->cSFunObject.W0_states[0] = rtu_0;

  /* End of Start for MATLABSystem: '<S4>/Lowpass Filter1' */

  /* MATLABSystem: '<S4>/Lowpass Filter1' */
  localB->LowpassFilter1 = acc1;
}

/*
 * Termination for atomic system:
 *    'synthesized block'
 *    'synthesized block'
 *    'synthesized block'
 *    'synthesized block'
 */
void Controller__LowpassFilter1_Term(DW_LowpassFilter1_Controller__T *localDW)
{
  /* Start for MATLABSystem: '<S4>/Lowpass Filter1' incorporates:
   *  Terminate for MATLABSystem: '<S4>/Lowpass Filter1'
   */
  if (localDW->obj.isInitialized == 1) {
    localDW->obj.isInitialized = 2;
    if (localDW->obj.FilterObj->isInitialized == 1) {
      localDW->obj.FilterObj->isInitialized = 2;
    }

    localDW->obj.NumChannels = -1.0;
  }

  /* End of Start for MATLABSystem: '<S4>/Lowpass Filter1' */
}

/* System initialize for referenced model: 'Controller_ABS_VLC_AYC_ALGO2' */
void Controller_ABS_VLC_AYC_ALG_Init(DW_Controller_ABS_VLC_AYC_A_f_T *localDW)
{
  real_T tmp;
  int32_T i;

  /* InitializeConditions for Derivative: '<S1>/Derivative' */
  localDW->TimeStampA = (rtInf);
  localDW->TimeStampB = (rtInf);

  /* InitializeConditions for Derivative: '<S4>/Derivative' */
  localDW->TimeStampA_n = (rtInf);
  localDW->TimeStampB_c = (rtInf);

  /* InitializeConditions for Derivative: '<S5>/Derivative' */
  localDW->TimeStampA_d = (rtInf);
  localDW->TimeStampB_o = (rtInf);

  /* InitializeConditions for Derivative: '<S8>/Derivative' */
  localDW->TimeStampA_nm = (rtInf);
  localDW->TimeStampB_a = (rtInf);

  /* InitializeConditions for Derivative: '<S9>/Derivative' */
  localDW->TimeStampA_m = (rtInf);
  localDW->TimeStampB_m = (rtInf);

  /* InitializeConditions for Derivative: '<S10>/Derivative' */
  localDW->TimeStampA_ml = (rtInf);
  localDW->TimeStampB_j = (rtInf);
  Controller__LowpassFilter1_Init(&localDW->LowpassFilter1);
  Controller__LowpassFilter1_Init(&localDW->LowpassFilter1_p);

  /* Start for MATLABSystem: '<S6>/Lowpass Filter1' incorporates:
   *  InitializeConditions for MATLABSystem: '<S6>/Lowpass Filter1'
   */
  if ((localDW->obj.isInitialized == 1) && (localDW->
       obj.FilterObj->isInitialized == 1)) {
    /* System object Initialization function: dsp.FIRFilter */
    tmp = localDW->obj.FilterObj->cSFunObject.P0_InitialStates;
    for (i = 0; i < 116; i++) {
      localDW->obj.FilterObj->cSFunObject.W0_states[i] = tmp;
    }
  }

  /* End of Start for MATLABSystem: '<S6>/Lowpass Filter1' */
  Controller__LowpassFilter1_Init(&localDW->LowpassFilter1_pna);
  Controller__LowpassFilter1_Init(&localDW->LowpassFilter1_pnae);

  /* Start for MATLABSystem: '<S10>/Lowpass Filter1' incorporates:
   *  InitializeConditions for MATLABSystem: '<S10>/Lowpass Filter1'
   */
  if ((localDW->obj_dz.isInitialized == 1) && (localDW->
       obj_dz.FilterObj->isInitialized == 1)) {
    /* System object Initialization function: dsp.FIRFilter */
    tmp = localDW->obj_dz.FilterObj->cSFunObject.P0_InitialStates;
    for (i = 0; i < 180; i++) {
      localDW->obj_dz.FilterObj->cSFunObject.W0_states[i] = tmp;
    }
  }

  /* End of Start for MATLABSystem: '<S10>/Lowpass Filter1' */

  /* Start for MATLABSystem: '<S1>/Lowpass Filter1' incorporates:
   *  InitializeConditions for MATLABSystem: '<S1>/Lowpass Filter1'
   */
  if ((localDW->obj_d.isInitialized == 1) && (localDW->
       obj_d.FilterObj->isInitialized == 1)) {
    /* System object Initialization function: dsp.FIRFilter */
    tmp = localDW->obj_d.FilterObj->cSFunObject.P0_InitialStates;
    for (i = 0; i < 25; i++) {
      localDW->obj_d.FilterObj->cSFunObject.W0_states[i] = tmp;
    }
  }

  /* End of Start for MATLABSystem: '<S1>/Lowpass Filter1' */
}

/* System reset for referenced model: 'Controller_ABS_VLC_AYC_ALGO2' */
void Controller_ABS_VLC_AYC_AL_Reset(DW_Controller_ABS_VLC_AYC_A_f_T *localDW)
{
  real_T tmp;
  int32_T i;

  /* InitializeConditions for Derivative: '<S1>/Derivative' */
  localDW->TimeStampA = (rtInf);
  localDW->TimeStampB = (rtInf);

  /* InitializeConditions for Derivative: '<S4>/Derivative' */
  localDW->TimeStampA_n = (rtInf);
  localDW->TimeStampB_c = (rtInf);

  /* InitializeConditions for Derivative: '<S5>/Derivative' */
  localDW->TimeStampA_d = (rtInf);
  localDW->TimeStampB_o = (rtInf);

  /* InitializeConditions for Derivative: '<S8>/Derivative' */
  localDW->TimeStampA_nm = (rtInf);
  localDW->TimeStampB_a = (rtInf);

  /* InitializeConditions for Derivative: '<S9>/Derivative' */
  localDW->TimeStampA_m = (rtInf);
  localDW->TimeStampB_m = (rtInf);

  /* InitializeConditions for Derivative: '<S10>/Derivative' */
  localDW->TimeStampA_ml = (rtInf);
  localDW->TimeStampB_j = (rtInf);
  Controller_LowpassFilter1_Reset(&localDW->LowpassFilter1);
  Controller_LowpassFilter1_Reset(&localDW->LowpassFilter1_p);

  /* Start for MATLABSystem: '<S6>/Lowpass Filter1' incorporates:
   *  InitializeConditions for MATLABSystem: '<S6>/Lowpass Filter1'
   */
  if ((localDW->obj.isInitialized == 1) && (localDW->
       obj.FilterObj->isInitialized == 1)) {
    /* System object Initialization function: dsp.FIRFilter */
    tmp = localDW->obj.FilterObj->cSFunObject.P0_InitialStates;
    for (i = 0; i < 116; i++) {
      localDW->obj.FilterObj->cSFunObject.W0_states[i] = tmp;
    }
  }

  /* End of Start for MATLABSystem: '<S6>/Lowpass Filter1' */
  Controller_LowpassFilter1_Reset(&localDW->LowpassFilter1_pna);
  Controller_LowpassFilter1_Reset(&localDW->LowpassFilter1_pnae);

  /* Start for MATLABSystem: '<S10>/Lowpass Filter1' incorporates:
   *  InitializeConditions for MATLABSystem: '<S10>/Lowpass Filter1'
   */
  if ((localDW->obj_dz.isInitialized == 1) && (localDW->
       obj_dz.FilterObj->isInitialized == 1)) {
    /* System object Initialization function: dsp.FIRFilter */
    tmp = localDW->obj_dz.FilterObj->cSFunObject.P0_InitialStates;
    for (i = 0; i < 180; i++) {
      localDW->obj_dz.FilterObj->cSFunObject.W0_states[i] = tmp;
    }
  }

  /* End of Start for MATLABSystem: '<S10>/Lowpass Filter1' */

  /* Start for MATLABSystem: '<S1>/Lowpass Filter1' incorporates:
   *  InitializeConditions for MATLABSystem: '<S1>/Lowpass Filter1'
   */
  if ((localDW->obj_d.isInitialized == 1) && (localDW->
       obj_d.FilterObj->isInitialized == 1)) {
    /* System object Initialization function: dsp.FIRFilter */
    tmp = localDW->obj_d.FilterObj->cSFunObject.P0_InitialStates;
    for (i = 0; i < 25; i++) {
      localDW->obj_d.FilterObj->cSFunObject.W0_states[i] = tmp;
    }
  }

  /* End of Start for MATLABSystem: '<S1>/Lowpass Filter1' */
}

/* Start for referenced model: 'Controller_ABS_VLC_AYC_ALGO2' */
void Controller_ABS_VLC_AYC_AL_Start(DW_Controller_ABS_VLC_AYC_A_f_T *localDW)
{
  dspcodegen_FIRFilter_Contr_po_T *iobj_0;
  dspcodegen_FIRFilter_Cont_pow_T *iobj_0_0;
  dspcodegen_FIRFilter_Contro_p_T *iobj_0_1;
  int32_T i;
  static const real_T tmp[117] = { 7.386635898848976E-5, 2.0045263525107591E-5,
    -0.000100743749299954, -0.00033903627132568466, -0.0006553668957550063,
    -0.00094161217151976376, -0.0010492927684376467, -0.00085511683313927789,
    -0.00033940010534460116, 0.00036449531057297408, 0.00098843992997010659,
    0.0012280599569835823, 0.00089076415386003176, 2.6136698982016109E-5,
    -0.001038116156248807, -0.0017991604681135431, -0.0018006653671993954,
    -0.00088149332356594141, 0.00067560504016334023, 0.0021975592129299558,
    0.0028923582404687142, 0.0022398253917829828, 0.0003175076574093704,
    -0.0021232666862976667, -0.0039208154929948735, -0.0040322787940498192,
    -0.0020979126831190374, 0.0012479414293160127, 0.0045297080097873978,
    0.0060482167846317141, 0.00472218647020527, 0.00075321792689438008,
    -0.0042613172855576736, -0.00791817401013393, -0.0081123931357878643,
    -0.0041698538937716091, 0.002569683857392595, 0.0091074313629209633,
    0.012047447395068363, 0.0092680286496410243, 0.0012184861202350461,
    -0.0088749287819593278, -0.016173774858642808, -0.016400742529104489,
    -0.0081031366015003447, 0.0060986807997789917, 0.020057685159755183,
    0.026476110416459014, 0.020280307721021469, 0.0015755021035574464,
    -0.023244666504376395, -0.043013922400577345, -0.045644542860781331,
    -0.022969429701025935, 0.0253381376832623, 0.090474147994072773,
    0.15660232704982435, 0.20577337234502036, 0.22393178340536932,
    0.20577337234502036, 0.15660232704982435, 0.090474147994072773,
    0.0253381376832623, -0.022969429701025935, -0.045644542860781331,
    -0.043013922400577345, -0.023244666504376395, 0.0015755021035574464,
    0.020280307721021469, 0.026476110416459014, 0.020057685159755183,
    0.0060986807997789917, -0.0081031366015003447, -0.016400742529104489,
    -0.016173774858642808, -0.0088749287819593278, 0.0012184861202350461,
    0.0092680286496410243, 0.012047447395068363, 0.0091074313629209633,
    0.002569683857392595, -0.0041698538937716091, -0.0081123931357878643,
    -0.00791817401013393, -0.0042613172855576736, 0.00075321792689438008,
    0.00472218647020527, 0.0060482167846317141, 0.0045297080097873978,
    0.0012479414293160127, -0.0020979126831190374, -0.0040322787940498192,
    -0.0039208154929948735, -0.0021232666862976667, 0.0003175076574093704,
    0.0022398253917829828, 0.0028923582404687142, 0.0021975592129299558,
    0.00067560504016334023, -0.00088149332356594141, -0.0018006653671993954,
    -0.0017991604681135431, -0.001038116156248807, 2.6136698982016109E-5,
    0.00089076415386003176, 0.0012280599569835823, 0.00098843992997010659,
    0.00036449531057297408, -0.00033940010534460116, -0.00085511683313927789,
    -0.0010492927684376467, -0.00094161217151976376, -0.0006553668957550063,
    -0.00033903627132568466, -0.000100743749299954, 2.0045263525107591E-5,
    7.386635898848976E-5 };

  static const real_T tmp_0[181] = { 7.1304380923446032E-5,
    5.7098933374758445E-5, 7.7436913907066447E-5, 0.00010028481292981024,
    0.00012496233628925989, 0.00015054913635705378, 0.0001758116268392333,
    0.00019927957335407696, 0.00021921638416897869, 0.00023371820107284685,
    0.00024071320664146036, 0.00023811411848902473, 0.00022383399901438949,
    0.00019597097436338176, 0.00015285802723030522, 9.3258619724152889E-5,
    1.6422350531876982E-5, -7.77223329305013E-5, -0.00018854053846359119,
    -0.00031452329947580753, -0.000453291791856472, -0.00060149736382054605,
    -0.00075490316901425823, -0.0009083509068270846, -0.0010559311451048196,
    -0.0011910558921389659, -0.0013067086236574397, -0.0013956019228707787,
    -0.001450529822413569, -0.0014645911612990404, -0.0014315830364135492,
    -0.0013462813651420123, -0.0012048282824414604, -0.0010050001429179525,
    -0.00074654122966514344, -0.00043144228604227318, -6.3914096937219236E-5,
    0.00034901294679453779, 0.0007981893614315502, 0.0012716793255070061,
    0.0017553330123004456, 0.0022331331302626813, 0.002687300554446586,
    0.0030989029164921089, 0.0034484843327408071, 0.0037167259603726084,
    0.0038850395173103155, 0.0039364179458849964, 0.0038561843094769622,
    0.0036328282284960938, 0.0032586324029172968, 0.002730379987169592,
    0.0020498623803806098, 0.0012244099114135257, 0.00026711264286924955,
    -0.00080300488197500839, -0.0019611362690186494, -0.0031768878684394846,
    -0.0044147987782467155, -0.00563492726638966, -0.0067937881446493353,
    -0.0078453203793012547, -0.0087421322022481554, -0.009436743599063227,
    -0.0098830472043438342, -0.010037650996711937, -0.009861380068217955,
    -0.0093206252124639968, -0.0083886894987317256, -0.007046903962718858,
    -0.005285680357344287, -0.0031052325185597436, -0.00051607663744762449,
    0.0024606230005759381, 0.0057932610380944452, 0.0094399777490120716,
    0.013349395299726521, 0.017461593452973941, 0.021709232831409953,
    0.026019117668470756, 0.0303138173727609, 0.034513508415151363,
    0.038537870592217922, 0.042308176910426987, 0.045749199612727651,
    0.048791198764498743, 0.05137168371708313, 0.053437130566602153,
    0.054944325579732591, 0.055861577639457136, 0.05616949500096801,
    0.055861577639457136, 0.054944325579732591, 0.053437130566602153,
    0.05137168371708313, 0.048791198764498743, 0.045749199612727651,
    0.042308176910426987, 0.038537870592217922, 0.034513508415151363,
    0.0303138173727609, 0.026019117668470756, 0.021709232831409953,
    0.017461593452973941, 0.013349395299726521, 0.0094399777490120716,
    0.0057932610380944452, 0.0024606230005759381, -0.00051607663744762449,
    -0.0031052325185597436, -0.005285680357344287, -0.007046903962718858,
    -0.0083886894987317256, -0.0093206252124639968, -0.009861380068217955,
    -0.010037650996711937, -0.0098830472043438342, -0.009436743599063227,
    -0.0087421322022481554, -0.0078453203793012547, -0.0067937881446493353,
    -0.00563492726638966, -0.0044147987782467155, -0.0031768878684394846,
    -0.0019611362690186494, -0.00080300488197500839, 0.00026711264286924955,
    0.0012244099114135257, 0.0020498623803806098, 0.002730379987169592,
    0.0032586324029172968, 0.0036328282284960938, 0.0038561843094769622,
    0.0039364179458849964, 0.0038850395173103155, 0.0037167259603726084,
    0.0034484843327408071, 0.0030989029164921089, 0.002687300554446586,
    0.0022331331302626813, 0.0017553330123004456, 0.0012716793255070061,
    0.0007981893614315502, 0.00034901294679453779, -6.3914096937219236E-5,
    -0.00043144228604227318, -0.00074654122966514344, -0.0010050001429179525,
    -0.0012048282824414604, -0.0013462813651420123, -0.0014315830364135492,
    -0.0014645911612990404, -0.001450529822413569, -0.0013956019228707787,
    -0.0013067086236574397, -0.0011910558921389659, -0.0010559311451048196,
    -0.0009083509068270846, -0.00075490316901425823, -0.00060149736382054605,
    -0.000453291791856472, -0.00031452329947580753, -0.00018854053846359119,
    -7.77223329305013E-5, 1.6422350531876982E-5, 9.3258619724152889E-5,
    0.00015285802723030522, 0.00019597097436338176, 0.00022383399901438949,
    0.00023811411848902473, 0.00024071320664146036, 0.00023371820107284685,
    0.00021921638416897869, 0.00019927957335407696, 0.0001758116268392333,
    0.00015054913635705378, 0.00012496233628925989, 0.00010028481292981024,
    7.7436913907066447E-5, 5.7098933374758445E-5, 7.1304380923446032E-5 };

  static const real_T tmp_1[26] = { -0.0039367421859610174,
    0.0065960362718635391, -0.010866421259157469, 0.015523621567767366,
    -0.019570526864945238, 0.02160928335361819, -0.019889119395638935,
    0.012347577763789114, 0.0035611266878216724, -0.03172981218558911,
    0.080863640626527622, -0.18253075126518334, 0.62641358074609388,
    0.62641358074609388, -0.18253075126518334, 0.080863640626527622,
    -0.03172981218558911, 0.0035611266878216724, 0.012347577763789114,
    -0.019889119395638935, 0.02160928335361819, -0.019570526864945238,
    0.015523621567767366, -0.010866421259157469, 0.0065960362718635391,
    -0.0039367421859610174 };

  static const real_T tmp_2[117] = { 7.386635898848976E-5, 2.0045263525107591E-5,
    -0.000100743749299954, -0.00033903627132568466, -0.0006553668957550063,
    -0.00094161217151976376, -0.0010492927684376467, -0.00085511683313927789,
    -0.00033940010534460116, 0.00036449531057297408, 0.00098843992997010659,
    0.0012280599569835823, 0.00089076415386003176, 2.6136698982016109E-5,
    -0.001038116156248807, -0.0017991604681135431, -0.0018006653671993954,
    -0.00088149332356594141, 0.00067560504016334023, 0.0021975592129299558,
    0.0028923582404687142, 0.0022398253917829828, 0.0003175076574093704,
    -0.0021232666862976667, -0.0039208154929948735, -0.0040322787940498192,
    -0.0020979126831190374, 0.0012479414293160127, 0.0045297080097873978,
    0.0060482167846317141, 0.00472218647020527, 0.00075321792689438008,
    -0.0042613172855576736, -0.00791817401013393, -0.0081123931357878643,
    -0.0041698538937716091, 0.002569683857392595, 0.0091074313629209633,
    0.012047447395068363, 0.0092680286496410243, 0.0012184861202350461,
    -0.0088749287819593278, -0.016173774858642808, -0.016400742529104489,
    -0.0081031366015003447, 0.0060986807997789917, 0.020057685159755183,
    0.026476110416459014, 0.020280307721021469, 0.0015755021035574464,
    -0.023244666504376395, -0.043013922400577345, -0.045644542860781331,
    -0.022969429701025935, 0.0253381376832623, 0.090474147994072773,
    0.15660232704982435, 0.20577337234502036, 0.22393178340536932,
    0.20577337234502036, 0.15660232704982435, 0.090474147994072773,
    0.0253381376832623, -0.022969429701025935, -0.045644542860781331,
    -0.043013922400577345, -0.023244666504376395, 0.0015755021035574464,
    0.020280307721021469, 0.026476110416459014, 0.020057685159755183,
    0.0060986807997789917, -0.0081031366015003447, -0.016400742529104489,
    -0.016173774858642808, -0.0088749287819593278, 0.0012184861202350461,
    0.0092680286496410243, 0.012047447395068363, 0.0091074313629209633,
    0.002569683857392595, -0.0041698538937716091, -0.0081123931357878643,
    -0.00791817401013393, -0.0042613172855576736, 0.00075321792689438008,
    0.00472218647020527, 0.0060482167846317141, 0.0045297080097873978,
    0.0012479414293160127, -0.0020979126831190374, -0.0040322787940498192,
    -0.0039208154929948735, -0.0021232666862976667, 0.0003175076574093704,
    0.0022398253917829828, 0.0028923582404687142, 0.0021975592129299558,
    0.00067560504016334023, -0.00088149332356594141, -0.0018006653671993954,
    -0.0017991604681135431, -0.001038116156248807, 2.6136698982016109E-5,
    0.00089076415386003176, 0.0012280599569835823, 0.00098843992997010659,
    0.00036449531057297408, -0.00033940010534460116, -0.00085511683313927789,
    -0.0010492927684376467, -0.00094161217151976376, -0.0006553668957550063,
    -0.00033903627132568466, -0.000100743749299954, 2.0045263525107591E-5,
    7.386635898848976E-5 };

  static const real_T tmp_3[181] = { 7.1304380923446032E-5,
    5.7098933374758445E-5, 7.7436913907066447E-5, 0.00010028481292981024,
    0.00012496233628925989, 0.00015054913635705378, 0.0001758116268392333,
    0.00019927957335407696, 0.00021921638416897869, 0.00023371820107284685,
    0.00024071320664146036, 0.00023811411848902473, 0.00022383399901438949,
    0.00019597097436338176, 0.00015285802723030522, 9.3258619724152889E-5,
    1.6422350531876982E-5, -7.77223329305013E-5, -0.00018854053846359119,
    -0.00031452329947580753, -0.000453291791856472, -0.00060149736382054605,
    -0.00075490316901425823, -0.0009083509068270846, -0.0010559311451048196,
    -0.0011910558921389659, -0.0013067086236574397, -0.0013956019228707787,
    -0.001450529822413569, -0.0014645911612990404, -0.0014315830364135492,
    -0.0013462813651420123, -0.0012048282824414604, -0.0010050001429179525,
    -0.00074654122966514344, -0.00043144228604227318, -6.3914096937219236E-5,
    0.00034901294679453779, 0.0007981893614315502, 0.0012716793255070061,
    0.0017553330123004456, 0.0022331331302626813, 0.002687300554446586,
    0.0030989029164921089, 0.0034484843327408071, 0.0037167259603726084,
    0.0038850395173103155, 0.0039364179458849964, 0.0038561843094769622,
    0.0036328282284960938, 0.0032586324029172968, 0.002730379987169592,
    0.0020498623803806098, 0.0012244099114135257, 0.00026711264286924955,
    -0.00080300488197500839, -0.0019611362690186494, -0.0031768878684394846,
    -0.0044147987782467155, -0.00563492726638966, -0.0067937881446493353,
    -0.0078453203793012547, -0.0087421322022481554, -0.009436743599063227,
    -0.0098830472043438342, -0.010037650996711937, -0.009861380068217955,
    -0.0093206252124639968, -0.0083886894987317256, -0.007046903962718858,
    -0.005285680357344287, -0.0031052325185597436, -0.00051607663744762449,
    0.0024606230005759381, 0.0057932610380944452, 0.0094399777490120716,
    0.013349395299726521, 0.017461593452973941, 0.021709232831409953,
    0.026019117668470756, 0.0303138173727609, 0.034513508415151363,
    0.038537870592217922, 0.042308176910426987, 0.045749199612727651,
    0.048791198764498743, 0.05137168371708313, 0.053437130566602153,
    0.054944325579732591, 0.055861577639457136, 0.05616949500096801,
    0.055861577639457136, 0.054944325579732591, 0.053437130566602153,
    0.05137168371708313, 0.048791198764498743, 0.045749199612727651,
    0.042308176910426987, 0.038537870592217922, 0.034513508415151363,
    0.0303138173727609, 0.026019117668470756, 0.021709232831409953,
    0.017461593452973941, 0.013349395299726521, 0.0094399777490120716,
    0.0057932610380944452, 0.0024606230005759381, -0.00051607663744762449,
    -0.0031052325185597436, -0.005285680357344287, -0.007046903962718858,
    -0.0083886894987317256, -0.0093206252124639968, -0.009861380068217955,
    -0.010037650996711937, -0.0098830472043438342, -0.009436743599063227,
    -0.0087421322022481554, -0.0078453203793012547, -0.0067937881446493353,
    -0.00563492726638966, -0.0044147987782467155, -0.0031768878684394846,
    -0.0019611362690186494, -0.00080300488197500839, 0.00026711264286924955,
    0.0012244099114135257, 0.0020498623803806098, 0.002730379987169592,
    0.0032586324029172968, 0.0036328282284960938, 0.0038561843094769622,
    0.0039364179458849964, 0.0038850395173103155, 0.0037167259603726084,
    0.0034484843327408071, 0.0030989029164921089, 0.002687300554446586,
    0.0022331331302626813, 0.0017553330123004456, 0.0012716793255070061,
    0.0007981893614315502, 0.00034901294679453779, -6.3914096937219236E-5,
    -0.00043144228604227318, -0.00074654122966514344, -0.0010050001429179525,
    -0.0012048282824414604, -0.0013462813651420123, -0.0014315830364135492,
    -0.0014645911612990404, -0.001450529822413569, -0.0013956019228707787,
    -0.0013067086236574397, -0.0011910558921389659, -0.0010559311451048196,
    -0.0009083509068270846, -0.00075490316901425823, -0.00060149736382054605,
    -0.000453291791856472, -0.00031452329947580753, -0.00018854053846359119,
    -7.77223329305013E-5, 1.6422350531876982E-5, 9.3258619724152889E-5,
    0.00015285802723030522, 0.00019597097436338176, 0.00022383399901438949,
    0.00023811411848902473, 0.00024071320664146036, 0.00023371820107284685,
    0.00021921638416897869, 0.00019927957335407696, 0.0001758116268392333,
    0.00015054913635705378, 0.00012496233628925989, 0.00010028481292981024,
    7.7436913907066447E-5, 5.7098933374758445E-5, 7.1304380923446032E-5 };

  static const real_T tmp_4[26] = { -0.0039367421859610174,
    0.0065960362718635391, -0.010866421259157469, 0.015523621567767366,
    -0.019570526864945238, 0.02160928335361819, -0.019889119395638935,
    0.012347577763789114, 0.0035611266878216724, -0.03172981218558911,
    0.080863640626527622, -0.18253075126518334, 0.62641358074609388,
    0.62641358074609388, -0.18253075126518334, 0.080863640626527622,
    -0.03172981218558911, 0.0035611266878216724, 0.012347577763789114,
    -0.019889119395638935, 0.02160928335361819, -0.019570526864945238,
    0.015523621567767366, -0.010866421259157469, 0.0065960362718635391,
    -0.0039367421859610174 };

  Controller_LowpassFilter1_Start(&localDW->LowpassFilter1);
  Controller_LowpassFilter1_Start(&localDW->LowpassFilter1_p);

  /* Start for MATLABSystem: '<S6>/Lowpass Filter1' */
  localDW->obj.pSampleRateInherit = -1.0;
  localDW->obj.isInitialized = 0;
  localDW->obj.NumChannels = -1.0;
  localDW->objisempty_l = true;
  iobj_0 = &localDW->gobj_0_j;
  localDW->obj.isInitialized = 1;
  localDW->obj.pSampleRateInherit = 1000.0;
  localDW->gobj_0_j.isInitialized = 0;

  /* System object Constructor function: dsp.FIRFilter */
  iobj_0->cSFunObject.P0_InitialStates = 0.0;
  for (i = 0; i < 117; i++) {
    iobj_0->cSFunObject.P1_Coefficients[i] = tmp_2[i];
  }

  for (i = 0; i < 117; i++) {
    iobj_0->cSFunObject.P1_Coefficients[i] = tmp[i];
  }

  localDW->obj.FilterObj = &localDW->gobj_0_j;
  localDW->obj.NumChannels = 1.0;

  /* End of Start for MATLABSystem: '<S6>/Lowpass Filter1' */
  Controller_LowpassFilter1_Start(&localDW->LowpassFilter1_pna);
  Controller_LowpassFilter1_Start(&localDW->LowpassFilter1_pnae);

  /* Start for MATLABSystem: '<S10>/Lowpass Filter1' */
  localDW->obj_dz.pSampleRateInherit = -1.0;
  localDW->obj_dz.isInitialized = 0;
  localDW->obj_dz.NumChannels = -1.0;
  localDW->objisempty = true;
  iobj_0_0 = &localDW->gobj_0;
  localDW->obj_dz.isInitialized = 1;
  localDW->obj_dz.pSampleRateInherit = 1000.0;
  localDW->gobj_0.isInitialized = 0;

  /* System object Constructor function: dsp.FIRFilter */
  iobj_0_0->cSFunObject.P0_InitialStates = 0.0;
  for (i = 0; i < 181; i++) {
    iobj_0_0->cSFunObject.P1_Coefficients[i] = tmp_3[i];
  }

  for (i = 0; i < 181; i++) {
    iobj_0_0->cSFunObject.P1_Coefficients[i] = tmp_0[i];
  }

  localDW->obj_dz.FilterObj = &localDW->gobj_0;
  localDW->obj_dz.NumChannels = 1.0;

  /* End of Start for MATLABSystem: '<S10>/Lowpass Filter1' */

  /* Start for MATLABSystem: '<S1>/Lowpass Filter1' */
  localDW->obj_d.pSampleRateInherit = -1.0;
  localDW->obj_d.isInitialized = 0;
  localDW->obj_d.NumChannels = -1.0;
  localDW->objisempty_d = true;
  iobj_0_1 = &localDW->gobj_0_jd;
  localDW->obj_d.isInitialized = 1;
  localDW->obj_d.pSampleRateInherit = 1000.0;
  localDW->gobj_0_jd.isInitialized = 0;

  /* System object Constructor function: dsp.FIRFilter */
  iobj_0_1->cSFunObject.P0_InitialStates = 0.0;
  for (i = 0; i < 26; i++) {
    iobj_0_1->cSFunObject.P1_Coefficients[i] = tmp_4[i];
  }

  for (i = 0; i < 26; i++) {
    iobj_0_1->cSFunObject.P1_Coefficients[i] = tmp_1[i];
  }

  localDW->obj_d.FilterObj = &localDW->gobj_0_jd;
  localDW->obj_d.NumChannels = 1.0;

  /* End of Start for MATLABSystem: '<S1>/Lowpass Filter1' */
}

/* Outputs for referenced model: 'Controller_ABS_VLC_AYC_ALGO2' */
void Controller_ABS_VLC_AYC_ALGO2(RT_MODEL_Controller_ABS_VLC_A_T * const
  Controller_ABS_VLC_AYC_ALGO2_M, const real_T *rtu_Vx_cont, const real_T
  *rtu_r_cont, const real_T *rtu_S_Ang, const real_T *rtu_FL_SR, const real_T
  *rtu_FR_SR, const real_T *rtu_RL_SR, const real_T *rtu_RR_SR, const real_T
  *rtu_T_P, const real_T *rtu_FLFzN, const real_T *rtu_FRFzN, const real_T
  *rtu_RLFzN, const real_T *rtu_RRFzN, real_T *rty_TFL, real_T *rty_TFR, real_T *
  rty_TRL, real_T *rty_TRR, real_T *rty_YE, real_T *rty_LAE,
  B_Controller_ABS_VLC_AYC_AL_c_T *localB, DW_Controller_ABS_VLC_AYC_A_f_T
  *localDW)
{
  /* local block i/o variables */
  real_T rtb_Sum2_fz;
  real_T rtb_DesiredLatAccelNumerator;
  real_T rtb_Sum1_o;
  real_T rtb_Sum7;
  real_T *lastU;
  dsp_FIRFilter_0_Controller_lu_T *obj;
  dsp_FIRFilter_0_Controlle_lul_T *obj_0;
  dsp_FIRFilter_0_Controller__l_T *obj_1;
  int32_T j;
  real_T rtb_Sum8;
  real_T rtb_DesiredLatAccelDenominator;
  real_T rtb_Switch2;
  real_T rtb_Derivative;
  real_T rtb_Product;
  real_T rtb_TorquefrompedalNm;

  /* Gain: '<S1>/Available Torque [Nm]' */
  rtb_TorquefrompedalNm = 150.0 * *rtu_T_P;

  /* Derivative: '<S1>/Derivative' */
  if ((localDW->TimeStampA >= (*(Controller_ABS_VLC_A_TimingBrdg->
         taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]))) && (localDW->TimeStampB
       >= (*(Controller_ABS_VLC_A_TimingBrdg->
             taskTime[Controller_ABS_VLC_AY_GlobalTID[0]])))) {
    rtb_Derivative = 0.0;
  } else {
    rtb_Switch2 = localDW->TimeStampA;
    lastU = &localDW->LastUAtTimeA;
    if (localDW->TimeStampA < localDW->TimeStampB) {
      if (localDW->TimeStampB < (*(Controller_ABS_VLC_A_TimingBrdg->
            taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]))) {
        rtb_Switch2 = localDW->TimeStampB;
        lastU = &localDW->LastUAtTimeB;
      }
    } else {
      if (localDW->TimeStampA >= (*(Controller_ABS_VLC_A_TimingBrdg->
            taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]))) {
        rtb_Switch2 = localDW->TimeStampB;
        lastU = &localDW->LastUAtTimeB;
      }
    }

    rtb_Derivative = (*rtu_r_cont - *lastU) /
      ((*(Controller_ABS_VLC_A_TimingBrdg->
          taskTime[Controller_ABS_VLC_AY_GlobalTID[0]])) - rtb_Switch2);
  }

  /* End of Derivative: '<S1>/Derivative' */

  /* Abs: '<S4>/Abs' */
  rtb_Sum1_o = fabs(*rtu_FL_SR);

  /* Sum: '<S11>/Sum2' incorporates:
   *  ManualSwitch: '<S11>/Manual Switch1'
   */
  rtb_Sum2_fz = *rtu_RLFzN + *rtu_RRFzN;

  /* Sum: '<S11>/Sum1' incorporates:
   *  ManualSwitch: '<S11>/Manual Switch1'
   */
  rtb_DesiredLatAccelNumerator = *rtu_FLFzN + *rtu_FRFzN;

  /* Fcn: '<S11>/Desired Lat Accel Denominator' incorporates:
   *  ManualSwitch: '<S11>/Manual Switch1'
   */
  rtb_DesiredLatAccelDenominator = (rtb_DesiredLatAccelNumerator / 90000.0 -
    rtb_Sum2_fz / 90000.0) * rt_powd_snf(*rtu_Vx_cont, 2.0) / 9.81 + 3.0;

  /* Fcn: '<S11>/Desired Lat Accel Numerator' incorporates:
   *  ManualSwitch: '<S11>/Manual Switch1'
   */
  rtb_DesiredLatAccelNumerator = *rtu_Vx_cont * *rtu_S_Ang;

  /* ManualSwitch: '<S11>/Manual Switch1' incorporates:
   *  Product: '<S11>/Divide'
   *  Sum: '<S11>/Sum3'
   */
  *rty_YE = rtb_DesiredLatAccelNumerator / rtb_DesiredLatAccelDenominator -
    *rtu_r_cont;

  /* Abs: '<S4>/Abs1' */
  rtb_Switch2 = fabs(*rty_YE);

  /* Switch: '<S4>/Switch2' incorporates:
   *  Constant: '<S4>/Slip limit'
   */
  if (!(rtb_Switch2 > 0.07)) {
    rtb_Switch2 = 0.0;
  }

  /* End of Switch: '<S4>/Switch2' */

  /* Sum: '<S4>/Sum' incorporates:
   *  Constant: '<S4>/Target Slip'
   *  Sum: '<S4>/Sum2'
   */
  localB->FLSlipError = (0.2 - rtb_Switch2) - rtb_Sum1_o;

  /* Derivative: '<S4>/Derivative' */
  if ((localDW->TimeStampA_n >= (*(Controller_ABS_VLC_A_TimingBrdg->
         taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]))) &&
      (localDW->TimeStampB_c >= (*(Controller_ABS_VLC_A_TimingBrdg->
         taskTime[Controller_ABS_VLC_AY_GlobalTID[0]])))) {
    rtb_Switch2 = 0.0;
  } else {
    rtb_Switch2 = localDW->TimeStampA_n;
    lastU = &localDW->LastUAtTimeA_o;
    if (localDW->TimeStampA_n < localDW->TimeStampB_c) {
      if (localDW->TimeStampB_c < (*(Controller_ABS_VLC_A_TimingBrdg->
            taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]))) {
        rtb_Switch2 = localDW->TimeStampB_c;
        lastU = &localDW->LastUAtTimeB_c;
      }
    } else {
      if (localDW->TimeStampA_n >= (*(Controller_ABS_VLC_A_TimingBrdg->
            taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]))) {
        rtb_Switch2 = localDW->TimeStampB_c;
        lastU = &localDW->LastUAtTimeB_c;
      }
    }

    rtb_Switch2 = (localB->FLSlipError - *lastU) /
      ((*(Controller_ABS_VLC_A_TimingBrdg->
          taskTime[Controller_ABS_VLC_AY_GlobalTID[0]])) - rtb_Switch2);
  }

  /* End of Derivative: '<S4>/Derivative' */
  if (rtmIsMajorTimeStep(Controller_ABS_VLC_AYC_ALGO2_M)) {
    /* DigitalClock: '<S4>/Digital Clock' */
    localB->DigitalClock = (( rtmGetClockTick1() + rtmGetClockTickH1()*
      4294967296.0 ) * 0.001);
  }

  /* Switch: '<S4>/Switch' incorporates:
   *  Sum: '<S4>/Sum1'
   */
  localB->Switch = (localB->FLSlipError + rtb_Switch2 > 0.0);
  if (rtmIsMajorTimeStep(Controller_ABS_VLC_AYC_ALGO2_M)) {
    Controller_ABS_V_LowpassFilter1(localB->Switch, &localB->LowpassFilter1,
      &localDW->LowpassFilter1);
  }

  /* Switch: '<S4>/Switch1' */
  if (localB->DigitalClock >= 0.02) {
    rtb_DesiredLatAccelDenominator = localB->LowpassFilter1.LowpassFilter1;
  } else {
    rtb_DesiredLatAccelDenominator = localB->Switch;
  }

  /* End of Switch: '<S4>/Switch1' */

  /* Abs: '<S5>/Abs1' */
  rtb_Sum1_o = fabs(*rty_YE);

  /* Switch: '<S5>/Switch2' incorporates:
   *  Constant: '<S5>/Slip limit'
   */
  if (!(rtb_Sum1_o > 0.07)) {
    rtb_Sum1_o = 0.0;
  }

  /* End of Switch: '<S5>/Switch2' */

  /* Sum: '<S5>/Sum' incorporates:
   *  Abs: '<S5>/Abs'
   *  Constant: '<S5>/Target Slip'
   *  Sum: '<S5>/Sum2'
   */
  localB->FLSlipError_k = (0.2 - rtb_Sum1_o) - fabs(*rtu_FR_SR);

  /* Derivative: '<S5>/Derivative' */
  if ((localDW->TimeStampA_d >= (*(Controller_ABS_VLC_A_TimingBrdg->
         taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]))) &&
      (localDW->TimeStampB_o >= (*(Controller_ABS_VLC_A_TimingBrdg->
         taskTime[Controller_ABS_VLC_AY_GlobalTID[0]])))) {
    rtb_Switch2 = 0.0;
  } else {
    rtb_Switch2 = localDW->TimeStampA_d;
    lastU = &localDW->LastUAtTimeA_i;
    if (localDW->TimeStampA_d < localDW->TimeStampB_o) {
      if (localDW->TimeStampB_o < (*(Controller_ABS_VLC_A_TimingBrdg->
            taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]))) {
        rtb_Switch2 = localDW->TimeStampB_o;
        lastU = &localDW->LastUAtTimeB_h;
      }
    } else {
      if (localDW->TimeStampA_d >= (*(Controller_ABS_VLC_A_TimingBrdg->
            taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]))) {
        rtb_Switch2 = localDW->TimeStampB_o;
        lastU = &localDW->LastUAtTimeB_h;
      }
    }

    rtb_Switch2 = (localB->FLSlipError_k - *lastU) /
      ((*(Controller_ABS_VLC_A_TimingBrdg->
          taskTime[Controller_ABS_VLC_AY_GlobalTID[0]])) - rtb_Switch2);
  }

  /* End of Derivative: '<S5>/Derivative' */
  if (rtmIsMajorTimeStep(Controller_ABS_VLC_AYC_ALGO2_M)) {
    /* DigitalClock: '<S5>/Digital Clock' */
    localB->DigitalClock_m = (( rtmGetClockTick1() + rtmGetClockTickH1()*
      4294967296.0 ) * 0.001);
  }

  /* Switch: '<S5>/Switch' incorporates:
   *  Sum: '<S5>/Sum1'
   */
  localB->Switch_k = (localB->FLSlipError_k + rtb_Switch2 > 0.0);
  if (rtmIsMajorTimeStep(Controller_ABS_VLC_AYC_ALGO2_M)) {
    Controller_ABS_V_LowpassFilter1(localB->Switch_k, &localB->LowpassFilter1_p,
      &localDW->LowpassFilter1_p);
  }

  /* Sum: '<S7>/Sum1' */
  rtb_Sum1_o = *rtu_FLFzN + *rtu_FRFzN;

  /* Sum: '<S7>/Sum2' */
  rtb_Sum7 = *rtu_RLFzN + *rtu_RRFzN;

  /* Sum: '<S7>/Sum' incorporates:
   *  Fcn: '<S7>/Desired Lat Accel Denominator'
   *  Fcn: '<S7>/Desired Lat Accel Numerator'
   *  Product: '<S7>/Divide'
   */
  *rty_LAE = rtb_Derivative - *rtu_Vx_cont / 9.81 * *rtu_S_Ang / ((rtb_Sum1_o /
    90000.0 - rtb_Sum7 / 90000.0) * rt_powd_snf(*rtu_Vx_cont, 2.0) / 9.81 + 3.0);
  if (rtmIsMajorTimeStep(Controller_ABS_VLC_AYC_ALGO2_M)) {
    /* MATLABSystem: '<S6>/Lowpass Filter1' */
    rtb_Derivative = *rty_LAE;

    /* Start for MATLABSystem: '<S6>/Lowpass Filter1' incorporates:
     *  MATLABSystem: '<S6>/Lowpass Filter1'
     */
    if (localDW->obj.FilterObj->isInitialized != 1) {
      localDW->obj.FilterObj->isInitialized = 1;

      /* System object Initialization function: dsp.FIRFilter */
      rtb_Switch2 = localDW->obj.FilterObj->cSFunObject.P0_InitialStates;
      for (j = 0; j < 116; j++) {
        localDW->obj.FilterObj->cSFunObject.W0_states[j] = rtb_Switch2;
      }
    }

    obj = &localDW->obj.FilterObj->cSFunObject;

    /* System object Outputs function: dsp.FIRFilter */
    /* Consume delay line and beginning of input samples */
    rtb_Sum8 = 0.0;
    j = 0;
    while (j < 1) {
      rtb_Switch2 = rtb_Derivative * obj->P1_Coefficients[0];
      rtb_Sum8 += rtb_Switch2;
      j = 1;
    }

    for (j = 0; j < 116; j++) {
      rtb_Switch2 = obj->P1_Coefficients[1 + j] * obj->W0_states[j];
      rtb_Sum8 += rtb_Switch2;
    }

    /* Update delay line for next frame */
    for (j = 114; j >= 0; j--) {
      obj->W0_states[1 + j] = obj->W0_states[j];
    }

    localDW->obj.FilterObj->cSFunObject.W0_states[0] = rtb_Derivative;

    /* End of Start for MATLABSystem: '<S6>/Lowpass Filter1' */

    /* MATLABSystem: '<S6>/Lowpass Filter1' */
    localB->LowpassFilter1_k = rtb_Sum8;

    /* DigitalClock: '<S6>/Digital Clock' */
    localB->DigitalClock_o = (( rtmGetClockTick1() + rtmGetClockTickH1()*
      4294967296.0 ) * 0.001);
  }

  /* Switch: '<S6>/Switch1' */
  if (localB->DigitalClock_o >= 0.02) {
    rtb_Switch2 = localB->LowpassFilter1_k;
  } else {
    rtb_Switch2 = *rty_LAE;
  }

  /* End of Switch: '<S6>/Switch1' */

  /* Abs: '<S6>/Abs' */
  rtb_Sum8 = fabs(rtb_Switch2);

  /* Saturate: '<S4>/Saturation' */
  if (rtb_DesiredLatAccelDenominator > 1.0) {
    rtb_DesiredLatAccelDenominator = 1.0;
  } else {
    if (rtb_DesiredLatAccelDenominator < 0.0) {
      rtb_DesiredLatAccelDenominator = 0.0;
    }
  }

  /* End of Saturate: '<S4>/Saturation' */

  /* Product: '<S1>/Product' */
  rtb_Product = rtb_Sum8 * rtb_TorquefrompedalNm *
    rtb_DesiredLatAccelDenominator;

  /* Switch: '<S5>/Switch1' */
  if (localB->DigitalClock_m >= 0.02) {
    rtb_DesiredLatAccelDenominator = localB->LowpassFilter1_p.LowpassFilter1;
  } else {
    rtb_DesiredLatAccelDenominator = localB->Switch_k;
  }

  /* End of Switch: '<S5>/Switch1' */

  /* Saturate: '<S5>/Saturation' */
  if (rtb_DesiredLatAccelDenominator > 1.0) {
    rtb_DesiredLatAccelDenominator = 1.0;
  } else {
    if (rtb_DesiredLatAccelDenominator < 0.0) {
      rtb_DesiredLatAccelDenominator = 0.0;
    }
  }

  /* Product: '<S1>/Product1' incorporates:
   *  Saturate: '<S5>/Saturation'
   */
  rtb_Derivative = rtb_Sum8 * rtb_TorquefrompedalNm *
    rtb_DesiredLatAccelDenominator;

  /* Abs: '<S8>/Abs' */
  rtb_Sum7 = fabs(*rtu_RL_SR);

  /* Abs: '<S8>/Abs1' */
  rtb_Switch2 = fabs(*rty_YE);

  /* Switch: '<S8>/Switch2' incorporates:
   *  Constant: '<S8>/Slip limit'
   */
  if (!(rtb_Switch2 > 0.07)) {
    rtb_Switch2 = 0.0;
  }

  /* End of Switch: '<S8>/Switch2' */

  /* Sum: '<S8>/Sum' incorporates:
   *  Constant: '<S8>/Target Slip'
   *  Sum: '<S8>/Sum2'
   */
  localB->FLSlipError_f = (0.2 - rtb_Switch2) - rtb_Sum7;

  /* Derivative: '<S8>/Derivative' */
  if ((localDW->TimeStampA_nm >= (*(Controller_ABS_VLC_A_TimingBrdg->
         taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]))) &&
      (localDW->TimeStampB_a >= (*(Controller_ABS_VLC_A_TimingBrdg->
         taskTime[Controller_ABS_VLC_AY_GlobalTID[0]])))) {
    rtb_Sum7 = 0.0;
  } else {
    rtb_Switch2 = localDW->TimeStampA_nm;
    lastU = &localDW->LastUAtTimeA_p;
    if (localDW->TimeStampA_nm < localDW->TimeStampB_a) {
      if (localDW->TimeStampB_a < (*(Controller_ABS_VLC_A_TimingBrdg->
            taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]))) {
        rtb_Switch2 = localDW->TimeStampB_a;
        lastU = &localDW->LastUAtTimeB_n;
      }
    } else {
      if (localDW->TimeStampA_nm >= (*(Controller_ABS_VLC_A_TimingBrdg->
            taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]))) {
        rtb_Switch2 = localDW->TimeStampB_a;
        lastU = &localDW->LastUAtTimeB_n;
      }
    }

    rtb_Sum7 = (localB->FLSlipError_f - *lastU) /
      ((*(Controller_ABS_VLC_A_TimingBrdg->
          taskTime[Controller_ABS_VLC_AY_GlobalTID[0]])) - rtb_Switch2);
  }

  /* End of Derivative: '<S8>/Derivative' */

  /* Switch: '<S8>/Switch' incorporates:
   *  Sum: '<S8>/Sum1'
   */
  localB->Switch_o = (localB->FLSlipError_f + rtb_Sum7 > 0.0);
  if (rtmIsMajorTimeStep(Controller_ABS_VLC_AYC_ALGO2_M)) {
    Controller_ABS_V_LowpassFilter1(localB->Switch_o,
      &localB->LowpassFilter1_pna, &localDW->LowpassFilter1_pna);

    /* DigitalClock: '<S8>/Digital Clock' */
    localB->DigitalClock_k = (( rtmGetClockTick1() + rtmGetClockTickH1()*
      4294967296.0 ) * 0.001);
  }

  /* Switch: '<S8>/Switch1' */
  if (localB->DigitalClock_k >= 0.02) {
    rtb_Sum7 = localB->LowpassFilter1_pna.LowpassFilter1;
  } else {
    rtb_Sum7 = localB->Switch_o;
  }

  /* End of Switch: '<S8>/Switch1' */

  /* Saturate: '<S8>/Saturation' */
  if (rtb_Sum7 > 1.0) {
    rtb_Switch2 = 1.0;
  } else if (rtb_Sum7 < 0.0) {
    rtb_Switch2 = 0.0;
  } else {
    rtb_Switch2 = rtb_Sum7;
  }

  /* Product: '<S1>/Product2' incorporates:
   *  Saturate: '<S8>/Saturation'
   */
  rtb_DesiredLatAccelDenominator = rtb_Sum8 * rtb_TorquefrompedalNm *
    rtb_Switch2;

  /* Abs: '<S9>/Abs' */
  rtb_Sum7 = fabs(*rtu_RR_SR);

  /* Abs: '<S9>/Abs1' */
  rtb_Switch2 = fabs(*rty_YE);

  /* Switch: '<S9>/Switch2' incorporates:
   *  Constant: '<S9>/Slip limit'
   */
  if (!(rtb_Switch2 > 0.07)) {
    rtb_Switch2 = 0.0;
  }

  /* End of Switch: '<S9>/Switch2' */

  /* Sum: '<S9>/Sum' incorporates:
   *  Constant: '<S9>/Target Slip'
   *  Sum: '<S9>/Sum2'
   */
  localB->FLSlipError_n = (0.2 - rtb_Switch2) - rtb_Sum7;

  /* Derivative: '<S9>/Derivative' */
  if ((localDW->TimeStampA_m >= (*(Controller_ABS_VLC_A_TimingBrdg->
         taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]))) &&
      (localDW->TimeStampB_m >= (*(Controller_ABS_VLC_A_TimingBrdg->
         taskTime[Controller_ABS_VLC_AY_GlobalTID[0]])))) {
    rtb_Sum7 = 0.0;
  } else {
    rtb_Switch2 = localDW->TimeStampA_m;
    lastU = &localDW->LastUAtTimeA_a;
    if (localDW->TimeStampA_m < localDW->TimeStampB_m) {
      if (localDW->TimeStampB_m < (*(Controller_ABS_VLC_A_TimingBrdg->
            taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]))) {
        rtb_Switch2 = localDW->TimeStampB_m;
        lastU = &localDW->LastUAtTimeB_hf;
      }
    } else {
      if (localDW->TimeStampA_m >= (*(Controller_ABS_VLC_A_TimingBrdg->
            taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]))) {
        rtb_Switch2 = localDW->TimeStampB_m;
        lastU = &localDW->LastUAtTimeB_hf;
      }
    }

    rtb_Sum7 = (localB->FLSlipError_n - *lastU) /
      ((*(Controller_ABS_VLC_A_TimingBrdg->
          taskTime[Controller_ABS_VLC_AY_GlobalTID[0]])) - rtb_Switch2);
  }

  /* End of Derivative: '<S9>/Derivative' */

  /* Switch: '<S9>/Switch' incorporates:
   *  Sum: '<S9>/Sum1'
   */
  localB->Switch_ka = (localB->FLSlipError_n + rtb_Sum7 > 0.0);
  if (rtmIsMajorTimeStep(Controller_ABS_VLC_AYC_ALGO2_M)) {
    Controller_ABS_V_LowpassFilter1(localB->Switch_ka,
      &localB->LowpassFilter1_pnae, &localDW->LowpassFilter1_pnae);

    /* DigitalClock: '<S9>/Digital Clock' */
    localB->DigitalClock_g = (( rtmGetClockTick1() + rtmGetClockTickH1()*
      4294967296.0 ) * 0.001);
  }

  /* Switch: '<S9>/Switch1' */
  if (localB->DigitalClock_g >= 0.02) {
    rtb_Sum7 = localB->LowpassFilter1_pnae.LowpassFilter1;
  } else {
    rtb_Sum7 = localB->Switch_ka;
  }

  /* End of Switch: '<S9>/Switch1' */

  /* Saturate: '<S9>/Saturation' */
  if (rtb_Sum7 > 1.0) {
    rtb_Switch2 = 1.0;
  } else if (rtb_Sum7 < 0.0) {
    rtb_Switch2 = 0.0;
  } else {
    rtb_Switch2 = rtb_Sum7;
  }

  /* Product: '<S1>/Product3' incorporates:
   *  Saturate: '<S9>/Saturation'
   */
  rtb_TorquefrompedalNm = rtb_Sum8 * rtb_TorquefrompedalNm * rtb_Switch2;

  /* Derivative: '<S10>/Derivative' */
  if ((localDW->TimeStampA_ml >= (*(Controller_ABS_VLC_A_TimingBrdg->
         taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]))) &&
      (localDW->TimeStampB_j >= (*(Controller_ABS_VLC_A_TimingBrdg->
         taskTime[Controller_ABS_VLC_AY_GlobalTID[0]])))) {
    rtb_Sum8 = 0.0;
  } else {
    rtb_Switch2 = localDW->TimeStampA_ml;
    lastU = &localDW->LastUAtTimeA_l;
    if (localDW->TimeStampA_ml < localDW->TimeStampB_j) {
      if (localDW->TimeStampB_j < (*(Controller_ABS_VLC_A_TimingBrdg->
            taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]))) {
        rtb_Switch2 = localDW->TimeStampB_j;
        lastU = &localDW->LastUAtTimeB_a;
      }
    } else {
      if (localDW->TimeStampA_ml >= (*(Controller_ABS_VLC_A_TimingBrdg->
            taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]))) {
        rtb_Switch2 = localDW->TimeStampB_j;
        lastU = &localDW->LastUAtTimeB_a;
      }
    }

    rtb_Sum8 = (*rty_YE - *lastU) /
      ((*(Controller_ABS_VLC_A_TimingBrdg->
          taskTime[Controller_ABS_VLC_AY_GlobalTID[0]])) - rtb_Switch2);
  }

  /* End of Derivative: '<S10>/Derivative' */

  /* Sum: '<S10>/Sum' incorporates:
   *  Gain: '<S10>/Yaw Acceleration Error Weighting Gain'
   */
  rtb_Sum8 = 5.0 * rtb_Sum8 + *rty_YE;

  /* Signum: '<S10>/Sign' */
  if (rtb_Sum8 < 0.0) {
    rtb_Sum8 = -1.0;
  } else if (rtb_Sum8 > 0.0) {
    rtb_Sum8 = 1.0;
  } else {
    if (rtb_Sum8 == 0.0) {
      rtb_Sum8 = 0.0;
    }
  }

  /* Gain: '<S10>/Available Torque [Nm]' incorporates:
   *  Signum: '<S10>/Sign'
   */
  localB->DeltaTorqueFromAYCController = 150.0 * rtb_Sum8;
  if (rtmIsMajorTimeStep(Controller_ABS_VLC_AYC_ALGO2_M)) {
    /* Start for MATLABSystem: '<S10>/Lowpass Filter1' incorporates:
     *  MATLABSystem: '<S10>/Lowpass Filter1'
     */
    if (localDW->obj_dz.FilterObj->isInitialized != 1) {
      localDW->obj_dz.FilterObj->isInitialized = 1;

      /* System object Initialization function: dsp.FIRFilter */
      rtb_Switch2 = localDW->obj_dz.FilterObj->cSFunObject.P0_InitialStates;
      for (j = 0; j < 180; j++) {
        localDW->obj_dz.FilterObj->cSFunObject.W0_states[j] = rtb_Switch2;
      }
    }

    obj_0 = &localDW->obj_dz.FilterObj->cSFunObject;

    /* System object Outputs function: dsp.FIRFilter */
    /* Consume delay line and beginning of input samples */
    rtb_Sum8 = 0.0;
    j = 0;
    while (j < 1) {
      rtb_Switch2 = localB->DeltaTorqueFromAYCController *
        obj_0->P1_Coefficients[0];
      rtb_Sum8 += rtb_Switch2;
      j = 1;
    }

    for (j = 0; j < 180; j++) {
      rtb_Switch2 = obj_0->P1_Coefficients[1 + j] * obj_0->W0_states[j];
      rtb_Sum8 += rtb_Switch2;
    }

    /* Update delay line for next frame */
    for (j = 178; j >= 0; j--) {
      obj_0->W0_states[1 + j] = obj_0->W0_states[j];
    }

    localDW->obj_dz.FilterObj->cSFunObject.W0_states[0] =
      localB->DeltaTorqueFromAYCController;

    /* End of Start for MATLABSystem: '<S10>/Lowpass Filter1' */

    /* MATLABSystem: '<S10>/Lowpass Filter1' */
    localB->LowpassFilter1_o = rtb_Sum8;

    /* DigitalClock: '<S10>/Digital Clock' */
    localB->DigitalClock_l = (( rtmGetClockTick1() + rtmGetClockTickH1()*
      4294967296.0 ) * 0.001);
  }

  /* Switch: '<S10>/Switch1' */
  if (localB->DigitalClock_l >= 0.02) {
    rtb_Sum8 = localB->LowpassFilter1_o;
  } else {
    rtb_Sum8 = localB->DeltaTorqueFromAYCController;
  }

  /* End of Switch: '<S10>/Switch1' */

  /* Sum: '<S1>/Sum5' */
  rtb_Sum7 = rtb_Sum8 + rtb_Product;

  /* Saturate: '<S1>/Saturation' */
  if (rtb_Sum7 > 150.0) {
    *rty_TFL = 150.0;
  } else if (rtb_Sum7 < -150.0) {
    *rty_TFL = -150.0;
  } else {
    *rty_TFL = rtb_Sum7;
  }

  /* End of Saturate: '<S1>/Saturation' */

  /* Sum: '<S1>/Sum6' */
  rtb_Sum7 = rtb_Derivative - rtb_Sum8;

  /* Saturate: '<S1>/Saturation1' */
  if (rtb_Sum7 > 150.0) {
    *rty_TFR = 150.0;
  } else if (rtb_Sum7 < -150.0) {
    *rty_TFR = -150.0;
  } else {
    *rty_TFR = rtb_Sum7;
  }

  /* End of Saturate: '<S1>/Saturation1' */

  /* Sum: '<S1>/Sum7' */
  rtb_Sum7 = rtb_Sum8 + rtb_DesiredLatAccelDenominator;

  /* Saturate: '<S1>/Saturation2' */
  if (rtb_Sum7 > 150.0) {
    *rty_TRL = 150.0;
  } else if (rtb_Sum7 < -150.0) {
    *rty_TRL = -150.0;
  } else {
    *rty_TRL = rtb_Sum7;
  }

  /* End of Saturate: '<S1>/Saturation2' */

  /* Sum: '<S1>/Sum8' */
  rtb_Sum8 = rtb_TorquefrompedalNm - rtb_Sum8;

  /* Saturate: '<S1>/Saturation3' */
  if (rtb_Sum8 > 150.0) {
    *rty_TRR = 150.0;
  } else if (rtb_Sum8 < -150.0) {
    *rty_TRR = -150.0;
  } else {
    *rty_TRR = rtb_Sum8;
  }

  /* End of Saturate: '<S1>/Saturation3' */
  if (rtmIsMajorTimeStep(Controller_ABS_VLC_AYC_ALGO2_M)) {
    /* Start for MATLABSystem: '<S1>/Lowpass Filter1' incorporates:
     *  MATLABSystem: '<S1>/Lowpass Filter1'
     */
    if (localDW->obj_d.FilterObj->isInitialized != 1) {
      localDW->obj_d.FilterObj->isInitialized = 1;

      /* System object Initialization function: dsp.FIRFilter */
      rtb_Switch2 = localDW->obj_d.FilterObj->cSFunObject.P0_InitialStates;
      for (j = 0; j < 25; j++) {
        localDW->obj_d.FilterObj->cSFunObject.W0_states[j] = rtb_Switch2;
      }
    }

    obj_1 = &localDW->obj_d.FilterObj->cSFunObject;

    /* System object Outputs function: dsp.FIRFilter */
    /* Consume delay line and beginning of input samples */
    /* Update delay line for next frame */
    for (j = 23; j >= 0; j--) {
      obj_1->W0_states[1 + j] = obj_1->W0_states[j];
    }

    localDW->obj_d.FilterObj->cSFunObject.W0_states[0] = 0.0;

    /* End of Start for MATLABSystem: '<S1>/Lowpass Filter1' */
  }
}

/* Update for referenced model: 'Controller_ABS_VLC_AYC_ALGO2' */
void Controller_ABS_VLC_AYC_A_Update(const real_T *rtu_r_cont, real_T *rty_YE,
  B_Controller_ABS_VLC_AYC_AL_c_T *localB, DW_Controller_ABS_VLC_AYC_A_f_T
  *localDW)
{
  real_T *lastU;

  /* Update for Derivative: '<S1>/Derivative' */
  if (localDW->TimeStampA == (rtInf)) {
    localDW->TimeStampA = (*(Controller_ABS_VLC_A_TimingBrdg->
      taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeA;
  } else if (localDW->TimeStampB == (rtInf)) {
    localDW->TimeStampB = (*(Controller_ABS_VLC_A_TimingBrdg->
      taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeB;
  } else if (localDW->TimeStampA < localDW->TimeStampB) {
    localDW->TimeStampA = (*(Controller_ABS_VLC_A_TimingBrdg->
      taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeA;
  } else {
    localDW->TimeStampB = (*(Controller_ABS_VLC_A_TimingBrdg->
      taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeB;
  }

  *lastU = *rtu_r_cont;

  /* End of Update for Derivative: '<S1>/Derivative' */

  /* Update for Derivative: '<S4>/Derivative' */
  if (localDW->TimeStampA_n == (rtInf)) {
    localDW->TimeStampA_n = (*(Controller_ABS_VLC_A_TimingBrdg->
      taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeA_o;
  } else if (localDW->TimeStampB_c == (rtInf)) {
    localDW->TimeStampB_c = (*(Controller_ABS_VLC_A_TimingBrdg->
      taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeB_c;
  } else if (localDW->TimeStampA_n < localDW->TimeStampB_c) {
    localDW->TimeStampA_n = (*(Controller_ABS_VLC_A_TimingBrdg->
      taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeA_o;
  } else {
    localDW->TimeStampB_c = (*(Controller_ABS_VLC_A_TimingBrdg->
      taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeB_c;
  }

  *lastU = localB->FLSlipError;

  /* End of Update for Derivative: '<S4>/Derivative' */

  /* Update for Derivative: '<S5>/Derivative' */
  if (localDW->TimeStampA_d == (rtInf)) {
    localDW->TimeStampA_d = (*(Controller_ABS_VLC_A_TimingBrdg->
      taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeA_i;
  } else if (localDW->TimeStampB_o == (rtInf)) {
    localDW->TimeStampB_o = (*(Controller_ABS_VLC_A_TimingBrdg->
      taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeB_h;
  } else if (localDW->TimeStampA_d < localDW->TimeStampB_o) {
    localDW->TimeStampA_d = (*(Controller_ABS_VLC_A_TimingBrdg->
      taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeA_i;
  } else {
    localDW->TimeStampB_o = (*(Controller_ABS_VLC_A_TimingBrdg->
      taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeB_h;
  }

  *lastU = localB->FLSlipError_k;

  /* End of Update for Derivative: '<S5>/Derivative' */

  /* Update for Derivative: '<S8>/Derivative' */
  if (localDW->TimeStampA_nm == (rtInf)) {
    localDW->TimeStampA_nm = (*(Controller_ABS_VLC_A_TimingBrdg->
      taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeA_p;
  } else if (localDW->TimeStampB_a == (rtInf)) {
    localDW->TimeStampB_a = (*(Controller_ABS_VLC_A_TimingBrdg->
      taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeB_n;
  } else if (localDW->TimeStampA_nm < localDW->TimeStampB_a) {
    localDW->TimeStampA_nm = (*(Controller_ABS_VLC_A_TimingBrdg->
      taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeA_p;
  } else {
    localDW->TimeStampB_a = (*(Controller_ABS_VLC_A_TimingBrdg->
      taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeB_n;
  }

  *lastU = localB->FLSlipError_f;

  /* End of Update for Derivative: '<S8>/Derivative' */

  /* Update for Derivative: '<S9>/Derivative' */
  if (localDW->TimeStampA_m == (rtInf)) {
    localDW->TimeStampA_m = (*(Controller_ABS_VLC_A_TimingBrdg->
      taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeA_a;
  } else if (localDW->TimeStampB_m == (rtInf)) {
    localDW->TimeStampB_m = (*(Controller_ABS_VLC_A_TimingBrdg->
      taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeB_hf;
  } else if (localDW->TimeStampA_m < localDW->TimeStampB_m) {
    localDW->TimeStampA_m = (*(Controller_ABS_VLC_A_TimingBrdg->
      taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeA_a;
  } else {
    localDW->TimeStampB_m = (*(Controller_ABS_VLC_A_TimingBrdg->
      taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeB_hf;
  }

  *lastU = localB->FLSlipError_n;

  /* End of Update for Derivative: '<S9>/Derivative' */

  /* Update for Derivative: '<S10>/Derivative' */
  if (localDW->TimeStampA_ml == (rtInf)) {
    localDW->TimeStampA_ml = (*(Controller_ABS_VLC_A_TimingBrdg->
      taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeA_l;
  } else if (localDW->TimeStampB_j == (rtInf)) {
    localDW->TimeStampB_j = (*(Controller_ABS_VLC_A_TimingBrdg->
      taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeB_a;
  } else if (localDW->TimeStampA_ml < localDW->TimeStampB_j) {
    localDW->TimeStampA_ml = (*(Controller_ABS_VLC_A_TimingBrdg->
      taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeA_l;
  } else {
    localDW->TimeStampB_j = (*(Controller_ABS_VLC_A_TimingBrdg->
      taskTime[Controller_ABS_VLC_AY_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeB_a;
  }

  *lastU = *rty_YE;

  /* End of Update for Derivative: '<S10>/Derivative' */
}

/* Termination for referenced model: 'Controller_ABS_VLC_AYC_ALGO2' */
void Controller_ABS_VLC_AYC_ALG_Term(DW_Controller_ABS_VLC_AYC_A_f_T *localDW)
{
  Controller__LowpassFilter1_Term(&localDW->LowpassFilter1);
  Controller__LowpassFilter1_Term(&localDW->LowpassFilter1_p);

  /* Start for MATLABSystem: '<S6>/Lowpass Filter1' incorporates:
   *  Terminate for MATLABSystem: '<S6>/Lowpass Filter1'
   */
  if (localDW->obj.isInitialized == 1) {
    localDW->obj.isInitialized = 2;
    if (localDW->obj.FilterObj->isInitialized == 1) {
      localDW->obj.FilterObj->isInitialized = 2;
    }

    localDW->obj.NumChannels = -1.0;
  }

  /* End of Start for MATLABSystem: '<S6>/Lowpass Filter1' */
  Controller__LowpassFilter1_Term(&localDW->LowpassFilter1_pna);
  Controller__LowpassFilter1_Term(&localDW->LowpassFilter1_pnae);

  /* Start for MATLABSystem: '<S10>/Lowpass Filter1' incorporates:
   *  Terminate for MATLABSystem: '<S10>/Lowpass Filter1'
   */
  if (localDW->obj_dz.isInitialized == 1) {
    localDW->obj_dz.isInitialized = 2;
    if (localDW->obj_dz.FilterObj->isInitialized == 1) {
      localDW->obj_dz.FilterObj->isInitialized = 2;
    }

    localDW->obj_dz.NumChannels = -1.0;
  }

  /* End of Start for MATLABSystem: '<S10>/Lowpass Filter1' */

  /* Start for MATLABSystem: '<S1>/Lowpass Filter1' incorporates:
   *  Terminate for MATLABSystem: '<S1>/Lowpass Filter1'
   */
  if (localDW->obj_d.isInitialized == 1) {
    localDW->obj_d.isInitialized = 2;
    if (localDW->obj_d.FilterObj->isInitialized == 1) {
      localDW->obj_d.FilterObj->isInitialized = 2;
    }

    localDW->obj_d.NumChannels = -1.0;
  }

  /* End of Start for MATLABSystem: '<S1>/Lowpass Filter1' */
}

/* Model initialize function */
void Controller_ABS_VLC_A_initialize(const char_T **rt_errorStatus,
  RTWSolverInfo *rt_solverInfo, const rtTimingBridge *timingBridge, int_T
  mdlref_TID0, int_T mdlref_TID1, RT_MODEL_Controller_ABS_VLC_A_T *const
  Controller_ABS_VLC_AYC_ALGO2_M, B_Controller_ABS_VLC_AYC_AL_c_T *localB,
  DW_Controller_ABS_VLC_AYC_A_f_T *localDW)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)Controller_ABS_VLC_AYC_ALGO2_M, 0,
                sizeof(RT_MODEL_Controller_ABS_VLC_A_T));

  /* setup the global timing engine */
  Controller_ABS_VLC_AY_GlobalTID[0] = mdlref_TID0;
  Controller_ABS_VLC_AY_GlobalTID[1] = mdlref_TID1;
  Controller_ABS_VLC_A_TimingBrdg = timingBridge;

  /* initialize error status */
  rtmSetErrorStatusPointer(Controller_ABS_VLC_AYC_ALGO2_M, rt_errorStatus);

  /* initialize RTWSolverInfo */
  Controller_ABS_VLC_AYC_ALGO2_M->solverInfo = (rt_solverInfo);

  /* Set the Timing fields to the appropriate data in the RTWSolverInfo */
  rtmSetSimTimeStepPointer(Controller_ABS_VLC_AYC_ALGO2_M, rtsiGetSimTimeStepPtr
    (Controller_ABS_VLC_AYC_ALGO2_M->solverInfo));
  Controller_ABS_VLC_AYC_ALGO2_M->Timing.stepSize0 = (rtsiGetStepSize
    (Controller_ABS_VLC_AYC_ALGO2_M->solverInfo));

  /* block I/O */
  (void) memset(((void *) localB), 0,
                sizeof(B_Controller_ABS_VLC_AYC_AL_c_T));

  /* states (dwork) */
  (void) memset((void *)localDW, 0,
                sizeof(DW_Controller_ABS_VLC_AYC_A_f_T));
}
