/**
 * atpll.c
 * 
 * Hosts components of the ATPLL estimator
 * 
 * Component: commutation
 */

/* *********************************************************************
 * Motor Control Application Framework
 * R8/RC38 (commit 128946, build on 2025 Apr 09)
 *
 * (c) 2017 - 2023 Microchip Technology Inc. and its subsidiaries. You may use
 * this software and any derivatives exclusively with Microchip products.
 *
 * This software and any accompanying information is for suggestion only.
 * It does not modify Microchip's standard warranty for its products.
 * You agree that you are solely responsible for testing the software and
 * determining its suitability.  Microchip has no obligation to modify,
 * test, certify, or support the software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE, OR ITS INTERACTION WITH
 * MICROCHIP PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY
 * APPLICATION.
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL,
 * PUNITIVE, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF
 * ANY KIND WHATSOEVER RELATED TO THE USE OF THIS SOFTWARE, THE
 * motorBench(R) DEVELOPMENT SUITE TOOL, PARAMETERS AND GENERATED CODE,
 * HOWEVER CAUSED, BY END USERS, WHETHER MICROCHIP'S CUSTOMERS OR
 * CUSTOMER'S CUSTOMERS, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGES OR THE DAMAGES ARE FORESEEABLE. TO THE
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
 * CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT
 * OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS
 * SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF
 * THESE TERMS.
 *
* *****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "util.h"
#include "parameters/atpll_params.h"
#include "parameters/motor_params.h"
#include "motor_control.h"
#include "motor_control_function_mapping.h"
#include "atpll.h"
#include "commutation/common.h"

void MCAF_EstimatorAtPllInit(MCAF_ESTIMATOR_ATPLL_T *atpll, 
        const MCAF_MOTOR_PARAMETERS_T *pmotor)
{
    atpll->kpGain = ATPLL_KP_GAIN;
    atpll->thetaElectrical = 0;
    atpll->kiOut = 0;
    atpll->omegaPIOutFiltStateVar = 0;
    atpll->thetaStateVar = 0;
    atpll->omegaElectrical = 0;
}

void MCAF_EstimatorAtPllStartupInit(MCAF_ESTIMATOR_ATPLL_T *atpll)
{
    // thetaElectrical and thetaStateVar not re-initialized to maintain angle continuity
    atpll->kiOut = 0;
    atpll->omegaPIOutFiltStateVar = 0;
    atpll->omegaElectrical = 0;
    atpll->omegaFiltStateVar = 0;
}

void MCAF_EstimatorAtPllStep(MCAF_ESTIMATOR_ATPLL_T *atpll, 
                const MCAF_STANDARD_INPUT_SIGNALS_T *pinput,
                const MCAF_MOTOR_PARAMETERS_T *pmotor,
                const int16_t speedRef)
{    
    /* Calculate sine and cosine components of the rotor angle */
    MC_CalculateSineCosine(atpll->thetaElectrical, &atpll->sincos);
    
    /* Transform the estimated BEMF voltage into rotor reference frame using:
     *  Esd =  Ealpha*cos(Angle) + Ebeta*sin(Angle)
     *  Esq = -Ealpha*sin(Angle) + Ebeta*cos(Angle)
     * i.e. equivalent to Park transform
     */
    atpll->esdq.d = UTIL_Shr15(__builtin_mulss(pinput->ealphabeta.alpha, atpll->sincos.cos) +
                               __builtin_mulss(pinput->ealphabeta.beta, atpll->sincos.sin));
    
    /* Input to pi controller is EdRef - EdCalculated. 
     * Since EdRef = 0, error becomes 0-Ed = -Ed 
     */
    atpll->esdNeg = -atpll->esdq.d;
    
    /* two point averaging (acts as low pass filter) */
    atpll->esdError = (atpll->esdError + atpll->esdNeg) >> 1;
    
    /* Kp Output */
    atpll->kpOut = __builtin_mulss(atpll->esdError,(atpll->kpGain * pinput->direction))
                                                        << (15-ATPLL_KP_GAIN_Q);
    
    /* Ki Output
     * Ki varies with speed till nominal speed to offset the plant zero
     * variation with speed */
    const int16_t kiInFactor = speedRef;
    atpll->kiIn = UTIL_MulQ15(atpll->esdError ,kiInFactor);
    atpll->kiOut += __builtin_mulss(atpll->kiIn, ATPLL_NORM_DELTAT_KI);
    
    /* PI Output */
    const int16_t piOutput = UTIL_Shr15(atpll->kpOut + atpll->kiOut);

    /* The scale down of stator voltages/flux is compensated by decreasing the
     * shift count by 1 */
    atpll->omegaPIOut = UTIL_SatShrS16(__builtin_mulss(piOutput, pmotor->keInverse), 
                                    (MCAF_MOTOR_KE_INVERSE_Q - 1));

    /* Filter on PI Output */
    int16_t filtErr = atpll->omegaPIOut - atpll->omegaPIOutFilt;
    atpll->omegaPIOutFiltStateVar += __builtin_mulss(filtErr, KFILTER_PI_OMEGA);
    atpll->omegaPIOutFilt = UTIL_Shr15(atpll->omegaPIOutFiltStateVar);

    /* Calculate Speed and angle */
    atpll->omega = (int16_t)(atpll->omegaPIOutFilt + speedRef);
    atpll->thetaStateVar += __builtin_mulss(atpll->omega, ATPLL_NORM_DELTAT);

    atpll->thetaElectrical = UTIL_Shr15(atpll->thetaStateVar);
    
    /* Filter on estimated speed for outer speed loop */
    filtErr = atpll->omega - atpll->omegaElectrical;
    
    atpll->omegaFiltStateVar += __builtin_mulss(filtErr, KFILTER_OMEGA);
    
    atpll->omegaElectrical = UTIL_Shr15(atpll->omegaFiltStateVar);
}

/*******************************************************************************/
