/**
 * atpll.h
 * 
 * Hosts components of the AT PLL estimator
 * 
 * Component: commutation
 */

/* *********************************************************************
 *
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

#ifndef __ATPLL_H
#define __ATPLL_H

#include <stdint.h>
#include "motor_control_types.h"
#include "units.h"
#include "commutation/common.h"
#include "startup_types.h"
#include "foc_types.h"

#ifdef __cplusplus
extern "C" {
#endif
    
/**
 * State variables for AT PLL estimator
 */
typedef struct tagMCAF_ESTIMATOR_ATPLL
{
    /*
     * Runtime-adjustable parameters
     *
     * These values are typically set once, at startup,
     * but may be adjusted using real-time diagnostic tools.
     */
    int16_t kpGain;             /** ATPLL Kp Gain */
    int16_t kiIn;               /** input to I controller */
    
    /*
     * State variables
     *
     * The state of the estimator at any given instant 
     * is completely determined by its state variables.
     *
     * At each new step of the estimator, the state variables can theoretically
     * be expressed as a deterministic function of the following values:
     *
     *   - state variables at the previous step
     *   - runtime-adjustable parameters
     *   - inputs to the estimator step function
     */
    int32_t omegaPIOutFiltStateVar;     /** state variable for PI Controller output filter */
    int32_t omegaFiltStateVar;      /** state variable for estimated velocity*/
    int32_t thetaStateVar;          /** state variable for rotor angle */
    MCAF_U_ANGLE_ELEC thetaElectrical;           /** Calculated rotor angle*/
    MCAF_U_VELOCITY_ELEC omegaElectrical;        /** Estimated velocity - scaled and filtered */
    
    /*
     * Auxiliary variables
     *
     * These values can be derived from the state variables
     * using memory-less calculations. 
     *
     * They are usually output or intermediate variables,
     * and are typically retained in RAM to support data logging 
     * with real-time diagnostic tools.
     */
    MCAF_U_DIMENSIONLESS_SINCOS sincos;          /** Sine and cosine component of calculated angle */
    MCAF_U_VOLTAGE_DQ_Q14 esdq;                      /** Calculated BEMF in d-q reference frame */
    MCAF_U_VOLTAGE_Q14 esdError;    /** Back EMF Error which is forced to zero*/
    MCAF_U_VOLTAGE_Q14 esdNeg;      /** negative Esd */
    int16_t omegaPIOut;         /** PI Output */
    int16_t omegaPIOutFilt;     /** PI Output Filtered */
    int16_t omega;              /** Estimated Speed */
    int32_t kpOut;              /** P controller output */
    int32_t kiOut;              /** I controller output */
    
} MCAF_ESTIMATOR_ATPLL_T;


/**
 * Initializes ATPLL state variables on reset.
 * Summary: Initializes ATPLL state variables.
 * @param atpll ATPLL state variable structure
 * @param pmotor motor parameters
 */
void MCAF_EstimatorAtPllInit(MCAF_ESTIMATOR_ATPLL_T *atpll, const MCAF_MOTOR_PARAMETERS_T *pmotor);

/**
 * Initializes ATPLL state variables prior to starting motor.
 * 
 * @param atpll ATPLL state variable structure
 */
void MCAF_EstimatorAtPllStartupInit(MCAF_ESTIMATOR_ATPLL_T *atpll);

/**
 * Executes one control step of the ATPLL estimator.
 * Summary: Executes one control step of the ATPLL estimator.
 * @param atpll ATPLL state variable structure
 * @param pinput Common input signals (e.g. stationary-frame voltage and current)
 * @param speed ref speed reference
 */
void MCAF_EstimatorAtPllStep(MCAF_ESTIMATOR_ATPLL_T *atpll, 
                const MCAF_STANDARD_INPUT_SIGNALS_T *pinput,
                const MCAF_MOTOR_PARAMETERS_T *pmotor,
                const int16_t speedRef);

/**
 * Returns commutation angle
 * 
 * @param atpll state
 * @return commutation angle
 */
inline static MCAF_U_ANGLE_ELEC MCAF_EstimatorAtPllCommutationAngle(const MCAF_ESTIMATOR_ATPLL_T *atpll)
{
    return atpll->thetaElectrical;
}

/**
 * Returns electrical frequency
 * 
 * @param atpll state
 * @return electrical frequency
 */
inline static MCAF_U_VELOCITY_ELEC MCAF_EstimatorAtPllElectricalFrequency(const MCAF_ESTIMATOR_ATPLL_T *atpll)
{
    return atpll->omegaElectrical;
}

/**
 * Determine whether startup delay is requested
 * 
 * @param atpll state
 * @param startupStatus startup status
 * @return whether a startup delay is requested
 */
inline static bool MCAF_EstimatorAtPllStartupDelayRequested(const MCAF_ESTIMATOR_ATPLL_T *atpll, MCAF_STARTUP_STATUS_T startupStatus)
{
    return false;
}

#ifdef __cplusplus
}
#endif

#endif /* __ATPLL_H */
