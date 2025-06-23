/**
 * startup.c
 *
 * Module for startup and open loop to closed loop transition
 * 
 * Component: state machine
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

#include <stdint.h>
#include <stdbool.h>
#include "startup.h"
#include "parameters/timing_params.h"
#include "parameters/startup_params.h"
#include "ui.h"
#include "error_codes.h"
#include "util.h"

void MCAF_StartupTransitioningInit(MCAF_MOTOR_STARTUP_DATA *pstartup)
{
    pstartup->velocityThreshold[0] = STARTUP_ACCEL0_VELOCITY_THRESHOLD;
    pstartup->velocityThreshold[1] = STARTUP_ACCEL1_VELOCITY_THRESHOLD;
    pstartup->acceleration[0] = STARTUP_ACCELERATION0;
    pstartup->acceleration[1] = STARTUP_ACCELERATION1;
    pstartup->alignTime = STARTUP_ALIGN_TIME;
    pstartup->holdTime = STARTUP_HOLD_TIME;
    pstartup->rampupAngle = STARTUP_RAMPUP_ANGLE;
    pstartup->alignAngleDelta = STARTUP_ALIGN_ANGLE_DELTA;
    pstartup->thetaDelta = Q15_THETADELTA;
    pstartup->iAmplitude = MCAF_STARTUP_CURRENT;
    pstartup->dt = STARTUP_DELTA_T_FACTOR;
    pstartup->dtAcceleration = STARTUP_DELTA_T_FACTOR_ACCELERATION;
    
    pstartup->activeDamping.iqmax = STARTUP_DAMPING_IQMAX;
    pstartup->activeDamping.k = STARTUP_DAMPING_GAIN;
    pstartup->activeDamping.velocityThreshold = STARTUP_DAMPING_THRESHOLD;
    
    pstartup->referenceFrameAlign.thetaThreshold = MCAF_STARTUP_REF_FRAME_ALIGN_THRESHOLD;
    pstartup->referenceFrameAlign.omegaOffsetMagnitude = MCAF_STARTUP_REF_FRAME_ALIGN_FREQUENCY;
    pstartup->referenceFrameAlign.shiftCount = MCAF_STARTUP_REF_FRAME_ALIGN_SHIFT;
}

inline static int16_t limit32(int32_t x, int16_t limitLo, int16_t limitHi)
{
    if (x > limitHi)
    {
        return limitHi;
    }
    else if (x < limitLo)
    {
        return limitLo;
    }
    else
    {
        return x;
    }
}

/**
 * add damping to nominal current
 *
 * @param pstartup startup data
 * @param nominalCurrent nominal current without damping
 * @return modified current
 */
inline static int16_t MCAF_StartupAddDamping(const MCAF_MOTOR_STARTUP_DATA *pstartup,
        int16_t nominalCurrent)
{
    const int16_t omega_error = pstartup->omegaElectrical.x16.hi
                                - pstartup->omegaElectricalEstimated;
    const int32_t iqdamping_q32 = UTIL_mulss(omega_error, pstartup->activeDamping.k)
                                  >> STARTUP_DAMPING_SHIFT;
    const int16_t iqdamping = limit32(iqdamping_q32,
                                        -pstartup->activeDamping.iqmax,
                                        pstartup->activeDamping.iqmax);
    return nominalCurrent + iqdamping;
}

enum MCAF_STARTUP_DAMPING { MSD_ENABLED = 1, MSD_CONDITIONAL = 0 };

/**
 *
 * @param pstartup startup data
 * @param nominalCurrent nominal current
 * @param enableDampingAlways whether to enable damping always
 * @return current command
 */
inline static int16_t MCAF_StartupCalcIq(const MCAF_MOTOR_STARTUP_DATA *pstartup,
        int16_t nominalCurrent,
        enum MCAF_STARTUP_DAMPING dampingPreference)
{
    int16_t result = nominalCurrent;
    if ((dampingPreference == MSD_ENABLED) ||
            UTIL_Abs16Approx(pstartup->omegaElectrical.x16.hi)
              > pstartup->activeDamping.velocityThreshold)
    {
        result = MCAF_StartupAddDamping(pstartup, nominalCurrent);
    }
    return result;
}

/**
 * Accelerate velocity, check whether velocity threshold is reached
 *
 * @param pstartup startup data
 * @param directedAcceleration acceleration (+ or -)
 * @param velocityThreshold threshold for completion
 * @return true if threshold is reached
 */
inline static bool MCAF_StartupAccelVelocityIsComplete(MCAF_MOTOR_STARTUP_DATA *pstartup,
        int16_t directedAcceleration,
        int16_t velocityThreshold)
{
    const bool thresholdReached =
        UTIL_Abs16(pstartup->omegaElectrical.x16.hi) >= velocityThreshold;
    if (!thresholdReached)
    {
        pstartup->omegaElectrical.x32 +=
                UTIL_mulss(pstartup->dtAcceleration, directedAcceleration);
    }
    return thresholdReached;
}

/**
 * Sets dq vector y to vector x rotated by a small amount
 * @param px pointer to input
 * @param py pointer to output
 * @param shift shift count
 * @param direction +1 for positive rotation, -1 for negative rotation
 */
inline static void dq_rotate(const MC_DQ_T *px, MC_DQ_T *py, int shift, int direction)
{
    py->d = px->d - direction * (px->q >> shift);
    py->q = px->q + direction * (px->d >> shift);
}

inline static bool startup_ref_frame_align(MCAF_MOTOR_STARTUP_REF_FRAME_ALIGN *palign,
         int16_t thetaError,
         const MC_DQ_T *pidqcmd, MC_DQ_T *pidqcmd_next)
{
    if (thetaError > palign->thetaThreshold)
    {
        palign->omegaOffset = -palign->omegaOffsetMagnitude;
        dq_rotate(pidqcmd, pidqcmd_next, palign->shiftCount, 1);
        return false;
    }
    else if (thetaError < -palign->thetaThreshold)
    {
        palign->omegaOffset = palign->omegaOffsetMagnitude;
        dq_rotate(pidqcmd, pidqcmd_next, palign->shiftCount, -1);
        return false;
    }
    else
    {
        palign->omegaOffset = 0;
        return true;
    }                                
}

#define MCAF_STARTUP_RESET_CYCLES 1

void MCAF_StartupTransitioningStep(MCAF_MOTOR_STARTUP_DATA *pstartup,
        MC_DQ_T *idqCommand, int16_t direction)
{
    const MC_DQ_T idqcmd = *idqCommand;
    MC_DQ_T idqcmd_next = idqcmd;
    
    switch (pstartup->state)
    {
        case SSM_START:
            /* startup in reset: stay in this state until re-enabled */
            if (pstartup->enable)
            {
                if (pstartup->counter < MCAF_STARTUP_RESET_CYCLES)
                {
                    ++pstartup->counter;
                }
                else
                {
                    pstartup->state = SSM_CURRENT_RAMPUP;
                }
            }
            pstartup->iNominal = pstartup->iAmplitude * direction;
            break;
        case SSM_CURRENT_RAMPUP:
        {
            idqcmd_next.q = UTIL_LimitSlewRateSymmetrical(
                    pstartup->iNominal,                /* input */
                    idqcmd.q,                           /* previousOutput */
                    pstartup->iRampupLimit);           /* limit */

            if (idqcmd.q == idqcmd_next.q)
            {
                pstartup->counter = 0;
                pstartup->state = SSM_ALIGN;
                pstartup->thetaElectrical.x16.hi += pstartup->alignAngleDelta;
            }
            break;
        }
        case SSM_ALIGN:
        {
            if (pstartup->counter < pstartup->alignTime)
            {
                ++pstartup->counter;
            }
            else if (!pstartup->delayRequest)
            {
                pstartup->state = SSM_ACCEL0;
            }
            break;
        }
        case SSM_ACCEL0:
        {
            int16_t directed_accel = pstartup->acceleration[0] * direction;
            if (MCAF_StartupAccelVelocityIsComplete(pstartup, directed_accel,
                    pstartup->velocityThreshold[0]))
            {
                pstartup->state = SSM_ACCEL1;
            }
            idqcmd_next.q = MCAF_StartupCalcIq(pstartup, pstartup->iNominal, MSD_CONDITIONAL);
            break;
        }
        case SSM_ACCEL1:
        {
            int16_t directed_accel = pstartup->acceleration[1] * direction;
            if (MCAF_StartupAccelVelocityIsComplete(pstartup, directed_accel,
                    pstartup->velocityThreshold[1]))
            {
                pstartup->counter = 0;
                pstartup->state = SSM_HOLD;
            }
            idqcmd_next.q = MCAF_StartupCalcIq(pstartup, pstartup->iNominal, MSD_CONDITIONAL);
            break;
        }
        case SSM_HOLD:
        {
            idqcmd_next.q = MCAF_StartupCalcIq(pstartup, pstartup->iNominal, MSD_ENABLED);
            if (pstartup->counter < pstartup->holdTime)
            {
                ++pstartup->counter;
            }
            else
            {                
                if (!pstartup->delayRequest)
                {
                    pstartup->counter = 0;
                    pstartup->state = SSM_REF_FRAME_ALIGN;
                    pstartup->torqueCmd32 = (int32_t)idqcmd_next.q << 16;
                }
            }
            break;
        }
        case SSM_REF_FRAME_ALIGN:
        {
            if (startup_ref_frame_align(&pstartup->referenceFrameAlign,
                                        MCAF_StartupGetThetaError(pstartup),
                                        &idqcmd,
                                        &idqcmd_next))
            {
                pstartup->complete = true;
                pstartup->state = SSM_COMPLETE;
            }
            break;
        }
        case SSM_COMPLETE:
            /* startup complete: do nothing */
            break;
        case SSM_INACTIVE:
            /* do nothing in test modes */
            break;
        default:
        {
            MCAF_UiFlashErrorCodeForever(ERR_INVALID_STARTUP_FSM_STATE);
        }
    }
    
    *idqCommand = idqcmd_next;
    pstartup->delayRequest = false;
}
