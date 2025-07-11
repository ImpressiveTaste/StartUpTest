/* This is an autogenerated file from a template. Do not edit this file as it will be overwritten.*/
/**
 *
 * atpll_params.h
 *
 * Component: commutation
 */ /*
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
 *
 ******************************************************************************/
#ifndef __ATPLL_PARAMS_H
#define __ATPLL_PARAMS_H

#ifdef  __cplusplus
extern "C" {
#endif

/* Normalized sampling time for theta calculation */
#define ATPLL_NORM_DELTAT                     655      // Q15(  0.01999) =  +49.97253 useconds    =  +50.00000 useconds    - 0.0549%

/* Normalized sampling time for estimator Ki output */
#define ATPLL_NORM_DELTAT_KI                   69      // Q15(  0.00211) =   +1.67567 useconds    =   +1.66667 useconds    + 0.5404%

/* Filter constant that is used for filtering the PI Omega Output */
#define KFILTER_PI_OMEGA                    16384      // Q15(  0.50000) =  +10.00000 krad/s      =  +10.00000 krad/s      + 0.0000%

/* Filter constant that is used for filtering the estimated Omega */
#define KFILTER_OMEGA                         748      // Q15(  0.02283) = +456.54297 rad/s       = +456.62100 rad/s       - 0.0171%

/* ATPLL estimator Kp gain */
#define ATPLL_KP_GAIN                       15565      // Q13(  1.90002) =   +1.90002             =   +1.90000             + 0.0013%
#define ATPLL_KP_GAIN_Q                        13

#ifdef  __cplusplus
}
#endif

#endif // __ATPLL_PARAMS_H
