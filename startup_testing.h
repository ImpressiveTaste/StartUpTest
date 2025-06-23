/**
 * startup_testing.h
 *
 * Motor startup testing application for MCAF
 * 
 * Component: main application
 */
/*
 *
 * Motor Control Application Framework
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

#ifndef STARTUP_TESTING_H
#define	STARTUP_TESTING_H

#include <stdint.h>
#include <stdbool.h>
#include "mcapi_types.h"
#include "mcapi.h"
#include "board_service.h"

#ifdef	__cplusplus
extern "C" {
#endif

#define APP_STARTUP_SPEED       12288
#define APP_START_TEST_COUNT    500
#define APP_STARTUP_HOLD_TIME   1000     // in ms
#define APP_STARTUP_TIMOUT      5000     // in ms
#define APP_SPIN_DOWN_TIME      100      // in ms
    
typedef struct tagAPPLICATION_DATA
{
    int16_t motorVelocityMeasured; /** motor velocity measured by MCAF */
    
    uint16_t testTimer;
    bool testEnable;
    bool testStop;
    uint16_t testStartupTimer;
    
    int16_t  configTestMotorVelocity;
    uint16_t configTestCount;
    uint16_t configSpinDownTime;
    uint16_t configHoldTime;
    bool     configStopOnFail;
    
    uint32_t statStartupTimeAccumulated;
    uint16_t statStartupTimeMax;
    uint16_t statStartupTimeMin;
    uint16_t statStartupTimeAverage;
    uint16_t statTestCount;
    uint16_t statTestPassCount;
    uint16_t statTestFailCount;
    uint16_t statTestTimeout;
    bool statReset;

    volatile MCAPI_MOTOR_DATA *apiData;
    MCAF_BOARD_DATA *pboard;
} STARTUP_TEST_APP_DATA;

/**
 * Initializes the application state variables.
 * @param apiData MCAPI data
 * @param pboard board state data
 */
void APP_StartupTestApplicationInitialize(volatile MCAPI_MOTOR_DATA *apiData, MCAF_BOARD_DATA *pboard);

/**
 * Executes one step of the motor startup testing application.
 * @param app application data
 */
void APP_StartupTestApplicationStep(STARTUP_TEST_APP_DATA *app);


#ifdef	__cplusplus
}
#endif

#endif	/* STARTUP_TESTING_H */

