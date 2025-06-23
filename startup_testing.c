/**
 * startup_testing.c
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

#include <stdint.h>
#include <stdbool.h>
#include "mcapi.h"
#include "startup_testing.h"
#include "util.h"
#include "mcapi_types.h"
#include "board_service.h"
#include "hal/hardware_access_functions.h"

STARTUP_TEST_APP_DATA app;
void APP_TimerCallback(void);

void APP_StartupTestApplicationInitialize(volatile MCAPI_MOTOR_DATA *apiData, MCAF_BOARD_DATA *pboard)
{
    STARTUP_TEST_APP_DATA *appData = &app;

    appData->apiData = apiData;
    appData->configTestMotorVelocity = APP_STARTUP_SPEED;
    appData->configTestCount = APP_START_TEST_COUNT;
    appData->configHoldTime = APP_STARTUP_HOLD_TIME;
    appData->configStopOnFail = true;
    appData->configSpinDownTime = APP_SPIN_DOWN_TIME;
    
    appData->testTimer = 0;
    appData->testEnable = false;
    appData->testStop = false;
    appData->testStartupTimer = 0;
    
    appData->statStartupTimeAccumulated = 0;
    appData->statStartupTimeAverage = 0;
    appData->statStartupTimeMax = 0;
    appData->statStartupTimeMin = 32767;
    appData->statTestCount = 0;
    appData->statTestFailCount = 0;
    appData->statTestPassCount = 0;
    appData->statTestTimeout = 0;
    
    appData->statReset = false;

    HAL_TMR_TICK_SetCallbackFunction(APP_TimerCallback);
}

void APP_StartupTestApplicationStep(STARTUP_TEST_APP_DATA *app)
{
    volatile MCAPI_MOTOR_DATA *api = app->apiData;

    if (app->testEnable)
    {
        MCAPI_MOTOR_STATE motorState = MCAPI_OperatingStatusGet(api);
        switch (motorState)
        {
            case MCAPI_MOTOR_STOPPED:
            {
                if (app->testStop || app->statTestCount >= app->configTestCount)
                {
                    if (app->statTestPassCount > 0)
                    {
                        app->statStartupTimeAverage = 
                                __builtin_divud(app->statStartupTimeAccumulated, 
                                                app->statTestPassCount);
                    }

                    app->testEnable = false;
                    app->testStop = false;
                }
                else
                {
                    /* start the motor after spin down time */
                    if (app->testTimer >= app->configSpinDownTime)
                    {
                        app->testTimer = 0;
                        MCAPI_VelocityReferenceSet(api, app->configTestMotorVelocity);
                        MCAPI_MotorStart(api);
                        app->statTestCount++;
                    }
                    else
                    {
                        app->testTimer++;
                    }
                }
                break;
            }

            case MCAPI_MOTOR_STARTING:
            {
                if (app->testStartupTimer > APP_STARTUP_TIMOUT)
                {
                    MCAPI_MotorStop(api);
                    
                    app->testStartupTimer = 0;
                    app->statTestTimeout++;
                    app->statTestFailCount++;
                }
                else
                {
                    app->testStartupTimer++;
                }
                
                app->testTimer = 0;
                break;
            }
            
            case MCAPI_MOTOR_RUNNING:
            {
                /* stop the motor after it reaches the 
                 * preset test velocity and stays there
                 * for some time without faults */
                app->motorVelocityMeasured = MCAPI_VelocityMeasuredGet(api);
                if (app->motorVelocityMeasured >= app->configTestMotorVelocity || app->testTimer > 0)
                {
                    if (app->testTimer >= app->configHoldTime)
                    {
                        app->testTimer = 0;
                        MCAPI_MotorStop(api);

                        app->statTestPassCount++;
                        
                        /* update the test stats */
                        app->statStartupTimeAccumulated += app->testStartupTimer;
                        if (app->testStartupTimer > app->statStartupTimeMax)
                        {
                            app->statStartupTimeMax = app->testStartupTimer;
                        }
                        if (app->testStartupTimer < app->statStartupTimeMin)
                        {
                            app->statStartupTimeMin = app->testStartupTimer;
                        }
                        app->testStartupTimer = 0;
                    }
                    else
                    {
                        app->testTimer++;
                    }
                }
                else
                {
                    app->testStartupTimer++;
                }
                
                break;
            }
            
            case MCAPI_MOTOR_STOPPING:
            {
                app->testTimer = 0;
                break;
            }

            case MCAPI_MOTOR_FAULT:
            {
                uint16_t faultFlags = MCAPI_FaultStatusGet(api);
                MCAPI_FaultStatusClear(api, faultFlags);
                
                if (app->configStopOnFail && app->testStartupTimer > 0)
                {
                    app->testEnable = 0;
                }
                
                if (app->statTestCount > 0 && app->testStartupTimer > 0)
                {
                    app->testStartupTimer = 0;
                    app->statTestFailCount++;
                }
                app->testTimer = 0;
                break;
            }

            case MCAPI_MOTOR_DIAGSTATE:
            {
                /* do nothing */
                break;
            }
        }
    }
    
    if (app->statReset)
    {
        app->statStartupTimeAccumulated = 0;
        app->statStartupTimeAverage = 0;
        app->statStartupTimeMax = 0;
        app->statStartupTimeMin = 32767;
        app->statTestCount = 0;
        app->statTestFailCount = 0;
        app->statTestPassCount = 0;
        app->statTestTimeout = 0;
        
        app->statReset = false;
    }
}

/**
 * This is an application owned timer the user is responsible for configuring. 
 * The application timer period needs to match the value set in motorBench Customize page.
 */
void APP_TimerCallback(void)
{
    MCAF_BoardServiceTasks(app.pboard);
    APP_StartupTestApplicationStep(&app);
}
