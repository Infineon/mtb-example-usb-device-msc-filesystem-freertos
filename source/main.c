/*****************************************************************************
* File Name        : main.c
*
* Description      : This is the source code for the USB device mass storage 
*                    file system Example for ModusToolbox.
*
* Related Document : See README.md
*
******************************************************************************
* Copyright 2022-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*****************************************************************************/
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "rtos.h"
#include "USB.h"
#include "USB_MSD.h"
#include "usb_comm.h"
#include "audio_in.h"


/*****************************************************************************
* Global Variables
*****************************************************************************/
TaskHandle_t rtos_usb_task;
TaskHandle_t rtos_audio_task;
SemaphoreHandle_t rtos_fs_mutex;


/*****************************************************************************
* Function Name: usb_task
******************************************************************************
* Summary:
*  Initialize and handle the USB communication.
*
* Parameters:
*  arg: not used
*
* Return:
*  None
*
*****************************************************************************/
static void usb_task(void *arg)
{
    if (NULL != rtos_fs_mutex)
    {
        /* Check if other tasks are accessing the file system by trying to obtain the mutex */
        if (pdTRUE == xSemaphoreTake(rtos_fs_mutex, portMAX_DELAY))
        {
            /* Initialize and enumerate the USB */
            usb_comm_init();

            /* Release the mutex so that file system can be used by other tasks */
            xSemaphoreGive(rtos_fs_mutex);
        }
    }
    while (1)
    {
        if (NULL != rtos_fs_mutex)
        {
            /* Check if other tasks are accessing the file system by trying to obtain the mutex */
            if (pdTRUE == xSemaphoreTake(rtos_fs_mutex, portMAX_DELAY))
            {
                /* check for configuration */
                if ((USBD_GetState() & (USB_STAT_CONFIGURED | USB_STAT_SUSPENDED)) == USB_STAT_CONFIGURED)
                {
                    USBD_MSD_Poll();
                }

                /* Release the mutex so that file system can be used by other tasks */
                xSemaphoreGive(rtos_fs_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/*****************************************************************************
* Function Name: main
******************************************************************************
* Summary:
*  This is the main function for CM4 CPU. It does...
*    1. Initializes the BSP, retarget_io, User Button and LED configuration.
*    2. Creates Audio Task, USB Task, and file system semaphore.
*    3. Starts the FreeRTOS scheduler.
*
* Parameters:
*  void
*
* Return:
*  int
*
*****************************************************************************/
int main(void)
{
    cy_rslt_t result;
    BaseType_t task_return;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the User Button */
    result = cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the User LED */
    result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    /* GPIO init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("******************"
           " emUSB Device: Mass storage file system "
           "****************** \r\n\n");

    /* Create the RTOS tasks */
    task_return = xTaskCreate(audio_in_task, "Audio Task",
                              RTOS_STACK_DEPTH, NULL, AUDIO_TASK_PRIORITY,
                              &rtos_audio_task);
    if (pdPASS != task_return) CY_ASSERT(0);

    task_return = xTaskCreate(usb_task, "USB Task",
                              RTOS_STACK_DEPTH, NULL, USB_TASK_PRIORITY,
                              &rtos_usb_task);
    if (pdPASS != task_return) CY_ASSERT(0);

     /* Create the file system semaphore */
    rtos_fs_mutex = xSemaphoreCreateMutex();

    /* Start the RTOS Scheduler */
    vTaskStartScheduler();

    /* Should never get there */
    printf("APP_LOG: Error: FreeRTOS doesn't start\r\n");

    return 0;
}

/* [] END OF FILE */
