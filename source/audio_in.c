/*****************************************************************************
* File Name    : audio_in.c
*
* Description  : This file provides the source code to implement the audio 
*                input task.
*
* Note         : See README.md
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
#include "audio_in.h"
#include "cyhal.h"
#include "cybsp.h"

#include "rtos.h"
#include "limits.h"

#include <stdio.h>
#include "USB_MSD.h"


/*****************************************************************************
* Macros
*****************************************************************************/
#define PDM_DECIMATION_RATE         32
#define PDM_PCM_BUFFER_SIZE         32768U

#define NOTIFY_BUTTON_PRESS         0x1
#define NOTIFY_PCM_DATA             0x2

#define DEBOUNCE_DELAY_MS           250

#ifndef CYBSP_PDM_DATA
    #define CYBSP_PDM_DATA          CYBSP_A5
#endif
#ifndef CYBSP_PDM_CLK
    #define CYBSP_PDM_CLK           CYBSP_A4
#endif


/*****************************************************************************
* Function prototypes
*****************************************************************************/

static void audio_in_pdm_pcm_handler(void *handler_arg, cyhal_pdm_pcm_event_t event);
static void audio_in_button_handler(void *handler_arg, cyhal_gpio_event_t event);


/*****************************************************************************
* Global variables
*****************************************************************************/
cyhal_gpio_callback_data_t cb_data =
{
    .callback     = audio_in_button_handler,
    .callback_arg = NULL
};

cyhal_pdm_pcm_t pdm_pcm;
uint8_t pdm_pcm_buf_0[PDM_PCM_BUFFER_SIZE];
uint8_t pdm_pcm_buf_1[PDM_PCM_BUFFER_SIZE];
volatile bool pdm_pcm_toggle = false;
volatile uint32_t num_samples_transfer;


/*****************************************************************************
* Function Name: audio_in_pdm_pcm_handler
******************************************************************************
* Summary:
*  Send the PDM/PCM notification to the audio task and prepare to read data
*  again.
*
* Parameters:
*  handler_arg: not used
*  event: event that occurred
*
* Return:
*  None
*
*****************************************************************************/
static void audio_in_pdm_pcm_handler(void *handler_arg, cyhal_pdm_pcm_event_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t *buf = (pdm_pcm_toggle) ? pdm_pcm_buf_0 : pdm_pcm_buf_1;

    CY_UNUSED_PARAMETER(handler_arg);
    CY_UNUSED_PARAMETER(event);

    /* Schedule the next read */
    cyhal_pdm_pcm_read_async(&pdm_pcm, buf, PDM_PCM_BUFFER_SIZE/2);

    pdm_pcm_toggle = !pdm_pcm_toggle;

    xTaskNotifyFromISR(rtos_audio_task,
                       NOTIFY_PCM_DATA,
                       eSetBits,
                       &xHigherPriorityTaskWoken);

    /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
    should be performed to ensure the interrupt returns directly to the highest
    priority task.  The macro used for this purpose is dependent on the port in
    use and may be called portEND_SWITCHING_ISR(). */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*****************************************************************************
* Function Name: audio_in_task
******************************************************************************
* Summary:
*  Handle the audio recordings.
*
* Parameters:
*  arg: not used
*
* Return:
*  None
*
*****************************************************************************/
void audio_in_task(void *arg)
{
    bool is_recording = false;
    bool first_time   = false;
    uint32_t sample_rate;
    bool     sample_mode;
    uint32_t notification_bits;
    cyhal_pdm_pcm_cfg_t pdm_pcm_cfg;

    if (NULL != rtos_fs_mutex)
    {
        /* Check if other tasks are accessing the file system by trying to obtain the mutex */
        if (pdTRUE == xSemaphoreTake(rtos_fs_mutex, portMAX_DELAY))
        {
            /* Init the audio file system. Force format if user button is pressed */
            audio_fs_init(cyhal_gpio_read(CYBSP_USER_BTN) == false);

            /* Wait till button is released */
            while (false == cyhal_gpio_read(CYBSP_USER_BTN))
            {
                vTaskDelay(pdMS_TO_TICKS(1));
            };

            /* List all the record files */
            audio_fs_list();

            /* Release the mutex so that file system can be used by other tasks */
            xSemaphoreGive(rtos_fs_mutex);
        }
    }

    /* Registering button event and enable it */
    cyhal_gpio_register_callback(CYBSP_USER_BTN, &cb_data);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL, CYHAL_ISR_PRIORITY_DEFAULT, true);

    while (1)
    {
        xTaskNotifyWait(0, ULONG_MAX, &notification_bits, portMAX_DELAY);

        /* Handle button presses */
        if (notification_bits & NOTIFY_BUTTON_PRESS)
        {
            /* Add some delay to debounce the button presses */
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));

            /* Re-enable the button event */
            cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL, CYHAL_ISR_PRIORITY_DEFAULT, true);

            /* Check if recording */
            if (is_recording)
            {
                printf("-- Record ended ---\n\r");

                /* Stop the PDM/PCM interface */
                cyhal_pdm_pcm_stop(&pdm_pcm);
                cyhal_pdm_pcm_free(&pdm_pcm);

                /* Turn off LED*/
                cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);

                /* Save the file */
                audio_fs_save();

                is_recording = false;

                /* Release the mutex so that file system can be used by other tasks */
                xSemaphoreGive(rtos_fs_mutex);
            }
            else /* Not recording */
            {
                if (NULL != rtos_fs_mutex)
                {
                    /* Check if other tasks are accessing the file system by trying to obtain the mutex */
                    if (pdTRUE == xSemaphoreTake(rtos_fs_mutex, portMAX_DELAY))
                    {
                        /* If not recording, create a new record */
                        if (audio_fs_new_record())
                        {
                            cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);

                            printf("\n\rStarted a new record with:\n\r");

                            /* Get configuration */
                            audio_fs_get_config(&sample_rate, &sample_mode);

                            /* Populate the config structure */
                            pdm_pcm_cfg.mode = (sample_mode) ? CYHAL_PDM_PCM_MODE_STEREO : CYHAL_PDM_PCM_MODE_LEFT;
                            pdm_pcm_cfg.decimation_rate = PDM_DECIMATION_RATE;
                            pdm_pcm_cfg.sample_rate = sample_rate;
                            pdm_pcm_cfg.word_length = 16;
                            pdm_pcm_cfg.right_gain = CYHAL_PDM_PCM_MAX_GAIN;
                            pdm_pcm_cfg.left_gain = CYHAL_PDM_PCM_MAX_GAIN;

                            cyhal_pdm_pcm_init(&pdm_pcm, CYBSP_PDM_DATA, CYBSP_PDM_CLK, NULL, &pdm_pcm_cfg);
                            cyhal_pdm_pcm_register_callback(&pdm_pcm, &audio_in_pdm_pcm_handler, NULL);
                            cyhal_pdm_pcm_enable_event(&pdm_pcm, CYHAL_PDM_PCM_ASYNC_COMPLETE, CYHAL_ISR_PRIORITY_DEFAULT, true);
                            cyhal_pdm_pcm_start(&pdm_pcm);

                            pdm_pcm_toggle = false;

                            /* Initial read request */
                            cyhal_pdm_pcm_read_async(&pdm_pcm, pdm_pcm_buf_0, PDM_PCM_BUFFER_SIZE/2);

                            is_recording = true;
                            first_time = true;
                        }
                        else
                        {
                            /* Failed creating a record, turn off the LED */
                            cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);

                            /* Release the mutex so that file system can be used by other tasks */
                            xSemaphoreGive(rtos_fs_mutex);

                        }
                    }
                }
            }
        }

        /* Handle PDM/PCM data */
        if (notification_bits & NOTIFY_PCM_DATA)
        {
            /* Ignore the first batch of data to avoid noise in the PDM/PCM output */
            if (first_time)
            {
                first_time = false;
            }
            else
            {
                if (is_recording)
                {
                    uint8_t *buf = (pdm_pcm_toggle) ? pdm_pcm_buf_0 : pdm_pcm_buf_1;

                    /* Write to the record file */
                    if (false == audio_fs_write(buf, PDM_PCM_BUFFER_SIZE))
                    {
                        /* Error writing to the file, stop PDM/PCM interface */
                        cyhal_pdm_pcm_stop(&pdm_pcm);
                        cyhal_pdm_pcm_free(&pdm_pcm);

                        /* Turn off LED */
                        cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);

                        is_recording = false;

                        /* Release the mutex so that file system can be used by other tasks */
                        xSemaphoreGive(rtos_fs_mutex);
                    }
                }
            }
        }
    }
}

/*****************************************************************************
* Function Name: audio_in_button_callback
******************************************************************************
* Summary:
*  Send the button notification to the task.
*
* Parameters:
*  handler_arg: not used
*  event: event that occurred
*
* Return:
*  None
*
*****************************************************************************/
static void audio_in_button_handler(void *handler_arg, cyhal_gpio_event_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    CY_UNUSED_PARAMETER(handler_arg);
    CY_UNUSED_PARAMETER(event);

    xTaskNotifyFromISR(rtos_audio_task,
                       NOTIFY_BUTTON_PRESS,
                       eSetBits,
                       &xHigherPriorityTaskWoken );

    /* Disable button event to avoid multiple triggers to the button */
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL, CYHAL_ISR_PRIORITY_DEFAULT, false);

    /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
    should be performed to ensure the interrupt returns directly to the highest
    priority task.  The macro used for this purpose is dependent on the port in
    use and may be called portEND_SWITCHING_ISR(). */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/* [] END OF FILE */
