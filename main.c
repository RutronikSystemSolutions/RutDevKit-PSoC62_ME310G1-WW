/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RutDevKit-PSoC62_I2C_Scanner
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*  Created on: 2021-05-27
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Jonavos g. 30, Kaunas 44262, Lithuania
*  Author: GDR
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "StringCommandParser.h"
#include "modem.h"

#define UART_RX_PRIO				2
#define RTC_INTERRUPT_PRIORITY		0

static void isr_scp_timer(void *callback_arg, cyhal_timer_event_t event);
static cy_rslt_t scp_timer_init(void);
static cy_rslt_t ardu_uart_init();
void ardu_uart_isr(void *handler_arg, cyhal_uart_event_t event);
static cy_rslt_t adc_hw_init(void);
void dma_interrupt(void);
cy_rslt_t Hibernate(cyhal_rtc_t *obj, uint32_t seconds);
void handle_error(void);

/*SCP Timer object */
cyhal_timer_t scp_timer;

/*Arduino UART object*/
cyhal_uart_t ardu_uart;

uint8_t aRxBuffer;

cyhal_rtc_t rtc_obj;

/* This flag set in the DMA interrupt handler */
volatile bool dmaIntrTriggered = false;
/* DMA Transfer complete/error flags sent in DMA interrupt Handler*/
volatile uint8_t adc_dma_error;   /* ADCDma error flag */
volatile uint8_t adc_dma_done;    /* ADCDma done flag */
/* DMA interrupt configuration structure */
/* Source is set to DW 0 and Priority as 7 */
const cy_stc_sysint_t intRxDma_cfg =
{
        .intrSrc      = cpuss_interrupts_dw0_28_IRQn,
        .intrPriority = 7
};
/* Buffer to store data from SAR using DMA */
int16_t aADCdata[2] = {0};

TaskHandle_t URCReceiverTaskHandle = NULL;
SemaphoreHandle_t cmd_mutex = NULL;

int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
    	handle_error();
    }

    __enable_irq();

    /*Initialize LEDs*/
    result = cyhal_gpio_init( LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    result = cyhal_gpio_init( LED2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    /*Initialize ME310G1 Modem GPIOs*/
    result = cyhal_gpio_init( ARDU_IO3, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1); /*Power Control for SMPS*/
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    result = cyhal_gpio_init( ARDU_IO4, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLDOWN, 0); /*USB PD Alarm Input*/
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    result = cyhal_gpio_init( ARDU_IO5, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0); /*USB PD Reset Control*/
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    result = cyhal_gpio_init( ARDU_IO6, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0); /*Modem Reset Control*/
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    result = cyhal_gpio_init( ARDU_IO7, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0); /*Modem Wake-Up Control*/
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    result = cyhal_gpio_init( ARDU_IO8, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0); /*Modem ON/OFF Control*/
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    /*Enable debug output via KitProg UART*/
    result = cy_retarget_io_init( KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    printf("\x1b[2J\x1b[;H");
    printf("RutDevKit-PSoC62 ME310G1-WW Application.\r\n");

    /* Initialize RTC */
    result = cyhal_rtc_init(&rtc_obj);
    if (CY_RSLT_SUCCESS != result)
    {handle_error();}

    /*Initialize String Command Parser Timer*/
    result = scp_timer_init();
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    /*Initialize The Arduino UART*/
    result = ardu_uart_init();
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    /*Configure the ADC hardware*/
    result = adc_hw_init();
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    /* Start the TCPWM Timer triggering the ADC*/
    Cy_TCPWM_TriggerStart_Single(ADC_TIMER_HW, ADC_TIMER_NUM);

    /* Create a mutex for the SCP. */
    cmd_mutex = xSemaphoreCreateMutex();
    if( cmd_mutex == NULL )
    {CY_ASSERT(0);}

    /*Create UCR Receiver Task*/
    xTaskCreate(URCReceiverTask, "urc_task", configMINIMAL_STACK_SIZE*8, NULL, configMAX_PRIORITIES - 4, &URCReceiverTaskHandle);
    if(URCReceiverTaskHandle == NULL)
    {
    	printf("Error: could not create URC Receiver Task.\r\n");
    	handle_error();
    }

    vTaskStartScheduler();
    /* RTOS scheduler exited */
    /* Halt the CPU if scheduler exits */
    CY_ASSERT(0);
}

static void isr_scp_timer(void *callback_arg, cyhal_timer_event_t event)
{
    (void) callback_arg;
    (void) event;

    /*Is called every ten milliseconds*/
    SCP_Tick(10);
}

static cy_rslt_t scp_timer_init(void)
{
	 cy_rslt_t result;
	 const cyhal_timer_cfg_t scp_cfg =
	 {
	     .compare_value = 0,                 /* Timer compare value, not used */
	     .period = 99,                      /* Defines the timer period */
	     .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
	     .is_compare = false,                /* Don't use compare mode */
	     .is_continuous = true,              /* Run the timer indefinitely */
	     .value = 0                          /* Initial value of counter */
	 };

	 result = cyhal_timer_init(&scp_timer, NC, NULL);
	 if (result != CY_RSLT_SUCCESS)
	 {return result;}

	 result = cyhal_timer_configure(&scp_timer, &scp_cfg);
	 if (result != CY_RSLT_SUCCESS)
	 {return result;}

	 result = cyhal_timer_set_frequency(&scp_timer, 10000);
	 if (result != CY_RSLT_SUCCESS)
	 {return result;}

	 cyhal_timer_register_callback(&scp_timer, isr_scp_timer, NULL);

	 cyhal_timer_enable_event(&scp_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT, 3, true);

	 result =  cyhal_timer_start(&scp_timer);
	 if (result != CY_RSLT_SUCCESS)
	 {return result;}

	 return result;
}


void ardu_uart_isr(void *handler_arg, cyhal_uart_event_t event)
{
    (void) handler_arg;
    if ((event & CYHAL_UART_IRQ_TX_ERROR) == CYHAL_UART_IRQ_TX_ERROR)
    {
        /* An error occurred in Tx */
        /* Insert application code to handle Tx error */
    }
    else if ((event & CYHAL_UART_IRQ_RX_ERROR) == CYHAL_UART_IRQ_RX_ERROR)
    {
        /* An error occurred in Rx */
        /* Insert application code to handle Rx error */
    	cyhal_uart_read_async(&ardu_uart, (void *)&aRxBuffer, 1);
    }
    else if ((event & CYHAL_UART_IRQ_RX_DONE) == CYHAL_UART_IRQ_RX_DONE)
    {
        /* All Rx data has been received */
    	SCP_ByteReceived(aRxBuffer);
    	cyhal_uart_read_async(&ardu_uart, (void *)&aRxBuffer, 1);
    }
}

static cy_rslt_t ardu_uart_init(void)
{
	cy_rslt_t result;

    /* Initialize the UART configuration structure */
    const cyhal_uart_cfg_t uart_config =
    {
        .data_bits = 8,
        .stop_bits = 1,
        .parity = CYHAL_UART_PARITY_NONE,
        .rx_buffer = NULL,
        .rx_buffer_size = 0
    };

    /* Initialize the UART Block */
    result = cyhal_uart_init(&ardu_uart, ARDU_TX, ARDU_RX, NULL, &uart_config);
	if (result != CY_RSLT_SUCCESS)
	{return result;}

	/*Connect internal pull-up resistors*/
	cyhal_gpio_configure(ARDU_RX, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP);

    /* The UART callback handler registration */
    cyhal_uart_register_callback(&ardu_uart, ardu_uart_isr, NULL);

    /* Enable required UART events */
    cyhal_uart_enable_event(&ardu_uart, (cyhal_uart_event_t)(CYHAL_UART_IRQ_RX_ERROR | CYHAL_UART_IRQ_TX_ERROR | CYHAL_UART_IRQ_RX_DONE), UART_RX_PRIO, true);

	return result;
}

cy_rslt_t adc_hw_init(void)
{
	cy_en_tcpwm_status_t tcpwm_res;
	cy_en_sysanalog_status_t aref_res;
	cy_en_sar_status_t sar_res;
	cy_en_dma_status_t dma_res;

	/*TCPWM*/
	tcpwm_res = Cy_TCPWM_Counter_Init(ADC_TIMER_HW, ADC_TIMER_NUM, &ADC_TIMER_config);
	if(tcpwm_res == CY_TCPWM_SUCCESS)
	{
		Cy_TCPWM_Counter_Enable(ADC_TIMER_HW, ADC_TIMER_NUM);
	}
	else {goto return_err;}

	/*VREF*/
	aref_res = Cy_SysAnalog_Init(&ADC_VREF_config);
	if(aref_res == CY_SYSANALOG_SUCCESS)
	{
		Cy_SysAnalog_Enable();
	}
	else {goto return_err;}

	/*SAR*/
	sar_res = Cy_SAR_Init(SAR_ADC_HW, &SAR_ADC_config);
	if(sar_res == CY_SAR_SUCCESS)
	{
		Cy_SAR_Enable(SAR_ADC_HW);
	}
	else {goto return_err;}

	/*DMA*/
	/* Initialize descriptor 0 */
	dma_res = Cy_DMA_Descriptor_Init(&cpuss_0_dw0_0_chan_28_Descriptor_0,&cpuss_0_dw0_0_chan_28_Descriptor_0_config);
	if(dma_res != CY_DMA_SUCCESS) {goto return_err;}

	/* Initialize the channel and associate the descriptor to it */
	dma_res = Cy_DMA_Channel_Init(cpuss_0_dw0_0_chan_28_HW, cpuss_0_dw0_0_chan_28_CHANNEL,&cpuss_0_dw0_0_chan_28_channelConfig);
	if(dma_res != CY_DMA_SUCCESS) {goto return_err;}

	/*Get the pointer to the SAR measurement data*/
	uint32_t *sar_data_ptr = (uint32_t*)&SAR_CHAN_RESULT(SAR_ADC_HW, 0);

    /* Set DMA Source and Destination address */
    Cy_DMA_Descriptor_SetSrcAddress(&cpuss_0_dw0_0_chan_28_Descriptor_0, sar_data_ptr);
    Cy_DMA_Descriptor_SetDstAddress(&cpuss_0_dw0_0_chan_28_Descriptor_0, aADCdata);

    /*Set DMA Descriptor */
    Cy_DMA_Channel_SetDescriptor(cpuss_0_dw0_0_chan_28_HW, cpuss_0_dw0_0_chan_28_CHANNEL, &cpuss_0_dw0_0_chan_28_Descriptor_0);

    /* Initialize and enable the interrupt from SAR DMA */
    Cy_SysInt_Init(&intRxDma_cfg, &dma_interrupt);
    NVIC_EnableIRQ((IRQn_Type)intRxDma_cfg.intrSrc);

    /* Enable DMA interrupt source. */
    Cy_DMA_Channel_SetInterruptMask(cpuss_0_dw0_0_chan_28_HW, cpuss_0_dw0_0_chan_28_CHANNEL, CY_DMA_INTR_MASK);

    /* Enable DMA Channel and DMA Block to start descriptor execution process */
    Cy_DMA_Channel_Enable(cpuss_0_dw0_0_chan_28_HW, cpuss_0_dw0_0_chan_28_CHANNEL);
    Cy_DMA_Enable(cpuss_0_dw0_0_chan_28_HW);

	return CY_RSLT_SUCCESS;

	return_err:
	return CY_RSLT_TYPE_ERROR;
}

void dma_interrupt(void)
{
    dmaIntrTriggered = true;

    /* Check interrupt cause to capture errors. */
    if (CY_DMA_INTR_CAUSE_COMPLETION == Cy_DMA_Channel_GetStatus(cpuss_0_dw0_0_chan_28_HW, cpuss_0_dw0_0_chan_28_CHANNEL))
    {
        adc_dma_done = 1;
    }
    else if((CY_DMA_INTR_CAUSE_COMPLETION != Cy_DMA_Channel_GetStatus(cpuss_0_dw0_0_chan_28_HW,cpuss_0_dw0_0_chan_28_CHANNEL)) &&
                                                (CY_DMA_INTR_CAUSE_CURR_PTR_NULL !=
                                                Cy_DMA_Channel_GetStatus(cpuss_0_dw0_0_chan_28_HW, cpuss_0_dw0_0_chan_28_CHANNEL)))
    {
        /* DMA error occurred while ADC operations */
        adc_dma_error = Cy_DMA_Channel_GetStatus(cpuss_0_dw0_0_chan_28_HW, cpuss_0_dw0_0_chan_28_CHANNEL);
    }

    /* Clear SAR DMA interrupt */
    Cy_DMA_Channel_ClearInterrupt(cpuss_0_dw0_0_chan_28_HW, cpuss_0_dw0_0_chan_28_CHANNEL);
}

/*Hibernate Function*/
cy_rslt_t Hibernate(cyhal_rtc_t *obj, uint32_t seconds)
{
	cy_rslt_t result;
	struct tm date_time;
    time_t UnixTime = {0};

	/*Set the time fields to trigger an alarm event.*/
	cyhal_alarm_active_t alarm_active =
	{
		    alarm_active.en_sec = 1,
		    alarm_active.en_min = 1,
		    alarm_active.en_hour = 1,
		    alarm_active.en_day = 1,
		    alarm_active.en_date = 1,
		    alarm_active.en_month = 1
	};

	result = cyhal_rtc_read(obj, &date_time);
    if (CY_RSLT_SUCCESS != result)
    {return result;}

    /*Get time in Unix format*/
    UnixTime = mktime(&date_time);

    /*Add time in seconds here*/
    UnixTime = UnixTime + seconds;
    /*Convert back*/
    date_time = *(localtime(&UnixTime));

    /*Set the alarm*/
    result = cyhal_rtc_set_alarm(obj, &date_time, alarm_active);
    if (CY_RSLT_SUCCESS != result)
    {return result;}

    /* Enable the alarm event to trigger an interrupt */
    cyhal_rtc_enable_event(obj, CYHAL_RTC_ALARM, RTC_INTERRUPT_PRIORITY, true);

    /* Put the system into a hibernate state. Most of the system will stop. */
    result = cyhal_syspm_hibernate(CYHAL_SYSPM_HIBERNATE_RTC_ALARM);
    if (CY_RSLT_SUCCESS != result)
    {return result;}


    /*Should never reach this*/
    return result;
}

void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

/* [] END OF FILE */
