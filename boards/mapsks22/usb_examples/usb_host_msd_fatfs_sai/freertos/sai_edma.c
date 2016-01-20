/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "board.h"
#include "fsl_dmamux.h"
#include "fsl_sai_edma.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "fsl_wm8960.h"
#include "fsl_device_registers.h"
#include "fsl_common.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "type.h"
#include "ff.h"
#include "diskio.h"
#include "lcd.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* SAI and I2C instance and clock */
#define DEMO_SAI I2S0
#define DEMO_I2C LPI2C0
#define DEMO_SAI_CLKSRC kCLOCK_CoreSysClk
#define DEMO_I2C_CLKSRC kCLOCK_McgIrc48MClk
#define DEMO_CODEC_WM8960 (1)
#define EXAMPLE_DMA (DMA0)
#define EXAMPLE_CHANNEL (0)
#define EXAMPLE_SAI_TX_SOURCE (13)
#define OVER_SAMPLE_RATE (384U)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void callback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData);
extern usb_file_type g_usb_file;
/*******************************************************************************
 * Variables
 ******************************************************************************/
sai_edma_handle_t txHandle = {0};
edma_handle_t dmaHandle = {0};
#if defined(DEMO_CODEC_WM8960)
wm8960_handle_t codecHandle = {0};
#else
sgtl_handle_t codecHandle = {0};
#endif

#if defined(FSL_FEATURE_SOC_LPI2C_COUNT) && (FSL_FEATURE_SOC_LPI2C_COUNT)
lpi2c_master_handle_t i2cHandle = {0};
#else
i2c_master_handle_t i2cHandle = {0};
#endif
volatile bool isFinished = false;

/*******************************************************************************
 * Code
 ******************************************************************************/

static void callback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
{
    isFinished = true;
}

/*!
 * @brief Main function
 */
extern EventGroupHandle_t xEventMusic;
EventBits_t uxBits;
extern uint32_t total_count;

void SAI_Task(void* param)
{
    sai_config_t config;
    uint32_t mclkSourceClockHz = 0U;
    sai_transfer_format_t format;
    sai_transfer_t xfer;
    edma_config_t dmaConfig = {0};
#if defined(FSL_FEATURE_SOC_LPI2C_COUNT) && (FSL_FEATURE_SOC_LPI2C_COUNT)
    lpi2c_master_config_t i2cConfig = {0};
#else
    i2c_master_config_t i2cConfig = {0};
#endif
    uint32_t i2cSourceClock;
    uint32_t temp = 0;

    CLOCK_SetLpi2c0Clock(1);

    ////PRINTF("SAI example started!\n\r");

    /* Create EDMA handle */
    EDMA_GetDefaultConfig(&dmaConfig);
    EDMA_Init(EXAMPLE_DMA, &dmaConfig);
    EDMA_CreateHandle(&dmaHandle, EXAMPLE_DMA, EXAMPLE_CHANNEL);

    DMAMUX_Init(DMAMUX0);
    DMAMUX_SetSource(DMAMUX0, EXAMPLE_CHANNEL, EXAMPLE_SAI_TX_SOURCE);
    DMAMUX_EnableChannel(DMAMUX0, EXAMPLE_CHANNEL);

    /* Init SAI module */
    SAI_TxGetDefaultConfig(&config);
    SAI_TxInit(DEMO_SAI, &config);

    /* Configure the audio format */
    format.bitWidth = kSAI_WordWidth8bits;
    format.channel = 0U;
    format.sampleRate_Hz = kSAI_SampleRate22050Hz;
#if defined(FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER) && (FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER)
    format.masterClockHz = OVER_SAMPLE_RATE * format.sampleRate_Hz;
#else
    format.masterClockHz = CLOCK_GetFreq(DEMO_SAI_CLKSRC);
#endif
    format.protocol = config.protocol;
    format.stereo = kSAI_Stereo;
#if defined(FSL_FEATURE_SAI_FIFO_COUNT) && (FSL_FEATURE_SAI_FIFO_COUNT > 1)
    format.watermark = FSL_FEATURE_SAI_FIFO_COUNT / 2U;
#endif

    /* Configure Sgtl5000 I2C */
    codecHandle.base = DEMO_I2C;
    codecHandle.i2cHandle = &i2cHandle;
    i2cSourceClock = CLOCK_GetFreq(DEMO_I2C_CLKSRC);

#if defined(FSL_FEATURE_SOC_LPI2C_COUNT) && (FSL_FEATURE_SOC_LPI2C_COUNT)
    LPI2C_MasterGetDefaultConfig(&i2cConfig);
    LPI2C_MasterInit(DEMO_I2C, &i2cConfig, i2cSourceClock);
    LPI2C_MasterTransferCreateHandle(DEMO_I2C, &i2cHandle, NULL, NULL);
#else
    I2C_MasterGetDefaultConfig(&i2cConfig);
    I2C_MasterInit(DEMO_I2C, &i2cConfig, i2cSourceClock);
    I2C_MasterTransferCreateHandle(DEMO_I2C, &i2cHandle, NULL, NULL);
#endif

#if defined(DEMO_CODEC_WM8960)
    WM8960_Init(&codecHandle, NULL);
    WM8960_ConfigDataFormat(&codecHandle, format.masterClockHz, format.sampleRate_Hz, format.bitWidth);
#else
    /* Use default settings for sgtl5000 */
    SGTL_Init(&codecHandle, NULL);
    /* Configure codec format */
    SGTL_ConfigDataFormat(&codecHandle, format.masterClockHz, format.sampleRate_Hz, format.bitWidth);
#endif

    SAI_TransferTxCreateHandleEDMA(DEMO_SAI, &txHandle, callback, NULL, &dmaHandle);

    mclkSourceClockHz = CLOCK_GetFreq(DEMO_SAI_CLKSRC);
    SAI_TransferTxSetFormatEDMA(DEMO_SAI, &txHandle, &format, mclkSourceClockHz, format.masterClockHz);

    uint8_t header[50];

    UINT count_read;
    FIL file;
    FILINFO fileInfo;
    FRESULT fatfsCode;

    while(1)
    {
        do
        {
            uxBits = xEventGroupWaitBits(xEventMusic,(1<<0),pdTRUE,pdFALSE,100);
        }while((uxBits&1) == 0);

        if (g_usb_file.loaded_file_index != g_usb_file.current_file_index)
        {
            
            PRINTF("File name to read: %s\n\r",g_usb_file.file_name[g_usb_file.current_file_index]);

            fatfsCode = f_open(&file, g_usb_file.file_name[g_usb_file.current_file_index],FA_READ);

            uint8_t *dest = (uint8_t *)(0x1FFFC000);
            
            PRINTF("Reading music .");

            f_read(&file, header, 44, &count_read); // read header
            LCD_PutString(0,56,"--- Loading Music ---");

            total_count = 0;
            while(count_read>0 && total_count<0x4000)
            {
                fatfsCode = f_read(&file, dest, 32, &count_read);

                if (0 == count_read)
                    break; 
                
                dest+=32;
                total_count+=32;
                
                PRINTF(".");
         
                vTaskDelay(5);
            }
            f_close(&file);
            
            PRINTF("\r\n");

            g_usb_file.loaded_file_index = g_usb_file.current_file_index;
        }
        
        LCD_PutString(0,56,"--- Ready to Play ---");
    
        
        PRINTF("Play Music ...\r\n");
        /*  xfer structure */
        xfer.data = (uint8_t *)(0x1FFFC000);
        xfer.dataSize = total_count;
        isFinished = false;
        
        SAI_TransferSendEDMA(DEMO_SAI, &txHandle, &xfer);
        /* Wait until finished */
        while (isFinished != true)
        {
            vTaskDelay(100);
        }
        
        vTaskDelay(100);
        
    }

#if defined(DEMO_CODEC_WM8960)
    WM8960_Deinit(&codecHandle);
#else
    SGTL_Deinit(&codecHandle);
#endif

#if defined(FSL_FEATURE_SOC_LPI2C_COUNT) && (FSL_FEATURE_SOC_LPI2C_COUNT)
    LPI2C_MasterDeinit(DEMO_I2C);
#else
    I2C_MasterDeinit(DEMO_I2C);
#endif
    PRINTF("\n\r SAI example finished!\n\r ");
    while (1)
    {
        vTaskDelay(100);
    }
}
