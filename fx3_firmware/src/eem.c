/*
 * Copyright (c) 2026 Nuand LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <cyu3error.h>
#include <cyu3gpio.h>
#include <cyu3usb.h>
#include "bladeRF.h"
#include "cyfxbladeRF.h"
#include "eem.h"
#include "gpif.h"

#define THIS_FILE LOGGER_ID_EEM_C

/* USB -> FPGA (host writes EEM frames into PIB socket 2 / TX2) */
static CyU3PDmaChannel glChHandleEEMUtoP;

/* FPGA -> USB (FPGA writes EEM frames into PIB socket 1 / RX1) */
static CyU3PDmaChannel glChHandleEEMPtoU;

void NuandEEMStart(void)
{
    uint16_t size = 0;
    CyU3PEpConfig_t epCfg;
    CyU3PDmaChannelConfig_t dmaCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed();

    NuandAllowSuspend(CyFalse);

    /* Bring up the IO matrix in 32-bit GPIF mode (warm reconfig). The same
     * settings RFlink uses, since EEMlink uses the same physical pins. */
    NuandGPIOReconfigure(CyTrue, CyTrue);

    /* Pulse SYS_RST so the FPGA's user logic comes up cleanly with EEM as
     * its only active GPIF peer. */
    CyU3PGpioSetValue(GPIO_SYS_RST, CyTrue);
    CyU3PGpioSetValue(GPIO_RX_EN, CyFalse);
    CyU3PGpioSetValue(GPIO_TX_EN, CyFalse);
    CyU3PGpioSetValue(GPIO_SYS_RST, CyFalse);

    apiRetStatus = NuandConfigureGpif(GPIF_CONFIG_EEMLINK);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    switch (usbSpeed) {
        case CY_U3P_FULL_SPEED:  size = 64;   break;
        case CY_U3P_HIGH_SPEED:  size = 512;  break;
        case CY_U3P_SUPER_SPEED: size = 1024; break;
        default:
            LOG_ERROR(usbSpeed);
            CyFxAppErrorHandler(CY_U3P_ERROR_FAILURE);
            break;
    }

    CyU3PMemSet((uint8_t *)&epCfg, 0, sizeof(epCfg));
    epCfg.enable   = CyTrue;
    epCfg.epType   = CY_U3P_USB_EP_BULK;
    epCfg.burstLen = 1;
    epCfg.streams  = 0;
    epCfg.pcktSize = size;

    apiRetStatus = CyU3PSetEpConfig(BLADE_RF_EEM_EP_PRODUCER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    apiRetStatus = CyU3PSetEpConfig(BLADE_RF_EEM_EP_CONSUMER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* USB OUT -> PIB socket 2 (TX2 to FPGA) */
    CyU3PMemSet((uint8_t *)&dmaCfg, 0, sizeof(dmaCfg));
    dmaCfg.size           = size * 2;
    dmaCfg.count          = 4;
    dmaCfg.prodSckId      = BLADE_RF_EEM_EP_PRODUCER_USB_SOCKET;
    dmaCfg.consSckId      = CY_U3P_PIB_SOCKET_2;
    dmaCfg.dmaMode        = CY_U3P_DMA_MODE_BYTE;
    dmaCfg.notification   = 0;
    dmaCfg.cb             = 0;
    dmaCfg.prodHeader     = 0;
    dmaCfg.prodFooter     = 0;
    dmaCfg.consHeader     = 0;
    dmaCfg.prodAvailCount = 0;

    apiRetStatus = CyU3PDmaChannelCreate(&glChHandleEEMUtoP,
                                         CY_U3P_DMA_TYPE_AUTO, &dmaCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* PIB socket 1 (RX1 from FPGA) -> USB IN */
    dmaCfg.prodSckId = CY_U3P_PIB_SOCKET_1;
    dmaCfg.consSckId = BLADE_RF_EEM_EP_CONSUMER_USB_SOCKET;
    apiRetStatus = CyU3PDmaChannelCreate(&glChHandleEEMPtoU,
                                         CY_U3P_DMA_TYPE_AUTO, &dmaCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    CyU3PUsbFlushEp(BLADE_RF_EEM_EP_PRODUCER);
    CyU3PUsbFlushEp(BLADE_RF_EEM_EP_CONSUMER);

    apiRetStatus = CyU3PDmaChannelSetXfer(&glChHandleEEMUtoP, BLADE_DMA_TX_SIZE);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    apiRetStatus = CyU3PDmaChannelSetXfer(&glChHandleEEMPtoU, BLADE_DMA_RX_SIZE);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }
}

void NuandEEMStop(void)
{
    CyU3PEpConfig_t epCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    CyU3PUsbFlushEp(BLADE_RF_EEM_EP_PRODUCER);
    CyU3PUsbFlushEp(BLADE_RF_EEM_EP_CONSUMER);

    CyU3PDmaChannelDestroy(&glChHandleEEMUtoP);
    CyU3PDmaChannelDestroy(&glChHandleEEMPtoU);

    CyU3PMemSet((uint8_t *)&epCfg, 0, sizeof(epCfg));
    epCfg.enable = CyFalse;

    apiRetStatus = CyU3PSetEpConfig(BLADE_RF_EEM_EP_PRODUCER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    apiRetStatus = CyU3PSetEpConfig(BLADE_RF_EEM_EP_CONSUMER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    NuandConfigureGpif(GPIF_CONFIG_DISABLED);

    /* Park the FPGA in reset so it doesn't continue driving GPIF lines
     * while no FX3-side peer is listening. */
    CyU3PGpioSetValue(GPIO_SYS_RST, CyTrue);

    NuandAllowSuspend(CyTrue);
}
