/*
 * Copyright (c) 2013-2017 Nuand LLC
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

/*
 * CDC EEM bulk path on GPIF threads RX1/TX2 (PIB sockets 1 and 2).
 * Must only be started while GPIF is in GPIF_CONFIG_RF_LINK mode.
 */
#include <cyu3error.h>
#include <cyu3usb.h>
#include "eem.h"

#define THIS_FILE LOGGER_ID_EEM_C

static CyU3PDmaChannel glChHandleEEMUtoP;   /* EP 0x03 OUT → PIB socket 2 (GPIF TX2) */
static CyU3PDmaChannel glChHandleEEMPtoU;   /* PIB socket 1 (GPIF RX1) → EP 0x83 IN  */

static uint8_t EEM_status_bits[] = {
    [BLADE_RF_EEM_EP_PRODUCER] = 0,
    [BLADE_RF_EEM_EP_CONSUMER] = 0,
};

void NuandEEMLinkStart(void)
{
    uint16_t size = 0;
    CyU3PEpConfig_t epCfg;
    CyU3PDmaChannelConfig_t dmaCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed();

    switch (usbSpeed) {
        case CY_U3P_FULL_SPEED:  size = 64;   break;
        case CY_U3P_HIGH_SPEED:  size = 512;  break;
        case CY_U3P_SUPER_SPEED: size = 1024; break;
        default:
            LOG_ERROR(usbSpeed);
            CyFxAppErrorHandler(CY_U3P_ERROR_FAILURE);
            return;
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

    CyU3PMemSet((uint8_t *)&dmaCfg, 0, sizeof(dmaCfg));
    dmaCfg.size           = size * 4;   /* 4 max-packets: fits one full Ethernet frame */
    dmaCfg.count          = BLADE_DMA_BUF_COUNT;
    dmaCfg.dmaMode        = CY_U3P_DMA_MODE_BYTE;
    dmaCfg.notification   = 0;
    dmaCfg.cb             = 0;
    dmaCfg.prodHeader     = 0;
    dmaCfg.prodFooter     = 0;
    dmaCfg.consHeader     = 0;
    dmaCfg.prodAvailCount = 0;

    /* host → FPGA: USB socket 3 (EP 0x03 OUT) → PIB socket 2 (GPIF TX2) */
    dmaCfg.prodSckId = BLADE_RF_EEM_EP_PRODUCER_USB_SOCKET;
    dmaCfg.consSckId = CY_U3P_PIB_SOCKET_2;
    apiRetStatus = CyU3PDmaChannelCreate(&glChHandleEEMUtoP,
                                         CY_U3P_DMA_TYPE_AUTO, &dmaCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* FPGA → host: PIB socket 1 (GPIF RX1) → USB socket 3 (EP 0x83 IN) */
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

    apiRetStatus = CyU3PDmaChannelSetXfer(&glChHandleEEMPtoU, BLADE_DMA_TX_SIZE);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }
}

void NuandEEMLinkStop(void)
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
}

static CyU3PReturnStatus_t NuandEEMLinkResetEndpoint(uint8_t endpoint)
{
    CyU3PReturnStatus_t status = CY_U3P_ERROR_BAD_ARGUMENT;

    switch (endpoint) {
        case BLADE_RF_EEM_EP_PRODUCER:
            status = ClearDMAChannel(endpoint, &glChHandleEEMUtoP, BLADE_DMA_TX_SIZE);
            break;
        case BLADE_RF_EEM_EP_CONSUMER:
            status = ClearDMAChannel(endpoint, &glChHandleEEMPtoU, BLADE_DMA_TX_SIZE);
            break;
    }
    return status;
}

static CyBool_t NuandEEMLinkHaltEndpoint(CyBool_t set, uint16_t endpoint)
{
    CyBool_t isHandled = CyFalse;
    CyU3PReturnStatus_t status = CY_U3P_ERROR_BAD_ARGUMENT;

    switch (endpoint) {
        case BLADE_RF_EEM_EP_PRODUCER:
        case BLADE_RF_EEM_EP_CONSUMER:
            isHandled = !set;
            EEM_status_bits[endpoint] = set;
            status = NuandEEMLinkResetEndpoint((uint8_t)endpoint);
            break;
    }

    if (status == CY_U3P_SUCCESS) {
        CyU3PUsbStall(endpoint, CyFalse, CyTrue);
        if (!set) {
            CyU3PUsbAckSetup();
        }
    }

    return isHandled && (status == CY_U3P_SUCCESS);
}

static CyBool_t NuandEEMLinkHalted(uint16_t endpoint, uint8_t *data)
{
    CyBool_t isHandled = CyFalse;

    switch (endpoint) {
        case BLADE_RF_EEM_EP_PRODUCER:
        case BLADE_RF_EEM_EP_CONSUMER:
            *data = EEM_status_bits[endpoint];
            isHandled = CyTrue;
            break;
    }

    return isHandled;
}

const struct NuandApplication NuandEEMLink = {
    .start          = NuandEEMLinkStart,
    .stop           = NuandEEMLinkStop,
    .halt_endpoint  = NuandEEMLinkHaltEndpoint,
    .halted         = NuandEEMLinkHalted,
    .reset_endpoint = NuandEEMLinkResetEndpoint,
};
