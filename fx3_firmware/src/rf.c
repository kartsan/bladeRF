/*
 * Copyright (c) 2013 Nuand LLC
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
#include <cyu3uart.h>
#include "gpif.h"
#include "rf.h"

#define THIS_FILE LOGGER_ID_RF_C

static CyU3PDmaChannel glChHandlebladeRFUtoUART;   /* DMA Channel for U2P transfers */
static CyU3PDmaChannel glChHandlebladeRFUARTtoU;   /* DMA Channel for U2P transfers */

static CyU3PDmaChannel glChHandleUtoP;
static CyU3PDmaChannel glChHandlePtoU;

static CyU3PDmaChannel glChHandleEEMUtoP;
static CyU3PDmaChannel glChHandleEEMPtoU;

static int loopback = 0;
static int loopback_when_created;

/* Tracks whether the shared GPIF (FX3 PCLK + the GPIF II state machine that the
 * RF sample and EEM datapaths both ride on) and the EEM endpoints/DMA have been
 * brought up. Owned by the device-ready lifecycle, not by the RF sample
 * interface. */
static CyBool_t glGpifLinkUp = CyFalse;

void NuandRFLinkLoopBack(int lp) {
    loopback = lp;
}

int NuandRFLinkGetLoopBack() {
    return loopback;
}

/* Max bulk packet size for the current USB speed. */
static uint16_t NuandUsbMaxPktSize(void)
{
    switch (CyU3PUsbGetSpeed()) {
        case CY_U3P_FULL_SPEED:  return 64;
        case CY_U3P_HIGH_SPEED:  return 512;
        case CY_U3P_SUPER_SPEED: return 1024;
        default:
            LOG_ERROR(CY_U3P_ERROR_FAILURE);
            CyFxAppErrorHandler(CY_U3P_ERROR_FAILURE);
            return 0;
    }
}

static void UartBridgeStart(void)
{
    uint16_t size = 0;
    CyU3PEpConfig_t epCfg;
    CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed();

    CyU3PDmaChannelConfig_t dmaCfg;
    CyU3PUartConfig_t uartConfig;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;


    /* Initialize the UART for printing debug messages */
    apiRetStatus = CyU3PUartInit();
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Set UART configuration */
    CyU3PMemSet ((uint8_t *)&uartConfig, 0, sizeof (uartConfig));
    uartConfig.baudRate = CY_U3P_UART_BAUDRATE_4M; // CY_U3P_UART_BAUDRATE_115200;
    uartConfig.stopBit = CY_U3P_UART_ONE_STOP_BIT;
    uartConfig.parity = CY_U3P_UART_NO_PARITY;
    uartConfig.txEnable = CyTrue;
    uartConfig.rxEnable = CyTrue;
    uartConfig.flowCtrl = CyFalse;
    uartConfig.isDma = CyTrue;

    apiRetStatus = CyU3PUartSetConfig (&uartConfig, NULL);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Set UART Tx and Rx transfer Size to infinite */
    apiRetStatus = CyU3PUartTxSetBlockXfer(0xFFFFFFFF);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyFxAppErrorHandler(apiRetStatus);
    }

    apiRetStatus = CyU3PUartRxSetBlockXfer(0xFFFFFFFF);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyFxAppErrorHandler(apiRetStatus);
    }


    /* Determine max packet size based on USB speed */
    switch (usbSpeed)
    {
        case CY_U3P_FULL_SPEED:
            size = 64;
            break;

        case CY_U3P_HIGH_SPEED:
            size = 512;
            break;

        case CY_U3P_SUPER_SPEED:
            size = 1024;
            break;

        default:
            LOG_ERROR(usbSpeed);
            CyFxAppErrorHandler (CY_U3P_ERROR_FAILURE);
            break;
    }

    CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
    epCfg.enable = CyTrue;
    epCfg.epType = CY_U3P_USB_EP_BULK;
    epCfg.burstLen = 1;
    epCfg.streams = 0;
    epCfg.pcktSize = size;

    /* Producer endpoint configuration */
    apiRetStatus = CyU3PSetEpConfig(BLADE_UART_EP_PRODUCER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Consumer endpoint configuration */
    apiRetStatus = CyU3PSetEpConfig(BLADE_UART_EP_CONSUMER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    CyU3PMemSet((uint8_t *)&dmaCfg, 0, sizeof(dmaCfg));
    dmaCfg.size  = 16;
    dmaCfg.count = 10;
    dmaCfg.prodSckId = CY_U3P_UIB_SOCKET_PROD_2;
    dmaCfg.consSckId = CY_U3P_LPP_SOCKET_UART_CONS;
    dmaCfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
    dmaCfg.notification = 0;
    dmaCfg.cb = 0;
    dmaCfg.prodHeader = 0;
    dmaCfg.prodFooter = 0;
    dmaCfg.consHeader = 0;
    dmaCfg.prodAvailCount = 0;

    apiRetStatus = CyU3PDmaChannelCreate(&glChHandlebladeRFUtoUART,
            CY_U3P_DMA_TYPE_AUTO, &dmaCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    dmaCfg.prodSckId = CY_U3P_LPP_SOCKET_UART_PROD;
    dmaCfg.consSckId = CY_U3P_UIB_SOCKET_CONS_2;
    apiRetStatus = CyU3PDmaChannelCreate(&glChHandlebladeRFUARTtoU,
            CY_U3P_DMA_TYPE_AUTO, &dmaCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Flush the endpoint memory */
    CyU3PUsbFlushEp(BLADE_UART_EP_PRODUCER);
    CyU3PUsbFlushEp(BLADE_UART_EP_CONSUMER);

    /* Set DMA channel transfer size */
    apiRetStatus = CyU3PDmaChannelSetXfer(&glChHandlebladeRFUtoUART, BLADE_DMA_TX_SIZE);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    apiRetStatus = CyU3PDmaChannelSetXfer(&glChHandlebladeRFUARTtoU, BLADE_DMA_TX_SIZE);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

     /* Set UART Tx and Rx transfer Size to infinite */
     apiRetStatus = CyU3PUartTxSetBlockXfer(0xFFFFFFFF);
     if (apiRetStatus != CY_U3P_SUCCESS) {
         CyFxAppErrorHandler(apiRetStatus);
     }

     apiRetStatus = CyU3PUartRxSetBlockXfer(0xFFFFFFFF);
     if (apiRetStatus != CY_U3P_SUCCESS) {
         CyFxAppErrorHandler(apiRetStatus);
     }
}

static void UartBridgeStop(void)
{
    CyU3PEpConfig_t epCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Flush the endpoint memory */
    CyU3PUsbFlushEp(BLADE_UART_EP_PRODUCER);
    CyU3PUsbFlushEp(BLADE_UART_EP_CONSUMER);

    /* Destroy the channel */
    CyU3PDmaChannelDestroy(&glChHandlebladeRFUARTtoU);
    CyU3PDmaChannelDestroy(&glChHandlebladeRFUtoUART);

    /* Disable endpoints. */
    CyU3PMemSet((uint8_t *)&epCfg, 0, sizeof (epCfg));
    epCfg.enable = CyFalse;

    /* Producer endpoint configuration. */
    apiRetStatus = CyU3PSetEpConfig(BLADE_UART_EP_PRODUCER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    apiRetStatus = CyU3PSetEpConfig(BLADE_UART_EP_CONSUMER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }
    CyU3PUartDeInit();
}

/* Bring up the shared GPIF -- this is what starts the FX3 PCLK and the GPIF II
 * state machine that the RF sample and EEM datapaths both ride on -- together
 * with the EEM endpoints and EEM DMA channels (PIB sockets 1 & 2).
 *
 * This is owned by the device-ready lifecycle, NOT by the RF sample interface.
 * It is the only path that loads the RF_LINK GPIF configuration: reloading the
 * GPIF once the link is up would reset the state machine shared by RF and EEM.
 * Safe to call repeatedly. */
void NuandGpifLinkStart(void)
{
    uint16_t size;
    CyU3PEpConfig_t epCfg;
    CyU3PDmaChannelConfig_t dmaCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    if (glGpifLinkUp) {
        return;
    }

    size = NuandUsbMaxPktSize();

    /* The IO matrix must be re-asserted unconditionally: NuandFlashInit()
     * corrupts it, so it cannot be assumed valid here. */
    NuandGPIOReconfigure(CyTrue, CyTrue);

    apiRetStatus = NuandConfigureGpif(GPIF_CONFIG_RF_LINK);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* One -- and only one -- reset of the shared fx3_gpif FSM, performed while
     * neither RF nor EEM is streaming. The per-RF-session GPIO_SYS_RST pulse
     * has been removed precisely because it also resets the EEM FIFOs. */
    CyU3PGpioSetValue(GPIO_RX_EN, CyFalse);
    CyU3PGpioSetValue(GPIO_TX_EN, CyFalse);
    CyU3PGpioSetValue(GPIO_SYS_RST, CyTrue);
    CyU3PGpioSetValue(GPIO_SYS_RST, CyFalse);

    /* EEM endpoints */
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

    /* EEM DMA channels */
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

    /* host -> FPGA: USB socket 3 (EP 0x03 OUT) -> PIB socket 2 (GPIF TX2) */
    dmaCfg.prodSckId = BLADE_RF_EEM_EP_PRODUCER_USB_SOCKET;
    dmaCfg.consSckId = CY_U3P_PIB_SOCKET_2;
    apiRetStatus = CyU3PDmaChannelCreate(&glChHandleEEMUtoP,
                                         CY_U3P_DMA_TYPE_AUTO, &dmaCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* FPGA -> host: PIB socket 1 (GPIF RX1) -> USB socket 3 (EP 0x83 IN) */
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

    glGpifLinkUp = CyTrue;
}

/* Tear down the EEM datapath and stop the shared GPIF. Disabling the GPIF
 * deinits the PIB, which stops the FX3 PCLK. */
void NuandGpifLinkStop(void)
{
    CyU3PEpConfig_t epCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    if (!glGpifLinkUp) {
        return;
    }

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

    apiRetStatus = NuandConfigureGpif(GPIF_CONFIG_DISABLED);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    glGpifLinkUp = CyFalse;
}

/* Bring the GPIF/EEM link up once its preconditions hold: boot init done, USB
 * configured, and the FPGA actually configured. Idempotent -- safe to call
 * from multiple lifecycle hooks. */
void NuandTryStartGpifLink(void)
{
    CyBool_t confdone = CyFalse;

    if (glGpifLinkUp || !glDeviceReady || glUsbConfiguration == 0) {
        return;
    }

    if (CyU3PGpioGetValue(GPIO_CONFDONE, &confdone) != CY_U3P_SUCCESS || !confdone) {
        return;
    }

    NuandGpifLinkStart();
}

/* This function starts the RF data transport mechanism. This is the second
 * interface of the first and only descriptor.
 *
 * The shared GPIF and the EEM datapath are brought up separately, by
 * NuandGpifLinkStart() under the device-ready lifecycle. This function only
 * adds the RF sample endpoints/channels on top -- it must NOT (re)load the
 * GPIF or pulse GPIO_SYS_RST, since both would disrupt an active EEM link. */
static void NuandRFLinkStart(void)
{
    uint16_t size;
    CyU3PEpConfig_t epCfg;
    CyU3PDmaChannelConfig_t dmaCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed();

    NuandAllowSuspend(CyFalse);

    /* The GPIF and EEM datapath are owned by the device-ready lifecycle; just
     * make sure they are up (no-op if already up). */
    NuandTryStartGpifLink();

    /* Re-assert the IO matrix unconditionally (the SPI-flash interface may
     * have corrupted it). Do NOT pulse GPIO_SYS_RST here: it resets the
     * fx3_gpif FSM and the EEM FIFOs that the GPIF link owns. */
    NuandGPIOReconfigure(CyTrue, CyTrue);
    CyU3PGpioSetValue(GPIO_RX_EN, CyFalse);
    CyU3PGpioSetValue(GPIO_TX_EN, CyFalse);

    size = NuandUsbMaxPktSize();

    CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
    epCfg.enable = CyTrue;
    epCfg.epType = CY_U3P_USB_EP_BULK;
    epCfg.burstLen = (usbSpeed == CY_U3P_SUPER_SPEED ? 15 : 1);
    epCfg.streams = 0;
    epCfg.pcktSize = size;

    /* Producer endpoint configuration */
    apiRetStatus = CyU3PSetEpConfig(BLADE_RF_SAMPLE_EP_PRODUCER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Consumer endpoint configuration */
    apiRetStatus = CyU3PSetEpConfig(BLADE_RF_SAMPLE_EP_CONSUMER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    CyU3PMemSet((uint8_t *)&dmaCfg, 0, sizeof(dmaCfg));
    dmaCfg.size  = size * 8;
    dmaCfg.count = 11;
    dmaCfg.prodSckId = BLADE_RF_SAMPLE_EP_PRODUCER_USB_SOCKET;
    dmaCfg.consSckId = CY_U3P_PIB_SOCKET_3;
    dmaCfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
    dmaCfg.notification = 0;
    dmaCfg.cb = 0;
    dmaCfg.prodHeader = 0;
    dmaCfg.prodFooter = 0;
    dmaCfg.consHeader = 0;
    dmaCfg.prodAvailCount = 0;

    loopback_when_created = loopback;

    if (loopback) {
        dmaCfg.prodSckId = BLADE_RF_SAMPLE_EP_PRODUCER_USB_SOCKET;
        dmaCfg.consSckId = BLADE_RF_SAMPLE_EP_CONSUMER_USB_SOCKET;
    }

    apiRetStatus = CyU3PDmaChannelCreate(&glChHandleUtoP, CY_U3P_DMA_TYPE_AUTO, &dmaCfg);

    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    if (!loopback) {
        dmaCfg.prodSckId = CY_U3P_PIB_SOCKET_0;
        dmaCfg.consSckId = BLADE_RF_SAMPLE_EP_CONSUMER_USB_SOCKET;
        apiRetStatus = CyU3PDmaChannelCreate(&glChHandlePtoU, CY_U3P_DMA_TYPE_AUTO, &dmaCfg);
        if (apiRetStatus != CY_U3P_SUCCESS) {
            LOG_ERROR(apiRetStatus);
            CyFxAppErrorHandler(apiRetStatus);
        }
    }

    /* Flush the Endpoint memory */
    CyU3PUsbFlushEp(BLADE_RF_SAMPLE_EP_PRODUCER);
    CyU3PUsbFlushEp(BLADE_RF_SAMPLE_EP_CONSUMER);

    /* Set DMA channel transfer size. */

    apiRetStatus = CyU3PDmaChannelSetXfer (&glChHandleUtoP, BLADE_DMA_TX_SIZE);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    if (!loopback) {
        apiRetStatus = CyU3PDmaChannelSetXfer (&glChHandlePtoU, BLADE_DMA_TX_SIZE);
        if (apiRetStatus != CY_U3P_SUCCESS) {
            LOG_ERROR(apiRetStatus);
            CyFxAppErrorHandler(apiRetStatus);
        }
    }

    UartBridgeStart();
    glAppMode = MODE_RF_CONFIG;
}

/* This function stops the RF sample transport. It tears down only the RF
 * sample endpoints/channels; the shared GPIF and the EEM datapath are left
 * running (they are torn down by NuandGpifLinkStop() on reset/disconnect or
 * before an FPGA reprogram). */
static void NuandRFLinkStop (void)
{
    CyU3PEpConfig_t epCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* No GPIO_SYS_RST pulse: it would reset the shared fx3_gpif FSM and the
     * EEM FIFOs. The RF sample FIFOs self-clear when rx_enable/tx_enable
     * deassert. */
    CyU3PGpioSetValue(GPIO_RX_EN, CyFalse);
    CyU3PGpioSetValue(GPIO_TX_EN, CyFalse);

    /* Flush endpoint memory buffers */
    CyU3PUsbFlushEp(BLADE_RF_SAMPLE_EP_PRODUCER);
    CyU3PUsbFlushEp(BLADE_RF_SAMPLE_EP_CONSUMER);

    /* Destroy the RF sample channels */
    CyU3PDmaChannelDestroy(&glChHandleUtoP);
    if (!loopback_when_created)
        CyU3PDmaChannelDestroy(&glChHandlePtoU);

    /* Disable endpoints. */
    CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
    epCfg.enable = CyFalse;

    /* Disable producer endpoint */
    apiRetStatus = CyU3PSetEpConfig(BLADE_RF_SAMPLE_EP_PRODUCER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Disable consumer endpoint */
    apiRetStatus = CyU3PSetEpConfig(BLADE_RF_SAMPLE_EP_CONSUMER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        LOG_ERROR(apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    UartBridgeStop();
    NuandAllowSuspend(CyTrue);
    glAppMode = MODE_NO_CONFIG;
}

static uint8_t RF_status_bits[] = {
    [BLADE_RF_SAMPLE_EP_PRODUCER] = 0,
    [BLADE_RF_SAMPLE_EP_CONSUMER] = 0,
    [BLADE_RF_EEM_EP_PRODUCER] = 0,
    [BLADE_RF_EEM_EP_CONSUMER] = 0,
    [BLADE_UART_EP_PRODUCER] = 0,
    [BLADE_UART_EP_CONSUMER] = 0,
};

CyU3PReturnStatus_t NuandRFLinkResetEndpoint(uint8_t endpoint)
{
    CyU3PReturnStatus_t status = CY_U3P_ERROR_BAD_ARGUMENT;

    switch(endpoint) {
        case BLADE_RF_SAMPLE_EP_PRODUCER:
            status = ClearDMAChannel(endpoint, &glChHandleUtoP,
                                     BLADE_DMA_TX_SIZE);
            break;

        case BLADE_RF_SAMPLE_EP_CONSUMER:
            if (!loopback_when_created) {
                status = ClearDMAChannel(endpoint, &glChHandlePtoU,
                                         BLADE_DMA_TX_SIZE);
            } else {
                status = CY_U3P_SUCCESS;
            }
            break;

        case BLADE_RF_EEM_EP_PRODUCER:
            status = ClearDMAChannel(endpoint, &glChHandleEEMUtoP,
                                     BLADE_DMA_TX_SIZE);
            break;

        case BLADE_RF_EEM_EP_CONSUMER:
            status = ClearDMAChannel(endpoint, &glChHandleEEMPtoU,
                                     BLADE_DMA_TX_SIZE);
            break;

        case BLADE_UART_EP_PRODUCER:
            status = ClearDMAChannel(endpoint, &glChHandlebladeRFUtoUART,
                                     BLADE_DMA_TX_SIZE);
            break;

        case BLADE_UART_EP_CONSUMER:
            status = ClearDMAChannel(endpoint, &glChHandlebladeRFUARTtoU,
                                     BLADE_DMA_TX_SIZE);
            break;
    }

    return status;
}

CyBool_t NuandRFLinkHaltEndpoint(CyBool_t set, uint16_t endpoint)
{
    CyBool_t isHandled = CyFalse;
    CyU3PReturnStatus_t status = CY_U3P_ERROR_BAD_ARGUMENT;

    switch(endpoint) {
    case BLADE_RF_SAMPLE_EP_PRODUCER:
    case BLADE_RF_SAMPLE_EP_CONSUMER:
    case BLADE_RF_EEM_EP_PRODUCER:
    case BLADE_RF_EEM_EP_CONSUMER:
    case BLADE_UART_EP_PRODUCER:
    case BLADE_UART_EP_CONSUMER:
        isHandled = !set;
        RF_status_bits[endpoint] = set;
        status = NuandRFLinkResetEndpoint(endpoint);
        break;
    }

    if (status == CY_U3P_SUCCESS) {
        CyU3PUsbStall (endpoint, CyFalse, CyTrue);
        if(!set) {
            CyU3PUsbAckSetup ();
        }
    }

    return isHandled && status == CY_U3P_SUCCESS;
}

CyBool_t NuandRFLinkHalted(uint16_t endpoint, uint8_t * data)
{
    CyBool_t isHandled = CyFalse;

    switch(endpoint) {
    case BLADE_RF_SAMPLE_EP_PRODUCER:
    case BLADE_RF_SAMPLE_EP_CONSUMER:
    case BLADE_RF_EEM_EP_PRODUCER:
    case BLADE_RF_EEM_EP_CONSUMER:
    case BLADE_UART_EP_PRODUCER:
    case BLADE_UART_EP_CONSUMER:
        *data = RF_status_bits[endpoint];
        isHandled = CyTrue;
        break;
    }

    return isHandled;
}

const struct NuandApplication NuandRFLink = {
    .start = NuandRFLinkStart,
    .stop = NuandRFLinkStop,
    .halt_endpoint = NuandRFLinkHaltEndpoint,
    .halted = NuandRFLinkHalted,
    .reset_endpoint = NuandRFLinkResetEndpoint,
};
