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
#ifndef _RF_H_
#define _RF_H_

#include "bladeRF.h"

extern const struct NuandApplication NuandRFLink;

/* Enable FW sample loopback */
void NuandRFLinkLoopBack(int);

/* Check if FW sample loopback is enabled */
int NuandRFLinkGetLoopBack();

/* Shared GPIF/EEM link: the FX3 PCLK and the GPIF II state machine that the RF
 * sample and EEM datapaths both ride on, plus the EEM endpoints/DMA. Owned by
 * the device-ready lifecycle, independent of the RF sample interface.
 * NuandConfigureGpif(GPIF_CONFIG_RF_LINK) must only ever be driven through
 * these; reloading it elsewhere resets the shared state machine. */
void NuandGpifLinkStart(void);
void NuandGpifLinkStop(void);

/* Bring the GPIF/EEM link up once its preconditions hold (boot init done, USB
 * configured, FPGA configured). Idempotent; call from lifecycle hooks. */
void NuandTryStartGpifLink(void);

#endif /* _RF_H_ */
