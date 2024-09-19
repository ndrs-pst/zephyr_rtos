/*
 * Copyright (c) 2021 Argentum Systems Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _ATMEL_SAM_DAC_FIXUP_H_
#define _ATMEL_SAM_DAC_FIXUP_H_

#if defined(DAC_SYNCBUSY_MASK)
#define DAC_SYNC(DAC)   ((DAC)->SYNCBUSY.reg)
#define DAC_SYNC_MASK   (DAC_SYNCBUSY_MASK)
#elif defined(DAC_STATUS_SYNCBUSY)
#define DAC_SYNC(DAC)   ((DAC)->STATUS.reg)
#define DAC_SYNC_MASK   (DAC_STATUS_SYNCBUSY)
#else
#error DAC not supported...
#endif

#endif /* _ATMEL_SAM0_DAC_FIXUP_H_ */
