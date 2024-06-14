/*

 File: 		pdc.h
 Author:	André van Schoubroeck
 License:	MIT


 MIT License

 Copyright (c) 2017 - 2023 André van Schoubroeck <andre@blaatschaap.be>

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

 */

#ifndef BSRFID_DRIVERS_PDC_H_
#define BSRFID_DRIVERS_PDC_H_

#include "bshal_transport.h"

typedef void(*delay_ms_f)(uint32_t);
typedef int(*get_time_ms_f)(void);



typedef enum {
	STATUS_OK				= 0,	// Success
	STATUS_ERROR			= -1,	// Error in communication
	STATUS_HARD_ERROR		= -2,	// Hardware error (eg. I2C NACK)
	STATUS_COLLISION		= -3,	// Collission detected
	STATUS_TIMEOUT			= -4,	// Timeout in communication.
	STATUS_NO_ROOM			= -5,	// A buffer is not big enough.
	STATUS_INTERNAL_ERROR	= -6,	// Internal error in the code. Should not happen ;-)
	STATUS_INVALID			= -7,	// Invalid argument.
	STATUS_CRC_WRONG		= -8,	// The CRC_A does not match
	STATUS_MIFARE_NACK		= -9,		// A MIFARE PICC responded with NAK.
	STATUS_AUTH_ERROR		= -10,
	STATUS_EEPROM_ERROR		= -11,
} pdc_result_t;
typedef pdc_result_t rc52x_result_t;





typedef int (*TransceiveData_f)(void *pdc, void *sendData, size_t sendLen,
		void *backData, size_t *backLen, uint8_t *validBits,
		uint8_t rxAlign, uint8_t *collisionPos, bool sendCRC, bool recvCRC);

typedef int (*SetBitFraming_f)(void *pdc, int rxAlign, int txLastBits);

typedef struct {
	bshal_transport_type_t transport_type;
	bshal_transport_instance_t transport_instance;
	delay_ms_f delay_ms;
	get_time_ms_f get_time_ms;
	TransceiveData_f TransceiveData;
} bs_pdc_t;




#endif /* BSRFID_DRIVERS_PDC_H_ */
