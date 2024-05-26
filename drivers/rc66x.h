/******************************************************************************\

File:         rc66x.h
Author:       André van Schoubroeck <andre@blaatschaap.be>
License:      MIT

This implements the RC662 family of RFID reader ICs

* MFRC630
* MFRC631
* CLRC663
* Other compatibles 
********************************************************************************
MIT License

Copyright (c) 2020 André van Schoubroeck <andre@blaatschaap.be>

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
********************************************************************************

Card type support per IC.
|---------------|---------|---------|---------|---------|---------|
|///////////////| CLRC663 | CLRC661 | MFRC631 | MFRC630 | SLRC610 |
| ISO 14443 A   | X       | X       | X       | X       |         |
| ISO 14443 B   | X       |         | X       |         |         |
| JIS X 6319-4  | X       |         |         |         |         |
| ISO 15693     | X       | X       |         |         | X       |
| ISO 18000-3M3 | X       | X       |         |         | X       |
| ISO 18092     | X       |         |         |         |         |
|---------------|---------|---------|---------|---------|---------|

Please note according to the data sheets the Version register will read
the same, so I am not sure if we can tell them apart.
*******************************************************************************/




// SPI / UART
// READ = 0x01, WRITE = 0x00
// (addr << 1) | READ
// (addr << 1) | WRITE

#define RC66X_DIR_RECV        (0x01)
#define RC66X_DIR_SEND        (0x00)
#define RC66X_SPI_REG_SHIFT   (1)

// I²C
// register address as is,
// When reading, the register address is auto increased,
// except for the fifo register

#include "rc52x.h"
typedef  rc52x_t rc66x_t;
typedef rc52x_result_t rc66x_result_t;


#define RC66X_REG_Command         	(0x00) //  Starts and stops command execution
#define RC66X_REG_HostCtrl        	(0x01) //  Host control register
#define RC66X_REG_FIFOControl     	(0x02) //  Control register of the FIFO
#define RC66X_REG_WaterLevel      	(0x03) //  Level of the FIFO underflow and overflow warning
#define RC66X_REG_FIFOLength      	(0x04) //  Length of the FIFO
#define RC66X_REG_FIFOData        	(0x05) //  Data In/Out exchange register of FIFO buffer
#define RC66X_REG_IRQ0            	(0x06) //  Interrupt register 0
#define RC66X_REG_IRQ1            	(0x07) //  Interrupt register 1
#define RC66X_REG_IRQ0En          	(0x08) //  Interrupt enable register 0
#define RC66X_REG_IRQ1En          	(0x09) //  Interrupt enable register 1
#define RC66X_REG_Error           	(0x0A) //  Error bits showing the error status of the last command execution
#define RC66X_REG_Status          	(0x0B) //  Contains status of the communication
#define RC66X_REG_RxBitCtrl       	(0x0C) //  Control register for anticollision adjustments for bit oriented protocols
#define RC66X_REG_RxColl          	(0x0D) //  Collision position register
#define RC66X_REG_TControl        	(0x0E) //  Control of Timer 0..3
#define RC66X_REG_T0Control 	  	(0x0F) //  Control of Timer0
#define RC66X_REG_T0ReloadHi 		(0x10) //  High register of the reload value of Timer0
#define RC66X_REG_T0ReloadLo 		(0x11) //  Low register of the reload value of Timer0
#define RC66X_REG_T0CounterValHi 	(0x12) //  Counter value high register of Timer0
#define RC66X_REG_T0CounterValLo 	(0x13) //  Counter value low register of Timer0
#define RC66X_REG_T1Control 		(0x14) //  Control of Timer1
#define RC66X_REG_T1ReloadHi 		(0x15) //  High register of the reload value of Timer1
#define RC66X_REG_T1ReloadLo 		(0x16) //  Low register of the reload value of Timer1
#define RC66X_REG_T1CounterValHi 	(0x17) //  Counter value high register of Timer1
#define RC66X_REG_T1CounterValLo 	(0x18) //  Counter value low register of Timer1
#define RC66X_REG_T2Control 		(0x19) //  Control of Timer2
#define RC66X_REG_T2ReloadHi 		(0x1A) //  High byte of the reload value of Timer2
#define RC66X_REG_T2ReloadLo 		(0x1B) //  Low byte of the reload value of Timer2
#define RC66X_REG_T2CounterValHi 	(0x1C) //  Counter value high byte of Timer2
#define RC66X_REG_T2CounterValLo 	(0x1D) //  Counter value low byte of Timer2
#define RC66X_REG_T3Control 		(0x1E) //  Control of Timer3
#define RC66X_REG_T3ReloadHi 		(0x1F) //  High byte of the reload value of Timer3
#define RC66X_REG_T3ReloadLo		(0x20) //  Low byte of the reload value of Timer3
#define RC66X_REG_T3CounterValHi	(0x21) //  Counter value high byte of Timer3
#define RC66X_REG_T3CounterValLo	(0x22) //  Counter value low byte of Timer3
#define RC66X_REG_T4Control			(0x23) //  Control of Timer4
#define RC66X_REG_T4ReloadHi		(0x24) //  High byte of the reload value of Timer4
#define RC66X_REG_T4ReloadLo		(0x25) //  Low byte of the reload value of Timer4
#define RC66X_REG_T4CounterValHi	(0x26) //  Counter value high byte of Timer4
#define RC66X_REG_T4CounterValLo	(0x27) //  Counter value low byte of Timer4
#define RC66X_REG_DrvMode			(0x28) //  Driver mode register
#define RC66X_REG_TxAmp				(0x29) //  Transmitter amplifier register
#define RC66X_REG_DrvCon			(0x2A) //  Driver configuration register
#define RC66X_REG_Txl				(0x2B) //  Transmitter register
#define RC66X_REG_TxCrcPreset		(0x2C) //  Transmitter CRC control register, preset value
#define RC66X_REG_RxCrcPreset		(0x2D) //  Receiver CRC control register, preset value
#define RC66X_REG_TxDataNum			(0x2E) //  Transmitter data number register
#define RC66X_REG_TxModWidth		(0x2F) //  Transmitter modulation width register
#define RC66X_REG_TxSym10BurstLen 	(0x30) //  Transmitter symbol 1 + symbol 0 burst length register
#define RC66X_REG_TXWaitCtrl 		(0x31) //  Transmitter wait control
#define RC66X_REG_TxWaitLo 			(0x32) //  Transmitter wait low
#define RC66X_REG_FrameCon 			(0x33) //  Transmitter frame control
#define RC66X_REG_RxSofD 			(0x34) //  Receiver start of frame detection
#define RC66X_REG_RxCtrl 			(0x35) //  Receiver control register
#define RC66X_REG_RxWait 			(0x36) //  Receiver wait register
#define RC66X_REG_RxThreshold 		(0x37) //  Receiver threshold register
#define RC66X_REG_Rcv 				(0x38) //  Receiver register
#define RC66X_REG_RxAna 			(0x39) //  Receiver analog register
#define RC66X_REG_LPCD_Options 		(0x3A) //  For CLRC66303: Options for LPCD configuration (No function implemented for CLRC66301 and CLRC66302)
#define RC66X_REG_SerialSpeed 		(0x3B) //  Serial speed register
#define RC66X_REG_LFO_Trimm 		(0x3C) //  Low-power oscillator trimming register
#define RC66X_REG_PLL_Ctrl 			(0x3D) //  IntegerN PLL control register, for microcontroller clock output adjustment
#define RC66X_REG_PLL_DivOut 		(0x3E) //  IntegerN PLL control register, for microcontroller clock output adjustment
#define RC66X_REG_LPCD_QMin 		(0x3F) //  Low-power card detection Q channel minimum threshold
#define RC66X_REG_LPCD_QMax			(0X40) //  Low-power card detection Q channel maximum threshold
#define RC66X_REG_LPCD_IMin			(0X41) //  Low-power card detection I channel minimum threshold
#define RC66X_REG_LPCD_I_Result		(0X42) //  Low-power card detection I channel result register
#define RC66X_REG_LPCD_Q_Result		(0X43) //  Low-power card detection Q channel result register
#define RC66X_REG_PadEn				(0X44) //  PIN enable register
#define RC66X_REG_PadOut			(0X45) //  PIN out register
#define RC66X_REG_PadIn				(0X46) //  PIN in register
#define RC66X_REG_SigOut			(0X47) //  Enables and controls the SIGOUT Pin
#define RC66X_REG_TxBitMod			(0X48) //  Transmitter bit mode register
#define RC66X_REG_RFU_49 			(0X49)
#define RC66X_REG_TxDataCon			(0X4A) //  Transmitter data configuration register
#define RC66X_REG_TxDataMod			(0X4B) //  Transmitter data modulation register
#define RC66X_REG_TxSymFreq			(0X4C) //  Transmitter symbol frequency
#define RC66X_REG_TxSym0H			(0X4D) //  Transmitter symbol 0 high register
#define RC66X_REG_TxSym0L			(0X4E) //  Transmitter symbol 0 low register
#define RC66X_REG_TxSym1H			(0X4F) //  Transmitter symbol 1 high register
#define RC66X_REG_TxSym1L 			(0X50) //  Transmitter symbol 1 low register
#define RC66X_REG_TxSym2 			(0X51) //  Transmitter symbol 2 register
#define RC66X_REG_TxSym3 			(0X52) //  Transmitter symbol 3 register
#define RC66X_REG_TxSym10Len 		(0X53) //  Transmitter symbol 1 + symbol 0 length register
#define RC66X_REG_TxSym32Len 		(0X54) //  Transmitter symbol 3 + symbol 2 length register
#define RC66X_REG_TxSym10BurstCtrl 	(0X55) //  Transmitter symbol 1 + symbol 0 burst control register
#define RC66X_REG_TxSym10Mod 		(0X56) //  Transmitter symbol 1 + symbol 0 modulation register
#define RC66X_REG_TxSym32Mod 		(0X57) //  Transmitter symbol 3 + symbol 2 modulation register
#define RC66X_REG_RxBitMod 			(0X58) //  Receiver bit modulation register
#define RC66X_REG_RxEofSym 			(0X59) //  Receiver end of frame symbol register
#define RC66X_REG_RxSyncValH 		(0X5A) //  Receiver synchronisation value high register
#define RC66X_REG_RxSyncValL 		(0X5B) //  Receiver synchronisation value low register
#define RC66X_REG_RxSyncMod 		(0X5C) //  Receiver synchronisation mode register
#define RC66X_REG_RxMod 			(0X5D) //  Receiver modulation register
#define RC66X_REG_RxCorr 			(0X5E) //  Receiver correlation register
#define RC66X_REG_FabCal 			(0X5F) //  Calibration register of the receiver, calibration performed at production
#define RC66X_REG_Version 			(0x7F) //  Version and subversion register



#define RC66X_CMD_Idle				(0x00) //no action, cancels current command execution
#define RC66X_CMD_LPCD				(0x01)
#define RC66X_CMD_LoadKey			(0x02)
#define RC66X_CMD_MFAuthent			(0x03)
#define RC66X_CMD_AckReq			(0x04)
#define RC66X_CMD_Receive			(0x05)
#define RC66X_CMD_Transmit			(0x06)
#define RC66X_CMD_Transceive		(0x07)
#define RC66X_CMD_WriteE2			(0x08)
#define RC66X_CMD_WriteE2Page		(0x09)
#define RC66X_CMD_ReadE2			(0x0A)
#define RC66X_CMD_LoadReg			(0x0C)
#define RC66X_CMD_LoadProtocol		(0x0D)
#define RC66X_CMD_LoadKeyE2			(0x0E)
#define RC66X_CMD_StoreKeyE2		(0x0F)
#define RC66X_CMD_ReadRNR			(0x1C)
#define RC66X_CMD_SoftReset			(0x1F)

#define RC66X_TIMEOUT_ms			(40)

//------------
int rc66x_get_chip_version(rc66x_t *rc66x, uint8_t *chip_id);

void rc66x_antenna_on(rc66x_t *rc66x);
void rc66x_antenna_off(rc66x_t *rc66x);

rc66x_result_t rc66x_transceive(rc66x_t *rc66x,uint8_t *sendData,
		uint8_t sendLen,
		uint8_t *backData,
		uint8_t *backLen,
		uint8_t *validBits,
		uint8_t rxAlign,
		uint8_t * collpos,
		bool sendCRC ,
		bool recvCRC
		);

void rc66x_init(rc66x_t *rc66x);


void rc66x_test(rc66x_t *rc66x) ;
