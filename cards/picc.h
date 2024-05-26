/*
 * rfid.h
 *
 *  Created on: 21 dec. 2020
 *      Author: andre
 */

#ifndef BSRFID_CARDS_PICC_H_
#define BSRFID_CARDS_PICC_H_

#include "iso14443a.h"
#include "iso14443b.h"
#include "iso15693.h"

#include "pdc.h"

typedef enum {
	picc_protocol_undefined,    //
	picc_protocol_iso14443a,    // eg. MiFare, NTAG
	picc_protocol_iso14443b,	// eg. SRI512
	picc_protocol_iso15693,     // eg. ICode
	picc_protocol_jisx_6319_4,  // eg. FeliCa
} picc_protocol_t;

typedef enum {
	picc_type_unknown = 0x00,
	picc_type_mfc = 0x01,	// Mifare Classic
	picc_type_mfu = 0x02,	// Mifare UltraLight
	picc_type_ntag = 0x02,	// NTAG
} picc_type_t;

typedef enum {
	nfc_type_none = 0x00,//
	nfc_type_1 = 0x01, // Jewel
	nfc_type_2 = 0x02, // Mifare UltraLight, NTAG21x
	nfc_type_3 = 0x03, // Felica
	nfc_type_4 = 0x04, // ISO 14443-4
	nfc_type_5 = 0x05, // ISO 15693
	nfc_type_mfc = 0xFC,	// Unoffical Mifare Classic
} nfc_type_t;

typedef struct {
	// Number of bytes in the UID.
	// For ISO 14443-A 		4, 7 or 10.
	// For ISO 14443-B		4
	// For ISO 15693		8
	// For JIS X 6319-4 	8

	picc_protocol_t protocol;
	uint8_t uid_size;
	uint8_t uid[10];
	union {
		struct {
			iso14443a_sak_t sak;// The SAK (Select acknowledge) byte returned from the PICC after successful selection.
			iso14443a_atqa_t atqa;
			iso14443a_anticol_state_t anticol_state;
		};
	};
	picc_type_t card_type;	//TODO
	nfc_type_t nfc_type;
	uint8_t iso14443_4_pcb;
	//union {
		struct {
			uint8_t fixed_header;
			uint8_t vendor_id;
			uint8_t product_type;
			uint8_t product_subtype;
			uint8_t product_version_major;
			uint8_t product_version_minor;
			uint8_t storage_size;
			uint8_t protocol_type;
		} version_response;
		uint8_t rats[16];
	//};
	struct {
		uint8_t page_count;
		uint8_t page_size;
		uint8_t page_offset;
		// NB... for MFC more is needed as we need to skip pages
	} memory_stucture;
	union{
		struct {
			uint8_t key_a_or_b;
			uint8_t block_address;
			uint8_t key[6];
		} mfc_crypto1;
	};
} picc_t;

rc52x_result_t PICC_REQA_or_WUPA(bs_pdc_t *pdc, uint8_t command, ///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
		uint8_t *bufferATQA, ///< The buffer to store the ATQA (Answer to request) in
		size_t *bufferSize///< Buffer uid_size, at least two bytes. Also number of bytes returned if STATUS_OK.
		);

int MIFARE_READ(bs_pdc_t *pdc, picc_t *picc, int page, uint8_t *data);
int MFU_Write(bs_pdc_t *pdc, picc_t *picc, int page, uint8_t *data) ;

rc52x_result_t PICC_RequestA(bs_pdc_t *pdc, picc_t *picc);
rc52x_result_t PICC_Select(bs_pdc_t *pdc, picc_t *picc, uint8_t validBits);
#endif /* BSRFID_CARDS_PICC_H_ */
