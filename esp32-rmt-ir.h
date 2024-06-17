/*
https://github.com/junkfix/esp32-rmt-ir
*/

#ifndef ir_rmt_esp32
#define ir_rmt_esp32

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"

extern uint8_t irRxPin;
extern uint8_t irTxPin;

enum irproto { UNK, NEC, SONY, SAM, RC5, PROTO_COUNT };


typedef struct {
	rmt_encoder_t base;
	rmt_encoder_t *copy_encoder;
	uint8_t bit_index;
	int state;
} rmt_ir_encoder_t;

typedef struct {
	irproto irtype;
	uint32_t ircode;
	uint8_t bits;
} sendir_t;

typedef struct {
	uint16_t header_high;
	uint16_t header_low;
	uint16_t one_high;
	uint16_t one_low;
	uint16_t zero_high;
	uint16_t zero_low;
	uint16_t footer_high;
	uint8_t footer_low;
	uint16_t frequency;
	const char* name;
} ir_protocol_t;

extern const ir_protocol_t proto[PROTO_COUNT];

extern void irReceived(irproto brand, uint32_t code, size_t len, rmt_symbol_word_t *item);
void sendIR(irproto brand, uint32_t code, uint8_t bits = 32, uint8_t burst = 1, uint8_t repeat = 1);

IRAM_ATTR bool irrx_done(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *udata);

void recvIR(void* param);
uint32_t nec_check(rmt_symbol_word_t *item, size_t &len);
uint32_t sam_check(rmt_symbol_word_t *item, size_t &len);
uint32_t sony_check(rmt_symbol_word_t *item, size_t &len);
uint32_t rc5_check(rmt_symbol_word_t *item, size_t &len);
bool rc5_bit(uint32_t d, uint32_t v);
bool checkbit(rmt_symbol_word_t &item, uint16_t high, uint16_t low);
void fill_item(rmt_symbol_word_t &item, uint16_t high, uint16_t low, bool bit);

static esp_err_t rmt_ir_encoder_reset(rmt_encoder_t *encoder);
static esp_err_t rmt_del_ir_encoder(rmt_encoder_t *encoder);
static size_t rmt_encode_ir(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state);

#endif
