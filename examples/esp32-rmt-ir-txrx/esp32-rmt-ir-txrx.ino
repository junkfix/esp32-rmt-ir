/*
https://github.com/junkfix/esp32-rmt-ir
*/
#include "ir_rmt_esp32.h"

void irReceived(irproto brand, uint32_t code, size_t len, rmt_symbol_word_t *item){
	if( code ){
		Serial.printf("IR %s, code: %#x, bits: %d\n",  proto[brand].name, code, len);
	}
	
	if(true){//debug
		Serial.printf("Rx%d: ", len);							
		for (uint8_t i=0; i < len ; i++ ) {
			int d0 = item[i].duration0; if(!item[i].level0){d0 *= -1;}
			int d1 = item[i].duration1; if(!item[i].level1){d1 *= -1;}
			Serial.printf("%d,%d ", d0, d1);
		}								
		Serial.println();
	}
  
}

void setup() {
	Serial.begin(115200);
	irRxPin = 34;
	irTxPin = 4;
	xTaskCreatePinnedToCore(recvIR, "recvIR", 2048, NULL, 10, NULL, 1);
}

void loop() {
	sendIR(NEC, 0xC1AAFC03, 32, 1, 1);
	delay(1000);
	sendIR(SONY, 0x3e108, 20, 1, 1);
	delay(1000);
}
