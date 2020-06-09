/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "Strust4EmbeddedConf.h"
#include "Strust4Embedded.h"
#include "ButtonLEDs.h"
#include "MQTTClient.h"
#include "MPU6050Thread.h"
#include "ssd1306.h"
#include "PotReader.h"
#include "P9813_RGB_Driver.h"

BaseSequentialStream *GlobalDebugChannel = (BaseSequentialStream *)&PORTAB_SD;
/* VCP Serial configuration. */
static const SerialConfig myserialcfg = {
  115200,
  0,
  USART_CR2_STOP1_BITS,
  0
};

void initMain(void);
static void initDrivers(void);
#ifndef __cplusplus
void initMain(void){
 halInit();
 chSysInit();
}
#endif
int main(void) {
    initMain();

	sdStart(&PORTAB_SD, &myserialcfg);
	initDrivers();
	initActonEventThd();
	initButtonsLEDs();
#if S4E_USE_BLINKER_THD == TRUE
  initBlinkerThd(pRedLedPAL);
#endif
#if S4E_USE_ETHERNET == 1
	initMQTTClient();
#endif
	initMPU6050Thread();
	while (true){
	    chThdSleepMilliseconds(1500);
	}

	return 0;
}
static void initDrivers(void){
#if S4E_USE_RGB == 1
  initP9813RGBDriver(TOTAL_NUM_OF_LEDS);
#endif
#if S4E_USE_POT == TRUE
  initPotReader();
#endif
#if S4E_USE_SSD1306_LCD == TRUE
  ssd130InitAndConfig("MPU6050 w/ S4E");
#endif
}
void publishStatusToBroker(void);
void periodicSysTrigger(uint32_t i){(void)i;
#if S4E_USE_POT == TRUE
   checkOnPotVolumeChange();
#endif
//#if S4E_USE_MQTT == 1
//   if ( i > 0 && i % 8 == 0 )
//	   publishStatusToBroker();
//#endif
}
