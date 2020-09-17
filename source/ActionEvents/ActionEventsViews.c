/*
 * ActionEventsViews.c
 *
 *  Created on: Dec 27, 2019
 *      Author: abusous2000
 */
#include "ch.h"
#include "hal.h"
#include "Strust4EmbeddedConf.h"
#include "ActionEvents.h"
#include "Strust4Embedded.h"
#include "ssd1306.h"

void mpu6050FormatAngles(char *buff, uint8_t size);
int8_t getCurrentMute(void);
int8_t getCurrentVolume(void);
void updateScreen(void){
#if S4E_USE_SSD1306_LCD != 0
   char  buff[20]={0};
   chsnprintf(buff, sizeof(buff),"Vol:%d",getCurrentVolume());
   uint8_t row = 20;
   LCD_Display_String(buff,row, false);
   chsnprintf(buff, sizeof(buff),"Mute:%d",getCurrentMute());
   row +=14;
   LCD_Display_String(buff,row, false);
   mpu6050FormatAngles(buff, sizeof(buff));
   row +=14;
   LCD_Display_String(buff,row, false);
   LCD_Display_Update();
#endif
   return;
}


int32_t volumeView(ActionEvent_Typedef 	*pActionEvent){(void)pActionEvent;
   dbgprintf("Vol.: %d\r\n", getCurrentVolume());
   updateScreen();

   return MSG_OK;
}
