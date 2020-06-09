#include "ch.h"
#include "hal.h"
#include "Strust4EmbeddedConf.h"
#include "ActionEvents.h"
#include "Strust4Embedded.h"
#include "MPU6050Thread.h"

static int8_t 					count = 0;
static int8_t 					mute  = 0;
static int8_t 					volume 	= 80;
static int8_t       			pause   = 0;
extern bool 					sendMsgsToMQTT;
extern thread_reference_t 	    mpu6050RawThread;
extern thread_reference_t 	    dmp6050RawThread;
extern enum MPU605_ALGORITHM    currentAlgorithm;


int8_t isPaused(void){
	return pause;
}
int8_t getCurrentVolume(void){
	return volume;
}
int8_t getCurrentMute(void){
	return mute;
}
static int32_t setVolume(ActionEvent_Typedef   *pActionEvent){(void)pActionEvent;
  volume = pActionEvent->dataType==CHAR_DTYPE? atoi(pActionEvent->u.pData): (int8_t)pActionEvent->u.data;

  return MSG_OK;
}

static int32_t toggleMute(ActionEvent_Typedef 	*pActionEvent){(void)pActionEvent;
  mute= !mute;
  return MSG_OK;
}


static int32_t volumeDown(ActionEvent_Typedef 	*pActionEvent){(void)pActionEvent;
   volume -= 5;
   volume = volume<0?0:volume;
   return MSG_OK;
}
static int32_t volumeUp(ActionEvent_Typedef 	*pActionEvent){(void)pActionEvent;
   volume += 5;
   volume = volume>100?100:volume;

    return MSG_OK;
}

static int32_t togglePause(ActionEvent_Typedef 	*pActionEvent){(void)pActionEvent;
   sendMsgsToMQTT = !sendMsgsToMQTT;
   enableSleepModeMPU6050(!sendMsgsToMQTT);
   if ( sendMsgsToMQTT && currentAlgorithm == MPU605_ALGORITHM_RAW){
		chSysLock();
		chThdResumeI((thread_reference_t*)&mpu6050RawThread,1);
		chSysUnlock();
   }

   dbgprintf("Toggling pause:%d %d\r\n",sendMsgsToMQTT,count++);

   return MSG_OK;
}
static int32_t setAlgorithm(ActionEvent_Typedef 	*pActionEvent){
	thread_reference_t   *resumeThisThd = NULL;

	if ( strcmp(pActionEvent->u.pData,RAW_ALGPRITHM) == 0){
		if ( currentAlgorithm != MPU605_ALGORITHM_RAW){
			resumeThisThd = &mpu6050RawThread;
			currentAlgorithm = MPU605_ALGORITHM_RAW;
		}
	}
	else
	if ( strcmp(pActionEvent->u.pData,DMP_ALGPRITHM) == 0){
		if ( currentAlgorithm != MPU605_ALGORITHM_DMP){
			resumeThisThd = &dmp6050RawThread;
			currentAlgorithm = MPU605_ALGORITHM_DMP;
		}

	}
	sendMsgsToMQTT = true;
    if ( resumeThisThd != NULL ){
		chSysLock();
		chThdResumeI((thread_reference_t*)resumeThisThd,1);
		chSysUnlock();
    }
 	dbgprintf("Switching Algorithm to: %s\r\n",pActionEvent->u.pData);
	return MSG_OK;
}
static ActionEvent_Typedef actionEventToggleMute 	 	= {.name=TOGGLE_MUTE_AE_NAME,  			.eventSource="Center",      	.action=toggleMute,			.view=NULL,		.dataType = INT_DTYPE};
static ActionEvent_Typedef actionEventNextTrack  	 	= {.name=NEXT_TRACK_AE_NAME,			.eventSource="Up",          	.action=volumeUp,           .view=NULL,		.dataType = INT_DTYPE};
static ActionEvent_Typedef actionEventTogglePausePlay	= {.name=TOGGLE_PAUSE_AE_NAME,			.eventSource="Down",        	.action=togglePause};
static ActionEvent_Typedef actionEventVolumeDown   	 	= {.name=VOLUME_DOWN_AE_NAME,			.eventSource="Left",        	.action=volumeDown,         .view=NULL,			.dataType = INT_DTYPE};
static ActionEvent_Typedef actionEventVolumeUp       	= {.name=VOLUME_UP_AE_NAME,				.eventSource="Right",       	.action=volumeUp,			.view=NULL,			.dataType = INT_DTYPE};
static ActionEvent_Typedef actionEventSetVolume      	= {.name=SET_VOLUME_AE_NAME,			.eventSource="setVolume",   	.action=setVolume, 			.view=volumeView,			.dataType = INT_DTYPE};
static ActionEvent_Typedef actionEventTogglePause		= {.name=TOGGLE_PAUSE_AE_NAME,			.eventSource="Button",        	.action=togglePause};
static ActionEvent_Typedef actionEventSetAlgorithm		= {.name=SET_ALGORITHM_AE_NAME,			.eventSource="MQTT",        	.action=setAlgorithm, .dataType=CHAR_DTYPE};
ActionEvent_Typedef *gActionEvents[MAX_ACTION_EVENTS] ={&actionEventToggleMute,
														&actionEventNextTrack,
														&actionEventTogglePausePlay,
														&actionEventVolumeDown,
														&actionEventVolumeUp,
														&actionEventSetVolume,
														&actionEventTogglePause,
                                                        &actionEventSetAlgorithm};

