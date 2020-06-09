#ifndef ACTIONEVENTS_H_
#define ACTIONEVENTS_H_
#include "ActionEventsThd.h"

#define TOGGLE_MUTE_AE_NAME                 "toggleMute"
#define NEXT_TRACK_AE_NAME                  "nextTrack"
#define TOGGLE_PAUSE_AE_NAME                "togglePause"
#define VOLUME_UP_AE_NAME                   "volumeUp"
#define VOLUME_DOWN_AE_NAME                 "volumeDown"
#define SET_VOLUME_AE_NAME                  "setVolume"
#define TOGGLE_PAUSE_AE_NAME                "togglePause"
#define SET_ALGORITHM_AE_NAME			    "setAlgorithm"

#ifdef __cplusplus
 extern "C" {
#endif
int32_t volumeView(ActionEvent_Typedef 	*pActionEvent);
#ifdef __cplusplus
}
#endif
#endif /* ACTIONEVENTS_H_ */
