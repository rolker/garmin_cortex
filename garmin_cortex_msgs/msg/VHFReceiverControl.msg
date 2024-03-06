int8  RECEIVER_STATE_WATCH_ON = 1
int8  RECEIVER_STATE_OMNI_WATCH = 2
int8  RECEIVER_STATE_CALLING_CAPTURE = 3
int8  RECEIVER_STATE_PRIORITY_CAPTURE = 4
int8  RECEIVER_STATE_SCAN_ALL = 5

int8  RECEIVER_WATCH_MODE_NONE = 1
int8  RECEIVER_WATCH_MODE_DUAL_WATCH = 2

int8  DUAL_WATCH_OFF_BECAUSE_USER = 1
int8  DUAL_WATCH_OFF_BECAUSE_ATIS = 2
int8  DUAL_WATCH_OFF_BECAUSE_SQUELCH_OFF = 3
int8  DUAL_WATCH_OFF_BECAUSE_DSC = 4

int8  receiver_state

int32 calling_channel_id
bool  calling_channel_high_power

int32 back_channel_id
bool  back_channel_high_power

int32 priority_channel_id

bool  squelch_on

bool  dual_watch_disabled
int8  dual_watch_off_reason

int32 weather_channel_id

