std_msgs/Header header
string[] name
float64[] resolver_angle
float64[] effort
float64[] position_errors
float64[] velocity_errors 
int32[] calibration_state
int32 calibration_step

int32 IDLE=0
int32 START_LIMIT_SEARCH=1
int32 START_APPROACH_CALIB_REGION=2
int32 APPROACH_CALIB_REGION=3
int32 START_GO_HOME=4
int32 GO_HOME=5
int32 END_CALIBRATION=6

int32 UNCALIBRATED=0
int32 CALIBRATING=1
int32 CALIBRATED=2
