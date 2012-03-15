# This command ID just tells the mux to echo status
int8 ECHO=0

# These command IDs correspond to OROCOS component control
# Note, however, that LOAD with both load and configure a controller, and
# UNLOAD with both cleanup and unload a controller.
int8 LOAD=1   # no-op
int8 START=2  # no-op
int8 STOP=3   # no-op
int8 UNLOAD=4 # no-op

string name
string package
uint8 dof

uint8 cmd

bool enabled
uint8[] dof_map
float64[] gains
