Header header

int8 MOTOR_ID_FRONT_RIGHT = 0
int8 MOTOR_ID_FRONT_LEFT = 1
int8 MOTOR_ID_REAR_RIGHT = 2
int8 MOTOR_ID_REAR_LEFT = 3

int8 LIGHT_ID_FRONT = 0
int8 LIGHT_ID_REAR = 1

# motion state
float64 linear_velocity
float64 angular_velocity

# base state
uint8 vehicle_state
uint8 control_mode
uint16 error_code
float64 battery_voltage

# motor state
ScoutActuatorState[4] actuator_states

# light state
bool light_control_enabled
ScoutLightState front_light_state
ScoutLightState rear_light_state