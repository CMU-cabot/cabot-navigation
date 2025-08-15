int32 SENSOR_LOCATION_DEFAULT = 0
int32 SENSOR_LOCATION_HEADPHONE_LEFT = 1
int32 SENSOR_LOCATION_HEADPHONE_RIGHT = 2

std_msgs/Header header
geometry_msgs/Quaternion attitude
geometry_msgs/Vector3 gravity
float64 heading
CMCalibratedMagneticField magnetic_field
geometry_msgs/Vector3 rotation_rate
geometry_msgs/Vector3 user_acceleration

int32 sensor_location
