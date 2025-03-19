typedef struct {
	GLfloat x_orientation;
	GLfloat y_orientation;
	GLfloat z_orientation;
	GLfloat x_speed;
	GLfloat y_speed;
	GLfloat z_speed;
	float gyro_x_zero;
	float gyro_y_zero;
	float gyro_z_zero;
	float gyro_x_deviation;
	float gyro_y_deviation;
	float gyro_z_deviation;

} machine_attitude;
typedef struct {
	double x;
	double y;
	double z;
	double w;

} quaternion;
typedef struct{
	double yaw;
	double pitch;
	double roll;
} euler;
typedef struct {
  int gyro_deviation_multiplier;
  int accel_deviation_multiplier;
} user_args;
