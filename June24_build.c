#include <glib.h>
#include <glib/gprintf.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <GL/glut.h>
#include <math.h>
#include <attitude.h>


#define MPU_6050 0x68
#define GYRO_CONSTANT 5.6700
#define ACCEL_CONSTANT 16384.0000

/* Define MPU6050 registers */

#define XGOFFS_TC        0x00 // Bit 7 PWR_MODE, bits 6:1 XG_OFFS_TC, bit 0 OTP_BNK_VLD                 
#define YGOFFS_TC        0x01                                                                          
#define ZGOFFS_TC        0x02
#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B
#define SELF_TEST_X      0x0D
#define SELF_TEST_Y      0x0E    
#define SELF_TEST_Z      0x0F
#define SELF_TEST_A      0x10
#define XG_OFFS_USRH     0x13  // User-defined trim values for gyroscope; supported in MPU-6050?
#define XG_OFFS_USRL     0x14
#define YG_OFFS_USRH     0x15
#define YG_OFFS_USRL     0x16
#define ZG_OFFS_USRH     0x17
#define ZG_OFFS_USRL     0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define FF_THR           0x1D  // Free-fall
#define FF_DUR           0x1E  // Free-fall
#define MOT_THR          0x1F  // Motion detection threshold bits [7:0]
#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42

#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
	
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL   0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B  // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU6050 0x75  // Should return 0x68

/* End Definitions */


/* Data structure for holding attitude information */

machine_attitude report;
struct timespec cycle_clock_last;
struct timespec cycle_clock;
double _rotation_matrix[16] = {0};

/* End decleration */


/* Various i2c communication dunctions */

int8_t read_reg8(int file_handle, int i2c_addr, char address){
/*	Takes an i2c bus(file), slave address, and register address and returns 8 byte read value from device */
	
  const gchar *buffer;
	if (ioctl(file_handle, I2C_SLAVE, i2c_addr) < 0) {
		printf("%s \n", "Byte read failed...failed to acquire bus access and/or talk to slave.");
		return 1;
	}
	char buf0[1] = {0};
	char reg_addr[10];
	reg_addr[0] = address;
	if (write(file_handle,reg_addr,1) != 1){
		/*error handling: i2c transaction failed*/
		printf("Failed to write to the i2c bus.\n");
		buffer = g_strerror(errno);
		printf("%s",buffer);
		printf("\n\n");
		return 1;
	}
	if (read(file_handle,buf0,1) != 1){
		/*error handling: i2c transaction failed*/
		printf("Failed to write to the i2c bus.\n");
		buffer = g_strerror(errno);
		printf("%s",buffer);
		printf("\n\n");
		return 1;
	}
	return buf0[0];
}

int8_t write_reg8(int file_handle, int i2c_addr, char reg_addr, uint8_t value){
/*	Takes an i2c bus(file), slave address, register address, and 8 bit int
	then writes to a single register on device 				*/
	
  const gchar *buffer;
	if (ioctl(file_handle, I2C_SLAVE, i2c_addr) < 0) {
		printf("%s \n", "Byte write failed...failed to acquire bus access and/or talk to slave.");
		return 1;
	}
	char message[10] = {0};
	message[0] = reg_addr;
	message[1] = value;
	if (write(file_handle, message, 2) != 2) {
        	/* ERROR HANDLING: i2c transaction failed */
        	printf("Byte write failed...failed to write l_value.\n");
        	buffer = g_strerror(errno);
		printf("%s",buffer);
        	printf("\n\n");
		return 1;
	}
	return 0;
}

int16_t write_reg16(int file_handle, int i2c_addr, int reg_addr, uint16_t value){
/*	Takes an i2c bus(file), slave address, register address, and 16 bit integer
	and writes DWORD (2 sequential 8 bit registers) value to I2C device.  Data
	format is HSV then LSV in the next sequential register */
  const gchar *buffer;
	if (ioctl(file_handle, I2C_SLAVE, i2c_addr) < 0) {
		printf("%s \n", "DWORD write failed...failed to acquire bus access and/or talk to slave.");
		return 1;
	}
	char message[10] = {0};
	message[0] = reg_addr;
	message[1] = ((((value >> 8)) & 0xff));
	if (write(file_handle, message, 2) != 2) {
        	/* ERROR HANDLING: i2c transaction failed */
        	printf("DWORD write failed...failed to write l_value.\n");
        	buffer = g_strerror(errno);
		printf("%s",buffer);
        	printf("\n\n");
		return 1;
	}
	message[0] = ((reg_addr + 1));
	message[1] = ((value & 0xff));
	if (write(file_handle, message, 2) != 2) {
        	 /*ERROR HANDLING: i2c transaction failed */
        	printf("DWORD write failed...failed to write h_value.\n");
        	buffer = g_strerror(errno);
		printf("%s",buffer);
        	printf("\n\n");
		return 1;
	}
	return 0;
}

int16_t read_reg16(int file_handle, int i2c_addr, char address){
/*	Takes an i2c bus(file), slave address, and register address
	then reads from two sequential 8 bit registersby incrementing
	and returns the corresponding 16 bit value			*/
	
  const gchar *buffer;
	if (ioctl(file_handle, I2C_SLAVE, i2c_addr) < 0) {
		printf("%s \n", "DWORD read failed...failed to acquire bus access and/or talk to slave.");
		return 1;
	}
  char buf0[2] = {0};
  char reg_addr[1];
  reg_addr[0] = address;
  int16_t retval;
  if (write(file_handle,reg_addr,1) != 1){
      /* ERROR HANDLING: i2c transaction failed */
      printf("Failed to write to the i2c bus.\n");
      buffer = g_strerror(errno);
      printf("%s",buffer);
      printf("\n\n");
  }
  if (read(file_handle,buf0,2) != 2){
      /* ERROR HANDLING: i2c transaction failed */
      printf("Failed to read from the i2c bus.\n");
      buffer = g_strerror(errno);
      printf("%s",buffer);
      printf("\n\n");
  }

	/* Bitwise operation combines l_value and h_value */
  retval = (buf0[0] << 8 ) | buf0[1];
  return retval;
}


/* End i2c function definitions */

/* Quaternion / Euler Utility Functions */

quaternion _to_quaternion ( euler angles ){
  quaternion hypersphere;
  hypersphere.x = (sin(angles.roll/2)*cos(angles.pitch/2)*cos(angles.yaw/2))-(cos(angles.roll/2)*sin(angles.pitch/2)*sin(angles.yaw/2));
  hypersphere.y = (cos(angles.roll/2)*sin(angles.pitch/2)*cos(angles.yaw/2))+(sin(angles.roll/2)*cos(angles.pitch/2)*sin(angles.yaw/2));
  hypersphere.z = (cos(angles.roll/2)*cos(angles.pitch/2)*sin(angles.yaw/2))-(sin(angles.roll/2)*sin(angles.pitch/2)*cos(angles.yaw/2));
  hypersphere.w = (cos(angles.roll/2)*cos(angles.pitch/2)*cos(angles.yaw/2))+(sin(angles.roll/2)*sin(angles.pitch/2)*sin(angles.yaw/2));
  return hypersphere;
}

/* euler _to_euler( quaternion hypersphere ){

  double t0 = +2.0 * (hypersphere.w * hypersphere.x + hypersphere.y * hypersphere.z);
  double t1 = +1.0 - 2.0 * (hypersphere.x * hypersphere.x + hypersphere.y * hypersphere.y);
  double t2 = t2 = +2.0 * (hypersphere.w * hypersphere.y - hypersphere.z * hypersphere.x);
  double t2 = +1.0;
  double t2 = -1.0
  double t3 = +2.0 * (hypersphere.w * hypersphere.z + hypersphere.x * hypersphere.y);
  double t4 = +1.0 - 2.0 * (hypersphere.y * hypersphere.y + hypersphere.z * hypersphere.z);

  euler angles;
  euler.yaw = atan2(t3, t4);
  euler.pitch = asin(t2);
  euler.roll = atan2(t0, t1);

  return angles;
}*/

/* End Quaternion / Euler Utility Functions */


/* Display Loop */

void displayMe(void)
{
    int len, i, z;
    const char *test = "10";
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor3f( 255, 255, 255);
    //glPushMatrix ();
    glBegin(GL_POLYGON);
        glVertex3f(0.05, -0.005, -0.2);
        glVertex3f(0.05, 0.005, -0.2);
        glVertex3f(0.02, 0.005, -0.2);
        glVertex3f(0.02, -0.005, -0.2);
	glEnd();
    glBegin(GL_POLYGON);
        glVertex3f(0.02, -0.005, -0.2);
        glVertex3f(0.02, 0.005, -0.2);
        glVertex3f(0.0, -0.010, -0.2);
        glVertex3f(0.0, -0.025, -0.2);
	glEnd();
    glBegin(GL_POLYGON);
        glVertex3f(-0.02, -0.005, -0.2);
        glVertex3f(-0.02, 0.005, -0.2);
        glVertex3f(0.0, -0.010, -0.2);
        glVertex3f(0.0, -0.025, -0.2);
	glEnd();
    glBegin(GL_POLYGON);
        glVertex3f(-0.05, -0.005, -0.2);
        glVertex3f(-0.05, 0.005, -0.2);
        glVertex3f(-0.02, 0.005, -0.2);
        glVertex3f(-0.02, -0.005, -0.2);
	glEnd();
    glBegin(GL_POLYGON);
        glVertex3f(-0.01, -0.48, -0.2);
        glVertex3f(0.01, -0.48, -0.2);
        glVertex3f(0.005, -0.46, -0.2);
        glVertex3f(-0.005, -0.46, -0.2);
	glEnd();
    glColor3f( 0, 255, 0);
    
    /*Unused in current implementation
      float interpolated_x = report.x_orientation - 0.5;
      if (interpolated_x < (0-1)) interpolated_x += 2;*/


    //glPushMatrix ();

    if (_rotation_matrix){
      //glMultMatrixd(_rotation_matrix);
      glRotated (report.z_orientation, 0, 1.0, 0);
      glRotated (report.y_orientation, 1.0, 0, 0);
      glRotated (report.x_orientation, 0, 0, 1.0);
    }
    else {
      glRotated (report.z_orientation, 0, 1.0, 0);
      glRotated (report.y_orientation, 1.0, 0, 0);
      glRotated (report.x_orientation, 0, 0, 1.0);
    }

    glGetDoublev (GL_PROJECTION_MATRIX, _rotation_matrix);

    
    //Compass Rotations

/*Generated Compass Matrix*/

glBegin(GL_POLYGON); 
glVertex3f(-1.005000, -0.4, 0.000000);
glVertex3f(-0.995000, -0.4, 0.000000);
glVertex3f(-0.995000, -0.44, 0.000000);
glVertex3f(-1.005000, -0.44, 0.000000);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-1.005000, -0.4, 0.000000);
glVertex3f(-0.995000, -0.4, 0.000000);
glVertex3f(-0.995000, -0.44, 0.000000);
glVertex3f(-1.005000, -0.44, 0.000000);
glEnd(); 
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.989808, -0.4, -0.173648);
glVertex3f(-0.979808, -0.4, -0.173648);
glVertex3f(-0.979808, -0.44, -0.173648);
glVertex3f(-0.989808, -0.44, -0.173648);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.989808, -0.4, -0.173648);
glVertex3f(-0.979808, -0.4, -0.173648);
glVertex3f(-0.979808, -0.44, -0.173648);
glVertex3f(-0.989808, -0.44, -0.173648);
glEnd(); 
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.944693, -0.4, -0.342020);
glVertex3f(-0.934693, -0.4, -0.342020);
glVertex3f(-0.934693, -0.44, -0.342020);
glVertex3f(-0.944693, -0.44, -0.342020);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.944693, -0.4, -0.342020);
glVertex3f(-0.934693, -0.4, -0.342020);
glVertex3f(-0.934693, -0.44, -0.342020);
glVertex3f(-0.944693, -0.44, -0.342020);
glEnd(); 
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.871025, -0.4, -0.500000);
glVertex3f(-0.861025, -0.4, -0.500000);
glVertex3f(-0.861025, -0.44, -0.500000);
glVertex3f(-0.871025, -0.44, -0.500000);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.871025, -0.4, -0.500000);
glVertex3f(-0.861025, -0.4, -0.500000);
glVertex3f(-0.861025, -0.44, -0.500000);
glVertex3f(-0.871025, -0.44, -0.500000);
glEnd(); 
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.771044, -0.4, -0.642788);
glVertex3f(-0.761044, -0.4, -0.642788);
glVertex3f(-0.761044, -0.44, -0.642788);
glVertex3f(-0.771044, -0.44, -0.642788);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.771044, -0.4, -0.642788);
glVertex3f(-0.761044, -0.4, -0.642788);
glVertex3f(-0.761044, -0.44, -0.642788);
glVertex3f(-0.771044, -0.44, -0.642788);
glEnd(); 
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.647788, -0.4, -0.766044);
glVertex3f(-0.637788, -0.4, -0.766044);
glVertex3f(-0.637788, -0.44, -0.766044);
glVertex3f(-0.647788, -0.44, -0.766044);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.647788, -0.4, -0.766044);
glVertex3f(-0.637788, -0.4, -0.766044);
glVertex3f(-0.637788, -0.44, -0.766044);
glVertex3f(-0.647788, -0.44, -0.766044);
glEnd(); 
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.505000, -0.4, -0.866025);
glVertex3f(-0.495000, -0.4, -0.866025);
glVertex3f(-0.495000, -0.44, -0.866025);
glVertex3f(-0.505000, -0.44, -0.866025);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.505000, -0.4, -0.866025);
glVertex3f(-0.495000, -0.4, -0.866025);
glVertex3f(-0.495000, -0.44, -0.866025);
glVertex3f(-0.505000, -0.44, -0.866025);
glEnd(); 
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.347020, -0.4, -0.939693);
glVertex3f(-0.337020, -0.4, -0.939693);
glVertex3f(-0.337020, -0.44, -0.939693);
glVertex3f(-0.347020, -0.44, -0.939693);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.347020, -0.4, -0.939693);
glVertex3f(-0.337020, -0.4, -0.939693);
glVertex3f(-0.337020, -0.44, -0.939693);
glVertex3f(-0.347020, -0.44, -0.939693);
glEnd(); 
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.178648, -0.4, -0.984808);
glVertex3f(-0.168648, -0.4, -0.984808);
glVertex3f(-0.168648, -0.44, -0.984808);
glVertex3f(-0.178648, -0.44, -0.984808);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.178648, -0.4, -0.984808);
glVertex3f(-0.168648, -0.4, -0.984808);
glVertex3f(-0.168648, -0.44, -0.984808);
glVertex3f(-0.178648, -0.44, -0.984808);
glEnd(); 
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.005000, -0.4, -1.000000);
glVertex3f(0.005000, -0.4, -1.000000);
glVertex3f(0.005000, -0.44, -1.000000);
glVertex3f(-0.005000, -0.44, -1.000000);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.005000, -0.4, -1.000000);
glVertex3f(0.005000, -0.4, -1.000000);
glVertex3f(0.005000, -0.44, -1.000000);
glVertex3f(-0.005000, -0.44, -1.000000);
glEnd(); 
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.168648, -0.4, -0.984808);
glVertex3f(0.178648, -0.4, -0.984808);
glVertex3f(0.178648, -0.44, -0.984808);
glVertex3f(0.168648, -0.44, -0.984808);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.168648, -0.4, -0.984808);
glVertex3f(0.178648, -0.4, -0.984808);
glVertex3f(0.178648, -0.44, -0.984808);
glVertex3f(0.168648, -0.44, -0.984808);
glEnd(); 
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.337020, -0.4, -0.939693);
glVertex3f(0.347020, -0.4, -0.939693);
glVertex3f(0.347020, -0.44, -0.939693);
glVertex3f(0.337020, -0.44, -0.939693);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.337020, -0.4, -0.939693);
glVertex3f(0.347020, -0.4, -0.939693);
glVertex3f(0.347020, -0.44, -0.939693);
glVertex3f(0.337020, -0.44, -0.939693);
glEnd(); 
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.495000, -0.4, -0.866025);
glVertex3f(0.505000, -0.4, -0.866025);
glVertex3f(0.505000, -0.44, -0.866025);
glVertex3f(0.495000, -0.44, -0.866025);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.495000, -0.4, -0.866025);
glVertex3f(0.505000, -0.4, -0.866025);
glVertex3f(0.505000, -0.44, -0.866025);
glVertex3f(0.495000, -0.44, -0.866025);
glEnd(); 
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.637788, -0.4, -0.766044);
glVertex3f(0.647788, -0.4, -0.766044);
glVertex3f(0.647788, -0.44, -0.766044);
glVertex3f(0.637788, -0.44, -0.766044);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.637788, -0.4, -0.766044);
glVertex3f(0.647788, -0.4, -0.766044);
glVertex3f(0.647788, -0.44, -0.766044);
glVertex3f(0.637788, -0.44, -0.766044);
glEnd(); 
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.761044, -0.4, -0.642788);
glVertex3f(0.771044, -0.4, -0.642788);
glVertex3f(0.771044, -0.44, -0.642788);
glVertex3f(0.761044, -0.44, -0.642788);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.761044, -0.4, -0.642788);
glVertex3f(0.771044, -0.4, -0.642788);
glVertex3f(0.771044, -0.44, -0.642788);
glVertex3f(0.761044, -0.44, -0.642788);
glEnd(); 
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.861025, -0.4, -0.500000);
glVertex3f(0.871025, -0.4, -0.500000);
glVertex3f(0.871025, -0.44, -0.500000);
glVertex3f(0.861025, -0.44, -0.500000);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.861025, -0.4, -0.500000);
glVertex3f(0.871025, -0.4, -0.500000);
glVertex3f(0.871025, -0.44, -0.500000);
glVertex3f(0.861025, -0.44, -0.500000);
glEnd(); 
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.934693, -0.4, -0.342020);
glVertex3f(0.944693, -0.4, -0.342020);
glVertex3f(0.944693, -0.44, -0.342020);
glVertex3f(0.934693, -0.44, -0.342020);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.934693, -0.4, -0.342020);
glVertex3f(0.944693, -0.4, -0.342020);
glVertex3f(0.944693, -0.44, -0.342020);
glVertex3f(0.934693, -0.44, -0.342020);
glEnd(); 
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.979808, -0.4, -0.173648);
glVertex3f(0.989808, -0.4, -0.173648);
glVertex3f(0.989808, -0.44, -0.173648);
glVertex3f(0.979808, -0.44, -0.173648);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.979808, -0.4, -0.173648);
glVertex3f(0.989808, -0.4, -0.173648);
glVertex3f(0.989808, -0.44, -0.173648);
glVertex3f(0.979808, -0.44, -0.173648);
glEnd(); 
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.995000, -0.4, 0.000000);
glVertex3f(1.005000, -0.4, 0.000000);
glVertex3f(1.005000, -0.44, 0.000000);
glVertex3f(0.995000, -0.44, 0.000000);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.995000, -0.4, 0.000000);
glVertex3f(1.005000, -0.4, 0.000000);
glVertex3f(1.005000, -0.44, 0.000000);
glVertex3f(0.995000, -0.44, 0.000000);
glEnd(); 
glEnd(); 
glRasterPos3f(-1.000000, -0.45, -0.000000);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '2');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '7');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(-1.000000, -0.45, 0.000000);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '2');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '7');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(-0.984808, -0.45, 0.173648);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '2');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '6');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(-0.984808, -0.45, -0.173648);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '2');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '8');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(-0.939693, -0.45, 0.342020);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '2');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '5');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(-0.939693, -0.45, -0.342020);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '2');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '9');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(-0.866025, -0.45, 0.500000);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '2');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '4');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(-0.866025, -0.45, -0.500000);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '3');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(-0.766044, -0.45, 0.642788);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '2');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '3');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(-0.766044, -0.45, -0.642788);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '3');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '1');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(-0.642788, -0.45, 0.766044);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '2');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '2');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(-0.642788, -0.45, -0.766044);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '3');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '2');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(-0.500000, -0.45, 0.866025);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '2');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '1');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(-0.500000, -0.45, -0.866025);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '3');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '3');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(-0.342020, -0.45, 0.939693);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '2');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(-0.342020, -0.45, -0.939693);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '3');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '4');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(-0.173648, -0.45, 0.984808);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '1');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '9');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(-0.173648, -0.45, -0.984808);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '3');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '5');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.000000, -0.45, 1.000000);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '1');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '8');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.000000, -0.45, -1.000000);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.173648, -0.45, 0.984808);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '1');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '7');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.173648, -0.45, -0.984808);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '1');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.342020, -0.45, 0.939693);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '1');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '6');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.342020, -0.45, -0.939693);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '2');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.500000, -0.45, 0.866025);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '1');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '5');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.500000, -0.45, -0.866025);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '3');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.642788, -0.45, 0.766044);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '1');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '4');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.642788, -0.45, -0.766044);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '4');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.766044, -0.45, 0.642788);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '1');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '3');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.766044, -0.45, -0.642788);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '5');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.866025, -0.45, 0.500000);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '1');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '2');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.866025, -0.45, -0.500000);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '6');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.939693, -0.45, 0.342020);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '1');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '1');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.939693, -0.45, -0.342020);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '7');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.984808, -0.45, 0.173648);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '1');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.984808, -0.45, -0.173648);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '8');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(1.000000, -0.45, -0.000000);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '9');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(1.000000, -0.45, 0.000000);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '9');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
/*End Matrix*/

    //Unrotate Z (compass)
    //glPopMatrix();

    //Pitch Ladder Rotations

/*Generated Pitch Ladder Matrix*/ 

glBegin(GL_POLYGON); 
glVertex3f(0.35, -1.000000, 0.000000);
glVertex3f(0.35, -0.990000, 0.000000);
glVertex3f(0.15, -0.990000, 0.000000);
glVertex3f(0.15, -1.000000, 0.000000);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, -1.000000, 0.000000);
glVertex3f(-0.35, -0.990000, 0.000000);
glVertex3f(-0.15, -0.990000, 0.000000);
glVertex3f(-0.15, -1.000000, 0.000000);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, -1.000000, -0.000000);
glVertex3f(0.35, -0.990000, -0.000000);
glVertex3f(0.15, -0.990000, -0.000000);
glVertex3f(0.15, -1.000000, -0.000000);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, -1.000000, -0.000000);
glVertex3f(-0.35, -0.990000, -0.000000);
glVertex3f(-0.15, -0.990000, -0.000000);
glVertex3f(-0.15, -1.000000, -0.000000);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, -0.984808, -0.173648);
glVertex3f(0.35, -0.974808, -0.173648);
glVertex3f(0.15, -0.974808, -0.173648);
glVertex3f(0.15, -0.984808, -0.173648);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, -0.984808, -0.173648);
glVertex3f(-0.35, -0.974808, -0.173648);
glVertex3f(-0.15, -0.974808, -0.173648);
glVertex3f(-0.15, -0.984808, -0.173648);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, -0.984808, 0.173648);
glVertex3f(0.35, -0.974808, 0.173648);
glVertex3f(0.15, -0.974808, 0.173648);
glVertex3f(0.15, -0.984808, 0.173648);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, -0.984808, 0.173648);
glVertex3f(-0.35, -0.974808, 0.173648);
glVertex3f(-0.15, -0.974808, 0.173648);
glVertex3f(-0.15, -0.984808, 0.173648);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, -0.939693, -0.342020);
glVertex3f(0.35, -0.929693, -0.342020);
glVertex3f(0.15, -0.929693, -0.342020);
glVertex3f(0.15, -0.939693, -0.342020);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, -0.939693, -0.342020);
glVertex3f(-0.35, -0.929693, -0.342020);
glVertex3f(-0.15, -0.929693, -0.342020);
glVertex3f(-0.15, -0.939693, -0.342020);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, -0.939693, 0.342020);
glVertex3f(0.35, -0.929693, 0.342020);
glVertex3f(0.15, -0.929693, 0.342020);
glVertex3f(0.15, -0.939693, 0.342020);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, -0.939693, 0.342020);
glVertex3f(-0.35, -0.929693, 0.342020);
glVertex3f(-0.15, -0.929693, 0.342020);
glVertex3f(-0.15, -0.939693, 0.342020);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, -0.866025, -0.500000);
glVertex3f(0.35, -0.856025, -0.500000);
glVertex3f(0.15, -0.856025, -0.500000);
glVertex3f(0.15, -0.866025, -0.500000);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, -0.866025, -0.500000);
glVertex3f(-0.35, -0.856025, -0.500000);
glVertex3f(-0.15, -0.856025, -0.500000);
glVertex3f(-0.15, -0.866025, -0.500000);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, -0.866025, 0.500000);
glVertex3f(0.35, -0.856025, 0.500000);
glVertex3f(0.15, -0.856025, 0.500000);
glVertex3f(0.15, -0.866025, 0.500000);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, -0.866025, 0.500000);
glVertex3f(-0.35, -0.856025, 0.500000);
glVertex3f(-0.15, -0.856025, 0.500000);
glVertex3f(-0.15, -0.866025, 0.500000);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, -0.766044, -0.642788);
glVertex3f(0.35, -0.756044, -0.642788);
glVertex3f(0.15, -0.756044, -0.642788);
glVertex3f(0.15, -0.766044, -0.642788);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, -0.766044, -0.642788);
glVertex3f(-0.35, -0.756044, -0.642788);
glVertex3f(-0.15, -0.756044, -0.642788);
glVertex3f(-0.15, -0.766044, -0.642788);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, -0.766044, 0.642788);
glVertex3f(0.35, -0.756044, 0.642788);
glVertex3f(0.15, -0.756044, 0.642788);
glVertex3f(0.15, -0.766044, 0.642788);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, -0.766044, 0.642788);
glVertex3f(-0.35, -0.756044, 0.642788);
glVertex3f(-0.15, -0.756044, 0.642788);
glVertex3f(-0.15, -0.766044, 0.642788);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, -0.642788, -0.766044);
glVertex3f(0.35, -0.632788, -0.766044);
glVertex3f(0.15, -0.632788, -0.766044);
glVertex3f(0.15, -0.642788, -0.766044);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, -0.642788, -0.766044);
glVertex3f(-0.35, -0.632788, -0.766044);
glVertex3f(-0.15, -0.632788, -0.766044);
glVertex3f(-0.15, -0.642788, -0.766044);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, -0.642788, 0.766044);
glVertex3f(0.35, -0.632788, 0.766044);
glVertex3f(0.15, -0.632788, 0.766044);
glVertex3f(0.15, -0.642788, 0.766044);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, -0.642788, 0.766044);
glVertex3f(-0.35, -0.632788, 0.766044);
glVertex3f(-0.15, -0.632788, 0.766044);
glVertex3f(-0.15, -0.642788, 0.766044);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, -0.500000, -0.866025);
glVertex3f(0.35, -0.490000, -0.866025);
glVertex3f(0.15, -0.490000, -0.866025);
glVertex3f(0.15, -0.500000, -0.866025);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, -0.500000, -0.866025);
glVertex3f(-0.35, -0.490000, -0.866025);
glVertex3f(-0.15, -0.490000, -0.866025);
glVertex3f(-0.15, -0.500000, -0.866025);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, -0.500000, 0.866025);
glVertex3f(0.35, -0.490000, 0.866025);
glVertex3f(0.15, -0.490000, 0.866025);
glVertex3f(0.15, -0.500000, 0.866025);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, -0.500000, 0.866025);
glVertex3f(-0.35, -0.490000, 0.866025);
glVertex3f(-0.15, -0.490000, 0.866025);
glVertex3f(-0.15, -0.500000, 0.866025);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, -0.342020, -0.939693);
glVertex3f(0.35, -0.332020, -0.939693);
glVertex3f(0.15, -0.332020, -0.939693);
glVertex3f(0.15, -0.342020, -0.939693);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, -0.342020, -0.939693);
glVertex3f(-0.35, -0.332020, -0.939693);
glVertex3f(-0.15, -0.332020, -0.939693);
glVertex3f(-0.15, -0.342020, -0.939693);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, -0.342020, 0.939693);
glVertex3f(0.35, -0.332020, 0.939693);
glVertex3f(0.15, -0.332020, 0.939693);
glVertex3f(0.15, -0.342020, 0.939693);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, -0.342020, 0.939693);
glVertex3f(-0.35, -0.332020, 0.939693);
glVertex3f(-0.15, -0.332020, 0.939693);
glVertex3f(-0.15, -0.342020, 0.939693);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, -0.173648, -0.984808);
glVertex3f(0.35, -0.163648, -0.984808);
glVertex3f(0.15, -0.163648, -0.984808);
glVertex3f(0.15, -0.173648, -0.984808);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, -0.173648, -0.984808);
glVertex3f(-0.35, -0.163648, -0.984808);
glVertex3f(-0.15, -0.163648, -0.984808);
glVertex3f(-0.15, -0.173648, -0.984808);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, -0.173648, 0.984808);
glVertex3f(0.35, -0.163648, 0.984808);
glVertex3f(0.15, -0.163648, 0.984808);
glVertex3f(0.15, -0.173648, 0.984808);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, -0.173648, 0.984808);
glVertex3f(-0.35, -0.163648, 0.984808);
glVertex3f(-0.15, -0.163648, 0.984808);
glVertex3f(-0.15, -0.173648, 0.984808);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, 0.000000, -1.000000);
glVertex3f(0.35, 0.010000, -1.000000);
glVertex3f(0.15, 0.010000, -1.000000);
glVertex3f(0.15, 0.000000, -1.000000);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, 0.000000, -1.000000);
glVertex3f(-0.35, 0.010000, -1.000000);
glVertex3f(-0.15, 0.010000, -1.000000);
glVertex3f(-0.15, 0.000000, -1.000000);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, 0.000000, 1.000000);
glVertex3f(0.35, 0.010000, 1.000000);
glVertex3f(0.15, 0.010000, 1.000000);
glVertex3f(0.15, 0.000000, 1.000000);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, 0.000000, 1.000000);
glVertex3f(-0.35, 0.010000, 1.000000);
glVertex3f(-0.15, 0.010000, 1.000000);
glVertex3f(-0.15, 0.000000, 1.000000);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, 0.173648, -0.984808);
glVertex3f(0.35, 0.183648, -0.984808);
glVertex3f(0.15, 0.183648, -0.984808);
glVertex3f(0.15, 0.173648, -0.984808);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, 0.173648, -0.984808);
glVertex3f(-0.35, 0.183648, -0.984808);
glVertex3f(-0.15, 0.183648, -0.984808);
glVertex3f(-0.15, 0.173648, -0.984808);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, 0.173648, 0.984808);
glVertex3f(0.35, 0.183648, 0.984808);
glVertex3f(0.15, 0.183648, 0.984808);
glVertex3f(0.15, 0.173648, 0.984808);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, 0.173648, 0.984808);
glVertex3f(-0.35, 0.183648, 0.984808);
glVertex3f(-0.15, 0.183648, 0.984808);
glVertex3f(-0.15, 0.173648, 0.984808);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, 0.342020, -0.939693);
glVertex3f(0.35, 0.352020, -0.939693);
glVertex3f(0.15, 0.352020, -0.939693);
glVertex3f(0.15, 0.342020, -0.939693);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, 0.342020, -0.939693);
glVertex3f(-0.35, 0.352020, -0.939693);
glVertex3f(-0.15, 0.352020, -0.939693);
glVertex3f(-0.15, 0.342020, -0.939693);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, 0.342020, 0.939693);
glVertex3f(0.35, 0.352020, 0.939693);
glVertex3f(0.15, 0.352020, 0.939693);
glVertex3f(0.15, 0.342020, 0.939693);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, 0.342020, 0.939693);
glVertex3f(-0.35, 0.352020, 0.939693);
glVertex3f(-0.15, 0.352020, 0.939693);
glVertex3f(-0.15, 0.342020, 0.939693);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, 0.500000, -0.866025);
glVertex3f(0.35, 0.510000, -0.866025);
glVertex3f(0.15, 0.510000, -0.866025);
glVertex3f(0.15, 0.500000, -0.866025);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, 0.500000, -0.866025);
glVertex3f(-0.35, 0.510000, -0.866025);
glVertex3f(-0.15, 0.510000, -0.866025);
glVertex3f(-0.15, 0.500000, -0.866025);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, 0.500000, 0.866025);
glVertex3f(0.35, 0.510000, 0.866025);
glVertex3f(0.15, 0.510000, 0.866025);
glVertex3f(0.15, 0.500000, 0.866025);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, 0.500000, 0.866025);
glVertex3f(-0.35, 0.510000, 0.866025);
glVertex3f(-0.15, 0.510000, 0.866025);
glVertex3f(-0.15, 0.500000, 0.866025);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, 0.642788, -0.766044);
glVertex3f(0.35, 0.652788, -0.766044);
glVertex3f(0.15, 0.652788, -0.766044);
glVertex3f(0.15, 0.642788, -0.766044);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, 0.642788, -0.766044);
glVertex3f(-0.35, 0.652788, -0.766044);
glVertex3f(-0.15, 0.652788, -0.766044);
glVertex3f(-0.15, 0.642788, -0.766044);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, 0.642788, 0.766044);
glVertex3f(0.35, 0.652788, 0.766044);
glVertex3f(0.15, 0.652788, 0.766044);
glVertex3f(0.15, 0.642788, 0.766044);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, 0.642788, 0.766044);
glVertex3f(-0.35, 0.652788, 0.766044);
glVertex3f(-0.15, 0.652788, 0.766044);
glVertex3f(-0.15, 0.642788, 0.766044);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, 0.766044, -0.642788);
glVertex3f(0.35, 0.776044, -0.642788);
glVertex3f(0.15, 0.776044, -0.642788);
glVertex3f(0.15, 0.766044, -0.642788);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, 0.766044, -0.642788);
glVertex3f(-0.35, 0.776044, -0.642788);
glVertex3f(-0.15, 0.776044, -0.642788);
glVertex3f(-0.15, 0.766044, -0.642788);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, 0.766044, 0.642788);
glVertex3f(0.35, 0.776044, 0.642788);
glVertex3f(0.15, 0.776044, 0.642788);
glVertex3f(0.15, 0.766044, 0.642788);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, 0.766044, 0.642788);
glVertex3f(-0.35, 0.776044, 0.642788);
glVertex3f(-0.15, 0.776044, 0.642788);
glVertex3f(-0.15, 0.766044, 0.642788);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, 0.866025, -0.500000);
glVertex3f(0.35, 0.876025, -0.500000);
glVertex3f(0.15, 0.876025, -0.500000);
glVertex3f(0.15, 0.866025, -0.500000);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, 0.866025, -0.500000);
glVertex3f(-0.35, 0.876025, -0.500000);
glVertex3f(-0.15, 0.876025, -0.500000);
glVertex3f(-0.15, 0.866025, -0.500000);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, 0.866025, 0.500000);
glVertex3f(0.35, 0.876025, 0.500000);
glVertex3f(0.15, 0.876025, 0.500000);
glVertex3f(0.15, 0.866025, 0.500000);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, 0.866025, 0.500000);
glVertex3f(-0.35, 0.876025, 0.500000);
glVertex3f(-0.15, 0.876025, 0.500000);
glVertex3f(-0.15, 0.866025, 0.500000);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, 0.939693, -0.342020);
glVertex3f(0.35, 0.949693, -0.342020);
glVertex3f(0.15, 0.949693, -0.342020);
glVertex3f(0.15, 0.939693, -0.342020);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, 0.939693, -0.342020);
glVertex3f(-0.35, 0.949693, -0.342020);
glVertex3f(-0.15, 0.949693, -0.342020);
glVertex3f(-0.15, 0.939693, -0.342020);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, 0.939693, 0.342020);
glVertex3f(0.35, 0.949693, 0.342020);
glVertex3f(0.15, 0.949693, 0.342020);
glVertex3f(0.15, 0.939693, 0.342020);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, 0.939693, 0.342020);
glVertex3f(-0.35, 0.949693, 0.342020);
glVertex3f(-0.15, 0.949693, 0.342020);
glVertex3f(-0.15, 0.939693, 0.342020);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, 0.984808, -0.173648);
glVertex3f(0.35, 0.994808, -0.173648);
glVertex3f(0.15, 0.994808, -0.173648);
glVertex3f(0.15, 0.984808, -0.173648);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, 0.984808, -0.173648);
glVertex3f(-0.35, 0.994808, -0.173648);
glVertex3f(-0.15, 0.994808, -0.173648);
glVertex3f(-0.15, 0.984808, -0.173648);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, 0.984808, 0.173648);
glVertex3f(0.35, 0.994808, 0.173648);
glVertex3f(0.15, 0.994808, 0.173648);
glVertex3f(0.15, 0.984808, 0.173648);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, 0.984808, 0.173648);
glVertex3f(-0.35, 0.994808, 0.173648);
glVertex3f(-0.15, 0.994808, 0.173648);
glVertex3f(-0.15, 0.984808, 0.173648);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, 1.000000, 0.000000);
glVertex3f(0.35, 1.010000, 0.000000);
glVertex3f(0.15, 1.010000, 0.000000);
glVertex3f(0.15, 1.000000, 0.000000);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, 1.000000, 0.000000);
glVertex3f(-0.35, 1.010000, 0.000000);
glVertex3f(-0.15, 1.010000, 0.000000);
glVertex3f(-0.15, 1.000000, 0.000000);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(0.35, 1.000000, -0.000000);
glVertex3f(0.35, 1.010000, -0.000000);
glVertex3f(0.15, 1.010000, -0.000000);
glVertex3f(0.15, 1.000000, -0.000000);
glEnd(); 
glBegin(GL_POLYGON); 
glVertex3f(-0.35, 1.000000, -0.000000);
glVertex3f(-0.35, 1.010000, -0.000000);
glVertex3f(-0.15, 1.010000, -0.000000);
glVertex3f(-0.15, 1.000000, -0.000000);
glEnd(); 
glRasterPos3f(0.4, -1.000000, 0.000000);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '-');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '9');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, -0.984808, 0.173648);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '-');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '8');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, -0.984808, -0.173648);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '-');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '8');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, -0.939693, 0.342020);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '-');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '7');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, -0.939693, -0.342020);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '-');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '7');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, -0.866025, 0.500000);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '-');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '6');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, -0.866025, -0.500000);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '-');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '6');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, -0.766044, 0.642788);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '-');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '5');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, -0.766044, -0.642788);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '-');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '5');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, -0.642788, 0.766044);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '-');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '4');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, -0.642788, -0.766044);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '-');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '4');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, -0.500000, 0.866025);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '-');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '3');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, -0.500000, -0.866025);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '-');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '3');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, -0.342020, 0.939693);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '-');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '2');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, -0.342020, -0.939693);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '-');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '2');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, -0.173648, 0.984808);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '-');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '1');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, -0.173648, -0.984808);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '-');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '1');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, 0.000000, 1.000000);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, 0.000000, -1.000000);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, 0.173648, 0.984808);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '1');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, 0.173648, -0.984808);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '1');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, 0.342020, 0.939693);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '2');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, 0.342020, -0.939693);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '2');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, 0.500000, 0.866025);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '3');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, 0.500000, -0.866025);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '3');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, 0.642788, 0.766044);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '4');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, 0.642788, -0.766044);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '4');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, 0.766044, 0.642788);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '5');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, 0.766044, -0.642788);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '5');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, 0.866025, 0.500000);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '6');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, 0.866025, -0.500000);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '6');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, 0.939693, 0.342020);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '7');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, 0.939693, -0.342020);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '7');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, 0.984808, 0.173648);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '8');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, 0.984808, -0.173648);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '8');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');
glRasterPos3f(0.4, 1.000000, 0.000000);
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '9');
glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, '0');

/*End Generated Polygon Matrix*/ 

    //glPopMatrix ();
    //glPopMatrix ();
    glFlush();
}

/* End Display Loop */

/*--comment--
  This function is a placeholder for gyroscope input:
  w = Pitch Up
  s = Pitch Down
  a = Roll Left
  d = Roll Right
  q = Yaw Left
  e = Yaw Right
  --end comment--*/

void keyUp (unsigned char key, int x, int y) {
  if (key == 'w'){
  report.x_orientation += 0;
  report.y_orientation += 0;
  report.z_orientation += 0;
  displayMe();
  }
  return;
}  

int read_IMU(int file){
  int check_interrupt = read_reg8(file, MPU_6050, INT_STATUS) & 0b00000001;

  if (check_interrupt){

  /*initialize variables*/
  int16_t gyro_raw[3];
  int16_t accel_raw[3];
  int16_t temp[1];
  double gyro_corrected[3];
  double accel_corrected[3];
  double cycle_time;

  /*solve for time since last cycle*/
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cycle_clock);
  //cycle_time = (cycle_clock.tv_sec + (cycle_clock.tv_nsec * 1e-9)) - (cycle_clock_last.tv_sec + (cycle_clock_last.tv_nsec * 1e-9));
  cycle_time = ((cycle_clock.tv_sec - cycle_clock_last.tv_sec) + (cycle_clock.tv_nsec * 1e-9)) - (cycle_clock_last.tv_nsec * 1e-9);
  cycle_clock_last = cycle_clock;
  //printf("%.6f Hertz", (1.0/cycle_time));
  //printf("\e[1;1H\e[2J");
  //printf("Cycle time:  %.6f\n", cycle_time);
  /*end time calculations*/


  /* Burst read to try and reduce error */
  const gchar *buffer;
	if (ioctl(file, I2C_SLAVE, MPU_6050) < 0) {
		printf("%s \n", "DWORD read failed...failed to acquire bus access and/or talk to slave.");
		return 1;
	}
  char buf0[14] = {0};
  char reg_addr[1];
  reg_addr[0] = ACCEL_XOUT_H;
  if (write(file,reg_addr,1) != 1){
      /* ERROR HANDLING: i2c transaction failed */
      printf("Failed to write to the i2c bus.\n");
      buffer = g_strerror(errno);
      printf("%s",buffer);
      printf("\n\n");
  }
  if (read(file,buf0,14) != 14){
      /* ERROR HANDLING: i2c transaction failed */
      printf("Failed to read from the i2c bus.\n");
      buffer = g_strerror(errno);
      printf("%s",buffer);
      printf("\n\n");
  }


  /* transcribe rotations from buffer to variables */
  gyro_raw[0] = (buf0[8] << 8 ) | buf0[9];
  gyro_raw[1] = (buf0[10] << 8 ) | buf0[11];
  gyro_raw[2] = (buf0[12] << 8 ) | buf0[13];
  accel_raw[0] = (buf0[0] << 8 ) | buf0[1];
  accel_raw[1] = (buf0[2] << 8 ) | buf0[3];
  accel_raw[2] = (buf0[4] << 8 ) | buf0[5];
  temp[0] = (buf0[6] << 8 ) | buf0[7];
  //temp[1] = (buf0[6] << 8 ) | buf0[8];
  /* end IMU reads*/

  /* correct raw values (subtrack idle drift -> multiply by manufacturers constant -> convert to radians for trig functions*/

  gyro_corrected[0] = (((double)(gyro_raw[0] - report.gyro_x_zero) / GYRO_CONSTANT) * cycle_time);// * ( M_PI / 180 );
  gyro_corrected[1] = (((double)(gyro_raw[1] - report.gyro_y_zero) / GYRO_CONSTANT) * cycle_time);// * ( M_PI / 180 );
  gyro_corrected[2] = (((double)(gyro_raw[2] - report.gyro_z_zero) / GYRO_CONSTANT) * cycle_time);// * ( M_PI / 180 );
  //accel_corrected[0] = (float)accel_raw[0] / ACCEL_CONSTANT;
  //accel_corrected[1] = (float)accel_raw[1] / ACCEL_CONSTANT;
  //accel_corrected[2] = (float)accel_raw[2] / ACCEL_CONSTANT;
  //printf("Gyro Raw Output:  X_Axis: %.6f   Y_Axis: %.6f   Z_Axis: %.6f\n\n", gyro_corrected[0], gyro_corrected[1], gyro_corrected[2]);

  /* end corrections */
  

  report.x_orientation = gyro_corrected[0];
  report.y_orientation = gyro_corrected[1];
  report.z_orientation = gyro_corrected[2];


  }
  else printf("%s \n", "data not ready");
  return 1;

}

/* deviation function unused in current model */
float deviation_16(int file, int device, int register_address ){

  /* initialize variables */
  int dataset[200];
  float mean;
  float mean_time;
  float deviation;
  int counter;
  mean = 0;
  deviation = 0;
  /* end */

  /* set clock and reset gyro register */
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cycle_clock_last);
  read_reg16( file, device, register_address );
  /* end */

  /* generate 200 data samples */
  for (counter = 0; counter < 200; counter++){
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cycle_clock);
    mean_time += (cycle_clock.tv_sec + (cycle_clock.tv_nsec * 1e-9)) - (cycle_clock_last.tv_sec + (cycle_clock_last.tv_nsec * 1e-9));
    cycle_clock_last = cycle_clock;
    dataset[counter] = read_reg16( file, device, register_address ) / GYRO_CONSTANT;
    mean += dataset[counter];
  }
  /* end */


  /* calculate average values */
  mean = mean / 200;
  mean_time = mean_time / 200;
  /* end */

  /* add distance from mean for all data points */
  for (counter = 0; counter < 200; counter++){
    deviation += fabsf(mean - (float)dataset[counter]);
  }

  /* divide by number of samples to get average deviation,
   * muliply by average sample time to get deviation per
   * second of operation                                  */
  deviation = (deviation / 200) / mean_time;

  return deviation;
}

int main(int argc, char** argv)
{
    printf("%s \n\n", "Brosnan Inertial Navigation V.0");

    /* establish i2c link */

    int file;
	char *filename = "/dev/i2c-1";

	printf("%s \n", "Connecting to I2C device");
	if ((file = open(filename, O_RDWR)) < 0) {
      /*error handling*/
      perror("Failed to open the i2c bus (0)");
      exit(1);
    }
	if (ioctl(file, I2C_SLAVE, MPU_6050) < 0) {
      /*error handling*/
      printf("%s \n", "Failed to acquire bus access and/or talk to slave.");
      exit(1);
    }
    printf("%s \n", "done");

    /*wake up servo controller*/
    printf("%s \n", "Booting IMU...");

    write_reg8(file, MPU_6050, PWR_MGMT_1, 0x0);
    sleep(1);
    printf("%s \n", "done");

    /*set low pass filter */
    printf("%s \n", "setting low pass filter...");
    write_reg8(file, MPU_6050, CONFIG, 0b00000110);
    sleep(1);

    /*set sensitivity */
    printf("%s \n%s \n", "done.", "setting gyroscope sensitivity to 2000 Deg/Sec...");
    write_reg8(file, MPU_6050, GYRO_CONFIG, 0b00011000);
    sleep(1);
    printf("%s \n", "done");

    /*set sample frequency */
    printf("%s \n", "setting internal sample rate...");
    write_reg8(file, MPU_6050, SMPLRT_DIV, 0b00011111);
    sleep(1);
    printf("%s \n", "done");


    /*set data interrupt enable */
    printf("%s \n", "turning on interrupt...");
    write_reg8(file, MPU_6050, INT_ENABLE, 0b00000001);
    sleep(1);
    printf("%s \n", "done");


    /* end i2c */

    /* opengl initialize */

    glutInit(&argc, argv);  //do I want to pass all cmdline to glut?
    glutInitDisplayMode(GLUT_SINGLE);
    glutInitWindowSize(800, 800);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("Gimbal");
    glEnable(GL_DEPTH_TEST);  //required to draw objects on top of others
    glDepthFunc(GL_LESS);  //also required to draw objects on top of others
    glMatrixMode(GL_PROJECTION); //useful for projecting gimbal onto real world
    glLoadIdentity();
    glOrtho(-0.5, 0.5, -0.5, 0.5, 0.01, 2); //field of view stuff
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glutKeyboardUpFunc(keyUp); //useless with current display loop...
    glutDisplayFunc(displayMe);

    /* end opengl */

    /* zero gyro */
    int32_t gyro_x_total;
    int32_t gyro_y_total;
    int32_t gyro_z_total;
    gyro_x_total = 0;
    gyro_y_total = 0;
    gyro_z_total = 0;
    int looper;
    printf("%s \n", "calculating gyro drift...");
    for (looper = 0; looper < 300 ; looper++){
      gyro_x_total += (int32_t)read_reg16(file, MPU_6050, GYRO_XOUT_H);
      gyro_y_total += (int32_t)read_reg16(file, MPU_6050, GYRO_YOUT_H);
      gyro_z_total += (int32_t)read_reg16(file, MPU_6050, GYRO_ZOUT_H);
    }
    report.gyro_x_zero = gyro_x_total / 300.0;
    report.gyro_y_zero = gyro_y_total / 300.0;
    report.gyro_z_zero = gyro_z_total / 300.0;
    printf("%s \n", "done.");
    /* end */
    //report.gyro_x_zero = 0;
    //report.gyro_y_zero = 0;
    //report.gyro_z_zero = 0;

    /* populate deviation value for each gyroscope axis */

    printf("%s \n", "calculating X axis deviation...");
    report.gyro_x_deviation = deviation_16(file, MPU_6050, GYRO_XOUT_H);
    printf("%s \n", "calculating y axis deviation...");
    report.gyro_y_deviation = deviation_16(file, MPU_6050, GYRO_YOUT_H);
    printf("%s \n", "calculating z axis deviation...");
    report.gyro_z_deviation = deviation_16(file, MPU_6050, GYRO_ZOUT_H);
    printf("%s \n", "done.");


    /* start IMU program */

    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cycle_clock_last);
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cycle_clock);

    report.x_orientation = 0.0;
    report.y_orientation  = 0.0;
    report.z_orientation = 0.0;

    while(1){
     read_IMU(file);
     displayMe();
    }


    glutMainLoop();

    return 0;

}
