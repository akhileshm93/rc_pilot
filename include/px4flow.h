/* Standard Libraries */
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>   // For I2C
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

/* 7 Bit I2C Address of the Flow Module
   Default 0x42 (user selectable bits 0,1,2) */
#define I2C_PX4FLOW_PORT 1
#define I2C_PX4FLOW_ADDRESS 0x42
#define PX4FLOW_FREQ 60 // in Hz
#define PX4FLOW_NUM_SAMPLES 20

/* As described in the documentation
   http://pixhawk.org/modules/px4flow */
typedef struct px4flow_frame{
    uint16_t frame_count;// counts created I2C frames
    int16_t pixel_flow_x_sum;// accumulated x flow in pixels*10 since last I2C frame
    int16_t pixel_flow_y_sum;// accumulated y flow in pixels*10 since last I2C frame
    int16_t flow_comp_m_x;// x velocity*1000 in meters / timestep
    int16_t flow_comp_m_y;// y velocity*1000 in meters / timestep
    int16_t qual;// Optical flow quality / confidence 0: bad, 255: maximum quality
    int16_t gyro_x_rate; //gyro x rate
    int16_t gyro_y_rate; //gyro y rate
    int16_t gyro_z_rate; //gyro z rate
    uint8_t gyro_range; // gyro range
    uint8_t sonar_timestamp;// timestep in milliseconds between I2C frames
    int16_t ground_distance;// Ground distance in meters*1000. Positive value: distance known. Negative value: Unknown distance
} px4flow_frame;

typedef struct px4flow_integral_frame{
    uint16_t frame_count_since_last_readout;//number of flow measurements since last I2C readout [#frames]
    int16_t pixel_flow_x_integral;//accumulated flow in radians*10000 around x axis since last I2C readout [rad*10000]
    int16_t pixel_flow_y_integral;//accumulated flow in radians*10000 around y axis since last I2C readout [rad*10000]
    int16_t gyro_x_rate_integral;//accumulated gyro x rates in radians*10000 since last I2C readout [rad*10000] 
    int16_t gyro_y_rate_integral;//accumulated gyro y rates in radians*10000 since last I2C readout [rad*10000] 
    int16_t gyro_z_rate_integral;//accumulated gyro z rates in radians*10000 since last I2C readout [rad*10000] 
    uint32_t integration_timespan;//accumulation timespan in microseconds since last I2C readout [microseconds]
    uint32_t sonar_timestamp;// time since last sonar update [microseconds]
    int16_t ground_distance;// Ground distance in meters*1000 [meters*1000]
    int16_t gyro_temperature;// Temperature * 100 in centi-degrees Celsius [degcelsius*100]
    uint8_t quality;// averaged quality of accumulated flow values [0:bad quality;255: max quality]
} px4flow_integral_frame;

typedef struct I2C_data
{
  uint8_t         port;            // I2C port
  uint8_t         address;         // I2C device address (7-bit value)
  int             fd;              // I2C file descriptor
} I2C_data;

// functions
int PX4Flow_Initialize(I2C_data *px4flow);
int PX4Flow_ReadFrame(I2C_data *px4flow, px4flow_frame *frame);
int PX4Flow_ReadIntFrame(I2C_data *px4flow, px4flow_integral_frame *iframe);
<<<<<<< HEAD
int PX4Flow_ReadAllFrames(I2C_data *px4flow, px4flow_frame *frame, px4flow_integral_frame *iframe);
=======
<<<<<<< HEAD
int PX4Flow_ReadAllFrames(I2C_data *px4flow, px4flow_frame *frame, px4flow_integral_frame *iframe);
=======
<<<<<<< HEAD
int PX4Flow_ReadAllFrames(I2C_data *px4flow, px4flow_frame *frame, px4flow_integral_frame *iframe);
=======
int PX4Flow_ReadAllFrames(I2C_data *px4flow, px4flow_frame *frame, px4flow_integral_frame *iframe);
>>>>>>> 10ac3fbd078868976fcbbd45d462eac1f3cc19c3
>>>>>>> 5f08dc3a05cac60167486d13896ca274dfdd273a
>>>>>>> ad3cf265d1ec65fa8bc0428646a415059218c915
