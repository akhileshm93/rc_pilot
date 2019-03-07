
#include "px4flow.h"

// void * PX4Flow_Run(){
//     struct I2C_data flow;
//     struct px4flow_frame frame;
//     struct px4flow_integral_frame iframe;
//     /* Initialize PX4Flow */
//     if(PX4Flow_Initialize(&flow) == -1) return NULL;
//     while(1){
//         PX4Flow_ReadFrame(&flow, &frame);
//         PX4Flow_ReadIntFrame(&flow, &iframe);
//     }
// }

int PX4Flow_Initialize(I2C_data *px4flow){

    px4flow->port = I2C_PX4FLOW_PORT;
    px4flow->address = (I2C_PX4FLOW_ADDRESS << 1);
    return 0;
}

int PX4Flow_ReadFrame(I2C_data *px4flow, px4flow_frame *frame){

    //send 0x0 to PX4FLOW module and receive back 22 Bytes data 
    uint8_t buffer[22];
    uint8_t reg = 0x0;

	char str[16];
	sprintf(str, "/dev/i2c-%d", px4flow->port);
	int16_t fd = open(str, O_RDWR);
	if (fd < 0)
	{
		fprintf(stderr, "ERROR: Failed to open i2c device\n");
		close(fd);
		return -1;
	}
	//setup i2c
	if (ioctl(fd, I2C_SLAVE, (px4flow->address)>>1) < 0)
	{  
		fprintf(stderr, "ERROR: ioctl slave address change failed\n");
        fprintf(stderr, "Address: 0x%x\n", px4flow->address);
		close(fd);
		return -1;
	}

    write(fd, &reg, 1);
    int16_t ret = read(fd, buffer, 22);
	
    if (ret != 22)
	{
		fprintf(stderr, "ERROR: did not read correct number of bytes\n");
		close(fd);
		return -1;
	}

    frame->frame_count       = buffer[0] + (uint16_t)(buffer[1] << 8);
    frame->pixel_flow_x_sum  = buffer[2] + (uint16_t)(buffer[3] << 8);
    frame->pixel_flow_y_sum  = buffer[4] + (uint16_t)(buffer[5] << 8);
    frame->flow_comp_m_x     = buffer[6] + (uint16_t)(buffer[7] << 8);
    frame->flow_comp_m_y     = buffer[8] + (uint16_t)(buffer[9] << 8);
    frame->qual              = buffer[10] + (uint16_t)(buffer[11] << 8);
    frame->gyro_x_rate       = buffer[12] + (uint16_t)(buffer[13] << 8);
    frame->gyro_y_rate       = buffer[14] + (uint16_t)(buffer[15] << 8);
    frame->gyro_z_rate       = buffer[16] + (uint16_t)(buffer[17] << 8);
    frame->gyro_range        = (uint8_t)(buffer[18]);
    frame->sonar_timestamp   = (uint8_t)(buffer[19]);
    frame->ground_distance   = buffer[20] + (uint16_t)(buffer[21] << 8);
    close(fd);
    return 0;
}

int PX4Flow_ReadIntFrame(I2C_data *px4flow, px4flow_integral_frame *iframe){

    //send 0x16 to PX4FLOW module and receive back 25 Bytes data 
    // changed length to 26 based on bug reported to github
    uint8_t buffer[26];
    uint8_t reg = 0x16;
    
    
    char str[16];
	sprintf(str, "/dev/i2c-%d", px4flow->port);
	int16_t fd = open(str, O_RDWR);
	if (fd < 0)
	{
		fprintf(stderr, "ERROR: Failed to open i2c device\n");
		close(fd);
		return -1;
	}
	//setup i2c
	if (ioctl(fd, I2C_SLAVE, (px4flow->address)>>1) < 0)
	{  
		fprintf(stderr, "ERROR: ioctl slave address change failed\n");
        fprintf(stderr, "Address: 0x%x\n", px4flow->address);
		close(fd);
		return -1;
	}

<<<<<<< HEAD
    write(fd, &reg, 1);
=======
    write(fd, &reg, 2);
>>>>>>> 440b64bd7750f358244b630e0963738ff62d3bd3
    int16_t ret = read(fd, buffer, 26);
	
    if (ret != 26)
	{
		fprintf(stderr, "ERROR: did not read correct number of bytes\n");
		close(fd);
		return -1;
	}

    iframe->frame_count_since_last_readout = buffer[0] + (uint16_t)(buffer[1] << 8);
    iframe->pixel_flow_x_integral  = buffer[2] + (uint16_t)(buffer[3] << 8);
    iframe->pixel_flow_y_integral  = buffer[4] + (uint16_t)(buffer[5] << 8);
    iframe->gyro_x_rate_integral   = buffer[6] + (uint16_t)(buffer[7] << 8);
    iframe->gyro_y_rate_integral   = buffer[8] + (uint16_t)(buffer[9] << 8);
    iframe->gyro_z_rate_integral   = buffer[10] + (uint16_t)(buffer[11] << 8);
    iframe->integration_timespan   = (uint32_t)(buffer[12] + (uint16_t)(buffer[13] << 8))
                           + (uint32_t)((buffer[14] + (uint16_t)(buffer[15] << 8)) << 16);
    iframe->sonar_timestamp        = (uint32_t)(buffer[16] + (uint16_t)(buffer[17] << 8))
                           + (uint32_t)((buffer[18] + (uint16_t)(buffer[19] << 8)) << 16);
    iframe->ground_distance        = buffer[20] + (uint16_t)(buffer[21] << 8);
    iframe->gyro_temperature       = buffer[22] + (uint16_t)(buffer[23] << 8);
    iframe->quality                = (uint8_t)(buffer[24]);
    close(fd);
    return 0;

}

<<<<<<< HEAD
int PX4Flow_ReadAllFrames(I2C_data *px4flow, px4flow_frame *frame, px4flow_integral_frame *iframe){

    //send 0x0 to PX4FLOW module and receive back 48 Bytes data 
    uint8_t buffer[48];
    uint8_t reg = 0x0;

	char str[16];
	sprintf(str, "/dev/i2c-%d", px4flow->port);
	int16_t fd = open(str, O_RDWR);
	if (fd < 0)
	{
		fprintf(stderr, "ERROR: Failed to open i2c device\n");
		close(fd);
		return -1;
	}
	//setup i2c
	if (ioctl(fd, I2C_SLAVE, (px4flow->address)>>1) < 0)
	{  
		fprintf(stderr, "ERROR: ioctl slave address change failed\n");
        fprintf(stderr, "Address: 0x%x\n", px4flow->address);
		close(fd);
		return -1;
	}

    write(fd, &reg, 1);
    int16_t ret = read(fd, buffer, 48);
	
    if (ret != 48)
	{
		fprintf(stderr, "ERROR: did not read correct number of bytes\n");
		close(fd);
		return -1;
	}

    frame->frame_count       = buffer[0] + (uint16_t)(buffer[1] << 8);
    frame->pixel_flow_x_sum  = buffer[2] + (uint16_t)(buffer[3] << 8);
    frame->pixel_flow_y_sum  = buffer[4] + (uint16_t)(buffer[5] << 8);
    frame->flow_comp_m_x     = buffer[6] + (uint16_t)(buffer[7] << 8);
    frame->flow_comp_m_y     = buffer[8] + (uint16_t)(buffer[9] << 8);
    frame->qual              = buffer[10] + (uint16_t)(buffer[11] << 8);
    frame->gyro_x_rate       = buffer[12] + (uint16_t)(buffer[13] << 8);
    frame->gyro_y_rate       = buffer[14] + (uint16_t)(buffer[15] << 8);
    frame->gyro_z_rate       = buffer[16] + (uint16_t)(buffer[17] << 8);
    frame->gyro_range        = (uint8_t)(buffer[18]);
    frame->sonar_timestamp   = (uint8_t)(buffer[19]);
    frame->ground_distance   = buffer[20] + (uint16_t)(buffer[21] << 8);
    
    iframe->frame_count_since_last_readout = buffer[22+0] + (uint16_t)(buffer[22+1] << 8);
    iframe->pixel_flow_x_integral  = buffer[22+2] + (uint16_t)(buffer[22+3] << 8);
    iframe->pixel_flow_y_integral  = buffer[22+4] + (uint16_t)(buffer[22+5] << 8);
    iframe->gyro_x_rate_integral   = buffer[22+6] + (uint16_t)(buffer[22+7] << 8);
    iframe->gyro_y_rate_integral   = buffer[22+8] + (uint16_t)(buffer[22+9] << 8);
    iframe->gyro_z_rate_integral   = buffer[22+10] + (uint16_t)(buffer[22+11] << 8);
    iframe->integration_timespan   = (uint32_t)(buffer[22+12] + (uint16_t)(buffer[22+13] << 8))
                           + (uint32_t)((buffer[22+14] + (uint16_t)(buffer[22+15] << 8)) << 16);
    iframe->sonar_timestamp        = (uint32_t)(buffer[22+16] + (uint16_t)(buffer[22+17] << 8))
                           + (uint32_t)((buffer[22+18] + (uint16_t)(buffer[22+19] << 8)) << 16);
    iframe->ground_distance        = buffer[22+20] + (uint16_t)(buffer[22+21] << 8);
    iframe->gyro_temperature       = buffer[22+22] + (uint16_t)(buffer[22+23] << 8);
    iframe->quality                = (uint8_t)(buffer[22+24]);
    close(fd);
    return 0;
}

=======
>>>>>>> 440b64bd7750f358244b630e0963738ff62d3bd3
