/**
 * @file state_estimator.c
 *
 */

#include <stdio.h>
#include <math.h>
#include <rc/math/filter.h>
#include <rc/math/kalman.h>
#include <rc/math/quaternion.h>
#include <rc/math/matrix.h>
#include <rc/math/other.h>
#include <rc/start_stop.h>
#include <rc/led.h>
#include <rc/mpu.h>
#include <rc/adc.h>
#include <rc/time.h>
#include <rc/bmp.h>

#include <rc_pilot_defs.h>
#include <state_estimator.h>
#include <settings.h>

#include "vl53l1x.h"
//#include "px4flow.h"
#include <robotcontrol.h>
#include <xbee_packet_t.h>

#define TWO_PI (M_PI*2.0)

state_estimate_t state_estimate; // extern variable in state_estimator.h

//optic flow senser data 
I2C_data px4flow;
px4flow_frame frame;
px4flow_integral_frame iframe;

double px = 0;
double py = 0;
double focal_length_px = (16) / (4.0f * 6.0f) * 1000.0f;


// Lidar data
VL53L1_Dev_t Device;
uint8_t addr = VL53L1X_DEFAULT_DEVICE_ADDRESS;
uint8_t i2cbus = 1;
int16_t status = 0;
uint16_t rtn;

// sensor data structs
rc_mpu_data_t mpu_data;
static rc_bmp_data_t bmp_data;

// battery filter
static rc_filter_t batt_lp = RC_FILTER_INITIALIZER;

// altitude filter components
static rc_kalman_t alt_kf = RC_KALMAN_INITIALIZER;
static rc_filter_t acc_lp = RC_FILTER_INITIALIZER;
static rc_filter_t PX4_lp = RC_FILTER_INITIALIZER;

// counter to slow down I2C readings
int counter = 1;
int *counter_pt = &counter;

static void __batt_init(void)
{
	// init the battery low pass filter
	rc_filter_moving_average(&batt_lp, 20, DT);
	double tmp = rc_adc_dc_jack();
	if(tmp<3.0){
		tmp = settings.v_nominal;
		if(settings.warnings_en){
			fprintf(stderr, "WARNING: ADC read %0.1fV on the barrel jack. Please connect\n");
			fprintf(stderr, "battery to barrel jack, assuming nominal voltage for now.\n");
		}
	}
	rc_filter_prefill_inputs(&batt_lp, tmp);
	rc_filter_prefill_outputs(&batt_lp, tmp);
	return;
}


static void __batt_march(void)
{
	double tmp = rc_adc_dc_jack();
	if(tmp<3.0) tmp = settings.v_nominal;
	state_estimate.v_batt_raw = tmp;
	state_estimate.v_batt_lp = rc_filter_march(&batt_lp, tmp);
	return;
}

static void __batt_cleanup(void)
{
	rc_filter_free(&batt_lp);
	return;
}


static void __imu_march(void)
{
	static double last_yaw = 0.0;
	static int num_yaw_spins = 0;
	double diff;

	// gyro and accel require converting to NED coordinates
	state_estimate.gyro[0] =  mpu_data.gyro[1];
	state_estimate.gyro[1] =  mpu_data.gyro[0];
	state_estimate.gyro[2] = -mpu_data.gyro[2];
	state_estimate.accel[0] =  mpu_data.accel[1];
	state_estimate.accel[1] =  mpu_data.accel[0];
	state_estimate.accel[2] = -mpu_data.accel[2];

	// quaternion also needs coordinate transform
	state_estimate.quat_imu[0] =  mpu_data.dmp_quat[0]; // W
	state_estimate.quat_imu[1] =  mpu_data.dmp_quat[2]; // X (i)
	state_estimate.quat_imu[2] =  mpu_data.dmp_quat[1]; // Y (j)
	state_estimate.quat_imu[3] = -mpu_data.dmp_quat[3]; // Z (k)

	// normalize it just in case
	rc_quaternion_norm_array(state_estimate.quat_imu);
	// generate tait bryan angles
	rc_quaternion_to_tb_array(state_estimate.quat_imu, state_estimate.tb_imu);

	// yaw is more annoying since we have to detect spins
	// also make sign negative since NED coordinates has Z point down
	diff = state_estimate.tb_imu[2] + (num_yaw_spins * TWO_PI) - last_yaw;
	//detect the crossover point at +-PI and update num yaw spins
	if(diff < -M_PI) num_yaw_spins++;
	else if(diff > M_PI) num_yaw_spins--;

	// finally the new value can be written
	state_estimate.imu_continuous_yaw = state_estimate.tb_imu[2] + (num_yaw_spins * TWO_PI);
	last_yaw = state_estimate.imu_continuous_yaw;
	return;
}


static void __mag_march(void)
{
	static double last_yaw = 0.0;
	static int num_yaw_spins = 0;

	// don't do anything if mag isn't enabled
	if(!settings.enable_magnetometer) return;

	// mag require converting to NED coordinates
	state_estimate.mag[0] =  mpu_data.mag[1];
	state_estimate.mag[1] =  mpu_data.mag[0];
	state_estimate.mag[2] = -mpu_data.mag[2];

	// quaternion also needs coordinate transform
	state_estimate.quat_mag[0] =  mpu_data.fused_quat[0]; // W
	state_estimate.quat_mag[1] =  mpu_data.fused_quat[2]; // X (i)
	state_estimate.quat_mag[2] =  mpu_data.fused_quat[1]; // Y (j)
	state_estimate.quat_mag[3] = -mpu_data.fused_quat[3]; // Z (k)

	// normalize it just in case
	rc_quaternion_norm_array(state_estimate.quat_mag);
	// generate tait bryan angles
	rc_quaternion_to_tb_array(state_estimate.quat_mag, state_estimate.tb_mag);

	// heading
	state_estimate.mag_heading_raw = mpu_data.compass_heading_raw;
	state_estimate.mag_heading = state_estimate.tb_mag[2];

	// yaw is more annoying since we have to detect spins
	// also make sign negative since NED coordinates has Z point down
	double diff = state_estimate.tb_mag[2] + (num_yaw_spins * TWO_PI) - last_yaw;
	//detect the crossover point at +-PI and update num yaw spins
	if(diff < -M_PI) num_yaw_spins++;
	else if(diff > M_PI) num_yaw_spins--;

	// finally the new value can be written
	state_estimate.mag_heading_continuous = state_estimate.tb_mag[2] + (num_yaw_spins * TWO_PI);
	last_yaw = state_estimate.mag_heading_continuous;
	return;
}


/**
 * @brief      initialize the altitude kalman filter
 *
 * @return     0 on success, -1 on failure
 */
static int __altitude_init(void)
{   
	// initialize Lidar
	status = VL53L1X_InitDriver(&Device, i2cbus, addr);
	if(status!=0){
		printf("ERROR: VL53LX Not Responding\n");
		return -1;
	}
	rc_usleep(1E4);
	printf("Initializing Lidar...\n");
	VL53L1X_SensorInit(&Device);
	rc_usleep(1E4);
<<<<<<< HEAD
=======
	//VL53L1X_SetDistanceMode(&Device,2);
	//VL53L1X_SetInterMeasurementInMs(&Device,200);
	//VL53L1X_SetTimingBudgetInMs(&Device,100);

    //initialize px4flow
	printf("Initializing Optic Flow Sensor...\n");
    PX4Flow_Initialize(&px4flow);
	rc_usleep(1E4);
>>>>>>> 10ac3fbd078868976fcbbd45d462eac1f3cc19c3

	VL53L1X_GetDistanceMode(&Device,&rtn);
	printf("Distance Mode: %d\n", rtn);

	VL53L1X_GetInterMeasurementInMs(&Device,&rtn);
	printf("Measurement Period: %dms\n", rtn);
	uint16_t rate = rtn;

	VL53L1X_GetTimingBudgetInMs(&Device,&rtn);
	printf("Timing Budget: %dms\n", rtn);

	VL53L1X_StartRanging(&Device);

<<<<<<< HEAD
	//initialize px4flow
	printf("Initializing Optic Flow Sensor...\n");
    PX4Flow_Initialize(&px4flow);
	rc_usleep(1E4);

	//initialize altitude kalman filter and bmp sensor
   	rc_matrix_t F = RC_MATRIX_INITIALIZER;
    	rc_matrix_t G = RC_MATRIX_INITIALIZER;
    	rc_matrix_t H = RC_MATRIX_INITIALIZER;
    	rc_matrix_t Q = RC_MATRIX_INITIALIZER;
    	rc_matrix_t R = RC_MATRIX_INITIALIZER;
    	rc_matrix_t Pi = RC_MATRIX_INITIALIZER;
=======
	//initialize altitude kalman filter and bmp sensor
   	rc_matrix_t F = RC_MATRIX_INITIALIZER;
    rc_matrix_t G = RC_MATRIX_INITIALIZER;
    rc_matrix_t H = RC_MATRIX_INITIALIZER;
    rc_matrix_t Q = RC_MATRIX_INITIALIZER;
    rc_matrix_t R = RC_MATRIX_INITIALIZER;
    rc_matrix_t Pi = RC_MATRIX_INITIALIZER;
>>>>>>> 10ac3fbd078868976fcbbd45d462eac1f3cc19c3

	const int Nx = 3;
	const int Ny = 1;
	const int Nu = 1;

	// allocate appropirate memory for system
	rc_matrix_zeros(&F, Nx, Nx);
	rc_matrix_zeros(&G, Nx, Nu);
	rc_matrix_zeros(&H, Ny, Nx);
	rc_matrix_zeros(&Q, Nx, Nx);
	rc_matrix_zeros(&R, Ny, Ny);
	rc_matrix_zeros(&Pi, Nx, Nx);

	// define system -DT; // accel bias
	F.d[0][0] = 1.0;
	F.d[0][1] = DT;
	F.d[0][2] = 0.0;
	F.d[1][0] = 0.0;
	F.d[1][1] = 1.0;
	F.d[1][2] = -DT; // subtract accel bias
	F.d[2][0] = 0.0;
	F.d[2][1] = 0.0;
	F.d[2][2] = 1.0; // accel bias state

	G.d[0][0] = 0.5*DT*DT;
	G.d[0][1] = DT;
	G.d[0][2] = 0.0;

	H.d[0][0] = 1.0;
	H.d[0][1] = 0.0;
	H.d[0][2] = 0.0;

	// covariance matrices
	Q.d[0][0] = 0.0001;
	Q.d[1][1] = 0.0001;
	Q.d[2][2] = 0.0001; // don't want bias to change too quickly
	R.d[0][0] = 0.01;

	// initial P, cloned from converged P while running
	Pi.d[0][0] = 1258.69;
	Pi.d[0][1] = 158.6114;
	Pi.d[0][2] = -9.9937;
	Pi.d[1][0] = 158.6114;
	Pi.d[1][1] = 29.9870;
	Pi.d[1][2] = -2.5191;
	Pi.d[2][0] = -9.9937;
	Pi.d[2][1] = -2.5191;
	Pi.d[2][2] = 0.3174;

	// initialize the kalman filter
	if(rc_kalman_alloc_lin(&alt_kf,F,G,H,Q,R,Pi)==-1) return -1;
	rc_matrix_free(&F);
	rc_matrix_free(&G);
	rc_matrix_free(&H);
	rc_matrix_free(&Q);
	rc_matrix_free(&R);
	rc_matrix_free(&Pi);

	// initialize the little LP filter to take out accel noise
	if(rc_filter_first_order_lowpass(&acc_lp, DT, 20*DT)) return -1;
	// initialize the butterworth LP filter to take out PX4 noise
	if(rc_filter_butterworth_lowpass(&PX4_lp, ORDER, DT, CUTOFF_FREQ)) return -1;

	// init barometer and read in first data
	if(rc_bmp_read(&bmp_data)) return -1;

	return 0;
}

static void __altitude_march(void)
{
	int i;
	double accel_vec[3];
	static rc_vector_t u = RC_VECTOR_INITIALIZER;
	static rc_vector_t y = RC_VECTOR_INITIALIZER;
	uint16_t distance = 0;
	uint8_t tmp = 0;

	// grab data from Lidar
	state_estimate.bmp_pressure_raw = bmp_data.pressure_pa;
	VL53L1X_GetDistance(&Device, &distance);
	state_estimate.alt_bmp_raw = distance*cos(state_estimate.roll)*cos(state_estimate.pitch)/1000.0;
<<<<<<< HEAD
	state_estimate.bmp_temp = bmp_data.temp_c;
	

	//grab data from PX4flow
	PX4Flow_ReadIntFrame(&px4flow, &iframe);
=======
	//state_estimate.alt_bmp_raw = bmp_data.alt_m;
	state_estimate.bmp_temp = bmp_data.temp_c;
	//printf("\n Lidar Distance: %f m  ", distance/1000.0);


	PX4Flow_ReadIntFrame(&px4flow, &iframe);
	//PX4Flow_ReadIntFrame(&px4flow, &iframe);
    //printf(" timestamp: %3d \r",frame.sonar_timestamp);
	//fflush(stdout);
    //rc_usleep(1E6*DT);

>>>>>>> 10ac3fbd078868976fcbbd45d462eac1f3cc19c3
	state_estimate.PX4_pix_x_int = iframe.pixel_flow_x_integral;
	state_estimate.PX4_pix_y_int = iframe.pixel_flow_y_integral;
	state_estimate.PX4_gyro_x_int = iframe.gyro_x_rate_integral;
	state_estimate.PX4_gyro_y_int = iframe.gyro_y_rate_integral;
	state_estimate.PX4_gyro_z_int = iframe.gyro_z_rate_integral;
	state_estimate.PX4_ground_distance_int = iframe.ground_distance;
	state_estimate.PX4_dt_int = iframe.integration_timespan;
	state_estimate.PX4_quality = iframe.quality;
	
<<<<<<< HEAD
	




=======
	//printf("\n %3d | %3d \r", frame.qual, state_estimate.PX4_qual);
	PX4_velocity_calculation(&state_estimate);

	
	//printf("\n Pix_x: %3d | Pix_y: %3d | dt: %3d | wx: %3d | wy: %3d | wz: %3d | Z: %3d | Tx: %3d | Ty: %3d \r",state_estimate.PX4_pix_x,state_estimate.PX4_pix_y,frame.sonar_timestamp,frame.gyro_x_rate,frame.gyro_y_rate,frame.gyro_z_rate,state_estimate.alt_bmp_raw,state_estimate.PX4_Tx,state_estimate.PX4_Ty);
	//printf("\n Lidar distance: %3f | sonar_distance: %3f \r",state_estimate.alt_bmp_raw,iframe.ground_distance);
	//printf("\n Lidar_distance: %3f | sonar_distance: %3f | Tx: %3f | Ty: %3f | qual: %3d \r",state_estimate.alt_bmp_raw,iframe.ground_distance,state_estimate.PX4_Tx,state_estimate.PX4_Ty,iframe.quality);
	
>>>>>>> 10ac3fbd078868976fcbbd45d462eac1f3cc19c3
	// make copy of acceleration reading before rotating
	for(i=0;i<3;i++) accel_vec[i] = state_estimate.accel[i];

	// rotate accel vector
	rc_quaternion_rotate_vector_array(accel_vec, state_estimate.quat_imu);

	// do first-run filter setup
	if(alt_kf.step==0){
        	rc_vector_zeros(&u, 1);
        	rc_vector_zeros(&y, 1);  //changed 1 to 2 for combining lidar output
		alt_kf.x_est.d[0] = -distance*cos(state_estimate.roll)*cos(state_estimate.pitch)/1000.0;
		rc_filter_prefill_inputs(&acc_lp, accel_vec[2]+GRAVITY);
		rc_filter_prefill_outputs(&acc_lp, accel_vec[2]+GRAVITY);
		PX4_filter_prefill(&state_estimate);
	}

	//PX4flow butterworth filter march
	PX4_filter_march(&state_estimate);
	PX4_velocity_calculation(&state_estimate);

	// calculate acceleration and smooth it just a tad
	// put result in u for kalman and flip sign since with altitude, positive
	// is up whereas acceleration in Z points down.
	rc_filter_march(&acc_lp, accel_vec[2]+GRAVITY);
	u.d[0] = acc_lp.newest_output;

	// don't bother filtering Barometer, kalman will deal with that
	y.d[0] = -distance*cos(state_estimate.roll)*cos(state_estimate.pitch)/1000.0;

	rc_kalman_update_lin(&alt_kf, u, y);

	// altitude estimate
	state_estimate.alt_bmp		= alt_kf.x_est.d[0];
	state_estimate.alt_bmp_vel	= alt_kf.x_est.d[1];
	state_estimate.alt_bmp_accel	= alt_kf.x_est.d[2];

	return;
}

static void __feedback_select(void)
{
	state_estimate.roll = state_estimate.tb_imu[0];
	state_estimate.pitch = state_estimate.tb_imu[1];
	state_estimate.yaw = state_estimate.tb_imu[2];
	state_estimate.continuous_yaw = state_estimate.imu_continuous_yaw;
	state_estimate.X = xbeeMsg.x;
	state_estimate.Y = xbeeMsg.y;
	state_estimate.Z = state_estimate.alt_bmp;
}

static void __altitude_cleanup(void)
{
	rc_kalman_free(&alt_kf);
	rc_filter_free(&acc_lp);
	return;
}

static void __mocap_check_timeout(void)
{
	if(state_estimate.mocap_running){
		uint64_t current_time = rc_nanos_since_boot();
		// check if mocap data is > 3 steps old
		if((current_time-state_estimate.mocap_timestamp_ns) > (3*1E7)){
			state_estimate.mocap_running = 0;
			if(settings.warnings_en){
				fprintf(stderr,"WARNING, MOCAP LOST VISUAL\n");
			}
		}
	}
	return;
}


int state_estimator_init(void)
{
	__batt_init();
	if(__altitude_init()) return -1;
	state_estimate.initialized = 1;
	return 0;
}

int state_estimator_march(void)
{
	if(state_estimate.initialized==0){
		fprintf(stderr, "ERROR in state_estimator_march, estimator not initialized\n");
		return -1;
	}

	// populate state_estimate struct one setion at a time, top to bottom
	__batt_march();
	__imu_march();
	__mag_march();
	__feedback_select();
	__altitude_march();
	//__feedback_select();	
	__mocap_check_timeout();
	return 0;
}


int state_estimator_jobs_after_feedback(void)
{
	static int bmp_sample_counter = 0;

	// check if we need to sample BMP this loop
	if(bmp_sample_counter>=BMP_RATE_DIV){
		// perform the i2c reads to the sensor, on bad read just try later
		if(rc_bmp_read(&bmp_data)) return -1;
		bmp_sample_counter=0;
	}
	bmp_sample_counter++;
	return 0;
}


int state_estimator_cleanup(void)
{
	__batt_cleanup();
	__altitude_cleanup();
	return 0;
}

<<<<<<< HEAD
=======
/*
void PX4_velocity_calculation(state_estimate_t *state_estimator) {
	double Tz = 0;
	double x =  (state_estimate.PX4_pix_x)/10.0*24*10E-6;   //m
	double y =  (state_estimate.PX4_pix_y)/10.0*24*10E-6;   //m
	double wx =  state_estimate.PX4_gyro_x;                 //rad/s
	double wy =  state_estimate.PX4_gyro_y;                 //rad/s
	double wz =  state_estimate.PX4_gyro_z;                 //rad/s
	double Z =  -state_estimate.alt_bmp_raw;                 //m
	double dt = state_estimate.PX4_dt*10E-3;				 //sec
	double f = 16/1000.0;                                    //mag


	double Tx,Ty;

	state_estimate.PX4_Tx = -(x/dt + wy*f - wz*y)*Z/f;     // m/s TODO:check sign 
	state_estimate.PX4_Ty = -(y/dt - wx*f + wz*x)*Z/f;     // m/s TODO:chekc sign



}
*/
>>>>>>> 10ac3fbd078868976fcbbd45d462eac1f3cc19c3

void PX4_velocity_calculation(state_estimate_t *state_estimate) {
	
	double velocity_x,velocity_y; 
	double x_rate = state_estimate->PX4_gyro_x_int / 10.0;       // mrad
    double y_rate = state_estimate->PX4_gyro_y_int / 10.0;       // mrad
    double flow_x = state_estimate->PX4_pix_x_int / 10.0;      // mrad
    double flow_y = state_estimate->PX4_pix_y_int/ 10.0;      // mrad  
    
	int timespan = state_estimate->PX4_dt_int;             // microseconds
    int ground_distance = state_estimate->PX4_ground_distance_int;       // mm
    uint8_t quality = state_estimate->PX4_quality;
		
		
	 if (quality > 100) {
      // Update flow rate with gyro rate
      double pixel_x = flow_x + x_rate; // mrad
      double pixel_y = flow_y + y_rate; // mrad
      
      // Scale based on ground distance and compute speed
      // (flow/1000) * (ground_distance/1000) / (timespan/1000000)
    
      
      

	  velocity_x = pixel_x * ground_distance / timespan;  // m/s
	  velocity_y = pixel_y * ground_distance / timespan;  // m/s
      // Integrate velocity to get pose estimate
      px = px + velocity_x * 100;
      py = py + velocity_y * 100;


	 state_estimate->PX4_Tx = velocity_x;
	 state_estimate->PX4_Ty = velocity_y;
<<<<<<< HEAD
	 state_estimate->PX4_X = px;
	 state_estimate->PX4_Y = py;
	 
	 
=======
	 
	 
	/*
	 printf("\n Lidar_distance: %3f | sonar_distance: %3f | Vx: %5f | Vy: %5f | qual: %3d | dt: %3d \r",
	 			state_estimate->alt_bmp_raw,\
				state_estimate->PX4_ground_distance_int,\
				state_estimate->PX4_pix_x_int,\
				state_estimate->PX4_pix_y_int,\
				state_estimate->PX4_quality,\
				state_estimate->PX4_dt_int);

	 */
	 
>>>>>>> 10ac3fbd078868976fcbbd45d462eac1f3cc19c3
	 }

	
}
<<<<<<< HEAD

void PX4_filter_prefill(state_estimate_t *state_estimate){

	rc_filter_prefill_inputs(&PX4_lp, state_estimate->PX4_gyro_x_int);
    rc_filter_prefill_outputs(&PX4_lp, state_estimate->PX4_gyro_x_int);

	rc_filter_prefill_inputs(&PX4_lp, state_estimate->PX4_gyro_y_int);
    rc_filter_prefill_outputs(&PX4_lp, state_estimate->PX4_gyro_y_int);

	rc_filter_prefill_inputs(&PX4_lp, state_estimate->PX4_pix_x_int);
    rc_filter_prefill_outputs(&PX4_lp, state_estimate->PX4_pix_x_int);

	rc_filter_prefill_inputs(&PX4_lp, state_estimate->PX4_pix_y_int);
    rc_filter_prefill_outputs(&PX4_lp, state_estimate->PX4_pix_y_int);

	rc_filter_prefill_inputs(&PX4_lp, state_estimate->PX4_dt_int);
    rc_filter_prefill_outputs(&PX4_lp, state_estimate->PX4_dt_int);

	rc_filter_prefill_inputs(&PX4_lp, state_estimate->PX4_ground_distance_int);
    rc_filter_prefill_outputs(&PX4_lp, state_estimate->PX4_ground_distance_int);


}


void PX4_filter_march(state_estimate_t *state_estimate){

	state_estimate->PX4_gyro_x_int = rc_filter_march(&PX4_lp,state_estimate->PX4_gyro_x_int);
	state_estimate->PX4_gyro_y_int = rc_filter_march(&PX4_lp,state_estimate->PX4_gyro_y_int);
	state_estimate->PX4_pix_x_int = rc_filter_march(&PX4_lp,state_estimate->PX4_pix_x_int);
	state_estimate->PX4_pix_y_int = rc_filter_march(&PX4_lp,state_estimate->PX4_pix_y_int);
	state_estimate->PX4_dt_int = rc_filter_march(&PX4_lp,state_estimate->PX4_dt_int);
	state_estimate->PX4_ground_distance_int = rc_filter_march(&PX4_lp,state_estimate->PX4_ground_distance_int);

}
=======
>>>>>>> 10ac3fbd078868976fcbbd45d462eac1f3cc19c3
