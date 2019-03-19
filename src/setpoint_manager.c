/**
* @file setpoint_manager.c
*
*
**/
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h> // for memset

#include <robotcontrol.h>
#include <rc/start_stop.h>
#include <rc/time.h>

#include <setpoint_manager.h>
#include <settings.h>
#include <input_manager.h>
#include <feedback.h>
#include <state_estimator.h>
#include <rc_pilot_defs.h>
#include <flight_mode.h>
#include <xbee_packet_t.h>

#define XYZ_MAX_ERROR	0.1 ///< meters.

setpoint_t setpoint; // extern variable in setpoint_manager.h

<<<<<<< HEAD
static uint64_t start_index;

double X0,Y0,Z0;
int alt_hold;
int k = 0;

double pt[6][3];
double limit = 0.1;
=======
static uint64_t start_time;
>>>>>>> ad3cf265d1ec65fa8bc0428646a415059218c915

void __update_yaw(void)
{
	// if throttle stick is down all the way, probably landed, so
	// keep the yaw setpoint at current yaw so it takes off straight
	if(user_input.thr_stick < -0.95){
		setpoint.yaw = state_estimate.yaw;
		setpoint.yaw_dot = 0.0;
		return;
	}
	// otherwise, scale yaw_rate by max yaw rate in rad/s
	// and move yaw setpoint
	setpoint.yaw_dot = user_input.yaw_stick * MAX_YAW_RATE;
	setpoint.yaw += setpoint.yaw_dot*DT;
	return;
}

void __update_Z(void)
{
	// make sure setpoint doesn't go too far below current altitude since we
	// can't sink into the ground
	if(setpoint.Z > (state_estimate.Z + XYZ_MAX_ERROR)){
		setpoint.Z = state_estimate.Z + XYZ_MAX_ERROR;
		setpoint.Z_dot = 0.0;
		return;
	}
	setpoint.Z_dot = -user_input.thr_stick*settings.max_Z_velocity;
	setpoint.Z += setpoint.Z_dot*DT;
	return;
}

void __update_Z_alt_hold(void)
{
	setpoint.Z_dot = -user_input.thr_stick*settings.max_Z_velocity;
	if((-user_input.thr_stick < 0.01) && (-user_input.thr_stick > -0.01)){
		setpoint.Z_dot = 0.0;
	}
	setpoint.Z += setpoint.Z_dot*DT;
	return;
}

void __update_XY_pos(void)
{
	setpoint.X_dot = -user_input.pitch_stick*settings.max_XY_velocity;
	setpoint.Y_dot =  user_input.roll_stick*settings.max_XY_velocity;

	if((-user_input.pitch_stick<0.01) && (-user_input.pitch_stick > -0.01)){
		setpoint.X_dot = 0.0;
	}

	if((user_input.roll_stick < 0.01) && (user_input.roll_stick > -0.01)){
		setpoint.Y_dot = 0.0;
	}
<<<<<<< HEAD

	setpoint.X += setpoint.X_dot*DT;
<<<<<<< HEAD
=======
=======
<<<<<<< HEAD
	// make sure setpoint doesn't go too far from state in case touching something
=======
>>>>>>> 10ac3fbd078868976fcbbd45d462eac1f3cc19c3
	//if(setpoint.X > (state_estimate.X + XYZ_MAX_ERROR)){
	//	setpoint.X = state_estimate.X + XYZ_MAX_ERROR;
	//	setpoint.X_dot = 0.0;
	//}
	//else if(setpoint.X < (state_estimate.X - XYZ_MAX_ERROR)){
	//	setpoint.X = state_estimate.X - XYZ_MAX_ERROR;
	//	setpoint.X_dot = 0.0;
	//	return;
	//}
	//else{
	//	setpoint.X += setpoint.X_dot*DT;
	//}

	//if(setpoint.Y > (state_estimate.Y + XYZ_MAX_ERROR)){
	//	setpoint.Y = state_estimate.Y + XYZ_MAX_ERROR;
	//	setpoint.Y_dot = 0.0;
	//	return;
	//}
	//else if(setpoint.Y < (state_estimate.Y - XYZ_MAX_ERROR)){
	//	setpoint.Y = state_estimate.Y - XYZ_MAX_ERROR;
	//	setpoint.Y_dot = 0.0;
	//	return;
	//}
	//else{
	//	setpoint.Y += setpoint.Y_dot*DT;
	//}
	setpoint.X += setpoint.X_dot*DT;
<<<<<<< HEAD
>>>>>>> 5f08dc3a05cac60167486d13896ca274dfdd273a
>>>>>>> ad3cf265d1ec65fa8bc0428646a415059218c915
	setpoint.Y += setpoint.Y_dot*DT;
	return;
}

=======
>>>>>>> 10ac3fbd078868976fcbbd45d462eac1f3cc19c3

int setpoint_manager_init(void)
{
	if(setpoint.initialized){
		fprintf(stderr, "ERROR in setpoint_manager_init, already initialized\n");
		return -1;
	}
	memset(&setpoint,0,sizeof(setpoint_t));
	setpoint.initialized = 1;
<<<<<<< HEAD

=======
	start_time = rc_nanos_since_boot();
>>>>>>> ad3cf265d1ec65fa8bc0428646a415059218c915
	return 0;
}

void __update_AutoZ(void) {

		if (setpoint.count == 0) {
			setpoint.Z_dot = 0.13*settings.max_Z_velocity;
			alt_hold = 0;
		}
		else if (setpoint.count == 5){
			setpoint.Z_dot = -0.13*settings.max_Z_velocity;
			alt_hold = 0;
		}
		else {
			setpoint.Z_dot = 0.0;
			alt_hold = 1;
		}
		setpoint.Z = setpoint.Z - setpoint.Z_dot*DT;
		rc_saturate_double(&setpoint.Z,-0.65,1.0);

}

void __update_AutoY(void) {
		
	if (alt_hold == 1){
		if (setpoint.count == 1 || setpoint.count == 3){								//setpoint.count == 1 || setpoint.count == 3
			setpoint.Y_dot = 0.6*settings.max_XY_velocity;
		} 
		else if (setpoint.count == 2 || setpoint.count == 4){							//setpoint.count == 2 || setpoint.count == 4
			setpoint.Y_dot = -0.6*settings.max_XY_velocity;
		}
		else{
			setpoint.Y_dot = 0.0;
		}
		setpoint.Y = setpoint.Y + setpoint.Y_dot*DT;
		rc_saturate_double(&setpoint.Y,Y0,Y0+0.8);
	}

}


void __update_AutoX(void) {
	if (alt_hold == 1){	

		if (setpoint.count == 1){								//setpoint.count == 1
			setpoint.X_dot = 0.6*settings.max_XY_velocity;
		} 
		else if (setpoint.count == 3){							//setpoint.count == 3
			setpoint.X_dot = -0.6*settings.max_XY_velocity;
		}
		else{
			setpoint.X_dot = 0.0;
		}
		setpoint.X = setpoint.X + setpoint.X_dot*DT;
		rc_saturate_double(&setpoint.X,X0,X0+0.8);
	}
}

void __update_Pt(void) {
	if ((fstate.loop_index) <= 6000 && (fstate.loop_index) > 1500){
		setpoint.count = 0;
	}
	else if ((fstate.loop_index) > 6000 && ((fstate.loop_index) <= 10500)){
		setpoint.count = 1;
	}
	else if ((fstate.loop_index) > 10500 && ((fstate.loop_index) <= 15000)){
		setpoint.count = 2;
	}
	else if ((fstate.loop_index) > 15000 && ((fstate.loop_index) <= 19500)){
		setpoint.count = 3;
	}
	else if ((fstate.loop_index) > 19500 && ((fstate.loop_index) <= 24000)){
		setpoint.count = 4;
	}
	else if ((fstate.loop_index) > 24000 && ((fstate.loop_index) <= 28500)){
		setpoint.count = 5;
	}
	else{
		setpoint.count = -1;
		setpoint.X = X0;
		setpoint.Y = Y0;
		setpoint.Z = Z0;
	}
}


int setpoint_manager_update(void)
{
	double tmp_Z_throttle;
<<<<<<< HEAD
	static int autostart = 1;
=======
<<<<<<< HEAD
	static uint64_t curr_time;
	curr_time = rc_nanos_since_boot() - start_time;
=======
>>>>>>> 5f08dc3a05cac60167486d13896ca274dfdd273a
>>>>>>> ad3cf265d1ec65fa8bc0428646a415059218c915

	if(setpoint.initialized==0){
		fprintf(stderr, "ERROR in setpoint_manager_update, not initialized yet\n");
		return -1;
	}

	if(user_input.initialized==0){
		fprintf(stderr, "ERROR in setpoint_manager_update, input_manager not initialized yet\n");
		return -1;
	}

	// if PAUSED or UNINITIALIZED, do nothing
	if(rc_get_state()!=RUNNING) return 0;

	// shutdown feedback on kill switch
	if(user_input.requested_arm_mode == DISARMED){
		if(fstate.arm_state!=DISARMED) feedback_disarm();
		return 0;
	}

	// finally, switch between flight modes and adjust setpoint properly
	switch(user_input.flight_mode){


	case TEST_BENCH_4DOF:
		// configure which controllers are enabled
		setpoint.en_6dof	= 0;
		setpoint.en_rpy_ctrl	= 0;
		setpoint.en_Z_ctrl	= 0;
		setpoint.en_XY_vel_ctrl	= 0;
		setpoint.en_XY_pos_ctrl	= 0;

		setpoint.roll_throttle	=  user_input.roll_stick;
		setpoint.pitch_throttle	=  user_input.pitch_stick;
		setpoint.yaw_throttle	=  user_input.yaw_stick;
		setpoint.Z_throttle	= -user_input.thr_stick;
		// TODO add these two throttle modes as options to settings, I use a radio
		// with self-centering throttle so having 0 in the middle is safest
		// setpoint.Z_throttle = -(user_input.thr_stick+1.0)/2.0;
		break;

	case TEST_BENCH_6DOF:
		setpoint.en_6dof	= 1;
		setpoint.en_rpy_ctrl	= 0;
		setpoint.en_Z_ctrl	= 0;
		setpoint.en_XY_vel_ctrl	= 0;
		setpoint.en_XY_pos_ctrl	= 0;

		setpoint.X_throttle	= -user_input.pitch_stick;
		setpoint.Y_throttle	=  user_input.roll_stick;
		setpoint.roll_throttle	=  0.0;
		setpoint.pitch_throttle	=  0.0;
		setpoint.yaw_throttle	=  user_input.yaw_stick;
		setpoint.Z_throttle	= -user_input.thr_stick;
		break;

	case DIRECT_THROTTLE_4DOF:
		setpoint.en_6dof	= 0;
		setpoint.en_rpy_ctrl	= 1;
		setpoint.en_Z_ctrl	= 0;
		setpoint.en_XY_vel_ctrl	= 0;
		setpoint.en_XY_pos_ctrl	= 0;

		setpoint.roll		=  user_input.roll_stick;
		setpoint.pitch		=  user_input.pitch_stick;
		setpoint.Z_throttle	= -user_input.thr_stick;
		__update_yaw();
		break;

	case DIRECT_THROTTLE_6DOF:
		setpoint.en_6dof	= 1;
		setpoint.en_rpy_ctrl	= 1;
		setpoint.en_Z_ctrl	= 0;
		setpoint.en_XY_vel_ctrl	= 0;
		setpoint.en_XY_pos_ctrl	= 0;

		setpoint.X_throttle	= -user_input.pitch_stick;
		setpoint.Y_throttle	=  user_input.roll_stick;
		setpoint.Z_throttle	= -user_input.thr_stick;
		__update_yaw();
		break;

	case ALT_HOLD_4DOF:
		setpoint.en_6dof		= 0;
		setpoint.en_rpy_ctrl		= 1;
		setpoint.en_Z_ctrl		= 1;
		setpoint.en_XY_vel_ctrl		= 0;
		setpoint.en_XY_pos_ctrl		= 0;
		//tmp_Z_throttle 		= setpoint.Z_throttle;
		//setpoint.Z_throttle		= -user_input.thr_stick;

		setpoint.roll			= user_input.roll_stick;
		setpoint.pitch			= user_input.pitch_stick;

		__update_Z_alt_hold();
		__update_yaw();
		break;

	case ALT_HOLD_6DOF:
		setpoint.en_6dof	= 1;
		setpoint.en_rpy_ctrl	= 1;
		setpoint.en_Z_ctrl	= 1;
		setpoint.en_XY_vel_ctrl	= 0;
		setpoint.en_XY_pos_ctrl	= 0;

		setpoint.roll		= 0.0;
		setpoint.pitch		= 0.0;
		setpoint.X_throttle	= -user_input.pitch_stick;
		setpoint.Y_throttle	=  user_input.roll_stick;
		__update_Z();
		__update_yaw();
		break;

	case VELOCITY_CONTROL_4DOF:
		setpoint.en_6dof	= 0;
		setpoint.en_rpy_ctrl	= 1;
		setpoint.en_Z_ctrl	= 1;
		setpoint.en_XY_vel_ctrl	= 1;
		setpoint.en_XY_pos_ctrl	= 0;

		setpoint.X_dot = -user_input.pitch_stick * settings.max_XY_velocity;
		setpoint.Y_dot =  user_input.roll_stick  * settings.max_XY_velocity;
		__update_Z();
		__update_yaw();
		break;

	case VELOCITY_CONTROL_6DOF:
		setpoint.en_6dof	= 1;
		setpoint.en_rpy_ctrl	= 1;
		setpoint.en_Z_ctrl	= 1;
		setpoint.en_XY_vel_ctrl	= 1;
		setpoint.en_XY_pos_ctrl	= 0;

		setpoint.X_dot = -user_input.pitch_stick * settings.max_XY_velocity;
		setpoint.Y_dot =  user_input.roll_stick  * settings.max_XY_velocity;
		__update_Z();
		__update_yaw();
		break;

	case POSITION_CONTROL_4DOF:
		setpoint.en_6dof	= 0;
		setpoint.en_rpy_ctrl	= 1;
		setpoint.en_Z_ctrl	= 1;
		setpoint.en_XY_vel_ctrl	= 0;
		setpoint.en_XY_pos_ctrl	= 1;
		//setpoint.yaw = 0.0;

<<<<<<< HEAD
=======
<<<<<<< HEAD
>>>>>>> ad3cf265d1ec65fa8bc0428646a415059218c915
		//setpoint.X_dot = -user_input.pitch_stick * settings.max_XY_velocity;
		//setpoint.Y_dot =  user_input.roll_stick  * settings.max_XY_velocity;
		//setpoint.X = 0.5;
		//setpoint.Y = 0.5;
<<<<<<< HEAD
		__update_XY_pos();
		__update_Z_alt_hold();
		__update_yaw();
		break;

	case AUTO_4DOF:
		if (autostart == 1){

			start_index = fstate.loop_index;
			X0 = xbeeMsg.x;
			Y0 = xbeeMsg.y;
			Z0 = state_estimate.alt_bmp;

			setpoint.X = xbeeMsg.x;
			setpoint.Y = xbeeMsg.y;
			setpoint.Z = state_estimate.alt_bmp;

			setpoint.count = 0;
			autostart = 0;
			alt_hold = 0;

		}
		setpoint.en_6dof	= 0;
		setpoint.en_rpy_ctrl = 1;
		setpoint.en_Z_ctrl	= 1;
		setpoint.en_XY_vel_ctrl	= 0;
		setpoint.en_XY_pos_ctrl	= 1;

		__update_Pt();
		__update_AutoZ();
		__update_AutoX();
		__update_AutoY();
=======
=======
		setpoint.X_dot = -user_input.pitch_stick * settings.max_XY_velocity;
		setpoint.Y_dot =  user_input.roll_stick  * settings.max_XY_velocity;
<<<<<<< HEAD
		setpoint.X = 0.5;
		setpoint.Y = 0.5;
		//__update_XY_pos();
		__update_Z_alt_hold();
		setpoint.yaw = 0.0;
		//__update_yaw();
=======
>>>>>>> 5f08dc3a05cac60167486d13896ca274dfdd273a
		__update_XY_pos();
		__update_Z_alt_hold();
>>>>>>> ad3cf265d1ec65fa8bc0428646a415059218c915
		__update_yaw();
>>>>>>> 10ac3fbd078868976fcbbd45d462eac1f3cc19c3
		break;

	case POSITION_CONTROL_6DOF:
		setpoint.en_6dof	= 1;
		setpoint.en_rpy_ctrl	= 1;
		setpoint.en_Z_ctrl	= 1;
		setpoint.en_XY_vel_ctrl	= 0;
		setpoint.en_XY_pos_ctrl	= 1;
		__update_XY_pos();
		__update_Z();
		__update_yaw();
		break;

	default: // should never get here
		fprintf(stderr,"ERROR in setpoint_manager thread, unknown flight mode\n");
		break;

	} // end switch(user_input.flight_mode)

	// arm feedback when requested
	if(user_input.requested_arm_mode == MID_ARMING){
		if(fstate.arm_state==DISARMED) feedback_mid_arm();
	}
	if(user_input.requested_arm_mode == ARMED){
		if(fstate.arm_state==MID_ARMING) feedback_arm();
	}


	return 0;
}


int setpoint_manager_cleanup(void)
{
	setpoint.initialized=0;
	return 0;
}