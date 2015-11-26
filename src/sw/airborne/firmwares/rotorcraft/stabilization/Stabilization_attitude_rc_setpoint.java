package sw.airborne.firmwares.rotorcraft.stabilization;

import static sw.airborne.math.Pprz_trig_int.*;
import static sw.airborne.math.Pprz_algebra.*;
import static sw.airborne.firmwares.rotorcraft.Stabilization.*;
import static sw.include.Std.*;

import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_rc_setpoint.*;
import static sw.airborne.math.Pprz_algebra_int.*;
import static sw.airborne.State.*;
import sw.airborne.math.*;
import  sw.airborne.math.Int32Quat;
import sw.airborne.math.Int32Vect2;
import sw.include.Std;
import static sw.airborne.math.Pprz_geodetic.*;
import static sw.airborne.math.Pprz_orientation_conversion.*;
import static sw.airborne.math.Pprz_geodetic_int.*;
import static sw.airborne.math.Pprz_geodetic_float.*;
import static sw.airborne.math.Pprz_algebra_float.*;


public class Stabilization_attitude_rc_setpoint {
	
//	public static boolean ROLL_DEADBAND_EXCEEDED(){
//		if(STABILIZATION_ATTITUDE_DEADBAND_A_DEFINED){
//			return (radio_control.values[RADIO_ROLL] >  STABILIZATION_ATTITUDE_DEADBAND_A 
//					|| radio_control.values[RADIO_ROLL] < -STABILIZATION_ATTITUDE_DEADBAND_A);
//		}else{
//			return true;
//		}
//	}
//	
//	public static boolean  PITCH_DEADBAND_EXCEEDED(){
//		if(STABILIZATION_ATTITUDE_DEADBAND_E_DEFINED){
//			return  (radio_control.values[RADIO_PITCH] >  STABILIZATION_ATTITUDE_DEADBAND_E || 
//					   radio_control.values[RADIO_PITCH] < -STABILIZATION_ATTITUDE_DEADBAND_E);
//		}else{
//			return true;
//		}
//	}
//	
//	public static boolean YAW_DEADBAND_EXCEEDED(){
//		return (radio_control.values[RADIO_YAW] >  STABILIZATION_ATTITUDE_DEADBAND_R || 
//				   radio_control.values[RADIO_YAW] < -STABILIZATION_ATTITUDE_DEADBAND_R);
//	}
	//--------------------------------------stabilization_attitude_rc_setpoint.c-----------------------------
	
	public static final int RC_UPDATE_FREQ = 40;
	
	public static float care_free_heading = 0;
	
	/// reset the heading for care-free mode to current heading
	public static void stabilization_attitude_reset_care_free_heading() {
	  care_free_heading = stateGetNedToBodyEulers_f().psi;
	}

	/*   This is a different way to obtain yaw. It will not switch when going beyond 90 degrees pitch.
	     However, when rolling more then 90 degrees in combination with pitch it switches. For a
	     transition vehicle this is better as 90 degrees pitch will occur, but more than 90 degrees roll probably not. */
	public static long stabilization_attitude_get_heading_i() {
	   Int32Eulers att = stateGetNedToBodyEulers_i();

	  long heading;

	  if(Math.abs(att.phi) < INT32_ANGLE_PI_2) {
	    long sin_theta = 0;
	    sin_theta = PPRZ_ITRIG_SIN( att.theta);
	    heading = att.psi - INT_MULT_RSHIFT(sin_theta, att.phi, INT32_TRIG_FRAC);
	  }
	  else if(ANGLE_FLOAT_OF_BFP(att.theta) > 0)
	    heading = att.psi - att.phi;
	  else
	    heading = att.psi + att.phi;

	  return heading;
	}

	public static float stabilization_attitude_get_heading_f() {
	   FloatEulers att = stateGetNedToBodyEulers_f();

	  float heading;

	  if(Math.abs(att.phi) < M_PI/2) {
	    heading = att.psi - (float) Math.sin(att.theta)*att.phi;
	  }
	  else if(att.theta > 0)
	    heading = att.psi - att.phi;
	  else
	    heading = att.psi + att.phi;

	  return heading;
	}


//	public static void stabilization_attitude_read_rc_setpoint_eulers( Int32Eulers sp, boolean in_flight, boolean in_carefree, boolean coordinated_turn) {
//	  int max_rc_phi = (int) ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_PHI);
//	  int max_rc_theta = (int) ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_THETA);
//	  int max_rc_r = (int) ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_R);
//
//	  sp.phi = (int) ((radio_control.values[RADIO_ROLL] * max_rc_phi) /  MAX_PPRZ);
//	  sp.theta = (int) ((radio_control.values[RADIO_PITCH] * max_rc_theta) /  MAX_PPRZ);
//
//	  if (in_flight) {
//	    if (YAW_DEADBAND_EXCEEDED()) {
//	      sp.psi += (int) ((radio_control.values[RADIO_YAW] * max_rc_r) /  MAX_PPRZ / RC_UPDATE_FREQ);
//	      INT32_ANGLE_NORMALIZE(sp.psi);
//	    }
//	    if (coordinated_turn) {
//	      //Coordinated turn
//	      //feedforward estimate angular rotation omega = g*tan(phi)/v
//	      //Take v = 9.81/1.3 m/s
//	      int omega;
//	      int max_phi = ANGLE_BFP_OF_REAL(RadOfDeg(85.0));
//	      if(abs(sp.phi) < max_phi)
//	        omega = ANGLE_BFP_OF_REAL(1.3*tanf(ANGLE_FLOAT_OF_BFP(sp.phi)));
//	      else //max 60 degrees roll, then take constant omega
//	        omega = ANGLE_BFP_OF_REAL(1.3*1.72305* ((sp.phi > 0) - (sp.phi < 0)));
//
//	      sp.psi += omega/RC_UPDATE_FREQ;
//	    }
//	    if(STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT){
//	    	// Make sure the yaw setpoint does not differ too much from the real yaw
//	    	// to prevent a sudden switch at 180 deg
//	    	const int delta_limit = ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT);
//
//	    	int heading = stabilization_attitude_get_heading_i();
//
//	    	int delta_psi = sp.psi - heading;
//	    	INT32_ANGLE_NORMALIZE(delta_psi);
//	    	if (delta_psi > delta_limit){
//	    		sp.psi = heading + delta_limit;
//	    	}
//	    	else if (delta_psi < -delta_limit){
//	    		sp.psi = heading - delta_limit;
//	    	}
//	    	INT32_ANGLE_NORMALIZE(sp.psi);
//
//	    }
//	    //Care Free mode
//	    if (in_carefree) {
//	      //care_free_heading has been set to current psi when entering care free mode.
//	      int cos_psi;
//	      int sin_psi;
//	      int temp_theta;
//	      int care_free_delta_psi_i;
//
//	      care_free_delta_psi_i = sp.psi - ANGLE_BFP_OF_REAL(care_free_heading);
//
//	      INT32_ANGLE_NORMALIZE(care_free_delta_psi_i);
//
//	      PPRZ_ITRIG_SIN(sin_psi, care_free_delta_psi_i);
//	      PPRZ_ITRIG_COS(cos_psi, care_free_delta_psi_i);
//
//	      temp_theta = INT_MULT_RSHIFT(cos_psi, sp.theta, INT32_ANGLE_FRAC) - INT_MULT_RSHIFT(sin_psi, sp.phi, INT32_ANGLE_FRAC);
//	      sp.phi = INT_MULT_RSHIFT(cos_psi, sp.phi, INT32_ANGLE_FRAC) - INT_MULT_RSHIFT(sin_psi, sp.theta, INT32_ANGLE_FRAC);
//
//	      sp.theta = temp_theta;
//	    }
//	  }
//	  else { /* if not flying, use current yaw as setpoint */
//	    sp.psi = stateGetNedToBodyEulers_i().psi;
//	  }
//	}


//	public static void stabilization_attitude_read_rc_setpoint_eulers_f( FloatEulers sp, boolean in_flight, boolean in_carefree, boolean coordinated_turn) {
//	  sp.phi = (radio_control.values[RADIO_ROLL]  * STABILIZATION_ATTITUDE_SP_MAX_PHI / MAX_PPRZ);
//	  sp.theta = (radio_control.values[RADIO_PITCH] * STABILIZATION_ATTITUDE_SP_MAX_THETA / MAX_PPRZ);
//
//	  if (in_flight) {
//	    if (YAW_DEADBAND_EXCEEDED()) {
//	      sp.psi += (radio_control.values[RADIO_YAW] * STABILIZATION_ATTITUDE_SP_MAX_R / MAX_PPRZ / RC_UPDATE_FREQ);
//	      FLOAT_ANGLE_NORMALIZE(sp.psi);
//	    }
//	    if (coordinated_turn) {
//	      //Coordinated turn
//	      //feedforward estimate angular rotation omega = g*tan(phi)/v
//	      //Take v = 9.81/1.3 m/s
//	      float omega;
//	      float max_phi = RadOfDeg(85.0);
//	      if(abs(sp.phi) < max_phi)
//	        omega = 1.3*tanf(sp.phi);
//	      else //max 60 degrees roll, then take constant omega
//	        omega = 1.3*1.72305* ((sp.phi > 0) - (sp.phi < 0));
//
//	      sp.psi += omega/RC_UPDATE_FREQ;
//	    }
//	    if(STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT){
//
//	    	// Make sure the yaw setpoint does not differ too much from the real yaw
//	    	// to prevent a sudden switch at 180 deg
//	    	float heading = stabilization_attitude_get_heading_f();
//
//	    	float delta_psi = sp.psi - heading;
//	    	FLOAT_ANGLE_NORMALIZE(delta_psi);
//	    	if (delta_psi > STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT){
//	    		sp.psi = heading + STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT;
//	    	}
//	    	else if (delta_psi < -STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT){
//	    		sp.psi = heading - STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT;
//	    	}
//	    	FLOAT_ANGLE_NORMALIZE(sp.psi);
//	    }
//	
//	    //Care Free mode
//	    if (in_carefree) {
//	      //care_free_heading has been set to current psi when entering care free mode.
//	      float cos_psi;
//	      float sin_psi;
//	      float temp_theta;
//
//	      float care_free_delta_psi_f = sp.psi - care_free_heading;
//
//	      FLOAT_ANGLE_NORMALIZE(care_free_delta_psi_f);
//
//	      sin_psi = sinf(care_free_delta_psi_f);
//	      cos_psi = cosf(care_free_delta_psi_f);
//
//	      temp_theta = cos_psi*sp.theta - sin_psi*sp.phi;
//	      sp.phi = cos_psi*sp.phi - sin_psi*sp.theta;
//
//	      sp.theta = temp_theta;
//	    }
//	  }
//	  else { /* if not flying, use current yaw as setpoint */
//	    sp.psi = stateGetNedToBodyEulers_f().psi;
//	  }
//	}

//
//	
//	public static void stabilization_attitude_read_rc_roll_pitch_quat_f( FloatQuat q) {
//	  /* orientation vector describing simultaneous rotation of roll/pitch */
//	   FloatVect3 ov;
//	  ov.x = radio_control.values[RADIO_ROLL] * STABILIZATION_ATTITUDE_SP_MAX_PHI / MAX_PPRZ;
//	  ov.y = radio_control.values[RADIO_PITCH] * STABILIZATION_ATTITUDE_SP_MAX_THETA / MAX_PPRZ;
//	  ov.z = 0.0;
//
//	  /* quaternion from that orientation vector */
//	  FLOAT_QUAT_OF_ORIENTATION_VECT(q, ov);
//	}

//	public static void stabilization_attitude_read_rc_roll_pitch_earth_quat_f( FloatQuat q) {
//		/* only non-zero entries for roll quaternion */
//		float roll2 = radio_control.values[RADIO_ROLL] * STABILIZATION_ATTITUDE_SP_MAX_PHI / MAX_PPRZ / 2;
//		float qx_roll = sinf(roll2);
//		float qi_roll = cosf(roll2);
//
//		//An offset is added if in forward mode
//		/* only non-zero entries for pitch quaternion */
//		float pitch2 = (ANGLE_FLOAT_OF_BFP(transition_theta_offset) + radio_control.values[RADIO_PITCH] * STABILIZATION_ATTITUDE_SP_MAX_THETA / MAX_PPRZ) / 2;
//		float qy_pitch = sinf(pitch2);
//		float qi_pitch = cosf(pitch2);
//
//		/* only multiply non-zero entries of FLOAT_QUAT_COMP(*q, q_roll, q_pitch) */
//		q.qi = qi_roll * qi_pitch;
//		q.qx = qx_roll * qi_pitch;
//		q.qy = qi_roll * qy_pitch;
//		q.qz = qx_roll * qy_pitch;
//	}


//	public static void stabilization_attitude_read_rc_setpoint_quat_f( FloatQuat q_sp, boolean in_flight, boolean in_carefree, boolean coordinated_turn) {
//
//		// FIXME: remove me, do in quaternion directly
//		// is currently still needed, since the yaw setpoint integration is done in eulers
//		if(STABILIZATION_ATTITUDE_TYPE_INT_DEFINED)
//			stabilization_attitude_read_rc_setpoint_eulers(stab_att_sp_euler, in_flight, in_carefree, coordinated_turn);
//		else
//			stabilization_attitude_read_rc_setpoint_eulers_f(stab_att_sp_euler, in_flight, in_carefree, coordinated_turn);
//
//
//		FloatQuat q_rp_cmd;
//		stabilization_attitude_read_rc_roll_pitch_quat_f(q_rp_cmd);
//
//		/* get current heading */
//		FloatVect3 zaxis = {0., 0., 1.};
//		FloatQuat q_yaw;
//
//		//Care Free mode
//		if (in_carefree) {
//			//care_free_heading has been set to current psi when entering care free mode.
//			FLOAT_QUAT_OF_AXIS_ANGLE(q_yaw, zaxis, care_free_heading);
//		}
//		else {
//			FLOAT_QUAT_OF_AXIS_ANGLE(q_yaw, zaxis, stateGetNedToBodyEulers_f().psi);
//		}
//
//		/* roll/pitch commands applied to to current heading */
//		FloatQuat q_rp_sp;
//		FLOAT_QUAT_COMP(q_rp_sp, q_yaw, q_rp_cmd);
//		FLOAT_QUAT_NORMALIZE(q_rp_sp);
//
//		if (in_flight)
//		{
//			/* get current heading setpoint */
//			FloatQuat q_yaw_sp;
//			if(STABILIZATION_ATTITUDE_TYPE_INT_DEFINED)
//				FLOAT_QUAT_OF_AXIS_ANGLE(q_yaw_sp, zaxis, ANGLE_FLOAT_OF_BFP(stab_att_sp_euler.psi));
//			else
//				FLOAT_QUAT_OF_AXIS_ANGLE(q_yaw_sp, zaxis, stab_att_sp_euler.psi);
//
//
//			/* rotation between current yaw and yaw setpoint */
//			FloatQuat q_yaw_diff;
//			FLOAT_QUAT_COMP_INV(q_yaw_diff, q_yaw_sp, q_yaw);
//
//			/* compute final setpoint with yaw */
//			FLOAT_QUAT_COMP_NORM_SHORTEST(q_sp, q_rp_sp, q_yaw_diff);
//		} else {
//			QUAT_COPY(q_sp, q_rp_sp);
//		}
//	}

//	//Function that reads the rc setpoint in an earth bound frame
//	public static void stabilization_attitude_read_rc_setpoint_quat_earth_bound_f( FloatQuat q_sp, boolean in_flight, boolean in_carefree, boolean coordinated_turn) {
//	  // FIXME: remove me, do in quaternion directly
//	  // is currently still needed, since the yaw setpoint integration is done in eulers
//	  if (STABILIZATION_ATTITUDE_TYPE_INT_DEFINED)
//	  stabilization_attitude_read_rc_setpoint_eulers(stab_att_sp_euler, in_flight, in_carefree, coordinated_turn);
//	  else
//	  stabilization_attitude_read_rc_setpoint_eulers_f(stab_att_sp_euler, in_flight, in_carefree, coordinated_turn);
//	 
//
//	  FloatVect3 zaxis = {0., 0., 1.};
//
//	   FloatQuat q_rp_cmd;
//	  stabilization_attitude_read_rc_roll_pitch_earth_quat_f(q_rp_cmd);
//
//	  if (in_flight) {
//	    /* get current heading setpoint */
//	     FloatQuat q_yaw_sp;
//
//	    if(STABILIZATION_ATTITUDE_TYPE_INT_DEFINED)
//	    FLOAT_QUAT_OF_AXIS_ANGLE(q_yaw_sp, zaxis, ANGLE_FLOAT_OF_BFP(stab_att_sp_euler.psi));
//	    else
//	    FLOAT_QUAT_OF_AXIS_ANGLE(q_yaw_sp, zaxis, stab_att_sp_euler.psi);
//	   
//
//	    FLOAT_QUAT_COMP(q_sp, q_yaw_sp, q_rp_cmd);
//	  }
//	  else {
//	     FloatQuat q_yaw;
//	    FLOAT_QUAT_OF_AXIS_ANGLE(q_yaw, zaxis, stateGetNedToBodyEulers_f().psi);
//
//	    /* roll/pitch commands applied to to current heading */
//	     FloatQuat q_rp_sp;
//	    FLOAT_QUAT_COMP(q_rp_sp, q_yaw, q_rp_cmd);
//	    FLOAT_QUAT_NORMALIZE(q_rp_sp);
//
//	    QUAT_COPY(q_sp, q_rp_sp);
//	  }
//	}
}
