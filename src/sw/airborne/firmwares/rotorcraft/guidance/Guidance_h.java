package sw.airborne.firmwares.rotorcraft.guidance;
import static sw.airborne.firmwares.rotorcraft.guidance.Guidance_h_ref.*;
import static sw.airborne.firmwares.rotorcraft.guidance.Guidance_v_ref.*;
import static sw.airborne.firmwares.rotorcraft.guidance.Guidance_v.*;


import sw.airborne.math.*;
import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_none.*;
import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_rate.*;
import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_rc_setpoint.*;
import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_euler_int.*;
import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_ref_euler_int.*;
import static sw.airborne.math.Pprz_algebra_int.*;
import static sw.airborne.math.Pprz_geodetic_int.*;
import static sw.airborne.firmwares.rotorcraft.Stabilization.*;
import static sw.airborne.math.Pprz_algebra.*;
import static sw.airborne.firmwares.rotorcraft.Navigation.*;
import static sw.airborne.State.*;

//import sw.airborne.math.*; 
import sw.include.Std;
import static sw.airborne.math.Pprz_algebra_int.*;
//import static sw.airborne.math.Pprz_algebra.*;
import static sw.include.Std.*;
import static sw.airborne.math.Pprz_geodetic.*;
import static sw.airborne.math.Pprz_orientation_conversion.*;
//import static sw.airborne.math.Pprz_geodetic_int.*;
import static sw.airborne.math.Pprz_geodetic_float.*;
import static sw.airborne.math.Pprz_algebra_float.*;
import static sw.airborne.math.Pprz_trig_int.*;
//import static sw.airborne.State.*;
//import static sw.airborne.firmwares.rotorcraft.guidance.Guidance_h_ref.*;

//import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_none.*;
//import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_rate.*;
//import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_rc_setpoint.*;
//import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_euler_int.*;


public class Guidance_h {

	public static boolean GUIDANCE_H_USE_REF = true;
	public static boolean GUIDANCE_H_USE_SPEED_REF = true;

	public final static int GUIDANCE_H_MODE_KILL       = 0;
	public final static int GUIDANCE_H_MODE_RATE      =  1;
	public final static int GUIDANCE_H_MODE_ATTITUDE   = 2;
	public final static int GUIDANCE_H_MODE_HOVER      = 3;
	public final static int GUIDANCE_H_MODE_NAV       =  4;
	public final static int GUIDANCE_H_MODE_RC_DIRECT =  5;
	public final static int GUIDANCE_H_MODE_CARE_FREE =  6;
	public final static int GUIDANCE_H_MODE_FORWARD   =  7;

	public static void guidance_h_SetKi(int _val) {            
		guidance_h_igain = _val;                
		INT_VECT2_ZERO(guidance_h_trim_att_integrator);	
	}

	/* Make sure that ref can only be temporarily disabled for testing,
	 * but not enabled if GUIDANCE_H_USE_REF was defined to false.
	 */
	public static void guidance_h_SetUseRef(int _val) {                    
		guidance_h_use_ref = (_val!=0) && GUIDANCE_H_USE_REF;    
	}

	public static void guidance_h_SetMaxSpeed(int _val) {          
		gh_set_max_speed(_val);                     
	}

	//---------------------guidance_h.c ------------------

	public static final int GUIDANCE_H_AGAIN =0;
	public static final int GUIDANCE_H_VGAIN =0;

	public static final double GUIDANCE_H_MAX_BANK =(20)*(3.1415926)/180.0;

	public static final boolean GUIDANCE_H_APPROX_FORCE_BY_THRUST= false;
	private static final boolean NO_ATTITUDE_RESET_ON_MODE_CHANGE = false;


	public static int guidance_h_mode;
	public static boolean guidance_h_use_ref;
	public static boolean guidance_h_approx_force_by_thrust;

	public static Int32Vect2 guidance_h_pos_sp;
	public static Int32Vect2 guidance_h_pos_ref;
	public static Int32Vect2 guidance_h_speed_ref;
	public static Int32Vect2 guidance_h_accel_ref;
	//#if GUIDANCE_H_USE_SPEED_REF
	public static Int32Vect2 guidance_h_speed_sp;
	//#endif
	public static Int32Vect2 guidance_h_pos_err;
	public static Int32Vect2 guidance_h_speed_err;
	public static Int32Vect2 guidance_h_trim_att_integrator;

	public static Int32Vect2  guidance_h_cmd_earth;
	public static Int32Eulers guidance_h_rc_sp;
	public static long guidance_h_heading_sp;

	public static int guidance_h_pgain;
	public static int guidance_h_dgain;
	public static int guidance_h_igain;
	public static int guidance_h_again;
	public static int guidance_h_vgain;

	public static int transition_percentage;
	public static int transition_theta_offset;

	public static void send_gh() {
		 NedCoor_i pos = stateGetPositionNed_i();
//		DOWNLINK_SEND_GUIDANCE_H_INT(DefaultChannel, DefaultDevice,
//				guidance_h_pos_sp.x, guidance_h_pos_sp.y,
//				guidance_h_pos_ref.x, guidance_h_pos_ref.y,
//				(pos.x), (pos.y));
	}

	public static void send_hover_loop() {
		 NedCoor_i pos = stateGetPositionNed_i();
		 NedCoor_i speed = stateGetSpeedNed_i();
		 NedCoor_i accel = stateGetAccelNed_i();
//		DOWNLINK_SEND_HOVER_LOOP(DefaultChannel, DefaultDevice,
//				guidance_h_pos_sp.x,
//				guidance_h_pos_sp.y,
//				(pos.x), (pos.y),
//				(speed.x), (speed.y),
//				(accel.x), (accel.y),
//				guidance_h_pos_err.x,
//				guidance_h_pos_err.y,
//				guidance_h_speed_err.x,
//				guidance_h_speed_err.y,
//				guidance_h_trim_att_integrator.x,
//				guidance_h_trim_att_integrator.y,
//				guidance_h_cmd_earth.x,
//				guidance_h_cmd_earth.y,
//				guidance_h_heading_sp);
	}

	public static void send_href() {
//		DOWNLINK_SEND_GUIDANCE_H_REF_INT(DefaultChannel, DefaultDevice,
//				guidance_h_pos_sp.x, guidance_h_pos_ref.x,
//				guidance_h_speed_ref.x, guidance_h_accel_ref.x,
//				guidance_h_pos_sp.y, guidance_h_pos_ref.y,
//				guidance_h_speed_ref.y, guidance_h_accel_ref.y);
	}

	public static void send_tune_hover() {
//		DOWNLINK_SEND_ROTORCRAFT_TUNE_HOVER(DefaultChannel, DefaultDevice,
//				radio_control.values[RADIO_ROLL],
//				radio_control.values[RADIO_PITCH],
//				radio_control.values[RADIO_YAW],
//				stabilization_cmd[COMMAND_ROLL],
//				stabilization_cmd[COMMAND_PITCH],
//				stabilization_cmd[COMMAND_YAW],
//				stabilization_cmd[COMMAND_THRUST],
//				(stateGetNedToBodyEulers_i().phi),
//				(stateGetNedToBodyEulers_i().theta),
//				(stateGetNedToBodyEulers_i().psi));
	}

	

	public static void guidance_h_init() {

		guidance_h_mode = GUIDANCE_H_MODE_KILL;
		guidance_h_use_ref = GUIDANCE_H_USE_REF;
		guidance_h_approx_force_by_thrust = GUIDANCE_H_APPROX_FORCE_BY_THRUST;

		INT_VECT2_ZERO(guidance_h_pos_sp);
		INT_VECT2_ZERO(guidance_h_trim_att_integrator);
		INT_EULERS_ZERO(guidance_h_rc_sp);
		guidance_h_heading_sp = 0;
		guidance_h_pgain = 50;// GUIDANCE_H_PGAIN;//hardcode TODO
		guidance_h_igain = 20;//GUIDANCE_H_IGAIN;//hardcode TODO
		guidance_h_dgain = 100;//GUIDANCE_H_DGAIN;//hardcode TODO
		guidance_h_again = GUIDANCE_H_AGAIN;
		guidance_h_vgain = GUIDANCE_H_VGAIN;
		transition_percentage = 0;
		transition_theta_offset = 0;

//		if(PERIODIC_TELEMETRY){
//			register_periodic_telemetry(DefaultPeriodic, "GUIDANCE_H_INT", send_gh);
//			register_periodic_telemetry(DefaultPeriodic, "HOVER_LOOP", send_hover_loop);
//			register_periodic_telemetry(DefaultPeriodic, "GUIDANCE_H_REF", send_href);
//			register_periodic_telemetry(DefaultPeriodic, "ROTORCRAFT_TUNE_HOVER", send_tune_hover);
//		}
	}


	public static void reset_guidance_reference_from_current_position() {
		VECT2_COPY(guidance_h_pos_ref, stateGetPositionNed_i());
		VECT2_COPY(guidance_h_speed_ref, stateGetSpeedNed_i());
		INT_VECT2_ZERO(guidance_h_accel_ref);
		gh_set_ref(guidance_h_pos_ref, guidance_h_speed_ref, guidance_h_accel_ref);

		INT_VECT2_ZERO(guidance_h_trim_att_integrator);
	}

	public static void guidance_h_mode_changed(int new_mode) {
		if (new_mode == guidance_h_mode)
			return;

		if (new_mode != GUIDANCE_H_MODE_FORWARD && new_mode != GUIDANCE_H_MODE_RATE) {
			transition_percentage = 0;
			transition_theta_offset = 0;
		}

		switch (new_mode) {
		case GUIDANCE_H_MODE_RC_DIRECT:
			stabilization_none_enter();
			break;

		case GUIDANCE_H_MODE_RATE:
			stabilization_rate_enter();
			break;

		case GUIDANCE_H_MODE_CARE_FREE:
			stabilization_attitude_reset_care_free_heading();
		case GUIDANCE_H_MODE_FORWARD:
		case GUIDANCE_H_MODE_ATTITUDE:
			if(NO_ATTITUDE_RESET_ON_MODE_CHANGE){
			/* reset attitude stabilization if previous mode was not using it */
				if (guidance_h_mode == GUIDANCE_H_MODE_KILL ||
						guidance_h_mode == GUIDANCE_H_MODE_RATE ||
						guidance_h_mode == GUIDANCE_H_MODE_RC_DIRECT) stabilization_attitude_enter();
				else break;
			}
				stabilization_attitude_enter();
			break;

		case GUIDANCE_H_MODE_HOVER:
			guidance_h_hover_enter();
			if(NO_ATTITUDE_RESET_ON_MODE_CHANGE){
				/* reset attitude stabilization if previous mode was not using it */
				if (guidance_h_mode == GUIDANCE_H_MODE_KILL ||
						guidance_h_mode == GUIDANCE_H_MODE_RATE ||
						guidance_h_mode == GUIDANCE_H_MODE_RC_DIRECT)
					stabilization_attitude_enter();
				else break;
			}
				stabilization_attitude_enter();
			break;

		case GUIDANCE_H_MODE_NAV:
			guidance_h_nav_enter();
			if(NO_ATTITUDE_RESET_ON_MODE_CHANGE){
				/* reset attitude stabilization if previous mode was not using it */
				if (guidance_h_mode == GUIDANCE_H_MODE_KILL ||
						guidance_h_mode == GUIDANCE_H_MODE_RATE ||
						guidance_h_mode == GUIDANCE_H_MODE_RC_DIRECT)
					stabilization_attitude_enter();
				else break;
			}
				stabilization_attitude_enter();
			break;

		default:
			break;
		}

		guidance_h_mode = new_mode;

	}


//	public static void guidance_h_read_rc(boolean  in_flight) {
//
//		switch ( guidance_h_mode ) {
//
//		case GUIDANCE_H_MODE_RC_DIRECT:
//			stabilization_none_read_rc();
//			break;
//
//		case GUIDANCE_H_MODE_RATE:
//			if(SWITCH_STICKS_FOR_RATE_CONTROL){
//			stabilization_rate_read_rc_switched_sticks();
//			}
//			else
//				stabilization_rate_read_rc();
//			
//			break;
//		case GUIDANCE_H_MODE_CARE_FREE:
//			stabilization_attitude_read_rc(in_flight, true, false);
//			break;
//		case GUIDANCE_H_MODE_FORWARD:
//			stabilization_attitude_read_rc(in_flight, false, true);
//			break;
//		case GUIDANCE_H_MODE_ATTITUDE:
//			stabilization_attitude_read_rc(in_flight, false, false);
//			break;
//		case GUIDANCE_H_MODE_HOVER:
//			stabilization_attitude_read_rc_setpoint_eulers(guidance_h_rc_sp, in_flight, false, false);
//			if(GUIDANCE_H_USE_SPEED_REF)
//			read_rc_setpoint_speed_i(guidance_h_speed_sp, in_flight);
//			
//			break;
//
//		case GUIDANCE_H_MODE_NAV:
//			if (radio_control.status == RC_OK) {
//				stabilization_attitude_read_rc_setpoint_eulers(guidance_h_rc_sp, in_flight, false, false);
//			}
//			else {
//				INT_EULERS_ZERO(guidance_h_rc_sp);
//			}
//			break;
//		default:
//			break;
//		}
//
//	}


	public static void guidance_h_run(boolean  in_flight) {
		switch ( guidance_h_mode ) {

		case GUIDANCE_H_MODE_RC_DIRECT:
			stabilization_none_run(in_flight);
			break;

		case GUIDANCE_H_MODE_RATE:
			stabilization_rate_run(in_flight);
			break;

		case GUIDANCE_H_MODE_FORWARD:
			if(transition_percentage < (100<<INT32_PERCENTAGE_FRAC)) {
				transition_run();
			}
		case GUIDANCE_H_MODE_CARE_FREE:
		case GUIDANCE_H_MODE_ATTITUDE:
			stabilization_attitude_run(in_flight);
			break;

		case GUIDANCE_H_MODE_HOVER:
			if (!in_flight)
				guidance_h_hover_enter();

			guidance_h_update_reference();

			/* set psi command */
			guidance_h_heading_sp = guidance_h_rc_sp.psi;
			/* compute x,y earth commands */
			guidance_h_traj_run(in_flight);
			/* set final attitude setpoint */
			stabilization_attitude_set_earth_cmd_i(guidance_h_cmd_earth,
					guidance_h_heading_sp);
			stabilization_attitude_run(in_flight);
			break;

		case GUIDANCE_H_MODE_NAV:
			if (!in_flight)
				guidance_h_nav_enter();

			if (horizontal_mode == HORIZONTAL_MODE_ATTITUDE) {
				 Int32Eulers sp_cmd_i = new Int32Eulers();
				sp_cmd_i.phi = nav_roll;
				sp_cmd_i.theta = nav_pitch;
				
				sp_cmd_i.psi = stateGetNedToBodyEulers_i().psi;
				stabilization_attitude_set_rpy_setpoint_i(sp_cmd_i);
			}
			else {
				INT32_VECT2_NED_OF_ENU(guidance_h_pos_sp, navigation_carrot);

				guidance_h_update_reference();

				/* set psi command */
				guidance_h_heading_sp = nav_heading;
				INT32_ANGLE_NORMALIZE(guidance_h_heading_sp);
				/* compute x,y earth commands */
				guidance_h_traj_run(in_flight);
				/* set final attitude setpoint */
				stabilization_attitude_set_earth_cmd_i(guidance_h_cmd_earth,
						guidance_h_heading_sp);
			}
			stabilization_attitude_run(in_flight);
			break;

		default:
			break;
		}
	}


	public static void guidance_h_update_reference() {
		/* compute reference even if usage temporarily disabled via guidance_h_use_ref */
		if(GUIDANCE_H_USE_REF){
			if(GUIDANCE_H_USE_SPEED_REF){
				if(guidance_h_mode == GUIDANCE_H_MODE_HOVER)
					Guidance_h_ref.gh_update_ref_from_speed_sp(guidance_h_speed_sp);
				else
					Guidance_h_ref.gh_update_ref_from_pos_sp(guidance_h_pos_sp);
			}else
				Guidance_h_ref.gh_update_ref_from_pos_sp(guidance_h_pos_sp);
		}

		/* either use the reference or simply copy the pos setpoint */
		if (guidance_h_use_ref) {
			/* convert our reference to generic representation */
			INT32_VECT2_RSHIFT(guidance_h_pos_ref,   gh_pos_ref,  (int) (GH_POS_REF_FRAC - INT32_POS_FRAC));
			INT32_VECT2_LSHIFT(guidance_h_speed_ref, gh_speed_ref, (int)(INT32_SPEED_FRAC - GH_SPEED_REF_FRAC));
			INT32_VECT2_LSHIFT(guidance_h_accel_ref, gh_accel_ref, (INT32_ACCEL_FRAC - GH_ACCEL_REF_FRAC));
		} else {
			VECT2_COPY(guidance_h_pos_ref, guidance_h_pos_sp);
			INT_VECT2_ZERO(guidance_h_speed_ref);
			INT_VECT2_ZERO(guidance_h_accel_ref);
		}

		if (GUIDANCE_H_USE_SPEED_REF){
			if (guidance_h_mode == GUIDANCE_H_MODE_HOVER) {
				VECT2_COPY(guidance_h_pos_sp, guidance_h_pos_ref); // for display only
			}
		}
	}


	public static final double MAX_POS_ERR =  POS_BFP_OF_REAL((float)16.0);
	public static final double MAX_SPEED_ERR = SPEED_BFP_OF_REAL((float)16.0);

	public static final int GUIDANCE_H_THRUST_CMD_FILTER = 10;
	

	/* with a pgain of 100 and a scale of 2,
	 * you get an angle of 5.6 degrees for 1m pos error */
	public static final int GH_GAIN_SCALE = 2;
	private static final int TRANSITION_MAX_OFFSET = 10;// ?????

	private static int  traj_max_bank;
	private static int total_max_bank;
	private static long thrust_cmd_filt;
	public static void guidance_h_traj_run(boolean in_flight) {
		/* maximum bank angle: default 20 deg, max 40 deg*/
		traj_max_bank = Math.max(BFP_OF_REAL((float)GUIDANCE_H_MAX_BANK, INT32_ANGLE_FRAC),
				BFP_OF_REAL((float)(40*3.14159/180.0), INT32_ANGLE_FRAC));
		total_max_bank = BFP_OF_REAL((float)(45*3.14159/180.0), INT32_ANGLE_FRAC);

		/* compute position error    */
		VECT2_DIFF(guidance_h_pos_err, guidance_h_pos_ref, stateGetPositionNed_i());
		/* saturate it               */
		VECT2_STRIM(guidance_h_pos_err, (int)-MAX_POS_ERR, (int)MAX_POS_ERR);

		/* compute speed error    */
		VECT2_DIFF(guidance_h_speed_err, guidance_h_speed_ref, stateGetSpeedNed_i());
		/* saturate it               */
		VECT2_STRIM(guidance_h_speed_err, (int)-MAX_SPEED_ERR, (int)MAX_SPEED_ERR);

		/* run PID */
		long pd_x =
				((guidance_h_pgain * guidance_h_pos_err.x) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
				((guidance_h_dgain * (guidance_h_speed_err.x >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2));
		long pd_y =
				((guidance_h_pgain * guidance_h_pos_err.y) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
				((guidance_h_dgain * (guidance_h_speed_err.y >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2));
		guidance_h_cmd_earth.x =
				pd_x +
				((guidance_h_vgain * guidance_h_speed_ref.x) >> 17) + /* speed feedforward gain */
				((guidance_h_again * guidance_h_accel_ref.x) >> 8);   /* acceleration feedforward gain */
		guidance_h_cmd_earth.y =
				pd_y +
				((guidance_h_vgain * guidance_h_speed_ref.x) >> 17) + /* speed feedforward gain */
				((guidance_h_again * guidance_h_accel_ref.y) >> 8);   /* acceleration feedforward gain */

		/* trim max bank angle from PD */
		VECT2_STRIM(guidance_h_cmd_earth, -traj_max_bank, traj_max_bank);

		/* Update pos & speed error integral, zero it if not in_flight.
		 * Integrate twice as fast when not only POS but also SPEED are wrong,
		 * but do not integrate POS errors when the SPEED is already catching up.
		 */
		if (in_flight) {
			/* ANGLE_FRAC (12) * GAIN (8) * LOOP_FREQ (9) . INTEGRATOR HIGH RES ANGLE_FRAX (28) */
			guidance_h_trim_att_integrator.x += (guidance_h_igain * pd_x);
			guidance_h_trim_att_integrator.y += (guidance_h_igain * pd_y);
			/* saturate it  */
			VECT2_STRIM(guidance_h_trim_att_integrator, -(traj_max_bank << 16), (traj_max_bank << 16));
			/* add it to the command */
			guidance_h_cmd_earth.x += (guidance_h_trim_att_integrator.x >> 16);
			guidance_h_cmd_earth.y += (guidance_h_trim_att_integrator.y >> 16);
		} else {
			INT_VECT2_ZERO(guidance_h_trim_att_integrator);
		}

		/* compute a better approximation of force commands by taking thrust into account */
		if (guidance_h_approx_force_by_thrust && in_flight) {
			//static int thrust_cmd_filt;
			long vertical_thrust = (stabilization_cmd[COMMAND_THRUST] * guidance_v_thrust_coeff) >> INT32_TRIG_FRAC;
			thrust_cmd_filt = (thrust_cmd_filt * GUIDANCE_H_THRUST_CMD_FILTER + vertical_thrust) / (GUIDANCE_H_THRUST_CMD_FILTER + 1);
			guidance_h_cmd_earth.x = ANGLE_BFP_OF_REAL((float)Math.atan2((guidance_h_cmd_earth.x * MAX_PPRZ / INT32_ANGLE_PI_2), thrust_cmd_filt));
			guidance_h_cmd_earth.y = ANGLE_BFP_OF_REAL((float)Math.atan2((guidance_h_cmd_earth.y * MAX_PPRZ / INT32_ANGLE_PI_2), thrust_cmd_filt));
		}

		VECT2_STRIM(guidance_h_cmd_earth, -total_max_bank, total_max_bank);
	}

	public static void guidance_h_hover_enter() {

		/* set horizontal setpoint to current position */
		VECT2_COPY(guidance_h_pos_sp, stateGetPositionNed_i());

		reset_guidance_reference_from_current_position();

		guidance_h_rc_sp.psi = stateGetNedToBodyEulers_i().psi;
	}

	public static void guidance_h_nav_enter() {

		/* horizontal position setpoint from navigation/flightplan */
		INT32_VECT2_NED_OF_ENU(guidance_h_pos_sp, navigation_carrot);

		reset_guidance_reference_from_current_position();

		nav_heading = stateGetNedToBodyEulers_i().psi;
	}

	public static  void transition_run() {
		//Add 0.00625%
		transition_percentage += 1<<(INT32_PERCENTAGE_FRAC-4);

		if(TRANSITION_MAX_OFFSET!=0){
			int max_offset = ANGLE_BFP_OF_REAL(TRANSITION_MAX_OFFSET);
			transition_theta_offset = INT_MULT_RSHIFT((transition_percentage<<(INT32_ANGLE_FRAC-INT32_PERCENTAGE_FRAC))/100, max_offset, INT32_ANGLE_FRAC);
		}
	}

	/// read speed setpoint from RC
//	public static void read_rc_setpoint_speed_i( Int32Vect2 speed_sp, boolean in_flight) {
//		if (in_flight) {
//			// negative pitch is forward
//			int rc_x = -radio_control.values[RADIO_PITCH];
//			int rc_y = radio_control.values[RADIO_ROLL];
//			DeadBand(rc_x, MAX_PPRZ/20);
//			DeadBand(rc_y, MAX_PPRZ/20);
//
//			// convert input from MAX_PPRZ range to SPEED_BFP
//			int max_speed = SPEED_BFP_OF_REAL(GUIDANCE_H_REF_MAX_SPEED);
//			/// @todo calc proper scale while making sure a division by zero can't occur
//			//int rc_norm = sqrtf(rc_x * rc_x + rc_y * rc_y);
//			//int max_pprz = rc_norm * MAX_PPRZ / Max(abs(rc_x), abs(rc_y);
//			rc_x = rc_x * max_speed / MAX_PPRZ;
//			rc_y = rc_y * max_speed / MAX_PPRZ;
//
//			/* Rotate from body to NED frame by negative psi angle */
//			int psi = -stateGetNedToBodyEulers_i().psi;
//			int s_psi, c_psi;
//			s_psi = PPRZ_ITRIG_SIN(psi);
//			c_psi = PPRZ_ITRIG_COS(psi);
//			speed_sp.x = (int)(( (int)c_psi * rc_x + (int)s_psi * rc_y) >> INT32_TRIG_FRAC);
//			speed_sp.y = (int)((-(int)s_psi * rc_x + (int)c_psi * rc_y) >> INT32_TRIG_FRAC);
//		}
//		else {
//			speed_sp.x = 0;
//			speed_sp.y = 0;
//		}
//	}
}
