package sw.airborne.firmwares.rotorcraft.guidance;
import static sw.airborne.State.*;
import static sw.airborne.math.Pprz_algebra_int.*;
import static sw.include.Std.*;
import static sw.airborne.firmwares.rotorcraft.Navigation.*;
import static sw.airborne.firmwares.rotorcraft.Stabilization.*;
import static sw.airborne.firmwares.rotorcraft.guidance.Guidance_v_ref.*;
import static sw.airborne.firmwares.rotorcraft.guidance.Guidance_v_adapt.*;
import sw.airborne.math.Int32RMat;
import static sw.airborne.subsystems.datalink.telemetry.*;

public class Guidance_v {
	public static final int GUIDANCE_V_MODE_KILL     = 0;
	public static final int GUIDANCE_V_MODE_RC_DIRECT =1;
	public static final int GUIDANCE_V_MODE_RC_CLIMB  =2;
	public static final int GUIDANCE_V_MODE_CLIMB    = 3;
	public static final int GUIDANCE_V_MODE_HOVER    = 4;
	public static final int GUIDANCE_V_MODE_NAV     =  5;

	public static void guidance_v_SetKi(int _val) {			
		guidance_v_ki = _val;				
		guidance_v_z_sum_err = 0;			
	}

	//------------------------------guidance_v.c -----------------------------

	public static boolean GUIDANCE_V_ADAPT_THROTTLE_ENABLED = false; 
	public static double GUIDANCE_V_NOMINAL_HOVER_THROTTLE = 0.4;
//	static{
//		if(!GUIDANCE_V_NOMINAL_HOVER_THROTLLE_DEFINED){
//			if(!GUIDANCE_V_ADAPT_THROTTLE_ENBALED_DEFINED){
//				GUIDANCE_V_ADAPT_THROTTLE_ENABLED =false;
//			}
//		}else{
//			GUIDANCE_V_NOMINAL_HOVER_THROTTLE = 0.4;
//			if(!GUIDANCE_V_ADAPT_THROTTLE_ENBALED_DEFINED){
//				GUIDANCE_V_ADAPT_THROTTLE_ENABLED =true;
//			}
//		}
//	}

	private static final int MAX_PPRZ = 9600;
	public static final double GUIDANCE_V_CLIMB_RC_DEADBAND =MAX_PPRZ/10;//9600
	public static final double GUIDANCE_V_MAX_RC_CLIMB_SPEED =GUIDANCE_V_REF_MIN_ZD;
	public static final double GUIDANCE_V_MAX_RC_DESCENT_SPEED =GUIDANCE_V_REF_MAX_ZD;
	
	public static final int COMMAND_THRUST = 3;//TODO from generated airframe.h
	private static final boolean NO_RC_THRUST_LIMIT = false;

	public static long guidance_v_mode;
	public static long guidance_v_ff_cmd;
	public static long guidance_v_fb_cmd;
	public static long guidance_v_delta_t;

	public static double guidance_v_nominal_throttle;
	public static boolean guidance_v_adapt_throttle_enabled;

	public static long guidance_v_rc_delta_t;

	public static long guidance_v_rc_zd_sp;

	public static long guidance_v_z_sp;
	public static long guidance_v_zd_sp;
	public static long guidance_v_z_ref;
	public static long guidance_v_zd_ref;
	public static long guidance_v_zdd_ref;

	public static long guidance_v_kp;
	public static long guidance_v_kd;
	public static long guidance_v_ki;

	public static long guidance_v_z_sum_err;

	public static long guidance_v_thrust_coeff;

	public static void GuidanceVSetRef(int _pos, int _speed, int _accel) { 
		gv_set_ref(_pos, _speed, _accel);        
		guidance_v_z_ref = _pos;            
		guidance_v_zd_ref = _speed;          
		guidance_v_zdd_ref = _accel;             
	}
	public static void GuidanceVSetRef(long _pos, int _speed, int _accel) { 
		gv_set_ref(_pos, _speed, _accel);        
		guidance_v_z_ref = _pos;            
		guidance_v_zd_ref = _speed;          
		guidance_v_zdd_ref = _accel;             
	}
	public static void GuidanceVSetRef(long _pos, long _speed, int _accel) { 
		gv_set_ref(_pos, _speed, _accel);        
		guidance_v_z_ref = _pos;            
		guidance_v_zd_ref = _speed;          
		guidance_v_zdd_ref = _accel;             
	}

	public static void send_vert_loop() {
//		DOWNLINK_SEND_VERT_LOOP(DefaultChannel, DefaultDevice,
//				guidance_v_z_sp, guidance_v_zd_sp,
//				(stateGetPositionNed_i().z),
//				(stateGetSpeedNed_i().z),
//				(stateGetAccelNed_i().z),
//				guidance_v_z_ref, guidance_v_zd_ref,
//				guidance_v_zdd_ref,
//				gv_adapt_X,
//				gv_adapt_P,
//				gv_adapt_Xmeas,
//				guidance_v_z_sum_err,
//				guidance_v_ff_cmd,
//				guidance_v_fb_cmd,
//				guidance_v_delta_t);
	}

	public static void send_tune_vert() {
//		DOWNLINK_SEND_TUNE_VERT(DefaultChannel, DefaultDevice,
//				guidance_v_z_sp,
//				(stateGetPositionNed_i().z),
//				guidance_v_z_ref,
//				guidance_v_zd_ref);
	}


	public static void guidance_v_init() {

		guidance_v_mode = GUIDANCE_V_MODE_KILL;

		guidance_v_kp = 150;// in nps/generated/airframe.h GUIDANCE_V_HOVER_KP;
		guidance_v_kd = 80 ;//GUIDANCE_V_HOVER_KD;
		guidance_v_ki = 20;//GUIDANCE_V_HOVER_KI;

		guidance_v_z_sum_err = 0;

		guidance_v_nominal_throttle = GUIDANCE_V_NOMINAL_HOVER_THROTTLE;
		guidance_v_adapt_throttle_enabled = GUIDANCE_V_ADAPT_THROTTLE_ENABLED;

		gv_adapt_init();

//		if(PERIODIC_TELEMETRY){
//			register_periodic_telemetry(DefaultPeriodic, "VERT_LOOP", send_vert_loop);
//			register_periodic_telemetry(DefaultPeriodic, "TUNE_VERT", send_tune_vert);
//		}
	}

	private static int climb_scale; 
	private static int descent_scale;
//	public static void guidance_v_read_rc() {
//
//		/* used in RC_DIRECT directly and as saturation in CLIMB and HOVER */
//		guidance_v_rc_delta_t = (int)radio_control.values[RADIO_THROTTLE];
//
//		/* used in RC_CLIMB */
//		guidance_v_rc_zd_sp = (MAX_PPRZ/2) - (int)radio_control.values[RADIO_THROTTLE];
//		DeadBand(guidance_v_rc_zd_sp, GUIDANCE_V_CLIMB_RC_DEADBAND);
//
//		climb_scale = (int) Math.abs(SPEED_BFP_OF_REAL(GUIDANCE_V_MAX_RC_CLIMB_SPEED) / (MAX_PPRZ/2 - GUIDANCE_V_CLIMB_RC_DEADBAND));
//		descent_scale = (int) Math.abs(SPEED_BFP_OF_REAL(GUIDANCE_V_MAX_RC_DESCENT_SPEED) / (MAX_PPRZ/2 - GUIDANCE_V_CLIMB_RC_DEADBAND));
//
//		if(guidance_v_rc_zd_sp > 0)
//			guidance_v_rc_zd_sp *= descent_scale;
//		else
//			guidance_v_rc_zd_sp *= climb_scale;
//	}

	public static void guidance_v_mode_changed(int new_mode) {

		if (new_mode == guidance_v_mode)
			return;

		switch (new_mode) {
		case GUIDANCE_V_MODE_HOVER:
			guidance_v_z_sp = stateGetPositionNed_i().z; // set current altitude as setpoint
			guidance_v_z_sum_err = 0;
			GuidanceVSetRef(stateGetPositionNed_i().z, 0, 0);
			break;

		case GUIDANCE_V_MODE_RC_CLIMB:
		case GUIDANCE_V_MODE_CLIMB:
			guidance_v_zd_sp = 0;
		case GUIDANCE_V_MODE_NAV:
			guidance_v_z_sum_err = 0;
			GuidanceVSetRef(stateGetPositionNed_i().z, stateGetSpeedNed_i().z, 0);
			break;

		default:
			break;

		}

		guidance_v_mode = new_mode;

	}

	public static void guidance_v_notify_in_flight( boolean in_flight) {
		if (in_flight) {
			Guidance_v_adapt.gv_adapt_init();
		}
	}


	public static void guidance_v_run(boolean in_flight) {

		// FIXME... SATURATIONS NOT TAKEN INTO ACCOUNT
		// AKA SUPERVISION and co
		guidance_v_thrust_coeff = get_vertical_thrust_coeff();
		if (in_flight) {
			long vertical_thrust = (stabilization_cmd[COMMAND_THRUST] * guidance_v_thrust_coeff) >> 14;
			Guidance_v_adapt.gv_adapt_run(stateGetAccelNed_i().z, vertical_thrust, guidance_v_zd_ref);
		}
		else {
			/* reset estimate while not in_flight */
			Guidance_v_adapt.gv_adapt_init();
		}

		switch ((int)guidance_v_mode) {

		case GUIDANCE_V_MODE_RC_DIRECT:
			guidance_v_z_sp = stateGetPositionNed_i().z; // for display only
			stabilization_cmd[COMMAND_THRUST] = guidance_v_rc_delta_t;
			break;

		case GUIDANCE_V_MODE_RC_CLIMB:
			guidance_v_zd_sp = guidance_v_rc_zd_sp;
			Guidance_v_ref.gv_update_ref_from_zd_sp(guidance_v_zd_sp);
			run_hover_loop(in_flight);
			stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
			break;

		case GUIDANCE_V_MODE_CLIMB:
			Guidance_v_ref.gv_update_ref_from_zd_sp(guidance_v_zd_sp);
			run_hover_loop(in_flight);
			if(NO_RC_THRUST_LIMIT){

				stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
			}
			else{
				// saturate max authority with RC stick
				stabilization_cmd[COMMAND_THRUST] = Math.min(guidance_v_rc_delta_t, guidance_v_delta_t);

			}

			break;

		case GUIDANCE_V_MODE_HOVER:
			guidance_v_zd_sp = 0;
			Guidance_v_ref.gv_update_ref_from_z_sp(guidance_v_z_sp);
			run_hover_loop(in_flight);
			if(NO_RC_THRUST_LIMIT)
			stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
			else
				
				stabilization_cmd[COMMAND_THRUST] = Math.min(guidance_v_rc_delta_t, guidance_v_delta_t);
			
			break;

		case GUIDANCE_V_MODE_NAV:
		{
			if (vertical_mode == VERTICAL_MODE_ALT) {
				guidance_v_z_sp = -nav_flight_altitude;
				guidance_v_zd_sp = 0;
				Guidance_v_ref.gv_update_ref_from_z_sp(guidance_v_z_sp);
				run_hover_loop(in_flight);
			}
			else if (vertical_mode == VERTICAL_MODE_CLIMB) {
				guidance_v_z_sp = stateGetPositionNed_i().z;
				guidance_v_zd_sp = -nav_climb;
				Guidance_v_ref.gv_update_ref_from_zd_sp(guidance_v_zd_sp);
				run_hover_loop(in_flight);
			}
			else if (vertical_mode == VERTICAL_MODE_MANUAL) {
				guidance_v_z_sp = stateGetPositionNed_i().z;
				guidance_v_zd_sp = stateGetSpeedNed_i().z;
				GuidanceVSetRef(guidance_v_z_sp, guidance_v_zd_sp, 0);
				guidance_v_z_sum_err = 0;
				guidance_v_delta_t = nav_throttle;
			}
			if(NO_RC_THRUST_LIMIT)
			stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
//			else{
//				/* use rc limitation if available */
//				if (radio_control.status == RC_OK)
//					stabilization_cmd[COMMAND_THRUST] = Math.min(guidance_v_rc_delta_t, guidance_v_delta_t);
//				else
//					stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
//			}
			break;
		}
		default:
			break;
		}
	}

	/// get the cosine of the angle between thrust vector and gravity vector
	private static int max_bank_coef;
	public static long get_vertical_thrust_coeff() {
		//static const int max_bank_coef = BFP_OF_REAL(RadOfDeg(30.), INT32_TRIG_FRAC);
		max_bank_coef = BFP_OF_REAL((float)((30.0) * (3.1415926/180.0)), 14);
		Int32RMat att = stateGetNedToBodyRMat_i();
		/* thrust vector:
		 *  INT32_RMAT_VMULT(thrust_vect, att, zaxis)
		 * same as last colum of rmat with INT32_TRIG_FRAC
		 * struct Int32Vect thrust_vect = {att.m[2], att.m[5], att.m[8]};
		 *
		 * Angle between two vectors v1 and v2:
		 *  angle = acos(dot(v1, v2) / (norm(v1) * norm(v2)))
		 * since here both are already of unit length:
		 *  angle = acos(dot(v1, v2))
		 * since we we want the cosine of the angle we simply need
		 *  thrust_coeff = dot(v1, v2)
		 * also can be simplified considering: v1 is zaxis with (0,0,1)
		 *  dot(v1, v2) = v1.z * v2.z = v2.z
		 */
		long coef = att.m[8];
		if (coef < max_bank_coef)
			coef = max_bank_coef;
		return coef;
	}


	public static final int FF_CMD_FRAC = 18;

	public static void run_hover_loop(boolean in_flight) {

		/* convert our reference to generic representation */
		long tmp  = gv_z_ref>>(GV_Z_REF_FRAC - INT32_POS_FRAC);
		guidance_v_z_ref = tmp;
		guidance_v_zd_ref = gv_zd_ref<<(INT32_SPEED_FRAC - GV_ZD_REF_FRAC);
		guidance_v_zdd_ref = gv_zdd_ref<<(INT32_ACCEL_FRAC - GV_ZDD_REF_FRAC);
		/* compute the error to our reference */
		long err_z  = guidance_v_z_ref - stateGetPositionNed_i().z;
		//Bound(err_z, GUIDANCE_V_MIN_ERR_Z, GUIDANCE_V_MAX_ERR_Z);
		Bound(err_z, POS_BFP_OF_REAL(-10.), POS_BFP_OF_REAL(-10.));
		long err_zd = guidance_v_zd_ref - stateGetSpeedNed_i().z;
		//Bound(err_zd, GUIDANCE_V_MIN_ERR_ZD, GUIDANCE_V_MAX_ERR_ZD);
		Bound(err_zd, SPEED_BFP_OF_REAL(-10.), SPEED_BFP_OF_REAL(10.));

		if (in_flight) {
			guidance_v_z_sum_err += err_z;
			//Bound(guidance_v_z_sum_err, -GUIDANCE_V_MAX_SUM_ERR, GUIDANCE_V_MAX_SUM_ERR);
			Bound(guidance_v_z_sum_err, -2000000, 2000000);
		}
		else
			guidance_v_z_sum_err = 0;

		/* our nominal command : (g + zdd)*m   */
		int inv_m;
		if (guidance_v_adapt_throttle_enabled) {
			inv_m =  gv_adapt_X >> (GV_ADAPT_X_FRAC - FF_CMD_FRAC);
		}
		else {
			/* use the fixed nominal throttle */
			inv_m = BFP_OF_REAL((float)(9.81 / (guidance_v_nominal_throttle * MAX_PPRZ)), FF_CMD_FRAC);
		}

		long g_m_zdd = BFP_OF_REAL((float)9.81, FF_CMD_FRAC) -
				(guidance_v_zdd_ref << (FF_CMD_FRAC - INT32_ACCEL_FRAC));

		guidance_v_ff_cmd = g_m_zdd / inv_m;
		/* feed forward command */
		guidance_v_ff_cmd = (guidance_v_ff_cmd << INT32_TRIG_FRAC) / guidance_v_thrust_coeff;

		/* bound the nominal command to 0.9*MAX_PPRZ */
		Bound(guidance_v_ff_cmd, 0, 8640);


		/* our error feed back command                   */
		/* z-axis pointing down . positive error means we need less thrust */
		guidance_v_fb_cmd = ((-guidance_v_kp * err_z)  >> 7) +
				((-guidance_v_kd * err_zd) >> 16) +
				((-guidance_v_ki * guidance_v_z_sum_err) >> 16);

		guidance_v_delta_t = guidance_v_ff_cmd + guidance_v_fb_cmd;

		/* bound the result */
		Bound(guidance_v_delta_t, 0, MAX_PPRZ);

	}
}
