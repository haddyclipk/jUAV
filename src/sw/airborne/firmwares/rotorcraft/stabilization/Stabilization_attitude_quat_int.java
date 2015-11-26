//package sw.airborne.firmwares.rotorcraft.stabilization;
//import static sw.airborne.math.Pprz_trig_int.*;
//import static sw.airborne.math.Pprz_algebra.*;
//import static sw.airborne.firmwares.rotorcraft.Stabilization.*;
//import static sw.include.Std.*;
//
//import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_rc_setpoint.*;
//import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_ref_quat_int.*;
//import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_ref_euler_int.*;
//import static sw.airborne.math.Pprz_algebra_int.*;
//import static sw.airborne.State.*;
//import sw.airborne.math.*;
//import  sw.airborne.math.Int32Quat;
//import sw.airborne.math.Int32Vect2;
//import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_quat_transformations.*;
//import sw.include.Std;
//import static sw.airborne.math.Pprz_geodetic.*;
//import static sw.airborne.math.Pprz_orientation_conversion.*;
//import static sw.airborne.math.Pprz_geodetic_int.*;
//import static sw.airborne.math.Pprz_geodetic_float.*;
//import static sw.airborne.math.Pprz_algebra_float.*;
//
//public class Stabilization_attitude_quat_int {
//	//TODO
//	public static Int32AttitudeGains stabilization_gains = {
//		{STABILIZATION_ATTITUDE_PHI_PGAIN, STABILIZATION_ATTITUDE_THETA_PGAIN, STABILIZATION_ATTITUDE_PSI_PGAIN },
//		{STABILIZATION_ATTITUDE_PHI_DGAIN, STABILIZATION_ATTITUDE_THETA_DGAIN, STABILIZATION_ATTITUDE_PSI_DGAIN },
//		{STABILIZATION_ATTITUDE_PHI_DDGAIN, STABILIZATION_ATTITUDE_THETA_DDGAIN, STABILIZATION_ATTITUDE_PSI_DDGAIN },
//		{STABILIZATION_ATTITUDE_PHI_IGAIN, STABILIZATION_ATTITUDE_THETA_IGAIN, STABILIZATION_ATTITUDE_PSI_IGAIN }
//	};
//
//
//
//	public static Int32Quat stabilization_att_sum_err_quat;
//	public static Int32Eulers stabilization_att_sum_err;
//
//	public static int stabilization_att_fb_cmd[] = new int[COMMANDS_NB];
//	public static int stabilization_att_ff_cmd[]= new int[COMMANDS_NB];
//
//	public static final int IERROR_SCALE = 1024;
//	public static final int  GAIN_PRESCALER_FF= 48;
//	public static final int  GAIN_PRESCALER_P =48;
//	public static final int GAIN_PRESCALER_D =48;
//	public static final int  GAIN_PRESCALER_I =48;
//
//
//
//	private static final int REF_RATE_FRAC = 16;
//
//
//
//	private static final int MAX_PPRZ = 9600;
//
//
//
//	public static void send_att() { //FIXME really use this message here ?
//		Int32Rates body_rate = stateGetBodyRates_i();
//		Int32Eulers att = stateGetNedToBodyEulers_i();
////		DOWNLINK_SEND_STAB_ATTITUDE_INT(DefaultChannel, DefaultDevice,
////				(body_rate.p), (body_rate.q), (body_rate.r),
////				(att.phi), (att.theta), (att.psi),
////				stab_att_sp_euler.phi,
////				stab_att_sp_euler.theta,
////				stab_att_sp_euler.psi,
////				stabilization_att_sum_err.phi,
////				stabilization_att_sum_err.theta,
////				stabilization_att_sum_err.psi,
////				stabilization_att_fb_cmd[COMMAND_ROLL],
////				stabilization_att_fb_cmd[COMMAND_PITCH],
////				stabilization_att_fb_cmd[COMMAND_YAW],
////				stabilization_att_ff_cmd[COMMAND_ROLL],
////				stabilization_att_ff_cmd[COMMAND_PITCH],
////				stabilization_att_ff_cmd[COMMAND_YAW],
////				stabilization_cmd[COMMAND_ROLL],
////				stabilization_cmd[COMMAND_PITCH],
////				stabilization_cmd[COMMAND_YAW]);
//	}
//
//	public static void send_att_ref() {
////		DOWNLINK_SEND_STAB_ATTITUDE_REF_INT(DefaultChannel, DefaultDevice,
////				stab_att_sp_euler.phi,
////				stab_att_sp_euler.theta,
////				stab_att_sp_euler.psi,
////				stab_att_ref_euler.phi,
////				stab_att_ref_euler.theta,
////				stab_att_ref_euler.psi,
////				stab_att_ref_rate.p,
////				stab_att_ref_rate.q,
////				stab_att_ref_rate.r,
////				stab_att_ref_accel.p,
////				stab_att_ref_accel.q,
////				stab_att_ref_accel.r);
//	}
//
//	public static void send_ahrs_ref_quat() {
//		Int32Quat quat = stateGetNedToBodyQuat_i();
////		DOWNLINK_SEND_AHRS_REF_QUAT(DefaultChannel, DefaultDevice,
////				stab_att_ref_quat.qi,
////				stab_att_ref_quat.qx,
////				stab_att_ref_quat.qy,
////				stab_att_ref_quat.qz,
////				(quat.qi),
////				(quat.qx),
////				(quat.qy),
////				(quat.qz));
//	}
//
//
//	public static void stabilization_attitude_init() {
//
//		stabilization_attitude_ref_init();
//
//		INT32_QUAT_ZERO( stabilization_att_sum_err_quat );
//		INT_EULERS_ZERO( stabilization_att_sum_err );
//
////		if(PERIODIC_TELEMETRY){
////			register_periodic_telemetry(DefaultPeriodic, "STAB_ATTITUDE", send_att);
////			register_periodic_telemetry(DefaultPeriodic, "STAB_ATTITUDE_REF", send_att_ref);
////			register_periodic_telemetry(DefaultPeriodic, "AHRS_REF_QUAT", send_ahrs_ref_quat);
////		}
//	}
//
//	public static  Int32Eulers stab_att_sp_euler;
//
//	public static void stabilization_attitude_enter() {
//
//		/* reset psi setpoint to current psi angle */
//		stab_att_sp_euler.psi = stabilization_attitude_get_heading_i();
//
//		stabilization_attitude_ref_enter();
//
//		INT32_QUAT_ZERO(stabilization_att_sum_err_quat);
//		INT_EULERS_ZERO(stabilization_att_sum_err);
//
//	}
//
//	public static Int32Quat stab_att_sp_quat=new Int32Quat();
//
//	public static void stabilization_attitude_set_failsafe_setpoint() {
//		/* set failsafe to zero roll/pitch and current heading */
//		int heading2 = stabilization_attitude_get_heading_i() / 2;
//		stab_att_sp_quat.qi = PPRZ_ITRIG_COS( heading2);
//		stab_att_sp_quat.qx = 0;
//		stab_att_sp_quat.qy = 0;
//		stab_att_sp_quat.qz = PPRZ_ITRIG_SIN( heading2);
//	}
//
//	public static void stabilization_attitude_set_rpy_setpoint_i( Int32Eulers rpy) {
//		// stab_att_sp_euler.psi still used in ref..
//		//	memcpy(stab_att_sp_euler, rpy, sizeof( Int32Eulers));
//		stab_att_sp_euler.phi = rpy.phi;
//		stab_att_sp_euler.psi = rpy.psi;
//		stab_att_sp_euler.theta = rpy.theta;
//
//		
//		quat_from_rpy_cmd_i(stab_att_sp_quat, stab_att_sp_euler);
//	}
//
//	public static void stabilization_attitude_set_earth_cmd_i( Int32Vect2 cmd, int heading) {
//		// stab_att_sp_euler.psi still used in ref..
//		stab_att_sp_euler.psi = heading;
//
//		// compute sp_euler phi/theta for debugging/telemetry
//		/* Rotate horizontal commands to body frame by psi */
//		long psi = stateGetNedToBodyEulers_i().psi;
//		long s_psi, c_psi;
//		s_psi = PPRZ_ITRIG_SIN( psi);
//		c_psi = PPRZ_ITRIG_COS( psi);
//		stab_att_sp_euler.phi = (-s_psi * cmd.x + c_psi * cmd.y) >> INT32_TRIG_FRAC;
//		stab_att_sp_euler.theta = -(c_psi * cmd.x + s_psi * cmd.y) >> INT32_TRIG_FRAC;
//
//		quat_from_earth_cmd_i(stab_att_sp_quat, cmd, heading);
//	}
//	public static void stabilization_attitude_set_earth_cmd_i( Int32Vect2 cmd, long heading) {
//		// stab_att_sp_euler.psi still used in ref..
//		stab_att_sp_euler.psi = heading;
//		
//		// compute sp_euler phi/theta for debugging/telemetry
//		/* Rotate horizontal commands to body frame by psi */
//		long psi = stateGetNedToBodyEulers_i().psi;
//		long s_psi, c_psi;
//		s_psi = PPRZ_ITRIG_SIN( psi);
//		c_psi = PPRZ_ITRIG_COS( psi);
//		stab_att_sp_euler.phi = (-s_psi * cmd.x + c_psi * cmd.y) >> INT32_TRIG_FRAC;
//		stab_att_sp_euler.theta = -(c_psi * cmd.x + s_psi * cmd.y) >> INT32_TRIG_FRAC;
//		
//		quat_from_earth_cmd_i(stab_att_sp_quat, cmd, heading);
//	}
//
//	public static int OFFSET_AND_ROUND(int _a,int _b){
//		return (((_a)+(1<<((_b)-1)))>>(_b));
//	}
//	public static long OFFSET_AND_ROUND(long _a,int _b){
//		return (((_a)+(1<<((_b)-1)))>>(_b));
//	}
//	//#define OFFSET_AND_ROUND(_a, _b) (((_a)+(1<<((_b)-1)))>>(_b))
//
//	public static int OFFSET_AND_ROUND2(int _a, int _b) {
//
//		return (((_a)+(1<<((_b)-1))-((_a)<0?1:0))>>(_b));
//	}
//	//#define OFFSET_AND_ROUND2(_a, _b) (((_a)+(1<<((_b)-1))-((_a)<0?1:0))>>(_b))
//
//	public static void attitude_run_ff(int ff_commands[],  Int32AttitudeGains gains,  Int32Rates ref_accel)
//	{
//		/* Compute feedforward based on reference acceleration */
//
//		ff_commands[COMMAND_ROLL]  = GAIN_PRESCALER_FF * gains.dd.x * RATE_FLOAT_OF_BFP(ref_accel.p) / (1 << 7);
//		ff_commands[COMMAND_PITCH] = GAIN_PRESCALER_FF * gains.dd.y * RATE_FLOAT_OF_BFP(ref_accel.q) / (1 << 7);
//		ff_commands[COMMAND_YAW]   = GAIN_PRESCALER_FF * gains.dd.z * RATE_FLOAT_OF_BFP(ref_accel.r) / (1 << 7);
//	}
//
//	public static void attitude_run_fb(int fb_commands[],  Int32AttitudeGains gains,  Int32Quat att_err,
//			Int32Rates rate_err,  Int32Quat sum_err)
//	{
//		/*  PID feedback */
//		fb_commands[COMMAND_ROLL] =
//				GAIN_PRESCALER_P * gains.p.x  * QUAT1_FLOAT_OF_BFP(att_err.qx) / 4 +
//				GAIN_PRESCALER_D * gains.d.x  * RATE_FLOAT_OF_BFP(rate_err.p) / 16 +
//				GAIN_PRESCALER_I * gains.i.x  * QUAT1_FLOAT_OF_BFP(sum_err.qx) / 2;
//
//		fb_commands[COMMAND_PITCH] =
//				GAIN_PRESCALER_P * gains.p.y  * QUAT1_FLOAT_OF_BFP(att_err.qy) / 4 +
//				GAIN_PRESCALER_D * gains.d.y  * RATE_FLOAT_OF_BFP(rate_err.q)  / 16 +
//				GAIN_PRESCALER_I * gains.i.y  * QUAT1_FLOAT_OF_BFP(sum_err.qy) / 2;
//
//		fb_commands[COMMAND_YAW] =
//				GAIN_PRESCALER_P * gains.p.z  * QUAT1_FLOAT_OF_BFP(att_err.qz) / 4 +
//				GAIN_PRESCALER_D * gains.d.z  * RATE_FLOAT_OF_BFP(rate_err.r)  / 16 +
//				GAIN_PRESCALER_I * gains.i.z  * QUAT1_FLOAT_OF_BFP(sum_err.qz) / 2;
//
//	}
//	public static Int32Quat stab_att_ref_quat;
//	public static Int32Rates stab_att_ref_rate;
//	public static Int32Rates stab_att_ref_accel;
//
//	public static void stabilization_attitude_run(boolean enable_integrator) {
//
//		/*
//		 * Update reference
//		 */
//		stabilization_attitude_ref_update();
//
//		/*
//		 * Compute errors for feedback
//		 */
//
//		/* attitude error                          */
//		Int32Quat att_err;
//		Int32Quat att_quat = stateGetNedToBodyQuat_i();
//		INT32_QUAT_INV_COMP(att_err, att_quat, stab_att_ref_quat);
//		/* wrap it in the shortest direction       */
//		INT32_QUAT_WRAP_SHORTEST(att_err);
//		INT32_QUAT_NORMALIZE(att_err);
//
//		/*  rate error                */
//		Int32Rates rate_ref_scaled =new Int32Rates();
//		rate_ref_scaled.p=OFFSET_AND_ROUND(stab_att_ref_rate.p, (REF_RATE_FRAC - INT32_RATE_FRAC));
//				rate_ref_scaled.q=	OFFSET_AND_ROUND(stab_att_ref_rate.q, (REF_RATE_FRAC - INT32_RATE_FRAC));
//						rate_ref_scaled.r=		OFFSET_AND_ROUND(stab_att_ref_rate.r, (REF_RATE_FRAC - INT32_RATE_FRAC));
//		Int32Rates rate_err;
//		Int32Rates body_rate = stateGetBodyRates_i();
//		RATES_DIFF(rate_err, rate_ref_scaled, (body_rate));
//
//		/* integrated error */
//		if (enable_integrator) {
//			Int32Quat new_sum_err, scaled_att_err;
//			/* update accumulator */
//			scaled_att_err.qi = att_err.qi;
//			scaled_att_err.qx = att_err.qx / IERROR_SCALE;
//			scaled_att_err.qy = att_err.qy / IERROR_SCALE;
//			scaled_att_err.qz = att_err.qz / IERROR_SCALE;
//			INT32_QUAT_COMP(new_sum_err, stabilization_att_sum_err_quat, scaled_att_err);
//			INT32_QUAT_NORMALIZE(new_sum_err);
//			QUAT_COPY(stabilization_att_sum_err_quat, new_sum_err);
//			INT32_EULERS_OF_QUAT(stabilization_att_sum_err, stabilization_att_sum_err_quat);
//		} else {
//			/* reset accumulator */
//			INT32_QUAT_ZERO( stabilization_att_sum_err_quat );
//			INT_EULERS_ZERO( stabilization_att_sum_err );
//		}
//
//		/* compute the feed forward command */
//		attitude_run_ff(stabilization_att_ff_cmd, stabilization_gains, stab_att_ref_accel);
//
//		/* compute the feed back command */
//		attitude_run_fb(stabilization_att_fb_cmd, stabilization_gains, att_err, rate_err, stabilization_att_sum_err_quat);
//
//		/* sum feedforward and feedback */
//		stabilization_cmd[COMMAND_ROLL] = stabilization_att_fb_cmd[COMMAND_ROLL] + stabilization_att_ff_cmd[COMMAND_ROLL];
//		stabilization_cmd[COMMAND_PITCH] = stabilization_att_fb_cmd[COMMAND_PITCH] + stabilization_att_ff_cmd[COMMAND_PITCH];
//		stabilization_cmd[COMMAND_YAW] = stabilization_att_fb_cmd[COMMAND_YAW] + stabilization_att_ff_cmd[COMMAND_YAW];
//
//		/* bound the result */
//		BoundAbs(stabilization_cmd[COMMAND_ROLL], MAX_PPRZ);
//		BoundAbs(stabilization_cmd[COMMAND_PITCH], MAX_PPRZ);
//		BoundAbs(stabilization_cmd[COMMAND_YAW], MAX_PPRZ);
//	}
//
////	public static void stabilization_attitude_read_rc(boolean in_flight, boolean in_carefree, boolean coordinated_turn) {
////		FloatQuat q_sp;
////		if( USE_EARTH_BOUND_RC_SETPOINT)
////			stabilization_attitude_read_rc_setpoint_quat_earth_bound_f(q_sp, in_flight, in_carefree, coordinated_turn);
////		else
////			stabilization_attitude_read_rc_setpoint_quat_f(q_sp, in_flight, in_carefree, coordinated_turn);
////
////		QUAT_BFP_OF_REAL(stab_att_sp_quat, q_sp);
////	}
//}
