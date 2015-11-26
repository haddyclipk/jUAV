package sw.airborne.firmwares.rotorcraft.stabilization;

import static sw.airborne.math.Pprz_trig_int.*;
import static sw.airborne.math.Pprz_algebra.*;
import static sw.airborne.firmwares.rotorcraft.Stabilization.*;
import static sw.include.Std.*;

import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_rc_setpoint.*;
import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_ref_euler_int.*;
import static sw.airborne.math.Pprz_algebra_int.*;
import static sw.airborne.State.*;
import sw.airborne.math.*;
import  sw.airborne.math.Int32Quat;
import sw.airborne.math.Int32Vect2;
//import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_quat_transformations.*;
import sw.include.Std;
import static sw.airborne.math.Pprz_geodetic.*;
import static sw.airborne.math.Pprz_orientation_conversion.*;
import static sw.airborne.math.Pprz_geodetic_int.*;
import static sw.airborne.math.Pprz_geodetic_float.*;
import static sw.airborne.math.Pprz_algebra_float.*;
import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_ref_int.*;
import static sw.airborne.Paparazzi.*;

public class Stabilization_attitude_euler_int {
//
	public static Int32AttitudeGains  stabilization_gains;
	public static Int32Eulers stabilization_att_sum_err;
//
	public static long stabilization_att_fb_cmd[] = new long[COMMANDS_NB];
	public static long stabilization_att_ff_cmd[] = new long[COMMANDS_NB];
//
	public static void reset_psi_ref_from_body() {
		//sp has been set from body using stabilization_attitude_get_yaw_i, use that value
		stab_att_ref_euler.psi = stab_att_sp_euler.psi << (REF_ANGLE_FRAC - INT32_ANGLE_FRAC);
		stab_att_ref_rate.r = 0;
		stab_att_ref_accel.r = 0;
	}
//
//	
//
//	public static void send_att() {
//		Int32Rates body_rate = stateGetBodyRates_i();
//		Int32Eulers att = stateGetNedToBodyEulers_i();
//		DOWNLINK_SEND_STAB_ATTITUDE_INT(DefaultChannel, DefaultDevice,
//				(body_rate.p), (body_rate.q), (body_rate.r),
//				(att.phi), (att.theta), (att.psi),
//				stab_att_sp_euler.phi,
//				stab_att_sp_euler.theta,
//				stab_att_sp_euler.psi,
//				stabilization_att_sum_err.phi,
//				stabilization_att_sum_err.theta,
//				stabilization_att_sum_err.psi,
//				stabilization_att_fb_cmd[COMMAND_ROLL],
//				stabilization_att_fb_cmd[COMMAND_PITCH],
//				stabilization_att_fb_cmd[COMMAND_YAW],
//				stabilization_att_ff_cmd[COMMAND_ROLL],
//				stabilization_att_ff_cmd[COMMAND_PITCH],
//				stabilization_att_ff_cmd[COMMAND_YAW],
//				stabilization_cmd[COMMAND_ROLL],
//				stabilization_cmd[COMMAND_PITCH],
//				stabilization_cmd[COMMAND_YAW]);
//	}
//
//	public static void send_att_ref() {
//		DOWNLINK_SEND_STAB_ATTITUDE_REF_INT(DefaultChannel, DefaultDevice,
//				stab_att_sp_euler.phi,
//				stab_att_sp_euler.theta,
//				stab_att_sp_euler.psi,
//				stab_att_ref_euler.phi,
//				stab_att_ref_euler.theta,
//				stab_att_ref_euler.psi,
//				stab_att_ref_rate.p,
//				stab_att_ref_rate.q,
//				stab_att_ref_rate.r,
//				stab_att_ref_accel.p,
//				stab_att_ref_accel.q,
//				stab_att_ref_accel.r);
//	}
//	
//

	public static void stabilization_attitude_init() {

		stabilization_attitude_ref_init();


		VECT3_ASSIGN(stabilization_gains.p,
				STABILIZATION_ATTITUDE_PHI_PGAIN,
				STABILIZATION_ATTITUDE_THETA_PGAIN,
				STABILIZATION_ATTITUDE_PSI_PGAIN);

		VECT3_ASSIGN(stabilization_gains.d,
				STABILIZATION_ATTITUDE_PHI_DGAIN,
				STABILIZATION_ATTITUDE_THETA_DGAIN,
				STABILIZATION_ATTITUDE_PSI_DGAIN);

		VECT3_ASSIGN(stabilization_gains.i,
				STABILIZATION_ATTITUDE_PHI_IGAIN,
				STABILIZATION_ATTITUDE_THETA_IGAIN,
				STABILIZATION_ATTITUDE_PSI_IGAIN);

		VECT3_ASSIGN(stabilization_gains.dd,
				STABILIZATION_ATTITUDE_PHI_DDGAIN,
				STABILIZATION_ATTITUDE_THETA_DDGAIN,
				STABILIZATION_ATTITUDE_PSI_DDGAIN);


		INT_EULERS_ZERO( stabilization_att_sum_err );

//		if(PERIODIC_TELEMETRY){
//			register_periodic_telemetry(DefaultPeriodic, "STAB_ATTITUDE", send_att);
//			register_periodic_telemetry(DefaultPeriodic, "STAB_ATTITUDE_REF", send_att_ref);
//		}
	}
//
//	public static void stabilization_attitude_read_rc(boolean in_flight, boolean in_carefree, boolean coordinated_turn) {
//		stabilization_attitude_read_rc_setpoint_eulers(stab_att_sp_euler, in_flight, in_carefree, coordinated_turn);
//	}
//
	public static void stabilization_attitude_enter() {
		stab_att_sp_euler.psi = stateGetNedToBodyEulers_i().psi;
		reset_psi_ref_from_body();
		INT_EULERS_ZERO( stabilization_att_sum_err );
	}
//
	public static void stabilization_attitude_set_failsafe_setpoint() {
		stab_att_sp_euler.phi = 0;
		stab_att_sp_euler.theta = 0;
		stab_att_sp_euler.psi = stateGetNedToBodyEulers_i().psi;
	}
//
	public static void stabilization_attitude_set_rpy_setpoint_i( Int32Eulers rpy) {
		//memcpy(stab_att_sp_euler, rpy, sizeof( Int32Eulers));
		stab_att_sp_euler = rpy.clone();
	}
//
	public static void stabilization_attitude_set_earth_cmd_i( Int32Vect2 cmd, long heading) {
		/* Rotate horizontal commands to body frame by psi */
		long psi = stateGetNedToBodyEulers_i().psi;
		long s_psi, c_psi;
		s_psi = PPRZ_ITRIG_SIN( psi);
		c_psi = PPRZ_ITRIG_COS( psi);
		stab_att_sp_euler.phi = (-s_psi * cmd.x + c_psi * cmd.y) >> INT32_TRIG_FRAC;
		stab_att_sp_euler.theta = -(c_psi * cmd.x + s_psi * cmd.y) >> INT32_TRIG_FRAC;
		stab_att_sp_euler.psi = heading;
	}
//
	//#define OFFSET_AND_ROUND(_a, _b) (((_a)+(1<<((_b)-1)))>>(_b))
	public static int OFFSET_AND_ROUND(int _a, int _b){
		return (((_a)+(1<<((_b)-1)))>>(_b));
	}
	public static long OFFSET_AND_ROUND(long _a, int _b){
		return (((_a)+(1<<((_b)-1)))>>(_b));
	}
	
	//#define OFFSET_AND_ROUND2(_a, _b) (((_a)+(1<<((_b)-1))-((_a)<0?1:0))>>(_b))
	public static int OFFSET_AND_ROUND2(int _a, int _b){
		return  (((_a)+(1<<((_b)-1))-((_a)<0?1:0))>>(_b));
	}
	public static long OFFSET_AND_ROUND2(long _a, int _b){
		return  (((_a)+(1<<((_b)-1))-((_a)<0?1:0))>>(_b));
	}
//	
	//#define MAX_SUM_ERR 4000000
	public static final int MAX_SUM_ERR = 4000000;
	
	public static final int  CMD_SHIFT = 11;
	
	public static void stabilization_attitude_run(boolean  in_flight) {

		/* update reference */
		stabilization_attitude_ref_update();

		/* compute feedforward command */
		stabilization_att_ff_cmd[COMMAND_ROLL] =
				OFFSET_AND_ROUND(stabilization_gains.dd.x * stab_att_ref_accel.p, 5);
		stabilization_att_ff_cmd[COMMAND_PITCH] =
				OFFSET_AND_ROUND(stabilization_gains.dd.y * stab_att_ref_accel.q, 5);
		stabilization_att_ff_cmd[COMMAND_YAW] =
				OFFSET_AND_ROUND(stabilization_gains.dd.z * stab_att_ref_accel.r, 5);

		/* compute feedback command */
		/* attitude error            */
		Int32Eulers att_ref_scaled = new Int32Eulers();
			
				att_ref_scaled.phi=			OFFSET_AND_ROUND(stab_att_ref_euler.phi,   (REF_ANGLE_FRAC - INT32_ANGLE_FRAC));
				att_ref_scaled.theta=		OFFSET_AND_ROUND(stab_att_ref_euler.theta, (REF_ANGLE_FRAC - INT32_ANGLE_FRAC));
				att_ref_scaled.psi=		OFFSET_AND_ROUND(stab_att_ref_euler.psi,   (REF_ANGLE_FRAC - INT32_ANGLE_FRAC)) ;
		Int32Eulers att_err= new Int32Eulers();;
		Int32Eulers ltp_to_body_euler = stateGetNedToBodyEulers_i();
		EULERS_DIFF(att_err, att_ref_scaled, (ltp_to_body_euler));
		INT32_ANGLE_NORMALIZE(att_err.psi);

		if (in_flight) {
			/* update integrator */
			EULERS_ADD(stabilization_att_sum_err, att_err);
			EULERS_BOUND_CUBE(stabilization_att_sum_err, -MAX_SUM_ERR, MAX_SUM_ERR);
		}
		else {
			INT_EULERS_ZERO(stabilization_att_sum_err);
		}

		/* rate error                */
		Int32Rates rate_ref_scaled = new Int32Rates();
//			{
					rate_ref_scaled.p=	OFFSET_AND_ROUND(stab_att_ref_rate.p, (REF_RATE_FRAC - INT32_RATE_FRAC));
					rate_ref_scaled.q=	OFFSET_AND_ROUND(stab_att_ref_rate.q, (REF_RATE_FRAC - INT32_RATE_FRAC));
					rate_ref_scaled.r=	OFFSET_AND_ROUND(stab_att_ref_rate.r, (REF_RATE_FRAC - INT32_RATE_FRAC));
		Int32Rates rate_err = new Int32Rates();
		Int32Rates body_rate = stateGetBodyRates_i();
		RATES_DIFF(rate_err, rate_ref_scaled, (body_rate));

		/* PID                  */
		stabilization_att_fb_cmd[COMMAND_ROLL] =
				stabilization_gains.p.x    * att_err.phi +
				stabilization_gains.d.x    * rate_err.p +
				OFFSET_AND_ROUND2((stabilization_gains.i.x  * stabilization_att_sum_err.phi), 10);

		stabilization_att_fb_cmd[COMMAND_PITCH] =
				stabilization_gains.p.y    * att_err.theta +
				stabilization_gains.d.y    * rate_err.q +
				OFFSET_AND_ROUND2((stabilization_gains.i.y  * stabilization_att_sum_err.theta), 10);

		stabilization_att_fb_cmd[COMMAND_YAW] =
				stabilization_gains.p.z    * att_err.psi +
				stabilization_gains.d.z    * rate_err.r +
				OFFSET_AND_ROUND2((stabilization_gains.i.z  * stabilization_att_sum_err.psi), 10);


		/* with P gain of 100, att_err of 180deg (3.14 rad)
		 * fb cmd: 100 * 3.14 * 2^12 / 2^CMD_SHIFT = 628
		 * max possible command is 9600
		 */
		

		/* sum feedforward and feedback */
		stabilization_cmd[COMMAND_ROLL] =
		OFFSET_AND_ROUND((stabilization_att_fb_cmd[COMMAND_ROLL]+stabilization_att_ff_cmd[COMMAND_ROLL]), CMD_SHIFT);

		stabilization_cmd[COMMAND_PITCH] =
				OFFSET_AND_ROUND((stabilization_att_fb_cmd[COMMAND_PITCH]+stabilization_att_ff_cmd[COMMAND_PITCH]), CMD_SHIFT);

		stabilization_cmd[COMMAND_YAW] =
				OFFSET_AND_ROUND((stabilization_att_fb_cmd[COMMAND_YAW]+stabilization_att_ff_cmd[COMMAND_YAW]), CMD_SHIFT);

		/* bound the result */
		BoundAbs(stabilization_cmd[COMMAND_ROLL], MAX_PPRZ);
		BoundAbs(stabilization_cmd[COMMAND_PITCH], MAX_PPRZ);
		BoundAbs(stabilization_cmd[COMMAND_YAW], MAX_PPRZ);

	}
}
