<<<<<<< HEAD
package sw.airborne.firmwares.rotorcraft.stabilization;
import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_ref_euler_float.*;
import sw.airborne.math.FloatEulers;
import sw.airborne.math.Int32Eulers;
import static sw.airborne.State.*;
import static sw.airborne.math.Pprz_algebra_int.*;
import static sw.airborne.math.Pprz_algebra.*;

public //static Int32Eulers  stab_att_sp_euler;
public class Stabilization_attitude_euler_float {
	//static{ Int32Eulers stab_att_sp_euler; }

	public static FloatAttitudeGains stabilization_gains;
	public static FloatEulers stabilization_att_sum_err;

	public static float stabilization_att_fb_cmd[] = new float[COMMANDS_NB];
	public static float stabilization_att_ff_cmd[] = new float[COMMANDS_NB];

	public static void send_att() {
		FloatRates body_rate = stateGetBodyRates_f();
		FloatEulers att = stateGetNedToBodyEulers_f();
		float foo = 0.0;
		DOWNLINK_SEND_STAB_ATTITUDE_FLOAT(DefaultChannel, DefaultDevice,
				(body_rate.p), (body_rate.q), (body_rate.r),
				(att.phi), (att.theta), (att.psi),
				stab_att_sp_euler.phi,
				stab_att_sp_euler.theta,
				stab_att_sp_euler.psi,
				stabilization_att_sum_err.phi,
				stabilization_att_sum_err.theta,
				stabilization_att_sum_err.psi,
				stabilization_att_fb_cmd[COMMAND_ROLL],
				stabilization_att_fb_cmd[COMMAND_PITCH],
				stabilization_att_fb_cmd[COMMAND_YAW],
				stabilization_att_ff_cmd[COMMAND_ROLL],
				stabilization_att_ff_cmd[COMMAND_PITCH],
				stabilization_att_ff_cmd[COMMAND_YAW],
				stabilization_cmd[COMMAND_ROLL],
				stabilization_cmd[COMMAND_PITCH],
				stabilization_cmd[COMMAND_YAW],
				foo, foo, foo);
	}

	public static void send_att_ref() {
		DOWNLINK_SEND_STAB_ATTITUDE_REF_FLOAT(DefaultChannel, DefaultDevice,
				stab_att_sp_euler.phi,
				stab_att_sp_euler.theta,
				stab_att_sp_euler.psi,
				stab_att_ref_euler.phi,
				stab_att_ref_euler.theta,
				stab_att_ref_euler.psi,
				stab_att_ref_rate.p,
				stab_att_ref_rate.q,
				stab_att_ref_rate.r,
				stab_att_ref_accel.p,
				stab_att_ref_accel.q,
				stab_att_ref_accel.r);
	}
	

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

		FLOAT_EULERS_ZERO( stabilization_att_sum_err );

		if( PERIODIC_TELEMETRY){
			register_periodic_telemetry(DefaultPeriodic, "STAB_ATTITUDE", send_att);
			register_periodic_telemetry(DefaultPeriodic, "STAB_ATTITUDE_REF", send_att_ref);
		}
	}

	public static void stabilization_attitude_read_rc(boolean in_flight, boolean in_carefree, boolean coordinated_turn) {
		stabilization_attitude_read_rc_setpoint_eulers_f(stab_att_sp_euler, in_flight, in_carefree, coordinated_turn);
	}

	public static void stabilization_attitude_enter() {

		/* reset psi setpoint to current psi angle */
		stab_att_sp_euler.psi = stabilization_attitude_get_heading_f();

		stabilization_attitude_ref_enter();

		FLOAT_EULERS_ZERO(stabilization_att_sum_err);
	}

	

	public static void stabilization_attitude_set_failsafe_setpoint() {
		stab_att_sp_euler.phi = (float) 0.0;
		stab_att_sp_euler.theta = (float) 0.0;
		stab_att_sp_euler.psi = stateGetNedToBodyEulers_f().psi;
	}

	public static void stabilization_attitude_set_rpy_setpoint_i( Int32Eulers rpy) {
		EULERS_FLOAT_OF_BFP(stab_att_sp_euler, *rpy);
	}

	public static void stabilization_attitude_set_earth_cmd_i( Int32Vect2 cmd, int heading) {
		FloatVect2 cmd_f;
		cmd_f.x = ANGLE_FLOAT_OF_BFP(cmd.x);
		cmd_f.y = ANGLE_FLOAT_OF_BFP(cmd.y);

		/* Rotate horizontal commands to body frame by psi */
		float psi = stateGetNedToBodyEulers_f().psi;
		float s_psi = sinf(psi);
		float c_psi = cosf(psi);
		stab_att_sp_euler.phi = -s_psi * cmd_f.x + c_psi * cmd_f.y;
		stab_att_sp_euler.theta = -c_psi * cmd_f.x - s_psi * cmd_f.y;
		stab_att_sp_euler.psi = ANGLE_FLOAT_OF_BFP(heading);
	}

	public static final int MAX_SUM_ERR = 200;

	public static void stabilization_attitude_run(boolean  in_flight) {

		stabilization_attitude_ref_update();

		/* Compute feedforward */
		stabilization_att_ff_cmd[COMMAND_ROLL] =
				stabilization_gains.dd.x * stab_att_ref_accel.p;
		stabilization_att_ff_cmd[COMMAND_PITCH] =
				stabilization_gains.dd.y * stab_att_ref_accel.q;
		stabilization_att_ff_cmd[COMMAND_YAW] =
				stabilization_gains.dd.z * stab_att_ref_accel.r;

		/* Compute feedback                  */
		/* attitude error            */
		FloatEulers att_float = stateGetNedToBodyEulers_f();
		FloatEulers att_err;
		EULERS_DIFF(att_err, stab_att_ref_euler, att_float);
		FLOAT_ANGLE_NORMALIZE(att_err.psi);

		if (in_flight) {
			/* update integrator */
			EULERS_ADD(stabilization_att_sum_err, att_err);
			EULERS_BOUND_CUBE(stabilization_att_sum_err, -MAX_SUM_ERR, MAX_SUM_ERR);
		}
		else {
			FLOAT_EULERS_ZERO(stabilization_att_sum_err);
		}

		/*  rate error                */
		FloatRates rate_float = stateGetBodyRates_f();
		FloatRates rate_err;
		RATES_DIFF(rate_err, stab_att_ref_rate, rate_float);

		/*  PID                  */

		stabilization_att_fb_cmd[COMMAND_ROLL] =
				stabilization_gains.p.x  * att_err.phi +
				stabilization_gains.d.x  * rate_err.p +
				stabilization_gains.i.x  * stabilization_att_sum_err.phi;

		stabilization_att_fb_cmd[COMMAND_PITCH] =
				stabilization_gains.p.y  * att_err.theta +
				stabilization_gains.d.y  * rate_err.q +
				stabilization_gains.i.y  * stabilization_att_sum_err.theta;

		stabilization_att_fb_cmd[COMMAND_YAW] =
				stabilization_gains.p.z  * att_err.psi +
				stabilization_gains.d.z  * rate_err.r +
				stabilization_gains.i.z  * stabilization_att_sum_err.psi;


		stabilization_cmd[COMMAND_ROLL] =
				(stabilization_att_fb_cmd[COMMAND_ROLL]+stabilization_att_ff_cmd[COMMAND_ROLL]);
		stabilization_cmd[COMMAND_PITCH] =
				(stabilization_att_fb_cmd[COMMAND_PITCH]+stabilization_att_ff_cmd[COMMAND_PITCH]);
		stabilization_cmd[COMMAND_YAW] =
				(stabilization_att_fb_cmd[COMMAND_YAW]+stabilization_att_ff_cmd[COMMAND_YAW]);

		/* bound the result */
		BoundAbs(stabilization_cmd[COMMAND_ROLL], MAX_PPRZ);
		BoundAbs(stabilization_cmd[COMMAND_PITCH], MAX_PPRZ);
		BoundAbs(stabilization_cmd[COMMAND_YAW], MAX_PPRZ);
	}
}
=======
//package sw.airborne.firmwares.rotorcraft.stabilization;
//
//import static sw.airborne.math.Pprz_trig_int.*;
//import static sw.airborne.math.Pprz_algebra.*;
//import static sw.airborne.firmwares.rotorcraft.Stabilization.*;
//import static sw.include.Std.*;
//
//import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_rc_setpoint.*;
//import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_ref_quat_int.*;
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
//public class Stabilization_attitude_euler_float {
//
//	public static FloatAttitudeGains stabilization_gains;
//	public static FloatEulers stabilization_att_sum_err;
//
//	public static float stabilization_att_fb_cmd[] = new float[COMMANDS_NB];
//	public static float stabilization_att_ff_cmd[] = new float[COMMANDS_NB];
//
//	public static void send_att() {
//		FloatRates body_rate = stateGetBodyRates_f();
//		FloatEulers att = stateGetNedToBodyEulers_f();
//		float foo = 0.0;
//		DOWNLINK_SEND_STAB_ATTITUDE_FLOAT(DefaultChannel, DefaultDevice,
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
//				stabilization_cmd[COMMAND_YAW],
//				foo, foo, foo);
//	}
//
//	public static void send_att_ref() {
//		DOWNLINK_SEND_STAB_ATTITUDE_REF_FLOAT(DefaultChannel, DefaultDevice,
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
//	public static void stabilization_attitude_init() {
//
//		stabilization_attitude_ref_init();
//
//		VECT3_ASSIGN(stabilization_gains.p,
//				STABILIZATION_ATTITUDE_PHI_PGAIN,
//				STABILIZATION_ATTITUDE_THETA_PGAIN,
//				STABILIZATION_ATTITUDE_PSI_PGAIN);
//
//		VECT3_ASSIGN(stabilization_gains.d,
//				STABILIZATION_ATTITUDE_PHI_DGAIN,
//				STABILIZATION_ATTITUDE_THETA_DGAIN,
//				STABILIZATION_ATTITUDE_PSI_DGAIN);
//
//		VECT3_ASSIGN(stabilization_gains.i,
//				STABILIZATION_ATTITUDE_PHI_IGAIN,
//				STABILIZATION_ATTITUDE_THETA_IGAIN,
//				STABILIZATION_ATTITUDE_PSI_IGAIN);
//
//		VECT3_ASSIGN(stabilization_gains.dd,
//				STABILIZATION_ATTITUDE_PHI_DDGAIN,
//				STABILIZATION_ATTITUDE_THETA_DDGAIN,
//				STABILIZATION_ATTITUDE_PSI_DDGAIN);
//
//		FLOAT_EULERS_ZERO( stabilization_att_sum_err );
//
//		if( PERIODIC_TELEMETRY){
//			register_periodic_telemetry(DefaultPeriodic, "STAB_ATTITUDE", send_att);
//			register_periodic_telemetry(DefaultPeriodic, "STAB_ATTITUDE_REF", send_att_ref);
//		}
//	}
//
//	public static void stabilization_attitude_read_rc(boolean in_flight, boolean in_carefree, boolean coordinated_turn) {
//		stabilization_attitude_read_rc_setpoint_eulers_f(stab_att_sp_euler, in_flight, in_carefree, coordinated_turn);
//	}
//
//	public static void stabilization_attitude_enter() {
//
//		/* reset psi setpoint to current psi angle */
//		stab_att_sp_euler.psi = stabilization_attitude_get_heading_f();
//
//		stabilization_attitude_ref_enter();
//
//		FLOAT_EULERS_ZERO(stabilization_att_sum_err);
//	}
//
//	public static void stabilization_attitude_set_failsafe_setpoint() {
//		stab_att_sp_euler.phi = (long) 0.0;
//		stab_att_sp_euler.theta =(long)  0.0;
//		stab_att_sp_euler.psi = (long)  stateGetNedToBodyEulers_f().psi;
//	}
//
//	public static void stabilization_attitude_set_rpy_setpoint_i( Int32Eulers rpy) {
//		EULERS_FLOAT_OF_BFP(stab_att_sp_euler, *rpy);
//	}
//
//	public static void stabilization_attitude_set_earth_cmd_i( Int32Vect2 cmd, int heading) {
//		FloatVect2 cmd_f;
//		cmd_f.x = ANGLE_FLOAT_OF_BFP(cmd.x);
//		cmd_f.y = ANGLE_FLOAT_OF_BFP(cmd.y);
//
//		/* Rotate horizontal commands to body frame by psi */
//		float psi = stateGetNedToBodyEulers_f().psi;
//		float s_psi = sinf(psi);
//		float c_psi = cosf(psi);
//		stab_att_sp_euler.phi = -s_psi * cmd_f.x + c_psi * cmd_f.y;
//		stab_att_sp_euler.theta = -c_psi * cmd_f.x - s_psi * cmd_f.y;
//		stab_att_sp_euler.psi = ANGLE_FLOAT_OF_BFP(heading);
//	}
//
//	public static final int MAX_SUM_ERR = 200;
//
//	public static void stabilization_attitude_run(boolean  in_flight) {
//
//		stabilization_attitude_ref_update();
//
//		/* Compute feedforward */
//		stabilization_att_ff_cmd[COMMAND_ROLL] =
//				stabilization_gains.dd.x * stab_att_ref_accel.p;
//		stabilization_att_ff_cmd[COMMAND_PITCH] =
//				stabilization_gains.dd.y * stab_att_ref_accel.q;
//		stabilization_att_ff_cmd[COMMAND_YAW] =
//				stabilization_gains.dd.z * stab_att_ref_accel.r;
//
//		/* Compute feedback                  */
//		/* attitude error            */
//		FloatEulers att_float = stateGetNedToBodyEulers_f();
//		FloatEulers att_err;
//		EULERS_DIFF(att_err, stab_att_ref_euler, att_float);
//		FLOAT_ANGLE_NORMALIZE(att_err.psi);
//
//		if (in_flight) {
//			/* update integrator */
//			EULERS_ADD(stabilization_att_sum_err, att_err);
//			EULERS_BOUND_CUBE(stabilization_att_sum_err, -MAX_SUM_ERR, MAX_SUM_ERR);
//		}
//		else {
//			FLOAT_EULERS_ZERO(stabilization_att_sum_err);
//		}
//
//		/*  rate error                */
//		FloatRates rate_float = stateGetBodyRates_f();
//		FloatRates rate_err;
//		RATES_DIFF(rate_err, stab_att_ref_rate, rate_float);
//
//		/*  PID                  */
//
//		stabilization_att_fb_cmd[COMMAND_ROLL] =
//				stabilization_gains.p.x  * att_err.phi +
//				stabilization_gains.d.x  * rate_err.p +
//				stabilization_gains.i.x  * stabilization_att_sum_err.phi;
//
//		stabilization_att_fb_cmd[COMMAND_PITCH] =
//				stabilization_gains.p.y  * att_err.theta +
//				stabilization_gains.d.y  * rate_err.q +
//				stabilization_gains.i.y  * stabilization_att_sum_err.theta;
//
//		stabilization_att_fb_cmd[COMMAND_YAW] =
//				stabilization_gains.p.z  * att_err.psi +
//				stabilization_gains.d.z  * rate_err.r +
//				stabilization_gains.i.z  * stabilization_att_sum_err.psi;
//
//
//		stabilization_cmd[COMMAND_ROLL] =
//				(stabilization_att_fb_cmd[COMMAND_ROLL]+stabilization_att_ff_cmd[COMMAND_ROLL]);
//		stabilization_cmd[COMMAND_PITCH] =
//				(stabilization_att_fb_cmd[COMMAND_PITCH]+stabilization_att_ff_cmd[COMMAND_PITCH]);
//		stabilization_cmd[COMMAND_YAW] =
//				(stabilization_att_fb_cmd[COMMAND_YAW]+stabilization_att_ff_cmd[COMMAND_YAW]);
//
//		/* bound the result */
//		BoundAbs(stabilization_cmd[COMMAND_ROLL], MAX_PPRZ);
//		BoundAbs(stabilization_cmd[COMMAND_PITCH], MAX_PPRZ);
//		BoundAbs(stabilization_cmd[COMMAND_YAW], MAX_PPRZ);
//	}
//}
>>>>>>> e34036e32d4a5ccf069cfe711e9b8f7d04437f12
