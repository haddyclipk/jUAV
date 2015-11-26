//package sw.airborne.firmwares.rotorcraft.stabilization;
//
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
//public class Stabilization_attitude_ref_quat_float {
//
//	public static final  int STABILIZATION_ATTITUDE_GAIN_NB =1;
//	public static final  int STABILIZATION_ATTITUDE_GAIN_IDX_DEFAULT =0;
//
//	public static void stabilization_attitude_ref_quat_float_SetOmegaP(int _val) { 
//		stabilization_attitude_ref_set_omega_p(_val);               
//	}
//	public static void stabilization_attitude_ref_quat_float_SetOmegaQ(int _val) { 
//		stabilization_attitude_ref_set_omega_q(_val);               
//	}
//	public static void stabilization_attitude_ref_quat_float_SetOmegaR(int _val) { 
//		stabilization_attitude_ref_set_omega_r(_val);               
//	}
//
//	//=============================stabilization_attitude_ref_quat_float.c----------------
//
//	public static final  double REF_ACCEL_MAX_P =STABILIZATION_ATTITUDE_REF_MAX_PDOT;
//	public static final  double REF_ACCEL_MAX_Q =STABILIZATION_ATTITUDE_REF_MAX_QDOT;
//	public static final  double REF_ACCEL_MAX_R =STABILIZATION_ATTITUDE_REF_MAX_RDOT;
//
//	public static final  double REF_RATE_MAX_P =STABILIZATION_ATTITUDE_REF_MAX_P;
//	public static final  double REF_RATE_MAX_Q =STABILIZATION_ATTITUDE_REF_MAX_Q;
//	public static final  double REF_RATE_MAX_R= STABILIZATION_ATTITUDE_REF_MAX_R;
//
//	public static FloatEulers stab_att_sp_euler;
//	public static FloatQuat   stab_att_sp_quat;
//	public static FloatEulers stab_att_ref_euler;
//	public static FloatQuat   stab_att_ref_quat;
//	public static FloatRates  stab_att_ref_rate;
//	public static FloatRates  stab_att_ref_accel;
//
//	FloatRefModel stab_att_ref_model[STABILIZATION_ATTITUDE_GAIN_NB];
//
//	public static int ref_idx = STABILIZATION_ATTITUDE_GAIN_IDX_DEFAULT;
//
//	public static final float omega_p[] = STABILIZATION_ATTITUDE_REF_OMEGA_P;
//	public static final float  zeta_p[] = STABILIZATION_ATTITUDE_REF_ZETA_P;
//	public static final float  omega_q[] = STABILIZATION_ATTITUDE_REF_OMEGA_Q;
//	public static final float  zeta_q[] = STABILIZATION_ATTITUDE_REF_ZETA_Q;
//	public static final float  omega_r[] = STABILIZATION_ATTITUDE_REF_OMEGA_R;
//	public static final float  zeta_r[] = STABILIZATION_ATTITUDE_REF_ZETA_R;
//
//	public static FloatRates two_omega_squared[]= new FloatRates[STABILIZATION_ATTITUDE_GAIN_NB];
//
//	public static void reset_psi_ref_from_body() {
//		//sp has been set from body using stabilization_attitude_get_yaw_f, use that value
//		stab_att_ref_euler.psi = stab_att_sp_euler.psi;
//		stab_att_ref_rate.r = 0;
//		stab_att_ref_accel.r = 0;
//	}
//
//	public static void update_ref_quat_from_eulers() {
//		FloatRMat ref_rmat;
//		FLOAT_RMAT_OF_EULERS(ref_rmat, stab_att_ref_euler);
//		FLOAT_QUAT_OF_RMAT(stab_att_ref_quat, ref_rmat);
//		FLOAT_QUAT_WRAP_SHORTEST(stab_att_ref_quat);
//	}
//
//	public static void stabilization_attitude_ref_idx_set_omega_p(int idx, float omega) {
//		stab_att_ref_model[i].omega.p = omega;
//		two_omega_squared[i].p = 2 * omega * omega;
//	}
//
//	public static void stabilization_attitude_ref_idx_set_omega_q(int idx, float omega) {
//		stab_att_ref_model[i].omega.q = omega;
//		two_omega_squared[i].q = 2 * omega * omega;
//	}
//
//	public static void stabilization_attitude_ref_idx_set_omega_r(int idx, float omega) {
//		stab_att_ref_model[i].omega.r = omega;
//		two_omega_squared[i].r = 2 * omega * omega;
//	}
//
//	public static void stabilization_attitude_ref_set_omega_p(float omega) {
//		stabilization_attitude_ref_idx_set_omega_p(ref_idx, omega);
//	}
//
//	public static void stabilization_attitude_ref_set_omega_q(float omega) {
//		stabilization_attitude_ref_idx_set_omega_q(ref_idx, omega);
//	}
//
//	public static void stabilization_attitude_ref_set_omega_r(float omega) {
//		stabilization_attitude_ref_idx_set_omega_r(ref_idx, omega);
//	}
//
//
//	public static void stabilization_attitude_ref_init() {
//
//		FLOAT_EULERS_ZERO(stab_att_sp_euler);
//		FLOAT_QUAT_ZERO(  stab_att_sp_quat);
//		FLOAT_EULERS_ZERO(stab_att_ref_euler);
//		FLOAT_QUAT_ZERO(  stab_att_ref_quat);
//		FLOAT_RATES_ZERO( stab_att_ref_rate);
//		FLOAT_RATES_ZERO( stab_att_ref_accel);
//
//		for (int i = 0; i < STABILIZATION_ATTITUDE_GAIN_NB; i++) {
//			RATES_ASSIGN(stab_att_ref_model[i].omega, omega_p[i], omega_q[i], omega_r[i]);
//			RATES_ASSIGN(stab_att_ref_model[i].zeta, zeta_p[i], zeta_q[i], zeta_r[i]);
//			RATES_ASSIGN(two_omega_squared[i], 2*omega_p[i]*omega_p[i], 2*omega_q[i]*omega_q[i], 2*omega_r[i]*omega_r[i]);
//		}
//
//	}
//
//	public static void stabilization_attitude_ref_schedule(int idx) {
//		ref_idx = idx;
//	}
//
//	public static void stabilization_attitude_ref_enter() {
//		reset_psi_ref_from_body();
//		update_ref_quat_from_eulers();
//	}
//
//	/*
//	 * Reference
//	 */
//	public static final double DT_UPDATE = (1./PERIODIC_FREQUENCY);
//
//	// default to fast but less precise quaternion integration
//	public static final boolean STABILIZATION_ATTITUDE_REF_QUAT_INFINITESIMAL_STEP = true;
//
//	public static void stabilization_attitude_ref_update() {
//
//		/* integrate reference attitude            */
//		if(STABILIZATION_ATTITUDE_REF_QUAT_INFINITESIMAL_STEP){
//			FloatQuat qdot;
//			FLOAT_QUAT_DERIVATIVE(qdot, stab_att_ref_rate, stab_att_ref_quat);
//			QUAT_SMUL(qdot, qdot, DT_UPDATE);
//			QUAT_ADD(stab_att_ref_quat, qdot);
//		}else {// use finite step (involves trig)
//			FloatQuat delta_q;
//			FLOAT_QUAT_DIFFERENTIAL(delta_q, stab_att_ref_rate, DT_UPDATE);
//			/* compose new ref_quat by quaternion multiplication of delta rotation and current ref_quat */
//			FloatQuat new_ref_quat;
//			FLOAT_QUAT_COMP(new_ref_quat, stab_att_ref_quat, delta_q);
//			QUAT_COPY(stab_att_ref_quat, new_ref_quat);
//		}
//		FLOAT_QUAT_NORMALIZE(stab_att_ref_quat);
//
//		/* integrate reference rotational speeds   */
//		FloatRates delta_rate;
//		RATES_SMUL(delta_rate, stab_att_ref_accel, DT_UPDATE);
//		RATES_ADD(stab_att_ref_rate, delta_rate);
//
//		/* compute reference angular accelerations */
//		FloatQuat err;
//		/* compute reference attitude error        */
//		FLOAT_QUAT_INV_COMP(err, stab_att_sp_quat, stab_att_ref_quat);
//		/* wrap it in the shortest direction       */
//		FLOAT_QUAT_WRAP_SHORTEST(err);
//		/* propagate the 2nd order linear model: xdotdot = -2*zeta*omega*xdot - omega^2*x  */
//		/* since error quaternion contains the half-angles we get 2*omega^2*err */
//		stab_att_ref_accel.p = -2.*stab_att_ref_model[ref_idx].zeta.p*stab_att_ref_model[ref_idx].omega.p*stab_att_ref_rate.p
//				- two_omega_squared[ref_idx].p * err.qx;
//		stab_att_ref_accel.q = -2.*stab_att_ref_model[ref_idx].zeta.q*stab_att_ref_model[ref_idx].omega.q*stab_att_ref_rate.q
//				- two_omega_squared[ref_idx].q * err.qy;
//		stab_att_ref_accel.r = -2.*stab_att_ref_model[ref_idx].zeta.r*stab_att_ref_model[ref_idx].omega.r*stab_att_ref_rate.r
//				- two_omega_squared[ref_idx].r * err.qz;
//
//		/*	saturate acceleration */
//		
//		//TODO
//		FloatRates MIN_ACCEL = { -REF_ACCEL_MAX_P, -REF_ACCEL_MAX_Q, -REF_ACCEL_MAX_R };
//		FloatRates MAX_ACCEL = {  REF_ACCEL_MAX_P,  REF_ACCEL_MAX_Q,  REF_ACCEL_MAX_R };
//		RATES_BOUND_BOX(stab_att_ref_accel, MIN_ACCEL, MAX_ACCEL);
//
//		/* saturate angular speed and trim accel accordingly */
//		SATURATE_SPEED_TRIM_ACCEL();
//
//		/* compute ref_euler */
//		FLOAT_EULERS_OF_QUAT(stab_att_ref_euler, stab_att_ref_quat);
//	}
//
//
//}
