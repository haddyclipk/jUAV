<<<<<<< HEAD
package sw.airborne.firmwares.rotorcraft.stabilization;
import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_ref_int.*;
import sw.airborne.math.*;
import static sw.airborne.math.Pprz_algebra_int.*;

public class Stabilization_attitude_ref_quat_int {
	public static void stabilization_attitude_ref_quat_int_SetOmegaP(int _val) { 
		stabilization_attitude_ref_set_omega_p(_val);             
	}

	public static void stabilization_attitude_ref_quat_int_SetOmegaQ(int _val) {   
		stabilization_attitude_ref_set_omega_q(_val);               
	}
	public static void stabilization_attitude_ref_quat_int_SetOmegaR(int _val) {   
		stabilization_attitude_ref_set_omega_r(_val);               
	}

	public static void stabilization_attitude_ref_quat_int_SetZetaP(int _val) {   
		stabilization_attitude_ref_set_zeta_p(_val);               
	}
	public static void stabilization_attitude_ref_quat_int_SetZetaQ(int _val) {    
		stabilization_attitude_ref_set_zeta_q(_val);                
	}
	public static void stabilization_attitude_ref_quat_int_SetZetaR(int _val) {    
		stabilization_attitude_ref_set_zeta_r(_val);                
	}
	
	//===================stabilization_attitude_ref_quat_int.c====================================
	
	public static final int REF_ACCEL_MAX_P= BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_PDOT, REF_ACCEL_FRAC);
	public static final int REF_ACCEL_MAX_Q= BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_QDOT, REF_ACCEL_FRAC);
	public static final int REF_ACCEL_MAX_R= BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_RDOT, REF_ACCEL_FRAC);
	public static final int REF_RATE_MAX_P =BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_P, REF_RATE_FRAC);
	public static final int REF_RATE_MAX_Q =BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_Q, REF_RATE_FRAC);
	public static final int REF_RATE_MAX_R =BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_R, REF_RATE_FRAC);


	 public static Int32Eulers stab_att_sp_euler;
	public static Int32Quat   stab_att_sp_quat;
	public static  Int32Eulers stab_att_ref_euler;
	public static  Int32Quat   stab_att_ref_quat;
	public static  Int32Rates  stab_att_ref_rate;
	public static  Int32Rates  stab_att_ref_accel;
//TODO
	public static FloatRefModel stab_att_ref_model = {
	  {STABILIZATION_ATTITUDE_REF_OMEGA_P, STABILIZATION_ATTITUDE_REF_OMEGA_Q, STABILIZATION_ATTITUDE_REF_OMEGA_R},
	  {STABILIZATION_ATTITUDE_REF_ZETA_P, STABILIZATION_ATTITUDE_REF_ZETA_Q, STABILIZATION_ATTITUDE_REF_ZETA_R}
	};

	public static final int TWO_ZETA_OMEGA_RES =10;
	public static final int TWO_OMEGA_2_RES= 7;
	public static  Int32Rates two_zeta_omega;
	public static  Int32Rates two_omega_2;

	public static void update_ref_model_p() {
	  two_zeta_omega.p = BFP_OF_REAL((2*stab_att_ref_model.zeta.p * stab_att_ref_model.omega.p), TWO_ZETA_OMEGA_RES);
	  two_omega_2.p = BFP_OF_REAL((2*stab_att_ref_model.omega.p * stab_att_ref_model.omega.p), TWO_OMEGA_2_RES);
	}

	public static void update_ref_model_q() {
	  two_zeta_omega.q = BFP_OF_REAL((2*stab_att_ref_model.zeta.q * stab_att_ref_model.omega.q), TWO_ZETA_OMEGA_RES);
	  two_omega_2.q = BFP_OF_REAL((2*stab_att_ref_model.omega.q * stab_att_ref_model.omega.q), TWO_OMEGA_2_RES);
	}

	public static void update_ref_model_r() {
	  two_zeta_omega.r = BFP_OF_REAL((2*stab_att_ref_model.zeta.r * stab_att_ref_model.omega.r), TWO_ZETA_OMEGA_RES);
	  two_omega_2.r = BFP_OF_REAL((2*stab_att_ref_model.omega.r * stab_att_ref_model.omega.r), TWO_OMEGA_2_RES);
	}

	public static void update_ref_model() {
	  update_ref_model_p();
	  update_ref_model_q();
	  update_ref_model_r();
	}


	public static void stabilization_attitude_ref_set_omega_p(float omega_p) {
	  stab_att_ref_model.omega.p = omega_p;
	  update_ref_model_p();
	}

	public static void stabilization_attitude_ref_set_omega_q(float omega_q) {
	  stab_att_ref_model.omega.q = omega_q;
	  update_ref_model_q();
	}

	public static void stabilization_attitude_ref_set_omega_r(float omega_r) {
	  stab_att_ref_model.omega.r = omega_r;
	  update_ref_model_r();
	}

	public static void stabilization_attitude_ref_set_omega( FloatRates omega) {
	  stabilization_attitude_ref_set_omega_p(omega.p);
	  stabilization_attitude_ref_set_omega_q(omega.q);
	  stabilization_attitude_ref_set_omega_r(omega.r);
	}

	public static void stabilization_attitude_ref_set_zeta_p(float zeta_p) {
	  stab_att_ref_model.zeta.p = zeta_p;
	  update_ref_model_p();
	}

	public static void stabilization_attitude_ref_set_zeta_q(float zeta_q)  {
	  stab_att_ref_model.zeta.q = zeta_q;
	  update_ref_model_q();
	}

	public static void stabilization_attitude_ref_set_zeta_r(float zeta_r) {
	  stab_att_ref_model.zeta.r = zeta_r;
	  update_ref_model_r();
	}

	public static void stabilization_attitude_ref_set_zeta( FloatRates zeta) {
	  stabilization_attitude_ref_set_zeta_p(zeta.p);
	  stabilization_attitude_ref_set_zeta_q(zeta.q);
	  stabilization_attitude_ref_set_zeta_r(zeta.r);
	}


	public static void reset_psi_ref_from_body() {
	  //sp has been set from body using stabilization_attitude_get_yaw_i, use that value
	  stab_att_ref_euler.psi = stab_att_sp_euler.psi << (REF_ANGLE_FRAC - INT32_ANGLE_FRAC);
	  stab_att_ref_rate.r = 0;
	  stab_att_ref_accel.r = 0;
	}

	public static void stabilization_attitude_ref_init() {

	  INT_EULERS_ZERO(stab_att_sp_euler);
	  INT32_QUAT_ZERO(stab_att_sp_quat);
	  INT_EULERS_ZERO(stab_att_ref_euler);
	  INT32_QUAT_ZERO(stab_att_ref_quat);
	  INT_RATES_ZERO(stab_att_ref_rate);
	  INT_RATES_ZERO(stab_att_ref_accel);

	  update_ref_model();

	}

	public static void stabilization_attitude_ref_enter()
	{
	  reset_psi_ref_from_body();

	  /* convert reference attitude with REF_ANGLE_FRAC to eulers with normal INT32_ANGLE_FRAC */
	   Int32Eulers ref_eul;
	  INT32_EULERS_RSHIFT(ref_eul, stab_att_ref_euler, (REF_ANGLE_FRAC - INT32_ANGLE_FRAC));
	  INT32_QUAT_OF_EULERS(stab_att_ref_quat, ref_eul);
	  INT32_QUAT_WRAP_SHORTEST(stab_att_ref_quat);

	  /* set reference rate and acceleration to zero */
	  //TODO
	  //memset(&stab_att_ref_accel, 0, sizeof( Int32Rates));
	  //memset(&stab_att_ref_rate, 0, sizeof( Int32Rates));
	  
	  
	}

	/*
	 * Reference
	 */
	public static final double DT_UPDATE =(1./PERIODIC_FREQUENCY);
	// CAUTION! Periodic frequency is assumed to be 512 Hz
	// which is equal to >> 9
	public static final int F_UPDATE_RES = 9;

	public static int OFFSET_AND_ROUND(int _a, int _b) { 
		return (((_a)+(1<<((_b)-1)))>>(_b));
	}


	public static void stabilization_attitude_ref_update() {

	  /* integrate reference attitude            */
	    Int32Rates rate_ref_scaled = {
	    OFFSET_AND_ROUND(stab_att_ref_rate.p, (REF_RATE_FRAC - INT32_RATE_FRAC)),
	    OFFSET_AND_ROUND(stab_att_ref_rate.q, (REF_RATE_FRAC - INT32_RATE_FRAC)),
	    OFFSET_AND_ROUND(stab_att_ref_rate.r, (REF_RATE_FRAC - INT32_RATE_FRAC)) };
	   Int32Quat qdot;
	  INT32_QUAT_DERIVATIVE(qdot, rate_ref_scaled, stab_att_ref_quat);
	  //QUAT_SMUL(qdot, qdot, DT_UPDATE);
	  qdot.qi = qdot.qi >> F_UPDATE_RES;
	  qdot.qx = qdot.qx >> F_UPDATE_RES;
	  qdot.qy = qdot.qy >> F_UPDATE_RES;
	  qdot.qz = qdot.qz >> F_UPDATE_RES;
	  QUAT_ADD(stab_att_ref_quat, qdot);
	  INT32_QUAT_NORMALIZE(stab_att_ref_quat);

	  /* integrate reference rotational speeds
	   * delta rate = ref_accel * dt
	   * ref_rate = old_ref_rate + delta_rate
	   */
	    Int32Rates delta_rate = {
	         stab_att_ref_accel.p >> ( F_UPDATE_RES + REF_ACCEL_FRAC - REF_RATE_FRAC),
	         stab_att_ref_accel.q >> ( F_UPDATE_RES + REF_ACCEL_FRAC - REF_RATE_FRAC),
	         stab_att_ref_accel.r >> ( F_UPDATE_RES + REF_ACCEL_FRAC - REF_RATE_FRAC)};
	  RATES_ADD(stab_att_ref_rate, delta_rate);

	  /* compute reference angular accelerations */
	   Int32Quat err;
	  /* compute reference attitude error        */
	  INT32_QUAT_INV_COMP(err, stab_att_sp_quat, stab_att_ref_quat);
	  /* wrap it in the shortest direction       */
	  INT32_QUAT_WRAP_SHORTEST(err);

	  /* propagate the 2nd order linear model : accel = -2*zeta*omega * rate - omega^2 * angle  */

	    Int32Rates accel_rate = {
	    (-two_zeta_omega.p * (stab_att_ref_rate.p >> (REF_RATE_FRAC - REF_ACCEL_FRAC))) >> (TWO_ZETA_OMEGA_RES),
	    (-two_zeta_omega.q * (stab_att_ref_rate.q >> (REF_RATE_FRAC - REF_ACCEL_FRAC))) >> (TWO_ZETA_OMEGA_RES),
	    (-two_zeta_omega.r * (stab_att_ref_rate.r >> (REF_RATE_FRAC - REF_ACCEL_FRAC))) >> (TWO_ZETA_OMEGA_RES) };

	  /* since error quaternion contains the half-angles we get 2*omega^2*err */
	    Int32Rates accel_angle = {
	    (-two_omega_2.p * (err.qx >> (INT32_QUAT_FRAC - REF_ACCEL_FRAC))) >> (TWO_OMEGA_2_RES),
	    (-two_omega_2.q * (err.qy >> (INT32_QUAT_FRAC - REF_ACCEL_FRAC))) >> (TWO_OMEGA_2_RES),
	    (-two_omega_2.r * (err.qz >> (INT32_QUAT_FRAC - REF_ACCEL_FRAC))) >> (TWO_OMEGA_2_RES) };

	  RATES_SUM(stab_att_ref_accel, accel_rate, accel_angle);


	  /* saturate acceleration */
	    Int32Rates MIN_ACCEL = { -REF_ACCEL_MAX_P, -REF_ACCEL_MAX_Q, -REF_ACCEL_MAX_R };
	    Int32Rates MAX_ACCEL = {  REF_ACCEL_MAX_P,  REF_ACCEL_MAX_Q,  REF_ACCEL_MAX_R };
	  RATES_BOUND_BOX(stab_att_ref_accel, MIN_ACCEL, MAX_ACCEL);

	  /* saturate angular speed and trim accel accordingly */
	  SATURATE_SPEED_TRIM_ACCEL();


	  /* compute ref_euler for debugging and telemetry */
	   Int32Eulers ref_eul;
	  INT32_EULERS_OF_QUAT(ref_eul, stab_att_ref_quat);
	  INT32_EULERS_LSHIFT(stab_att_ref_euler, ref_eul, (REF_ANGLE_FRAC - INT32_ANGLE_FRAC));
	}

}
=======
//package sw.airborne.firmwares.rotorcraft.stabilization;
//
//import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_rate.*;
//import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_none.*;
//import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_ref_int.*;
//import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_ref_float.*;
//import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_ref_quat_float.*;
//import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_ref_quat_int.*;
//import sw.airborne.math.*;
//import static sw.airborne.math.Pprz_algebra.*;
//import static sw.airborne.math.Pprz_algebra_int.*;
//import static sw.airborne.math.Pprz_algebra_double.*;
//import static sw.airborne.math.Pprz_geodetic_double.*;
//import static sw.airborne.math.Pprz_geodetic_int.*;
//import static sw.airborne.math.Pprz_trig_int.*;
//
//public class Stabilization_attitude_ref_quat_int {
//	public static void stabilization_attitude_ref_quat_int_SetOmegaP(int _val) { 
//		stabilization_attitude_ref_set_omega_p(_val);             
//	}
//
//	public static void stabilization_attitude_ref_quat_int_SetOmegaQ(int _val) {   
//		stabilization_attitude_ref_set_omega_q(_val);               
//	}
//	public static void stabilization_attitude_ref_quat_int_SetOmegaR(int _val) {   
//		stabilization_attitude_ref_set_omega_r(_val);               
//	}
//
//	public static void stabilization_attitude_ref_quat_int_SetZetaP(int _val) {   
//		stabilization_attitude_ref_set_zeta_p(_val);               
//	}
//	public static void stabilization_attitude_ref_quat_int_SetZetaQ(int _val) {    
//		stabilization_attitude_ref_set_zeta_q(_val);                
//	}
//	public static void stabilization_attitude_ref_quat_int_SetZetaR(int _val) {    
//		stabilization_attitude_ref_set_zeta_r(_val);                
//	}
//	
//	//===================stabilization_attitude_ref_quat_int.c====================================
//	
//	public static final int REF_ACCEL_MAX_P= BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_PDOT, REF_ACCEL_FRAC);
//	public static final int REF_ACCEL_MAX_Q= BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_QDOT, REF_ACCEL_FRAC);
//	public static final int REF_ACCEL_MAX_R= BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_RDOT, REF_ACCEL_FRAC);
//	public static final int REF_RATE_MAX_P =BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_P, REF_RATE_FRAC);
//	public static final int REF_RATE_MAX_Q =BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_Q, REF_RATE_FRAC);
//	public static final int REF_RATE_MAX_R =BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_R, REF_RATE_FRAC);
//
//
//	 public static Int32Eulers stab_att_sp_euler;
//	public static Int32Quat   stab_att_sp_quat;
//	public static  Int32Eulers stab_att_ref_euler;
//	public static  Int32Quat   stab_att_ref_quat;
//	public static  Int32Rates  stab_att_ref_rate;
//	public static  Int32Rates  stab_att_ref_accel;
////TODO
//	public static FloatRefModel stab_att_ref_model = {
//	  {STABILIZATION_ATTITUDE_REF_OMEGA_P, STABILIZATION_ATTITUDE_REF_OMEGA_Q, STABILIZATION_ATTITUDE_REF_OMEGA_R},
//	  {STABILIZATION_ATTITUDE_REF_ZETA_P, STABILIZATION_ATTITUDE_REF_ZETA_Q, STABILIZATION_ATTITUDE_REF_ZETA_R}
//	};
//
//	public static final int TWO_ZETA_OMEGA_RES =10;
//	public static final int TWO_OMEGA_2_RES= 7;
//	public static  Int32Rates two_zeta_omega;
//	public static  Int32Rates two_omega_2;
//
//	public static void update_ref_model_p() {
//	  two_zeta_omega.p = BFP_OF_REAL((2*stab_att_ref_model.zeta.p * stab_att_ref_model.omega.p), TWO_ZETA_OMEGA_RES);
//	  two_omega_2.p = BFP_OF_REAL((2*stab_att_ref_model.omega.p * stab_att_ref_model.omega.p), TWO_OMEGA_2_RES);
//	}
//
//	public static void update_ref_model_q() {
//	  two_zeta_omega.q = BFP_OF_REAL((2*stab_att_ref_model.zeta.q * stab_att_ref_model.omega.q), TWO_ZETA_OMEGA_RES);
//	  two_omega_2.q = BFP_OF_REAL((2*stab_att_ref_model.omega.q * stab_att_ref_model.omega.q), TWO_OMEGA_2_RES);
//	}
//
//	public static void update_ref_model_r() {
//	  two_zeta_omega.r = BFP_OF_REAL((2*stab_att_ref_model.zeta.r * stab_att_ref_model.omega.r), TWO_ZETA_OMEGA_RES);
//	  two_omega_2.r = BFP_OF_REAL((2*stab_att_ref_model.omega.r * stab_att_ref_model.omega.r), TWO_OMEGA_2_RES);
//	}
//
//	public static void update_ref_model() {
//	  update_ref_model_p();
//	  update_ref_model_q();
//	  update_ref_model_r();
//	}
//
//
//	public static void stabilization_attitude_ref_set_omega_p(float omega_p) {
//	  stab_att_ref_model.omega.p = omega_p;
//	  update_ref_model_p();
//	}
//
//	public static void stabilization_attitude_ref_set_omega_q(float omega_q) {
//	  stab_att_ref_model.omega.q = omega_q;
//	  update_ref_model_q();
//	}
//
//	public static void stabilization_attitude_ref_set_omega_r(float omega_r) {
//	  stab_att_ref_model.omega.r = omega_r;
//	  update_ref_model_r();
//	}
//
//	public static void stabilization_attitude_ref_set_omega( FloatRates omega) {
//	  stabilization_attitude_ref_set_omega_p(omega.p);
//	  stabilization_attitude_ref_set_omega_q(omega.q);
//	  stabilization_attitude_ref_set_omega_r(omega.r);
//	}
//
//	public static void stabilization_attitude_ref_set_zeta_p(float zeta_p) {
//	  stab_att_ref_model.zeta.p = zeta_p;
//	  update_ref_model_p();
//	}
//
//	public static void stabilization_attitude_ref_set_zeta_q(float zeta_q)  {
//	  stab_att_ref_model.zeta.q = zeta_q;
//	  update_ref_model_q();
//	}
//
//	public static void stabilization_attitude_ref_set_zeta_r(float zeta_r) {
//	  stab_att_ref_model.zeta.r = zeta_r;
//	  update_ref_model_r();
//	}
//
//	public static void stabilization_attitude_ref_set_zeta( FloatRates zeta) {
//	  stabilization_attitude_ref_set_zeta_p(zeta.p);
//	  stabilization_attitude_ref_set_zeta_q(zeta.q);
//	  stabilization_attitude_ref_set_zeta_r(zeta.r);
//	}
//
//
//	public static void reset_psi_ref_from_body() {
//	  //sp has been set from body using stabilization_attitude_get_yaw_i, use that value
//	  stab_att_ref_euler.psi = stab_att_sp_euler.psi << (REF_ANGLE_FRAC - INT32_ANGLE_FRAC);
//	  stab_att_ref_rate.r = 0;
//	  stab_att_ref_accel.r = 0;
//	}
//
//	public static void stabilization_attitude_ref_init() {
//
//	  INT_EULERS_ZERO(stab_att_sp_euler);
//	  INT32_QUAT_ZERO(stab_att_sp_quat);
//	  INT_EULERS_ZERO(stab_att_ref_euler);
//	  INT32_QUAT_ZERO(stab_att_ref_quat);
//	  INT_RATES_ZERO(stab_att_ref_rate);
//	  INT_RATES_ZERO(stab_att_ref_accel);
//
//	  update_ref_model();
//
//	}
//
//	public static void stabilization_attitude_ref_enter()
//	{
//	  reset_psi_ref_from_body();
//
//	  /* convert reference attitude with REF_ANGLE_FRAC to eulers with normal INT32_ANGLE_FRAC */
//	   Int32Eulers ref_eul;
//	  INT32_EULERS_RSHIFT(ref_eul, stab_att_ref_euler, (REF_ANGLE_FRAC - INT32_ANGLE_FRAC));
//	  INT32_QUAT_OF_EULERS(stab_att_ref_quat, ref_eul);
//	  INT32_QUAT_WRAP_SHORTEST(stab_att_ref_quat);
//
//	  /* set reference rate and acceleration to zero */
//	  //TODO
//	  //memset(&stab_att_ref_accel, 0, sizeof( Int32Rates));
//	  //memset(&stab_att_ref_rate, 0, sizeof( Int32Rates));
//	  
//	  
//	}
//
//	/*
//	 * Reference
//	 */
//	public static final double DT_UPDATE =(1./512); //TODO
//	// CAUTION! Periodic frequency is assumed to be 512 Hz
//	// which is equal to >> 9
//	public static final int F_UPDATE_RES = 9;
//
//	public static int OFFSET_AND_ROUND(int _a, int _b) { 
//		return (((_a)+(1<<((_b)-1)))>>(_b));
//	}
//	public static long OFFSET_AND_ROUND(long _a, int _b) { 
//		return (((_a)+(1<<((_b)-1)))>>(_b));
//	}
//
//
//	public static void stabilization_attitude_ref_update() {
//
//	  /* integrate reference attitude            */
//	    Int32Rates rate_ref_scaled = new Int32Rates();//{
//	    	rate_ref_scaled.p= OFFSET_AND_ROUND(stab_att_ref_rate.p, (REF_RATE_FRAC - INT32_RATE_FRAC));
//	    	rate_ref_scaled.q =OFFSET_AND_ROUND(stab_att_ref_rate.q, (REF_RATE_FRAC - INT32_RATE_FRAC));
//	    	rate_ref_scaled.r=OFFSET_AND_ROUND(stab_att_ref_rate.r, (REF_RATE_FRAC - INT32_RATE_FRAC)) ;
//	   Int32Quat qdot= new Int32Quat();
//	  INT32_QUAT_DERIVATIVE(qdot, rate_ref_scaled, stab_att_ref_quat);
//	  //QUAT_SMUL(qdot, qdot, DT_UPDATE);
//	  qdot.qi = qdot.qi >> F_UPDATE_RES;
//	  qdot.qx = qdot.qx >> F_UPDATE_RES;
//	  qdot.qy = qdot.qy >> F_UPDATE_RES;
//	  qdot.qz = qdot.qz >> F_UPDATE_RES;
//	  QUAT_ADD(stab_att_ref_quat, qdot);
//	  INT32_QUAT_NORMALIZE(stab_att_ref_quat);
//
//	  /* integrate reference rotational speeds
//	   * delta rate = ref_accel * dt
//	   * ref_rate = old_ref_rate + delta_rate
//	   */
//	    Int32Rates delta_rate = {
//	         stab_att_ref_accel.p >> ( F_UPDATE_RES + REF_ACCEL_FRAC - REF_RATE_FRAC),
//	         stab_att_ref_accel.q >> ( F_UPDATE_RES + REF_ACCEL_FRAC - REF_RATE_FRAC),
//	         stab_att_ref_accel.r >> ( F_UPDATE_RES + REF_ACCEL_FRAC - REF_RATE_FRAC)};
//	  RATES_ADD(stab_att_ref_rate, delta_rate);
//
//	  /* compute reference angular accelerations */
//	   Int32Quat err;
//	  /* compute reference attitude error        */
//	  INT32_QUAT_INV_COMP(err, stab_att_sp_quat, stab_att_ref_quat);
//	  /* wrap it in the shortest direction       */
//	  INT32_QUAT_WRAP_SHORTEST(err);
//
//	  /* propagate the 2nd order linear model : accel = -2*zeta*omega * rate - omega^2 * angle  */
//
//	    Int32Rates accel_rate = {
//	    (-two_zeta_omega.p * (stab_att_ref_rate.p >> (REF_RATE_FRAC - REF_ACCEL_FRAC))) >> (TWO_ZETA_OMEGA_RES),
//	    (-two_zeta_omega.q * (stab_att_ref_rate.q >> (REF_RATE_FRAC - REF_ACCEL_FRAC))) >> (TWO_ZETA_OMEGA_RES),
//	    (-two_zeta_omega.r * (stab_att_ref_rate.r >> (REF_RATE_FRAC - REF_ACCEL_FRAC))) >> (TWO_ZETA_OMEGA_RES) };
//
//	  /* since error quaternion contains the half-angles we get 2*omega^2*err */
//	    Int32Rates accel_angle = {
//	    (-two_omega_2.p * (err.qx >> (INT32_QUAT_FRAC - REF_ACCEL_FRAC))) >> (TWO_OMEGA_2_RES),
//	    (-two_omega_2.q * (err.qy >> (INT32_QUAT_FRAC - REF_ACCEL_FRAC))) >> (TWO_OMEGA_2_RES),
//	    (-two_omega_2.r * (err.qz >> (INT32_QUAT_FRAC - REF_ACCEL_FRAC))) >> (TWO_OMEGA_2_RES) };
//
//	  RATES_SUM(stab_att_ref_accel, accel_rate, accel_angle);
//
//
//	  /* saturate acceleration */
//	    Int32Rates MIN_ACCEL = { -REF_ACCEL_MAX_P, -REF_ACCEL_MAX_Q, -REF_ACCEL_MAX_R };
//	    Int32Rates MAX_ACCEL = {  REF_ACCEL_MAX_P,  REF_ACCEL_MAX_Q,  REF_ACCEL_MAX_R };
//	  RATES_BOUND_BOX(stab_att_ref_accel, MIN_ACCEL, MAX_ACCEL);
//
//	  /* saturate angular speed and trim accel accordingly */
//	  SATURATE_SPEED_TRIM_ACCEL();
//
//
//	  /* compute ref_euler for debugging and telemetry */
//	   Int32Eulers ref_eul;
//	  INT32_EULERS_OF_QUAT(ref_eul, stab_att_ref_quat);
//	  INT32_EULERS_LSHIFT(stab_att_ref_euler, ref_eul, (REF_ANGLE_FRAC - INT32_ANGLE_FRAC));
//	}
//
//}
>>>>>>> e34036e32d4a5ccf069cfe711e9b8f7d04437f12
