package sw.airborne.firmwares.rotorcraft.guidance;
import sw.airborne.math.*; 
import sw.include.Std;
import static sw.airborne.math.Pprz_algebra_int.*;
import static sw.airborne.math.Pprz_algebra.*;
import static sw.include.Std.*;
import static sw.airborne.math.Pprz_geodetic.*;
import static sw.airborne.math.Pprz_orientation_conversion.*;
import static sw.airborne.math.Pprz_geodetic_int.*;
import static sw.airborne.math.Pprz_geodetic_float.*;
import static sw.airborne.math.Pprz_algebra_float.*;
import static sw.airborne.math.Pprz_trig_int.*;

public class Guidance_h_ref {
	
	

	public static final double GUIDANCE_H_REF_MAX_SPEED = 5. ;
	public static final double GUIDANCE_H_REF_MAX_ACCEL = 5.66;


	public static final int GH_FREQ_FRAC = 9;
	public static final int GH_FREQ = (1<<GH_FREQ_FRAC);

	public static final int GH_ACCEL_REF_FRAC = 8;

	public static final int GH_SPEED_REF_FRAC = (GH_ACCEL_REF_FRAC + GH_FREQ_FRAC);

	public static final int GH_POS_REF_FRAC = (GH_SPEED_REF_FRAC + GH_FREQ_FRAC);
	//-------------------------------------------------guidance_h_ref.c------------------------------------
	public static Int32Vect2 gh_accel_ref;

	public static Int32Vect2 gh_speed_ref;

	public static Int64Vect2 gh_pos_ref;

	public static final int gh_max_accel = BFP_OF_REAL(GUIDANCE_H_REF_MAX_ACCEL, GH_ACCEL_REF_FRAC);

	 
	 public static float gh_max_speed =(float) GUIDANCE_H_REF_MAX_SPEED;
	 
	 public static final int GH_MAX_SPEED_REF_FRAC =7;
	 
	 public static int gh_max_speed_int = BFP_OF_REAL(GUIDANCE_H_REF_MAX_SPEED, GH_MAX_SPEED_REF_FRAC);
	 
	 public static final double GUIDANCE_H_REF_OMEGA = RadOfDeg(67.);
	 
	 public static final double GUIDANCE_H_REF_ZETA = 0.85;
	 public static final int GH_ZETA_OMEGA_FRAC = 10;
	 public static final int GH_OMEGA_2_FRAC = 7;
	 
	 public static final int gh_zeta_omega = BFP_OF_REAL((GUIDANCE_H_REF_ZETA*GUIDANCE_H_REF_OMEGA), GH_ZETA_OMEGA_FRAC);
	 public static final int gh_omega_2= BFP_OF_REAL((GUIDANCE_H_REF_OMEGA*GUIDANCE_H_REF_OMEGA), GH_OMEGA_2_FRAC);

	 public static final double GUIDANCE_H_REF_TAU = 0.5;
	 public static final int GH_REF_INV_TAU_FRAC = 16;

	 public static final int gh_ref_inv_tau = BFP_OF_REAL((1./GUIDANCE_H_REF_TAU), GH_REF_INV_TAU_FRAC);

	 public static Int32Vect2 gh_max_speed_ref;
	 public static Int32Vect2 gh_max_accel_ref;

	 public static int route_ref;
	 public static int s_route_ref;
	 public static int c_route_ref;

	 public static float gh_set_max_speed(float max_speed) {
		 /* limit to 100m/s as int version would overflow at  2^14 = 128 m/s */
		 gh_max_speed = Math.min(Math.abs(max_speed), 100.0f);
		 gh_max_speed_int = BFP_OF_REAL(gh_max_speed, GH_MAX_SPEED_REF_FRAC);
		 return gh_max_speed;
	 }

	 public static void gh_set_ref(Int32Vect2 pos, Int32Vect2 speed, Int32Vect2 accel) {
		  Int64Vect2 new_pos = new Int64Vect2();
		 new_pos.x = ((int)pos.x)<<(GH_POS_REF_FRAC - INT32_POS_FRAC);
		 new_pos.y = ((int)pos.y)<<(GH_POS_REF_FRAC - INT32_POS_FRAC);
		 gh_pos_ref = new_pos;
		 INT32_VECT2_RSHIFT(gh_speed_ref, speed, (INT32_SPEED_FRAC - GH_SPEED_REF_FRAC));
		 INT32_VECT2_RSHIFT(gh_accel_ref, accel, (INT32_ACCEL_FRAC - GH_ACCEL_REF_FRAC));
	 }

	 public static void gh_update_ref_from_pos_sp( Int32Vect2 pos_sp) {

		 VECT2_ADD(gh_pos_ref, gh_speed_ref);
		 VECT2_ADD(gh_speed_ref, gh_accel_ref);

		 // compute the "speed part" of accel = -2*zeta*omega*speed -omega^2(pos - pos_sp)
		  Int32Vect2 speed = new Int32Vect2();
		 INT32_VECT2_RSHIFT(speed, gh_speed_ref, (GH_SPEED_REF_FRAC - GH_ACCEL_REF_FRAC));
		 VECT2_SMUL(speed, speed, -2 * gh_zeta_omega);
		 INT32_VECT2_RSHIFT(speed, speed, GH_ZETA_OMEGA_FRAC);
		 // compute pos error in pos_sp resolution
		  Int32Vect2 pos_err = new Int32Vect2();;
		 INT32_VECT2_RSHIFT(pos_err, gh_pos_ref, (GH_POS_REF_FRAC - INT32_POS_FRAC));
		 VECT2_DIFF(pos_err, pos_err, pos_sp);
		 // convert to accel resolution
		 INT32_VECT2_RSHIFT(pos_err, pos_err, (INT32_POS_FRAC - GH_ACCEL_REF_FRAC));
		 // compute the "pos part" of accel
		  Int32Vect2 pos = new Int32Vect2();
		 VECT2_SMUL(pos, pos_err, -gh_omega_2);
		 INT32_VECT2_RSHIFT(pos, pos, GH_OMEGA_2_FRAC);
		 // sum accel
		 VECT2_SUM(gh_accel_ref, speed, pos);

		 /* Compute max ref accel/speed along route before saturation */
		 gh_compute_ref_max(pos_err);

		 gh_saturate_ref_accel();
		 gh_saturate_ref_speed();
	 }


	 public static void gh_update_ref_from_speed_sp( Int32Vect2 speed_sp) {
		 /* WARNING: SPEED SATURATION UNTESTED */
		 VECT2_ADD(gh_pos_ref, gh_speed_ref);
		 VECT2_ADD(gh_speed_ref, gh_accel_ref);

		 // compute speed error
		  Int32Vect2 speed_err = new Int32Vect2();
		 INT32_VECT2_RSHIFT(speed_err, speed_sp, (INT32_SPEED_FRAC - GH_SPEED_REF_FRAC));
		 VECT2_DIFF(speed_err, gh_speed_ref, speed_err);
		 // convert to accel resolution
		 INT32_VECT2_RSHIFT(speed_err, speed_err, (GH_SPEED_REF_FRAC - GH_ACCEL_REF_FRAC));
		 // compute accel from speed_sp
		 VECT2_SMUL(gh_accel_ref, speed_err, -gh_ref_inv_tau);
		 INT32_VECT2_RSHIFT(gh_accel_ref, gh_accel_ref, GH_REF_INV_TAU_FRAC);

		 /* Compute max ref accel/speed along route before saturation */
		 gh_compute_ref_max_speed(speed_sp);
		 gh_compute_ref_max_accel(speed_err);

		 gh_saturate_ref_accel();
		 gh_saturate_ref_speed();
	 }

	 public static void gh_compute_route_ref( Int32Vect2 ref_vector) {
		 float f_route_ref =(float) Math.atan2(-ref_vector.y, -ref_vector.x);
		 route_ref = ANGLE_BFP_OF_REAL(f_route_ref);
		 /* Compute North and East route components */
		 s_route_ref =  PPRZ_ITRIG_SIN(route_ref);
		 c_route_ref = PPRZ_ITRIG_COS( route_ref);
		 c_route_ref = Math.abs(c_route_ref);
		 s_route_ref = Math.abs(s_route_ref);
	 }

	 public static void gh_compute_ref_max( Int32Vect2 ref_vector) {
		 /* Bound ref to max speed/accel along route reference angle.
		  * If angle can't be computed, simply set both axes to max magnitude/sqrt(2).
		  */
		 if (ref_vector.x == 0 && ref_vector.y == 0) {
			 gh_max_accel_ref.x = gh_max_accel_ref.y = (int) (gh_max_accel * 0.707);
			 gh_max_speed_ref.x = gh_max_speed_ref.y = (int) (gh_max_speed_int * 0.707);
		 }
		 else {
			 gh_compute_route_ref(ref_vector);
			 /* Compute maximum acceleration*/
			 gh_max_accel_ref.x = INT_MULT_RSHIFT(gh_max_accel, c_route_ref, INT32_TRIG_FRAC);
			 gh_max_accel_ref.y = INT_MULT_RSHIFT(gh_max_accel, s_route_ref, INT32_TRIG_FRAC);
			 /* Compute maximum reference x/y velocity from absolute max_speed */
			 gh_max_speed_ref.x = INT_MULT_RSHIFT(gh_max_speed_int, c_route_ref, INT32_TRIG_FRAC);
			 gh_max_speed_ref.y = INT_MULT_RSHIFT(gh_max_speed_int, s_route_ref, INT32_TRIG_FRAC);
		 }
		 /* restore gh_speed_ref range (Q14.17) */
		 INT32_VECT2_LSHIFT(gh_max_speed_ref, gh_max_speed_ref, (GH_SPEED_REF_FRAC - GH_MAX_SPEED_REF_FRAC));
	 }

	 public static void gh_compute_ref_max_accel( Int32Vect2 ref_vector) {
		 /* Bound ref to max accel along reference vector.
		  * If angle can't be computed, simply set both axes to max magnitude/sqrt(2).
		  */
		 if (ref_vector.x == 0 && ref_vector.y == 0) {
			 gh_max_accel_ref.x = gh_max_accel_ref.y =(int) ( gh_max_accel * 0.707);
		 }
		 else {
			 gh_compute_route_ref(ref_vector);
			 /* Compute maximum acceleration*/
			 gh_max_accel_ref.x = INT_MULT_RSHIFT(gh_max_accel, c_route_ref, INT32_TRIG_FRAC);
			 gh_max_accel_ref.y = INT_MULT_RSHIFT(gh_max_accel, s_route_ref, INT32_TRIG_FRAC);
		 }
	 }

	 public static void gh_compute_ref_max_speed( Int32Vect2 ref_vector) {
		 /* Bound ref to max speed along reference vector.
		  * If angle can't be computed, simply set both axes to max magnitude/sqrt(2).
		  */
		 if (ref_vector.x == 0 && ref_vector.y == 0) {
			 gh_max_speed_ref.x = gh_max_speed_ref.y = (int) (gh_max_speed_int * 0.707);
		 }
		 else {
			 gh_compute_route_ref(ref_vector);
			 /* Compute maximum reference x/y velocity from absolute max_speed */
			 gh_max_speed_ref.x = INT_MULT_RSHIFT(gh_max_speed_int, c_route_ref, INT32_TRIG_FRAC);
			 gh_max_speed_ref.y = INT_MULT_RSHIFT(gh_max_speed_int, s_route_ref, INT32_TRIG_FRAC);
		 }
		 /* restore gh_speed_ref range (Q14.17) */
		 INT32_VECT2_LSHIFT(gh_max_speed_ref, gh_max_speed_ref, (GH_SPEED_REF_FRAC - GH_MAX_SPEED_REF_FRAC));
	 }

	 public static void gh_saturate_ref_accel() {
		 /* Saturate accelerations */
		 BoundAbs(gh_accel_ref.x, gh_max_accel_ref.x);
		 BoundAbs(gh_accel_ref.y, gh_max_accel_ref.y);
	 }

	 public static void gh_saturate_ref_speed() {
		 if (gh_speed_ref.x < -gh_max_speed_ref.x) {
			 gh_speed_ref.x = -gh_max_speed_ref.x;
			 if (gh_accel_ref.x < 0)
				 gh_accel_ref.x = 0;
		 }
		 else if (gh_speed_ref.x > gh_max_speed_ref.x) {
			 gh_speed_ref.x = gh_max_speed_ref.x;
			 if (gh_accel_ref.x > 0)
				 gh_accel_ref.x = 0;
		 }
		 if (gh_speed_ref.y < -gh_max_speed_ref.y) {
			 gh_speed_ref.y = -gh_max_speed_ref.y;
			 if (gh_accel_ref.y < 0)
				 gh_accel_ref.y = 0;
		 }
		 else if (gh_speed_ref.y > gh_max_speed_ref.y) {
			 gh_speed_ref.y = gh_max_speed_ref.y;
			 if (gh_accel_ref.y > 0)
				 gh_accel_ref.y = 0;
		 }
	 }
	 
	 
	 

}
