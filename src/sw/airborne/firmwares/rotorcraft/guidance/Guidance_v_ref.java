package sw.airborne.firmwares.rotorcraft.guidance;
import static sw.include.Std.*;
import static sw.airborne.math.Pprz_algebra_int.*;

public class Guidance_v_ref {

	public static final double GUIDANCE_V_REF_MIN_ZD = (-3.);
	public static final double GUIDANCE_V_REF_MAX_ZD = ( 3.);

	public static final int GV_FREQ_FRAC = 9;
	public static final int GV_FREQ =  (1<<GV_FREQ_FRAC);

	public static final int GV_ZDD_REF_FRAC = 8;
	public static final int GV_ZD_REF_FRAC  = (GV_ZDD_REF_FRAC + GV_FREQ_FRAC);
	public static final int GV_Z_REF_FRAC = (GV_ZD_REF_FRAC + GV_FREQ_FRAC);

	//-----------------------------guidance_v_ref.c-----------------------------------
	public static long gv_zdd_ref;
	public static long gv_zd_ref;
	public static long gv_z_ref;

	public static final double GUIDANCE_V_REF_MIN_ZDD = (-2.0*9.81);
	public static final int GV_MIN_ZDD = BFP_OF_REAL((float)GUIDANCE_V_REF_MIN_ZDD, GV_ZDD_REF_FRAC);
	public static final double GUIDANCE_V_REF_MAX_ZDD =( 0.8*9.81);
	public static final int GV_MAX_ZDD = BFP_OF_REAL((float)GUIDANCE_V_REF_MAX_ZDD, GV_ZDD_REF_FRAC);

	public static final int GV_MIN_ZD  = BFP_OF_REAL((float)GUIDANCE_V_REF_MIN_ZD , GV_ZD_REF_FRAC);
	public static final int GV_MAX_ZD = BFP_OF_REAL((float)GUIDANCE_V_REF_MAX_ZD , GV_ZD_REF_FRAC);

	public static final double GUIDANCE_V_REF_OMEGA =RadOfDeg(100.);
	public static final double GUIDANCE_V_REF_ZETA = 0.85;

	public static final int GV_ZETA_OMEGA_FRAC =10;
	public static final int  GV_ZETA_OMEGA =BFP_OF_REAL((float)(GUIDANCE_V_REF_ZETA*GUIDANCE_V_REF_OMEGA), GV_ZETA_OMEGA_FRAC);
	public static final int GV_OMEGA_2_FRAC= 7;
	public static final int  GV_OMEGA_2   = BFP_OF_REAL((float)(GUIDANCE_V_REF_OMEGA*GUIDANCE_V_REF_OMEGA), GV_OMEGA_2_FRAC);

	/* first order time constant */
	public static final double  GV_REF_THAU_F = 0.25;
	public static final int GV_REF_INV_THAU_FRAC =16;
	public static final int  GV_REF_INV_THAU = BFP_OF_REAL((float)(1./0.25), GV_REF_INV_THAU_FRAC);
	
	public static void gv_set_ref(int alt, int speed, int accel) {
		int new_z = ((int)alt)<<(GV_Z_REF_FRAC - INT32_POS_FRAC);
		gv_z_ref   = new_z;
		gv_zd_ref  = speed>>(INT32_SPEED_FRAC - GV_ZD_REF_FRAC);
		gv_zdd_ref = accel>>(INT32_ACCEL_FRAC - GV_ZDD_REF_FRAC);
	}
	
	public static void gv_set_ref(long alt, int speed, int accel) {
		int new_z = ((int)alt)<<(GV_Z_REF_FRAC - INT32_POS_FRAC);
		gv_z_ref   = new_z;
		gv_zd_ref  = speed>>(INT32_SPEED_FRAC - GV_ZD_REF_FRAC);
		gv_zdd_ref = accel>>(INT32_ACCEL_FRAC - GV_ZDD_REF_FRAC);
	}
	public static void gv_set_ref(long alt, long speed, int accel) {
		int new_z = ((int)alt)<<(GV_Z_REF_FRAC - INT32_POS_FRAC);
		gv_z_ref   = new_z;
		gv_zd_ref  = speed>>(INT32_SPEED_FRAC - GV_ZD_REF_FRAC);
		gv_zdd_ref = accel>>(INT32_ACCEL_FRAC - GV_ZDD_REF_FRAC);
	}

	public static void gv_update_ref_from_z_sp(int z_sp) {

		gv_z_ref  += gv_zd_ref;
		gv_zd_ref += gv_zdd_ref;

		// compute the "speed part" of zdd = -2*zeta*omega*zd -omega^2(z_sp - z)
		long zd_zdd_res = gv_zd_ref>>(GV_ZD_REF_FRAC - GV_ZDD_REF_FRAC);
		long zdd_speed = ((int)(-2*GV_ZETA_OMEGA)*zd_zdd_res)>>(GV_ZETA_OMEGA_FRAC);
		// compute z error in z_sp resolution
		int z_err_sp = z_sp - (int)(gv_z_ref>>(GV_Z_REF_FRAC-INT32_POS_FRAC));
		// convert to accel resolution
		int z_err_accel = z_err_sp>>(INT32_POS_FRAC-GV_ZDD_REF_FRAC);
		int zdd_pos = ((int)(GV_OMEGA_2)*z_err_accel)>>GV_OMEGA_2_FRAC;
		gv_zdd_ref = zdd_speed + zdd_pos;

		/* Saturate accelerations */
		Bound(gv_zdd_ref, GV_MIN_ZDD, GV_MAX_ZDD);

		/* Saturate speed and adjust acceleration accordingly */
		if (gv_zd_ref <= GV_MIN_ZD) {
			gv_zd_ref = GV_MIN_ZD;
			if (gv_zdd_ref < 0)
				gv_zdd_ref = 0;
		}
		else if (gv_zd_ref >= GV_MAX_ZD) {
			gv_zd_ref = GV_MAX_ZD;
			if (gv_zdd_ref > 0)
				gv_zdd_ref = 0;
		}
	}
	public static void gv_update_ref_from_z_sp(long z_sp) {
		
		gv_z_ref  += gv_zd_ref;
		gv_zd_ref += gv_zdd_ref;
		
		// compute the "speed part" of zdd = -2*zeta*omega*zd -omega^2(z_sp - z)
		long zd_zdd_res = gv_zd_ref>>(GV_ZD_REF_FRAC - GV_ZDD_REF_FRAC);
		long zdd_speed = ((-2*GV_ZETA_OMEGA)*zd_zdd_res)>>(GV_ZETA_OMEGA_FRAC);
		// compute z error in z_sp resolution
		long z_err_sp = z_sp - (gv_z_ref>>(GV_Z_REF_FRAC-INT32_POS_FRAC));
		// convert to accel resolution
		long z_err_accel = z_err_sp>>(INT32_POS_FRAC-GV_ZDD_REF_FRAC);
		long zdd_pos = ((GV_OMEGA_2)*z_err_accel)>>GV_OMEGA_2_FRAC;
		gv_zdd_ref = zdd_speed + zdd_pos;
		
		/* Saturate accelerations */
		Bound(gv_zdd_ref, GV_MIN_ZDD, GV_MAX_ZDD);
		
		/* Saturate speed and adjust acceleration accordingly */
		if (gv_zd_ref <= GV_MIN_ZD) {
			gv_zd_ref = GV_MIN_ZD;
			if (gv_zdd_ref < 0)
				gv_zdd_ref = 0;
		}
		else if (gv_zd_ref >= GV_MAX_ZD) {
			gv_zd_ref = GV_MAX_ZD;
			if (gv_zdd_ref > 0)
				gv_zdd_ref = 0;
		}
	}


	public static void gv_update_ref_from_zd_sp(int zd_sp) {

		gv_z_ref  += gv_zd_ref;
		gv_zd_ref += gv_zdd_ref;

		long zd_err = gv_zd_ref - (zd_sp>>(INT32_SPEED_FRAC - GV_ZD_REF_FRAC));
		long zd_err_zdd_res = zd_err>>(GV_ZD_REF_FRAC-GV_ZDD_REF_FRAC);
		gv_zdd_ref = (-(int)GV_REF_INV_THAU * zd_err_zdd_res)>>GV_REF_INV_THAU_FRAC;

		/* Saturate accelerations */
		Bound(gv_zdd_ref, GV_MIN_ZDD, GV_MAX_ZDD);

		/* Saturate speed and adjust acceleration accordingly */
		if (gv_zd_ref <= GV_MIN_ZD) {
			gv_zd_ref = GV_MIN_ZD;
			if (gv_zdd_ref < 0)
				gv_zdd_ref = 0;
		}
		else if (gv_zd_ref >= GV_MAX_ZD) {
			gv_zd_ref = GV_MAX_ZD;
			if (gv_zdd_ref > 0)
				gv_zdd_ref = 0;
		}
	}
	public static void gv_update_ref_from_zd_sp(long zd_sp) {
		
		gv_z_ref  += gv_zd_ref;
		gv_zd_ref += gv_zdd_ref;
		
		long zd_err = gv_zd_ref - (zd_sp>>(INT32_SPEED_FRAC - GV_ZD_REF_FRAC));
		long zd_err_zdd_res = zd_err>>(GV_ZD_REF_FRAC-GV_ZDD_REF_FRAC);
		gv_zdd_ref = (-(int)GV_REF_INV_THAU * zd_err_zdd_res)>>GV_REF_INV_THAU_FRAC;
		
		/* Saturate accelerations */
		Bound(gv_zdd_ref, GV_MIN_ZDD, GV_MAX_ZDD);
		
		/* Saturate speed and adjust acceleration accordingly */
		if (gv_zd_ref <= GV_MIN_ZD) {
			gv_zd_ref = GV_MIN_ZD;
			if (gv_zdd_ref < 0)
				gv_zdd_ref = 0;
		}
		else if (gv_zd_ref >= GV_MAX_ZD) {
			gv_zd_ref = GV_MAX_ZD;
			if (gv_zdd_ref > 0)
				gv_zdd_ref = 0;
		}
	}

}
