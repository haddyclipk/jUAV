package sw.airborne.firmwares.rotorcraft.stabilization;

import static sw.airborne.math.Pprz_trig_int.*;
import static sw.airborne.math.Pprz_algebra.*;
import static sw.airborne.firmwares.rotorcraft.Stabilization.*;
import static sw.include.Std.*;

import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_rc_setpoint.*;
//import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_ref_quat_int.*;
import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_ref_int.*;
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
import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_ref_saturate.*;
//
//
public class Stabilization_attitude_ref_euler_int {

	public static Int32Eulers stab_att_sp_euler;
	public static Int32Eulers stab_att_ref_euler;  ///< with #REF_ANGLE_FRAC
	public static Int32Rates  stab_att_ref_rate;
	public static Int32Rates  stab_att_ref_accel;
	public static long STABILIZATION_ATTITUDE_THETA_IGAIN = 200;
	public static long STABILIZATION_ATTITUDE_PHI_IGAIN = 200;
	public static long STABILIZATION_ATTITUDE_PSI_IGAIN = 10;
	public static long STABILIZATION_ATTITUDE_PHI_PGAIN =1000;
	public static long STABILIZATION_ATTITUDE_THETA_PGAIN =1000;
	public static long STABILIZATION_ATTITUDE_PSI_PGAIN =500;
	public static long STABILIZATION_ATTITUDE_PHI_DGAIN =400;
	public static long STABILIZATION_ATTITUDE_THETA_DGAIN =400;
	public static long STABILIZATION_ATTITUDE_PSI_DGAIN =300;
	public static long STABILIZATION_ATTITUDE_PHI_DDGAIN =300;
	public static long STABILIZATION_ATTITUDE_THETA_DDGAIN =300;
	public static long STABILIZATION_ATTITUDE_PSI_DDGAIN =300;
	
	
	public static double STABILIZATION_ATTITUDE_REF_OMEGA_P = 6.981317;//TODO: generated
	public static double STABILIZATION_ATTITUDE_REF_OMEGA_Q = 6.981317;//TODO: generated
	public static double STABILIZATION_ATTITUDE_REF_ZETA_P = 0.85;//TODO: generated
	public static double STABILIZATION_ATTITUDE_REF_ZETA_Q = 0.85;//TODO: generated
	public static double STABILIZATION_ATTITUDE_REF_MAX_PDOT = RadOfDeg(8000.);//TODO: generated
	public static double STABILIZATION_ATTITUDE_REF_MAX_QDOT = RadOfDeg(8000.);//TODO: generated
	public static double STABILIZATION_ATTITUDE_REF_MAX_RDOT = RadOfDeg(1800.);//TODO: generated
	public static double STABILIZATION_ATTITUDE_REF_MAX_P  = 6.981317;//TODO: generated
	public static double STABILIZATION_ATTITUDE_REF_MAX_Q  = 6.981317;//TODO: generated
	public static double STABILIZATION_ATTITUDE_REF_MAX_R  = 3.14159265;//TODO: generated
	public static double  STABILIZATION_ATTITUDE_REF_OMEGA_R =4.363323125;//TODO: generated
	public static double  STABILIZATION_ATTITUDE_REF_ZETA_R =0.85;//TODO: generated
	
	
	public static void stabilization_attitude_ref_init() {

		INT_EULERS_ZERO(stab_att_sp_euler);
		INT_EULERS_ZERO(stab_att_ref_euler);
		INT_RATES_ZERO( stab_att_ref_rate);
		INT_RATES_ZERO( stab_att_ref_accel);

	}

	public static final int F_UPDATE_RES = 9;
	public static final int F_UPDATE  = (1<<F_UPDATE_RES);

	public static final int REF_ACCEL_MAX_P =BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_PDOT, REF_ACCEL_FRAC);
	public static final int REF_ACCEL_MAX_Q =BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_QDOT, REF_ACCEL_FRAC);
	public static final int REF_ACCEL_MAX_R= BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_RDOT, REF_ACCEL_FRAC);

	public static final int REF_RATE_MAX_P =BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_P, REF_RATE_FRAC);
	public static final int REF_RATE_MAX_Q =BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_Q, REF_RATE_FRAC);
	public static final int REF_RATE_MAX_R =BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_R, REF_RATE_FRAC);

	public static final double OMEGA_P =  STABILIZATION_ATTITUDE_REF_OMEGA_P; 
	public static final double ZETA_P  =  STABILIZATION_ATTITUDE_REF_ZETA_P;
	public static final int ZETA_OMEGA_P_RES =10;
	public static final int ZETA_OMEGA_P=BFP_OF_REAL((ZETA_P*OMEGA_P), ZETA_OMEGA_P_RES);
	public static final int OMEGA_2_P_RES =7;
	public static final int OMEGA_2_P  =  BFP_OF_REAL((OMEGA_P*OMEGA_P), OMEGA_2_P_RES);

	public static final double OMEGA_Q  = STABILIZATION_ATTITUDE_REF_OMEGA_Q;
	public static final double ZETA_Q  =  STABILIZATION_ATTITUDE_REF_ZETA_Q;
	public static final int ZETA_OMEGA_Q_RES =10;
	public static final int ZETA_OMEGA_Q =BFP_OF_REAL((ZETA_Q*OMEGA_Q), ZETA_OMEGA_Q_RES);
	public static final int OMEGA_2_Q_RES =7;
	public static final int OMEGA_2_Q  =  BFP_OF_REAL((OMEGA_Q*OMEGA_Q), OMEGA_2_Q_RES);

	public static final double OMEGA_R  = STABILIZATION_ATTITUDE_REF_OMEGA_R;
	public static final double ZETA_R   = STABILIZATION_ATTITUDE_REF_ZETA_R;
	public static final int ZETA_OMEGA_R_RES =10;
	public static final int ZETA_OMEGA_R =BFP_OF_REAL((ZETA_R*OMEGA_R), ZETA_OMEGA_R_RES);
	public static final int OMEGA_2_R_RES =7;
	public static final int OMEGA_2_R  =  BFP_OF_REAL((OMEGA_R*OMEGA_R), OMEGA_2_R_RES);


	public static final int REF_ANGLE_PI    =  BFP_OF_REAL(3.1415926535897932384626433832795029, REF_ANGLE_FRAC);
	public static final int REF_ANGLE_TWO_PI = BFP_OF_REAL(2.*3.1415926535897932384626433832795029, REF_ANGLE_FRAC);
	public static final void ANGLE_REF_NORMALIZE(int _a) {                       
		while (_a >  REF_ANGLE_PI)  _a -= REF_ANGLE_TWO_PI; 
		while (_a < -REF_ANGLE_PI)  _a += REF_ANGLE_TWO_PI; 
	}


	/* explicitly define to zero to disable attitude reference generation */

	public static final boolean USE_ATTITUDE_REF = true;
//
//
	public static void stabilization_attitude_ref_update() {

		if (USE_ATTITUDE_REF){

			/* dumb integrate reference attitude        */
			Int32Eulers d_angle =  new Int32Eulers();
			d_angle.phi	=stab_att_ref_rate.p >> ( F_UPDATE_RES + REF_RATE_FRAC - REF_ANGLE_FRAC);
			d_angle.theta=stab_att_ref_rate.q >> ( F_UPDATE_RES + REF_RATE_FRAC - REF_ANGLE_FRAC);
			d_angle.psi=stab_att_ref_rate.r >> ( F_UPDATE_RES + REF_RATE_FRAC - REF_ANGLE_FRAC);
			EULERS_ADD(stab_att_ref_euler, d_angle );
			//ANGLE_REF_NORMALIZE(stab_att_ref_euler.psi);
			while (stab_att_ref_euler.psi >  REF_ANGLE_PI)  stab_att_ref_euler.psi -= REF_ANGLE_TWO_PI; 
			while (stab_att_ref_euler.psi < -REF_ANGLE_PI)  stab_att_ref_euler.psi += REF_ANGLE_TWO_PI; 
			/* integrate reference rotational speeds   */
			Int32Rates d_rate = new Int32Rates();// {
			d_rate.p=	stab_att_ref_accel.p >> ( F_UPDATE_RES + REF_ACCEL_FRAC - REF_RATE_FRAC);
			d_rate.q=	stab_att_ref_accel.q >> ( F_UPDATE_RES + REF_ACCEL_FRAC - REF_RATE_FRAC);
			d_rate.r=	stab_att_ref_accel.r >> ( F_UPDATE_RES + REF_ACCEL_FRAC - REF_RATE_FRAC);
			RATES_ADD(stab_att_ref_rate, d_rate);

			/* attitude setpoint with REF_ANGLE_FRAC   */
			Int32Eulers sp_ref = new Int32Eulers();
			INT32_EULERS_LSHIFT(sp_ref, stab_att_sp_euler, (REF_ANGLE_FRAC - INT32_ANGLE_FRAC));

			/* compute reference attitude error        */
			Int32Eulers ref_err = new Int32Eulers();
			EULERS_DIFF(ref_err, stab_att_ref_euler, sp_ref);
			/* wrap it in the shortest direction       */
			//ANGLE_REF_NORMALIZE(ref_err.psi);
			while (ref_err.psi >  REF_ANGLE_PI)  ref_err.psi -= REF_ANGLE_TWO_PI; 
			while (ref_err.psi < -REF_ANGLE_PI)  ref_err.psi += REF_ANGLE_TWO_PI; 
			/* compute reference angular accelerations */
			Int32Rates accel_rate = new Int32Rates();//{
			accel_rate.p=	((int)(-2.*ZETA_OMEGA_P) * (stab_att_ref_rate.p >> (REF_RATE_FRAC - REF_ACCEL_FRAC)))
					>> (ZETA_OMEGA_P_RES);
					accel_rate.q=		((int)(-2.*ZETA_OMEGA_Q) * (stab_att_ref_rate.q >> (REF_RATE_FRAC - REF_ACCEL_FRAC)))
							>> (ZETA_OMEGA_Q_RES);
							accel_rate.r=		((int)(-2.*ZETA_OMEGA_R) * (stab_att_ref_rate.r >> (REF_RATE_FRAC - REF_ACCEL_FRAC)))
									>> (ZETA_OMEGA_R_RES) ;

			Int32Rates accel_angle = new Int32Rates();// {
			accel_angle.p=		((int)(-OMEGA_2_P)* (ref_err.phi   >> (REF_ANGLE_FRAC - REF_ACCEL_FRAC))) >> (OMEGA_2_P_RES);
			accel_angle.q=	((int)(-OMEGA_2_Q)* (ref_err.theta >> (REF_ANGLE_FRAC - REF_ACCEL_FRAC))) >> (OMEGA_2_Q_RES);
			accel_angle.r=		((int)(-OMEGA_2_R)* (ref_err.psi   >> (REF_ANGLE_FRAC - REF_ACCEL_FRAC))) >> (OMEGA_2_R_RES) ;

			RATES_SUM(stab_att_ref_accel, accel_rate, accel_angle);

			/*	saturate acceleration */
			Int32Rates MIN_ACCEL = new Int32Rates();
			MIN_ACCEL.p =-REF_ACCEL_MAX_P; //{ -REF_ACCEL_MAX_P, -REF_ACCEL_MAX_Q, -REF_ACCEL_MAX_R };
					MIN_ACCEL.q =		-REF_ACCEL_MAX_Q;
							MIN_ACCEL.r =	-REF_ACCEL_MAX_R	;
			Int32Rates MAX_ACCEL = new Int32Rates();//{  REF_ACCEL_MAX_P,  REF_ACCEL_MAX_Q,  REF_ACCEL_MAX_R };
			MAX_ACCEL.p = REF_ACCEL_MAX_P;
					MAX_ACCEL.q =REF_ACCEL_MAX_Q;
					MAX_ACCEL.r =REF_ACCEL_MAX_R;
			RATES_BOUND_BOX(stab_att_ref_accel, MIN_ACCEL, MAX_ACCEL);

			/* saturate speed and trim accel accordingly */
			SATURATE_SPEED_TRIM_ACCEL();
		}
		else  {/* !USE_ATTITUDE_REF  */
			INT32_EULERS_LSHIFT(stab_att_ref_euler, stab_att_sp_euler, (REF_ANGLE_FRAC - INT32_ANGLE_FRAC));
			INT_RATES_ZERO(stab_att_ref_rate);
			INT_RATES_ZERO(stab_att_ref_accel);
		}
		/* USE_ATTITUDE_REF   */

	}
}
