package sw.airborne.firmwares.rotorcraft.guidance;
import static sw.airborne.math.Pprz_algebra_int.*;
import sw.include.Std;
public class Guidance_v_adapt {

	public static final int GV_ADAPT_X_FRAC = 24;
	public static final int GV_ADAPT_P_FRAC = 18;

	public static final double GUIDANCE_V_ADAPT_INITIAL_HOVER_THROTTLE =0.3;
	public static final double GUIDANCE_V_ADAPT_MIN_HOVER_THROTTLE =0.2;
	public static final double GUIDANCE_V_ADAPT_MAX_HOVER_THROTTLE= 0.75;
	public static final double GUIDANCE_V_ADAPT_NOISE_FACTOR =1.0;
	public static final double GUIDANCE_V_ADAPT_MAX_ACCEL =4.0;
	public static final double GUIDANCE_V_ADAPT_MAX_CMD =0.9;
	public static final double GUIDANCE_V_ADAPT_MIN_CMD = 0.1;


	public static int gv_adapt_X;
	public static int gv_adapt_P;
	public static long gv_adapt_Xmeas;

	public static final double GV_ADAPT_SYS_NOISE_F =0.00005;

	public static final int GV_ADAPT_SYS_NOISE = BFP_OF_REAL((float)GV_ADAPT_SYS_NOISE_F, GV_ADAPT_P_FRAC);

	/* Measuremement noises */
	public static final double GV_ADAPT_MEAS_NOISE_HOVER_F =(50.0*GUIDANCE_V_ADAPT_NOISE_FACTOR);
	public static final int GV_ADAPT_MEAS_NOISE_HOVER =BFP_OF_REAL((float)GV_ADAPT_MEAS_NOISE_HOVER_F, GV_ADAPT_P_FRAC);
	public static final double GV_ADAPT_MEAS_NOISE_OF_ZD =(100.0*GUIDANCE_V_ADAPT_NOISE_FACTOR);

	/* Initial Covariance    */
	public static final double GV_ADAPT_P0_F =0.1;
	public static  int gv_adapt_P0 = BFP_OF_REAL((float)GV_ADAPT_P0_F, GV_ADAPT_P_FRAC);
	public static  int gv_adapt_X0 = (int) (BFP_OF_REAL((float)9.81, GV_ADAPT_X_FRAC) / (GUIDANCE_V_ADAPT_INITIAL_HOVER_THROTTLE * 9600));

	public static void gv_adapt_init() {
		gv_adapt_X = gv_adapt_X0;
		gv_adapt_P = gv_adapt_P0;
	}

	public static final int K_FRAC =12;
	
	private static int gv_adapt_min_cmd;
	private static int gv_adapt_max_cmd;
	private static int gv_adapt_max_accel;
	private static int max_out;
	private static int min_out;
	public static void gv_adapt_run(int zdd_meas, int thrust_applied, int zd_ref) {

		gv_adapt_min_cmd = (int) (GUIDANCE_V_ADAPT_MIN_CMD * 9600);
		gv_adapt_max_cmd = (int) (GUIDANCE_V_ADAPT_MAX_CMD * 9600);
		gv_adapt_max_accel = ACCEL_BFP_OF_REAL((float)GUIDANCE_V_ADAPT_MAX_ACCEL);

		/* Update only if accel and commands are in a valid range */
		/* This also ensures we don't divide by zero */
		if (thrust_applied < gv_adapt_min_cmd || thrust_applied > gv_adapt_max_cmd
				|| zdd_meas < -gv_adapt_max_accel || zdd_meas > gv_adapt_max_accel) {
			return;
		}

		/* We don't propagate state, it's constant !       */
		/* We propagate our covariance                     */
		gv_adapt_P =  gv_adapt_P + GV_ADAPT_SYS_NOISE;

		/* Compute our measurement. If zdd_meas is in the range +/-5g, meas is less than 30 bits */
		int g_m_zdd = ((int)BFP_OF_REAL((float)9.81, INT32_ACCEL_FRAC) - zdd_meas)<<(GV_ADAPT_X_FRAC - INT32_ACCEL_FRAC);
		if ( g_m_zdd > 0) {
			gv_adapt_Xmeas = (g_m_zdd + (thrust_applied>>1)) / thrust_applied;
		} else {
			gv_adapt_Xmeas = (g_m_zdd - (thrust_applied>>1)) / thrust_applied;
		}

		/* Compute a residual */
		long residual = gv_adapt_Xmeas - gv_adapt_X;

		/* Covariance Error  E = P + R  */
		int ref = zd_ref >> (INT32_SPEED_FRAC - GV_ADAPT_P_FRAC);
		if (zd_ref < 0) ref = -ref;
		int E = (int) (gv_adapt_P + GV_ADAPT_MEAS_NOISE_HOVER + ref * GV_ADAPT_MEAS_NOISE_OF_ZD);

		/* Kalman gain  K = P / (P + R) = P / E  */
		int K = (gv_adapt_P<<K_FRAC) / E;

		/* Update Covariance  Pnew = P - K * P   */
		gv_adapt_P = gv_adapt_P - ((K * gv_adapt_P)>>K_FRAC);
		/* Don't let covariance climb over initial value */
		if (gv_adapt_P > gv_adapt_P0) {
			gv_adapt_P = gv_adapt_P0;
		}
		/* Update State */
		gv_adapt_X = gv_adapt_X + (((int)(K * residual))>>K_FRAC);

		/* Output bounds.
		 * Don't let it climb over a value that would
		 * give less than #GUIDANCE_V_ADAPT_MIN_HOVER_THROTTLE % throttle
		 * or more than #GUIDANCE_V_ADAPT_MAX_HOVER_THROTTLE % throttle.
		 */
		max_out = (int) (BFP_OF_REAL((float)9.81, GV_ADAPT_X_FRAC) /
				(GUIDANCE_V_ADAPT_MIN_HOVER_THROTTLE * 9600));
		min_out = (int) (BFP_OF_REAL((float)9.81, GV_ADAPT_X_FRAC) /
				(GUIDANCE_V_ADAPT_MAX_HOVER_THROTTLE * 9600));
		Std.Bound(gv_adapt_X, min_out, max_out);
	}
	public static void gv_adapt_run(long zdd_meas, int thrust_applied, long zd_ref) {

		gv_adapt_min_cmd = (int) (GUIDANCE_V_ADAPT_MIN_CMD * 9600);
		gv_adapt_max_cmd = (int) (GUIDANCE_V_ADAPT_MAX_CMD * 9600);
		gv_adapt_max_accel = ACCEL_BFP_OF_REAL((float)GUIDANCE_V_ADAPT_MAX_ACCEL);

		/* Update only if accel and commands are in a valid range */
		/* This also ensures we don't divide by zero */
		if (thrust_applied < gv_adapt_min_cmd || thrust_applied > gv_adapt_max_cmd
				|| zdd_meas < -gv_adapt_max_accel || zdd_meas > gv_adapt_max_accel) {
			return;
		}

		/* We don't propagate state, it's constant !       */
		/* We propagate our covariance                     */
		gv_adapt_P =  gv_adapt_P + GV_ADAPT_SYS_NOISE;

		/* Compute our measurement. If zdd_meas is in the range +/-5g, meas is less than 30 bits */
		long g_m_zdd = (BFP_OF_REAL((float)9.81, INT32_ACCEL_FRAC) - zdd_meas)<<(GV_ADAPT_X_FRAC - INT32_ACCEL_FRAC);
		if ( g_m_zdd > 0) {
			gv_adapt_Xmeas = (g_m_zdd + (thrust_applied>>1)) / thrust_applied;
		} else {
			gv_adapt_Xmeas = (g_m_zdd - (thrust_applied>>1)) / thrust_applied;
		}

		/* Compute a residual */
		long residual = gv_adapt_Xmeas - gv_adapt_X;

		/* Covariance Error  E = P + R  */
		long ref = zd_ref >> (INT32_SPEED_FRAC - GV_ADAPT_P_FRAC);
		if (zd_ref < 0) ref = -ref;
		int E = (int) (gv_adapt_P + GV_ADAPT_MEAS_NOISE_HOVER + ref * GV_ADAPT_MEAS_NOISE_OF_ZD);

		/* Kalman gain  K = P / (P + R) = P / E  */
		int K = (gv_adapt_P<<K_FRAC) / E;

		/* Update Covariance  Pnew = P - K * P   */
		gv_adapt_P = gv_adapt_P - ((K * gv_adapt_P)>>K_FRAC);
		/* Don't let covariance climb over initial value */
		if (gv_adapt_P > gv_adapt_P0) {
			gv_adapt_P = gv_adapt_P0;
		}

		/* Update State */
		gv_adapt_X = gv_adapt_X + (((int)(K * residual))>>K_FRAC);

		/* Output bounds.
		 * Don't let it climb over a value that would
		 * give less than #GUIDANCE_V_ADAPT_MIN_HOVER_THROTTLE % throttle
		 * or more than #GUIDANCE_V_ADAPT_MAX_HOVER_THROTTLE % throttle.
		 */
		max_out = (int) (BFP_OF_REAL((float)9.81, GV_ADAPT_X_FRAC) /
				(GUIDANCE_V_ADAPT_MIN_HOVER_THROTTLE * 9600));
		min_out = (int) (BFP_OF_REAL((float)9.81, GV_ADAPT_X_FRAC) /
				(GUIDANCE_V_ADAPT_MAX_HOVER_THROTTLE * 9600));
		Std.Bound(gv_adapt_X, min_out, max_out);
	}
	public static void gv_adapt_run(long zdd_meas, long thrust_applied, long zd_ref) {
		
		gv_adapt_min_cmd = (int) (GUIDANCE_V_ADAPT_MIN_CMD * 9600);
		gv_adapt_max_cmd = (int) (GUIDANCE_V_ADAPT_MAX_CMD * 9600);
		gv_adapt_max_accel = ACCEL_BFP_OF_REAL((float)GUIDANCE_V_ADAPT_MAX_ACCEL);
		
		/* Update only if accel and commands are in a valid range */
		/* This also ensures we don't divide by zero */
		if (thrust_applied < gv_adapt_min_cmd || thrust_applied > gv_adapt_max_cmd
				|| zdd_meas < -gv_adapt_max_accel || zdd_meas > gv_adapt_max_accel) {
			return;
		}
		
		/* We don't propagate state, it's constant !       */
		/* We propagate our covariance                     */
		gv_adapt_P =  gv_adapt_P + GV_ADAPT_SYS_NOISE;
		
		/* Compute our measurement. If zdd_meas is in the range +/-5g, meas is less than 30 bits */
		long g_m_zdd = (BFP_OF_REAL((float)9.81, INT32_ACCEL_FRAC) - zdd_meas)<<(GV_ADAPT_X_FRAC - INT32_ACCEL_FRAC);
		if ( g_m_zdd > 0) {
			gv_adapt_Xmeas = (g_m_zdd + (thrust_applied>>1)) / thrust_applied;
		} else {
			gv_adapt_Xmeas = (g_m_zdd - (thrust_applied>>1)) / thrust_applied;
		}
		
		/* Compute a residual */
		long residual = gv_adapt_Xmeas - gv_adapt_X;
		
		/* Covariance Error  E = P + R  */
		long ref = zd_ref >> (INT32_SPEED_FRAC - GV_ADAPT_P_FRAC);
		if (zd_ref < 0) ref = -ref;
		int E = (int) (gv_adapt_P + GV_ADAPT_MEAS_NOISE_HOVER + ref * GV_ADAPT_MEAS_NOISE_OF_ZD);
		
		/* Kalman gain  K = P / (P + R) = P / E  */
		int K = (gv_adapt_P<<K_FRAC) / E;
		
		/* Update Covariance  Pnew = P - K * P   */
		gv_adapt_P = gv_adapt_P - ((K * gv_adapt_P)>>K_FRAC);
		/* Don't let covariance climb over initial value */
		if (gv_adapt_P > gv_adapt_P0) {
			gv_adapt_P = gv_adapt_P0;
		}
		
		/* Update State */
		gv_adapt_X = gv_adapt_X + (((int)(K * residual))>>K_FRAC);
		
		/* Output bounds.
		 * Don't let it climb over a value that would
		 * give less than #GUIDANCE_V_ADAPT_MIN_HOVER_THROTTLE % throttle
		 * or more than #GUIDANCE_V_ADAPT_MAX_HOVER_THROTTLE % throttle.
		 */
		max_out = (int) (BFP_OF_REAL((float)9.81, GV_ADAPT_X_FRAC) /
				(GUIDANCE_V_ADAPT_MIN_HOVER_THROTTLE * 9600));
		min_out = (int) (BFP_OF_REAL((float)9.81, GV_ADAPT_X_FRAC) /
				(GUIDANCE_V_ADAPT_MAX_HOVER_THROTTLE * 9600));
		Std.Bound(gv_adapt_X, min_out, max_out);
	}

}
