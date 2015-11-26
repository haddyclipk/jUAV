package sw.airborne.firmwares.rotorcraft.stabilization;



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
import static sw.airborne.State.*;
import static sw.airborne.firmwares.rotorcraft.guidance.Guidance_h_ref.*;
import static sw.airborne.firmwares.rotorcraft.Stabilization.*;
import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_none.*;
import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_rate.*;
import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_rc_setpoint.*;
import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_euler_int.*;


public class Stabilization_rate {
	
	public static final int F_UPDATE_RES= 9;
	public static final int REF_DOT_FRAC =11;
	public static final int REF_FRAC  =16;

	public static final int MAX_SUM_ERR= 4000000;
	public static final int STABILIZATION_RATE_DDGAIN_P =0;
	public static final int STABILIZATION_RATE_DDGAIN_Q =0;
	public static final int STABILIZATION_RATE_DDGAIN_R =0;
	public static final int STABILIZATION_RATE_IGAIN_P =0;
	public static final int STABILIZATION_RATE_IGAIN_Q =0;
	public static final int STABILIZATION_RATE_IGAIN_R =0;
	
	public static final int STABILIZATION_RATE_REF_TAU =4;
	
	public static int OFFSET_AND_ROUND(int _a, int _b) {
		return (((_a)+(1<<((_b)-1)))>>(_b));
	}
	public static long OFFSET_AND_ROUND(long _a, int _b) {
		return (((_a)+(1<<((_b)-1)))>>(_b));
	}
	public static int OFFSET_AND_ROUND2(int _a, int _b){
		return (((_a)+(1<<((_b)-1))-((_a)<0?1:0))>>(_b));
	}
	public static long OFFSET_AND_ROUND2(long _a, int _b){
		return (((_a)+(1<<((_b)-1))-((_a)<0?1:0))>>(_b));
	}

	public static Int32Rates stabilization_rate_sp;
	public static Int32Rates stabilization_rate_gain;
	public static Int32Rates stabilization_rate_igain;
	public static Int32Rates stabilization_rate_ddgain;
	public static Int32Rates stabilization_rate_ref;
	public static Int32Rates stabilization_rate_refdot;
	public static Int32Rates stabilization_rate_sum_err;

	public static Int32Rates stabilization_rate_fb_cmd;
	public static Int32Rates stabilization_rate_ff_cmd;

	public static final int STABILIZATION_RATE_DEADBAND_P =0;
	public static final int STABILIZATION_RATE_DEADBAND_Q =0;
	public static final int STABILIZATION_RATE_DEADBAND_R =200;
	public static final int MAX_PPRZ = 9600;
	
//	public static boolean ROLL_RATE_DEADBAND_EXCEEDED() {
//		
//		return (radio_control.values[RADIO_ROLL] >  STABILIZATION_RATE_DEADBAND_P || 
//				radio_control.values[RADIO_ROLL] < -STABILIZATION_RATE_DEADBAND_P);
//	}
//
//	public static boolean PITCH_RATE_DEADBAND_EXCEEDED(){
//		
//	return 	(radio_control.values[RADIO_PITCH] >  STABILIZATION_RATE_DEADBAND_Q || 
//				radio_control.values[RADIO_PITCH] < -STABILIZATION_RATE_DEADBAND_Q);
//	}                                         
//
//	public static boolean YAW_RATE_DEADBAND_EXCEEDED()   {
//		return (radio_control.values[RADIO_YAW] >  STABILIZATION_RATE_DEADBAND_R || 
//				   radio_control.values[RADIO_YAW] < -STABILIZATION_RATE_DEADBAND_R);
//	}                                     
	  
	   
	  public static void send_rate() {
//		  DOWNLINK_SEND_RATE_LOOP(DefaultChannel, DefaultDevice,
//		      stabilization_rate_sp.p,
//		      stabilization_rate_sp.q,
//		      stabilization_rate_sp.r,
//		      stabilization_rate_ref.p,
//		      stabilization_rate_ref.q,
//		      stabilization_rate_ref.r,
//		      stabilization_rate_refdot.p,
//		      stabilization_rate_refdot.q,
//		      stabilization_rate_refdot.r,
//		      stabilization_rate_sum_err.p,
//		      stabilization_rate_sum_err.q,
//		      stabilization_rate_sum_err.r,
//		      stabilization_rate_ff_cmd.p,
//		      stabilization_rate_ff_cmd.q,
//		      stabilization_rate_ff_cmd.r,
//		      stabilization_rate_fb_cmd.p,
//		      stabilization_rate_fb_cmd.q,
//		      stabilization_rate_fb_cmd.r,
//		      stabilization_cmd[COMMAND_THRUST]);
		}
		

		public static void stabilization_rate_init() {

		  INT_RATES_ZERO(stabilization_rate_sp);

		  RATES_ASSIGN(stabilization_rate_gain,
		               400, //STABILIZATION_RATE_GAIN_P,
		               400,//STABILIZATION_RATE_GAIN_Q,
		               350);//STABILIZATION_RATE_GAIN_R);
		  RATES_ASSIGN(stabilization_rate_igain,
		               STABILIZATION_RATE_IGAIN_P,
		               STABILIZATION_RATE_IGAIN_Q,
		               STABILIZATION_RATE_IGAIN_R);
		  RATES_ASSIGN(stabilization_rate_ddgain,
		               STABILIZATION_RATE_DDGAIN_P,
		               STABILIZATION_RATE_DDGAIN_Q,
		               STABILIZATION_RATE_DDGAIN_R);

		  INT_RATES_ZERO(stabilization_rate_ref);
		  INT_RATES_ZERO(stabilization_rate_refdot);
		  INT_RATES_ZERO(stabilization_rate_sum_err);

//		if(PERIODIC_TELEMETRY){
//		  register_periodic_telemetry(DefaultPeriodic, "RATE_LOOP", send_rate);
//		}
		}


//		public static void stabilization_rate_read_rc() {
//
//		  if(ROLL_RATE_DEADBAND_EXCEEDED())
//		    stabilization_rate_sp.p = (int)radio_control.values[RADIO_ROLL] * STABILIZATION_RATE_SP_MAX_P / MAX_PPRZ;
//		  else
//		    stabilization_rate_sp.p = 0;
//
//		  if(PITCH_RATE_DEADBAND_EXCEEDED())
//		    stabilization_rate_sp.q = (int)radio_control.values[RADIO_PITCH] * STABILIZATION_RATE_SP_MAX_Q / MAX_PPRZ;
//		  else
//		    stabilization_rate_sp.q = 0;
//
//		  if(YAW_RATE_DEADBAND_EXCEEDED())
//		    stabilization_rate_sp.r = (int)radio_control.values[RADIO_YAW] * STABILIZATION_RATE_SP_MAX_R / MAX_PPRZ;
//		  else
//		    stabilization_rate_sp.r = 0;
//
//		  // Setpoint at ref resolution
//		  INT_RATES_LSHIFT(stabilization_rate_sp, stabilization_rate_sp, REF_FRAC - INT32_RATE_FRAC);
//		}

		//Read rc with roll and yaw sitcks switched if the default orientation is vertical but airplane sticks are desired
//		public static void stabilization_rate_read_rc_switched_sticks() {
//
//		  if(ROLL_RATE_DEADBAND_EXCEEDED())
//		    stabilization_rate_sp.r = (int) -radio_control.values[RADIO_ROLL] * STABILIZATION_RATE_SP_MAX_P / MAX_PPRZ;
//		  else
//		    stabilization_rate_sp.r = 0;
//
//		  if(PITCH_RATE_DEADBAND_EXCEEDED())
//		    stabilization_rate_sp.q = (int)radio_control.values[RADIO_PITCH] * STABILIZATION_RATE_SP_MAX_Q / MAX_PPRZ;
//		  else
//		    stabilization_rate_sp.q = 0;
//
//		  if(YAW_RATE_DEADBAND_EXCEEDED())
//		    stabilization_rate_sp.p = (int)radio_control.values[RADIO_YAW] * STABILIZATION_RATE_SP_MAX_R / MAX_PPRZ;
//		  else
//		    stabilization_rate_sp.p = 0;
//
//		  // Setpoint at ref resolution
//		    INT_RATES_LSHIFT(stabilization_rate_sp, stabilization_rate_sp, REF_FRAC - INT32_RATE_FRAC);
//		}

		public static void stabilization_rate_enter() {
		  RATES_COPY(stabilization_rate_ref, stabilization_rate_sp);
		  INT_RATES_ZERO(stabilization_rate_sum_err);
		}

		public static void stabilization_rate_run(boolean in_flight) {

		  /* reference */
		   Int32Rates _r = new Int32Rates();
		  RATES_DIFF(_r, stabilization_rate_sp, stabilization_rate_ref);
		  RATES_SDIV(stabilization_rate_refdot, _r, STABILIZATION_RATE_REF_TAU);
		  /* integrate ref */

		    Int32Rates _delta_ref = new Int32Rates();
		   _delta_ref.p= stabilization_rate_refdot.p >> ( F_UPDATE_RES + REF_DOT_FRAC - REF_FRAC);
		_delta_ref.q= stabilization_rate_refdot.q >> ( F_UPDATE_RES + REF_DOT_FRAC - REF_FRAC);
		_delta_ref.r=stabilization_rate_refdot.r >> ( F_UPDATE_RES + REF_DOT_FRAC - REF_FRAC);

		  RATES_ADD(stabilization_rate_ref, _delta_ref);

		  /* compute feed-forward command */
		  RATES_EWMULT_RSHIFT(stabilization_rate_ff_cmd, stabilization_rate_ddgain, stabilization_rate_refdot, 9);


		  /* compute feed-back command */
		  /* error for feedback */

//		    Int32Rates _ref_scaled = {
//	    OFFSET_AND_ROUND(stabilization_rate_ref.p, (REF_FRAC - INT32_RATE_FRAC)),
//		    OFFSET_AND_ROUND(stabilization_rate_ref.q, (REF_FRAC - INT32_RATE_FRAC)),
//		    OFFSET_AND_ROUND(stabilization_rate_ref.r, (REF_FRAC - INT32_RATE_FRAC)) };
		  Int32Rates _ref_scaled =  new Int32Rates();
		  _ref_scaled.p=OFFSET_AND_ROUND(stabilization_rate_ref.p, (REF_FRAC - INT32_RATE_FRAC));
		  _ref_scaled.q=OFFSET_AND_ROUND(stabilization_rate_ref.q, (REF_FRAC - INT32_RATE_FRAC));
		  _ref_scaled.r = OFFSET_AND_ROUND(stabilization_rate_ref.r, (REF_FRAC - INT32_RATE_FRAC));

		   Int32Rates _error = new Int32Rates();
		   Int32Rates body_rate = stateGetBodyRates_i();
		  RATES_DIFF(_error, _ref_scaled, (body_rate));
		  if (in_flight) {
		    /* update integrator */
		    RATES_ADD(stabilization_rate_sum_err, _error);
		    RATES_BOUND_CUBE(stabilization_rate_sum_err, -MAX_SUM_ERR, MAX_SUM_ERR);
		  }
		  else {
		    INT_RATES_ZERO(stabilization_rate_sum_err);
		  }

		  /* PI */
		  stabilization_rate_fb_cmd.p = stabilization_rate_gain.p * _error.p +
		    OFFSET_AND_ROUND2((stabilization_rate_igain.p  * stabilization_rate_sum_err.p), 10);

		  stabilization_rate_fb_cmd.q = stabilization_rate_gain.q * _error.q +
		    OFFSET_AND_ROUND2((stabilization_rate_igain.q  * stabilization_rate_sum_err.q), 10);

		  stabilization_rate_fb_cmd.r = stabilization_rate_gain.r * _error.r +
		    OFFSET_AND_ROUND2((stabilization_rate_igain.r  * stabilization_rate_sum_err.r), 10);

		  stabilization_rate_fb_cmd.p = stabilization_rate_fb_cmd.p >> 11;
		  stabilization_rate_fb_cmd.q = stabilization_rate_fb_cmd.q >> 11;
		  stabilization_rate_fb_cmd.r = stabilization_rate_fb_cmd.r >> 11;

		  /* sum to final command */
		  stabilization_cmd[0]  = stabilization_rate_ff_cmd.p + stabilization_rate_fb_cmd.p;
		  stabilization_cmd[1] = stabilization_rate_ff_cmd.q + stabilization_rate_fb_cmd.q;
		  stabilization_cmd[2]   = stabilization_rate_ff_cmd.r + stabilization_rate_fb_cmd.r;

		  /* bound the result */
		  BoundAbs(stabilization_cmd[0], MAX_PPRZ);
		  BoundAbs(stabilization_cmd[1], MAX_PPRZ);
		  BoundAbs(stabilization_cmd[2], MAX_PPRZ);

		}
   
}
