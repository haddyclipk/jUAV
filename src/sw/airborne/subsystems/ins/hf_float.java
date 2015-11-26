//package sw.airborne.subsystems.ins;
//import sw.airborne.math.*;
//import static sw.airborne.subsystems.Imu.*;
//import static sw.airborne.math.Pprz_algebra_int.*;
//import sw.airborne.filters.*;
//import static sw.airborne.filters.low_pass_filter.*;
//import static sw.airborne.State.*;
//public class hf_float {
//	public static final int PERIODIC_FREQUENCY = 512;
//	public static final int AHRS_PROPAGATE_FREQUENCY = PERIODIC_FREQUENCY;
//	public static final int GPS_LAG = 0;
//	public static final int HFF_RB_MAXN = ((int) (GPS_LAG * 4));
//	public static final int SAVE_NOW = 0;
//	public static final int SAVING = -1;
//	public static final int SAVE_DONE = -2;
//	public static int b2_hff_ps_counter = 1;
//	public static int past_save_counter = SAVE_DONE;
//	public static int save_counter = -1;
//	public static short b2_hff_lost_limit;
//	public static short b2_hff_lost_counter;
//	public static int HFF_STATE_SIZE = 2;
//	public static int i = 0;
//	public static int j = 0;
//	public static HfilterFloat b2_hff_rb_last = new HfilterFloat();
//	public static HfilterFloat [] b2_hff_rb = new HfilterFloat[HFF_RB_MAXN];
//	public static HfilterFloat b2_hff_rb_put = new HfilterFloat();
//	public static HfilterFloat b2_hff_state = new HfilterFloat();
//	public static int MAX_PP_STEPS= 6;
//	public static int acc_buf_r = 0;
//	public static int acc_buf_w = 0;
//	public static int acc_buf_n = 0;	
//	public static int b2_hff_rb_n = 0;
//	public static int HFF_PRESCALER = 10;
//	public static int HFF_FREQ = (AHRS_PROPAGATE_FREQUENCY/HFF_PRESCALER);
//	public static int GPS_LAG_N = ((int) (GPS_LAG*HFF_FREQ + 0.5));
//	public static int ACC_BUF_MAXN = (GPS_LAG_N+10);
//	public static float b2_hff_xdd_meas;
//	public static float b2_hff_ydd_meas;
//	public static FloatVect2 []past_accel = new FloatVect2[ACC_BUF_MAXN];
//	public static float DT_HFILTER = (float) (1./HFF_FREQ);
//	public static float HFF_ACCEL_NOISE=(float) 0.5;
//	public static float Q = (float) (HFF_ACCEL_NOISE*DT_HFILTER*DT_HFILTER/2.);
//	public static float Qdotdot = HFF_ACCEL_NOISE*DT_HFILTER;
//	public static Butterworth2LowPass_int filter_x = new Butterworth2LowPass_int();
//	public static Butterworth2LowPass_int filter_y = new Butterworth2LowPass_int();
//	public static Butterworth2LowPass_int filter_z = new Butterworth2LowPass_int();
//	
//	public static void b2_hff_get_past_accel(int back_n) {
//		  int i;
//		  if (back_n > acc_buf_n) {
//		    //PRINT_DBG(1, ("Cannot go back %d steps, going back only %d instead!n", back_n, acc_buf_n));
//		    back_n = acc_buf_n;
//		  } else if (back_n == 0) {
//		    //PRINT_DBG(1, ("Cannot go back zero steps!n"));
//		    return;
//		  }
//		  if ((int)(acc_buf_w - back_n) < 0)
//		    i = acc_buf_w - back_n  + ACC_BUF_MAXN;
//		  else
//		    i = acc_buf_w - back_n;
//		  b2_hff_xdd_meas = past_accel[i].x;
//		  b2_hff_ydd_meas = past_accel[i].y;
//		  //PRINT_DBG(3, ("get past accel. buf_n: %2d tbuf_w: %2d tback_n: %2d ti: %2d txdd: %f tydd: %fn", acc_buf_n, acc_buf_w, back_n, i, b2_hff_xdd_meas, b2_hff_ydd_meas));
//		}
//	
//	public static void b2_hff_propagate_x(HfilterFloat hff_work) {
//		  /* update state */
//		  hff_work.xdotdot = b2_hff_xdd_meas;
//		  hff_work.x = hff_work.x + DT_HFILTER * hff_work.xdot + DT_HFILTER*DT_HFILTER/2 * hff_work.xdotdot;
//		  hff_work.xdot = hff_work.xdot + DT_HFILTER * hff_work.xdotdot;
//		  /* update covariance */
//		   float FPF00 = hff_work.xP[0][0] + DT_HFILTER * ( hff_work.xP[1][0] + hff_work.xP[0][1] + DT_HFILTER * hff_work.xP[1][1] );
//		   float FPF01 = hff_work.xP[0][1] + DT_HFILTER * hff_work.xP[1][1];
//		   float FPF10 = hff_work.xP[1][0] + DT_HFILTER * hff_work.xP[1][1];
//		   float FPF11 = hff_work.xP[1][1];
//
//		  hff_work.xP[0][0] = FPF00 + Q;
//		  hff_work.xP[0][1] = FPF01;
//		  hff_work.xP[1][0] = FPF10;
//		  hff_work.xP[1][1] = FPF11 + Qdotdot;
//		}
//	
//	static void b2_hff_propagate_y(HfilterFloat hff_work) {
//		  /* update state */
//		  hff_work.ydotdot = b2_hff_ydd_meas;
//		  hff_work.y = hff_work.y + DT_HFILTER * hff_work.ydot + DT_HFILTER*DT_HFILTER/2 * hff_work.ydotdot;
//		  hff_work.ydot = hff_work.ydot + DT_HFILTER * hff_work.ydotdot;
//		  /* update covariance */
//		   float FPF00 = hff_work.yP[0][0] + DT_HFILTER * ( hff_work.yP[1][0] + hff_work.yP[0][1] + DT_HFILTER * hff_work.yP[1][1] );
//		   float FPF01 = hff_work.yP[0][1] + DT_HFILTER * hff_work.yP[1][1];
//		   float FPF10 = hff_work.yP[1][0] + DT_HFILTER * hff_work.yP[1][1];
//		   float FPF11 = hff_work.yP[1][1];
//
//		  hff_work.yP[0][0] = FPF00 + Q;
//		  hff_work.yP[0][1] = FPF01;
//		  hff_work.yP[1][0] = FPF10;
//		  hff_work.yP[1][1] = FPF11 + Qdotdot;
//		}
//	public static void b2_hff_set_state(HfilterFloat dest, HfilterFloat source) {
//		  dest.x       = source.x;
//		  dest.xdot    = source.xdot;
//		  dest.xdotdot = source.xdotdot;
//		  dest.y       = source.y;
//		  dest.ydot    = source.ydot;
//		  dest.ydotdot = source.ydotdot;
//		  for (int i=0; i < HFF_STATE_SIZE; i++) {
//		    for (int j=0; j < HFF_STATE_SIZE; j++) {
//		      dest.xP[i][j] = source.xP[i][j];
//		      dest.yP[i][j] = source.yP[i][j];
//		    }
//		  }
//		}
//
//	public static void INC_RB_POINTER(HfilterFloat ptr){
//		if (ptr == b2_hff_rb[HFF_RB_MAXN-1]){		
//		  i = 0;
//	      ptr = b2_hff_rb[0];
//		}
//	    else										
//	      ptr = b2_hff_rb[i++];									
//	}
//	
//	public static void b2_hff_rb_put_state(HfilterFloat source) {
//		  /* copy state from source into buffer */
//		  b2_hff_set_state(b2_hff_rb_put, source);
//		  b2_hff_rb_put.lag_counter = 0;
//		  b2_hff_rb_put.rollback = false;
//
//		  /* forward write pointer */
//		  INC_RB_POINTER(b2_hff_rb_put);
//
//		  /* increase fill count and forward last pointer if neccessary */
//		  if (b2_hff_rb_n < HFF_RB_MAXN) {
//		    b2_hff_rb_n++;
//		  } else {
//		    INC_RB_POINTER(b2_hff_rb_last);
//		  }
//		  //PRINT_DBG(2, ("put state. fill count now: %dn", b2_hff_rb_n));
//		}
//	
//
//	public static void b2_hff_rb_drop_last() {
//		  if (b2_hff_rb_n > 0){
//			    //INC_RB_POINTER(b2_hff_rb_last);
//			    b2_hff_rb_n--;
//			  } else {
//			    //PRINT_DBG(2, ("hff ringbuffer empty!\n"));
//			    b2_hff_rb_last.lag_counter = 0;
//			    b2_hff_rb_last.rollback = false;
//			  }
//			  //PRINT_DBG(2, ("drop last state. fill count now: %d\n", b2_hff_rb_n));
//			}
//	
//	public static void b2_hff_propagate_past(HfilterFloat hff_past) {
//		  //PRINT_DBG(1, ("enter propagate past: %dn", hff_past.lag_counter));
//		  /* run max MAX_PP_STEPS propagation steps */
//		  for (int i=0; i < MAX_PP_STEPS; i++) {
//		    if (hff_past.lag_counter > 0) {
//		      b2_hff_get_past_accel(hff_past.lag_counter);
//		      //PRINT_DBG(2, ("propagate past: %dn", hff_past.lag_counter));
//		      b2_hff_propagate_x(hff_past);
//		      b2_hff_propagate_y(hff_past);
//		      hff_past.lag_counter--;
//
//		      if (past_save_counter > 0) {
//		        past_save_counter--;
//		        //PRINT_DBG(2, ("dec past_save_counter: %dn", past_save_counter));
//		      } else if (past_save_counter == SAVE_NOW) {
//		        /* next GPS measurement valid at this state . save */
//		        //PRINT_DBG(2, ("save past staten"));
//		        b2_hff_rb_put_state(hff_past);
//		        past_save_counter = SAVING;
//		      } else if (past_save_counter == SAVING) {
//		        /* increase lag counter on if next state is already saved */
//		        if (hff_past == b2_hff_rb[HFF_RB_MAXN-1]){
//		          b2_hff_rb[0].lag_counter++;
//		          j =0;
//		        }
//		        else
//		          b2_hff_rb[j++].lag_counter++;
//		      }
//		    }
//
//		    /* finished re-propagating the past values */
//		    if (hff_past.lag_counter == 0) {
//		      b2_hff_set_state(b2_hff_state, hff_past);
//		      b2_hff_rb_drop_last();
//		      past_save_counter = SAVE_DONE;
//		      break;
//		    }
//		  }
//		}
//	
//	public static int INC_ACC_IDX(int idx){	return idx = (idx + 1) < ACC_BUF_MAXN ? (idx + 1) : 0;	};
//	
//	public static void b2_hff_store_accel_ltp(float x, float y) {
//		  past_accel[acc_buf_w].x = x;
//		  past_accel[acc_buf_w].y = y;
//		  INC_ACC_IDX(acc_buf_w);
//
//		  if (acc_buf_n < ACC_BUF_MAXN) {
//		    acc_buf_n++;
//		  } else {
//		    INC_ACC_IDX(acc_buf_r);
//		  }
//		}
//
//	
//	public static void b2_hff_propagate() {
//	
//		  if (b2_hff_lost_counter < b2_hff_lost_limit)
//		    b2_hff_lost_counter++;
//
//		  if(GPS_LAG!=0)
//		  /* continue re-propagating to catch up with the present */
//		  if (b2_hff_rb_last.rollback) {
//		    b2_hff_propagate_past(b2_hff_rb_last);
//		  }
//		
//
//		  /* rotate imu accel measurement to body frame and filter */
//		   Int32Vect3 acc_meas_body= new Int32Vect3();
//		  INT32_RMAT_TRANSP_VMULT(acc_meas_body, imu.body_to_imu_rmat,  imu.accel);
//
//		   Int32Vect3 acc_body_filtered = new Int32Vect3();;
//		  acc_body_filtered.x = update_butterworth_2_low_pass_int(filter_x, acc_meas_body.x);
//		  acc_body_filtered.y = update_butterworth_2_low_pass_int(filter_y, acc_meas_body.y);
//		  acc_body_filtered.z = update_butterworth_2_low_pass_int(filter_z, acc_meas_body.z);
//
//		  /* propagate current state if it is time */
//		  if (b2_hff_ps_counter == HFF_PRESCALER) {
//		    b2_hff_ps_counter = 1;
//		    if (b2_hff_lost_counter < b2_hff_lost_limit) {
//		       Int32Vect3 filtered_accel_ltp = new Int32Vect3();
//		       Int32RMat ltp_to_body_rmat = new Int32RMat(); 
//		       ltp_to_body_rmat = stateGetNedToBodyRMat_i();
//		      INT32_RMAT_TRANSP_VMULT(filtered_accel_ltp, (ltp_to_body_rmat), acc_body_filtered);
//		      b2_hff_xdd_meas = ACCEL_FLOAT_OF_BFP(filtered_accel_ltp.x);
//		      b2_hff_ydd_meas = ACCEL_FLOAT_OF_BFP(filtered_accel_ltp.y);
//		      if(GPS_LAG!=0)
//		    	  b2_hff_store_accel_ltp(b2_hff_xdd_meas, b2_hff_ydd_meas);
//		//#endif
//		      /*
//		       * propagate current state
//		       */
//		      b2_hff_propagate_x(b2_hff_state);
//		      b2_hff_propagate_y(b2_hff_state);
//
//		      if(GPS_LAG!=0)
//		      /* increase lag counter on last saved state */
//		    	  if (b2_hff_rb_n > 0)
//		    		  b2_hff_rb_last.lag_counter++;
//
//		      /* save filter state if needed */
//		      if (save_counter == 0) {
//		        //PRINT_DBG(1, ("save current staten"));
//		        b2_hff_rb_put_state(b2_hff_state);
//		        save_counter = -1;
//		      } else if (save_counter > 0) {
//		        save_counter--;
//		      }
//		//#endif
//		    }
//		  } else {
//		    b2_hff_ps_counter++;
//		  }
//		}
//
//}
