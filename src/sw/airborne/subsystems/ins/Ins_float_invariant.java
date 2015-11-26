package sw.airborne.subsystems.ins;
//package sw.airborne.subsystems.ins;
//import static sw.airborne.subsystems.ahrs.Ahrs_aligner.*;
//import sw.airborne.math.*;
//import static sw.airborne.math.Pprz_algebra.*;
//import static sw.airborne.math.Pprz_algebra_float.*;
//import static sw.airborne.math.Pprz_algebra_int.*;
//import static sw.airborne.subsystems.Ahrs.*;
//import static sw.airborne.subsystems.ins.Ins.*;
//import static sw.airborne.subsystems.Imu.*;
//import static sw.airborne.subsystems.Gps.*;
//import static sw.airborne.State.*;
//import static sw.airborne.subsystems.ins.hf_float.*;
//import static sw.airborne.math.Pprz_rk_float.*;
//
//public class ins_float_invariant {
//	public static final int INV_STATE_DIM = 15;
//	public static final int INV_COMMAND_DIM = 6;
//	public static final float dt = (float) (1./ ((float)AHRS_PROPAGATE_FREQUENCY));;
//	public static InsFloatInv ins_impl = new InsFloatInv();
//	public static boolean ins_baro_initialized;
//	public static FloatVect3 A = new FloatVect3();
//	public static FloatVect3 B = new FloatVect3();
//	public static float  INS_ROLL_NEUTRAL_DEFAULT = (float) 0.;
//	public static float  INS_PITCH_NEUTRAL_DEFAULT = (float) 0.;
//	static float ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
//	static float ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;
//	//FloatVect3 B = { (float)(INS_H_X), (float)(INS_H_Y), (float)(INS_H_Z) };
//	//static const struct FloatVect3 A = { 0.f, 0.f, 9.81f };
//	//?????
//	
//	public static void invariant_model(inv_state o, inv_state x, int n, inv_command u, int m) {
//
//		  inv_state s = new inv_state();
//				  s = x;
//		  inv_command c = new inv_command();
//		  		  c = u;
//		  inv_state s_dot = new inv_state(); 
//		  FloatRates rates = new FloatRates();
//		  FloatVect3 tmp_vect = new FloatVect3();
//		  FloatQuat tmp_quat = new FloatQuat();
//		  float  norm;
//
//		  // test accel sensitivity
//		  if (Math.abs(s.as) < 0.1) {
//		    // too small, return x_dot = 0 to avoid division by 0
//		    float_vect_zero(o, n);
//		    // TODO set ins state to error
//		    return;
//		  }
//
//		  /* dot_q = 0.5 * q * (x_rates - x_bias) + LE * q + (1 - ||q||^2) * q */
//		  RATES_DIFF(rates, c.rates, s.bias);
//		  FLOAT_VECT3_ASSIGN(tmp_vect, rates.p, rates.q, rates.r);
//		  FLOAT_QUAT_VMUL_LEFT(s_dot.quat, s.quat, tmp_vect);
//		  FLOAT_QUAT_SMUL(s_dot.quat, s_dot.quat, 0.5);
//
//		  FLOAT_QUAT_VMUL_RIGHT(tmp_quat, s.quat, ins_impl.corr.LE);
//		  FLOAT_QUAT_ADD(s_dot.quat, tmp_quat);
//
//		  norm = FLOAT_QUAT_NORM(s.quat);
//		  norm = (float) (1. - (norm*norm));
//		  FLOAT_QUAT_SMUL(tmp_quat, s.quat, norm);
//		  FLOAT_QUAT_ADD(s_dot.quat, tmp_quat);
//
//		  /* dot_V = A + (1/as) * (q * am * q-1) + ME */
//		  FLOAT_QUAT_RMAT_B2N(s_dot.speed, s.quat, c.accel);
//		  FLOAT_VECT3_SMUL(s_dot.speed, s_dot.speed, 1. / (s.as));
//		  FLOAT_VECT3_ADD(s_dot.speed, A);
//		  FLOAT_VECT3_ADD(s_dot.speed, ins_impl.corr.ME);
//
//		  /* dot_X = V + NE */
//		  FLOAT_VECT3_SUM(s_dot.pos, s.speed, ins_impl.corr.NE);
//
//		  /* bias_dot = q-1 * (OE) * q */
//		  FLOAT_QUAT_RMAT_N2B(tmp_vect, s.quat, ins_impl.corr.OE);
//		  RATES_ASSIGN(s_dot.bias, tmp_vect.x, tmp_vect.y, tmp_vect.z);
//
//		  /* as_dot = as * RE */
//		  s_dot.as = (s.as) * (ins_impl.corr.RE);
//
//		  /* hb_dot = SE */
//		  s_dot.hb = ins_impl.corr.SE;
//
//		  // set output
//		  //memcpy(o, &s_dot, n*sizeof(float)); ?????
//		}
//
//	
//	
//	public static void ahrs_align()
//	{
//	  /* Compute an initial orientation from accel and mag directly as quaternion */
//	  ahrs_float_get_quat_from_accel_mag(ins_impl.state.quat, ahrs_aligner.lp_accel, ahrs_aligner.lp_mag);
//
//	  /* use average gyro as initial value for bias */
//	  FloatRates bias0 = new FloatRates();
//	  RATES_COPY(bias0, ahrs_aligner.lp_gyro);
//	  RATES_FLOAT_OF_BFP(ins_impl.state.bias, bias0);
//
//	  // ins and ahrs are now running
//	  ahrs.status = AHRS_RUNNING;
//	  ins.status = InsStatus.INS_RUNNING;
//	}
//	
//	public static void init_invariant_state() {
//		  // init state
//		  FLOAT_QUAT_ZERO(ins_impl.state.quat);
//		  FLOAT_RATES_ZERO(ins_impl.state.bias);
//		  FLOAT_VECT3_ZERO(ins_impl.state.pos);
//		  FLOAT_VECT3_ZERO(ins_impl.state.speed);
//		  ins_impl.state.as = 1.0f;
//		  ins_impl.state.hb = 0.0f;
//
//		  // init measures
//		  FLOAT_VECT3_ZERO(ins_impl.meas.pos_gps);
//		  FLOAT_VECT3_ZERO(ins_impl.meas.speed_gps);
//		  ins_impl.meas.baro_alt = 0.0f;
//
//		  // init baro
//		  ins_baro_initialized = false;
//	}
//	
//	public static void error_output(InsFloatInv _ins) {
//
//		  FloatVect3 YBt = new FloatVect3();
//		  FloatVect3 I = new FloatVect3();
//		  FloatVect3 Ev = new FloatVect3();
//		  FloatVect3 Eb = new FloatVect3();
//		  FloatVect3 Ex = new FloatVect3();
//		  FloatVect3 Itemp = new FloatVect3();
//		  FloatVect3 Ebtemp = new FloatVect3();
//		  FloatVect3 Evtemp = new FloatVect3();
//		  
//		  float Eh;
//		  float temp;
//
//		  // test accel sensitivity
//		  if (Math.abs(_ins.state.as) < 0.1) {
//		    // too small, don't do anything to avoid division by 0
//		    return;
//		  }
//
//		  /* YBt = q * yB * q-1  */
//		  FLOAT_QUAT_RMAT_B2N(YBt, _ins.state.quat, _ins.meas.mag);
//
//		  FLOAT_QUAT_RMAT_B2N(I, _ins.state.quat, _ins.cmd.accel);
//		  FLOAT_VECT3_SMUL(I, I, 1. / (_ins.state.as));
//
//		  /*--------- E = ( ŷ - y ) ----------*/
//		  /* Eb = ( B - YBt ) */
//		  FLOAT_VECT3_DIFF(Eb, B, YBt);
//
//		  // pos and speed error only if GPS data are valid
//		  if (gps.fix == GPS_FIX_3D && ins.status == InsStatus.INS_RUNNING && state.utm_initialized_f || state.ned_initialized_f) {
//		    /* Ev = (V - YV)   */
//		    FLOAT_VECT3_DIFF(Ev, _ins.state.speed, _ins.meas.speed_gps);
//		    /* Ex = (X - YX)  */
//		    FLOAT_VECT3_DIFF(Ex, _ins.state.pos, _ins.meas.pos_gps);
//		  }
//		  else {
//		    FLOAT_VECT3_ZERO(Ev);
//		    FLOAT_VECT3_ZERO(Ex);
//		  }
//		  /* Eh = < X,e3 > - hb - YH */
//		  Eh = _ins.state.pos.z - _ins.state.hb - _ins.meas.baro_alt;
//
//		  /*--------------Gains--------------*/
//
//		  /**** LvEv + LbEb = -lvIa x Ev +  lb < B x Eb, Ia > Ia *****/
//		  FLOAT_VECT3_SMUL(Itemp, I, -_ins.gains.lv/100.);
//		  FLOAT_VECT3_CROSS_PRODUCT(Evtemp, Itemp, Ev);
//
//		  FLOAT_VECT3_CROSS_PRODUCT(Ebtemp, B, Eb);
//		  temp = FLOAT_VECT3_DOT_PRODUCT(Ebtemp, I);
//		  temp = (float) ((_ins.gains.lb/100.) * temp);
//
//		  FLOAT_VECT3_SMUL(Ebtemp, I, temp);
//		  FLOAT_VECT3_ADD(Evtemp, Ebtemp);
//		  FLOAT_VECT3_COPY(_ins.corr.LE, Evtemp);
//
//		  /***** MvEv + MhEh = -mv * Ev + (-mh * <Eh,e3>)********/
//		  _ins.corr.ME.x = (float) ((-_ins.gains.mv) * Ev.x + 0.);
//		  _ins.corr.ME.y = (float) ((-_ins.gains.mv) * Ev.y + 0.);
//		  _ins.corr.ME.z = ((-_ins.gains.mvz) * Ev.z) + ((-_ins.gains.mh) * Eh);
//
//		  /****** NxEx + NhEh = -nx * Ex + (-nh * <Eh, e3>) ********/
//		  _ins.corr.NE.x = (float) ((-_ins.gains.nx) * Ex.x + 0.);
//		  _ins.corr.NE.y = (float) ((-_ins.gains.nx) * Ex.y + 0.);
//		  _ins.corr.NE.z = ((-_ins.gains.nxz) * Ex.z) + ((-_ins.gains.nh) * Eh);
//
//		  /****** OvEv + ObEb = ovIa x Ev - ob < B x Eb, Ia > Ia ********/
//		  FLOAT_VECT3_SMUL(Itemp, I, _ins.gains.ov/1000.);
//		  FLOAT_VECT3_CROSS_PRODUCT(Evtemp, Itemp, Ev);
//
//		  FLOAT_VECT3_CROSS_PRODUCT(Ebtemp, B, Eb);
//		  temp = FLOAT_VECT3_DOT_PRODUCT(Ebtemp, I);
//		  temp = (float) ((-_ins.gains.ob/1000.) * temp);
//
//		  FLOAT_VECT3_SMUL(Ebtemp, I, temp);
//		  FLOAT_VECT3_ADD(Evtemp, Ebtemp);
//		  FLOAT_VECT3_COPY(_ins.corr.OE, Evtemp);
//
//		  /* a scalar */
//		  /****** RvEv + RhEh = rv < Ia, Ev > + (-rhEh) **************/
//		  _ins.corr.RE = (float) (((_ins.gains.rv/100.) * FLOAT_VECT3_DOT_PRODUCT(Ev, I)) + ((-_ins.gains.rh/10000.) * Eh));
//
//		  /****** ShEh ******/
//		  _ins.corr.SE = (_ins.gains.sh) * Eh;
//
//		}
//
//	
//	public static void ahrs_propagate() {
//		   NedCoor_f accel = new NedCoor_f();
//		   FloatRates body_rates = new FloatRates();
//		   FloatEulers eulers = new FloatEulers();
//
//		  // realign all the filter if needed
//		  // a complete init cycle is required
//		  if (ins_impl.reset) {
//		    ins_impl.reset = false;
//		    ins.status = InsStatus.INS_UNINIT;
//		    ahrs.status = AHRS_UNINIT;
//		    init_invariant_state();
//		  }
//
//		  // fill command vector
//		  Int32Rates gyro_meas_body = new Int32Rates();
//		  INT32_RMAT_TRANSP_RATEMULT(gyro_meas_body, imu.body_to_imu_rmat, imu.gyro);
//		  RATES_FLOAT_OF_BFP(ins_impl.cmd.rates, gyro_meas_body);
//		   Int32Vect3 accel_meas_body = new Int32Vect3();
//		  INT32_RMAT_TRANSP_VMULT(accel_meas_body, imu.body_to_imu_rmat, imu.accel);
//		  ACCELS_FLOAT_OF_BFP(ins_impl.cmd.accel, accel_meas_body);
//
//		  // update correction gains
//		  error_output(ins_impl);
//
//		  // propagate model
//		   inv_state new_state = new inv_state();
//		   runge_kutta_4_float(new_state,
//		      ins_impl.state, INV_STATE_DIM,
//		      ins_impl.cmd, INV_COMMAND_DIM,
//		      //invariant_model, 
//		      dt);
//		  ins_impl.state = new_state;
//
//		  // normalize quaternion
//		  FLOAT_QUAT_NORMALIZE(ins_impl.state.quat);
//
//		  // set global state
//		  FLOAT_EULERS_OF_QUAT(eulers, ins_impl.state.quat);
//		//#if INS_UPDATE_FW_ESTIMATOR
//		  // Some stupid lines of code for neutrals
//		  eulers.phi -= ins_roll_neutral;
//		  eulers.theta -= ins_pitch_neutral;
//		  stateSetNedToBodyEulers_f(eulers);
//		//#else
//		  stateSetNedToBodyQuat_f(ins_impl.state.quat);
//		//#endif
//		  RATES_DIFF(body_rates, ins_impl.cmd.rates, ins_impl.state.bias);
//		  stateSetBodyRates_f(body_rates);
//		  stateSetPositionNed_f(ins_impl.state.pos);
//		  stateSetSpeedNed_f(ins_impl.state.speed);
//		  // untilt accel and remove gravity
//		  FLOAT_QUAT_RMAT_B2N(accel, ins_impl.state.quat, ins_impl.cmd.accel);
//		  FLOAT_VECT3_SMUL(accel, accel, 1. / (ins_impl.state.as));
//		  FLOAT_VECT3_ADD(accel, A);
//		  stateSetAccelNed_f(accel);
//
//		  //------------------------------------------------------------//
//
////		  RunOnceEvery(3,{
////		      DOWNLINK_SEND_INV_FILTER(DefaultChannel, DefaultDevice,
////		        &ins_impl.state.quat.qi,
////		        &eulers.phi,
////		        &eulers.theta,
////		        &eulers.psi,
////		        &ins_impl.state.speed.x,
////		        &ins_impl.state.speed.y,
////		        &ins_impl.state.speed.z,
////		        &ins_impl.state.pos.x,
////		        &ins_impl.state.pos.y,
////		        &ins_impl.state.pos.z,
////		        &ins_impl.state.bias.p,
////		        &ins_impl.state.bias.q,
////		        &ins_impl.state.bias.r,
////		        &ins_impl.state.as,
////		        &ins_impl.state.hb,
////		        &ins_impl.meas.baro_alt,
////		        &ins_impl.meas.pos_gps.z)
////		      });
//
////		#if LOG_INVARIANT_FILTER
////		  if (pprzLogFile.fs != NULL) {
////		    if (!log_started) {
////		      // log file header
////		      sdLogWriteLog(&pprzLogFile, "p q r ax ay az gx gy gz gvx gvy gvz mx my mz b qi qx qy qz bp bq br vx vy vz px py pz hb as\n");
////		      log_started = TRUE;
////		    }
////		    else {
////		      sdLogWriteLog(&pprzLogFile, "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
////		          ins_impl.cmd.rates.p,
////		          ins_impl.cmd.rates.q,
////		          ins_impl.cmd.rates.r,
////		          ins_impl.cmd.accel.x,
////		          ins_impl.cmd.accel.y,
////		          ins_impl.cmd.accel.z,
////		          ins_impl.meas.pos_gps.x,
////		          ins_impl.meas.pos_gps.y,
////		          ins_impl.meas.pos_gps.z,
////		          ins_impl.meas.speed_gps.x,
////		          ins_impl.meas.speed_gps.y,
////		          ins_impl.meas.speed_gps.z,
////		          ins_impl.meas.mag.x,
////		          ins_impl.meas.mag.y,
////		          ins_impl.meas.mag.z,
////		          ins_impl.meas.baro_alt,
////		          ins_impl.state.quat.qi,
////		          ins_impl.state.quat.qx,
////		          ins_impl.state.quat.qy,
////		          ins_impl.state.quat.qz,
////		          ins_impl.state.bias.p,
////		          ins_impl.state.bias.q,
////		          ins_impl.state.bias.r,
////		          ins_impl.state.speed.x,
////		          ins_impl.state.speed.y,
////		          ins_impl.state.speed.z,
////		          ins_impl.state.pos.x,
////		          ins_impl.state.pos.y,
////		          ins_impl.state.pos.z,
////		          ins_impl.state.hb,
////		          ins_impl.state.as);
////		    }
////		  }
////		#endif
//		}
//
//	
//
//}
