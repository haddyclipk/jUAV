package sw.airborne;

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

public class State {
	/* *
	 * This general state interface holds all the most important vehicle states like
	 * position, velocity, attitude, etc. It handles coordinate system and
	 * fixed-/floating-point conversion on the fly when needed.
	 *
	 * You can set e.g. the position in any coordinate system you wish:
	 * stateSetPositionNed_i() to set the position in fixed-point NED coordinates.
	 * If you need to read the position somewhere else in a different representation,
	 * call: stateGetPositionLla_f() and only then the LLA float position representation
	 * is calculated on the fly and returned. It's also only calculated once,
	 * until a new position is set which invalidates all the other representations again.
	 */

	
	public static final int POS_ECEF_I = 0;
	public static final int POS_NED_I = 1;
	public static final int POS_ENU_I  =2;
	public static final int POS_LLA_I  =3;
	public static final int POS_UTM_I = 4;
	public static final int POS_ECEF_F =5;
	public static final int POS_NED_F  =6;
	public static final int POS_ENU_F  =7;
	public static final int POS_LLA_F  =8;
	public static final int POS_UTM_F  =9;
	public static final int POS_LOCAL_COORD = ((1<<POS_NED_I)|(1<<POS_NED_F)|(1<<POS_ENU_I)|(1<<POS_ENU_F));
	public static final int POS_GLOBAL_COORD = ((1<<POS_ECEF_I)|(1<<POS_ECEF_F)|(1<<POS_LLA_I)|(1<<POS_LLA_F)|(1<<POS_UTM_I)|(1<<POS_UTM_F));
	
	public static final int SPEED_ECEF_I = 0;
	public static final int SPEED_NED_I   =1;
	public static final int SPEED_ENU_I  = 2;
	public static final int SPEED_HNORM_I= 3;
	public static final int SPEED_HDIR_I  =4;
	public static final int SPEED_ECEF_F  =5;
	public static final int SPEED_NED_F  = 6;
	public static final int SPEED_ENU_F  = 7;
	public static final int SPEED_HNORM_F= 8;
	public static final int SPEED_HDIR_F = 9;
	public static final int SPEED_LOCAL_COORD = ((1<<SPEED_NED_I)|(1<<SPEED_ENU_I)|(1<<SPEED_NED_F)|(1<<SPEED_ENU_F));
	
	public static final int ACCEL_ECEF_I= 0;
	public static final int ACCEL_NED_I = 1;
	public static final int ACCEL_ECEF_F= 2;
	public static final int ACCEL_NED_F = 3;
	
	public static final int RATE_I= 0;
	public static final int RATE_F= 1;
	
	public static final int WINDSPEED_I =0;
	public static final int AIRSPEED_I  =1;
	public static final int WINDSPEED_F =2;
	public static final int AIRSPEED_F  =3;
	public static final int AOA_F       =4;
	public static final int SIDESLIP_F  =5;
	
	public static State_struct state;
	
	public static void stateSetLocalOrigin_i(LtpDef_i ltp_def) {
		//memcpy(&state.ned_origin_i, ltp_def, sizeof( LtpDef_i));
		state.ned_origin_i  = ltp_def.clone();  
		/* convert to float */
		Pprz_algebra.ECEF_FLOAT_OF_BFP(state.ned_origin_f.ecef, state.ned_origin_i.ecef);
        
// (state.ned_origin_f.ecef).x = (float)(((state.ned_origin_i.ecef).x)/1e2);          
// (state.ned_origin_f.ecef).y = (float)(((state.ned_origin_i.ecef).y)/1e2);         
// (state.ned_origin_f.ecef).z = (float)(((state.ned_origin_i.ecef).z)/1e2);          

		
		//LLA_FLOAT_OF_BFP(state.ned_origin_f.lla, state.ned_origin_i.lla);
		{                   
		    (state.ned_origin_f.lla).lat = (float)(((state.ned_origin_i.lla).lat)/1e7);    
		    (state.ned_origin_f.lla).lon = (float)(((state.ned_origin_i.lla).lon)/1e7);    
		    (state.ned_origin_f.lla).alt = (float)(((state.ned_origin_i.lla).alt)/1e3);          
		  }
		
		
		//HIGH_RES_RMAT_FLOAT_OF_BFP(state.ned_origin_f.ltp_of_ecef, state.ned_origin_i.ltp_of_ecef);
		{                 
		    (state.ned_origin_f.ltp_of_ecef).m[0] = ((float)((state.ned_origin_i.ltp_of_ecef).m[0])/(1<<(20))); 
		    (state.ned_origin_f.ltp_of_ecef).m[1] = ((float)((state.ned_origin_i.ltp_of_ecef).m[1])/(1<<(20))); 
		    (state.ned_origin_f.ltp_of_ecef).m[2] = ((float)((state.ned_origin_i.ltp_of_ecef).m[2])/(1<<(20))); 
		    (state.ned_origin_f.ltp_of_ecef).m[3] = ((float)((state.ned_origin_i.ltp_of_ecef).m[3])/(1<<(20))); 
		    (state.ned_origin_f.ltp_of_ecef).m[4] = ((float)((state.ned_origin_i.ltp_of_ecef).m[4])/(1<<(20))); 
		    (state.ned_origin_f.ltp_of_ecef).m[5] = ((float)((state.ned_origin_i.ltp_of_ecef).m[5])/(1<<(20))); 
		    (state.ned_origin_f.ltp_of_ecef).m[6] = ((float)((state.ned_origin_i.ltp_of_ecef).m[6])/(1<<(20))); 
		    (state.ned_origin_f.ltp_of_ecef).m[7] = ((float)((state.ned_origin_i.ltp_of_ecef).m[7])/(1<<(20))); 
		    (state.ned_origin_f.ltp_of_ecef).m[8] = ((float)((state.ned_origin_i.ltp_of_ecef).m[8])/(1<<(20))); 
		  }
		
		state.ned_origin_f.hmsl = (float)((state.ned_origin_i.hmsl)/1e3);//M_OF_MM(state.ned_origin_i.hmsl);

		/* clear bits for all local frame representations */
		state.pos_status &= ~(POS_LOCAL_COORD);
		state.speed_status &= ~(SPEED_LOCAL_COORD);
		ClearBit(state.accel_status, ACCEL_NED_I);
		ClearBit(state.accel_status, ACCEL_NED_F);

		state.ned_initialized_i = true;
		state.ned_initialized_f = true;
	}
	
	public static void stateSetLocalUtmOrigin_f(UtmCoor_f utm_def) {
		  //memcpy(&state.utm_origin_f, utm_def, sizeof( UtmCoor_f));
		state.utm_origin_f = utm_def.clone();  
		state.utm_initialized_f = true;

		/* clear bits for all local frame representations */
		state.pos_status &= ~(POS_LOCAL_COORD);
		state.speed_status &= ~(SPEED_LOCAL_COORD);
		ClearBit(state.accel_status, ACCEL_NED_I);
		ClearBit(state.accel_status, ACCEL_NED_F);
	}
	
	// Test if local coordinates are valid.
	public static boolean stateIsLocalCoordinateValid() {
		return ((state.ned_initialized_i || state.ned_initialized_f || state.utm_initialized_f) && ((state.pos_status  & (POS_LOCAL_COORD )) != 0));
	}
	/// Test if global coordinates are valid.
	public static boolean stateIsGlobalCoordinateValid() {
		  return ((state.pos_status & (POS_GLOBAL_COORD)) !=0 || stateIsLocalCoordinateValid());
	}

	/// Set position from ECEF coordinates (int).
	public static void stateSetPositionEcef_i( EcefCoor_i ecef_pos) {
	  INT32_VECT3_COPY(state.ecef_pos_i, ecef_pos);
//	  {        
//		    (state.ecef_pos_i).x = ecef_pos.x;				
//		    (state.ecef_pos_i).y = ecef_pos.y;				
//		    (state.ecef_pos_i).z = ecef_pos.z;				
//		  }
	  
	  /* clear bits for all position representations and only set the new one */
	  state.pos_status = (1 << POS_ECEF_I);
	}

	/// Set position from local NED coordinates (int).
	public static void stateSetPositionNed_i(NedCoor_i ned_pos) {
	  
		INT32_VECT3_COPY(state.ned_pos_i, ned_pos);
//		{        
//		    (state.ned_pos_i).x = ned_pos.x;				
//		    (state.ned_pos_i).y = ned_pos.y;				
//		    (state.ned_pos_i).z = ned_pos.z;				
//		  }
		
		
		/* clear bits for all position representations and only set the new one */
	  state.pos_status = (1 << POS_NED_I);
	}

	/// Set position from local ENU coordinates (int).
	public static void stateSetPositionEnu_i( EnuCoor_i enu_pos) {
	  INT32_VECT3_COPY(state.enu_pos_i, enu_pos);
//	  
//	  {        
//		    (state.enu_pos_i).x = enu_pos.x;				
//		    (state.enu_pos_i).y = enu_pos.y;				
//		    (state.enu_pos_i).z = enu_pos.z;				
//		  }
	  /* clear bits for all position representations and only set the new one */
	  state.pos_status = (1 << POS_ENU_I);
	}

	/// Set position from LLA coordinates (int).
	public static  void stateSetPositionLla_i( LlaCoor_i lla_pos) {
	  LLA_COPY(state.lla_pos_i, lla_pos);
	  
//		{			\
//		    (state.lla_pos_i).lat = lla_pos.lat;			\
//		    (state.lla_pos_i).lon = lla_pos.lon;			\
//		    (state.lla_pos_i).alt = lla_pos.alt;			\
//		}
		
		/* clear bits for all position representations and only set the new one */
	  state.pos_status = (1 << POS_LLA_I);
	}

	/// Set multiple position coordinates (int).
	public static  void stateSetPosition_i(
	     EcefCoor_i ecef_pos,
	     NedCoor_i ned_pos,
	     EnuCoor_i enu_pos,
	     LlaCoor_i lla_pos) {
	  /* clear all status bit */
	  state.pos_status = 0;
	  if (ecef_pos != null) {
	   INT32_VECT3_COPY(state.ecef_pos_i, ecef_pos);
//	    {        \
//		    (state.ecef_pos_i).x = ecef_pos.x;				\
//		    (state.ecef_pos_i).y = ecef_pos.y;				\
//		    (state.ecef_pos_i).z = ecef_pos.z;				\
//		  }
	    
	    state.pos_status |= (1 << POS_ECEF_I);
	  }
	  if (ned_pos != null) {
	    INT32_VECT3_COPY(state.ned_pos_i, ned_pos);
//		  {        \
//			    (state.ned_pos_i).x = ned_pos.x;				\
//			    (state.ned_pos_i).y = ned_pos.y;				\
//			    (state.ned_pos_i).z = ned_pos.z;				\
//			  }
			
	    
	    state.pos_status |= (1 << POS_NED_I);
	  }
	  if (enu_pos != null) {
	    INT32_VECT3_COPY(state.enu_pos_i, enu_pos);
//		  {        \
//			    (state.enu_pos_i).x = enu_pos.x;				\
//			    (state.enu_pos_i).y = enu_pos.y;				\
//			    (state.enu_pos_i).z = enu_pos.z;				\
//			  }
		  
	    state.pos_status |= (1 << POS_ENU_I);
	  }
	  if (lla_pos != null) {
	   LLA_COPY(state.lla_pos_i, lla_pos);
//		  {			\
//			    (state.lla_pos_i).lat = lla_pos.lat;			\
//			    (state.lla_pos_i).lon = lla_pos.lon;			\
//			    (state.lla_pos_i).alt = lla_pos.alt;			\
//			}
		  
		  
		  state.pos_status |= (1 << POS_LLA_I);
	  }
	}

	/// Set position from UTM coordinates (float).
	public static void stateSetPositionUtm_f(UtmCoor_f utm_pos) {
	  //memcpy(&state.utm_pos_f, utm_pos, sizeof( UtmCoor_f));
		state.utm_pos_f = utm_pos.clone();
	  /* clear bits for all position representations and only set the new one */
	  state.pos_status = (1 << POS_UTM_F);
	}

	/// Set position from ECEF coordinates (float).
	public static void stateSetPositionEcef_f(EcefCoor_f ecef_pos) {
	  Pprz_algebra_float.VECT3_COPY(state.ecef_pos_f, ecef_pos);
//	  {        \
//		    (state.ecef_pos_f).x = ecef_pos.x;				\
//		    (state.ecef_pos_f).y = ecef_pos.y;				\
//		    (state.ecef_pos_f).z = ecef_pos.z;				\
//		  }
	  
	  
	  /* clear bits for all position representations and only set the new one */
	  state.pos_status = (1 << POS_ECEF_F);
	}

	/// Set position from local NED coordinates (float).
	public static  void stateSetPositionNed_f( NedCoor_f ned_pos) {
		Pprz_algebra_float.VECT3_COPY(state.ned_pos_f, ned_pos);
//	  {        \
//		    (state.ned_pos_f).x = ned_pos.x;				\
//		    (state.ned_pos_f).y = ned_pos.y;				\
//		    (state.ned_pos_f).z = ned_pos.z;				\
//		  }
		
	  
	  
	  /* clear bits for all position representations and only set the new one */
	  state.pos_status = (1 << POS_NED_F);
	}

	/// Set position from local ENU coordinates (float).
	public static  void stateSetPositionEnu_f( EnuCoor_f enu_pos) {
		Pprz_algebra_float.VECT3_COPY(state.enu_pos_f, enu_pos);
//	  {        \
//		    (state.enu_pos_f).x = enu_pos.x;				\
//		    (state.enu_pos_f).y = enu_pos.y;				\
//		    (state.enu_pos_f).z = enu_pos.z;				\
//		  }
	  /* clear bits for all position representations and only set the new one */
	  state.pos_status = (1 << POS_ENU_F);
	}

	/// Set position from LLA coordinates (float).
	public static  void stateSetPositionLla_f( LlaCoor_f lla_pos) {
	  LLA_COPY(state.lla_pos_f, lla_pos);
//	  {			\
//		    (state.lla_pos_f).lat = lla_pos.lat;			\
//		    (state.lla_pos_f).lon = lla_pos.lon;			\
//		    (state.lla_pos_f).alt = lla_pos.alt;			\
//		}
	  
	  
	  
	  /* clear bits for all position representations and only set the new one */
	  state.pos_status = (1 << POS_LLA_F);
	}

	/// Set multiple position coordinates (float).
	public static  void stateSetPosition_f(
			EcefCoor_f ecef_pos,
			NedCoor_f ned_pos,
			EnuCoor_f enu_pos,
			LlaCoor_f lla_pos,
			UtmCoor_f utm_pos) {
		/* clear all status bit */
		state.pos_status = 0;
		if (ecef_pos != null) {
			Pprz_algebra_float.VECT3_COPY(state.ecef_pos_f, ecef_pos);
//			 {        \
//				    (state.ecef_pos_f).x = ecef_pos.x;				\
//				    (state.ecef_pos_f).y = ecef_pos.y;				\
//				    (state.ecef_pos_f).z = ecef_pos.z;				\
//				  }
			
			state.pos_status |= (1 << POS_ECEF_F);
		}
		if (ned_pos != null) {
			Pprz_algebra_float.VECT3_COPY(state.ned_pos_f, ned_pos);
//			{        \
//			    (state.ned_pos_f).x = ned_pos.x;				\
//			    (state.ned_pos_f).y = ned_pos.y;				\
//			    (state.ned_pos_f).z = ned_pos.z;				\
//			  }
			
			
			state.pos_status |= (1 << POS_NED_F);
		}
		if (enu_pos != null) {
			Pprz_algebra_float.VECT3_COPY(state.enu_pos_f, enu_pos);
//			{        \
//			    (state.enu_pos_f).x = enu_pos.x;				\
//			    (state.enu_pos_f).y = enu_pos.y;				\
//			    (state.enu_pos_f).z = enu_pos.z;				\
//			  }
			
			state.pos_status |= (1 << POS_ENU_F);
		}
		if (lla_pos != null) {
			LLA_COPY(state.lla_pos_f, lla_pos);
//			{			\
//			    (state.lla_pos_f).lat = lla_pos.lat;			\
//			    (state.lla_pos_f).lon = lla_pos.lon;			\
//			    (state.lla_pos_f).alt = lla_pos.alt;			\
//			}
//			
			
			state.pos_status |= (1 << POS_LLA_F);
		}
		if (utm_pos != null) {
			//memcpy(state.utm_pos_f, utm_pos, sizeof( UtmCoor_f));
			state.utm_pos_f = utm_pos.clone();
			state.pos_status |= (1 << POS_UTM_F);
		}
	}

	/* *********************** Get functions *************************** */

	/// Get position in ECEF coordinates (int).
	public static   EcefCoor_i stateGetPositionEcef_i() {
	  if (!Std.bit_is_set(state.pos_status, POS_ECEF_I))
	    stateCalcPositionEcef_i();
	  return state.ecef_pos_i;
	}

	/// Get position in local NED coordinates (int).
	public static   NedCoor_i stateGetPositionNed_i() {
	  if (!Std.bit_is_set(state.pos_status, POS_NED_I))
	    stateCalcPositionNed_i();
	  return state.ned_pos_i;
	}

	/// Get position in local ENU coordinates (int).
	public static   EnuCoor_i stateGetPositionEnu_i() {
	  if (!Std.bit_is_set(state.pos_status, POS_ENU_I))
	    stateCalcPositionEnu_i();
	  return state.enu_pos_i;
	}

	/// Get position in LLA coordinates (int).
	public static   LlaCoor_i stateGetPositionLla_i() {
	  if (!Std.bit_is_set(state.pos_status, POS_LLA_I))
	    stateCalcPositionLla_i();
	  return state.lla_pos_i;
	}

	/// Get position in UTM coordinates (float).
	public static   UtmCoor_f stateGetPositionUtm_f() {
	  if (!Std.bit_is_set(state.pos_status, POS_UTM_F))
	    stateCalcPositionUtm_f();
	  return state.utm_pos_f;
	}

	/// Get position in ECEF coordinates (float).
	public static   EcefCoor_f stateGetPositionEcef_f() {
	  if (!Std.bit_is_set(state.pos_status, POS_ECEF_F))
	    stateCalcPositionEcef_f();
	  return state.ecef_pos_f;
	}

	/// Get position in local NED coordinates (float).
	public static   NedCoor_f stateGetPositionNed_f() {
	  if (!Std.bit_is_set(state.pos_status, POS_NED_F))
	    stateCalcPositionNed_f();
	  return state.ned_pos_f;
	}

	/// Get position in local ENU coordinates (float).
	public static   EnuCoor_f stateGetPositionEnu_f() {
	  if (!Std.bit_is_set(state.pos_status, POS_ENU_F))
	    stateCalcPositionEnu_f();
	  return state.enu_pos_f;
	}

	/// Get position in LLA coordinates (float).
	public static   LlaCoor_f stateGetPositionLla_f() {
	  if (!Std.bit_is_set(state.pos_status, POS_LLA_F))
	    stateCalcPositionLla_f();
	  return state.lla_pos_f;
	}

	/** @}*/



	/* ****************************************************************************
	 *                                                                            *
	 * Set and Get functions for the SPEED representations                        *
	 *                                                                            *
	 *************************************************************************** */
	

	/* ********************** Set functions ************************** */

	// Set ground speed in local NED coordinates (int).
	public static  void stateSetSpeedNed_i( NedCoor_i ned_speed) {
	  INT32_VECT3_COPY(state.ned_speed_i, ned_speed);
//	  {			\
//		    (state.ned_speed_i).x = ned_speed.x ;			\
//		    (state.ned_speed_i).y = ned_speed.y;			\
//		    (state.ned_speed_i).z = ned_speed.z;			\
//		}
//	  
	  /* clear bits for all speed representations and only set the new one */
	  state.speed_status = (1 << SPEED_NED_I);
	}

	// Set ground speed in local ENU coordinates (int).
	public static  void stateSetSpeedEnu_i( EnuCoor_i enu_speed) {
	  INT32_VECT3_COPY(state.enu_speed_i, enu_speed);
//	  {			\
//		    (state.enu_speed_i).x = enu_speed.x ;			\
//		    (state.enu_speed_i).y = enu_speed.y;			\
//		    (state.enu_speed_i).z = enu_speed.z;			\
//		}
	  
	  /* clear bits for all speed representations and only set the new one */
	  state.speed_status = (1 << SPEED_ENU_I);
	}

	/// Set ground speed in ECEF coordinates (int).
	public static  void stateSetSpeedEcef_i( EcefCoor_i ecef_speed) {
	 INT32_VECT3_COPY(state.ecef_speed_i, ecef_speed);
//		{			\
//		    (state.ecef_speed_i).x = ecef_speed.x ;			\
//		    (state.ecef_speed_i).y = ecef_speed.y;			\
//		    (state.ecef_speed_i).z = ecef_speed.z;			\
//		}
//		
		/* clear bits for all speed representations and only set the new one */
	  state.speed_status = (1 << SPEED_ECEF_I);
	}

	/// Set multiple speed coordinates (int).
	public static  void stateSetSpeed_i(
	     EcefCoor_i ecef_speed,
	     NedCoor_i ned_speed,
	     EnuCoor_i enu_speed) {
	  /* clear all status bit */
	  state.speed_status = 0;
	  if (ecef_speed!= null) {
	  INT32_VECT3_COPY(state.ecef_speed_i, ecef_speed);
//		  {			
//			    (state.ecef_speed_i).x = ecef_speed.x ;			
//			    (state.ecef_speed_i).y = ecef_speed.y;			
//			    (state.ecef_speed_i).z = ecef_speed.z;			
//			}
		  state.speed_status |= (1 << SPEED_ECEF_I);
	  }
	  if (ned_speed!= null) {
	    INT32_VECT3_COPY(state.ned_speed_i, ned_speed);
//		  {			\
//			    (state.ned_speed_i).x = ned_speed.x ;			\
//			    (state.ned_speed_i).y = ned_speed.y;			\
//			    (state.ned_speed_i).z = ned_speed.z;			\
//			}
		  state.speed_status |= (1 << SPEED_NED_I);
	  }
	  if (enu_speed!= null) {
	    INT32_VECT3_COPY(state.enu_speed_i, enu_speed);
//		  {			\
//			    (state.enu_speed_i).x = enu_speed.x ;			\
//			    (state.enu_speed_i).y = enu_speed.y;			\
//			    (state.enu_speed_i).z = enu_speed.z;			\
//			}
		  state.speed_status |= (1 << SPEED_ENU_I);
	  }
	}

	/// Set ground speed in local NED coordinates (float).
	public static  void stateSetSpeedNed_f( NedCoor_f ned_speed) {
		Pprz_algebra_float.VECT3_COPY(state.ned_speed_f, ned_speed);
//		 {			
//			    (state.ned_speed_f).x = ned_speed.x ;			
//			    (state.ned_speed_f).y = ned_speed.y;			
//			    (state.ned_speed_f).z = ned_speed.z;			
//			}
		
		/* clear bits for all speed representations and only set the new one */
	  state.speed_status = (1 << SPEED_NED_F);
	}

	/// Set ground speed in local ENU coordinates (float).
	public static  void stateSetSpeedEnu_f( EnuCoor_f enu_speed) {
		Pprz_algebra_float.VECT3_COPY(state.enu_speed_f, enu_speed);
//		{			
//		    (state.enu_speed_f).x = enu_speed.x ;			
//		    (state.enu_speed_f).y = enu_speed.y;			
//		    (state.enu_speed_f).z = enu_speed.z;			
//		}
		
		/* clear bits for all speed representations and only set the new one */
	  state.speed_status = (1 << SPEED_ENU_F);
	}

	/// Set ground speed in ECEF coordinates (float).
	public static  void stateSetSpeedEcef_f( EcefCoor_f ecef_speed) {
		Pprz_algebra_float.VECT3_COPY(state.ecef_speed_f, ecef_speed);
//		 {			\
//			    (state.ecef_speed_f).x = ecef_speed.x ;			\
//			    (state.ecef_speed_f).y = ecef_speed.y;			\
//			    (state.ecef_speed_f).z = ecef_speed.z;			\
//			}
		
		/* clear bits for all speed representations and only set the new one */
	  state.speed_status = (1 << SPEED_ECEF_F);
	}

	/// Set multiple speed coordinates (float).
	public static  void stateSetSpeed_f(
	     EcefCoor_f ecef_speed,
	     NedCoor_f ned_speed,
	     EnuCoor_f enu_speed) {
	  /* clear all status bit */
	  state.speed_status = 0;
	  if (ecef_speed != null) {
		  Pprz_algebra_float.VECT3_COPY(state.ecef_speed_f, ecef_speed);
//		  {			\
//			    (state.ecef_speed_f).x = ecef_speed.x ;			\
//			    (state.ecef_speed_f).y = ecef_speed.y;			\
//			    (state.ecef_speed_f).z = ecef_speed.z;			\
//			}
		  state.speed_status |= (1 << SPEED_ECEF_F);
	  }
	  if (ned_speed != null) {
		  Pprz_algebra_float.VECT3_COPY(state.ned_speed_f, ned_speed);
//		  {			\
//			    (state.ned_speed_f).x = ned_speed.x ;			\
//			    (state.ned_speed_f).y = ned_speed.y;			\
//			    (state.ned_speed_f).z = ned_speed.z;			\
//			}
		  state.speed_status |= (1 << SPEED_NED_F);
	  }
	  if (enu_speed != null) {
		  Pprz_algebra_float.VECT3_COPY(state.enu_speed_f, enu_speed);
//		  {			\
//			    (state.enu_speed_f).x = enu_speed.x ;			\
//			    (state.enu_speed_f).y = enu_speed.y;			\
//			    (state.enu_speed_f).z = enu_speed.z;			\
//			}
		  
		  state.speed_status |= (1 << SPEED_ENU_F);
	  }
	}

	/* ********************** Get functions ************************** */

	/// Get ground speed in local NED coordinates (int).
	public static   NedCoor_i stateGetSpeedNed_i() {
	  if (!Std.bit_is_set(state.speed_status, SPEED_NED_I))
	    stateCalcSpeedNed_i();
	  return state.ned_speed_i;
	}

	/// Get ground speed in local ENU coordinates (int).
	public static   EnuCoor_i stateGetSpeedEnu_i() {
	  if (!Std.bit_is_set(state.speed_status, SPEED_ENU_I))
	    stateCalcSpeedEnu_i();
	  return state.enu_speed_i;
	}

	/// Get ground speed in ECEF coordinates (int).
	public static   EcefCoor_i stateGetSpeedEcef_i() {
	  if (!Std.bit_is_set(state.speed_status, SPEED_ECEF_I))
	    stateCalcSpeedEcef_i();
	  return state.ecef_speed_i;
	}

	/// Get norm of horizontal ground speed (int).
	public static  long stateGetHorizontalSpeedNorm_i() {
	  if (!Std.bit_is_set(state.speed_status, SPEED_HNORM_I))
	    stateCalcHorizontalSpeedNorm_i();
	  return state.h_speed_norm_i;
	}

	/// Get dir of horizontal ground speed (int).
	public static  long stateGetHorizontalSpeedDir_i() {
	  if (!Std.bit_is_set(state.speed_status, SPEED_HDIR_I))
	    stateCalcHorizontalSpeedDir_i();
	  return state.h_speed_dir_i;
	}

	/// Get ground speed in local NED coordinates (float).
	public static   NedCoor_f stateGetSpeedNed_f() {
	  if (!Std.bit_is_set(state.speed_status, SPEED_NED_F))
	    stateCalcSpeedNed_f();
	  return state.ned_speed_f;
	}

	/// Get ground speed in local ENU coordinates (float).
	public static   EnuCoor_f stateGetSpeedEnu_f() {
	  if (!Std.bit_is_set(state.speed_status, SPEED_ENU_F))
	    stateCalcSpeedEnu_f();
	  return state.enu_speed_f;
	}

	/// Get ground speed in ECEF coordinates (float).
	public static   EcefCoor_f stateGetSpeedEcef_f() {
	  if (!Std.bit_is_set(state.speed_status, SPEED_ECEF_F))
	    stateCalcSpeedEcef_f();
	  return state.ecef_speed_f;
	}

	/// Get norm of horizontal ground speed (float).
	public static  float stateGetHorizontalSpeedNorm_f() {
	  if (!Std.bit_is_set(state.speed_status, SPEED_HNORM_F))
	    stateCalcHorizontalSpeedNorm_f();
	  return state.h_speed_norm_f;
	}

	/// Get dir of horizontal ground speed (float).
	public static  float stateGetHorizontalSpeedDir_f() {
	  if (!Std.bit_is_set(state.speed_status, SPEED_HDIR_F))
	    stateCalcHorizontalSpeedDir_f();
	  return state.h_speed_dir_f;
	}
	/** @}*/



	/* ****************************************************************************
	 *                                                                            *
	 * Set and Get functions for the ACCELERATION representations                 *
	 *                                                                            *
	 *************************************************************************** */
	/*  @addtogroup state_acceleration
	 *  @{ */

	

	/* ******************** validity test functions *************** **/

	/// Test if accelerations are valid.
	public static  boolean stateIsAccelValid() {
	  return (state.accel_status==0);
	}

	/* ********************** Set functions ****************************/

	/// Set acceleration in NED coordinates (int).
	public static  void stateSetAccelNed_i( NedCoor_i ned_accel) {
	  INT32_VECT3_COPY(state.ned_accel_i, ned_accel);
	  /* clear bits for all accel representations and only set the new one */
	  state.accel_status = (1 << ACCEL_NED_I);
	}

	/// Set acceleration in ECEF coordinates (int).
	public static  void stateSetAccelEcef_i( EcefCoor_i ecef_accel) {
	  INT32_VECT3_COPY(state.ecef_accel_i, ecef_accel);
	  /* clear bits for all accel representations and only set the new one */
	  state.accel_status = (1 << ACCEL_ECEF_I);
	}

	/// Set acceleration in NED coordinates (float).
	public static  void stateSetAccelNed_f( NedCoor_f ned_accel) {
	  Pprz_algebra_float.VECT3_COPY(state.ned_accel_f, ned_accel);
	  /* clear bits for all accel representations and only set the new one */
	  state.accel_status = (1 << ACCEL_NED_F);
	}

	/// Set acceleration in ECEF coordinates (float).
	public static  void stateSetAccelEcef_f( EcefCoor_f ecef_accel) {
		Pprz_algebra_float.VECT3_COPY(state.ecef_accel_f, ecef_accel);
	  /* clear bits for all accel representations and only set the new one */
	  state.accel_status = (1 << ACCEL_ECEF_F);
	}

	/* ********************** Get functions ************************* */

	/// Get acceleration in NED coordinates (int).
	public static   NedCoor_i stateGetAccelNed_i() {
	  if (!Std.bit_is_set(state.accel_status, ACCEL_NED_I))
	    stateCalcAccelNed_i();
	  return state.ned_accel_i;
	}

	/// Get acceleration in ECEF coordinates (int).
	public static   EcefCoor_i stateGetAccelEcef_i() {
	  if (!Std.bit_is_set(state.accel_status, ACCEL_ECEF_I))
	    stateCalcAccelEcef_i();
	  return state.ecef_accel_i;
	}

	/// Get acceleration in NED coordinates (float).
	public static   NedCoor_f stateGetAccelNed_f() {
	  if (!Std.bit_is_set(state.accel_status, ACCEL_NED_F))
	    stateCalcAccelNed_f();
	  return state.ned_accel_f;
	}

	/// Get acceleration in ECEF coordinates (float).
	public static   EcefCoor_f stateGetAccelEcef_f() {
	  if (!Std.bit_is_set(state.accel_status, ACCEL_ECEF_F))
	    stateCalcAccelEcef_f();
	  return state.ecef_accel_f;
	}
	/* @}*/

	/* ****************************************************************************
	*                                                                             *
	* Set and Get functions for the ATTITUDE representations                      *
	* (Calls the functions in math/pprz_orientation_conversion)                   *
	*                                                                             *
	************************************************************************* **/
	/* addtogroup state_attitude
	* @{ */
	/* ****************** validity test functions **************** */

	/// Test if attitudes are valid.
	public static  boolean stateIsAttitudeValid() {
	  return (orientationCheckValid(state.ned_to_body_orientation));
	}

	/* ******************** Set functions ************************* */

	/// Set vehicle body attitude from quaternion (int).
	public static  void stateSetNedToBodyQuat_i( Int32Quat ned_to_body_quat) {
	  orientationSetQuat_i(state.ned_to_body_orientation,ned_to_body_quat);
	}

	/// Set vehicle body attitude from rotation matrix (int).
	public static  void stateSetNedToBodyRMat_i( Int32RMat ned_to_body_rmat) {
	  orientationSetRMat_i(state.ned_to_body_orientation,ned_to_body_rmat);
	}

	/// Set vehicle body attitude from euler angles (int).
	public static  void stateSetNedToBodyEulers_i( Int32Eulers ned_to_body_eulers) {
	  orientationSetEulers_i(state.ned_to_body_orientation,ned_to_body_eulers);
	}

	/// Set vehicle body attitude from quaternion (float).
	public static  void stateSetNedToBodyQuat_f( FloatQuat ned_to_body_quat) {
	  orientationSetQuat_f(state.ned_to_body_orientation,ned_to_body_quat);
	}

	/// Set vehicle body attitude from rotation matrix (float).
	public static  void stateSetNedToBodyRMat_f( FloatRMat ned_to_body_rmat) {
	  orientationSetRMat_f(state.ned_to_body_orientation,ned_to_body_rmat);
	}

	/// Set vehicle body attitude from euler angles (float).
	public static  void stateSetNedToBodyEulers_f( FloatEulers ned_to_body_eulers) {
	  orientationSetEulers_f(state.ned_to_body_orientation,ned_to_body_eulers);
	}

	/*  ********************* Get functions ***************************/

	/// Get vehicle body attitude quaternion (int).
	public static   Int32Quat stateGetNedToBodyQuat_i() {
	  return orientationGetQuat_i(state.ned_to_body_orientation);
	}

	/// Get vehicle body attitude rotation matrix (int).
	public static   Int32RMat stateGetNedToBodyRMat_i() {
	  return orientationGetRMat_i(state.ned_to_body_orientation);
	}

	/// Get vehicle body attitude euler angles (int).
	public static   Int32Eulers stateGetNedToBodyEulers_i() {
	  return orientationGetEulers_i(state.ned_to_body_orientation);
	}

	/// Get vehicle body attitude quaternion (float).
	public static   FloatQuat stateGetNedToBodyQuat_f() {
	  return orientationGetQuat_f(state.ned_to_body_orientation);
	}

	/// Get vehicle body attitude rotation matrix (float).
	public static   FloatRMat stateGetNedToBodyRMat_f() {
	  return orientationGetRMat_f(state.ned_to_body_orientation);
	}

	/// Get vehicle body attitude euler angles (float).
	public static   FloatEulers stateGetNedToBodyEulers_f() {
	  return orientationGetEulers_f(state.ned_to_body_orientation);
	}


	/// Test if rates are valid.
	public static  boolean stateIsRateValid() {
	  return (state.rate_status==1);   //????????????
	}

	/* ********************* Set functions ************************* */

	/// Set vehicle body angular rate (int).
	public static  void stateSetBodyRates_i( Int32Rates body_rate) {
	  RATES_COPY(state.body_rates_i, body_rate);
	  /* clear bits for all attitude representations and only set the new one */
	  state.rate_status = (1 << RATE_I);
	}

	/// Set vehicle body angular rate (float).
	public static  void stateSetBodyRates_f( FloatRates body_rate) {
	  RATES_COPY(state.body_rates_f, body_rate);
	  /* clear bits for all attitude representations and only set the new one */
	  state.rate_status = (1 << RATE_F);
	}

	/* ******************** Get functions ************************ */

	/// Get vehicle body angular rate (int).
	public static   Int32Rates stateGetBodyRates_i() {
	  if (!Std.bit_is_set(state.rate_status, RATE_I))
	    stateCalcBodyRates_i();
	  return state.body_rates_i;
	}

	/// Get vehicle body angular rate (float).
	public static   FloatRates stateGetBodyRates_f() {
	  if (!Std.bit_is_set(state.rate_status, RATE_F))
	    stateCalcBodyRates_f();
	  return state.body_rates_f;
	}

	

	/// test if wind speed is available.
	public static  boolean stateIsWindspeedValid() {
	  return (state.wind_air_status &= ~((1<<WINDSPEED_I)|(1<<WINDSPEED_F))) != 0;
	}

	/// test if air speed is available.
	public static  boolean stateIsAirspeedValid() {
	  return (state.wind_air_status &= ~((1<<AIRSPEED_I)|(1<<AIRSPEED_F))) !=0;
	}

	/// test if angle of attack is available.
	public static  boolean stateIsAngleOfAttackValid() {
	  return (state.wind_air_status &= ~(1<<AOA_F))!=0;
	}

	/// test if sideslip is available.
	public static  boolean stateIsSideslipValid() {
	  return (state.wind_air_status &= ~(1<<SIDESLIP_F))!=0;
	}

	/************************ Set functions ****************************/

	/// Set horizontal windspeed (int).
	public static  void stateSetHorizontalWindspeed_i( Int32Vect2 h_windspeed) {
	  Pprz_algebra.VECT2_COPY(state.h_windspeed_i, h_windspeed);
	  /* clear bits for all windspeed representations and only set the new one */
	  Std.ClearBit(state.wind_air_status, WINDSPEED_F);
	  Std.SetBit(state.wind_air_status, WINDSPEED_I);
	}

	/// Set airspeed (int).
	public static  void stateSetAirspeed_i(int airspeed) {
	  state.airspeed_i = airspeed;
	  /* clear bits for all airspeed representations and only set the new one */
	  Std.ClearBit(state.wind_air_status, AIRSPEED_F);
	  Std.SetBit(state.wind_air_status, AIRSPEED_I);
	}

	/// Set horizontal windspeed (float).
	public static  void stateSetHorizontalWindspeed_f( FloatVect2 h_windspeed) {
	  Pprz_algebra_float.VECT2_COPY(state.h_windspeed_f, h_windspeed);
	  /* clear bits for all windspeed representations and only set the new one */
	  Std.ClearBit(state.wind_air_status, WINDSPEED_I);
	  Std.SetBit(state.wind_air_status, WINDSPEED_F);
	}

	/// Set airspeed (float).
	public static  void stateSetAirspeed_f(float airspeed) {
	  state.airspeed_f = airspeed;
	  /* clear bits for all airspeed representations and only set the new one */
	  Std.ClearBit(state.wind_air_status, AIRSPEED_I);
	  Std.SetBit(state.wind_air_status, AIRSPEED_F);
	}

	/// Set angle of attack in radians (float).
	public static  void stateSetAngleOfAttack_f(float aoa) {
	  state.angle_of_attack_f = aoa;
	  /* clear bits for all AOA representations and only set the new one */
	  /// @todo no integer yet
	  Std.SetBit(state.wind_air_status, AOA_F);
	}

	/// Set sideslip angle in radians (float).
	public static  void stateSetSideslip_f(float sideslip) {
	  state.sideslip_f = sideslip;
	  /* clear bits for all sideslip representations and only set the new one */
	  // @todo no integer yet
	  Std.SetBit(state.wind_air_status, SIDESLIP_F);
	}

	

	/// Get horizontal windspeed (int).
	public static   Int32Vect2 stateGetHorizontalWindspeed_i() {
	  if (!Std.bit_is_set(state.wind_air_status, WINDSPEED_I))
	    stateCalcHorizontalWindspeed_i();
	  return state.h_windspeed_i;
	}

	/// Get airspeed (int).
	public static  int stateGetAirspeed_i() {
	  if (!Std.bit_is_set(state.wind_air_status, AIRSPEED_I))
	    stateCalcAirspeed_i();
	  return state.airspeed_i;
	}

	/// Get horizontal windspeed (float).
	public static   FloatVect2 stateGetHorizontalWindspeed_f() {
	  if (!Std.bit_is_set(state.wind_air_status, WINDSPEED_F))
	    stateCalcHorizontalWindspeed_f();
	  return state.h_windspeed_f;
	}

	/// Get airspeed (float).
	public static  float stateGetAirspeed_f() {
	  if (!Std.bit_is_set(state.wind_air_status, AIRSPEED_F))
	    stateCalcAirspeed_f();
	  return state.airspeed_f;
	}

	/// Get angle of attack (float).
	public static  float stateGetAngleOfAttack_f() {
	  ///  @todo only float for now
	//  if (!bit_is_set(state.wind_air_status, AOA_F))
//	    stateCalcAOA_f();
	  return state.angle_of_attack_f;
	}

	/// Get sideslip (float).
	public static  float stateGetSideslip_f() {
	  ///  @todo only float for now
	//  if (!bit_is_set(state.wind_air_status, SIDESLIP_F))
//	    stateCalcSideslip_f();
	  return state.sideslip_f;
	}
	
//---------------------------------------------------------state.c--------------------------------------------
	public static void stateInit(){
		state.pos_status = 0;
		  state.speed_status = 0;
		  state.accel_status = 0;
		  state.ned_to_body_orientation.status = 0;
		  state.rate_status = 0;
		  state.wind_air_status = 0;
		  state.ned_initialized_i = false;
		  state.ned_initialized_f = false;
		  state.utm_initialized_f = false;
	}
	
	public static void stateCalcPositionEcef_i() {
		if (Std.bit_is_set(state.pos_status, POS_ECEF_I))
			return;

		if (Std.bit_is_set(state.pos_status, POS_ECEF_F)) {
			Pprz_algebra.ECEF_BFP_OF_REAL(state.ecef_pos_i, state.ecef_pos_f);
			     
		}
		else if (Std.bit_is_set(state.pos_status, POS_NED_I)) {
			ecef_of_ned_pos_i(state.ecef_pos_i, state.ned_origin_i, state.ned_pos_i);
		}
		else if (bit_is_set(state.pos_status, POS_NED_F)) {
			/* transform ned_f to ecef_f, set status bit, then convert to int */
			ecef_of_ned_point_f(state.ecef_pos_f, state.ned_origin_f, state.ned_pos_f);
			SetBit(state.pos_status, POS_ECEF_F);
			Pprz_algebra.ECEF_BFP_OF_REAL(state.ecef_pos_i, state.ecef_pos_f);
		}
		else if (bit_is_set(state.pos_status, POS_LLA_F)) {
			/* transform lla_f to ecef_f, set status bit, then convert to int */
			ecef_of_lla_f(state.ecef_pos_f, state.lla_pos_f);
			SetBit(state.pos_status, POS_ECEF_F);
			ECEF_BFP_OF_REAL(state.ecef_pos_i, state.ecef_pos_f);
		}
		else if (bit_is_set(state.pos_status, POS_LLA_I)) {
			ecef_of_lla_i(state.ecef_pos_i, state.lla_pos_i);
		}
		else {
			/* could not get this representation,  set errno */
			//struct EcefCoor_i _ecef_zero = {0};
			//return _ecef_zero;
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.pos_status, POS_ECEF_I);
	}
	
	public static void stateCalcPositionNed_i() {
		if (bit_is_set(state.pos_status, POS_NED_I))
			return;

		int errno = 0;
		if (state.ned_initialized_i) {
			if (bit_is_set(state.pos_status, POS_NED_F)) {
				NED_BFP_OF_REAL(state.ned_pos_i, state.ned_pos_f);
			}
			else if (bit_is_set(state.pos_status, POS_ENU_I)) {
				INT32_VECT3_NED_OF_ENU(state.ned_pos_i, state.enu_pos_i);
			}
			else if (bit_is_set(state.pos_status, POS_ENU_F)) {
				ENU_BFP_OF_REAL(state.enu_pos_i, state.enu_pos_f);
				SetBit(state.pos_status, POS_ENU_I);
				INT32_VECT3_NED_OF_ENU(state.ned_pos_i, state.enu_pos_i);
			}
			else if (bit_is_set(state.pos_status, POS_ECEF_I)) {
				ned_of_ecef_pos_i(state.ned_pos_i, state.ned_origin_i, state.ecef_pos_i);
			}
			else if (bit_is_set(state.pos_status, POS_ECEF_F)) {
				/* transform ecef_f -> ned_f, set status bit, then convert to int */
				ned_of_ecef_point_f(state.ned_pos_f, state.ned_origin_f, state.ecef_pos_f);
				SetBit(state.pos_status, POS_NED_F);
				NED_BFP_OF_REAL(state.ned_pos_i, state.ned_pos_f);
			}
			else if (bit_is_set(state.pos_status, POS_LLA_F)) {
				/* transform lla_f -> ecef_f -> ned_f, set status bits, then convert to int */
				ecef_of_lla_f(state.ecef_pos_f, state.lla_pos_f);
				SetBit(state.pos_status, POS_ECEF_F);
				ned_of_ecef_point_f(state.ned_pos_f, state.ned_origin_f, state.ecef_pos_f);
				SetBit(state.pos_status, POS_NED_F);
				NED_BFP_OF_REAL(state.ned_pos_i, state.ned_pos_f);
			}
			else if (bit_is_set(state.pos_status, POS_LLA_I)) {
				ned_of_lla_point_i(state.ned_pos_i, state.ned_origin_i, state.lla_pos_i);
			}
			else { /* could not get this representation,  set errno */
				errno = 1;
			}
		}
		else if (state.utm_initialized_f) {
			if (bit_is_set(state.pos_status, POS_NED_F)) {
				NED_BFP_OF_REAL(state.ned_pos_i, state.ned_pos_f);
			}
			else if (bit_is_set(state.pos_status, POS_ENU_I)) {
				INT32_VECT3_NED_OF_ENU(state.ned_pos_i, state.enu_pos_i);
			}
			else if (bit_is_set(state.pos_status, POS_ENU_F)) {
				ENU_BFP_OF_REAL(state.enu_pos_i, state.enu_pos_f);
				SetBit(state.pos_status, POS_ENU_I);
				INT32_VECT3_NED_OF_ENU(state.ned_pos_i, state.enu_pos_i);
			}
			else if (bit_is_set(state.pos_status, POS_UTM_F)) {
				/* transform utm_f -> ned_f -> ned_i, set status bits */
				NED_OF_UTM_DIFF(state.ned_pos_f, state.utm_pos_f, state.utm_origin_f);
				SetBit(state.pos_status, POS_NED_F);
				NED_BFP_OF_REAL(state.ned_pos_i, state.ned_pos_f);
			}
			else if (bit_is_set(state.pos_status, POS_LLA_F)) {
				/* transform lla_f -> utm_f -> ned_f -> ned_i, set status bits */
				utm_of_lla_f(state.utm_pos_f, state.lla_pos_f);
				SetBit(state.pos_status, POS_UTM_F);
				NED_OF_UTM_DIFF(state.ned_pos_f, state.utm_pos_f, state.utm_origin_f);
				SetBit(state.pos_status, POS_NED_F);
				NED_BFP_OF_REAL(state.ned_pos_i, state.ned_pos_f);
			}
			else if (bit_is_set(state.pos_status, POS_LLA_I)) {
				/* transform lla_i -> lla_f -> utm_f -> ned_f -> ned_i, set status bits */
				LLA_FLOAT_OF_BFP(state.lla_pos_f, state.lla_pos_i);
				SetBit(state.pos_status, POS_LLA_F);
				utm_of_lla_f(state.utm_pos_f, state.lla_pos_f);
				SetBit(state.pos_status, POS_UTM_F);
				NED_OF_UTM_DIFF(state.ned_pos_f, state.utm_pos_f, state.utm_origin_f);
				SetBit(state.pos_status, POS_NED_F);
				NED_BFP_OF_REAL(state.ned_pos_i, state.ned_pos_f);
			}
			else { /* could not get this representation,  set errno */
				errno = 2;
			}
		}
		else { /* ned coordinate system not initialized,  set errno */
			errno = 3;
		}
		if (errno != 0) {
			//struct NedCoor_i _ned_zero = {0};
			//return _ned_zero;
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.pos_status, POS_NED_I);
	}
	
	public static void stateCalcPositionEnu_i() {
		if (bit_is_set(state.pos_status, POS_ENU_I))
			return;

		int errno = 0;
		if (state.ned_initialized_i) {
			if (bit_is_set(state.pos_status, POS_NED_I)) {
				INT32_VECT3_ENU_OF_NED(state.enu_pos_i, state.ned_pos_i);
			}
			else if (bit_is_set(state.pos_status, POS_ENU_F)) {
				ENU_BFP_OF_REAL(state.enu_pos_i, state.enu_pos_f);
			}
			else if (bit_is_set(state.pos_status, POS_NED_F)) {
				NED_BFP_OF_REAL(state.ned_pos_i, state.ned_pos_f);
				SetBit(state.pos_status, POS_NED_I);
				INT32_VECT3_ENU_OF_NED(state.enu_pos_i, state.ned_pos_i);
			}
			else if (bit_is_set(state.pos_status, POS_ECEF_I)) {
				enu_of_ecef_pos_i(state.enu_pos_i, state.ned_origin_i, state.ecef_pos_i);
			}
			else if (bit_is_set(state.pos_status, POS_ECEF_F)) {
				/* transform ecef_f -> enu_f, set status bit, then convert to int */
				enu_of_ecef_point_f(state.enu_pos_f, state.ned_origin_f, state.ecef_pos_f);
				SetBit(state.pos_status, POS_ENU_F);
				ENU_BFP_OF_REAL(state.enu_pos_i, state.enu_pos_f);
			}
			else if (bit_is_set(state.pos_status, POS_LLA_F)) {
				/* transform lla_f -> ecef_f -> enu_f, set status bits, then convert to int */
				ecef_of_lla_f(state.ecef_pos_f, state.lla_pos_f);
				SetBit(state.pos_status, POS_ECEF_F);
				enu_of_ecef_point_f(state.enu_pos_f, state.ned_origin_f, state.ecef_pos_f);
				SetBit(state.pos_status, POS_ENU_F);
				ENU_BFP_OF_REAL(state.enu_pos_i, state.enu_pos_f);
			}
			else if (bit_is_set(state.pos_status, POS_LLA_I)) {
				enu_of_lla_point_i(state.enu_pos_i, state.ned_origin_i, state.lla_pos_i);
			}
			else { /* could not get this representation,  set errno */
				errno = 1;
			}
		}
		else if (state.utm_initialized_f) {
			if (bit_is_set(state.pos_status, POS_ENU_F)) {
				ENU_BFP_OF_REAL(state.enu_pos_i, state.enu_pos_f);
			}
			else if (bit_is_set(state.pos_status, POS_NED_I)) {
				INT32_VECT3_ENU_OF_NED(state.enu_pos_i, state.ned_pos_i);
			}
			else if (bit_is_set(state.pos_status, POS_NED_F)) {
				NED_BFP_OF_REAL(state.ned_pos_i, state.ned_pos_f);
				SetBit(state.pos_status, POS_NED_I);
				INT32_VECT3_ENU_OF_NED(state.enu_pos_i, state.ned_pos_i);
			}
			else if (bit_is_set(state.pos_status, POS_UTM_F)) {
				/* transform utm_f -> enu_f -> enu_i , set status bits */
				ENU_OF_UTM_DIFF(state.enu_pos_f, state.utm_pos_f, state.utm_origin_f);
				SetBit(state.pos_status, POS_ENU_F);
				ENU_BFP_OF_REAL(state.enu_pos_i, state.enu_pos_f);
			}
			else if (bit_is_set(state.pos_status, POS_LLA_F)) {
				/* transform lla_f -> utm_f -> enu_f -> enu_i , set status bits */
				utm_of_lla_f(state.utm_pos_f, state.lla_pos_f);
				SetBit(state.pos_status, POS_UTM_F);
				ENU_OF_UTM_DIFF(state.enu_pos_f, state.utm_pos_f, state.utm_origin_f);
				SetBit(state.pos_status, POS_ENU_F);
				ENU_BFP_OF_REAL(state.enu_pos_i, state.enu_pos_f);
			}
			else if (bit_is_set(state.pos_status, POS_LLA_I)) {
				/* transform lla_i -> lla_f -> utm_f -> enu_f -> enu_i , set status bits */
				LLA_FLOAT_OF_BFP(state.lla_pos_f, state.lla_pos_i);
				SetBit(state.pos_status, POS_LLA_F);
				utm_of_lla_f(state.utm_pos_f, state.lla_pos_f);
				SetBit(state.pos_status, POS_UTM_F);
				ENU_OF_UTM_DIFF(state.enu_pos_f, state.utm_pos_f, state.utm_origin_f);
				SetBit(state.pos_status, POS_ENU_F);
				ENU_BFP_OF_REAL(state.enu_pos_i, state.enu_pos_f);
			}
			else { /* could not get this representation,  set errno */
				errno = 2;
			}
		}
		else { /* ned coordinate system not initialized,  set errno */
			errno = 3;
		}
		if (errno!=0) {
			//struct EnuCoor_i _enu_zero = {0};
			//return _enu_zero;
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.pos_status, POS_ENU_I);
	}

	public static void stateCalcPositionLla_i() {
		if (bit_is_set(state.pos_status, POS_LLA_I))
			return;

		if (bit_is_set(state.pos_status, POS_LLA_F)) {
			LLA_BFP_OF_REAL(state.lla_pos_i, state.lla_pos_f);
		}
		else if (bit_is_set(state.pos_status, POS_ECEF_I)) {
			lla_of_ecef_i(state.lla_pos_i, state.ecef_pos_i);
		}
		else if (bit_is_set(state.pos_status, POS_ECEF_F)) {
			/* transform ecef_f -> lla_f, set status bit, then convert to int */
			lla_of_ecef_f(state.lla_pos_f, state.ecef_pos_f);
			SetBit(state.pos_status, POS_LLA_F);
			LLA_BFP_OF_REAL(state.lla_pos_i, state.lla_pos_f);
		}
		else if (bit_is_set(state.pos_status, POS_NED_F)) {
			/* transform ned_f -> ecef_f -> lla_f -> lla_i, set status bits */
			ecef_of_ned_point_f(state.ecef_pos_f, state.ned_origin_f, state.ned_pos_f);
			SetBit(state.pos_status, POS_ECEF_F);
			lla_of_ecef_f(state.lla_pos_f, state.ecef_pos_f);
			SetBit(state.pos_status, POS_LLA_F);
			LLA_BFP_OF_REAL(state.lla_pos_i, state.lla_pos_f);
		}
		else if (bit_is_set(state.pos_status, POS_NED_I)) {
			/* transform ned_i -> ecef_i -> lla_i, set status bits */
			ecef_of_ned_pos_i(state.ecef_pos_i, state.ned_origin_i, state.ned_pos_i);
			SetBit(state.pos_status, POS_ECEF_I);
			lla_of_ecef_i(state.lla_pos_i, state.ecef_pos_i); /* uses double version internally */
		}
		else if (bit_is_set(state.pos_status, POS_UTM_F)) {
			/* transform utm_f -> lla_f -> lla_i, set status bits */
			lla_of_utm_f(state.lla_pos_f, state.utm_pos_f);
			SetBit(state.pos_status, POS_LLA_F);
			LLA_BFP_OF_REAL(state.lla_pos_i, state.lla_pos_f);
		}
		else {
			/* could not get this representation,  set errno */
			//struct LlaCoor_i _lla_zero = {0};
			//return _lla_zero;
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.pos_status, POS_LLA_I);
	}
	
	public static void stateCalcPositionUtm_f() {
		if (bit_is_set(state.pos_status, POS_UTM_F))
			return;

		if (bit_is_set(state.pos_status, POS_LLA_F)) {
			utm_of_lla_f(state.utm_pos_f, state.lla_pos_f);
		}
		else if (bit_is_set(state.pos_status, POS_LLA_I)) {
			/* transform lla_i -> lla_f -> utm_f, set status bits */
			LLA_FLOAT_OF_BFP(state.lla_pos_f, state.lla_pos_i);
			SetBit(state.pos_status, POS_LLA_F);
			utm_of_lla_f(state.utm_pos_f, state.lla_pos_f);
		}
		else if (state.utm_initialized_f) {
			if (bit_is_set(state.pos_status, POS_ENU_F)) {
				UTM_OF_ENU_ADD(state.utm_pos_f, state.enu_pos_f, state.utm_origin_f);
			}
			else if (bit_is_set(state.pos_status, POS_ENU_I)) {
				ENU_FLOAT_OF_BFP(state.enu_pos_f, state.enu_pos_i);
				SetBit(state.pos_status, POS_ENU_F);
				UTM_OF_ENU_ADD(state.utm_pos_f, state.enu_pos_f, state.utm_origin_f);
			}
			else if (bit_is_set(state.pos_status, POS_NED_F)) {
				UTM_OF_NED_ADD(state.utm_pos_f, state.ned_pos_f, state.utm_origin_f);
			}
			else if (bit_is_set(state.pos_status, POS_NED_I)) {
				NED_FLOAT_OF_BFP(state.ned_pos_f, state.ned_pos_i);
				SetBit(state.pos_status, POS_NED_F);
				UTM_OF_NED_ADD(state.utm_pos_f, state.ned_pos_f, state.utm_origin_f);
			}
		}
		else {
			/* could not get this representation,  set errno */
			//struct EcefCoor_f _ecef_zero = {0.0f};
			//return _ecef_zero;
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.pos_status, POS_UTM_F);
	}
	
	public static void stateCalcPositionEcef_f() {
		if (Std.bit_is_set(state.pos_status, POS_ECEF_F))
			return;

		if (Std.bit_is_set(state.pos_status, POS_ECEF_I)) {
			ECEF_FLOAT_OF_BFP(state.ecef_pos_f, state.ecef_pos_i);
		}
		else if (Std.bit_is_set(state.pos_status, POS_NED_F)) {
			ecef_of_ned_point_f(state.ecef_pos_f, state.ned_origin_f, state.ned_pos_f);
		}
		else if (Std.bit_is_set(state.pos_status, POS_NED_I)) {
			/* transform ned_i -> ecef_i -> ecef_f, set status bits */
			ecef_of_ned_pos_i(state.ecef_pos_i, state.ned_origin_i, state.ned_pos_i);
			SetBit(state.pos_status, POS_ECEF_F);
			ECEF_FLOAT_OF_BFP(state.ecef_pos_f, state.ecef_pos_i);
		}
		else if (bit_is_set(state.pos_status, POS_LLA_F)) {
			ecef_of_lla_f(state.ecef_pos_f, state.lla_pos_f);
		}
		else if (bit_is_set(state.pos_status, POS_LLA_I)) {
			LLA_FLOAT_OF_BFP(state.lla_pos_f, state.lla_pos_i);
			SetBit(state.pos_status, POS_LLA_F);
			ecef_of_lla_f(state.ecef_pos_f, state.lla_pos_f);
		}
		else {
			/* could not get this representation,  set errno */
			//struct EcefCoor_f _ecef_zero = {0.0f};
			//return _ecef_zero;
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.pos_status, POS_ECEF_F);
	}
	
	public static void stateCalcPositionNed_f() {
		if (bit_is_set(state.pos_status, POS_NED_F))
			return;

		int errno = 0;
		if (state.ned_initialized_f) {
			if (bit_is_set(state.pos_status, POS_NED_I)) {
				NED_FLOAT_OF_BFP(state.ned_pos_f, state.ned_pos_i);
			}
			else if (bit_is_set(state.pos_status, POS_ECEF_F)) {
				ned_of_ecef_point_f(state.ned_pos_f, state.ned_origin_f, state.ecef_pos_f);
			}
			else if (bit_is_set(state.pos_status, POS_ECEF_I)) {
				/* transform ecef_i -> ned_i -> ned_f, set status bits */
				ned_of_ecef_pos_i(state.ned_pos_i, state.ned_origin_i, state.ecef_pos_i);
				SetBit(state.pos_status, POS_NED_I);
				NED_FLOAT_OF_BFP(state.ned_pos_f, state.ned_pos_i);
			}
			else if (bit_is_set(state.pos_status, POS_LLA_F)) {
				ned_of_lla_point_f(state.ned_pos_f, state.ned_origin_f, state.lla_pos_f);
			}
			else if (bit_is_set(state.pos_status, POS_LLA_I)) {
				/* transform lla_i -> ecef_i -> ned_i -> ned_f, set status bits */
				ecef_of_lla_i(state.ecef_pos_i, state.lla_pos_i); /* converts to doubles internally */
				SetBit(state.pos_status, POS_ECEF_I);
				ned_of_ecef_pos_i(state.ned_pos_i, state.ned_origin_i, state.ecef_pos_i);
				SetBit(state.pos_status, POS_NED_I);
				NED_FLOAT_OF_BFP(state.ned_pos_f, state.ned_pos_i);
			}
			else { /* could not get this representation,  set errno */
				errno = 1;
			}
		}
		else if (state.utm_initialized_f) {
			if (bit_is_set(state.pos_status, POS_NED_I)) {
				NED_FLOAT_OF_BFP(state.ned_pos_f, state.ned_pos_i);
			}
			else if (bit_is_set(state.pos_status, POS_ENU_I)) {
				ENU_FLOAT_OF_BFP(state.enu_pos_f, state.enu_pos_i);
				SetBit(state.pos_status, POS_ENU_F);
				VECT3_NED_OF_ENU(state.ned_pos_f, state.enu_pos_f);
			}
			else if (bit_is_set(state.pos_status, POS_ENU_F)) {
				VECT3_NED_OF_ENU(state.ned_pos_f, state.enu_pos_f);
			}
			else if (bit_is_set(state.pos_status, POS_UTM_F)) {
				NED_OF_UTM_DIFF(state.ned_pos_f, state.utm_pos_f, state.utm_origin_f);
			}
			else if (bit_is_set(state.pos_status, POS_LLA_F)) {
				/* transform lla_f -> utm_f -> ned, set status bits */
				utm_of_lla_f(state.utm_pos_f, state.lla_pos_f);
				SetBit(state.pos_status, POS_UTM_F);
				NED_OF_UTM_DIFF(state.ned_pos_f, state.utm_pos_f, state.utm_origin_f);
			}
			else if (bit_is_set(state.pos_status, POS_LLA_I)) {
				/* transform lla_i -> lla_f -> utm_f -> ned, set status bits */
				LLA_FLOAT_OF_BFP(state.lla_pos_f, state.lla_pos_i);
				SetBit(state.pos_status, POS_LLA_F);
				utm_of_lla_f(state.utm_pos_f, state.lla_pos_f);
				SetBit(state.pos_status, POS_UTM_F);
				NED_OF_UTM_DIFF(state.ned_pos_f, state.utm_pos_f, state.utm_origin_f);
			}
			else { /* could not get this representation,  set errno */
				errno = 2;
			}
		}
		else { /* ned coordinate system not initialized,  set errno */
			errno = 3;
		}
		if (errno!=0) {
			//struct NedCoor_f _ned_zero = {0.0f};
			//return _ned_zero;
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.pos_status, POS_NED_F);
	}
	
	public static void stateCalcPositionEnu_f() {
		if (Std.bit_is_set(state.pos_status, POS_ENU_F))
			return;

		int errno = 0;
		if (state.ned_initialized_f) {
			if (Std.bit_is_set(state.pos_status, POS_NED_F)) {
				VECT3_ENU_OF_NED(state.enu_pos_f, state.ned_pos_f);
			}
			else if (Std.bit_is_set(state.pos_status, POS_ENU_I)) {
				ENU_FLOAT_OF_BFP(state.enu_pos_f, state.enu_pos_i);
			}
			else if (Std.bit_is_set(state.pos_status, POS_NED_I)) {
				NED_FLOAT_OF_BFP(state.ned_pos_f, state.ned_pos_i);
				SetBit(state.pos_status, POS_NED_F);
				VECT3_ENU_OF_NED(state.enu_pos_f, state.ned_pos_f);
			}
			else if (Std.bit_is_set(state.pos_status, POS_ECEF_F)) {
				enu_of_ecef_point_f(state.enu_pos_f, state.ned_origin_f, state.ecef_pos_f);
			}
			else if (Std.bit_is_set(state.pos_status, POS_ECEF_I)) {
				/* transform ecef_i -> enu_i -> enu_f, set status bits */
				enu_of_ecef_pos_i(state.enu_pos_i, state.ned_origin_i, state.ecef_pos_i);
				SetBit(state.pos_status, POS_ENU_I);
				ENU_FLOAT_OF_BFP(state.enu_pos_f, state.enu_pos_i);
			}
			else if (Std.bit_is_set(state.pos_status, POS_LLA_F)) {
				enu_of_lla_point_f(state.enu_pos_f, state.ned_origin_f, state.lla_pos_f);
			}
			else if (Std.bit_is_set(state.pos_status, POS_LLA_I)) {
				/* transform lla_i -> ecef_i -> enu_i -> enu_f, set status bits */
				ecef_of_lla_i(state.ecef_pos_i, state.lla_pos_i); /* converts to doubles internally */
				Std.SetBit(state.pos_status, POS_ECEF_I);
				enu_of_ecef_pos_i(state.enu_pos_i, state.ned_origin_i, state.ecef_pos_i);
				Std.SetBit(state.pos_status, POS_ENU_I);
				ENU_FLOAT_OF_BFP(state.enu_pos_f, state.enu_pos_i);
			}
			else { /* could not get this representation,  set errno */
				errno = 1;
			}
		}
		else if (state.utm_initialized_f) {
			if (bit_is_set(state.pos_status, POS_ENU_I)) {
				ENU_FLOAT_OF_BFP(state.enu_pos_f, state.enu_pos_i);
			}
			else if (bit_is_set(state.pos_status, POS_NED_F)) {
				VECT3_ENU_OF_NED(state.enu_pos_f, state.ned_pos_f);
			}
			else if (bit_is_set(state.pos_status, POS_NED_I)) {
				NED_FLOAT_OF_BFP(state.ned_pos_f, state.ned_pos_i);
				SetBit(state.pos_status, POS_NED_F);
				VECT3_ENU_OF_NED(state.enu_pos_f, state.ned_pos_f);
			}
			else if (bit_is_set(state.pos_status, POS_UTM_F)) {
				ENU_OF_UTM_DIFF(state.enu_pos_f, state.utm_pos_f, state.utm_origin_f);
			}
			else if (bit_is_set(state.pos_status, POS_LLA_F)) {
				/* transform lla_f -> utm_f -> enu, set status bits */
				utm_of_lla_f(state.utm_pos_f, state.lla_pos_f);
				SetBit(state.pos_status, POS_UTM_F);
				ENU_OF_UTM_DIFF(state.enu_pos_f, state.utm_pos_f, state.utm_origin_f);
			}
			else if (bit_is_set(state.pos_status, POS_LLA_I)) {
				/* transform lla_i -> lla_f -> utm_f -> enu, set status bits */
				LLA_FLOAT_OF_BFP(state.lla_pos_f, state.lla_pos_i);
				SetBit(state.pos_status, POS_LLA_F);
				utm_of_lla_f(state.utm_pos_f, state.lla_pos_f);
				SetBit(state.pos_status, POS_UTM_F);
				ENU_OF_UTM_DIFF(state.enu_pos_f, state.utm_pos_f, state.utm_origin_f);
			}
			else { /* could not get this representation,  set errno */
				errno = 2;
			}
		}
		else { /* ned coordinate system not initialized,  set errno */
			errno = 3;
		}
		if (errno!=0) {
			//struct EnuCoor_f _enu_zero = {0.0f};
			//return _enu_zero;
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.pos_status, POS_ENU_F);
	}
	
	public static void stateCalcPositionLla_f() {
		if (bit_is_set(state.pos_status, POS_LLA_F))
			return;

		if (bit_is_set(state.pos_status, POS_LLA_I)) {
			LLA_FLOAT_OF_BFP(state.lla_pos_f, state.lla_pos_f);
		}
		else if (bit_is_set(state.pos_status, POS_ECEF_F)) {
			lla_of_ecef_f(state.lla_pos_f, state.ecef_pos_f);
		}
		else if (bit_is_set(state.pos_status, POS_ECEF_I)) {
			/* transform ecef_i -> ecef_f -> lla_f, set status bits */
			ECEF_FLOAT_OF_BFP(state.ecef_pos_f, state.ecef_pos_i);
			SetBit(state.pos_status, POS_ECEF_F);
			lla_of_ecef_f(state.lla_pos_f, state.ecef_pos_f);
		}
		else if (bit_is_set(state.pos_status, POS_NED_F)) {
			/* transform ned_f -> ecef_f -> lla_f, set status bits */
			ecef_of_ned_point_f(state.ecef_pos_f, state.ned_origin_f, state.ned_pos_f);
			SetBit(state.pos_status, POS_ECEF_F);
			lla_of_ecef_f(state.lla_pos_f, state.ecef_pos_f);
		}
		else if (bit_is_set(state.pos_status, POS_NED_I)) {
			/* transform ned_i -> ned_f -> ecef_f -> lla_f, set status bits */
			NED_FLOAT_OF_BFP(state.ned_pos_f, state.ned_pos_i);
			SetBit(state.pos_status, POS_NED_F);
			ecef_of_ned_point_f(state.ecef_pos_f, state.ned_origin_f, state.ned_pos_f);
			SetBit(state.pos_status, POS_ECEF_F);
			lla_of_ecef_f(state.lla_pos_f, state.ecef_pos_f);
		}
		else if (bit_is_set(state.pos_status, POS_UTM_F)) {
			lla_of_utm_f(state.lla_pos_f, state.utm_pos_f);
		}
		else {
			/* could not get this representation,  set errno */
			//struct LlaCoor_f _lla_zero = {0.0};
			//return _lla_zero;
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.pos_status, POS_LLA_F);
	}
	
	public static void stateCalcSpeedNed_i() {
		if (bit_is_set(state.speed_status, SPEED_NED_I))
			return;

		int errno = 0;
		if (state.ned_initialized_i) {
			if (bit_is_set(state.speed_status, SPEED_NED_F)) {
				SPEEDS_BFP_OF_REAL(state.ned_speed_i, state.ned_speed_f);
			}
			else if (bit_is_set(state.speed_status, SPEED_ENU_I)) {
				INT32_VECT3_NED_OF_ENU(state.ned_speed_i, state.enu_speed_i);
			}
			else if (bit_is_set(state.speed_status, SPEED_ENU_F)) {
				SPEEDS_BFP_OF_REAL(state.enu_speed_i, state.enu_speed_f);
				SetBit(state.speed_status, SPEED_ENU_I);
				INT32_VECT3_NED_OF_ENU(state.ned_speed_i, state.enu_speed_i);
			}
			else if (bit_is_set(state.speed_status, SPEED_ECEF_I)) {
				ned_of_ecef_vect_i(state.ned_speed_i, state.ned_origin_i, state.ecef_speed_i);
			}
			else if (bit_is_set(state.speed_status, SPEED_ECEF_F)) {
				/* transform ecef_f -> ecef_i -> ned_i , set status bits */
				SPEEDS_BFP_OF_REAL(state.ecef_speed_i, state.ecef_speed_f);
				SetBit(state.speed_status, SPEED_ECEF_I);
				ned_of_ecef_vect_i(state.ned_speed_i, state.ned_origin_i, state.ecef_speed_i);
			}
			else { /* could not get this representation,  set errno */
				errno = 1;
			}
		}
		else if (state.utm_initialized_f) {
			if (bit_is_set(state.speed_status, SPEED_NED_F)) {
				SPEEDS_BFP_OF_REAL(state.ned_speed_i, state.ned_speed_f);
			}
			else if (bit_is_set(state.speed_status, SPEED_ENU_I)) {
				INT32_VECT3_NED_OF_ENU(state.ned_speed_i, state.enu_speed_i);
			}
			else if (bit_is_set(state.speed_status, SPEED_ENU_F)) {
				SPEEDS_BFP_OF_REAL(state.enu_speed_i, state.enu_speed_f);
				SetBit(state.speed_status, SPEED_ENU_I);
				INT32_VECT3_NED_OF_ENU(state.ned_speed_i, state.enu_speed_i);
			}
			else { /* could not get this representation,  set errno */
				errno = 2;
			}
		}
		else { /* ned coordinate system not initialized,  set errno */
			errno = 3;
		}
		if (errno != 0) {
			//struct NedCoor_i _ned_zero = {0};
			//return _ned_zero;
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.speed_status, SPEED_NED_I);
	}
	
	public static void stateCalcSpeedEnu_i() {
		if (bit_is_set(state.speed_status, SPEED_ENU_I))
			return;

		int errno = 0;
		if (state.ned_initialized_i) {
			if (bit_is_set(state.speed_status, SPEED_NED_I)) {
				INT32_VECT3_ENU_OF_NED(state.enu_speed_i, state.ned_speed_i);
			}
			if (bit_is_set(state.speed_status, SPEED_ENU_F)) {
				ENU_BFP_OF_REAL(state.enu_speed_i, state.enu_speed_f);
			}
			else if (bit_is_set(state.speed_status, SPEED_NED_F)) {
				SPEEDS_BFP_OF_REAL(state.ned_speed_i, state.ned_speed_f);
				SetBit(state.pos_status, SPEED_NED_I);
				INT32_VECT3_ENU_OF_NED(state.enu_speed_i, state.ned_speed_i);
			}
			else if (bit_is_set(state.speed_status, SPEED_ECEF_I)) {
				enu_of_ecef_vect_i(state.enu_speed_i, state.ned_origin_i, state.ecef_speed_i);
			}
			else if (bit_is_set(state.speed_status, SPEED_ECEF_F)) {
				/* transform ecef_f -> ecef_i -> enu_i , set status bits */
				SPEEDS_BFP_OF_REAL(state.ecef_speed_i, state.ecef_speed_f);
				SetBit(state.speed_status, SPEED_ECEF_I);
				enu_of_ecef_vect_i(state.enu_speed_i, state.ned_origin_i, state.ecef_speed_i);
			}
			else { /* could not get this representation,  set errno */
				errno = 1;
			}
		}
		else if (state.utm_initialized_f) {
			if (bit_is_set(state.speed_status, SPEED_NED_I)) {
				INT32_VECT3_ENU_OF_NED(state.enu_speed_i, state.ned_speed_i);
			}
			if (bit_is_set(state.speed_status, SPEED_ENU_F)) {
				ENU_BFP_OF_REAL(state.enu_speed_i, state.enu_speed_f);
			}
			else if (bit_is_set(state.speed_status, SPEED_NED_F)) {
				SPEEDS_BFP_OF_REAL(state.ned_speed_i, state.ned_speed_f);
				SetBit(state.pos_status, SPEED_NED_I);
				INT32_VECT3_ENU_OF_NED(state.enu_speed_i, state.ned_speed_i);
			}
			else { /* could not get this representation,  set errno */
				errno = 2;
			}
		}
		else { /* ned coordinate system not initialized,  set errno */
			errno = 3;
		}
		if (errno != 0) {
			//struct EnuCoor_i _enu_zero = {0};
			//return _enu_zero;
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.speed_status, SPEED_ENU_I);
	}
	
	public static void stateCalcSpeedEcef_i() {
		if (bit_is_set(state.speed_status, SPEED_ECEF_I))
			return;

		if (bit_is_set(state.speed_status, SPEED_ECEF_F)) {
			SPEEDS_BFP_OF_REAL(state.ecef_speed_i, state.ecef_speed_f);
		}
		else if (bit_is_set(state.speed_status, SPEED_NED_I)) {
			ecef_of_ned_vect_i(state.ecef_speed_i, state.ned_origin_i, state.ned_speed_i);
		}
		else if (bit_is_set(state.speed_status, SPEED_NED_F)) {
			/* transform ned_f -> ned_i -> ecef_i , set status bits */
			SPEEDS_BFP_OF_REAL(state.ned_speed_i, state.ned_speed_f);
			SetBit(state.speed_status, SPEED_NED_I);
			ecef_of_ned_vect_i(state.ecef_speed_i, state.ned_origin_i, state.ned_speed_i);
		}
		else {
			/* could not get this representation,  set errno */
			//struct EcefCoor_i _ecef_zero = {0};
			//return _ecef_zero;
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.speed_status, SPEED_ECEF_I);
	}
	
	public static void stateCalcHorizontalSpeedNorm_i() {
		if (bit_is_set(state.speed_status, SPEED_HNORM_I))
			return;

		if (bit_is_set(state.speed_status, SPEED_HNORM_F)){
			state.h_speed_norm_i = SPEED_BFP_OF_REAL(state.h_speed_norm_f);
		}
		else if (bit_is_set(state.speed_status, SPEED_NED_I)) {
			long n2 = (state.ned_speed_i.x*state.ned_speed_i.x +
					state.ned_speed_i.y*state.ned_speed_i.y) >> INT32_SPEED_FRAC;
	  //INT32_SQRT(state.h_speed_norm_i, n2);
	  long _in = n2;
	  if ((_in) == 0)                                             
		  (state.h_speed_norm_i) = 0;                                               
	  else {                                                      
		  long s1, s2;                                          
		  int iter = 0;                                         
		  s2 = _in;                                                 
		  do {                                                      
			  s1 = s2;                                                
			  s2 = (_in) / s1;                                        
			  s2 += s1;                                               
			  s2 /= 2;                                                
			  iter++;                                                 
		  }                                                         
		  while( ( (s1-s2) > 1) && (iter < INT32_SQRT_MAX_ITER));   
		  (state.h_speed_norm_i) = s2;                                              
	  }                                                           
		}
		else if (bit_is_set(state.speed_status, SPEED_NED_F)) {
			//FLOAT_VECT2_NORM(state.h_speed_norm_f, state.ned_speed_f);
			state.h_speed_norm_f = (float) Math.sqrt(FLOAT_VECT2_NORM2(state.ned_speed_f));
			SetBit(state.speed_status, SPEED_HNORM_F);
			state.h_speed_norm_i = SPEED_BFP_OF_REAL(state.h_speed_norm_f);
		}
		else if (bit_is_set(state.speed_status, SPEED_ENU_I)) {
			long n2 = (state.enu_speed_i.x*state.enu_speed_i.x +
					state.enu_speed_i.y*state.enu_speed_i.y) >> INT32_SPEED_FRAC;
	  //INT32_SQRT(state.h_speed_norm_i, n2);
	  
	  long _in = n2;
	  if ((_in) == 0)                                             
		  (state.h_speed_norm_i) = 0;                                               
	  else {                                                      
		  long s1, s2;                                          
		  int iter = 0;                                         
		  s2 = _in;                                                 
		  do {                                                      
			  s1 = s2;                                                
			  s2 = (_in) / s1;                                        
			  s2 += s1;                                               
			  s2 /= 2;                                                
			  iter++;                                                 
		  }                                                         
		  while( ( (s1-s2) > 1) && (iter < INT32_SQRT_MAX_ITER));   
		  (state.h_speed_norm_i) = s2;                                              
	  }                                          
	  
		}
		else if (bit_is_set(state.speed_status, SPEED_ENU_F)) {
			//FLOAT_VECT2_NORM(state.h_speed_norm_f, state.enu_speed_f);
			state.h_speed_norm_f = (float) Math.sqrt(FLOAT_VECT2_NORM2(state.enu_speed_f));
			SetBit(state.speed_status, SPEED_HNORM_F);
			state.h_speed_norm_i = SPEED_BFP_OF_REAL(state.h_speed_norm_f);
		}
		else if (bit_is_set(state.speed_status, SPEED_ECEF_I)) {
			/* transform ecef speed to ned, set status bit, then compute norm */
			ned_of_ecef_vect_i(state.ned_speed_i, state.ned_origin_i, state.ecef_speed_i);
			SetBit(state.speed_status, SPEED_NED_I);
			long n2 = (state.ned_speed_i.x*state.ned_speed_i.x +
					state.ned_speed_i.y*state.ned_speed_i.y) >> INT32_SPEED_FRAC;
	 // INT32_SQRT(state.h_speed_norm_i, n2);
	  
	  long _in = n2;
	  if ((_in) == 0)                                             
		  (state.h_speed_norm_i) = 0;                                               
	  else {                                                      
		  long s1, s2;                                          
		  int iter = 0;                                         
		  s2 = _in;                                                 
		  do {                                                      
			  s1 = s2;                                                
			  s2 = (_in) / s1;                                        
			  s2 += s1;                                               
			  s2 /= 2;                                                
			  iter++;                                                 
		  }                                                         
		  while( ( (s1-s2) > 1) && (iter < INT32_SQRT_MAX_ITER));   
		  (state.h_speed_norm_i) = s2;                                              
	  }            
		}
		else if (bit_is_set(state.speed_status, SPEED_ECEF_F)) {
			ned_of_ecef_vect_f(state.ned_speed_f, state.ned_origin_f, state.ecef_speed_f);
			SetBit(state.speed_status, SPEED_NED_F);
			//FLOAT_VECT2_NORM(state.h_speed_norm_f, state.ned_speed_f);
			state.h_speed_norm_f = (float) Math.sqrt(FLOAT_VECT2_NORM2(state.ned_speed_f));
			SetBit(state.speed_status, SPEED_HNORM_F);
			state.h_speed_norm_i = SPEED_BFP_OF_REAL(state.h_speed_norm_f);
		}
		else {
			//int32_t _norm_zero = 0;
			//return _norm_zero;
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.speed_status, SPEED_HNORM_I);
	}
	
	public static void stateCalcHorizontalSpeedDir_i() {
		if (bit_is_set(state.speed_status, SPEED_HDIR_I))
			return;

		if (bit_is_set(state.speed_status, SPEED_HDIR_F)){
			state.h_speed_dir_i = SPEED_BFP_OF_REAL(state.h_speed_dir_f);
		}
		else if (bit_is_set(state.speed_status, SPEED_NED_I)) {
			//INT32_ATAN2(state.h_speed_dir_i, state.ned_speed_i.y, state.ned_speed_i.x);
			long _y = state.ned_speed_i.y, _x = state.ned_speed_i.x;
			int c1 = INT32_ANGLE_PI_4;        
		    int c2 = 3 * INT32_ANGLE_PI_4;        
		    long abs_y = Math.abs(_y) + 1;          
		    long r;                      
		    if ( (_x) >= 0) {                               
		      r = (((_x)-abs_y)<<R_FRAC) / ((_x)+abs_y);    
		      (state.h_speed_dir_i) = c1 - ((c1 * r)>>R_FRAC);               
		    }                          
		    else {                      
		      r = (((_x)+abs_y)<<R_FRAC) / (abs_y-(_x));    
		      (state.h_speed_dir_i) = c2 - ((c1 * r)>>R_FRAC);           
		    }                           
		    if ((_y)<0)                     
		      (state.h_speed_dir_i) = -(state.h_speed_dir_i);                 
		  
			//INT32_COURSE_NORMALIZE(state.h_speed_dir_i);
		    while ((state.h_speed_dir_i) < 0) (state.h_speed_dir_i) += INT32_ANGLE_2_PI;                  
		    while ((state.h_speed_dir_i) >= INT32_ANGLE_2_PI)  (state.h_speed_dir_i) -= INT32_ANGLE_2_PI; 
		   
		}
		else if (bit_is_set(state.speed_status, SPEED_ENU_I)) {
			//INT32_ATAN2(state.h_speed_dir_i, state.enu_speed_i.x, state.enu_speed_i.y);
			long _y = state.enu_speed_i.x, _x = state.enu_speed_i.y;
			int c1 = INT32_ANGLE_PI_4;        
		    int c2 = 3 * INT32_ANGLE_PI_4;        
		    long abs_y = Math.abs(_y) + 1;          
		    long r;                      
		    if ( (_x) >= 0) {                               
		      r = (((_x)-abs_y)<<R_FRAC) / ((_x)+abs_y);    
		      (state.h_speed_dir_i) = c1 - ((c1 * r)>>R_FRAC);               
		    }                          
		    else {                      
		      r = (((_x)+abs_y)<<R_FRAC) / (abs_y-(_x));    
		      (state.h_speed_dir_i) = c2 - ((c1 * r)>>R_FRAC);           
		    }                           
		    if ((_y)<0)                     
		      (state.h_speed_dir_i) = -(state.h_speed_dir_i);   
			//INT32_COURSE_NORMALIZE(state.h_speed_dir_i);
		    while ((state.h_speed_dir_i) < 0) (state.h_speed_dir_i) += INT32_ANGLE_2_PI;                  
		    while ((state.h_speed_dir_i) >= INT32_ANGLE_2_PI)  (state.h_speed_dir_i) -= INT32_ANGLE_2_PI; 
		    
		}
		else if (bit_is_set(state.speed_status, SPEED_NED_F)) {
			SPEEDS_BFP_OF_REAL(state.ned_speed_i, state.ned_speed_f);
			SetBit(state.speed_status, SPEED_NED_I);
			//INT32_ATAN2(state.h_speed_dir_i, state.ned_speed_i.y, state.ned_speed_i.x);
			long _y = state.ned_speed_i.y, _x = state.ned_speed_i.x;
			int c1 = INT32_ANGLE_PI_4;        
		    int c2 = 3 * INT32_ANGLE_PI_4;        
		    long abs_y = Math.abs(_y) + 1;          
		    long r;                      
		    if ( (_x) >= 0) {                               
		      r = (((_x)-abs_y)<<R_FRAC) / ((_x)+abs_y);    
		      (state.h_speed_dir_i) = c1 - ((c1 * r)>>R_FRAC);               
		    }                          
		    else {                      
		      r = (((_x)+abs_y)<<R_FRAC) / (abs_y-(_x));    
		      (state.h_speed_dir_i) = c2 - ((c1 * r)>>R_FRAC);           
		    }                           
		    if ((_y)<0)                     
		      (state.h_speed_dir_i) = -(state.h_speed_dir_i);                 
		  
			//INT32_COURSE_NORMALIZE(state.h_speed_dir_i);
		    while ((state.h_speed_dir_i) < 0) (state.h_speed_dir_i) += INT32_ANGLE_2_PI;                  
		    while ((state.h_speed_dir_i) >= INT32_ANGLE_2_PI)  (state.h_speed_dir_i) -= INT32_ANGLE_2_PI; 
		}
		else if (bit_is_set(state.speed_status, SPEED_ENU_F)) {
			SPEEDS_BFP_OF_REAL(state.enu_speed_i, state.enu_speed_f);
			SetBit(state.speed_status, SPEED_ENU_I);
			//INT32_ATAN2(state.h_speed_dir_i, state.enu_speed_i.x, state.enu_speed_i.y);
			long _y = state.enu_speed_i.x, _x = state.enu_speed_i.y;
			int c1 = INT32_ANGLE_PI_4;        
		    int c2 = 3 * INT32_ANGLE_PI_4;        
		    long abs_y = Math.abs(_y) + 1;          
		    long r;                      
		    if ( (_x) >= 0) {                               
		      r = (((_x)-abs_y)<<R_FRAC) / ((_x)+abs_y);    
		      (state.h_speed_dir_i) = c1 - ((c1 * r)>>R_FRAC);               
		    }                          
		    else {                      
		      r = (((_x)+abs_y)<<R_FRAC) / (abs_y-(_x));    
		      (state.h_speed_dir_i) = c2 - ((c1 * r)>>R_FRAC);           
		    }                           
		    if ((_y)<0)                     
		      (state.h_speed_dir_i) = -(state.h_speed_dir_i);   
			//INT32_COURSE_NORMALIZE(state.h_speed_dir_i);
		    while ((state.h_speed_dir_i) < 0) (state.h_speed_dir_i) += INT32_ANGLE_2_PI;                  
		    while ((state.h_speed_dir_i) >= INT32_ANGLE_2_PI)  (state.h_speed_dir_i) -= INT32_ANGLE_2_PI; 
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.speed_status, SPEED_HDIR_I);
	}
	
	public static void stateCalcSpeedNed_f() {
		if (bit_is_set(state.speed_status, SPEED_NED_F))
			return;

		int errno = 0;
		if (state.ned_initialized_f) {
			if (bit_is_set(state.speed_status, SPEED_NED_I)) {
				SPEEDS_FLOAT_OF_BFP(state.ned_speed_f, state.ned_speed_i);
			}
			else if (bit_is_set(state.speed_status, SPEED_ENU_F)) {
				VECT3_NED_OF_ENU(state.ned_speed_f, state.enu_speed_f);
			}
			else if (bit_is_set(state.speed_status, SPEED_ENU_I)) {
				SPEEDS_FLOAT_OF_BFP(state.enu_speed_f, state.enu_speed_i);
				SetBit(state.speed_status, SPEED_ENU_F);
				VECT3_NED_OF_ENU(state.ned_speed_f, state.enu_speed_f);
			}
			else if (bit_is_set(state.speed_status, SPEED_ECEF_F)) {
				ned_of_ecef_vect_f(state.ned_speed_f, state.ned_origin_f, state.ecef_speed_f);
			}
			else if (bit_is_set(state.speed_status, SPEED_ECEF_I)) {
				/* transform ecef_i -> ecef_f -> ned_f , set status bits */
				SPEEDS_FLOAT_OF_BFP(state.ecef_speed_f, state.ecef_speed_i);
				SetBit(state.speed_status, SPEED_ECEF_F);
				ned_of_ecef_vect_f(state.ned_speed_f, state.ned_origin_f, state.ecef_speed_f);
			}
			else { /* could not get this representation,  set errno */
				errno = 1;
			}
		}
		else if (state.utm_initialized_f) {
			if (bit_is_set(state.speed_status, SPEED_NED_I)) {
				SPEEDS_FLOAT_OF_BFP(state.ned_speed_f, state.ned_speed_i);
			}
			else if (bit_is_set(state.speed_status, SPEED_ENU_F)) {
				VECT3_NED_OF_ENU(state.ned_speed_f, state.enu_speed_f);
			}
			else if (bit_is_set(state.speed_status, SPEED_ENU_I)) {
				SPEEDS_FLOAT_OF_BFP(state.enu_speed_f, state.enu_speed_i);
				SetBit(state.speed_status, SPEED_ENU_F);
				VECT3_NED_OF_ENU(state.ned_speed_f, state.enu_speed_f);
			}
			else { /* could not get this representation,  set errno */
				errno = 2;
			}
		}
		else { /* ned coordinate system not initialized,  set errno */
			errno = 3;
		}
		if (errno !=0) {
			//struct NedCoor_f _ned_zero = {0.0f};
			//return _ned_zero;
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.speed_status, SPEED_NED_F);
	}

	public static void stateCalcSpeedEnu_f() {
		if (bit_is_set(state.speed_status, SPEED_ENU_F))
			return;

		int errno = 0;
		if (state.ned_initialized_f) {
			if (bit_is_set(state.speed_status, SPEED_NED_F)) {
				VECT3_ENU_OF_NED(state.enu_speed_f, state.ned_speed_f);
			}
			else if (bit_is_set(state.speed_status, SPEED_ENU_I)) {
				ENU_FLOAT_OF_BFP(state.enu_speed_f, state.enu_speed_i);
			}
			else if (bit_is_set(state.speed_status, SPEED_NED_I)) {
				SPEEDS_FLOAT_OF_BFP(state.ned_speed_f, state.ned_speed_i);
				SetBit(state.pos_status, SPEED_NED_F);
				VECT3_ENU_OF_NED(state.enu_speed_f, state.ned_speed_f);
			}
			else if (bit_is_set(state.speed_status, SPEED_ECEF_F)) {
				enu_of_ecef_vect_f(state.enu_speed_f, state.ned_origin_f, state.ecef_speed_f);
			}
			else if (bit_is_set(state.speed_status, SPEED_ECEF_I)) {
				/* transform ecef_I -> ecef_f -> enu_f , set status bits */
				SPEEDS_FLOAT_OF_BFP(state.ecef_speed_f, state.ecef_speed_i);
				SetBit(state.speed_status, SPEED_ECEF_F);
				enu_of_ecef_vect_f(state.enu_speed_f, state.ned_origin_f, state.ecef_speed_f);
			}
			else { /* could not get this representation,  set errno */
				errno = 1;
			}
		}
		else if (state.utm_initialized_f) {
			if (bit_is_set(state.speed_status, SPEED_NED_F)) {
				VECT3_ENU_OF_NED(state.enu_speed_f, state.ned_speed_f);
			}
			else if (bit_is_set(state.speed_status, SPEED_ENU_I)) {
				ENU_FLOAT_OF_BFP(state.enu_speed_f, state.enu_speed_i);
			}
			else if (bit_is_set(state.speed_status, SPEED_NED_I)) {
				SPEEDS_FLOAT_OF_BFP(state.ned_speed_f, state.ned_speed_i);
				SetBit(state.pos_status, SPEED_NED_F);
				VECT3_ENU_OF_NED(state.enu_speed_f, state.ned_speed_f);
			}
			else { /* could not get this representation,  set errno */
				errno = 2;
			}
		}
		else { /* ned coordinate system not initialized,  set errno */
			errno = 3;
		}
		if (errno != 0) {
			//struct EnuCoor_f _enu_zero = {0};
			//return _enu_zero;
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.speed_status, SPEED_ENU_F);
	}

	public static void stateCalcSpeedEcef_f() {
		if (bit_is_set(state.speed_status, SPEED_ECEF_F))
			return;

		if (bit_is_set(state.speed_status, SPEED_ECEF_I)) {
			SPEEDS_FLOAT_OF_BFP(state.ecef_speed_f, state.ned_speed_i);
		}
		else if (bit_is_set(state.speed_status, SPEED_NED_F)) {
			ecef_of_ned_vect_f(state.ecef_speed_f, state.ned_origin_f, state.ned_speed_f);
		}
		else if (bit_is_set(state.speed_status, SPEED_NED_I)) {
			/* transform ned_f -> ned_i -> ecef_i , set status bits */
			SPEEDS_FLOAT_OF_BFP(state.ned_speed_f, state.ned_speed_i);
			SetBit(state.speed_status, SPEED_NED_F);
			ecef_of_ned_vect_f(state.ecef_speed_f, state.ned_origin_f, state.ned_speed_f);
		}
		else {
			/* could not get this representation,  set errno */
			//struct EcefCoor_f _ecef_zero = {0.0f};
			//return _ecef_zero;
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.speed_status, SPEED_ECEF_F);
	}

	public static void stateCalcHorizontalSpeedNorm_f() {
		if (bit_is_set(state.speed_status, SPEED_HNORM_F))
			return;

		if (bit_is_set(state.speed_status, SPEED_HNORM_I)){
			state.h_speed_norm_f = SPEED_FLOAT_OF_BFP(state.h_speed_norm_i);
		}
		else if (bit_is_set(state.speed_status, SPEED_NED_F)) {
			//FLOAT_VECT2_NORM(state.h_speed_norm_f, state.ned_speed_f);
			state.h_speed_norm_f = (float) Math.sqrt(FLOAT_VECT2_NORM2(state.ned_speed_f));
		}
		else if (bit_is_set(state.speed_status, SPEED_ENU_F)) {
			//FLOAT_VECT2_NORM(state.h_speed_norm_f, state.enu_speed_f);
			state.h_speed_norm_f = (float) Math.sqrt(FLOAT_VECT2_NORM2(state.enu_speed_f));
		}
		else if (bit_is_set(state.speed_status, SPEED_NED_I)) {
			SPEEDS_FLOAT_OF_BFP(state.ned_speed_f, state.ned_speed_i);
			SetBit(state.speed_status, SPEED_NED_F);
			//FLOAT_VECT2_NORM(state.h_speed_norm_f, state.ned_speed_f);
			state.h_speed_norm_f = (float) Math.sqrt(FLOAT_VECT2_NORM2(state.ned_speed_f));
		}
		else if (bit_is_set(state.speed_status, SPEED_ENU_I)) {
			SPEEDS_FLOAT_OF_BFP(state.enu_speed_f, state.enu_speed_i);
			SetBit(state.speed_status, SPEED_ENU_F);
			//FLOAT_VECT2_NORM(state.h_speed_norm_f, state.enu_speed_f);
			state.h_speed_norm_f = (float) Math.sqrt(FLOAT_VECT2_NORM2(state.enu_speed_f));
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.speed_status, SPEED_HNORM_F);
	}

	public static void stateCalcHorizontalSpeedDir_f() {
		if (bit_is_set(state.speed_status, SPEED_HDIR_F))
			return;

		if (bit_is_set(state.speed_status, SPEED_HDIR_I)){
			state.h_speed_dir_f = SPEED_FLOAT_OF_BFP(state.h_speed_dir_i);
		}
		else if (bit_is_set(state.speed_status, SPEED_NED_F)) {
			state.h_speed_dir_f =(float) Math.atan2((double)state.ned_speed_f.y, (double)state.ned_speed_f.x);
		}
		else if (bit_is_set(state.speed_status, SPEED_ENU_F)) {
			state.h_speed_dir_f =(float) Math.atan2((double)state.enu_speed_f.x, (double)state.enu_speed_f.y);
		}
		else if (bit_is_set(state.speed_status, SPEED_NED_I)) {
			SPEEDS_FLOAT_OF_BFP(state.ned_speed_f, state.ned_speed_i);
			SetBit(state.speed_status, SPEED_NED_F);
			state.h_speed_dir_f = (float) Math.atan2((double) state.ned_speed_f.y,(double) state.ned_speed_f.x);
		}
		else if (bit_is_set(state.speed_status, SPEED_ENU_I)) {
			SPEEDS_FLOAT_OF_BFP(state.enu_speed_f, state.enu_speed_i);
			SetBit(state.speed_status, SPEED_ENU_F);
			state.h_speed_dir_f = (float) Math.atan2((double)state.enu_speed_f.x,(double) state.enu_speed_f.y);
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.speed_status, SPEED_HDIR_F);
	}


	public static void stateCalcAccelNed_i() {
		if (bit_is_set(state.accel_status, ACCEL_NED_I))
			return;

		int errno = 0;
		if (state.ned_initialized_i) {
			if (bit_is_set(state.accel_status, ACCEL_NED_F)) {
				ACCELS_BFP_OF_REAL(state.ned_accel_i, state.ned_accel_f);
			}
			else if (bit_is_set(state.accel_status, ACCEL_ECEF_I)) {
				ned_of_ecef_vect_i(state.ned_accel_i, state.ned_origin_i, state.ecef_accel_i);
			}
			else if (bit_is_set(state.accel_status, ACCEL_ECEF_F)) {
				/* transform ecef_f -> ecef_i -> ned_i , set status bits */
				ACCELS_BFP_OF_REAL(state.ecef_accel_i, state.ecef_accel_f);
				SetBit(state.accel_status, ACCEL_ECEF_I);
				ned_of_ecef_vect_i(state.ned_accel_i, state.ned_origin_i, state.ecef_accel_i);
			}
			else { /* could not get this representation,  set errno */
				errno = 1;
			}
		} else { /* ned coordinate system not initialized,  set errno */
			errno = 2;
		}
		if (errno!=0) {
			//struct NedCoor_i _ned_zero = {0};
			//return _ned_zero;
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.accel_status, ACCEL_NED_I);
	}

	public static void stateCalcAccelEcef_i() {
		if (bit_is_set(state.accel_status, ACCEL_ECEF_I))
			return;

		if (bit_is_set(state.accel_status, ACCEL_ECEF_F)) {
			ACCELS_BFP_OF_REAL(state.ecef_accel_i, state.ecef_accel_f);
		}
		else if (bit_is_set(state.accel_status, ACCEL_NED_I)) {
			ecef_of_ned_vect_i(state.ecef_accel_i, state.ned_origin_i, state.ned_accel_i);
		}
		else if (bit_is_set(state.accel_status, ACCEL_NED_F)) {
			/* transform ned_f -> ned_i -> ecef_i , set status bits */
			ACCELS_BFP_OF_REAL(state.ned_accel_i, state.ned_accel_f);
			SetBit(state.accel_status, ACCEL_NED_I);
			ecef_of_ned_vect_i(state.ecef_accel_i, state.ned_origin_i, state.ned_accel_i);
		}
		else {
			/* could not get this representation,  set errno */
			//struct EcefCoor_i _ecef_zero = {0};
			//return _ecef_zero;
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.accel_status, ACCEL_ECEF_I);
	}

	public static void stateCalcAccelNed_f() {
		if (bit_is_set(state.accel_status, ACCEL_NED_F))
			return;

		int errno = 0;
		if (state.ned_initialized_f) {
			if (bit_is_set(state.accel_status, ACCEL_NED_I)) {
				ACCELS_FLOAT_OF_BFP(state.ned_accel_f, state.ned_accel_i);
			}
			else if (bit_is_set(state.accel_status, ACCEL_ECEF_F)) {
				ned_of_ecef_vect_f(state.ned_accel_f, state.ned_origin_f, state.ecef_accel_f);
			}
			else if (bit_is_set(state.accel_status, ACCEL_ECEF_I)) {
				/* transform ecef_i -> ecef_f -> ned_f , set status bits */
				ACCELS_FLOAT_OF_BFP(state.ecef_accel_f, state.ecef_accel_i);
				SetBit(state.accel_status, ACCEL_ECEF_F);
				ned_of_ecef_vect_f(state.ned_accel_f, state.ned_origin_f, state.ecef_accel_f);
			}
			else { /* could not get this representation,  set errno */
				errno = 1;
			}
		} else { /* ned coordinate system not initialized,  set errno */
			errno = 2;
		}
		if (errno!=0) {
			//struct NedCoor_f _ned_zero = {0.0f};
			//return _ned_zero;
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.accel_status, ACCEL_NED_F);
	}

	public static void stateCalcAccelEcef_f() {
		if (bit_is_set(state.accel_status, ACCEL_ECEF_F))
			return;

		if (bit_is_set(state.accel_status, ACCEL_ECEF_I)) {
			ACCELS_FLOAT_OF_BFP(state.ecef_accel_f, state.ned_accel_i);
		}
		else if (bit_is_set(state.accel_status, ACCEL_NED_F)) {
			ecef_of_ned_vect_f(state.ecef_accel_f, state.ned_origin_f, state.ned_accel_f);
		}
		else if (bit_is_set(state.accel_status, ACCEL_NED_I)) {
			/* transform ned_f -> ned_i -> ecef_i , set status bits */
			ACCELS_FLOAT_OF_BFP(state.ned_accel_f, state.ned_accel_i);
			SetBit(state.accel_status, ACCEL_NED_F);
			ecef_of_ned_vect_f(state.ecef_accel_f, state.ned_origin_f, state.ned_accel_f);
		}
		else {
			/* could not get this representation,  set errno */
			//struct EcefCoor_f _ecef_zero = {0.0f};
			//return _ecef_zero;
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.accel_status, ACCEL_ECEF_F);
	}
	
	public static void stateCalcBodyRates_i() {
		if (bit_is_set(state.rate_status, RATE_I))
			return;

		if (bit_is_set(state.rate_status, RATE_F)) {
			RATES_BFP_OF_REAL(state.body_rates_i, state.body_rates_f);
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.rate_status, RATE_I);
	}

	public static void stateCalcBodyRates_f() {
		if (bit_is_set(state.rate_status, RATE_F))
			return;

		if (bit_is_set(state.rate_status, RATE_I)) {
			RATES_FLOAT_OF_BFP(state.body_rates_f, state.body_rates_i);
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.rate_status, RATE_F);
	}

	/** @}*/


	/******************************************************************************
	 *                                                                            *
	 * Transformation functions for the WIND- AND AIRSPEED representations        *
	 *                                                                            *
	 *****************************************************************************/
	/** @addtogroup state_wind_airspeed
	 *  @{ */

	public static void stateCalcHorizontalWindspeed_i() {
		if (bit_is_set(state.wind_air_status, WINDSPEED_I))
			return;

		if (bit_is_set(state.wind_air_status, WINDSPEED_F)) {
			state.h_windspeed_i.x = SPEED_BFP_OF_REAL(state.h_windspeed_f.x);
			state.h_windspeed_i.y = SPEED_BFP_OF_REAL(state.h_windspeed_f.y);
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.rate_status, WINDSPEED_I);
	}

	public static void stateCalcAirspeed_i() {
		if (bit_is_set(state.wind_air_status, AIRSPEED_I))
			return;

		if (bit_is_set(state.wind_air_status, AIRSPEED_F)) {
			state.airspeed_i = SPEED_BFP_OF_REAL(state.airspeed_f);
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.wind_air_status, AIRSPEED_I);
	}

	public static void stateCalcHorizontalWindspeed_f() {
		if (bit_is_set(state.wind_air_status, WINDSPEED_F))
			return;

		if (bit_is_set(state.wind_air_status, WINDSPEED_I)) {
			state.h_windspeed_f.x = SPEED_FLOAT_OF_BFP(state.h_windspeed_i.x);
			state.h_windspeed_f.x = SPEED_FLOAT_OF_BFP(state.h_windspeed_i.y);
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.rate_status, WINDSPEED_F);
	}

	public static void stateCalcAirspeed_f() {
		if (bit_is_set(state.wind_air_status, AIRSPEED_F))
			return;

		if (bit_is_set(state.wind_air_status, AIRSPEED_I)) {
			state.airspeed_f = SPEED_FLOAT_OF_BFP(state.airspeed_i);
		}
		/* set bit to indicate this representation is computed */
		SetBit(state.wind_air_status, AIRSPEED_F);
	}
}
