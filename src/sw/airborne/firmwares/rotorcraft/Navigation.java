package sw.airborne.firmwares.rotorcraft;

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
import static sw.airborne.firmwares.rotorcraft.guidance.Guidance_h.*;
import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_none.*;
import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_rate.*;
import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_rc_setpoint.*;
import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_euler_int.*;
import static sw.airborne.firmwares.rotorcraft.Autopilot.*;
import static sw.airborne.subsystems.ins.Ins_int.*;
import static sw.airborne.Paparazzi.*;

public class Navigation {
	public static final int VERTICAL_MODE_MANUAL=      0;
	public static final int VERTICAL_MODE_CLIMB=       1;
	public static final int VERTICAL_MODE_ALT   =      2;

	public static final int NB_WAYPOINT = 10;//////??????
	public static final int nb_waypoint = NB_WAYPOINT;
	public static EnuCoor_i[] waypoints = new EnuCoor_i[10];
	public static EnuCoor_i navigation_target;
	public static EnuCoor_i navigation_carrot;
	public static EnuCoor_i nav_last_point;
	
	public static int ground_alt;
	
	public static double FAILSAFE_MODE_DISTANCE = (1.5*MAX_DIST_FROM_HOME);
	public static final double max_dist_from_home = MAX_DIST_FROM_HOME;
	public static final double max_dist2_from_home = MAX_DIST_FROM_HOME * MAX_DIST_FROM_HOME;
	public static final double failsafe_mode_dist2 = FAILSAFE_MODE_DISTANCE * FAILSAFE_MODE_DISTANCE;
	
	public static float dist2_to_home;
	public static boolean too_far_from_home;

	public static int horizontal_mode;
	public static int  nav_segment_start, nav_segment_end;
	public static int  nav_circle_centre;
	public static long  nav_circle_radius, nav_circle_qdr, nav_circle_radians;

	public static long  nav_leg_progress;
	public static long  nav_leg_length;

	public static int  nav_roll, nav_pitch;
	public static long  nav_heading, nav_course;
	public static float nav_radius;
	
	public static final int DEFAULT_CIRCLE_RADIUS = 5;
	
	public static int vertical_mode;
	public static int nav_throttle;
	public static long nav_climb, nav_altitude, nav_flight_altitude;
	public static float flight_altitude;
	
	public static int CLOSE_TO_WAYPOINT = (15 << 8);
	public static int CARROT_DIST = (12 << 8);
	
	/** minimum horizontal distance to waypoint to mark as arrived */
	public static final double ARRIVED_AT_WAYPOINT = 3.0;
	private static final int WP_HOME = 0;///???????????????? 
//	
//	public static void send_nav_status() {
////		DOWNLINK_SEND_ROTORCRAFT_NAV_STATUS(DefaultChannel, DefaultDevice,
////				block_time, stage_time,
////				nav_block, nav_stage,
////				horizontal_mode);
//		if (horizontal_mode == HORIZONTAL_MODE_ROUTE) {
//			float sx = POS_FLOAT_OF_BFP(waypoints[nav_segment_start].x);
//			float sy = POS_FLOAT_OF_BFP(waypoints[nav_segment_start].y);
//			float ex = POS_FLOAT_OF_BFP(waypoints[nav_segment_end].x);
//			float ey = POS_FLOAT_OF_BFP(waypoints[nav_segment_end].y);
//			//DOWNLINK_SEND_SEGMENT(DefaultChannel, DefaultDevice, sx, sy, ex, ey);
//		}
//		else if (horizontal_mode == HORIZONTAL_MODE_CIRCLE) {
//			float cx = POS_FLOAT_OF_BFP(waypoints[nav_circle_centre].x);
//			float cy = POS_FLOAT_OF_BFP(waypoints[nav_circle_centre].y);
//			float r = POS_FLOAT_OF_BFP(nav_circle_radius);
//			//DOWNLINK_SEND_CIRCLE(DefaultChannel, DefaultDevice, cx, cy, r);
//		}
//	}
//	
//	private static int i_send_wp_moved;
//	public static void send_wp_moved() {
//		//static int i;
//		i_send_wp_moved++; if (i_send_wp_moved >= nb_waypoint) i_send_wp_moved = 0;
////		DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice,
////				i_send_wp_moved,
////				(waypoints[i].x),
////				(waypoints[i].y),
////				(waypoints[i].z));
//	}
//	
	public static void nav_init() {
		// convert to
		EnuCoor_f wp_tmp_float[] = WAYPOINTS;
		// init int32 waypoints
		int i = 0;
		for (i = 0; i < nb_waypoint; i++) {
			waypoints[i].x = POS_BFP_OF_REAL(wp_tmp_float[i].x);
			waypoints[i].y = POS_BFP_OF_REAL(wp_tmp_float[i].y);
			waypoints[i].z = POS_BFP_OF_REAL((wp_tmp_float[i].z - GROUND_ALT));
		}
		nav_block = 0;
		nav_stage = 0;
		ground_alt = POS_BFP_OF_REAL(GROUND_ALT);
		nav_altitude = POS_BFP_OF_REAL(SECURITY_HEIGHT);
		nav_flight_altitude = nav_altitude;
		flight_altitude = (float) SECURITY_ALT;
		INT32_VECT3_COPY(navigation_target, waypoints[WP_HOME]);
		INT32_VECT3_COPY(navigation_carrot, waypoints[WP_HOME]);

		horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
		vertical_mode = VERTICAL_MODE_ALT;

		nav_roll = 0;
		nav_pitch = 0;
		nav_heading = 0;
		nav_course = 0;
		nav_radius = DEFAULT_CIRCLE_RADIUS;
		nav_throttle = 0;
		nav_climb = 0;
		nav_leg_progress = 0;
		nav_leg_length = 1;

		too_far_from_home = false;
		dist2_to_home = 0;

//		if(PERIODIC_TELEMETRY){
//			register_periodic_telemetry(DefaultPeriodic, "ROTORCRAFT_NAV_STATUS", send_nav_status);
//			register_periodic_telemetry(DefaultPeriodic, "WP_MOVED", send_wp_moved);
//		}
	}
//	
	public static void nav_advance_carrot() {
		EnuCoor_i pos = stateGetPositionEnu_i();
		/* compute a vector to the waypoint */
		Int32Vect2 path_to_waypoint=new Int32Vect2();;
		VECT2_DIFF(path_to_waypoint, navigation_target, pos);

		/* saturate it */
		VECT2_STRIM(path_to_waypoint, -(1<<15), (1<<15));

		int dist_to_waypoint = 0;
		dist_to_waypoint = INT32_VECT2_NORM(dist_to_waypoint, path_to_waypoint);

		if (dist_to_waypoint < CLOSE_TO_WAYPOINT) {
			VECT2_COPY(navigation_carrot, navigation_target);
		}
		else {
			Int32Vect2 path_to_carrot = new Int32Vect2();
			VECT2_SMUL(path_to_carrot, path_to_waypoint, CARROT_DIST);
			VECT2_SDIV(path_to_carrot, path_to_carrot, dist_to_waypoint);
			VECT2_SUM(navigation_carrot, path_to_carrot, pos);
		}
	}
//	
//	public static void nav_circle(int wp_center, int radius) {
//		if (radius == 0) {
//			VECT2_COPY(navigation_target, waypoints[wp_center]);
//		}
//		else {
//			Int32Vect2 pos_diff;
//			VECT2_DIFF(pos_diff, stateGetPositionEnu_i(), waypoints[wp_center]);
//			// go back to half metric precision or values are too large
//			//INT32_VECT2_RSHIFT(pos_diff,pos_diff,INT32_POS_FRAC/2);
//			// store last qdr
//			long last_qdr = nav_circle_qdr;
//			// compute qdr
//			//INT32_ATAN2(nav_circle_qdr, pos_diff.y, pos_diff.x);
//			 int c1 = INT32_ANGLE_PI_4;        
//		     int c2 = 3 * INT32_ANGLE_PI_4;        
//		     long abs_y = Math.abs(pos_diff.y) + 1;          
//		    long r;                      
//		    if ( (pos_diff.x >= 0)) {                               
//		      r = (((pos_diff.x)-abs_y)<<R_FRAC) / ((pos_diff.x)+abs_y);    
//		      (nav_circle_qdr) = c1 - ((c1 * r)>>R_FRAC);               
//		    }                           
//		    else {                      
//		      r = (((pos_diff.x)+abs_y)<<R_FRAC) / (abs_y-(pos_diff.x));    
//		      (nav_circle_qdr) = c2 - ((c1 * r)>>R_FRAC);           
//		    }                           
//		    if ((pos_diff.y)<0)                     
//		      (nav_circle_qdr) = -(nav_circle_qdr);                 
//			// increment circle radians
//			if (nav_circle_radians != 0) {
//				long angle_diff = nav_circle_qdr - last_qdr;
//				//INT32_ANGLE_NORMALIZE(angle_diff);
//				while ((angle_diff) > INT32_ANGLE_PI)  (angle_diff) -= INT32_ANGLE_2_PI;    
//				while ((angle_diff) < -INT32_ANGLE_PI) (angle_diff) += INT32_ANGLE_2_PI;  
//				nav_circle_radians += angle_diff;
//			}
//			else {
//				// Smallest angle to increment at next step
//				nav_circle_radians = 1;
//			}
//
//			// direction of rotation
//			int sign_radius = radius > 0 ? 1 : -1;
//			// absolute radius
//			int abs_radius = Math.abs(radius);
//			// carrot_angle
//			long carrot_angle = ((CARROT_DIST<<INT32_ANGLE_FRAC) / abs_radius);
//			Bound(carrot_angle, (INT32_ANGLE_PI / 16), INT32_ANGLE_PI_4);
//			carrot_angle = nav_circle_qdr - sign_radius * carrot_angle;
//			long s_carrot, c_carrot;
//			s_carrot = PPRZ_ITRIG_SIN(carrot_angle);
//			c_carrot = PPRZ_ITRIG_COS( carrot_angle);
//			// compute setpoint
//			VECT2_ASSIGN(pos_diff, abs_radius * c_carrot, abs_radius * s_carrot);
//			INT32_VECT2_RSHIFT(pos_diff, pos_diff, INT32_TRIG_FRAC);
//			VECT2_SUM(navigation_target, waypoints[wp_center], pos_diff);
//		}
//		nav_circle_centre = wp_center;
//		nav_circle_radius = radius;
//		horizontal_mode = HORIZONTAL_MODE_CIRCLE;
//	}
//	
//	public static void nav_route(int wp_start, int wp_end) {
//		Int32Vect2 wp_diff,pos_diff;
//		VECT2_DIFF(wp_diff, waypoints[wp_end],waypoints[wp_start]);
//		VECT2_DIFF(pos_diff, stateGetPositionEnu_i(), waypoints[wp_start]);
//		// go back to metric precision or values are too large
//		INT32_VECT2_RSHIFT(wp_diff,wp_diff,INT32_POS_FRAC);
//		INT32_VECT2_RSHIFT(pos_diff,pos_diff,INT32_POS_FRAC);
//		long leg_length2 = Max((wp_diff.x * wp_diff.x + wp_diff.y * wp_diff.y),1);
//		//INT32_SQRT(nav_leg_length,leg_length2);
//		nav_leg_length =(int) Math.sqrt(leg_length2);//TODO: change here
//		nav_leg_progress = (pos_diff.x * wp_diff.x + pos_diff.y * wp_diff.y) / nav_leg_length;
//		int progress = Max((CARROT_DIST >> INT32_POS_FRAC), 0);
//		nav_leg_progress += progress;
//		long prog_2 = nav_leg_length;// + progress / 2;
//		Bound(nav_leg_progress, 0, prog_2);
//		Int32Vect2 progress_pos;
//		VECT2_SMUL(progress_pos, wp_diff, nav_leg_progress);
//		VECT2_SDIV(progress_pos, progress_pos, nav_leg_length);
//		INT32_VECT2_LSHIFT(progress_pos,progress_pos,INT32_POS_FRAC);
//		VECT2_SUM(navigation_target,waypoints[wp_start],progress_pos);
//		//printf("target %d %d | p %d %d | s %d %d | l %d %d %d\n",
//		//    navigation_target.x,
//		//    navigation_target.y,
//		//    progress_pos.x,
//		//    progress_pos.y,
//		//    waypoints[wp_start].x,
//		//    waypoints[wp_start].y,
//		//    leg_length, leg_length2, nav_leg_progress);
//		//fflush(stdout);
//
//		nav_segment_start = wp_start;
//		nav_segment_end = wp_end;
//		horizontal_mode = HORIZONTAL_MODE_ROUTE;
//	}
//	
//	public static boolean nav_approaching_from(int wp_idx, int from_idx, double approaching_time) {
//		int dist_to_point;
//		Int32Vect2 diff;
//		EnuCoor_i pos = stateGetPositionEnu_i();
//
//		/* if an approaching_time is given, estimate diff after approching_time secs */
//		if (approaching_time > 0) {
//			Int32Vect2 estimated_pos;
//			Int32Vect2 estimated_progress;
//			EnuCoor_i speed = stateGetSpeedEnu_i();
//			VECT2_SMUL(estimated_progress, speed, approaching_time);
//			INT32_VECT2_RSHIFT(estimated_progress, estimated_progress, (INT32_SPEED_FRAC - INT32_POS_FRAC));
//			VECT2_SUM(estimated_pos, pos, estimated_progress);
//			VECT2_DIFF(diff, waypoints[wp_idx], estimated_pos);
//		}
//		/* else use current position */
//		else {
//			VECT2_DIFF(diff, waypoints[wp_idx], pos);
//		}
//		/* compute distance of estimated/current pos to target wp
//		 * distance with half metric precision (6.25 cm)
//		 */
//		INT32_VECT2_RSHIFT(diff, diff, INT32_POS_FRAC/2);
//		INT32_VECT2_NORM(dist_to_point, diff);
//
//		/* return TRUE if we have arrived */
//		if (dist_to_point < BFP_OF_REAL(ARRIVED_AT_WAYPOINT, INT32_POS_FRAC/2))
//			return true;
//
//		/* if coming from a valid waypoint */
//		if (from_idx > 0 && from_idx < NB_WAYPOINT) {
//			/* return TRUE if normal line at the end of the segment is crossed */
//			Int32Vect2 from_diff;
//			VECT2_DIFF(from_diff, waypoints[wp_idx], waypoints[from_idx]);
//			INT32_VECT2_RSHIFT(from_diff, from_diff, INT32_POS_FRAC/2);
//			return (diff.x * from_diff.x + diff.y * from_diff.y < 0);
//		}
//
//		return false;
//	}
//	
//	
//	private static int wp_entry_time = 0;
//	private static boolean wp_reached = false;
//	private static int wp_idx_last = 0;
//	public static boolean nav_check_wp_time(int wp_idx, double stay_time) {
//		int time_at_wp;
//		int dist_to_point;
////		static int wp_entry_time = 0;
////		static boolean wp_reached = false;
////		static int wp_idx_last = 0;
//		Int32Vect2 diff;
//
//		if (wp_idx_last != wp_idx) {
//			wp_reached = false;
//			wp_idx_last = wp_idx;
//		}
//		VECT2_DIFF(diff, waypoints[wp_idx], stateGetPositionEnu_i());
//		INT32_VECT2_RSHIFT(diff, diff, INT32_POS_FRAC/2);
//		INT32_VECT2_NORM(dist_to_point, diff);
//		if (dist_to_point < BFP_OF_REAL(ARRIVED_AT_WAYPOINT, INT32_POS_FRAC/2)){
//			if (!wp_reached) {
//				wp_reached = true;
//				wp_entry_time = autopilot_flight_time;
//				time_at_wp = 0;
//			}
//			else {
//				time_at_wp = autopilot_flight_time - wp_entry_time;
//			}
//		}
//		else {
//			time_at_wp = 0;
//			wp_reached = false;
//		}
//		if (time_at_wp > stay_time) return true;
//		return false;
//	}
//	
	private static long last_nav_alt = 0;
	public static void nav_set_altitude() {
		if (Math.abs(nav_altitude - last_nav_alt) > (POS_BFP_OF_REAL((float)0.2))) {
			nav_flight_altitude = nav_altitude;
			last_nav_alt = nav_altitude;
		}
	}
//	
//	/** Reset the geographic reference to the current GPS fix */
//	public static int nav_reset_reference( ) {
//	  ins_reset_local_origin();
//	  ground_alt = POS_BFP_OF_REAL(state.ned_origin_f.hmsl);
//	  return 0;
//	}
//	
//	public static int nav_reset_alt( ) {
//		ins_reset_altitude_ref();
//		ground_alt = POS_BFP_OF_REAL(state.ned_origin_f.hmsl);
//		return 0;
//	}
//
//	public static void nav_init_stage(  ) {
//		INT32_VECT3_COPY(nav_last_point, stateGetPositionEnu_i());
//		stage_time = 0;
//		nav_circle_radians = 0;
//		horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
//	}
	public static int block_time=0;
	public static int stage_time=0;
	private static int SIXTEEN_PERIODIC_TASK = 0;
	public static void nav_periodic_task() {
		//RunOnceEvery(16, { stage_time++;  block_time++; });
		SIXTEEN_PERIODIC_TASK++;					
		if (SIXTEEN_PERIODIC_TASK >= 16) {			
			SIXTEEN_PERIODIC_TASK = 0;					
			
			stage_time++;  
			block_time++;					
		}	
		/* from flight_plan.h */
		//auto_nav();             ???????

		/* run carrot loop */
		nav_run();
	}
	
//	// add   nav_run
	public static void nav_run(){
		if(GUIDANCE_H_USE_REF){
			navigation_carrot.x=navigation_target.x;
			navigation_carrot.y=navigation_target.y;
		}else {nav_advance_carrot();}
		nav_set_altitude();
	}
	
//	
//	
//	
//	public static void nav_move_waypoint_lla(int wp_id, LlaCoor_i new_lla_pos) {
//		if (stateIsLocalCoordinateValid()) {
//			EnuCoor_i enu;
//			enu_of_lla_point_i(enu, state.ned_origin_i, new_lla_pos);
//			enu.x = POS_BFP_OF_REAL(enu.x)/100;
//			enu.y = POS_BFP_OF_REAL(enu.y)/100;
//			enu.z = POS_BFP_OF_REAL(enu.z)/100;
//			nav_move_waypoint(wp_id, enu);
//		}
//	}
//
//	public static void nav_move_waypoint(int wp_id,EnuCoor_i  new_pos) {
//		if (wp_id < nb_waypoint) {
//			INT32_VECT3_COPY(waypoints[wp_id],(new_pos));
////			DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, wp_id, (new_pos.x),
////					(new_pos.y), (new_pos.z));
//		}
//	}
//	
//	private static int TEN_PERIODIC_TASK = 0;
//	public static void navigation_update_wp_from_speed(int wp,Int16Vect3 speed_sp, int heading_rate_sp ) {
//		//  MY_ASSERT(wp < nb_waypoint); FIXME
//		long s_heading, c_heading;
//		s_heading=PPRZ_ITRIG_SIN( nav_heading);
//		c_heading=PPRZ_ITRIG_COS( nav_heading);
//		// FIXME : scale POS to SPEED
//		Int32Vect3 delta_pos;
//		VECT3_SDIV(delta_pos, speed_sp, NAV_FREQ); /* fixme :make sure the division is really a >> */
//		INT32_VECT3_RSHIFT(delta_pos, delta_pos, (INT32_SPEED_FRAC-INT32_POS_FRAC));
//		waypoints[wp].x += (s_heading * delta_pos.x + c_heading * delta_pos.y) >> INT32_TRIG_FRAC;
//		waypoints[wp].y += (c_heading * delta_pos.x - s_heading * delta_pos.y) >> INT32_TRIG_FRAC;
//		waypoints[wp].z += delta_pos.z;
//		int delta_heading = heading_rate_sp / NAV_FREQ;
//		delta_heading = delta_heading >> (INT32_SPEED_FRAC-INT32_POS_FRAC);
//		nav_heading += delta_heading;
//
//		//INT32_COURSE_NORMALIZE(nav_heading);
//		 while ((nav_heading) < 0) (nav_heading) += INT32_ANGLE_2_PI;                  
//		    while ((nav_heading) >= INT32_ANGLE_2_PI)  (nav_heading) -= INT32_ANGLE_2_PI; 
//		//RunOnceEvery(10,DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, wp, (waypoints[wp].x), (waypoints[wp].y), (waypoints[wp].z)));
//		TEN_PERIODIC_TASK++;					
//		if (TEN_PERIODIC_TASK >= 10) {			
//			TEN_PERIODIC_TASK = 0;					
//			//DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, wp, (waypoints[wp].x), (waypoints[wp].y), (waypoints[wp].z));					
//		}	
//	}
//	
//	public static boolean nav_detect_ground() {
//		if (!autopilot_ground_detected) return false;
//		autopilot_ground_detected = false;
//		return true;
//	}
//	
//	public static boolean nav_is_in_flight() {
//		return autopilot_in_flight;
//	}
//	
	/** Home mode navigation */
	public static void nav_home() {
		horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
		INT32_VECT3_COPY(navigation_target, waypoints[WP_HOME]);

		vertical_mode = VERTICAL_MODE_ALT;
		nav_altitude = waypoints[WP_HOME].z;
		nav_flight_altitude = nav_altitude;

		/* run carrot loop */
		nav_run();
	}
//	
//	/** Computes squared distance to the HOME waypoint potentially sets
//	 * #too_far_from_home
//	 */
	public static void compute_dist2_to_home() {
	  EnuCoor_i pos = stateGetPositionEnu_i();
	  Int32Vect2 home_d = new Int32Vect2();
	  VECT2_DIFF(home_d, waypoints[WP_HOME], pos);
	  INT32_VECT2_RSHIFT(home_d, home_d, 8);
	  dist2_to_home = (float)(home_d.x * home_d.x + home_d.y * home_d.y);
	  too_far_from_home = dist2_to_home > max_dist2_from_home;
	}
//	
//	/** Set nav_heading in degrees. */
//	public static boolean nav_set_heading_rad(float rad) {
//	  nav_heading = ANGLE_BFP_OF_REAL(rad);
//	//  INT32_COURSE_NORMALIZE(nav_heading);
//	  while ((nav_heading) < 0) (nav_heading) += INT32_ANGLE_2_PI;                  
//	    while ((nav_heading) >= INT32_ANGLE_2_PI)  (nav_heading) -= INT32_ANGLE_2_PI; 
//	  return false;
//	}
//
//	/** Set nav_heading in degrees. */
//	public static boolean nav_set_heading_deg(float deg) {
//	  return nav_set_heading_rad((float)((deg) * (3.14159/180.0)));
//	}
//	
//	/** Set heading to point towards x,y position in local coordinates */
//	public static boolean nav_set_heading_towards(float x, float y) {
//	  FloatVect2 target = new FloatVect2();
//	  target.x=x;
//	  target.y=y;
//	  FloatVect2 pos_diff;
//	  VECT2_DIFF(pos_diff, target, stateGetPositionEnu_f());
//	  // don't change heading if closer than 0.5m to target
//	  if (FLOAT_VECT2_NORM2(pos_diff) > 0.25) {
//	    float heading_f = (float) Math.atan2(pos_diff.x, pos_diff.y);
//	    nav_heading = ANGLE_BFP_OF_REAL(heading_f);
//	  }
//	  // return false so it can be called from the flightplan
//	  // meaning it will continue to the next stage
//	  return false;
//	}
//	
//	/** Set heading in the direction of a waypoint */
//	public static boolean nav_set_heading_towards_waypoint(int wp) {
//	  return nav_set_heading_towards(WaypointX(wp), WaypointY(wp));
//	}
//	
//	//---------------------------------------Start of Navigation.h-------------------------
	public static final int NAV_FREQ = 16;
	public static final int HORIZONTAL_MODE_WAYPOINT = 0;
	public static final int HORIZONTAL_MODE_ROUTE = 1;
	public static final int HORIZONTAL_MODE_CIRCLE = 2;
	public static final int HORIZONTAL_MODE_ATTITUDE = 3;
	
	public static final int CARROT = 0;
	
//	public static boolean NavKillThrottle(){
//		if (autopilot_mode == AP_MODE_NAV) { autopilot_set_motors_on(false); } 
//		return false;
//	}
//	
//	public static boolean NavResurrect(){
//		if (autopilot_mode == AP_MODE_NAV) { autopilot_set_motors_on(false); } 
//		return false;
//	}
//	
//	public static boolean NavSetGroundReferenceHere() { nav_reset_reference(); return false; }
//	public static boolean NavSetAltitudeReferenceHere() { nav_reset_alt(); return false; }
//	public static boolean NavSetWaypointHere(int _wp) { VECT2_COPY(waypoints[_wp], stateGetPositionEnu_i()); return false; }
//	public static boolean NavCopyWaypoint(int _wp1, int _wp2) { VECT2_COPY(waypoints[_wp1], waypoints[_wp2]); return false;  }
//	
//	public static float WaypointX(int _wp) {return POS_FLOAT_OF_BFP(waypoints[_wp].x);}
//	public static float WaypointY(int _wp) {return POS_FLOAT_OF_BFP(waypoints[_wp].y);}
//	public static float WaypointZ(int _wp) {return POS_FLOAT_OF_BFP(waypoints[_wp].z);}
//	
//	public static double Height(double h){return h;}
//	
//	/** Normalize a degree angle between 0 and 359 */
//	public static double NormCourse(double x){
//		while (x < 0) return x += 360; 
//		while (x >= 360) return x -= 360; 
//	}
//	
//	/*********** Navigation to  waypoint *************************************/
//	public static void NavGotoWaypoint(int _wp) { 
//		  horizontal_mode = HORIZONTAL_MODE_WAYPOINT; 
//		  INT32_VECT3_COPY( navigation_target, waypoints[_wp]); 
//		}
//	
//	public static void  NavCircleWaypoint(int _center, double _radius) { 
//		horizontal_mode = HORIZONTAL_MODE_CIRCLE; 
//		nav_circle(_center, POS_BFP_OF_REAL(_radius)); 
//	}
//	
//	public static double NavCircleCount() {return (Math.abs(nav_circle_radians) / INT32_ANGLE_2_PI);}
//	
//	public static int NavCircleQdr() { 
//		//int qdr = INT32_DEG_OF_RAD(INT32_ANGLE_2_PI_2 - nav_circle_qdr) >> INT32_ANGLE_FRAC; TODO: where is this defined 
//		int qdr;
//		NormCourse(qdr); 
//		return qdr;
//	}
//	
//	/** True if x (in degrees) is close to the current QDR (less than 10 degrees)*/
//	public static void NavQdrCloseTo(double x) {}
//	public static void NavCourseCloseTo(double x) {}
//	
//	/*********** Navigation along a line *************************************/
//	public static void NavSegment(int _start, int _end){
//		horizontal_mode = HORIZONTAL_MODE_ROUTE; 
//		  nav_route(_start, _end); 
//	}
//	
//	
//	/** Nav glide routine */
//	public static void NavGlide(int _last_wp, int _wp) { 
//	  long start_alt = waypoints[_last_wp].z; 
//	  long diff_alt = waypoints[_wp].z - start_alt; 
//	  long alt = start_alt + ((diff_alt * nav_leg_progress) / nav_leg_length); 
//	  NavVerticalAltitudeMode(POS_FLOAT_OF_BFP(alt),0); 
//	}
//	
//	/** Proximity tests on approaching a wp */
//	//public static boolean nav_approaching_from(int wp_idx, int from_idx, int approaching_time){return false;}
//	public static boolean NavApproaching(int wp, double time){ return nav_approaching_from(wp, 0, time);}
//	public static boolean NavApproachingFrom(int wp, int from, double time) { return nav_approaching_from(wp, from, time);}
//	
//	/** Check the time spent in a radius of 'ARRIVED_AT_WAYPOINT' around a wp  */
//	//public static boolean nav_check_wp_time(int wp_idx, int stay_time){return false;}
//	public static boolean NavCheckWaypointTime(int wp, double time) { return nav_check_wp_time(wp, time);}
//	
//	/** Set the climb control to auto-throttle with the specified pitch
//    pre-command */
//	public static void NavVerticalAutoThrottleMode(double _pitch) { 
//		nav_pitch = ANGLE_BFP_OF_REAL(_pitch); 
//	}
//	
//	/** Set the climb control to auto-pitch with the specified throttle
//    pre-command */
//	public static void NavVerticalAutoPitchMode(int _throttle) {}
//
//	/** Set the vertical mode to altitude control with the specified altitude
//	setpoint and climb pre-command. */
//	public static void NavVerticalAltitudeMode(double _alt,double _pre_climb) { 
//		vertical_mode = VERTICAL_MODE_ALT; 
//		nav_altitude = POS_BFP_OF_REAL(_alt); 
//	}
//
//	/** Set the vertical mode to climb control with the specified climb setpoint */
//	public static void NavVerticalClimbMode(double _climb) { 
//		vertical_mode = VERTICAL_MODE_CLIMB; 
//		nav_climb = SPEED_BFP_OF_REAL(_climb); 
//	}
//	
//	/** Set the vertical mode to fixed throttle with the specified setpoint */
//	public static void NavVerticalThrottleMode(int _throttle) { 
//	  vertical_mode = VERTICAL_MODE_MANUAL;      
//	  nav_throttle = _throttle;                  
//	}
//
//	public static void NavHeading(double _course) {}
//
//	public static void NavAttitude(double _roll) { 
//	  horizontal_mode = HORIZONTAL_MODE_ATTITUDE; 
//	  nav_roll = ANGLE_BFP_OF_REAL(_roll); 
//	}
//
//	public static boolean NavStartDetectGround() { 
//		autopilot_detect_ground_once = true; 
//		return false; 
//	}
//	public static boolean NavDetectGround() { return nav_detect_ground();}
//
//	public static void nav_IncreaseShift(double x) {}
//
//	public static void nav_SetNavRadius(double x) {}
//
//
//	public static void navigation_SetFlightAltitude(double x) { 
//	  flight_altitude = (float) x; 
//	  nav_flight_altitude = POS_BFP_OF_REAL(flight_altitude) - ground_alt; 
//	}
//
//
//	public static double GetPosX(){ return stateGetPositionEnu_f().x;}
//	public static double GetPosY(){ return stateGetPositionEnu_f().y;}
//	
//	public static double GetPosAlt() { return stateGetPositionEnu_f().z + ground_alt;}
}


