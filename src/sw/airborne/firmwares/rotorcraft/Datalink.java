package sw.airborne.firmwares.rotorcraft;

public class Datalink {
	//TODO: Check return type
	public static int IdOfMsg(int x[]) {
		return (x[1]) ;
	}
	
//	public static void dl_parse_msg() {
//
//		  datalink_time = 0;
//
//		  int msg_id = IdOfMsg(dl_buffer);
//		  switch (msg_id) {
//
//		  case  DL_PING:
//		    {
//		      DOWNLINK_SEND_PONG(DefaultChannel, DefaultDevice);
//		    }
//		    break;
//
//		  case DL_SETTING :
//		    {
//		      if (DL_SETTING_ac_id(dl_buffer) != AC_ID) break;
//		      int i = DL_SETTING_index(dl_buffer);
//		      float var = DL_SETTING_value(dl_buffer);
//		      DlSetting(i, var);
//		      DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, i, var);
//		    }
//		    break;
//
//		  case DL_GET_SETTING :
//		  {
//			  if (DL_GET_SETTING_ac_id(dl_buffer) != AC_ID) break;
//			  int i = DL_GET_SETTING_index(dl_buffer);
//			  float val = settings_get_value(i);
//			  DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, i, val);
//		  }
//		  break;
//
//		  case DL_BLOCK :
//		  {
//			  if (DL_BLOCK_ac_id(dl_buffer) != AC_ID) break;
//			  nav_goto_block(DL_BLOCK_block_id(dl_buffer));
//		  }
//		  break;
//
//		  case DL_MOVE_WP :
//		  {
//			  int ac_id = DL_MOVE_WP_ac_id(dl_buffer);
//			  if (ac_id != AC_ID) break;
//			  if (stateIsLocalCoordinateValid()) {
//				  int wp_id = DL_MOVE_WP_wp_id(dl_buffer);
//				  LlaCoor_i lla;
//				  lla.lat = INT32_RAD_OF_DEG(DL_MOVE_WP_lat(dl_buffer));
//				  lla.lon = INT32_RAD_OF_DEG(DL_MOVE_WP_lon(dl_buffer));
//				  /* WP_alt from message is alt above MSL in cm
//				   * lla.alt is above ellipsoid in mm
//				   */
//				  lla.alt = DL_MOVE_WP_alt(dl_buffer)*10 - state.ned_origin_i.hmsl +
//						  state.ned_origin_i.lla.alt;
//				  nav_move_waypoint_lla(wp_id, lla);
//			  }
//		  }
//		  break;
//		  /* USE_NAVIGATION */
//		
//		  case DL_RC_3CH :
//		if(RADIO_CONTROL_DATALINK_LED)
//		    LED_TOGGLE(RADIO_CONTROL_DATALINK_LED);
//		
//		    parse_rc_3ch_datalink(
//		        DL_RC_3CH_throttle_mode(dl_buffer),
//		        DL_RC_3CH_roll(dl_buffer),
//		        DL_RC_3CH_pitch(dl_buffer));
//		    break;
//		  case DL_RC_4CH :
//		if( RADIO_CONTROL_DATALINK_LED)
//		    LED_TOGGLE(RADIO_CONTROL_DATALINK_LED);
//		
//		    parse_rc_4ch_datalink(
//		        DL_RC_4CH_mode(dl_buffer),
//		        DL_RC_4CH_throttle(dl_buffer),
//		        DL_RC_4CH_roll(dl_buffer),
//		        DL_RC_4CH_pitch(dl_buffer),
//		        DL_RC_4CH_yaw(dl_buffer));
//		    break;
//		 // RADIO_CONTROL_TYPE_DATALINK
//		
//		  case DL_REMOTE_GPS :
//		    // Check if the GPS is for this AC
//		    if (DL_REMOTE_GPS_ac_id(dl_buffer) != AC_ID) break;
//
//		    // Parse the GPS
//		    parse_gps_datalink(
//		      DL_REMOTE_GPS_numsv(dl_buffer),
//		      DL_REMOTE_GPS_ecef_x(dl_buffer),
//		      DL_REMOTE_GPS_ecef_y(dl_buffer),
//		      DL_REMOTE_GPS_ecef_z(dl_buffer),
//		      DL_REMOTE_GPS_lat(dl_buffer),
//		      DL_REMOTE_GPS_lon(dl_buffer),
//		      DL_REMOTE_GPS_alt(dl_buffer),
//		      DL_REMOTE_GPS_hmsl(dl_buffer),
//		      DL_REMOTE_GPS_ecef_xd(dl_buffer),
//		      DL_REMOTE_GPS_ecef_yd(dl_buffer),
//		      DL_REMOTE_GPS_ecef_zd(dl_buffer),
//		      DL_REMOTE_GPS_tow(dl_buffer),
//		      DL_REMOTE_GPS_course(dl_buffer));
//		    break;
//		
//		  default:
//		    break;
//		  }
//		  /* Parse modules datalink */
//		  modules_parse_datalink(msg_id);
//		}
}
