package devices;

public class Gps {
	
	public static final int GPS_FIX_NONE = 0x00;
	public static final int GPS_FIX_2D = 0x02;
	public static final int GPS_FIX_3D = 0x03;
	
	public static final int GPS_TIMEOUT = 2;
	
	public static final int GPS_NB_CHANNELS = 1;
	
	public static GpsState gps;
	public static boolean gps_has_fix;
	public static boolean gps_available;
	
	public Gps(){
		gps = new GpsState();
	}
	
	/** initialize the global GPS state */
	public static void gps_init(){
		gps.fix = GPS_FIX_NONE;
		gps.week = 0;
		gps.tow = 0;
		gps.cacc = 0;

		gps.last_3dfix_ticks = 0;
		gps.last_3dfix_time = 0;
		gps.last_msg_ticks = 0;
		gps.last_msg_time = 0;
		
//		#ifdef GPS_POWER_GPIO
//		  gpio_setup_output(GPS_POWER_GPIO);
//		  GPS_POWER_GPIO_ON(GPS_POWER_GPIO);
//		#endif
//		#ifdef GPS_LED
//		  LED_OFF(GPS_LED);
//		#endif
//		#ifdef GPS_TYPE_H
		
		// Not needed same as constructor.
		
		//gps_impl_init();
		
//		#endif
//
//		#if PERIODIC_TELEMETRY
//		  register_periodic_telemetry(DefaultPeriodic, "GPS", send_gps);
//		  register_periodic_telemetry(DefaultPeriodic, "GPS_INT", send_gps_int);
//		  register_periodic_telemetry(DefaultPeriodic, "GPS_LLA", send_gps_lla);
//		  register_periodic_telemetry(DefaultPeriodic, "GPS_SOL", send_gps_sol);
//		#endif
	}
	
	/** Periodic GPS check.
	 * Marks GPS as lost when no GPS message was received for GPS_TIMEOUT seconds
	 */
	public void gps_periodic_check(){
		//TODO sys_time.nb_sec -> System.nanoTime()
		if (System.nanoTime() - gps.last_msg_time > GPS_TIMEOUT) {
			gps.fix = GPS_FIX_NONE;
		}
	}
	
	//void parse_gps_datalink(uint8_t numsv, int32_t ecef_x, int32_t ecef_y, int32_t ecef_z, int32_t lat, int32_t lon, int32_t alt,
	//int32_t hmsl, int32_t ecef_xd, int32_t ecef_yd, int32_t ecef_zd, uint32_t tow, int32_t course);
//	public void parse_data_link(int numsv, int ecef_x, int ecef_y, int ecef_z, int lat, 
//			int lon, int alt,int hmsl, int ecef_xd, int ecef_yd, int ecef_zd, int tow, int course){
//		 gps.lla_pos.lat = RadOfDeg(lat);
//		  gps.lla_pos.lon = RadOfDeg(lon);
//		  gps.lla_pos.alt = alt;
//		  gps.hmsl        = hmsl;
//
//		  gps.ecef_pos.x = ecef_x;
//		  gps.ecef_pos.y = ecef_y;
//		  gps.ecef_pos.z = ecef_z;
//
//		  gps.ecef_vel.x = ecef_xd;
//		  gps.ecef_vel.y = ecef_yd;
//		  gps.ecef_vel.z = ecef_zd;
//
//		  gps.course = course;
//		  gps.num_sv = numsv;
//		  gps.tow = tow;
//		  gps.fix = GPS_FIX_3D;
//		  gps_available = true;
//		  
//	}
	public static void gps_feed_value(Long[] values) {
		  gps.ecef_pos.x = values[1];
		  gps.ecef_pos.y = values[2];
		  gps.ecef_pos.z = values[3];
		  gps.ecef_vel.x = values[4];
		  gps.ecef_vel.y = values[5];
		  gps.ecef_vel.z = values[6];
		  //ecef pos seems to be based on geocentric model, hence we get a very high alt when converted to lla
		  gps.lla_pos.lat = values[7];
		  gps.lla_pos.lon = values[8];
		  gps.lla_pos.alt = values[9];
		  gps.hmsl        = values[10];

		  /* calc NED speed from ECEF */
		  gps.ned_vel.x = values[11];
		  gps.ned_vel.y = values[12];
		  gps.ned_vel.z = values[13];

		  /* horizontal and 3d ground speed in cm/s */
		  gps.gspeed = values[14];
		  gps.speed_3d = values[15];

		  /* ground course in radians * 1e7 */
		  gps.course = values[16];

//		#if GPS_USE_LATLONG
//		  /* Computes from (lat, long) in the referenced UTM zone */
//		  struct LlaCoor_f lla_f;
//		  lla_f.lat = ((float) gps.lla_pos.lat) / 1e7;
//		  lla_f.lon = ((float) gps.lla_pos.lon) / 1e7;
//		  struct UtmCoor_f utm_f;
//		  utm_f.zone = nav_utm_zone0;
//		  /* convert to utm */
//		  utm_of_lla_f(&utm_f, &lla_f);
//		  /* copy results of utm conversion */
//		  gps.utm_pos.east = utm_f.east*100;
//		  gps.utm_pos.north = utm_f.north*100;
//		  gps.utm_pos.alt = gps.lla_pos.alt;
//		  gps.utm_pos.zone = nav_utm_zone0;
//		#endif

		  if (gps_has_fix)
		    gps.fix = GPS_FIX_3D;
		  else
		    gps.fix = GPS_FIX_NONE;
		  gps_available = true;
		}
	
	public void gps_impl_init(){
		gps_available = false;
		gps_has_fix = true;
	}
	
}
