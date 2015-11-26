package sw.airborne.subsystems;

import static sw.airborne.subsystems.gps.Gps_sim_nps.*;

public class Gps {
	public static final int GPS_FIX_NONE = 0x00;
	public static final int GPS_FIX_2D = 0x02;
	public static final int GPS_FIX_3D = 0x03;
	
	public static final int GPS_TIMEOUT = 2;
	
	public static final int GPS_NB_CHANNELS = 1;
	
	public static GpsState gps = new GpsState();
	
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
		gps_impl_init();
//		#endif
//
//		#if PERIODIC_TELEMETRY
//		  register_periodic_telemetry(DefaultPeriodic, "GPS", send_gps);
//		  register_periodic_telemetry(DefaultPeriodic, "GPS_INT", send_gps_int);
//		  register_periodic_telemetry(DefaultPeriodic, "GPS_LLA", send_gps_lla);
//		  register_periodic_telemetry(DefaultPeriodic, "GPS_SOL", send_gps_sol);
//		#endif
	}
	
	public static void gps_periodic_check(){
		//TODO sys_time.nb_sec -> System.nanoTime()
		if (System.nanoTime() - gps.last_msg_time > GPS_TIMEOUT) {
			gps.fix = GPS_FIX_NONE;
		}
	}
}
