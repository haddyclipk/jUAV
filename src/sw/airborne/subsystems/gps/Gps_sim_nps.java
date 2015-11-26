package sw.airborne.subsystems.gps;

public class Gps_sim_nps {
	
	public static boolean gps_has_fix;
	public static boolean gps_available;
	
	public static void gps_impl_init(){
		gps_available = false;
		gps_has_fix = true;
	}
}
