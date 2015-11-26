package sw.airborne.subsystems;

public class Ahrs {
	public static final int AHRS_UNINIT = 0;
	public static final int AHRS_RUNNING = 1;
	public static final int AHRS_LOCKED = 2;

	
	public static AhrsState ahrs = new AhrsState();
	//public static void ahrs_init(){}
	
	public static void ahrs_update_accel(){
		
	}
	
	public static void ahrs_update_gps(){
		
	}
}
