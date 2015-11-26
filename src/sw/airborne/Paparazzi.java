package sw.airborne;
import sw.airborne.math.*;
public class Paparazzi {
	public static final int MAX_PPRZ = 9600;
	public static final int MIN_PPRZ = -MAX_PPRZ;
	public static final int COMMANDS_NB = 4;
	public static final int COMMAND_ROLL = 0;
	public static final int COMMAND_PITCH= 1;
	public static final int COMMAND_YAW= 2;
	public static final int COMMAND_THRUST= 3;
	public static final double MAX_DIST_FROM_HOME =  150. ;
	
	public static final int NB_WAYPOINT = 10;
	public static EnuCoor_f[] WAYPOINTS = new EnuCoor_f[10];//TODO
	
	public static final double GROUND_ALT = 147.;
	public static final double SECURITY_HEIGHT = 2.;
	public static final double SECURITY_ALT = 149.;
	
	public static char nav_block;
	public static char nav_stage;
	
}
