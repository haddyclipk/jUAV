package sw.airborne.subsystems.ins;
import static sw.airborne.subsystems.ins.vf_float.*;
public class Vff {
	  public static float z;       ///< z-position estimate in m (NED, z-down)
	  public static float zdot;    ///< z-velocity estimate in m/s (NED, z-down)
	  public static float bias;    ///< accel bias estimate in m/s^2
	  public static float zdotdot; ///< z-acceleration in m/s^2 (NED, z-down)
	  public static float z_meas;  ///< last measurement
	  public static float [][] P = new float [VFF_STATE_SIZE][VFF_STATE_SIZE];  ///< covariance matrix

}
