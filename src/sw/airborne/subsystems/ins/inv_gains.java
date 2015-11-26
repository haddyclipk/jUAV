package sw.airborne.subsystems.ins;
import sw.airborne.math.*;

public class inv_gains {
	public static float lv;     ///< Tuning parameter of speed error on attitude
	public static   float lb;     ///< Tuning parameter of mag error on attitude
	public static float mv;     ///< Tuning parameter of horizontal speed error on speed
	public static float mvz;    ///< Tuning parameter of vertical speed error on speed
	public static float mh;     ///< Tuning parameter of baro error on vertical speed
	public static float nx;     ///< Tuning parameter of horizontal position error on position
	public static float nxz;    ///< Tuning parameter of vertical position error on position
	public static float nh;     ///< Tuning parameter of baro error on vertical position
	public static float ov;     ///< Tuning parameter of speed error on gyro biases
	public static float ob;     ///< Tuning parameter of mag error on gyro biases
	public static float rv;     ///< Tuning parameter of speed error on accel biases
	public static float rh;     ///< Tuning parameter of baro error on accel biases (vertical projection)
	public static float sh;     ///< Tuning parameter of baro error on baro bias


}
