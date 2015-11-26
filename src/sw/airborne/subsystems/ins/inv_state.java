package sw.airborne.subsystems.ins;
import sw.airborne.math.*;
public class inv_state {
	   public static FloatQuat quat;  ///< Estimated attitude (quaternion)
	   public static FloatRates bias; ///< Estimated gyro biases
	   public static NedCoor_f speed; ///< Estimates speed
	   public static NedCoor_f pos;   ///< Estimates position
	   public static float hb;		            ///< Estimates barometers bias
	   public static float as;               ///< Estimated accelerometer sensitivity
	//public static float cs;               ///< Estimated magnetic sensitivity
}
