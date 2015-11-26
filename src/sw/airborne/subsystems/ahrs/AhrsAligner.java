package sw.airborne.subsystems.ahrs;

import sw.airborne.math.*;

public class AhrsAligner {
	public static Int32Rates lp_gyro;
	public static Int32Vect3 lp_accel;
	public static Int32Vect3 lp_mag;
	public static int noise;
	public static int low_noise_cnt;
	public static int status;
}
