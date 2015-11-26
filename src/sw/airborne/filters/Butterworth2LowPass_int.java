package sw.airborne.filters;

public class Butterworth2LowPass_int {
	  public static long [] a = new long[2]; ///< denominator gains
	  public static long [] b = new long[2]; ///< numerator gains
	  public static long [] i = new long[2]; ///< input history
	  public static long [] o = new long[2]; ///< output history
	  public static long loop_gain; ///< loop gain

}
