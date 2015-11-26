package sw.airborne.subsystems.ins;

public class InsFloatInv {
	   public static inv_state state;             ///< state vector
	   public static inv_measures meas;           ///< measurement vector
	   public static inv_command cmd;             ///< command vector
	   public static inv_correction_gains corr;   ///< correction gains
	   public static inv_gains gains;             ///< tuning gains

	   public static boolean reset;                       ///< flag to request reset/reinit the filter

}
