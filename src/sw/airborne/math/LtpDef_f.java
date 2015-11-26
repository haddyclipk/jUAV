package sw.airborne.math;

public class LtpDef_f {
	public EcefCoor_f ecef; ///< origin of local frame in ECEF
	 public LlaCoor_f  lla; ///< origin of local frame in LLA
	  public FloatMat33 ltp_of_ecef; ///< rotation from ECEF to local frame
	 public float hmsl;
}
