package sw.airborne.math;

public class LtpDef_i {
	 public EcefCoor_i ecef;        ///< Reference point in ecef
	  public LlaCoor_i  lla;         ///< Reference point in lla
	  public Int32Mat33 ltp_of_ecef; ///< Rotation matrix
	  public int hmsl; 
	  public LtpDef_i clone(){
		  return this;
	  }
}
