package sw.airborne.subsystems.ins;
import sw.airborne.math.*;
public class InsInt {
	public  LtpDef_i  ltp_def;
	public  boolean           ltp_initialized;

	/** request to realign horizontal filter.
	 * Sets to current position (local origin unchanged).
	 */
	public  boolean hf_realign;

	/** request to reset vertical filter.
	 * Sets the z-position to zero and resets the the z-reference to current altitude.
	 */
	public  boolean vf_reset;

	/* output LTP NED */
	public  NedCoor_i ltp_pos;
	public  NedCoor_i ltp_speed;
	public  NedCoor_i ltp_accel;

	/* baro */
	public  float baro_z;  ///< z-position calculated from baro in meters (z-down)
	public  float qfe;
	public  boolean baro_initialized;

	//#if USE_SONAR
	public  boolean update_on_agl; ///< use sonar to update agl if available
	//#endif


}
