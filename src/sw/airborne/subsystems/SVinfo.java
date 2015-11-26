package sw.airborne.subsystems;

public class SVinfo {
	int svid;  ///< Satellite ID
	int flags; ///< bitfield with GPS receiver specific flags
	int qi;    ///< quality bitfield (GPS receiver specific)
	int cno;   ///< Carrier to Noise Ratio (Signal Strength) in dbHz
	int elev;   ///< elevation in deg
	int azim;  ///< azimuth in deg
}
