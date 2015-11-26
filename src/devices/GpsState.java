package devices;

import sw.airborne.math.*;

public class GpsState {
	public EcefCoor_i ecef_pos;    ///< position in ECEF in cm
	public LlaCoor_i lla_pos;      ///< position in LLA (lat,lon: rad*1e7; alt: mm over ellipsoid)
	public UtmCoor_i utm_pos;      ///< position in UTM (north,east: cm; alt: mm over ellipsoid)
	public long hmsl;                  ///< height above mean sea level in mm
	public EcefCoor_i ecef_vel;    ///< speed ECEF in cm/s
	public NedCoor_i ned_vel;      ///< speed NED in cm/s
	public Long gspeed;                ///< norm of 2d ground speed in cm/s
	public Long speed_3d;              ///< norm of 3d speed in cm/s
	public Long course;                ///< GPS course over ground in rad*1e7, [0, 2*Pi]*1e7 (CW/north)
	public int pacc;                 ///< position accuracy in cm
	public int sacc;                 ///< speed accuracy in cm/s
	public int cacc;                 ///< course accuracy in rad*1e7
	public int pdop;                 ///< position dilution of precision scaled by 100
	public int num_sv;                ///< number of sat in fix
	public int fix;                   ///< status of fix
	public int week;                  ///< GPS week
	public int tow;                  ///< GPS time of week in ms

	public int nb_channels;           ///< Number of scanned satellites
	//SVinfo svinfos[] = new SVinfo[GPS_NB_CHANNELS]; ///< holds information from the Space Vehicles (Satellites)
	public SVinfo svinfos[] = new SVinfo[1]; ///< holds information from the Space Vehicles (Satellites)

	public int last_3dfix_ticks;     ///< cpu time ticks at last valid 3D fix
	public int last_3dfix_time;      ///< cpu time in sec at last valid 3D fix
	public int last_msg_ticks;       ///< cpu time ticks at last received GPS message
	public int last_msg_time;        ///< cpu time in sec at last received GPS message
	public int reset;                ///< hotstart, warmstart, coldstart
}
