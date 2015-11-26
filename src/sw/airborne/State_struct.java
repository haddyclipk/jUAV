package sw.airborne;

import sw.airborne.math.*;

public class State_struct {
	 /** @addtogroup state_position
	   *  @{ */

	  /**
	   * Holds the status bits for all position representations.
	   * When the corresponding bit is set the representation
	   * is already computed.
	   */
	  public int pos_status;

	  /**
	   * Position in EarthCenteredEarthFixed coordinates.
	   * Units: centimeters
	   */
	  public EcefCoor_i ecef_pos_i;

	  /**
	   * Position in Latitude, Longitude and Altitude.
	   * Units lat,lon: radians*1e7
	   * Units alt: milimeters above reference ellipsoid
	   */
	  public  LlaCoor_i lla_pos_i;

	  /**
	   * Definition of the local (flat earth) coordinate system.
	   * Defines the origin of the local NorthEastDown coordinate system
	   * in ECEF (EarthCenteredEarthFixed) and LLA (LatitudeLongitudeAlt)
	   * coordinates and the roation matrix from ECEF to local frame.
	   * (int version)
	   */
	  public  LtpDef_i ned_origin_i;

	  /**
	   * true if local int coordinate frame is initialsed
	   */
	  public  boolean ned_initialized_i;

	  /**
	   * Position in North East Down coordinates.
	   * with respect to ned_origin_i (flat earth)
	   * Units: m in BFP with INT32_POS_FRAC
	   */
	  public  NedCoor_i ned_pos_i;

	  /**
	   * Position in East North Up coordinates.
	   * with respect to ned_origin_i (flat earth)
	   * Units: m in BFP with INT32_POS_FRAC
	   */
	  public   EnuCoor_i enu_pos_i;

	  /**
	   * Position in UTM coordinates.
	   * Units x,y: meters.
	   * Units z: meters above MSL
	   */
	  public  UtmCoor_f utm_pos_f;

	  /**
	   * Altitude above ground level.
	   * Unit: meters
	   */
	  public  float alt_agl_f;

	  /**
	   * Position in Latitude, Longitude and Altitude.
	   * Units lat,lon: radians
	   * Units alt: meters above reference ellipsoid
	   */
	  public  LlaCoor_f lla_pos_f;

	  /**
	   * Position in EarthCenteredEarthFixed coordinates.
	   * Units: meters
	   */
	  public EcefCoor_f ecef_pos_f;

	  /**
	   * Definition of the local (flat earth) coordinate system.
	   * Defines the origin of the local NorthEastDown coordinate system
	   * in ECEF (EarthCenteredEarthFixed) and LLA (LatitudeLongitudeAlt)
	   * coordinates and the roation matrix from ECEF to local frame.
	   * (float version)
	   */
	  public  LtpDef_f ned_origin_f;

	  /// True if local float coordinate frame is initialsed
	  public  boolean ned_initialized_f;

	  /**
	   * Definition of the origin of Utm coordinate system.
	   * Defines the origin of the local NorthEastDown coordinate system
	   * in UTM coordinates, used as a reference when ned_origin is not
	   * initialized.
	   * (float version)
	   */
	  public   UtmCoor_f utm_origin_f;

	  /// True if utm origin (float) coordinate frame is initialsed
	  public  boolean utm_initialized_f;

	  /**
	   * Position in North East Down coordinates.
	   * with respect to ned_origin_i (flat earth)
	   * Units: meters
	   */
	  public NedCoor_f ned_pos_f;

	  /**
	   * Position in East North Up coordinates.
	   * with respect to ned_origin_i (flat earth)
	   * Units: meters
	   */
	  public   EnuCoor_f enu_pos_f;
	  /** @}*/


	  /** @addtogroup state_velocity
	   *  @{ */
	  /**
	   * Holds the status bits for all ground speed representations.
	   * When the corresponding bit is one the representation
	   * is already computed.
	   */
	  public  int speed_status;

	  /**
	   * Velocity in EarthCenteredEarthFixed coordinates.
	   * Units: m/s in BFP with #INT32_SPEED_FRAC
	   */
	  public  EcefCoor_i ecef_speed_i;

	  /**
	   * Velocity in North East Down coordinates.
	   * Units: m/s in BFP with #INT32_SPEED_FRAC
	   */
	  public  NedCoor_i ned_speed_i;

	  /**
	   * Velocity in East North Up coordinates.
	   * Units: m/s in BFP with #INT32_SPEED_FRAC
	   */
	  public   EnuCoor_i enu_speed_i;

	  /**
	   * Norm of horizontal ground speed.
	   * Unit: m/s in BFP with #INT32_SPEED_FRAC
	   */
	  public   long h_speed_norm_i;

	  /**
	   * Direction of horizontal ground speed.
	   * Unit: rad in BFP with #INT32_ANGLE_FRAC
	   * (clockwise, zero=north)
	   */
	  public  long h_speed_dir_i;

	  /**
	   * Velocity in EarthCenteredEarthFixed coordinates.
	   * Units: m/s
	   */
	  public EcefCoor_f ecef_speed_f;

	  /**
	   * @brief speed in North East Down coordinates
	   * @details Units: m/s */
	  public  NedCoor_f ned_speed_f;

	  /**
	   * Velocity in East North Up coordinates.
	   * Units: m/s
	   */
	  public  EnuCoor_f enu_speed_f;

	  /**
	   * Norm of horizontal ground speed.
	   * Unit: m/s
	   */
	  public  float h_speed_norm_f;

	  /**
	   * Direction of horizontal ground speed.
	   * Unit: rad (clockwise, zero=north)
	   */
	  public   float h_speed_dir_f;
	  /** @}*/


	  /** @addtogroup state_acceleration
	   *  @{ */
	  /**
	   * Holds the status bits for all acceleration representations.
	   * When the corresponding bit is one the representation
	   * is already computed.
	   */
	  public  int accel_status;

	  /**
	   * Acceleration in North East Down coordinates.
	   * Units: m/s^2 in BFP with #INT32_ACCEL_FRAC
	   */
	  public   NedCoor_i ned_accel_i;

	  /**
	   * Acceleration in EarthCenteredEarthFixed coordinates.
	   * Units: m/s^2 in BFP with INT32_ACCEL_FRAC
	   */
	  public  EcefCoor_i ecef_accel_i;

	  /**
	   * Acceleration in North East Down coordinates.
	   * Units: m/s^2
	   */
	  public  NedCoor_f ned_accel_f;

	  /**
	   * Acceleration in EarthCenteredEarthFixed coordinates.
	   * Units: m/s^2
	   */
	  public  EcefCoor_f ecef_accel_f;
	  /** @}*/


	  /** @defgroup state_attitude Attitude representations
	   */
	  public OrientationReps ned_to_body_orientation;


	  /** @addtogroup state_rate
	   *  @{ */
	  /**
	   * Holds the status bits for all angular rate representations.
	   * When the corresponding bit is one the representation
	   * is already computed.
	   */
	  public  int rate_status;

	  /**
	   * Angular rates in body frame.
	   * Units: rad/s in BFP with #INT32_RATE_FRAC
	   */
	  public  Int32Rates body_rates_i;

	  /**
	   * Angular rates in body frame.
	   * Units: rad/s
	   */
	  public FloatRates  body_rates_f;
	  /** @}*/


	  /** @addtogroup state_wind_airspeed
	   *  @{ */
	  /**
	   * Holds the status bits for all wind- and airspeed representations.
	   * When the corresponding bit is one the representation
	   * is already computed.
	   */
	  public int wind_air_status;

	  /**
	   * Horizontal windspeed in north/east.
	   * Units: m/s in BFP with #INT32_SPEED_FRAC
	   */
	  public Int32Vect2 h_windspeed_i;

	  /**
	   * Norm of horizontal ground speed.
	   * @details Unit: m/s in BFP with #INT32_SPEED_FRAC
	   */
	  public   int airspeed_i;

	  /**
	   * Horizontal windspeed.
	   * Units: m/s with x=north, y=east
	   */
	  public   FloatVect2 h_windspeed_f;

	  /**
	   * Norm of relative air speed.
	   * Unit: m/s
	   */
	  public  float airspeed_f;

	  /**
	   * Angle of attack
	   * Unit: rad
	   */
	  public float angle_of_attack_f;

	  /**
	   * Sideslip angle
	   * Unit: rad
	   */
	  public float sideslip_f;

	  /** @}*/
}
