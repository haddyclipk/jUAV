package sw.simulator.nps;

import sw.airborne.math.*;

public class NpsFdm {
	public double time;
	public double init_dt;
	public double curr_dt;
	public boolean on_ground;
	public int nan_count;

	/*  position */
	public EcefCoor_d ecef_pos;
	public  NedCoor_d ltpprz_pos;
	public  LlaCoor_d lla_pos;
	public double hmsl;
	// for debugging
	public  LlaCoor_d lla_pos_pprz; //lla converted by pprz from ecef
	public  LlaCoor_d lla_pos_geod; //geodetic lla from jsbsim
	public  LlaCoor_d lla_pos_geoc; //geocentric lla from jsbsim
	public double agl; //AGL from jsbsim in m

	/*  velocity and acceleration wrt inertial frame expressed in ecef frame */
	//   EcefCoor_d  ecef_inertial_vel;
	//   EcefCoor_d  ecef_inertial_accel;
	/*  velocity and acceleration wrt ecef frame expressed in ecef frame     */
	public  EcefCoor_d ecef_ecef_vel;
	public  EcefCoor_d ecef_ecef_accel;
	/*  velocity and acceleration wrt ecef frame expressed in body frame     */
	public  DoubleVect3 body_ecef_vel; /* aka UVW */
	public  DoubleVect3 body_ecef_accel;
	/*  velocity and acceleration wrt ecef frame expressed in ltp frame     */
	public  NedCoor_d ltp_ecef_vel;
	public  NedCoor_d ltp_ecef_accel;
	/*  velocity and acceleration wrt ecef frame expressed in ltppprz frame */
	public  NedCoor_d ltpprz_ecef_vel;
	public  NedCoor_d ltpprz_ecef_accel;

	/* attitude */
	public  DoubleQuat ecef_to_body_quat;
	public  DoubleQuat ltp_to_body_quat;
	public  DoubleEulers ltp_to_body_eulers;
	public  DoubleQuat ltpprz_to_body_quat;
	public  DoubleEulers ltpprz_to_body_eulers;

	/*  velocity and acceleration wrt ecef frame expressed in body frame     */
	public  DoubleRates body_ecef_rotvel;
	public  DoubleRates body_ecef_rotaccel;

	public  DoubleVect3 ltp_g;
	public  DoubleVect3 ltp_h;

	public  DoubleVect3 wind; ///< velocity in m/s in NED


}
