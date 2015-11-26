package sw.simulator.nps;

import sw.airborne.math.FloatQuat;
import sw.airborne.math.FloatRates;
import sw.airborne.math.NedCoor_f;
import static sw.airborne.math.Pprz_algebra.*;
import static sw.airborne.State.*;

import static sw.airborne.firmwares.rotorcraft.Main.*;
import devices.Gps;
import sw.airborne.subsystems.Imu;
import sw.airborne.subsystems.actuators.motor_mixing.Motor_mixing;
import static sw.simulator.nps.nps_fdm_jsbsim.*;
import static sw.airborne.Paparazzi.*;
import static sw.simulator.nps.Nps_autopilot.*;
import static sw.airborne.firmwares.rotorcraft.Main.*;

public class Nps_autopilot_rotorcraft 
{
	
	public static boolean nps_bypass_ahrs;
	public static boolean nps_bypass_ins;
	
	void nps_autopilot_run_step(double time) 
	{
		if (Imu.gyro_available) {
			//we were using main_event() which in turn called gyro and accel event removed the extra step and called gyro and accel event directly
			//ImuEvent(on_gyro_event, on_accel_event, on_mag_event);
			on_gyro_event();
			on_accel_event();
		  }
		  if (Gps.gps_available) {
			  //we were using main_event() which in turn called gps event removed the extra step and called gps event directly
			  //GpsEvent(on_gps_event);
			  on_gps_event();
		  }

		 handle_periodic_tasks();

		  //We'll be using this compare the output between c and java autopilot
		 //int MAX_PPRZ = 9600;
		  for (int  i=0; i < NPS_COMMANDS_NB; i++)
			  autopilot.commands[i] = (double)Motor_mixing.commands[i]/MAX_PPRZ;

		}

	
	public static void sim_overwrite_ahrs() {

		  FloatQuat quat_f = new FloatQuat();
		  QUAT_COPY(quat_f, fdm.ltp_to_body_quat);
		  stateSetNedToBodyQuat_f(quat_f);

		  FloatRates rates_f = new FloatRates();
		  RATES_COPY(rates_f, fdm.body_ecef_rotvel);
		  stateSetBodyRates_f(rates_f);

		}
	
	public static void sim_overwrite_ins() {

		  NedCoor_f ltp_pos = new NedCoor_f();
		  VECT3_COPY(ltp_pos, fdm.ltpprz_pos);
		  stateSetPositionNed_f(ltp_pos);

		  NedCoor_f ltp_speed = new NedCoor_f();
		  VECT3_COPY(ltp_speed, fdm.ltpprz_ecef_vel);
		  stateSetSpeedNed_f(ltp_speed);

		  NedCoor_f ltp_accel = new NedCoor_f();
		  VECT3_COPY(ltp_accel, fdm.ltpprz_ecef_accel);
		  stateSetAccelNed_f(ltp_accel);

	}

}
