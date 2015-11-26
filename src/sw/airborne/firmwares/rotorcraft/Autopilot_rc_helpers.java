package sw.airborne.firmwares.rotorcraft;

import static sw.airborne.Paparazzi.*;
import static sw.airborne.firmwares.rotorcraft.Autopilot.*;

public class Autopilot_rc_helpers {

	public static final double AUTOPILOT_THROTTLE_THRESHOLD    = (MAX_PPRZ / 20);
	public static final double AUTOPILOT_YAW_THRESHOLD          = (MAX_PPRZ * 19 / 20);
	public static final double AUTOPILOT_STICK_CENTER_THRESHOLD = (MAX_PPRZ * 1 / 20);

//	public static boolean THROTTLE_STICK_DOWN()   {
//
//		return (radio_control.values[RADIO_THROTTLE] < AUTOPILOT_THROTTLE_THRESHOLD);
//	}                                        
//
//	public static boolean YAW_STICK_PUSHED()   {
//
//		return	(radio_control.values[RADIO_YAW] > AUTOPILOT_YAW_THRESHOLD ||  
//				radio_control.values[RADIO_YAW] < -AUTOPILOT_YAW_THRESHOLD);
//	}                                   
//
//	public static boolean YAW_STICK_CENTERED()    {
//
//		return (radio_control.values[RADIO_YAW] < AUTOPILOT_STICK_CENTER_THRESHOLD && 
//				radio_control.values[RADIO_YAW] > -AUTOPILOT_STICK_CENTER_THRESHOLD);
//	}                                        
//	public static boolean PITCH_STICK_CENTERED(){
//
//		return (radio_control.values[RADIO_PITCH] < AUTOPILOT_STICK_CENTER_THRESHOLD && 
//				radio_control.values[RADIO_PITCH] > -AUTOPILOT_STICK_CENTER_THRESHOLD);
//	}                                          
//
//	public static boolean ROLL_STICK_CENTERED()  {
//
//		return (radio_control.values[RADIO_ROLL] < AUTOPILOT_STICK_CENTER_THRESHOLD && 
//				radio_control.values[RADIO_ROLL] > -AUTOPILOT_STICK_CENTER_THRESHOLD);
//	}                                         


//	public static boolean rc_attitude_sticks_centered() {
//		return ROLL_STICK_CENTERED() && PITCH_STICK_CENTERED() && YAW_STICK_CENTERED();
//	}


	public static boolean kill_switch_is_on() {
		
			return false;
		
	}


//	public static  int percent_from_rc(int channel)
//	{
//		int per = (MAX_PPRZ + (int)radio_control.values[channel]) * 50 / MAX_PPRZ;
//		if (per < 0)
//			per = 0;
//		else if (per > 100)
//			per = 100;
//		return per;
//	}

}
