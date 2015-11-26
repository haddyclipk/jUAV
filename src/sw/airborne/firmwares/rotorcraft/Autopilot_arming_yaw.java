package sw.airborne.firmwares.rotorcraft;

import static sw.airborne.Paparazzi.*;
import static sw.airborne.firmwares.rotorcraft.Autopilot.*;

public class Autopilot_arming_yaw {
	public enum Arming_yaw_state {
		STATUS_MOTORS_OFF,
		  STATUS_M_OFF_STICK_PUSHED,
		  STATUS_START_MOTORS,
		  STATUS_MOTORS_ON,
		  STATUS_M_ON_STICK_PUSHED,
		  STATUS_STOP_MOTORS
	}
	public static final int MOTOR_ARMING_DELAY  = 40;

	public static int autopilot_motors_on_counter;

	public static Arming_yaw_state autopilot_check_motor_status;


	public static  void autopilot_arming_init() {
		autopilot_motors_on_counter = 0;
		autopilot_check_motor_status = autopilot_check_motor_status.STATUS_MOTORS_OFF;
	}


	/** Update the status of the check_motors state machine.
	 */
	public static  void autopilot_arming_set(boolean motors_on) {
		if (motors_on)
			autopilot_check_motor_status = autopilot_check_motor_status.STATUS_MOTORS_ON;
		else
			autopilot_check_motor_status =autopilot_check_motor_status.STATUS_MOTORS_OFF;
	}
//TODO commented to remove error messages;
//	public static  void autopilot_arming_check_motors_on(  ) {
//		/* only allow switching motor if not in FAILSAFE or KILL mode */
//		if (autopilot_mode != AP_MODE_KILL && autopilot_mode != AP_MODE_FAILSAFE) {
//
//			switch(autopilot_check_motor_status) {
//			case STATUS_MOTORS_OFF:
//				autopilot_motors_on = false;
//				autopilot_motors_on_counter = 0;
//				if (THROTTLE_STICK_DOWN() && YAW_STICK_PUSHED()) // stick pushed
//					autopilot_check_motor_status = autopilot_check_motor_status.STATUS_M_OFF_STICK_PUSHED;
//				break;
//			case STATUS_M_OFF_STICK_PUSHED:
//				autopilot_motors_on = false;
//				autopilot_motors_on_counter++;
//				if (autopilot_motors_on_counter >= MOTOR_ARMING_DELAY)
//					autopilot_check_motor_status = autopilot_check_motor_status.STATUS_START_MOTORS;
//				else if (!(THROTTLE_STICK_DOWN() && YAW_STICK_PUSHED())) // stick released too soon
//					autopilot_check_motor_status = autopilot_check_motor_status.STATUS_MOTORS_OFF;
//				break;
//			case STATUS_START_MOTORS:
//				autopilot_motors_on = true;
//				autopilot_motors_on_counter = MOTOR_ARMING_DELAY;
//				if (!(THROTTLE_STICK_DOWN() && YAW_STICK_PUSHED())) // wait until stick released
//					autopilot_check_motor_status = autopilot_check_motor_status.STATUS_MOTORS_ON;
//				break;
//			case STATUS_MOTORS_ON:
//				autopilot_motors_on = true;
//				autopilot_motors_on_counter = MOTOR_ARMING_DELAY;
//				if (THROTTLE_STICK_DOWN() && YAW_STICK_PUSHED()) // stick pushed
//					autopilot_check_motor_status = autopilot_check_motor_status.STATUS_M_ON_STICK_PUSHED;
//				break;
//			case STATUS_M_ON_STICK_PUSHED:
//				autopilot_motors_on = true;
//				autopilot_motors_on_counter--;
//				if (autopilot_motors_on_counter == 0)
//					autopilot_check_motor_status = autopilot_check_motor_status.STATUS_STOP_MOTORS;
//				else if (!(THROTTLE_STICK_DOWN() && YAW_STICK_PUSHED())) // stick released too soon
//					autopilot_check_motor_status = autopilot_check_motor_status.STATUS_MOTORS_ON;
//				break;
//			case STATUS_STOP_MOTORS:
//				autopilot_motors_on = false;
//				autopilot_motors_on_counter = 0;
//				if (!(THROTTLE_STICK_DOWN() && YAW_STICK_PUSHED())) // wait until stick released
//					autopilot_check_motor_status = autopilot_check_motor_status.STATUS_MOTORS_OFF;
//				break;
//			default:
//				break;
//			};
//		}
//	}
}
