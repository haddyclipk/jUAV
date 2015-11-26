package sw.airborne.firmwares.rotorcraft;

public class Autopilot_arming_throttle {
	public enum Arming_throttle_state {
		STATE_UNINIT,
		STATE_WAITING,
		STATE_MOTORS_OFF_READY,
		STATE_ARMING,
		STATE_MOTORS_ON,
		STATE_UNARMING
	}
	public static final int AUTOPILOT_ARMING_DELAY = 10;
	
	public static Arming_throttle_state autopilot_arming_state;
	public static int autopilot_arming_delay_counter;
	public static boolean autopilot_unarmed_in_auto;

	public static  void autopilot_arming_init() {
	  autopilot_arming_state = autopilot_arming_state.STATE_UNINIT;
	  autopilot_arming_delay_counter = 0;
	  autopilot_unarmed_in_auto = false;
	}

	public static  void autopilot_arming_set(boolean motors_on) {
	  if (motors_on) {
	    autopilot_arming_state = autopilot_arming_state.STATE_MOTORS_ON;
	  }
	  else {
	    if (autopilot_arming_state == autopilot_arming_state.STATE_MOTORS_ON) {
	      autopilot_arming_state = autopilot_arming_state.STATE_WAITING;
	    }
	  }
	}
	
//	public static  void autopilot_arming_check_motors_on(  ) {
//
//		  /* only allow switching motor if not in FAILSAFE or KILL mode */
//		  if (autopilot_mode != AP_MODE_KILL && autopilot_mode != AP_MODE_FAILSAFE) {
//
//		    switch(autopilot_arming_state) {
//		    case STATE_UNINIT:
//		      autopilot_motors_on = false;
//		      autopilot_arming_delay_counter = 0;
//		      if (THROTTLE_STICK_DOWN()) {
//		        autopilot_arming_state = autopilot_arming_state.STATE_MOTORS_OFF_READY;
//		      }
//		      else {
//		        autopilot_arming_state = autopilot_arming_state.STATE_WAITING;
//		      }
//		      break;
//		    case STATE_WAITING:
//		      autopilot_motors_on = false;
//		      autopilot_arming_delay_counter = 0;
//		      if (THROTTLE_STICK_DOWN()) {
//		        autopilot_arming_state = autopilot_arming_state.STATE_MOTORS_OFF_READY;
//		      }
//		      break;
//		    case STATE_MOTORS_OFF_READY:
//		      autopilot_motors_on = false;
//		      autopilot_arming_delay_counter = 0;
//		      if (!THROTTLE_STICK_DOWN() &&
//		          rc_attitude_sticks_centered() &&
//		          (autopilot_mode == MODE_MANUAL || autopilot_unarmed_in_auto)) {
//		        autopilot_arming_state = autopilot_arming_state.STATE_ARMING;
//		      }
//		      break;
//		    case STATE_ARMING:
//		      autopilot_motors_on = false;
//		      autopilot_arming_delay_counter++;
//		      if (THROTTLE_STICK_DOWN() ||
//		          !rc_attitude_sticks_centered() ||
//		          (autopilot_mode != MODE_MANUAL && !autopilot_unarmed_in_auto)) {
//		        autopilot_arming_state = autopilot_arming_state.STATE_MOTORS_OFF_READY;
//		      }
//		      else if (autopilot_arming_delay_counter >= AUTOPILOT_ARMING_DELAY) {
//		        autopilot_arming_state = autopilot_arming_state.STATE_MOTORS_ON;
//		      }
//		      break;
//		    case STATE_MOTORS_ON:
//		      autopilot_motors_on = true;
//		      autopilot_arming_delay_counter = AUTOPILOT_ARMING_DELAY;
//		      if (THROTTLE_STICK_DOWN()) {
//		        autopilot_arming_state = autopilot_arming_state.STATE_UNARMING;
//		      }
//		      break;
//		    case STATE_UNARMING:
//		      autopilot_motors_on = true;
//		      autopilot_arming_delay_counter--;
//		      if (!THROTTLE_STICK_DOWN()) {
//		        autopilot_arming_state = autopilot_arming_state.STATE_MOTORS_ON;
//		      }
//		      else if (autopilot_arming_delay_counter == 0) {
//		        autopilot_arming_state = autopilot_arming_state.STATE_MOTORS_OFF_READY;
//		        if (autopilot_mode != MODE_MANUAL) {
//		          autopilot_unarmed_in_auto = true;
//		        }
//		        else {
//		          autopilot_unarmed_in_auto = false;
//		        }
//		      }
//		      break;
//		    default:
//		      break;
//		    }
//		  }
//
//		}
	
}
