package sw.airborne.firmwares.rotorcraft;

import static sw.airborne.Paparazzi.*;
import static sw.airborne.firmwares.rotorcraft.Autopilot.*;
import static sw.airborne.firmwares.rotorcraft.Autopilot_rc_helpers.*;

public class Autopilot_arming_switch {
	public enum arming_state {
		STATE_UNINIT,
		STATE_WAITING,
		STATE_STARTABLE,
		STATE_MOTORS_ON
	}
	
	public static arming_state autopilot_arming_state;
	
	public static boolean autopilot_unarmed_in_auto;

	public static  void autopilot_arming_init() {
	  autopilot_arming_state = arming_state.STATE_UNINIT;
	  autopilot_unarmed_in_auto = false;
	}

	public static  void autopilot_arming_set(boolean motors_on) {
	  if (motors_on) {
	    autopilot_arming_state = arming_state.STATE_MOTORS_ON;
	  }
	  else {
	    if (autopilot_arming_state == arming_state.STATE_MOTORS_ON) {
	      autopilot_arming_state = arming_state.STATE_STARTABLE;
	      /* if turned off in an AUTO mode, remember it so it can be turned on again in AUTO */
	      if (autopilot_mode != MODE_MANUAL) {
	        autopilot_unarmed_in_auto = true;
	      }
	      else {
	        autopilot_unarmed_in_auto = false;
	      }
	    }
	  }
	}

//	
//	public static  void autopilot_arming_check_motors_on() {
//	  switch(autopilot_arming_state) {
//	  case STATE_UNINIT:
//	    autopilot_motors_on = false;
//	    if (kill_switch_is_on()) {
//	      autopilot_arming_state = arming_state.STATE_STARTABLE;
//	    }
//	    else {
//	      autopilot_arming_state = arming_state.STATE_WAITING;
//	    }
//	    break;
//	  case STATE_WAITING:
//	    autopilot_motors_on = false;
//	    if (kill_switch_is_on()) {
//	      autopilot_arming_state = arming_state.STATE_STARTABLE;
//	    }
//	    break;
//	  case STATE_STARTABLE:
//	    autopilot_motors_on = false;
//	    if (!kill_switch_is_on() &&
//	        THROTTLE_STICK_DOWN() &&
//	        rc_attitude_sticks_centered() &&
//	        (autopilot_mode == MODE_MANUAL || autopilot_unarmed_in_auto)) {
//	      autopilot_arming_state = arming_state.STATE_MOTORS_ON;
//	    }
//	    break;
//	  case STATE_MOTORS_ON:
//	    autopilot_motors_on = true;
//	    if (kill_switch_is_on()) {
//	      /* if killed, go to STATE_STARTABLE where motors will be turned off */
//	      autopilot_arming_state = arming_state.STATE_STARTABLE;
//	      /* if turned off in an AUTO mode, remember it so it can be turned on again in AUTO */
//	      if (autopilot_mode != MODE_MANUAL) {
//	        autopilot_unarmed_in_auto = true;
//	      }
//	      else {
//	        autopilot_unarmed_in_auto = false;
//	      }
//	    }
//	    break;
//	  default:
//	    break;
//	  }
//
//	}

}
