package sw.airborne.firmwares.rotorcraft;
import static sw.airborne.State.*;
import static sw.airborne.subsystems.Ahrs.*;
import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_euler_int.*;
import sw.airborne.math.*;
import sw.airborne.mcu_periph.Sys_time;
import sw.airborne.subsystems.AhrsState;
import sw.airborne.subsystems.Gps;
import sw.airborne.subsystems.actuators.motor_mixing.Motor_mixing;
import sw.airborne.subsystems.datalink.telemetry;
//import static sw.airborne.subsystems.datalink.telemetry;
//import sw.airborne.subsystems.datalink.telemetry;
import static sw.airborne.math.Pprz_algebra_int.*;
import static sw.airborne.firmwares.rotorcraft.guidance.Guidance_v.*;
import static sw.airborne.firmwares.rotorcraft.guidance.Guidance_h.*;
import static sw.airborne.firmwares.rotorcraft.Stabilization.*;
import static sw.airborne.firmwares.rotorcraft.Autopilot_arming_switch.*;
import static sw.airborne.firmwares.rotorcraft.Navigation.*;
import static sw.airborne.subsystems.datalink.telemetry.*;
import static sw.airborne.firmwares.rotorcraft.Main.*;


public class Autopilot {
	
	public static final int AP_MODE_KILL = 0;
	public static final int AP_MODE_FAILSAFE      =     1;
	public static final int AP_MODE_HOME          =     2;
	public static final int AP_MODE_RATE_DIRECT    =    3;
	public static final int AP_MODE_ATTITUDE_DIRECT  =  4;
	public static final int AP_MODE_RATE_RC_CLIMB    =  5;
	public static final int AP_MODE_ATTITUDE_RC_CLIMB = 6;
	public static final int AP_MODE_ATTITUDE_CLIMB   =  7;
	public static final int AP_MODE_RATE_Z_HOLD     =   8;
	public static final int AP_MODE_ATTITUDE_Z_HOLD  =  9;
	public static final int AP_MODE_HOVER_DIRECT     =  10;
	public static final int AP_MODE_HOVER_CLIMB     =   11;
	public static final int AP_MODE_HOVER_Z_HOLD     =  12;
	public static final int AP_MODE_NAV            =    13;
	public static final int AP_MODE_RC_DIRECT       =   14;	// Safety Pilot Direct Commands for helicopter direct control
	public static final int AP_MODE_CARE_FREE_DIRECT  = 15;
	public static final int AP_MODE_FORWARD         =   16;
	private static boolean MODE_MANUAL_DEFINED = false;
	private static boolean MODE_AUTO1_DEFINED = false;
	private static boolean MODE_AUTO2_DEFINED = false;
	private static boolean THRESHOLD_GROUND_DETECT_DEFINED = false;
	private static final int MIN_PPRZ = -9600;
	
	static int MODE_MANUAL;
	static int MODE_AUTO1;
	static int MODE_AUTO2;
	
	static int THRESHOLD_GROUND_DETECT;
	
	static{
		if(!MODE_MANUAL_DEFINED) {
			MODE_MANUAL = AP_MODE_ATTITUDE_DIRECT;
			MODE_MANUAL_DEFINED = true;
		}
		if(!MODE_AUTO1_DEFINED){
			MODE_AUTO1 = AP_MODE_HOVER_Z_HOLD;
			MODE_AUTO1_DEFINED = true;
		}
		if(!MODE_AUTO2_DEFINED){
			MODE_AUTO2 = AP_MODE_NAV;
			MODE_AUTO2_DEFINED = true;
		}
		if(!THRESHOLD_GROUND_DETECT_DEFINED){
			THRESHOLD_GROUND_DETECT = 25;
			THRESHOLD_GROUND_DETECT_DEFINED = true;
		}
	}
		
	public static final int THRESHOLD_1_PPRZ = MIN_PPRZ / 2;
	private static final int MAX_PPRZ =9600;
	public static final int THRESHOLD_2_PPRZ = MAX_PPRZ / 2;
	private static boolean ROTORCRAFT_COMMANDS_YAW_ALWAYS_ENABLED_DEFINED = false;
	private static final int COMMAND_YAW = 0;
	private static final int COMMAND_THRUST = 1;
	private static final int COMMAND_PITCH = 2;
	private static final int COMMAND_ROLL = 3;
	private static boolean AUTOPILOT_DISABLE_AHRS_KILL_DEFINED = false;
	//private static  boolean USE_GPS = false;
	private static boolean USE_MOTOR_MIXING = false;
	private static final String DefaultChannel = null;
	private static final String DefaultDevice = null;
	private static final String MD5SUM = null;
	private static final int GPS_FIX_NONE = 0;
	private static final int PERIODIC_FREQUENCY = 20;
	private static boolean UNLOCKED_HOME_MODE_DEFINED = false;
	private static  boolean SetAutoCommandsFromRC_DEFINED = false;
	private static  boolean SetCommandsFromRC_DEFINED = false;
	private static  boolean KILL_AS_FAILSAFE_DEFINED = false;
	private static  boolean FAILSAFE_GROUND_DETECT = false;
	private static boolean ACTUATORS_DEFINED = false;
	public static telemetry DefaultPeriodic = new telemetry();
	
	public static int autopilot_mode;
	public static int autopilot_mode_auto2;
	public static boolean autopilot_motors_on;
	public static boolean kill_throttle;
	public static boolean autopilot_rc;
	public static boolean autopilot_power_switch;
	
	public static boolean autopilot_ground_detected;
	public static boolean autopilot_detect_ground_once;
	public static int autopilot_flight_time;
	private static int[] commands;
	
	public static void AP_MODE_OF_PPRZ (int _rc, int _mode){
		 if      (_rc > THRESHOLD_2_PPRZ)     
	      _mode = autopilot_mode_auto2;      
	    else if (_rc > THRESHOLD_1_PPRZ)     
	      _mode = MODE_AUTO1;                
	    else                                 
	      _mode = MODE_MANUAL;               
	}
	
	public static void autopilot_KillThrotlle(boolean _kill){
		 if (_kill)                          
	      autopilot_set_motors_on(false);   
	    else                                
	      autopilot_set_motors_on(true);    
	}
	
	public static void autopilot_SetPowerSwitch(boolean _v){
		autopilot_power_switch = _v;
//		if(POWER_SWITCH_GPIO_DEFINED){
//			if (_v) { gpio_set(POWER_SWITCH_GPIO); }  
//			else { gpio_clear(POWER_SWITCH_GPIO); }   
//		}
	}
	
	public static void SetRotorcraftCommands(int _cmd[], boolean _in_flight, boolean _motor_on ){
		if(!ROTORCRAFT_COMMANDS_YAW_ALWAYS_ENABLED_DEFINED){
			 if (!(_in_flight)) { _cmd[COMMAND_YAW] = 0; }               
			  if (!(_motor_on)) { _cmd[COMMAND_THRUST] = 0; }             
			  commands[COMMAND_ROLL] = _cmd[COMMAND_ROLL];                
			  commands[COMMAND_PITCH] = _cmd[COMMAND_PITCH];              
			  commands[COMMAND_YAW] = _cmd[COMMAND_YAW];                  
			  commands[COMMAND_THRUST] = _cmd[COMMAND_THRUST];  
			  ROTORCRAFT_COMMANDS_YAW_ALWAYS_ENABLED_DEFINED=true;
		}else{
			 if (!(_motor_on)) { _cmd[COMMAND_THRUST] = 0; }             
			  commands[COMMAND_ROLL] = _cmd[COMMAND_ROLL];                
			  commands[COMMAND_PITCH] = _cmd[COMMAND_PITCH];              
			  commands[COMMAND_YAW] = _cmd[COMMAND_YAW];                  
			  commands[COMMAND_THRUST] = _cmd[COMMAND_THRUST];            
		}
	}
	
	public static void DetectGroundEvent(){
		if (autopilot_mode == AP_MODE_FAILSAFE || autopilot_detect_ground_once) {
		    NedCoor_f accel = stateGetAccelNed_f();
		    if (accel.z < -THRESHOLD_GROUND_DETECT ||
		        accel.z > THRESHOLD_GROUND_DETECT) {
		      autopilot_ground_detected = true;
		      autopilot_detect_ground_once = false;
		    }
		  }
	}
	
	// ---------------- Beginning of autopilot.c------------------------
	//public static Gps gps;
	public static Motor_mixing motor_mixing;
	public static boolean autopilot_in_flight;
	public static int autopilot_in_flight_counter;
	
	public static int AUTOPILOT_IN_FLIGHT_TIME = 20;
	public static boolean AUTOPILOT_IN_FLIGHT_TIME_DEFINED = true;
	
	public static double AUTOPILOT_IN_FLIGHT_MIN_SPEED = 0.2;
	public static boolean AUTOPILOT_IN_FLIGHT_MIN_SPEED_DEFINED = true;
	
	public static double AUTOPILOT_IN_FLIGHT_MIN_ACCEL = 2.0;
	public static boolean AUTOPILOT_IN_FLIGHT_MIN_ACCEL_DEFINED = true;
	
	public static int AUTOPILOT_IN_FLIGHT_MIN_THRUST = 500;
	public static boolean AUTOPILOT_IN_FLIGHT_MIN_THRUST_DEFINED = true;
	
	public static boolean ahrs_is_aligned(){
		if(!AUTOPILOT_DISABLE_AHRS_KILL_DEFINED)
			{AUTOPILOT_DISABLE_AHRS_KILL_DEFINED=true;
			return (AhrsState.status == AHRS_RUNNING);}
		else return true;
	}
	
	public static double FAILSAFE_DESCENT_SPEED = 1.5;
	public static boolean FAILSAFE_DESCENT_SPEED_DEFINED = true;
	
	public static int FAILSAFE_MODE_TOO_FAR_FROM_HOME = AP_MODE_FAILSAFE;
	public static boolean FAILSAFE_MODE_TOO_FAR_FROM_HOME_DEFINED = true; 
	
	/*
	 * 
	#if USE_KILL_SWITCH_FOR_MOTOR_ARMING
	#include "autopilot_arming_switch.h"
	PRINT_CONFIG_MSG("Using kill switch for motor arming")
	#elif USE_THROTTLE_FOR_MOTOR_ARMING
	#include "autopilot_arming_throttle.h"
	PRINT_CONFIG_MSG("Using throttle for motor arming")
	#else
	#include "autopilot_arming_yaw.h"
	PRINT_CONFIG_MSG("Using 2 sec yaw for motor arming")
	#endif
	 */
	public static int MODE_STARTUP;
//	public static Autopilot_arming autopilot_arming;
//	static{
//		if(USE_KILL_SWITCH_FOR_MOTOR_ARMING)
//			autopilot_arming = new Autopilot_arming_switch();
//		else if (USE_THROTTLE_FOR_MOTOR_ARMING)
//			autopilot_arming = new Autopilot_arming_throttle();
//		else 
//			autopilot_arming = new Autopilot_arming_yaw();
//		
//		if(!MODE_STARTUP_DEFINED)
//			MODE_STARTUP = AP_MODE_KILL;
//	}、、？？？？？？？？？？？？？？？？？？？？？？？`
	
	
	public static void send_alive(){
		//DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);
	}
	
	public static void send_status(){
		int imu_nb_err = 0;
		int fix;
		int _motor_nb_err;
		//Object _motor_nb_err;
		if(USE_MOTOR_MIXING)
			 _motor_nb_err = motor_mixing.nb_saturation + motor_mixing.nb_failure * 10;
		else 
			 _motor_nb_err = 0;
		USE_MOTOR_MIXING=true;
		
		
		if(USE_GPS)  fix = Gps.gps.fix;
		else fix = GPS_FIX_NONE;
		
		int time_sec = Sys_time.nb_sec;
//		DOWNLINK_SEND_ROTORCRAFT_STATUS(DefaultChannel, DefaultDevice,
//		      imu_nb_err, _motor_nb_err,
//		    //  radio_control.status, radio_control.frame_rate,
//		      fix, autopilot_mode,
//		      autopilot_in_flight, autopilot_motors_on,
//		      guidance_h_mode, guidance_v_mode,
//		     // electrical.vsupply,
//		      time_sec);
	}
	
//	public static void send_energy(){
//		final int e = electrical.energy;
//		final float vsup = ((float)electrical.vsupply) / 10.0f;
//		float power = vsup * curs;
//		DOWNLINK_SEND_ENERGY(DefaultChannel, DefaultDevice, vsup, curs, e, power);
//	}
	
	public static void send_fp() {
		  long carrot_up = -guidance_v_z_sp;
//		  DOWNLINK_SEND_ROTORCRAFT_FP(DefaultChannel, DefaultDevice,
//		      (stateGetPositionEnu_i().x),
//		      (stateGetPositionEnu_i().y),
//		      (stateGetPositionEnu_i().z),
//		      (stateGetSpeedEnu_i().x),
//		      (stateGetSpeedEnu_i().y),
//		      (stateGetSpeedEnu_i().z),
//		      (stateGetNedToBodyEulers_i().phi),
//		      (stateGetNedToBodyEulers_i().theta),
//		      (stateGetNedToBodyEulers_i().psi),
//		      guidance_h_pos_sp.y,
//		      guidance_h_pos_sp.x,
//		      carrot_up,
//		      guidance_h_heading_sp,
//		      stabilization_cmd[COMMAND_THRUST],
//		      autopilot_flight_time);
		}
	/*
	public static void send_rc(){
		DOWNLINK_SEND_RC(DefaultChannel, DefaultDevice, RADIO_CONTROL_NB_CHANNEL, radio_control.values);
	}
	
	public static void send_rotorcraft_rc(){
		if(RADIO_KILL_SWITCH_DEFINED) 
			int _kill_switch = radio_control.values[RADIO_KILL_SWITCH];
		else 
			int _kill_switch = 42;
		
		DOWNLINK_SEND_ROTORCRAFT_RADIO_CONTROL(DefaultChannel, DefaultDevice,
			      radio_control.values[RADIO_ROLL],
			      radio_control.values[RADIO_PITCH],
			      radio_control.values[RADIO_YAW],
			      radio_control.values[RADIO_THROTTLE],
			      radio_control.values[RADIO_MODE],
			      _kill_switch,
			      radio_control.status);
	}
	*/
	public static void send_actuators() {
		 // DOWNLINK_SEND_ACTUATORS(DefaultChannel, DefaultDevice , ACTUATORS_NB, actuators);
	}
	
	public static void send_dl_value() {
		//  PeriodicSendDlValue(DefaultChannel, DefaultDevice);
	}
	
	public static void send_rotorcraft_cmd() {
//		  DOWNLINK_SEND_ROTORCRAFT_CMD(DefaultChannel, DefaultDevice,
//		      stabilization_cmd[COMMAND_ROLL],
//		      stabilization_cmd[COMMAND_PITCH],
//		      stabilization_cmd[COMMAND_YAW],
//		      stabilization_cmd[COMMAND_THRUST]);
	}

	
	public static void autopilot_init(){
		  /* mode is finally set at end of init if MODE_STARTUP is not KILL */
		  autopilot_mode = AP_MODE_KILL;
		  autopilot_motors_on = false;
		  kill_throttle = ! autopilot_motors_on;
		  autopilot_in_flight = false;
		  autopilot_in_flight_counter = 0;
		  autopilot_mode_auto2 = MODE_AUTO2;
		  autopilot_ground_detected = false;
		  autopilot_detect_ground_once = false;
		  autopilot_flight_time = 0;
		  autopilot_rc = true;
		  autopilot_power_switch = false;
		  
		//  if(!POWER_SWITCH_GPIO_DEFINED){
			//  gpio_setup_output(POWER_SWITCH_GPIO);
			 // gpio_clear(POWER_SWITCH_GPIO); // POWER OFF
		  //}
		  

		  autopilot_arming_init();//???????????????????????????????????????

		  nav_init();
		  guidance_h_init();
		  guidance_v_init();
		  stabilization_init();

		  /* set startup mode, propagates through to guidance h/v */
		  autopilot_set_mode(MODE_STARTUP);
		 // telemetry DefaultPeriodic=new telemetry();
		  register_periodic_telemetry(DefaultPeriodic, "ALIVE", 1);
		  register_periodic_telemetry(DefaultPeriodic, "ROTORCRAFT_STATUS", 2);
		  register_periodic_telemetry(DefaultPeriodic, "ENERGY", 3);
		  register_periodic_telemetry(DefaultPeriodic, "ROTORCRAFT_FP", 4);
		  register_periodic_telemetry(DefaultPeriodic, "ROTORCRAFT_CMD", 5);
		  register_periodic_telemetry(DefaultPeriodic, "DL_VALUE", 6);
		  
		  if(ACTUATORS_DEFINED) register_periodic_telemetry(DefaultPeriodic, "ACTUATORS", 7);
		  //if(RADIO_CONTROL_DEFINED){
			//  register_periodic_telemetry(DefaultPeriodic, "RC", send_rc);
			 // register_periodic_telemetry(DefaultPeriodic, "ROTORCRAFT_RADIO_CONTROL", send_rotorcraft_rc);
		  }
	
	
	public  static double  NAV_PRESCALER = (PERIODIC_FREQUENCY / NAV_FREQ);
	
	
	
	
	//SCALERS USED IN RUNONCEEVERY
	private static int NAV_PRESCALER_COMPUTE_DIST2_TO_HOME = 0;
	private static int NAV_PRESCALER_NAV_HOME = 0;
	private static int NAV_PRESCALER_NAV_PERIODIC_TASK = 0;
	
	
	public static void autopilot_periodic(){
		//RunOnceEvery(NAV_PRESCALER, C);
		NAV_PRESCALER_COMPUTE_DIST2_TO_HOME++;					
		if (NAV_PRESCALER_COMPUTE_DIST2_TO_HOME >= NAV_PRESCALER) {			
			NAV_PRESCALER_COMPUTE_DIST2_TO_HOME = 0;					
			compute_dist2_to_home();						
		}	

		if (autopilot_in_flight) {
			if (too_far_from_home) {
				if (dist2_to_home > failsafe_mode_dist2)
					autopilot_set_mode(FAILSAFE_MODE_TOO_FAR_FROM_HOME);
				else
					autopilot_set_mode(AP_MODE_HOME);
			}
		}

		if (autopilot_mode == AP_MODE_HOME) {
			//RunOnceEvery(NAV_PRESCALER, nav_home());
			NAV_PRESCALER_NAV_HOME++;					
			if (NAV_PRESCALER_NAV_HOME >= NAV_PRESCALER) {			
				NAV_PRESCALER_NAV_HOME = 0;					
				nav_home();						
			}	

		}
		else {
			// otherwise always call nav_periodic_task so that carrot is always updated in GCS for other modes
			//RunOnceEvery(NAV_PRESCALER, nav_periodic_task());
			NAV_PRESCALER_NAV_PERIODIC_TASK++;					
			if (NAV_PRESCALER_NAV_PERIODIC_TASK >= NAV_PRESCALER) {			
				NAV_PRESCALER_NAV_PERIODIC_TASK = 0;					
				nav_periodic_task();						
			}

		}


		/* If in FAILSAFE mode and either already not in_flight anymore
		 * or just "detected" ground, go to KILL mode.
		 */
		if (autopilot_mode == AP_MODE_FAILSAFE) {
			if (!autopilot_in_flight)
			autopilot_set_mode(AP_MODE_KILL);

			if(FAILSAFE_GROUND_DETECT){
				//INFO("Using FAILSAFE_GROUND_DETECT: KILL")
				if (autopilot_ground_detected)
				autopilot_set_mode(AP_MODE_KILL);
			}
		}

		/* Reset ground detection _after_ running flight plan
		 */
		if (!autopilot_in_flight || autopilot_ground_detected) {
			autopilot_ground_detected = false;
			autopilot_detect_ground_once = false;
		}

		/* Set fixed "failsafe" commands from airframe file if in KILL mode.
		 * If in FAILSAFE mode, run normal loops with failsafe attitude and
		 * downwards velocity setpoints.
		 */
		if (autopilot_mode == AP_MODE_KILL) {
			//SetCommands(commands_failsafe);
		}
		else {
			guidance_v_run( autopilot_in_flight );
			guidance_h_run( autopilot_in_flight );
			//SetRotorcraftCommands(stabilization_cmd, autopilot_in_flight, autopilot_motors_on);
		}

	}
	
	public static void autopilot_set_mode(int new_autopilot_mode) {

		/* force kill mode as long as AHRS is not aligned */
		if (!ahrs_is_aligned())
			new_autopilot_mode = AP_MODE_KILL;

		if (new_autopilot_mode != autopilot_mode) {
			/* horizontal mode */
			switch (new_autopilot_mode) {
			case AP_MODE_FAILSAFE:
				if(!KILL_AS_FAILSAFE_DEFINED){
					stabilization_attitude_set_failsafe_setpoint();
					guidance_h_mode_changed(GUIDANCE_H_MODE_ATTITUDE);
					break;
				}
			case AP_MODE_KILL:
				autopilot_in_flight =false;
				autopilot_in_flight_counter = 0;
				guidance_h_mode_changed(GUIDANCE_H_MODE_KILL);
				break;
			case AP_MODE_RC_DIRECT:
				guidance_h_mode_changed(GUIDANCE_H_MODE_RC_DIRECT);
				break;
			case AP_MODE_RATE_DIRECT:
			case AP_MODE_RATE_Z_HOLD:
				guidance_h_mode_changed(GUIDANCE_H_MODE_RATE);
				break;
			case AP_MODE_ATTITUDE_RC_CLIMB:
			case AP_MODE_ATTITUDE_DIRECT:
			case AP_MODE_ATTITUDE_CLIMB:
			case AP_MODE_ATTITUDE_Z_HOLD:
				guidance_h_mode_changed(GUIDANCE_H_MODE_ATTITUDE);
				break;
			case AP_MODE_FORWARD:
				guidance_h_mode_changed(GUIDANCE_H_MODE_FORWARD);
				break;
			case AP_MODE_CARE_FREE_DIRECT:
				guidance_h_mode_changed(GUIDANCE_H_MODE_CARE_FREE);
				break;
			case AP_MODE_HOVER_DIRECT:
			case AP_MODE_HOVER_CLIMB:
			case AP_MODE_HOVER_Z_HOLD:
				guidance_h_mode_changed(GUIDANCE_H_MODE_HOVER);
				break;
			case AP_MODE_HOME:
			case AP_MODE_NAV:
				guidance_h_mode_changed(GUIDANCE_H_MODE_NAV);
				break;
			default:
				break;
			}
			/* vertical mode */
			switch (new_autopilot_mode) {
			case AP_MODE_FAILSAFE:
				if(KILL_AS_FAILSAFE_DEFINED){
					guidance_v_mode_changed(GUIDANCE_V_MODE_CLIMB);
					guidance_v_zd_sp = SPEED_BFP_OF_REAL((float)FAILSAFE_DESCENT_SPEED);
					break;
				}
			case AP_MODE_KILL:
				autopilot_set_motors_on(false);
				stabilization_cmd[COMMAND_THRUST] = 0;
				guidance_v_mode_changed(GUIDANCE_V_MODE_KILL);
				break;
			case AP_MODE_RC_DIRECT:
			case AP_MODE_RATE_DIRECT:
			case AP_MODE_ATTITUDE_DIRECT:
			case AP_MODE_HOVER_DIRECT:
			case AP_MODE_CARE_FREE_DIRECT:
			case AP_MODE_FORWARD:
				guidance_v_mode_changed(GUIDANCE_V_MODE_RC_DIRECT);
				break;
			case AP_MODE_RATE_RC_CLIMB:
			case AP_MODE_ATTITUDE_RC_CLIMB:
				guidance_v_mode_changed(GUIDANCE_V_MODE_RC_CLIMB);
				break;
			case AP_MODE_ATTITUDE_CLIMB:
			case AP_MODE_HOVER_CLIMB:
				guidance_v_mode_changed(GUIDANCE_V_MODE_CLIMB);
				break;
			case AP_MODE_RATE_Z_HOLD:
			case AP_MODE_ATTITUDE_Z_HOLD:
			case AP_MODE_HOVER_Z_HOLD:
				guidance_v_mode_changed(GUIDANCE_V_MODE_HOVER);
				break;
			case AP_MODE_HOME:
			case AP_MODE_NAV:
				guidance_v_mode_changed(GUIDANCE_V_MODE_NAV);
				break;
			default:
				break;
			}
			autopilot_mode = new_autopilot_mode;
		}

	}
	
	public static void autopilot_check_in_flight(boolean motors_on) {
		if (autopilot_in_flight) {
			if (autopilot_in_flight_counter > 0) {
				/* probably in_flight if thrust, speed and accel above IN_FLIGHT_MIN thresholds */
				if ((stabilization_cmd[COMMAND_THRUST] <= AUTOPILOT_IN_FLIGHT_MIN_THRUST) &&
						(Math.abs(stateGetSpeedNed_f().z) < AUTOPILOT_IN_FLIGHT_MIN_SPEED) &&
						(Math.abs(stateGetAccelNed_f().z) < AUTOPILOT_IN_FLIGHT_MIN_ACCEL))
				{
					autopilot_in_flight_counter--;
					if (autopilot_in_flight_counter == 0) {
						autopilot_in_flight = false;
					}
				}
				else {  /* thrust, speed or accel not above min threshold, reset counter */
					autopilot_in_flight_counter = AUTOPILOT_IN_FLIGHT_TIME;
				}
			}
		}
		else { /* currently not in flight */
			if (autopilot_in_flight_counter < AUTOPILOT_IN_FLIGHT_TIME &&
					motors_on)
			{
				/* if thrust above min threshold, assume in_flight.
				 * Don't check for velocity and acceleration above threshold here...
				 */
				if (stabilization_cmd[COMMAND_THRUST] > AUTOPILOT_IN_FLIGHT_MIN_THRUST) {
					autopilot_in_flight_counter++;
					if (autopilot_in_flight_counter == AUTOPILOT_IN_FLIGHT_TIME)
						autopilot_in_flight = true;
				}
				else { /* currently not in_flight and thrust below threshold, reset counter */
					autopilot_in_flight_counter = 0;
				}
			}
		}
	}

	public static void autopilot_set_motors_on(boolean motors_on){
		if (ahrs_is_aligned() && motors_on)
			autopilot_motors_on = true;
		else
			autopilot_motors_on = false;
		kill_throttle = ! autopilot_motors_on;
		autopilot_arming_set(autopilot_motors_on);
	}

//	public static void autopilot_on_rc_frame(){
//		//if(kill_switch_is_on()){
//		if (false) {
//			autopilot_set_mode(AP_MODE_KILL);
//		}
//		else if ((autopilot_mode != AP_MODE_HOME) || (UNLOCKED_HOME_MODE_DEFINED && !too_far_from_home))
//		{
//			int new_autopilot_mode = 0;
//			//AP_MODE_OF_PPRZ(radio_control.values[RADIO_MODE], new_autopilot_mode);
//
//			
//			/* don't enter NAV mode if GPS is lost (this also prevents mode oscillations) */
//			if (USE_GPS && !(new_autopilot_mode == AP_MODE_NAV && GpsIsLost()))
//				autopilot_set_mode(new_autopilot_mode);
//			else
//				autopilot_set_mode(new_autopilot_mode);
//		}
//
//		/* if not in FAILSAFE or HOME mode check motor and in_flight status, read RC */
//		if (autopilot_mode != AP_MODE_FAILSAFE && autopilot_mode != AP_MODE_HOME) {
//
//			/* if there are some commands that should always be set from RC, do it */
//			if(SetAutoCommandsFromRC_DEFINED){
//				//SetAutoCommandsFromRC(commands, radio_control.values);
//			}
//
//			/* if not in NAV_MODE set commands from the rc */
//			if(SetCommandsFromRC_DEFINED){
//				if (autopilot_mode != AP_MODE_NAV) {
//					//SetCommandsFromRC(commands, radio_control.values);
//				}
//			}
//
//			/* an arming sequence is used to start/stop motors */
//			autopilot_arming_check_motors_on();
//			kill_throttle = ! autopilot_motors_on;
//
//			guidance_v_read_rc();
//			guidance_h_read_rc(autopilot_in_flight);
//		}
//
//	}
}