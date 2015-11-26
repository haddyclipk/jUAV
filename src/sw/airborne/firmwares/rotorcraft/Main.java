package sw.airborne.firmwares.rotorcraft;

import static sw.airborne.State.*;
import static sw.airborne.subsystems.Imu.*;
import static sw.airborne.subsystems.ImuFloat.*;
//import static sw.airborne.subsystems.Gps.*;
import static sw.airborne.firmwares.rotorcraft.Autopilot.*;
//import static sw.airborne.subsystems.ahrs.arhs_gx3.*;
import static sw.airborne.subsystems.ins.Ins_int.*;
import static sw.airborne.subsystems.Ahrs.*;
import static sw.airborne.subsystems.ahrs.Ahrs_aligner.*;
import static sw.airborne.subsystems.ahrs.Ahrs_sim.*;
//import sw.airborne.subsystems.ahrs.Ahrs_aligner.*;
import static sw.airborne.subsystems.Setting.*;
import static sw.airborne.mcu_periph.Sys_time.*;
import static sw.airborne.subsystems.datalink.Datalink.*;
//import static sw.airborne.subsystems.Setting.*;
//import static sw.airborne.subsystems.Setting.*;
//import static sw.airborne.subsystems.Setting.*;
import static sw.airborne.subsystems.gps.Gps_sim_nps.*;
import static sw.airborne.subsystems.Gps.*;
import sw.airborne.math.*;
import sw.airborne.mcu_periph.Sys_time;
import sw.airborne.subsystems.*;
import static sw.simulator.nps.Nps_autopilot_rotorcraft.*;
//import static sw.airborne.subsystems.ins.ins_float_invariant.*;
//import static sw.airborne.subsystems.ins.ins_int.*;
//import static sw.airborne.modules.vehicle_interface.vi_overo_link.*; //TODO: do we need vehicle interface
//import static sw.airborne.modules.vehicle_interface.vi_overo_link.*;
import static sw.airborne.subsystems.Ins.*;
import static sw.airborne.subsystems.ahrs.Ahrs_aligner.*;

public class Main {

	public static final int BARO_PERIODIC_FREQUENCY = 50;
	public static final int PERIODIC_FREQUENCY = 512;
	public static final int MODULES_FREQUENCY = 512;
	public static final int TELEMETRY_FREQUENCY = 512;
	private static boolean USE_BARO_BOARD = false;
	private static boolean USE_IMU_FLOAT = false;
	//public static final int PERIODIC_FREQUENCY = 512;
	//public static final int PERIODIC_FREQUENCY = 512;
	public static int datalink_time=0;
	public static int main_periodic_tid; ///< id for main_periodic() timer
	public static int modules_tid;       ///< id for modules_periodic_task() timer
	public static int failsafe_tid;      ///< id for failsafe_check() timer
	public static int radio_control_tid; ///< id for radio_control_periodic_task() timer
	public static int electrical_tid;    ///< id for electrical_periodic() timer
	public static int telemetry_tid;     ///< id for telemetry_periodic() timer

	public static int baro_tid; 
	
	//public static ImuState imu = new ImuState();
	//public static Pprz_algebra pprz_algebra = new Pprz_algebra();
	//public static Ahrs_aligner ahrs_aligner = new Ahrs_aligner();
	
	public static void main() {
		main_init();

		while(true) {
			handle_periodic_tasks();
			main_event();
		}
		//return 0;
	}

	public static void main_init() {

		//mcu_init();

		//electrical_init();

		stateInit();

		//actuators_init();
		//if(USE_MOTOR_MIXING) motor_mixing_init();
		

		
		/*
		air_data_init();
		if(USE_BARO_BOARD) baro_init();
	*/	
		imu_init();
		if(USE_IMU_FLOAT) {imu_float_init();USE_IMU_FLOAT=true;}
	
		ahrs_aligner_init();
		ahrs_init();

		sw.airborne.subsystems.Ins.ins_init();

		if(USE_GPS)	gps_init();
		
		autopilot_init();

		//modules_init();

		settings_init();    // ???

		/*mcu_int_enable();
		if(DATALINK == XBEE)
				xbee_init();
		

		if(DATALINK == UDP)
		udp_init();*/     // ????
		
		// register the timers for the periodic functions
		main_periodic_tid = sys_time_register_timer((float)(1./PERIODIC_FREQUENCY),null);
		modules_tid = sys_time_register_timer((float)1./MODULES_FREQUENCY, null);
		radio_control_tid = sys_time_register_timer((float)(1./60.), null);
		failsafe_tid = sys_time_register_timer((float)0.05, null);
		electrical_tid = sys_time_register_timer((float)0.1, null);
		telemetry_tid = sys_time_register_timer((float)(1./TELEMETRY_FREQUENCY), null);
		if(USE_BARO_BOARD)
		{		baro_tid = sys_time_register_timer((float)(1./BARO_PERIODIC_FREQUENCY), null);
		USE_BARO_BOARD=true;}
		
	}

	public static void handle_periodic_tasks() {
		if (sys_time_check_and_ack_timer(main_periodic_tid))
			main_periodic();
	/*	if (sys_time_check_and_ack_timer(modules_tid))
			modules_periodic_task();
		if (sys_time_check_and_ack_timer(radio_control_tid))
			radio_control_periodic_task();
		if (sys_time_check_and_ack_timer(failsafe_tid))
			failsafe_check();
		if (sys_time_check_and_ack_timer(electrical_tid))
			electrical_periodic();
		if (sys_time_check_and_ack_timer(telemetry_tid))
			telemetry_periodic();
		if(USE_BARO_BOARD)
		if (sys_time_check_and_ack_timer(baro_tid))
			baro_periodic();*/
		
	}
	
	private static int TEN_MAIN_PERIODIC = 0;
	private static int PERIODIC_FREQUENCY_MAIN_PERIODIC=0;
	public static void main_periodic() {

		//imu_periodic();

		/* run control loops */
		Autopilot.autopilot_periodic();
		/* set actuators     */
		//actuators_set(autopilot_motors_on);
	//	SetActuatorsFromCommands(commands, autopilot_mode);

		if (autopilot_in_flight) {
			//RunOnceEvery(PERIODIC_FREQUENCY, { autopilot_flight_time++; datalink_time++; });
			PERIODIC_FREQUENCY_MAIN_PERIODIC++;					
			if (PERIODIC_FREQUENCY_MAIN_PERIODIC >= PERIODIC_FREQUENCY) {			
				PERIODIC_FREQUENCY_MAIN_PERIODIC = 0;					
				{ autopilot_flight_time++; 
				
				datalink_time++;           // ???????
				}						
			}	
		}

		//RunOnceEvery(10, LED_PERIODIC());
		TEN_MAIN_PERIODIC++;					
		if (TEN_MAIN_PERIODIC >= 10) {			
			TEN_MAIN_PERIODIC = 0;					
			LED_PERIODIC();						
		}	
	}

	//public static void telemetry_periodic() {
//		periodic_telemetry_send_Main();
//	}

	private static void LED_PERIODIC() {
		// TODO Auto-generated method stub
		
	}

	/* mode to enter when RC is lost while using a mode with RC input (not AP_MODE_NAV) */
	public static final int RC_LOST_MODE = AP_MODE_FAILSAFE;
	public static boolean USE_GPS = false;
	public static final boolean FAILSAFE_GROUND_DETECT = false;
	public static final boolean KILL_ON_GROUND_DETECT = false;
	public static final boolean USE_VEHICLE_INTERFACE_DEFINED = false;
	//private static final boolean SITL_DEFINED = false;
	public static final boolean SITL = false;
	public static final boolean USE_VEHICLE_INTERFACE = false;
	

/*	public static void failsafe_check() {
		if (radio_control.status == RC_REALLY_LOST &&
				autopilot_mode != AP_MODE_KILL &&
				autopilot_mode != AP_MODE_HOME &&
				autopilot_mode != AP_MODE_FAILSAFE &&
				autopilot_mode != AP_MODE_NAV)
		{
			autopilot_set_mode(RC_LOST_MODE);
		}

		if(FAILSAFE_ON_BAT_CRITICAL)
		if (autopilot_mode != AP_MODE_KILL &&
		electrical.bat_critical)
		{
			autopilot_set_mode(AP_MODE_FAILSAFE);
		}
		

		if(USE_GPS){
		gps_periodic_check();
		if (autopilot_mode == AP_MODE_NAV &&
				autopilot_motors_on &&
				
				radio_control.status != RC_OK &&
				
				GpsIsLost())
		{
			autopilot_set_mode(AP_MODE_FAILSAFE);
		}

		if (autopilot_mode == AP_MODE_HOME &&
				autopilot_motors_on && GpsIsLost())
		{
			autopilot_set_mode(AP_MODE_FAILSAFE);
		}
	}

		autopilot_check_in_flight(autopilot_motors_on);
	}*/

	public static void main_event() {

		//i2c_event();      //????????????????????????????

		DatalinkEvent();
/*
		if (autopilot_rc) {
			RadioControlEvent(autopilot_on_rc_frame);
		}

		ImuEvent(on_gyro_event, on_accel_event, on_mag_event);?????????????????

		if(USE_BARO_BOARD)
		BaroEvent();*/
		

		if(USE_GPS){
		//GpsEvent(on_gps_event); //TODO why?
		if (gps_available) {                            
		      gps.last_msg_ticks = Sys_time.nb_sec_rem;     
		      gps.last_msg_time = Sys_time.nb_sec;          
		      if (gps.fix == 3) {                  
		        gps.last_3dfix_ticks = Sys_time.nb_sec_rem; 
		        gps.last_3dfix_time = Sys_time.nb_sec;      
		      }                                             
		      on_gps_event();                    
		      gps_available = false;  
		      //USE_GPS=true;
		    }                       
		}
		if( FAILSAFE_GROUND_DETECT || KILL_ON_GROUND_DETECT)
		DetectGroundEvent();
		

		//modules_event_task();

	}

	public static void on_accel_event( ) {
		ImuScaleAccel(imu);

		if (ahrs.status != AHRS_UNINIT) {
			ahrs_update_accel();
		}
	}

	public static void on_gyro_event() {

		ImuScaleGyro();
		
		if(ahrs.status == AHRS_UNINIT) {
			ahrs_aligner_run();
			if(ahrs_aligner.status == AHRS_ALIGNER_LOCKED){
				sw.airborne.subsystems.ahrs.Ahrs_sim.ahrs_align();
			}
		}
		else{
			sw.airborne.subsystems.ahrs.Ahrs_sim.ahrs_propagate();
		    if(SITL){//????value
		    	if(nps_bypass_ahrs){
		    		sim_overwrite_ahrs();
		    	}
		    }
		    else{
		        ins_propagate();
		    }
		    
//		    if(USE_VEHICLE_INTERFACE){
//		    	  vi_notify_imu_available();
//		    }
		}
	}
	
	

	public static void on_gps_event() {
		ahrs_update_gps();
		ins_update_gps();
//		if(USE_VEHICLE_INTERFACE_DEFINED)
//		if (gps.fix == 3)
//			vi_notify_gps_available();
		
	}
/*
	public static void on_mag_event() {
		ImuScaleMag(imu);

		if(USE_MAGNETOMETER){
		if (ahrs.status == AHRS_RUNNING) {
			ahrs_update_mag();
		}
		}

		if(USE_VEHICLE_INTERFACE_DEFINED)
		vi_notify_mag_available();
		
	}*/

	
	

}
