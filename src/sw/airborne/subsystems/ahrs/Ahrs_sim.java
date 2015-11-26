package sw.airborne.subsystems.ahrs;

import static sw.airborne.subsystems.Ahrs.*;

public class Ahrs_sim {
	
	public static boolean ahrs_sim_available;
	
	public static void ahrs_init(){
		 ahrs.status = AHRS_RUNNING;

		  ahrs_sim_available = false;
	}
	
	public static void ahrs_align(){
		/* Currently not really simulated
		   * body and imu have the same frame and always set to true value from sim
		   */

		  update_ahrs_from_sim();

		  ahrs.status = AHRS_RUNNING;
	}
	
	public static void ahrs_propagate(){
		  if (ahrs_sim_available) {
			    update_ahrs_from_sim();
			    ahrs_sim_available = false;
			  }
	}
	
	public static void update_ahrs_from_sim(){
//		  struct FloatEulers ltp_to_imu_euler = { sim_phi, sim_theta, sim_psi };
//		  #ifdef AHRS_UPDATE_FW_ESTIMATOR
//		    ltp_to_imu_euler.phi -= ins_roll_neutral;
//		    ltp_to_imu_euler.theta -= ins_pitch_neutral;
//		  #endif
//		    struct FloatRates imu_rate = { sim_p, sim_q, sim_r };
//		    /* set ltp_to_body to same as ltp_to_imu, currently no difference simulated */
//		    stateSetNedToBodyEulers_f(&ltp_to_imu_euler);
//		    stateSetBodyRates_f(&imu_rate);
	}

	
}
