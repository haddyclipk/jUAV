package sw.simulator.nps;

import sw.airborne.math.*;
import static sw.airborne.math.Pprz_algebra_int.*;
import static sw.airborne.math.Pprz_algebra_float.*;


public class Nps_sensor_gyro {
	
	
	public static void nps_sensor_gyro_init(NpsSensorGyro gyro, double time){
		  FLOAT_VECT3_ZERO(gyro.value);
//		  gyro.min = NPS_GYRO_MIN;
//		  gyro.max = NPS_GYRO_MAX;
//		  FLOAT_MAT33_DIAG(gyro.sensitivity,
//				   NPS_GYRO_SENSITIVITY_PP, NPS_GYRO_SENSITIVITY_QQ, NPS_GYRO_SENSITIVITY_RR);
//		  VECT3_ASSIGN(gyro.neutral,
//			       NPS_GYRO_NEUTRAL_P, NPS_GYRO_NEUTRAL_Q, NPS_GYRO_NEUTRAL_R);
//		  VECT3_ASSIGN(gyro.noise_std_dev,
//			       NPS_GYRO_NOISE_STD_DEV_P, NPS_GYRO_NOISE_STD_DEV_Q, NPS_GYRO_NOISE_STD_DEV_R);
//		  VECT3_ASSIGN(gyro.bias_initial,
//			       NPS_GYRO_BIAS_INITIAL_P, NPS_GYRO_BIAS_INITIAL_Q, NPS_GYRO_BIAS_INITIAL_R);
//		  VECT3_ASSIGN(gyro.bias_random_walk_std_dev,
//			       NPS_GYRO_BIAS_RANDOM_WALK_STD_DEV_P,
//			       NPS_GYRO_BIAS_RANDOM_WALK_STD_DEV_Q,
//			       NPS_GYRO_BIAS_RANDOM_WALK_STD_DEV_R);
//		  FLOAT_VECT3_ZERO(gyro.bias_random_walk_value);
		  gyro.next_update = time;
		  gyro.data_available = false;
	}
	
	public static void nps_sensor_gyro_run_step(NpsSensorGyro gyro, double time, DoubleRMat body_to_imu){
		
	}
}
