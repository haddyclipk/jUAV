package sw.airborne.subsystems.ahrs;

import sw.airborne.math.*;
import static sw.airborne.math.Pprz_algebra_int.*;

public class Ahrs_aligner {
	
	//public static final int AHRS_UNINIT = 0;
	//public static final int AHRS_RUNNING = 1;
	//public static final int AHRS_LOCKED = 2;
	
	public static final int AHRS_ALIGNER_RUNNING = 1;
	public static final int AHRS_ALIGNER_UNINIT = 0;
	public static final int AHRS_ALIGNER_LOCKED = 2;

	public static AhrsAligner ahrs_aligner = new AhrsAligner();

	public static Int32Rates gyro_sum = new Int32Rates();
	public static Int32Vect3 accel_sum = new Int32Vect3();
	public static Int32Vect3 mag_sum = new Int32Vect3();
	public static int samples_idx;
	
	public static void ahrs_aligner_init(){
		 ahrs_aligner.status = AHRS_ALIGNER_RUNNING;
		  INT_RATES_ZERO(gyro_sum);
		  INT_VECT3_ZERO(accel_sum);
		  INT_VECT3_ZERO(mag_sum);
		  samples_idx = 0;
		  ahrs_aligner.noise = 0;
		  ahrs_aligner.low_noise_cnt = 0;
//		  #if PERIODIC_TELEMETRY
//		  register_periodic_telemetry(DefaultPeriodic, "FILTER_ALIGNER", send_aligner);
//		  #endif
		
	}
	//TODO
	public static void ahrs_aligner_run(){}
}
