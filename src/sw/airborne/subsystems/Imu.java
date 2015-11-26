package sw.airborne.subsystems;

import static sw.airborne.math.Pprz_algebra.*;
import static sw.airborne.math.Pprz_algebra_int.*;
//import static sw.airborne.subsystems.imu.Imu_nps.*;
public class Imu {
	
	public static ImuState imu = new ImuState();
	public static ImuFloat imuf = new ImuFloat();
	
	public static final long IMU_BODY_TOimu_PHI = 0;
	public static final long IMU_BODY_TOimu_THETA = 0;
	public static final long IMU_BODY_TOimu_PSI = 0;
	
	public static final long IMU_GYRO_P_NEUTRAL = 0;
	public static final long IMU_GYRO_Q_NEUTRAL = 0;
	public static final long IMU_GYRO_R_NEUTRAL = 0;

	public static final long IMU_ACCEL_X_NEUTRAL = 0;
	public static final long IMU_ACCEL_Y_NEUTRAL = 0;
	public static final long IMU_ACCEL_Z_NEUTRAL = 0;
	private static final long IMU_GYRO_P_SENS_NUM = 4359;
	private static final long IMU_GYRO_Q_SENS_NUM = 4359;
	private static final long IMU_GYRO_R_SENS_NUM = 4359;
	private static final long IMU_GYRO_P_SENS_DEN = 1000;
	private static final long IMU_GYRO_Q_SENS_DEN = 1000;
	private static final long IMU_GYRO_R_SENS_DEN = 1000;
	public static double IMU_ACCEL_X_SENS =37.91;
	public static int IMU_ACCEL_X_SENS_NUM= 3791;
	public static int IMU_ACCEL_X_SENS_DEN= 100;
	public static double IMU_ACCEL_Y_SENS =37.91;
	public static int IMU_ACCEL_Y_SENS_NUM =3791;
	public static int IMU_ACCEL_Y_SENS_DEN =100;
	public static double IMU_ACCEL_Z_SENS =39.24;
	public static int IMU_ACCEL_Z_SENS_NUM =3924;
	public static int IMU_ACCEL_Z_SENS_DEN =100;
	
	public static boolean accel_available;
	public static boolean gyro_available;
	
	public static void imu_init(){
//		#ifdef IMU_POWER_GPIO
//		  gpio_setup_output(IMU_POWER_GPIO);
//		  IMU_POWER_GPIO_ON(IMU_POWER_GPIO);
//		#endif
		
		 /* initialises neutrals */
		  RATES_ASSIGN(imu.gyro_neutral,  IMU_GYRO_P_NEUTRAL,  IMU_GYRO_Q_NEUTRAL,  IMU_GYRO_R_NEUTRAL);

		  VECT3_ASSIGN(imu.accel_neutral, IMU_ACCEL_X_NEUTRAL, IMU_ACCEL_Y_NEUTRAL, IMU_ACCEL_Z_NEUTRAL);
		  //VECT3_ASSIGN(imu.mag_neutral,   IMU_MAG_X_NEUTRAL,   IMU_MAG_Y_NEUTRAL,   IMU_MAG_Z_NEUTRAL);
		  INT_VECT3_ZERO(imu.mag_neutral);
		  
		  accel_available = false;
		  gyro_available = false;
		  
	}
	public static void imu_feed_gyro_accel(long[] values) {

		  RATES_ASSIGN(imu.gyro_unscaled, values[0],values[1], values[2]);
		  VECT3_ASSIGN(imu.accel_unscaled, values[3], values[4], values[5]);

		  // set availability flags...
		  accel_available = true;
		  gyro_available = true;

		}
	
	public static void ImuScaleAccel(ImuState imu){
		 VECT3_COPY(imu.accel_prev, imu.accel);				
		    imu.accel.x = ((imu.accel_unscaled.x - imu.accel_neutral.x)*IMU_ACCEL_X_SENS_NUM)/IMU_ACCEL_X_SENS_DEN; 
		    imu.accel.y = ((imu.accel_unscaled.y - imu.accel_neutral.y)*IMU_ACCEL_Y_SENS_NUM)/IMU_ACCEL_Y_SENS_DEN; 
		    imu.accel.z = ((imu.accel_unscaled.z - imu.accel_neutral.z)*IMU_ACCEL_Z_SENS_NUM)/IMU_ACCEL_Z_SENS_DEN; 
	}
	
	public static void ImuScaleGyro(){
		RATES_COPY(imu.gyro_prev, imu.gyro);                              
	    imu.gyro.p = ((imu.gyro_unscaled.p - imu.gyro_neutral.p) * IMU_GYRO_P_SENS_NUM) / IMU_GYRO_P_SENS_DEN; 
	    imu.gyro.q = ((imu.gyro_unscaled.q - imu.gyro_neutral.q) * IMU_GYRO_Q_SENS_NUM) / IMU_GYRO_Q_SENS_DEN; 
	    imu.gyro.r = ((imu.gyro_unscaled.r - imu.gyro_neutral.r) * IMU_GYRO_R_SENS_NUM) / IMU_GYRO_R_SENS_DEN; 
	  
	}
	
	public static void ImuScaleMag(){
		
	}
	public static void imu_float_init(){}
}
