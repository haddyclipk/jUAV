package sw.airborne.subsystems;

import sw.airborne.math.*;

public class ImuFloat {
	  FloatRates   gyro;
	   FloatVect3   accel;
	   FloatVect3   mag;
	   FloatRates   gyro_prev;
	   FloatEulers  body_to_imu_eulers;
	   FloatQuat    body_to_imu_quat;
	   FloatRMat    body_to_imu_rmat;
	   int sample_count;
}
