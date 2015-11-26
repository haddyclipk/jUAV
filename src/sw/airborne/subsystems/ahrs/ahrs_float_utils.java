package sw.airborne.subsystems.ahrs;

import sw.airborne.math.*;
import static sw.airborne.math.Pprz_algebra.*;
import static sw.airborne.math.Pprz_algebra_float.*;
import static sw.airborne.subsystems.Parameters.*;


public class ahrs_float_utils {
	//public static float FLT_MIN = 1.17549435082228750797e-38F;
	private static float FLT_MIN = Float.MIN_VALUE; 
	public static void ahrs_float_get_quat_from_accel( FloatQuat q,  Int32Vect3 accel){
		  /* normalized accel measurement in floating point */
		  FloatVect3 acc_normalized = new FloatVect3();
		  ACCELS_FLOAT_OF_BFP(acc_normalized, accel);
		  FLOAT_VECT3_NORMALIZE(acc_normalized);

		  /* check for 180deg case */
		  //FLT_MIN in java is 1.4E-45 and in c++ (1E-37 or smaller) may cause calculation errors
		  if ( Math.abs(acc_normalized.z - 1.0) < 5*FLT_MIN ) {
		    QUAT_ASSIGN(q, 0.0, 1.0, 0.0, 0.0);
		  }
		  else {
		    /*
		     * axis we want to rotate around is cross product of accel and reference [0,0,-g]
		     * normalized: cross(acc_normalized, [0,0,-1])
		     * vector part of quaternion is the axis
		     * scalar part (angle): 1.0 + dot(acc_normalized, [0,0,-1])
		     */
		    q.qx = - acc_normalized.y;
		    q.qy = acc_normalized.x;
		    q.qz = (float) 0.0;
		    q.qi = (float) (1.0 - acc_normalized.z);
		    FLOAT_QUAT_NORMALIZE(q);
		  }
		}
	
	public static void ahrs_float_get_quat_from_accel_mag( FloatQuat q,  Int32Vect3 accel,  Int32Vect3 mag) {

		  /* the quaternion representing roll and pitch from acc measurement */
		   FloatQuat q_a = new FloatQuat();
		  ahrs_float_get_quat_from_accel(q_a, accel);


		  /* convert mag measurement to float */
		   FloatVect3 mag_float = new FloatVect3();
		   MAGS_FLOAT_OF_BFP(mag_float, mag);

		  /* and rotate to horizontal plane using the quat from above */
		   FloatRMat rmat_phi_theta = new FloatRMat();
		   FLOAT_RMAT_OF_QUAT(rmat_phi_theta, q_a);
		   FloatVect3 mag_ltp = new FloatVect3();
		   FLOAT_RMAT_VECT3_TRANSP_MUL(mag_ltp, rmat_phi_theta, mag_float);

		  /* heading from mag . make quaternion to rotate around ltp z axis*/
		   FloatQuat q_m = new FloatQuat();

		  /* dot([mag_n.x, mag_n.x, 0], [AHRS_H_X, AHRS_H_Y, 0]) */
		  float dot = mag_ltp.x * AHRS_H_X + mag_ltp.y * AHRS_H_Y;

		  /* |v1||v2| */
		  float norm2 = (float) (Math.sqrt(SQUARE(mag_ltp.x) + SQUARE(mag_ltp.y))
		    * Math.sqrt(SQUARE(AHRS_H_X) + SQUARE(AHRS_H_Y)));

		  // catch 180deg case
		  if (Math.abs(norm2 + dot) < 5*FLT_MIN) {
		    QUAT_ASSIGN(q_m, 0.0, 0.0, 0.0, 1.0);
		  } else {
		    /* q_xyz = cross([mag_n.x, mag_n.y, 0], [AHRS_H_X, AHRS_H_Y, 0]) */
		    q_m.qx = (float) 0.0;
		    q_m.qy = (float) 0.0;
		    q_m.qz = mag_ltp.x * AHRS_H_Y - mag_ltp.y * AHRS_H_X;
		    q_m.qi = norm2 + dot;
		    FLOAT_QUAT_NORMALIZE(q_m);
		  }

		  // q_ltp2imu = q_a * q_m
		  // and wrap and normalize
		  FLOAT_QUAT_COMP_NORM_SHORTEST(q, q_m, q_a);
		}

	

}
