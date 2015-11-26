//package sw.airborne.firmwares.rotorcraft.stabilization;
//import static sw.airborne.math.Pprz_trig_int.*;
//import static sw.airborne.math.Pprz_algebra.*;
//import static sw.airborne.firmwares.rotorcraft.Stabilization.*;
//import static sw.include.Std.*;
//
//import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_rc_setpoint.*;
//import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_ref_quat_int.*;
//import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_ref_euler_int.*;
//import static sw.airborne.math.Pprz_algebra_int.*;
//import static sw.airborne.State.*;
//import sw.airborne.math.*;
//import  sw.airborne.math.Int32Quat;
//import sw.airborne.math.Int32Vect2;
//import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_quat_transformations.*;
//import sw.include.Std;
//import static sw.airborne.math.Pprz_geodetic.*;
//import static sw.airborne.math.Pprz_orientation_conversion.*;
//import static sw.airborne.math.Pprz_geodetic_int.*;
//import static sw.airborne.math.Pprz_geodetic_float.*;
//import static sw.airborne.math.Pprz_algebra_float.*;
//
//public class Stabilization_attitude_quat_transformations {
//	public static void quat_from_rpy_cmd_i( Int32Quat quat,  Int32Eulers rpy) {
//		 FloatEulers rpy_f = null;
//		EULERS_FLOAT_OF_BFP(rpy_f, rpy);
//		 FloatQuat quat_f = null;
//		quat_from_rpy_cmd_f(quat_f, rpy_f);
//		QUAT_BFP_OF_REAL(quat, quat_f);
//	}
//
//	public static void quat_from_rpy_cmd_f( FloatQuat quat,  FloatEulers rpy) {
//		// only a plug for now... doesn't apply roll/pitch wrt. current yaw angle
//
//		/* orientation vector describing simultaneous rotation of roll/pitch/yaw */
//		FloatVect3 ov = {rpy.phi, rpy.theta, rpy.psi};
//		/* quaternion from that orientation vector */
//		FLOAT_QUAT_OF_ORIENTATION_VECT(quat, ov);
//
//	}
//
//	public static void quat_from_earth_cmd_i( Int32Quat quat,  Int32Vect2 cmd, int heading) {
//		// use float conversion for now...
//		 FloatVect2 cmd_f=new FloatVect2();
//		cmd_f.x = ANGLE_FLOAT_OF_BFP(cmd.x);
//		cmd_f.y = ANGLE_FLOAT_OF_BFP(cmd.y);
//		float heading_f = ANGLE_FLOAT_OF_BFP(heading);
//
//		 FloatQuat quat_f = null;
//		quat_from_earth_cmd_f(quat_f, cmd_f, heading_f);
//
//		// convert back to fixed point
//		QUAT_BFP_OF_REAL(quat, quat_f);
//	}
//	
//	public static void quat_from_earth_cmd_i( Int32Quat quat,  Int32Vect2 cmd, long heading) {
//		// use float conversion for now...
//		FloatVect2 cmd_f=new FloatVect2();
//		cmd_f.x = ANGLE_FLOAT_OF_BFP(cmd.x);
//		cmd_f.y = ANGLE_FLOAT_OF_BFP(cmd.y);
//		float heading_f = ANGLE_FLOAT_OF_BFP(heading);
//		
//		FloatQuat quat_f = null;
//		quat_from_earth_cmd_f(quat_f, cmd_f, heading_f);
//		
//		// convert back to fixed point
//		QUAT_BFP_OF_REAL(quat, quat_f);
//	}
//
//	public static void quat_from_earth_cmd_f( FloatQuat quat,  FloatVect2 cmd, float heading) {
//
//		/* cmd_x is positive to north = negative pitch
//		 * cmd_y is positive to east = positive roll
//		 *
//		 * orientation vector describing simultaneous rotation of roll/pitch
//		 */
//		FloatVect3 ov = new FloatVect3();
//		ov.x=cmd.y; ov.y=-cmd.x; ov.z=(float) 0.0;
//		/* quaternion from that orientation vector */
//		 FloatQuat q_rp;
//		FLOAT_QUAT_OF_ORIENTATION_VECT(q_rp, ov);
//
//		/* as rotation matrix */
//		 FloatRMat R_rp;
//		FLOAT_RMAT_OF_QUAT(R_rp, q_rp);
//		/* body x-axis (before heading command) is first column */
//		 FloatVect3 b_x;
//		VECT3_ASSIGN(b_x, R_rp.m[0], R_rp.m[3], R_rp.m[6]);
//		/* body z-axis (thrust vect) is last column */
//		 FloatVect3 thrust_vect;
//		VECT3_ASSIGN(thrust_vect, R_rp.m[2], R_rp.m[5], R_rp.m[8]);
//
//		/// @todo optimize yaw angle calculation
//
//		/*
//		 * Instead of using the psi setpoint angle to rotate around the body z-axis,
//		 * calculate the real angle needed to align the projection of the body x-axis
//		 * onto the horizontal plane with the psi setpoint.
//		 *
//		 * angle between two vectors a and b:
//		 * angle = atan2(norm(cross(a,b)), dot(a,b)) * sign(dot(cross(a,b), n))
//		 * where the normal n is the thrust vector (i.e. both a and b lie in that plane)
//		 */
//
//		// desired heading vect in earth x-y plane
//		FloatVect3 psi_vect = {cosf(heading), sinf(heading), 0.0};
//
//		/* projection of desired heading onto body x-y plane
//		 * b = v - dot(v,n)*n
//		 */
//		float dot = FLOAT_VECT3_DOT_PRODUCT(psi_vect, thrust_vect);
//		 FloatVect3 dotn;
//		FLOAT_VECT3_SMUL(dotn, thrust_vect, dot);
//
//		// b = v - dot(v,n)*n
//		 FloatVect3 b;
//		FLOAT_VECT3_DIFF(b, psi_vect, dotn);
//		dot = FLOAT_VECT3_DOT_PRODUCT(b_x, b);
//		 FloatVect3 cross;
//		VECT3_CROSS_PRODUCT(cross, b_x, b);
//		// norm of the cross product
//		float nc = FLOAT_VECT3_NORM(cross);
//		// angle = atan2(norm(cross(a,b)), dot(a,b))
//		float yaw2 = atan2(nc, dot) / 2.0;
//
//		// negative angle if needed
//		// sign(dot(cross(a,b), n)
//		float dot_cross_ab = FLOAT_VECT3_DOT_PRODUCT(cross, thrust_vect);
//		if (dot_cross_ab < 0) {
//			yaw2 = -yaw2;
//		}
//
//		/* quaternion with yaw command */
//		 FloatQuat q_yaw;
//		QUAT_ASSIGN(q_yaw, cosf(yaw2), 0.0, 0.0, sinf(yaw2));
//
//		/* final setpoint: apply roll/pitch, then yaw around resulting body z-axis */
//		FLOAT_QUAT_COMP(quat, q_rp, q_yaw);
//		FLOAT_QUAT_NORMALIZE(quat);
//		FLOAT_QUAT_WRAP_SHORTEST(quat);
//
//	}
//
//}
