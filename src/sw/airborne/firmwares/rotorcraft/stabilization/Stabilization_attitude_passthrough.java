//package sw.airborne.firmwares.rotorcraft.stabilization;
//import static sw.airborne.math.Pprz_trig_int.*;
//import static sw.airborne.math.Pprz_algebra.*;
//import static sw.airborne.firmwares.rotorcraft.Stabilization.*;
//import static sw.include.Std.*;
//
//import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_rc_setpoint.*;
//import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_ref_quat_int.*;
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
//public class Stabilization_attitude_passthrough {
//	 public static Int32Eulers stab_att_sp_euler;
//
//
//	public static void stabilization_attitude_init() {
//	  INT_EULERS_ZERO(stab_att_sp_euler);
//	}
//
//	public static void stabilization_attitude_read_rc(boolean in_flight, boolean in_carefree, boolean coordinated_turn) {
//	  //Read from RC
//	  stabilization_attitude_read_rc_setpoint_eulers(stab_att_sp_euler, in_flight, in_carefree, coordinated_turn);
//	}
//
//	public static void stabilization_attitude_enter() {
//
//	}
//
//	public static void stabilization_attitude_run(boolean  in_flight ) {
//
//	  /* For roll and pitch we pass trough the desired angles as stabilization command */
//	  int angle2cmd = (MAX_PPRZ/TRAJ_MAX_BANK);
//	  stabilization_cmd[COMMAND_ROLL] = stab_att_sp_euler.phi * angle2cmd;
//	  stabilization_cmd[COMMAND_PITCH] = stab_att_sp_euler.theta * angle2cmd;
//
//	  //TODO: Fix yaw with PID controller
//	  int yaw_error = stateGetNedToBodyEulers_i().psi - stab_att_sp_euler.psi;
//	  INT32_ANGLE_NORMALIZE(yaw_error);
//	  //	stabilization_cmd[COMMAND_YAW] = yaw_error * MAX_PPRZ / INT32_ANGLE_PI;
//
//	  /* bound the result */
//	  BoundAbs(stabilization_cmd[COMMAND_ROLL], MAX_PPRZ);
//	  BoundAbs(stabilization_cmd[COMMAND_PITCH], MAX_PPRZ);
//	  BoundAbs(stabilization_cmd[COMMAND_YAW], MAX_PPRZ);
//	}
//
//	public static void stabilization_attitude_set_failsafe_setpoint() {
//	  stab_att_sp_euler.phi = 0;
//	  stab_att_sp_euler.theta = 0;
//	  stab_att_sp_euler.psi = stateGetNedToBodyEulers_i().psi;
//	}
//
//	public static void stabilization_attitude_set_rpy_setpoint_i( Int32Eulers rpy) {
//	  //memcpy(&stab_att_sp_euler, rpy, sizeof( Int32Eulers));
//		stab_att_sp_euler = rpy.clone();
//	}
//
//	public static void stabilization_attitude_set_earth_cmd_i( Int32Vect2 cmd, int heading) {
//	  /* Rotate horizontal commands to body frame by psi */
//	  int psi = stateGetNedToBodyEulers_i().psi;
//	  int s_psi, c_psi;
//	  PPRZ_ITRIG_SIN(s_psi, psi);
//	  PPRZ_ITRIG_COS(c_psi, psi);
//	  stab_att_sp_euler.phi = (-s_psi * cmd.x + c_psi * cmd.y) >> INT32_TRIG_FRAC;
//	  stab_att_sp_euler.theta = -(c_psi * cmd.x + s_psi * cmd.y) >> INT32_TRIG_FRAC;
//	  stab_att_sp_euler.psi = heading;
//	}
//}
