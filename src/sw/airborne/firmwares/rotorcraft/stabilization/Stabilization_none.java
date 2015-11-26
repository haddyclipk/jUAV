package sw.airborne.firmwares.rotorcraft.stabilization;

import sw.airborne.math.*;
import static sw.airborne.math.Pprz_algebra.*;
import static sw.airborne.math.Pprz_algebra_int.*;
import static sw.airborne.firmwares.rotorcraft.Stabilization.*;
public class Stabilization_none {
	public static Int32Rates stabilization_none_rc_cmd = new Int32Rates();

	public static void stabilization_none_init() {
		INT_RATES_ZERO(stabilization_none_rc_cmd);
	}

//	public static void stabilization_none_read_rc(  ) {
//
//		stabilization_none_rc_cmd.p = (int)radio_control.values[RADIO_ROLL];
//		stabilization_none_rc_cmd.q = (int)radio_control.values[RADIO_PITCH];
//		stabilization_none_rc_cmd.r = (int)radio_control.values[RADIO_YAW];
//	}

	public static void stabilization_none_enter() {
		INT_RATES_ZERO(stabilization_none_rc_cmd);
	}

	public static void stabilization_none_run(boolean in_flight) {
		/* just directly pass rc commands through */
		stabilization_cmd[0]  = stabilization_none_rc_cmd.p;
		stabilization_cmd[1] = stabilization_none_rc_cmd.q;
		stabilization_cmd[2]   = stabilization_none_rc_cmd.r;
	}
}
