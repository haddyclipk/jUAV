package sw.airborne.subsystems.ins;
import static sw.airborne.firmwares.rotorcraft.Main.*;

public class vf_float {
	public static Vff vff = new Vff();
	public static int VFF_STATE_SIZE = 3;
	public static int INS_PROPAGATE_FREQUENCY = PERIODIC_FREQUENCY;
	public static float DT_VFILTER  = (float) (1./(INS_PROPAGATE_FREQUENCY));
	public static float VF_FLOAT_ACCEL_NOISE = (float) 0.5;
	public static float Qzz = (float) (VF_FLOAT_ACCEL_NOISE * DT_VFILTER * DT_VFILTER / 2.);
	public static float Qzdotzdot = VF_FLOAT_ACCEL_NOISE * DT_VFILTER;
	public static float Qbiasbias = (float) 1e-7;
	
	public static void vff_propagate(float accel) {
		  /* update state (Xk1) */
		  vff.zdotdot = (float) (accel + 9.81 - vff.bias);
		  vff.z = vff.z + DT_VFILTER * vff.zdot;
		  vff.zdot = vff.zdot + DT_VFILTER * vff.zdotdot;
		  /* update covariance (Pk1) */
		   float FPF00 = vff.P[0][0] + DT_VFILTER * ( vff.P[1][0] + vff.P[0][1] + DT_VFILTER * vff.P[1][1] );
		   float FPF01 = vff.P[0][1] + DT_VFILTER * ( vff.P[1][1] - vff.P[0][2] - DT_VFILTER * vff.P[1][2] );
		   float FPF02 = vff.P[0][2] + DT_VFILTER * ( vff.P[1][2] );
		   float FPF10 = vff.P[1][0] + DT_VFILTER * (-vff.P[2][0] + vff.P[1][1] - DT_VFILTER * vff.P[2][1] );
		   float FPF11 = vff.P[1][1] + DT_VFILTER * (-vff.P[2][1] - vff.P[1][2] + DT_VFILTER * vff.P[2][2] );
		   float FPF12 = vff.P[1][2] + DT_VFILTER * (-vff.P[2][2] );
		   float FPF20 = vff.P[2][0] + DT_VFILTER * ( vff.P[2][1] );
		   float FPF21 = vff.P[2][1] + DT_VFILTER * (-vff.P[2][2] );
		   float FPF22 = vff.P[2][2];

		  vff.P[0][0] = FPF00 + Qzz;
		  vff.P[0][1] = FPF01;
		  vff.P[0][2] = FPF02;
		  vff.P[1][0] = FPF10;
		  vff.P[1][1] = FPF11 + Qzdotzdot;
		  vff.P[1][2] = FPF12;
		  vff.P[2][0] = FPF20;
		  vff.P[2][1] = FPF21;
		  vff.P[2][2] = FPF22 + Qbiasbias;

		}

}
