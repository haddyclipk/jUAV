package sw.airborne.math;
//import static sw.airborne.subsystems.ins.Ins_float_invariant.*;
import static sw.airborne.math.Pprz_algebra_float.*;
import sw.airborne.subsystems.ins.*;

public class Pprz_rk_float {
//	public static void runge_kutta_4_float(inv_state  xo,
//			inv_state  x,  int n,
//		    inv_command  u,  int m,
//		    //void (f)(float  o,  float  x,  int n,  float  u,  int m),
//		     float dt){
//		inv_state k1 = new inv_state();
//		inv_state k2 = new inv_state();
//		inv_state k3 = new inv_state();
//		inv_state k4 = new inv_state();
//		inv_state ktmp = new inv_state();
//		
//		// k1 = invariant_model(x, u)
//		invariant_model(k1, x, n, u, m);
//
//		  // k2 = invariant_model(x + dt * (k1 / 2), u)
//		  float_vect_smul(ktmp, k1, dt / 2., n);
//		  float_vect_add(ktmp, x, n);
//		  invariant_model(k2, ktmp, n, u, m);
//
//		  // k3 = invariant_model(x + dt * (k2 / 2), u)
//		  float_vect_smul(ktmp, k2, dt / 2., n);
//		  float_vect_add(ktmp, x, n);
//		  invariant_model(k3, ktmp, n, u, m);
//
//		  // k4 = invariant_model(x + dt * k3, u)
//		  float_vect_smul(ktmp, k3, dt, n);
//		  float_vect_add(ktmp, x, n);
//		  invariant_model(k4, ktmp, n, u, m);
//
//		  // xo = x + (dt / 6) * (k1 + 2 * (k2 + k3) + k4)
//		  float_vect_add(k2, k3, n);
//		  float_vect_smul(k2, k2, 2., n);
//		  float_vect_add(k1, k2, n);
//		  float_vect_add(k1, k4, n);
//		  float_vect_smul(k1, k1, dt / 6., n);
//		  float_vect_sum(xo, x, k1, n);
//
//		
//	}

	
}
