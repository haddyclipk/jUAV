package sw.airborne.firmwares.rotorcraft.stabilization;

import static sw.airborne.firmwares.rotorcraft.stabilization.Stabilization_attitude_ref_euler_int.*;

public class Stabilization_attitude_ref_saturate {
	
	public static void SATURATE_SPEED_TRIM_ACCEL() {                   
	    if (stab_att_ref_rate.p >= REF_RATE_MAX_P) {		
	        stab_att_ref_rate.p = REF_RATE_MAX_P;             
	        if (stab_att_ref_accel.p > 0)                     
	          stab_att_ref_accel.p = 0;                       
	      }                                                   
	      else if (stab_att_ref_rate.p <= -REF_RATE_MAX_P) {	
	        stab_att_ref_rate.p = -REF_RATE_MAX_P;            
	        if (stab_att_ref_accel.p < 0)                     
	          stab_att_ref_accel.p = 0;                       
	      }                                                   
	      if (stab_att_ref_rate.q >= REF_RATE_MAX_Q) {		
	        stab_att_ref_rate.q = REF_RATE_MAX_Q;             
	        if (stab_att_ref_accel.q > 0)                     
	          stab_att_ref_accel.q = 0;                       
	      }                                                   
	      else if (stab_att_ref_rate.q <= -REF_RATE_MAX_Q) {	
	        stab_att_ref_rate.q = -REF_RATE_MAX_Q;            
	        if (stab_att_ref_accel.q < 0)                     
	          stab_att_ref_accel.q = 0;                       
	      }                                                   
	      if (stab_att_ref_rate.r >= REF_RATE_MAX_R) {		
	        stab_att_ref_rate.r = REF_RATE_MAX_R;             
	        if (stab_att_ref_accel.r > 0)                     
	          stab_att_ref_accel.r = 0;                       
	      }                                                   
	      else if (stab_att_ref_rate.r <= -REF_RATE_MAX_R) {	
	        stab_att_ref_rate.r = -REF_RATE_MAX_R;            
	        if (stab_att_ref_accel.r < 0)                     
	          stab_att_ref_accel.r = 0;                       
	      }                                                   
	    }
}
