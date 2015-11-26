package sw.airborne.subsystems.ahrs;

import static sw.airborne.subsystems.Ahrs.*;

public class arhs_gx3 {
	
		public static final int AHRS_ALIGNER_LED = 3;

		public static void ahrs_aligner_run() {
		if(AHRS_ALIGNER_LED!=0)//AHRS_ALIGNER_LED= 1or 2 or 3??
		  //LED_ON(AHRS_ALIGNER_LED); //?????
		
		  ahrs.status = AHRS_RUNNING;
		}

}
