package sw.airborne.subsystems;

public class Radio_control {
	public static int RC_AVG_PERIOD = 8;
	public static int RC_LOST_TIME = 30;
	public static int RC_REALLY_LOST_TIME = 60;
	public static int RC_REALLY_LOST = 2;
	
public static RadioControl radio_control = new RadioControl();
/************* INIT ******************************************************/
//public static void radio_control_init (  ) {
//  int i;
//  for (i=0; i<RADIO_CONTROL_NB_CHANNEL; i++)
//    radio_control.values[i] = 0;
//  radio_control.status = RC_REALLY_LOST;
//  radio_control.time_since_last_frame = RC_REALLY_LOST_TIME;
//  radio_control.radio_ok_cpt = 0;
//  radio_control.frame_rate = 0;
//  radio_control.frame_cpt = 0;
//  radio_control_impl_init();
//}
//
///************* PERIODIC ******************************************************/
//private static int _1Hz;
//public static  void radio_control_periodic_task (  ) {
//  //static int _1Hz;
//  _1Hz++;
//
//  if (_1Hz >= 60) {
//    _1Hz = 0;
//    radio_control.frame_rate = radio_control.frame_cpt;
//    radio_control.frame_cpt = 0;
//  }
//
//  if (radio_control.time_since_last_frame >= RC_REALLY_LOST_TIME) {
//    radio_control.status = RC_REALLY_LOST;
//  } else {
//    if (radio_control.time_since_last_frame >= RC_LOST_TIME) {
//      radio_control.status = RC_LOST;
//      radio_control.radio_ok_cpt = RC_OK_CPT;
//    }
//    radio_control.time_since_last_frame++;
//  }
//
//#if defined RADIO_CONTROL_LED
//  if (radio_control.status == RC_OK) {
//    LED_ON(RADIO_CONTROL_LED);
//  }
//  else {
//    LED_OFF(RADIO_CONTROL_LED);
//  }
//#endif

//}
}
