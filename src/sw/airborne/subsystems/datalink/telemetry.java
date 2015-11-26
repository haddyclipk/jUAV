package sw.airborne.subsystems.datalink;
import static sw.airborne.firmwares.rotorcraft.Autopilot.*;
public class telemetry {
	public int nb;                 ///< number of messages
	public telemetry_msg[] msgs=new telemetry_msg[13]; 
	public static void register_periodic_telemetry( telemetry  _pt, final String  _msg, int _cb) {
		// look for message name
		int i;
		for (i = 0; i < _pt.nb; i++) {
			if (_pt.msgs[i].msg== _msg) {
				// register callback if not already done
				// if (_pt.msgs[i].cb == NULL) {
				//_pt->msgs[i].cb = _cb;
				switch(_cb){
				case 1:send_alive();
				break;
				case 2:send_status();break;
				case 3://send_energy();
					break;
				case 4:send_fp();break;
				case 5:send_rotorcraft_cmd();break;
				case 6:send_dl_value();break;
				case 7: send_actuators();break;
				}

				//return true;
			}
			// else { return false; }
		}

		// message name is not in telemetry file
		// return false;
	}
}
class pprz_telemetry {
	int nb;                 ///< number of messages
	public telemetry_msg[] msgs=new telemetry_msg[13]; ///< the list of (msg name, callbacks)
}
class telemetry_msg {
	String msg=new String();     ///< name in telemetry xml file
	//telemetry_cb cb;  ///< callback funtion
}