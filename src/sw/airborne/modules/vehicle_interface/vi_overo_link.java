package sw.airborne.modules.vehicle_interface;
import static sw.airborne.modules.vehicle_interface.Vi.*;
import static sw.airborne.fms.fms_autopilot_msg.*;
public class vi_overo_link {
	
	public static void vi_notify_imu_available() {
		  vi.available_sensors |= (1<<VI_IMU_DATA_VALID);
	}


}
