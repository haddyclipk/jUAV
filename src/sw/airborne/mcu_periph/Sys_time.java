package sw.airborne.mcu_periph;

public class Sys_time {
	public static final int SYS_TIME_NB_TIMER = 8;
	public static int nb_sec; // /< full seconds since startup
	public static int nb_sec_rem; // /< remainder of seconds since startup in
									// CPU_TICKS
	public static int nb_tick; // /< SYS_TIME_TICKS since startup
	public static sys_time_timer[] timer;

	public float resolution; // /< sys_time_timer resolution in seconds
	public static int ticks_per_sec; // /< sys_time ticks per second
										// (SYS_TIME_FREQUENCY)
	public int resolution_cpu_ticks; // /< sys_time_timer resolution in cpu
										// ticks
	public int cpu_ticks_per_sec;

	public static int sys_time_register_timer(float duration, sys_time_cb cb) {
		int start_time = nb_tick;
		// sys_time systime=new sys_time();
		for (int i = 0; i < SYS_TIME_NB_TIMER; i++) {
			if (!timer[i].in_use) {
				timer[i].cb = cb;
				timer[i].elapsed = false;
				timer[i].end_time = start_time
						+ sys_time_ticks_of_sec(duration);
				timer[i].duration = sys_time_ticks_of_sec(duration);
				timer[i].in_use = true;
				return i;
			}
		}
		return SYS_TIME_NB_TIMER;
	}

	public static boolean sys_time_check_and_ack_timer(int id) {
		if (Sys_time.timer[id].elapsed) {
			Sys_time.timer[id].elapsed = false;
			return true;
		}
		return false;
	}

	public static int sys_time_ticks_of_sec(float seconds) {
		// TODO Auto-generated method stub
		return (int) (seconds * Sys_time.ticks_per_sec + 0.5);
		// return 0;
	}
}

class sys_time_cb {
	int cb;
}

// class sys_time{
// static int nb_sec; ///< full seconds since startup
// static int nb_sec_rem; ///< remainder of seconds since startup in CPU_TICKS
// static int nb_tick; ///< SYS_TIME_TICKS since startup
// static sys_time_timer[] timer;
//
// float resolution; ///< sys_time_timer resolution in seconds
// static int ticks_per_sec; ///< sys_time ticks per second (SYS_TIME_FREQUENCY)
// int resolution_cpu_ticks; ///< sys_time_timer resolution in cpu ticks
// int cpu_ticks_per_sec;
// }

class sys_time_timer {
	//Add constructor for class
	public boolean in_use;
	public sys_time_cb cb;
	public boolean elapsed;
	public int end_time; // /< in SYS_TIME_TICKS
	public int duration;
} // /< in SYS_TIME_TICKS

