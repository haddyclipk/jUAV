package sw.simulator.nps;

public class Nps_autopilot {
	public static boolean nps_bypass_ahrs;
	public static boolean nps_bypass_ins;
	//public static NpsFdm fdm = new NpsFdm();
	public static int MOTOR_MIXING_NB_MOTOR = 4; //Should be defined by firmware;
	public static int NPS_COMMANDS_NB = MOTOR_MIXING_NB_MOTOR; //Equal to this for rotorcraft;
	
	public static NpsAutopilot autopilot = new NpsAutopilot();
}
