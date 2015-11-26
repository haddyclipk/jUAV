package sw.airborne.subsystems.actuators.motor_mixing;

public class Motor_mixing {
	
	public static  int MOTOR_MIXING_NB_MOTOR=13;//????????????
	public static int[] commands=new int[MOTOR_MIXING_NB_MOTOR];
	int[] trim=new int[MOTOR_MIXING_NB_MOTOR];
	  boolean[] override_enabled=new boolean[MOTOR_MIXING_NB_MOTOR];
	  int[] override_value=new int[MOTOR_MIXING_NB_MOTOR];
	  public int nb_saturation;
	  public int nb_failure;
}
