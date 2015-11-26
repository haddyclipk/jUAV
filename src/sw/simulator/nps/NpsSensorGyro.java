package sw.simulator.nps;

import sw.airborne.math.*;

public class NpsSensorGyro {
	public DoubleVect3 value;
	public int min, max;
	public DoubleMat33 sensitivity;
	public DoubleVect3 neutral, noise_std_dev, bias_initial, bias_random_walk_std_dev, bias_random_walk_value;
	public double next_update;
	public boolean data_available;
}
