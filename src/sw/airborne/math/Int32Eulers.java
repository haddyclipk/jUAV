package sw.airborne.math;

public class Int32Eulers {
	public long phi; ///< in radians
	public long theta; ///< in radians
	public long psi; ///< in radians
	
	public Int32Eulers clone(){
		Int32Eulers temp = new Int32Eulers();
		temp.phi = this.phi;
		temp.theta = this.theta;
		temp.psi = this.psi;
		return temp;
	}
}
