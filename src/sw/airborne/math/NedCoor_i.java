package sw.airborne.math;

public class NedCoor_i{
	public long x, y, z;
	
	public boolean notequals0(){
		if(x!=0 && y!= 0 & z!= 0) return true;
		else return false;
	}
}
