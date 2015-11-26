package sw.airborne.math;

public class Int32Vect3 {
	public long x;
	public long y;
	public long z;
	public boolean equals(Int32Vect3 _a){
		if(this.x==_a.x) return false;
		if(this.y==_a.y) return false;
		if(this.z==_a.z) return false;
		return true;
	}
	public boolean notequals0(){
		if(this.x==0) return false;
		if(this.y==0) return false;
		if(this.z==0) return false;
		return true;
	}
	
}
