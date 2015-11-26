package sw.airborne.math;

public class FloatVect2 {
	public float x;
	public float y;
	public boolean equals(FloatVect2 _a){
		if(this.x==_a.x) return false;
		if(this.y==_a.y) return false;
		
		return true;
	}
	public boolean notequals0(){
		if(this.x==0.0) return false;
		if(this.y==0.0) return false;
		//if(this.z==0) return false;
		return true;}
}
