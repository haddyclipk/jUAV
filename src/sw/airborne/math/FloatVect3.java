package sw.airborne.math;

public class FloatVect3 {
	public float x;
	public float y;
	public float z;
	public boolean equals(FloatVect3 _a){
		if(x==_a.x && y == _a.y && z == _a.z)return true;
		return false;
	}
	public boolean notequals0(){
		if(this.x==0.0) return false;
		if(this.y==0.0) return false;
		if(this.z==0.0) return false;
		return true;}
}
