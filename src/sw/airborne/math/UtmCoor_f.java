package sw.airborne.math;

public class UtmCoor_f {
	public float north; ///< in meters
	  public float east; ///< in meters
	  public float alt; ///< in meters above WGS84 reference ellipsoid
	  public int zone;//unit8
	  public UtmCoor_f clone(){
		  UtmCoor_f temp= new UtmCoor_f();
		  temp.north = this.north;
		  temp.east = this.east;
		  temp.alt = this.alt;
		  temp.zone = this.zone;
		  return temp;
	  }
}
