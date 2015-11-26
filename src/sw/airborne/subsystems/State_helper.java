package sw.airborne.subsystems;

import sw.airborne.math.*;

public class State_helper {
	public static void VECT3_COPY( EcefCoor_i ecef1,EcefCoor_i ecef2){
		ecef1.x = ecef2.x;				
	    ecef1.y = ecef2.y;				
	    ecef1.z = ecef2.z;	}
	public static void VECT3_COPY(NedCoor_i ned1,NedCoor_i ned2 ){
		ned1.x = ned2.x;				
	    ned1.y = ned2.y;				
	    ned1.z = ned2.z;
	}
	public static void VECT3_COPY(EnuCoor_i enu1, EnuCoor_i enu2){
		enu1.x = enu2.x;				
		enu1.y = enu2.y;				
		enu1.z = enu2.z;
	}
	public static void VECT3_COPY( EcefCoor_f ecef1,EcefCoor_f ecef2){
		ecef1.x = ecef2.x;				
	    ecef1.y = ecef2.y;				
	    ecef1.z = ecef2.z;	}
	public static void VECT3_COPY(NedCoor_f ned1,NedCoor_f ned2 ){
		ned1.x = ned2.x;				
	    ned1.y = ned2.y;				
	    ned1.z = ned2.z;
	}
	public static void VECT3_COPY(EnuCoor_f enu1, EnuCoor_f enu2){
		enu1.x = enu2.x;				
		enu1.y = enu2.y;				
		enu1.z = enu2.z;
	}
	public static void LLA_COPY(LlaCoor_i lla1,LlaCoor_i lla2){
		lla1.lat=lla2.lat;
		lla1.lon=lla2.lon;
		lla1.alt=lla2.alt;
	}
	public static void LLA_COPY(LlaCoor_f lla1,LlaCoor_f lla2){
		lla1.lat=lla2.lat;
		lla1.lon=lla2.lon;
		lla1.alt=lla2.alt;
	}
	public static void INT32_VECT3_COPY( EcefCoor_i ecef1,EcefCoor_i ecef2){
		ecef1.x = ecef2.x;				
	    ecef1.y = ecef2.y;				
	    ecef1.z = ecef2.z;	}
	public static void INT32_VECT3_COPY(NedCoor_i ned1,NedCoor_i ned2 ){
		ned1.x = ned2.x;				
	    ned1.y = ned2.y;				
	    ned1.z = ned2.z;
	}
	public static void INT32_VECT3_COPY(EnuCoor_i enu1, EnuCoor_i enu2){
		enu1.x = enu2.x;				
		enu1.y = enu2.y;				
		enu1.z = enu2.z;
	}
	public static void INT32_VECT3_COPY( EcefCoor_f ecef1,EcefCoor_f ecef2){
		ecef1.x = ecef2.x;				
	    ecef1.y = ecef2.y;				
	    ecef1.z = ecef2.z;	}
	public static void INT32_VECT3_COPY(NedCoor_f ned1,NedCoor_f ned2 ){
		ned1.x = ned2.x;				
	    ned1.y = ned2.y;				
	    ned1.z = ned2.z;
	}
	public static void INT32_VECT3_COPY(EnuCoor_f enu1, EnuCoor_f enu2){
		enu1.x = enu2.x;				
		enu1.y = enu2.y;				
		enu1.z = enu2.z;
	}
}
