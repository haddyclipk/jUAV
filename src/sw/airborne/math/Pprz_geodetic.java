package sw.airborne.math;
import static sw.airborne.math.Pprz_algebra.*;


public class Pprz_geodetic {
	public  static void ENU_OF_TO_NED(NedCoor_i _po,EnuCoor_i _pi) {		
	    (_po).x =  (_pi).y;				
	    (_po).y =  (_pi).x;				
	    (_po).z = -(_pi).z;				
	  }
	public  static void ENU_OF_TO_NED(EnuCoor_f _po,NedCoor_f _pi) {		
		(_po).x =  (_pi).y;				
		(_po).y =  (_pi).x;				
		(_po).z = -(_pi).z;				
	}
	public  static void ENU_OF_TO_NED(EnuCoor_i _po,NedCoor_i _pi) {		
		(_po).x =  (_pi).y;				
		(_po).y =  (_pi).x;				
		(_po).z = -(_pi).z;				
	}
	public  static void ENU_OF_TO_NED(NedCoor_f _po,EnuCoor_f _pi) {		
		(_po).x =  (_pi).y;				
		(_po).y =  (_pi).x;				
		(_po).z = -(_pi).z;				
	}

	public static void LLA_ASSIGN(LlaCoor_i _pos,int _lat,int _lon,int _alt){	
	    (_pos).lat = (_lat);			
	    (_pos).lon = (_lon);			
	    (_pos).alt = (_alt);			
	  }

	public static void LLA_COPY(LlaCoor_i _pos1,LlaCoor_i _pos2){			
	    (_pos1).lat = (_pos2).lat;			
	    (_pos1).lon = (_pos2).lon;			
	    (_pos1).alt = (_pos2).alt;			
	}
	public static void LLA_COPY(LlaCoor_f _pos1,LlaCoor_f _pos2){			
		(_pos1).lat = (_pos2).lat;			
		(_pos1).lon = (_pos2).lon;			
		(_pos1).alt = (_pos2).alt;			
	}

	public static void LTP_DEF_COPY(LtpDef_i _def1,LtpDef_i _def2){                              
	    LLA_COPY((_def1).lla, (_def2).lla);                         
	    VECT3_COPY((_def1).ecef, (_def2).ecef);                     
	    RMAT_COPY((_def1).ltp_of_ecef, (_def2).ltp_of_ecef);        
	    (_def1).hmsl = (_def2).hmsl;                                
	  }

	public static void UTM_COPY(UtmCoor_i _u1,UtmCoor_i _u2) {     
	    (_u1).north = (_u2).north;   
	    (_u1).east = (_u2).east;     
	    (_u1).alt = (_u2).alt;       
	    (_u1).zone = (_u2).zone;     
	  }


	public static void ENU_OF_UTM_DIFF(EnuCoor_i _pos,UtmCoor_i _utm1,UtmCoor_i _utm2) { 
	  (_pos).x = (_utm1).east - (_utm2).east;     
	  (_pos).y = (_utm1).north - (_utm2).north;   
	  (_pos).z = (_utm1).alt - (_utm2).alt;       
	}
	public static void ENU_OF_UTM_DIFF(EnuCoor_f _pos,UtmCoor_f _utm1,UtmCoor_f _utm2) { 
		(_pos).x = (_utm1).east - (_utm2).east;     
		(_pos).y = (_utm1).north - (_utm2).north;   
		(_pos).z = (_utm1).alt - (_utm2).alt;       
	}

	public static void NED_OF_UTM_DIFF(NedCoor_i _pos,UtmCoor_i _utm1,UtmCoor_i _utm2) { 
	  (_pos).x = (_utm1).north - (_utm2).north;   
	  (_pos).y = (_utm1).east - (_utm2).east;     
	  (_pos).z = -(_utm1).alt + (_utm2).alt;      
	}
	
	public static void NED_OF_UTM_DIFF(NedCoor_f _pos,UtmCoor_f _utm1,UtmCoor_f _utm2) { 
		(_pos).x = (_utm1).north - (_utm2).north;   
		(_pos).y = (_utm1).east - (_utm2).east;     
		(_pos).z = -(_utm1).alt + (_utm2).alt;      
	}

	public static void UTM_OF_ENU_ADD(UtmCoor_i _utm, EnuCoor_i _pos, UtmCoor_i _utm0) { 
	  (_utm).east = (_utm0).east + (_pos).x;     
	  (_utm).north = (_utm0).north + (_pos).y;   
	  (_utm).alt = (_utm0).alt + (_pos).z;       
	}
	public static void UTM_OF_ENU_ADD(UtmCoor_f _utm, EnuCoor_f _pos, UtmCoor_f _utm0) { 
		(_utm).east = (_utm0).east + (_pos).x;     
		(_utm).north = (_utm0).north + (_pos).y;   
		(_utm).alt = (_utm0).alt + (_pos).z;       
	}

	public static void UTM_OF_NED_ADD(UtmCoor_i _utm,NedCoor_i _pos,UtmCoor_i _utm0) { 
	  (_utm).east = (_utm0).east + (_pos).y;     
	  (_utm).north = (_utm0).north + (_pos).x;   
	  (_utm).alt = (_utm0).alt - (_pos).z;       
	}
	public static void UTM_OF_NED_ADD(UtmCoor_f _utm,NedCoor_f _pos,UtmCoor_f _utm0) { 
		(_utm).east = (_utm0).east + (_pos).y;     
		(_utm).north = (_utm0).north + (_pos).x;   
		(_utm).alt = (_utm0).alt - (_pos).z;       
	}

}
