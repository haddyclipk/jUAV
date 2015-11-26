package sw.airborne.math;

import static sw.airborne.math.Pprz_algebra.*;
import static sw.airborne.math.Pprz_algebra_float.*;

public class Pprz_algebra_double {

	public static void DOUBLE_VECT3_ROUND(DoubleVect3 _v) {
		DOUBLE_VECT3_RINT(_v, _v);
	}


	public static void DOUBLE_VECT3_RINT(DoubleVect3 _vout,DoubleVect3  _vin) {    
		(_vout).x = ((_vin).x);         
		(_vout).y = ((_vin).y);         
		(_vout).z = ((_vin).z);         
	}

	public static void DOUBLE_VECT3_ASSIGN(DoubleVect3 _a,double _x,double _y,double _z) {
		VECT3_ASSIGN(_a, _x, _y, _z);
	}

	public static void DOUBLE_VECT3_COPY(DoubleVect3 _a,DoubleVect3  _b) {
		VECT3_COPY(_a, _b);
	}

	public static void DOUBLE_VECT3_SUM(DoubleVect3 _c,DoubleVect3 _a,DoubleVect3 _b) {            
		(_c).x = (_a).x + (_b).x;           
		(_c).y = (_a).y + (_b).y;           
		(_c).z = (_a).z + (_b).z;           
	}

	public static void DOUBLE_VECT3_CROSS_PRODUCT(DoubleVect3 vo,DoubleVect3  v1,DoubleVect3  v2) {
		FLOAT_VECT3_CROSS_PRODUCT(vo, v1, v2);
	}

	public static void DOUBLE_RMAT_OF_EULERS(DoubleRMat _rm, DoubleEulers  _e) {
		DOUBLE_RMAT_OF_EULERS_321(_rm, _e);
	}

	public static void DOUBLE_RMAT_OF_EULERS_321(DoubleRMat _rm,DoubleEulers  _e) {                
		
		 double sphi   = Math.sin((double)(_e).phi);                
		 double cphi   = Math.cos((_e).phi);                
		 double stheta = Math.sin((_e).theta);              
		 double ctheta = Math.cos((_e).theta);              
		 double spsi   = Math.sin((_e).psi);                
		 double cpsi   = Math.cos((_e).psi);                
		
		_rm.m[0] = ctheta*cpsi;                 
		_rm.m[1] = ctheta*spsi;                 
		_rm.m[2] = -stheta;                 
		_rm.m[3] = sphi*stheta*cpsi - cphi*spsi;        
		_rm.m[4] = sphi*stheta*spsi + cphi*cpsi;        
		_rm.m[5] = sphi*ctheta;                 
		_rm.m[6] = cphi*stheta*cpsi + sphi*spsi;        
		_rm.m[7] = cphi*stheta*spsi - sphi*cpsi;        
		_rm.m[8] = cphi*ctheta;                 
		
	}




	/* multiply _vin by _mat, store in _vout */
	public static void DOUBLE_MAT33_VECT3_MUL(DoubleVect3 _vout,DoubleMat33  _mat,DoubleVect3  _vin) {     
		(_vout).x = (_mat).m[0]*(_vin).x + (_mat).m[1]*(_vin).y + (_mat).m[2]*(_vin).z;   
		(_vout).y = (_mat).m[3]*(_vin).x + (_mat).m[4]*(_vin).y + (_mat).m[5]*(_vin).z;   
		(_vout).z = (_mat).m[6]*(_vin).x + (_mat).m[7]*(_vin).y + (_mat).m[8]*(_vin).z;   
	}

	/* multiply _vin by the transpose of _mat, store in _vout */
	public static void DOUBLE_MAT33_VECT3_TRANSP_MUL(DoubleVect3 _vout,DoubleMat33 _mat,DoubleVect3 _vin) {      
		(_vout).x = (_mat).m[0]*(_vin).x + (_mat).m[3]*(_vin).y + (_mat).m[6]*(_vin).z;   
		(_vout).y = (_mat).m[1]*(_vin).x + (_mat).m[4]*(_vin).y + (_mat).m[7]*(_vin).z;   
		(_vout).z = (_mat).m[2]*(_vin).x + (_mat).m[5]*(_vin).y + (_mat).m[8]*(_vin).z;   
	}

	public static void DOUBLE_QUAT_OF_EULERS(DoubleQuat _q,DoubleEulers _e) {                 
		
		 double phi2   = (_e).phi/ 2.0;                    
		 double theta2 = (_e).theta/2.0;               
		 double psi2   = (_e).psi/2.0;                 
		
		 double s_phi2   = Math.sin( phi2 );                
		 double c_phi2   = Math.cos( phi2 );                
		 double s_theta2 = Math.sin( theta2 );              
		 double c_theta2 = Math.cos( theta2 );              
		 double s_psi2   = Math.sin( psi2 );                
		 double c_psi2   = Math.cos( psi2 );                
		
		(_q).qi =  c_phi2 * c_theta2 * c_psi2 + s_phi2 * s_theta2 * s_psi2; 
		(_q).qx = -c_phi2 * s_theta2 * s_psi2 + s_phi2 * c_theta2 * c_psi2; 
		(_q).qy =  c_phi2 * s_theta2 * c_psi2 + s_phi2 * c_theta2 * s_psi2; 
		(_q).qz =  c_phi2 * c_theta2 * s_psi2 - s_phi2 * s_theta2 * c_psi2; 
		
	}

	public static void DOUBLE_EULERS_OF_QUAT(DoubleEulers _e,DoubleQuat _q) {                 
		
		 double qx2  = (_q).qx*(_q).qx;                
		 double qy2  = (_q).qy*(_q).qy;                
		 double qz2  = (_q).qz*(_q).qz;                
		 double qiqx = (_q).qi*(_q).qx;                
		 double qiqy = (_q).qi*(_q).qy;                
		 double qiqz = (_q).qi*(_q).qz;                
		 double qxqy = (_q).qx*(_q).qy;                
		 double qxqz = (_q).qx*(_q).qz;                
		 double qyqz = (_q).qy*(_q).qz;                
		 double dcm00 = 1.0 - 2.*(  qy2 +  qz2 );          
		 double dcm01 =       2.*( qxqy + qiqz );          
		 double dcm02 =       2.*( qxqz - qiqy );          
		 double dcm12 =       2.*( qyqz + qiqx );          
		 double dcm22 = 1.0 - 2.*(  qx2 +  qy2 );          
		
		(_e).phi = Math.atan2( dcm12, dcm22 );                   
		(_e).theta = -Math.asin( dcm02 );                    
		(_e).psi = Math.atan2( dcm01, dcm00 );                   
		
	}
	
	
	private static double a1 = 6378137.0;
	private static double f1= 1./298.257223563;
	public static void ecef_of_lla_d( EcefCoor_d ecef,  LlaCoor_d lla) {

		  // FIXME : make an ellipsoid struct
		  //static const double a = 6378137.0;           /* earth semimajor axis in meters */
		 // static const double f = 1./298.257223563;    /* reciprocal flattening          */
		   double e2 = 2.*f1-(f1*f1);                /* first eccentricity squared     */

		  double sin_lat = Math.sin(lla.lat);
		  double cos_lat = Math.cos(lla.lat);
		  double sin_lon = Math.sin(lla.lon);
		  double cos_lon = Math.cos(lla.lon);
		  double chi = Math.sqrt(1. - e2*sin_lat*sin_lat);
		  double a_chi = a1 / chi;

		  ecef.x = (a_chi + lla.alt) * cos_lat * cos_lon;
		  ecef.y = (a_chi + lla.alt) * cos_lat * sin_lon;
		  ecef.z = (a_chi*(1. - e2) + lla.alt) * sin_lat;
		}
	
	
	private static double a2 = 6378137.0; 
	private static double f2 =  1./298.257223563;
	public static void lla_of_ecef_d(LlaCoor_d lla,EcefCoor_d ecef) {

		  // FIXME : make an ellipsoid struct
		  //static  double a = 6378137.0;           /* earth semimajor axis in meters */
		  //static  double f = 1./298.257223563;    /* reciprocal flattening          */
		   double b = a2*(1.-f2);                   /* semi-minor axis                */
		   double b2 = b*b;

		   double e2 = 2.*f2-(f2*f2);                /* first eccentricity squared     */
		   double ep2 = f2*(2.-f2)/((1.-f2)*(1.-f2)); /* second eccentricity squared    */
		   double E2 = a2*a2 - b2;


		   double z2 = ecef.z*ecef.z;
		   double r2 = ecef.x*ecef.x+ecef.y*ecef.y;
		   double r = Math.sqrt(r2);
		   double F = 54.*b2*z2;
		   double G = r2 + (1-e2)*z2 - e2*E2;
		   double c = (e2*e2*F*r2)/(G*G*G);
		   double s = Math.pow( (1 + c + Math.sqrt(c*c + 2*c)), 1./3.);
		   double s1 = 1+s+1/s;
		   double P = F/(3*s1*s1*G*G);
		   double Q = Math.sqrt(1+2*e2*e2*P);
		   double ro = -(e2*P*r)/(1+Q) + Math.sqrt((a2*a2/2)*(1+1/Q) - ((1-e2)*P*z2)/(Q*(1+Q)) - P*r2/2);
		   double tmp = (r - e2*ro)*(r - e2*ro);
		   double U = Math.sqrt( tmp + z2 );
		   double V = Math.sqrt( tmp + (1-e2)*z2 );
		   double zo = (b2*ecef.z)/(a2*V);

		  lla.alt = U*(1 - b2/(a2*V));
		  lla.lat = Math.atan((ecef.z + ep2*zo)/r);
		  lla.lon = Math.atan2(ecef.y,ecef.x);

		}
}
