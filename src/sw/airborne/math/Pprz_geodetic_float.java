package sw.airborne.math;
import static sw.airborne.math.Pprz_geodetic.*;
import static sw.airborne.math.Pprz_algebra.*;
import static sw.airborne.math.Pprz_geodetic_utm.*;
import static sw.include.Std.*;

public class Pprz_geodetic_float {
	
	public static void CScal(int k, complex z){
		z.re *= k;  z.im *= k; 
	}
	
	public static void CScal(float k, complex z){
		z.re *= k;  z.im *= k; 
	}
	
	public static void CScal(double k, complex z){
		z.re *= k;  z.im *= k; 
	}
	
	public static void CSub(complex z1, complex z2){
		z2.re-=z1.re; z2.im-=z1.im;
	}
	
	public static void CI(complex z){
		float tmp = z.re;
		z.re = -z.im;
		z.im = tmp;
	}
	
	public static void CAdd(complex a, complex b){
		a.re+=b.re; a.im+=b.im;
	}
	
	public static void CSin(complex z){
		CI(z);
		complex _z = new complex();
		_z.re = -z.re;
		_z.im = -z.im;// {-z.re, -z.im}; 
		float e = (float) Math.exp(z.re); 
		float cos_z_im =(float)Math.cos(z.im); z.re = e*cos_z_im; 
		float sin_z_im = (float) Math.sin(z.im); z.im = e*sin_z_im; _z.re = cos_z_im/e; _z.im = -sin_z_im/e; CSub(_z, z); 
		CScal(-0.5, z); 
		CI(z); 
	}
	
	public static void ecef_of_ned_vect_f( EcefCoor_f ecef,  LtpDef_f def,  NedCoor_f ned) {
		   EnuCoor_f enu=new EnuCoor_f();
		  ENU_OF_TO_NED(enu, ned);
		  ecef_of_enu_vect_f(ecef, def, enu);
		}
	
	public static void ecef_of_enu_vect_f( EcefCoor_f ecef,  LtpDef_f def,  EnuCoor_f enu) {
		  /* convert used floats to double */
		   DoubleMat33 ltp_of_ecef_d = new DoubleMat33();
		  ltp_of_ecef_d.m[0] = (double) def.ltp_of_ecef.m[0];
		  ltp_of_ecef_d.m[1] = (double) def.ltp_of_ecef.m[1];
		  ltp_of_ecef_d.m[2] = (double) def.ltp_of_ecef.m[2];
		  ltp_of_ecef_d.m[3] = (double) def.ltp_of_ecef.m[3];
		  ltp_of_ecef_d.m[4] = (double) def.ltp_of_ecef.m[4];
		  ltp_of_ecef_d.m[5] = (double) def.ltp_of_ecef.m[5];
		  ltp_of_ecef_d.m[6] = (double) def.ltp_of_ecef.m[6];
		  ltp_of_ecef_d.m[7] = (double) def.ltp_of_ecef.m[7];
		  ltp_of_ecef_d.m[8] = (double) def.ltp_of_ecef.m[8];
		   EnuCoor_f enu_d = new EnuCoor_f();
		  enu_d.x = (float) enu.x;
		  enu_d.y = (float) enu.y;
		  enu_d.z = (float) enu.z;

		  /* compute in double */
		   EcefCoor_d ecef_d = new EcefCoor_d();
		  MAT33_VECT3_TRANSP_MUL(ecef_d, ltp_of_ecef_d, enu_d);

		  /* convert result back to float*/
		  ecef.x = (float) ecef_d.x;
		  ecef.y = (float) ecef_d.y;
		  ecef.z = (float) ecef_d.z;
		}
	
	public static void ned_of_ecef_vect_f( NedCoor_f ned,  LtpDef_f def,  EcefCoor_f ecef) {
		   EnuCoor_f enu = new EnuCoor_f();
		  enu_of_ecef_vect_f(enu, def, ecef);
		  ENU_OF_TO_NED(ned, enu);
		}
	
	public static void enu_of_ecef_vect_f( EnuCoor_f enu,  LtpDef_f def,  EcefCoor_f ecef) {
		  MAT33_VECT3_MUL(enu, def.ltp_of_ecef, ecef);
		}
	
	public static void ecef_of_ned_point_f( EcefCoor_f ecef,  LtpDef_f def,  NedCoor_f ned) {
		   EnuCoor_f enu = new EnuCoor_f();
		  ENU_OF_TO_NED(enu, ned);
		  ecef_of_enu_point_f(ecef, def, enu);
		}

	/*
	 * not enought precision with float - use double
	 */
	public static void ecef_of_enu_point_f( EcefCoor_f ecef,  LtpDef_f def,  EnuCoor_f enu) {
	  /* convert used floats to double */
	   DoubleMat33 ltp_of_ecef_d = new DoubleMat33();
	  ltp_of_ecef_d.m[0] = (double) def.ltp_of_ecef.m[0];
	  ltp_of_ecef_d.m[1] = (double) def.ltp_of_ecef.m[1];
	  ltp_of_ecef_d.m[2] = (double) def.ltp_of_ecef.m[2];
	  ltp_of_ecef_d.m[3] = (double) def.ltp_of_ecef.m[3];
	  ltp_of_ecef_d.m[4] = (double) def.ltp_of_ecef.m[4];
	  ltp_of_ecef_d.m[5] = (double) def.ltp_of_ecef.m[5];
	  ltp_of_ecef_d.m[6] = (double) def.ltp_of_ecef.m[6];
	  ltp_of_ecef_d.m[7] = (double) def.ltp_of_ecef.m[7];
	  ltp_of_ecef_d.m[8] = (double) def.ltp_of_ecef.m[8];
	   EnuCoor_f enu_d = new EnuCoor_f();
	  enu_d.x = enu.x;
	  enu_d.y =  enu.y;
	  enu_d.z = enu.z;

	  /* compute in double */
	   EcefCoor_d ecef_d = new EcefCoor_d();
	  MAT33_VECT3_TRANSP_MUL(ecef_d, ltp_of_ecef_d, enu_d);

	  /* convert result back to float and add it*/
	  ecef.x = (float) ecef_d.x + def.ecef.x;
	  ecef.y = (float) ecef_d.y + def.ecef.y;
	  ecef.z = (float) ecef_d.z + def.ecef.z;
	}
	
	public static void lla_of_utm_f( LlaCoor_f lla,  UtmCoor_f utm) {
		  float scale =(float)( 1 / N / serie_coeff_proj_mercator[0]);
		  float real = (float) ( (utm.north - DELTA_NORTH) * scale);
		  float img = (float) ((utm.east - DELTA_EAST) * scale);
		   complex z = new complex();
		   z.re = real;
		   z.im = img;
		  int k;
		  for(k = 1; k < 2; k++) {
		     //complex z_ = { real, img };
			  complex z_ = new complex();
			  z_.re = real;
			  z_.im = img;
		    CScal(2*k, z_);
		    CSin(z_);
		    CScal(serie_coeff_proj_mercator_inverse[k], z_);
		    CSub(z_, z);
		  }

		  float lambda_c = (float) LambdaOfUtmZone(utm.zone);
		  lla.lon = (float) (lambda_c + Math.atan (Math.sinh(z.im) / Math.cos(z.re)));
		  float phi_ = (float) (Math.asin (Math.sin(z.re) / Math.cosh(z.im)));
		  float il = isometric_latitude_fast_f(phi_);
		  lla.lat = inverse_isometric_latitude_f(il, (float) E, (float) 1e-8);

		  // copy alt above reference ellipsoid
		  lla.alt = utm.alt;
		}

	
	public static float isometric_latitude_f(float phi, float e) {
		  return (float) (Math.log(Math.tan(M_PI_4 + phi / 2.0)) - e / 2.0 * Math.log((1.0 + e * Math.sin(phi)) / (1.0 - e * Math.sin(phi))));
		}
	public static float isometric_latitude_f(float phi, double e) {
		return (float) (Math.log(Math.tan(M_PI_4 + phi / 2.0)) - e / 2.0 * Math.log((1.0 + e * Math.sin(phi)) / (1.0 - e * Math.sin(phi))));
	}

		public static float isometric_latitude_fast_f(float phi) {
		  return (float) Math.log(Math.tan (M_PI_4 + phi / 2.0));
		}
		
	public static float inverse_isometric_latitude_f(float lat, float e, float epsilon) {
			  float exp_l = (float) Math.exp(lat);
			  float phi0 = (float)( 2 * Math.atan(exp_l) - M_PI_2);
			  float phi_;
			  int max_iter = 3; /* To be sure to return */

			  do {
			    phi_ = phi0;
			    float sin_phi = (float) (e * Math.sin(phi_));
			    phi0 = (float) (2 * Math.atan (Math.pow((1 + sin_phi) / (1. - sin_phi), e/2.) * exp_l) - M_PI_2);
			    max_iter--;
			  } while (max_iter !=0 && Math.abs(phi_ - phi0) > epsilon);
			  return phi0;
			}
	
	public static void enu_of_ecef_point_f(EnuCoor_f enu,LtpDef_f def,EcefCoor_f ecef) {
		  EcefCoor_f delta = new EcefCoor_f();
		  VECT3_DIFF(delta, ecef, def.ecef);
		  MAT33_VECT3_MUL(enu, def.ltp_of_ecef, delta);
		}
	
	public static  void enu_of_lla_point_f(EnuCoor_f enu,  LtpDef_f def,  LlaCoor_f lla) {
		   EcefCoor_f ecef = new EcefCoor_f();
		  ecef_of_lla_f(ecef,lla);
		  enu_of_ecef_point_f(enu,def,ecef);
		}
	
	private static  float a = (float) 6378137.0;           /* earth semimajor axis in meters */
	  private static  float f =(float)(1./298.257223563);    /* reciprocal flattening          */
	public static void ecef_of_lla_f( EcefCoor_f out,  LlaCoor_f in) {

		  // FIXME : make an ellipsoid struct
		 //static  float a = 6378137.0;           /* earth semimajor axis in meters */
		 // static  float f = 1./298.257223563;    /* reciprocal flattening          */
		   float e2 = (float) (2.*f-(f*f));                /* first eccentricity squared     */

		   float sin_lat = (float) Math.sin(in.lat);
		   float cos_lat = (float) Math.cos(in.lat);
		   float sin_lon =(float) Math.sin(in.lon);
		   float cos_lon =(float) Math.cos(in.lon);
		   float chi = (float) Math.sqrt(1. - e2*sin_lat*sin_lat);
		   float a_chi = a / chi;

		  out.x = (a_chi + in.alt) * cos_lat * cos_lon;
		  out.y = (a_chi + in.alt) * cos_lat * sin_lon;
		  out.z = (float)( (a_chi*(1. - e2) + in.alt) * sin_lat);
		}
	
	public static void utm_of_lla_f( UtmCoor_f utm,  LlaCoor_f lla) {
		  float lambda_c =(float) LambdaOfUtmZone(utm.zone);
		  float ll = isometric_latitude_f(lla.lat , E);
		  float dl = lla.lon - lambda_c;
		  float phi_ =(float) Math.asin(Math.sin(dl) / Math.cosh(ll));
		  float ll_ = isometric_latitude_fast_f(phi_);
		  float lambda_ =(float) Math.atan(Math.sinh(ll) / Math.cos(dl));
		  // complex z_ = { lambda_,  ll_ };
		  complex z_ = new complex();
		   z_.re = lambda_;
		   z_.im = ll_;
		  CScal(serie_coeff_proj_mercator[0], z_);
		  int k;
		  for(k = 1; k < 3; k++) {
		    // complex z = { lambda_,  ll_ };
			  complex z =new complex();
			  z.re = lambda_;
			  z.im = ll_;
		    CScal(2*k, z);
		    CSin(z);
		    CScal(serie_coeff_proj_mercator[k], z);
		    CAdd(z, z_);
		  }
		  CScal(N, z_);
		  utm.east = (float) DELTA_EAST + z_.im;
		  utm.north = (float) DELTA_NORTH + z_.re;

		  // copy alt above reference ellipsoid
		  utm.alt = lla.alt;
		}
	
	public static void ned_of_ecef_point_f( NedCoor_f ned,  LtpDef_f def,  EcefCoor_f ecef) {
		   EnuCoor_f enu = new EnuCoor_f();
		  enu_of_ecef_point_f(enu, def, ecef);
		  ENU_OF_TO_NED(ned, enu);
		}

	public static void ned_of_lla_point_f( NedCoor_f ned,  LtpDef_f def,  LlaCoor_f lla) {
		   EcefCoor_f ecef = new EcefCoor_f();
		  ecef_of_lla_f(ecef,lla);
		  ned_of_ecef_point_f(ned,def,ecef);
		}
	
	}
