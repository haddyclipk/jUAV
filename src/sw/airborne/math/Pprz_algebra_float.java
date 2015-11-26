package sw.airborne.math;

import static sw.airborne.math.Pprz_algebra.*;
//import static sw.airborne.subsystems.Parameters.*;
import sw.airborne.subsystems.ins.*;

public class Pprz_algebra_float {
	
	public static final double M_SQRT2 =   1.41421356237309504880;
	
	public static  void FLOAT_VECT3_ZERO(DoubleVect3 _v) {
		VECT3_ASSIGN(_v, 0., 0., 0.);
	}
	
	public static  void FLOAT_VECT3_ZERO(FloatVect3 _v) {
		VECT3_ASSIGN(_v, 0., 0., 0.);
	}
	
	public static  void FLOAT_VECT3_ZERO(NedCoor_f _v) {
		VECT3_ASSIGN(_v, 0., 0., 0.);
	}
	
	public static void VECT3_COPY(FloatVect3 _a,FloatVect3 _b){
		(_a).x = (_b).x;				
	    (_a).y = (_b).y;				
	    (_a).z = (_b).z;
	}
	public static void VECT3_COPY(NedCoor_f _a,NedCoor_f _b){
		(_a).x = (_b).x;				
		(_a).y = (_b).y;				
		(_a).z = (_b).z;
	}
	public static void VECT2_COPY(FloatVect2 _a,FloatVect2 _b){
		(_a).x = (_b).x;				
	    (_a).y = (_b).y;				
	    
	}
	
	public static void  FLOAT_VECT3_CROSS_PRODUCT(DoubleVect3 _vo, DoubleVect3 _v1,DoubleVect3 _v2) {          
	    (_vo).x = (_v1).y*(_v2).z - (_v1).z*(_v2).y;            
	    (_vo).y = (_v1).z*(_v2).x - (_v1).x*(_v2).z;            
	    (_vo).z = (_v1).x*(_v2).y - (_v1).y*(_v2).x;            
	  }

	
	
	public static float FLOAT_VECT2_NORM2(FloatVect2 _v){
		return ((_v).x*(_v).x + (_v).y*(_v).y);
	}
	public static float FLOAT_VECT2_NORM2(NedCoor_f _v){
		return ((_v).x*(_v).x + (_v).y*(_v).y);
	}
	public static float FLOAT_VECT2_NORM2(EnuCoor_f _v){
		return ((_v).x*(_v).x + (_v).y*(_v).y);
	}
	
	private static float a = (float) 6378137.0, f = (float) (1./298.257223563);
	public static void lla_of_ecef_f( LlaCoor_f out,  EcefCoor_f in) {

		  // FIXME : make an ellipsoid 
		 // static  float a = 6378137.0;           /* earth semimajor axis in meters */
		 // static  float f = 1./298.257223563;    /* reciprocal flattening          */
		   float b =(float) (a*(1.-f));                   /* semi-minor axis                */
		   float b2 = b*b;

		   float e2 =(float)( 2.*f-(f*f));                /* first eccentricity squared     */
		   float ep2 =(float)( f*(2.-f)/((1.-f)*(1.-f))); /* second eccentricity squared    */
		   float E2 = a*a - b2;


		   float z2 = in.z*in.z;
		   float r2 = in.x*in.x+in.y*in.y;
		   float r =(float) Math.sqrt(r2);
		   float F =(float) (54.*b2*z2);
		   float G = (r2 + (1-e2)*z2 - e2*E2);
		   float c = (e2*e2*F*r2)/(G*G*G);
		   float s = (float)Math.pow( (1 + c + Math.sqrt(c*c + 2*c)), 1./3.);
		   float s1 = 1+s+1/s;
		   float P = F/(3*s1*s1*G*G);
		   float Q = (float) Math.sqrt(1+2*e2*e2*P);
		   float ro = (float) (-(e2*P*r)/(1+Q) + Math.sqrt((a*a/2)*(1+1/Q) - ((1-e2)*P*z2)/(Q*(1+Q)) - P*r2/2));
		   float tmp = (r - e2*ro)*(r - e2*ro);
		   float U = (float)Math.sqrt( tmp + z2 );
		   float V =(float) Math.sqrt( tmp + (1-e2)*z2 );
		   float zo = (b2*in.z)/(a*V);

		  out.alt = U*(1 - b2/(a*V));
		  out.lat = (float) Math.atan((in.z + ep2*zo)/r);
		  out.lon = (float) Math.atan2(in.y,in.x);

		}
	
	public static void FLOAT_EULERS_OF_RMAT(FloatEulers _e,FloatRMat _rm) {                 
		
		 float dcm00 = (_rm).m[0];                 
		 float dcm01 = (_rm).m[1];                 
		 float dcm02 = (_rm).m[2];                 
		 float dcm12 = (_rm).m[5];                 
		 float dcm22 = (_rm).m[8];                 
		(_e).phi   = (float)Math.atan2( dcm12, dcm22 );                
		(_e).theta = (float)-Math.asin( dcm02 );                   
		(_e).psi   = (float) Math.atan2( dcm01, dcm00 );                
		
	}
	
	public static void FLOAT_EULERS_OF_QUAT(FloatEulers _e,FloatQuat _q) {                  

		float qx2  = (_q).qx*(_q).qx;                 
		float qy2  = (_q).qy*(_q).qy;                 
		float qz2  = (_q).qz*(_q).qz;                 
		float qiqx = (_q).qi*(_q).qx;                 
		float qiqy = (_q).qi*(_q).qy;                 
		float qiqz = (_q).qi*(_q).qz;                 
		float qxqy = (_q).qx*(_q).qy;                 
		float qxqz = (_q).qx*(_q).qz;                 
		float qyqz = (_q).qy*(_q).qz;                 
		float dcm00 = (float)( 1.0 - 2.*(  qy2 +  qz2 ));           
		float dcm01 =  (float)   (  2.*( qxqy + qiqz ));           
		float dcm02 =  (float)    ( 2.*( qxqz - qiqy ));           
		float dcm12 = (float)     ( 2.*( qyqz + qiqx ));           
		float dcm22 = (float)(1.0 - 2.*(  qx2 +  qy2 ));           

		(_e).phi = (float)Math.atan2( dcm12, dcm22 );                  
		(_e).theta = (float)-Math.asin( dcm02 );                   
		(_e).psi =(float) Math.atan2( dcm01, dcm00 );                  


	}
	
	/* C n->b rotation matrix */
	public static void FLOAT_RMAT_OF_QUAT(FloatRMat _rm,FloatQuat _q) {                   
	     float _a = (float) (M_SQRT2*(_q).qi);                   
	     float _b =(float) ( M_SQRT2*(_q).qx);                   
	     float _c = (float) (M_SQRT2*(_q).qy);                   
	     float _d = (float) (M_SQRT2*(_q).qz);                   
	     float a2_1 = _a*_a-1;                     
	     float ab = _a*_b;                     
	     float ac = _a*_c;                     
	     float ad = _a*_d;                     
	     float bc = _b*_c;                     
	     float bd = _b*_d;                     
	     float cd = _c*_d;                     
	   // RMAT_ELMT(_rm, 0, 0) = a2_1+_b*_b;                  
	    _rm.m[0] = a2_1+_b*_b;                  
	    _rm.m[1]= bc+ad;                   
	    _rm.m[2]= bd-ac;                   
	    _rm.m[3]= bc-ad;                   
	    _rm.m[4]= a2_1+_c*_c;                   
	    _rm.m[5]= cd+ab;                   
	    _rm.m[6]=bd+ac;                   
	    _rm.m[7]= cd-ab;                   
	    _rm.m[8]=a2_1+_d*_d;                   
//	    RMAT_ELMT(_rm, 1, 0) = bc-ad;                   
//	    RMAT_ELMT(_rm, 1, 1) = a2_1+_c*_c;                  
//	    RMAT_ELMT(_rm, 1, 2) = cd+ab;                   
//	    RMAT_ELMT(_rm, 2, 0) = bd+ac;                   
//	    RMAT_ELMT(_rm, 2, 1) = cd-ab;                   
//	    RMAT_ELMT(_rm, 2, 2) = a2_1+_d*_d;                  
	  }
	
	public static void FLOAT_RMAT_OF_EULERS(FloatRMat _rm,FloatEulers _e){
		FLOAT_RMAT_OF_EULERS_321(_rm, _e);
	}

	/* C n->b rotation matrix */
	public static void FLOAT_RMAT_OF_EULERS_321(FloatRMat _rm,FloatEulers _e) {             
	                                        
	     float sphi   = (float) Math.sin((_e).phi);                
	     float cphi   = (float) Math.cos((_e).phi);                
	     float stheta = (float) Math.sin((_e).theta);              
	     float ctheta = (float) Math.cos((_e).theta);              
	     float spsi   = (float) Math.sin((_e).psi);                
	     float cpsi   = (float) Math.cos((_e).psi);                
	                                        
	     _rm.m[0] = ctheta*cpsi;                 
	     _rm.m[1] = ctheta*spsi;                 
	     _rm.m[2] = -stheta;                 
	     _rm.m[3]= sphi*stheta*cpsi - cphi*spsi;        
	     _rm.m[4]= sphi*stheta*spsi + cphi*cpsi;        
	     _rm.m[5]= sphi*ctheta;                 
	     _rm.m[6] = cphi*stheta*cpsi + sphi*spsi;        
	     _rm.m[7]= cphi*stheta*spsi - sphi*cpsi;        
	     _rm.m[8]= cphi*ctheta;                 
	                                        
	  }
	
	public static void FLOAT_QUAT_OF_RMAT(FloatQuat _q,FloatRMat _r) {                                    
	     float tr = RMAT_TRACE((_r));                                  
	    if (tr > 0) {                                                       
	       float two_qi =(float) Math.sqrt(1.+tr);                                
	       float four_qi = (float)(2. * two_qi);                                
	      (_q).qi = (float) (0.5 * two_qi);                                           
	      (_q).qx =  (_r.m[5] - _r.m[7])/four_qi;//(RMAT_ELMT((_r), 1, 2)-RMAT_ELMT((_r), 2, 1))/four_qi; 
	      (_q).qy =  (_r.m[6] -_r.m[2])/four_qi;//(RMAT_ELMT((_r), 2, 0)-RMAT_ELMT((_r), 0, 2))/four_qi; 
	      (_q).qz =  (_r.m[1] - _r.m[3])/four_qi;;//(RMAT_ELMT((_r), 0, 1)-RMAT_ELMT((_r), 1, 0))/four_qi; 
	      /*printf("tr > 0n");*/                                           
	    }                                                                   
	    else {                                                              
	      if (_r.m[0] > _r.m[4] &&              
	    		  _r.m[0] > _r.m[8]) {              
	         float two_qx = (float) Math.sqrt(_r.m[0] -_r.m[4] 
	                                   -_r.m[8] + 1);         
	         float four_qx =(float) (2. * two_qx);                             
	        (_q).qi = (_r.m[5]-_r.m[7])/four_qx; 
	        (_q).qx = (float)(0.5 * two_qx);                                         
	        (_q).qy = (_r.m[1] +_r.m[3])/four_qx; 
	        (_q).qz = (_r.m[6]+_r.m[2])/four_qx; 
	        /*printf("m00 largestn");*/                                    
	      }                                                                 
	      else if (_r.m[4] > _r.m[8]) {         
	         float two_qy =                                            
	          (float) Math.sqrt(_r.m[4] -_r.m[0] - _r.m[8] + 1); 
	         float four_qy =(float) ( 2. * two_qy);                              
	        (_q).qi = (_r.m[6] - _r.m[2])/four_qy; 
	        (_q).qx = (_r.m[1] + _r.m[3])/four_qy; 
	        (_q).qy = (float) (0.5 * two_qy);                                         
	        (_q).qz = (float) (_r.m[5] + _r.m[7])/four_qy; 
	        /*printf("m11 largestn");*/                                    
	      }                                                                 
	      else {                                                            
	         float two_qz =                                            
	         (float) Math.sqrt(_r.m[8] - _r.m[0] - _r.m[4] + 1); 
	         float four_qz =(float) (2. * two_qz);                              
	        (_q).qi = (_r.m[1]- _r.m[3])/four_qz; 
	        (_q).qx = (_r.m[6]+ _r.m[2])/four_qz; 
	        (_q).qy = (_r.m[5]+ _r.m[7])/four_qz; 
	        (_q).qz = (float) (0.5 * two_qz);                                         
	        /*printf("m22 largestn");*/                                    
	      }                                                                 
	    }                                                                   
	  }
	
	public static void FLOAT_QUAT_OF_EULERS(FloatQuat _q,FloatEulers _e) {                  

		float phi2   =(float)( (_e).phi/2.0);                  
		float theta2 =(float)( (_e).theta/2.0);                
		float psi2   =(float)( (_e).psi/2.0);                  

		float s_phi2   = (float)Math.sin( phi2 );                
		float c_phi2   =(float) Math.cos( phi2 );                
		float s_theta2 =(float) Math.sin( theta2 );              
		float c_theta2 = (float)Math.cos( theta2 );              
		float s_psi2   = (float)Math.sin( psi2 );                
		float c_psi2   = (float)Math.cos( psi2 );                

		(_q).qi =  c_phi2 * c_theta2 * c_psi2 + s_phi2 * s_theta2 * s_psi2; 
		(_q).qx = -c_phi2 * s_theta2 * s_psi2 + s_phi2 * c_theta2 * c_psi2; 
		(_q).qy =  c_phi2 * s_theta2 * c_psi2 + s_phi2 * c_theta2 * s_psi2; 
		(_q).qz =  c_phi2 * c_theta2 * s_psi2 - s_phi2 * s_theta2 * c_psi2; 

	}
	
	public static void FLOAT_VECT3_NORMALIZE(FloatVect3 _v){
		float n = FLOAT_VECT3_NORM(_v);   
	    FLOAT_VECT3_SMUL(_v, _v, 1./n);     
	  
	}
	
	public static float FLOAT_VECT3_NORM(FloatVect3 _v) {
		return (float) Math.sqrt(FLOAT_VECT3_NORM2(_v));
		}
	
	public static float FLOAT_VECT3_NORM2(FloatVect3 _v){
		return (_v).x*(_v).x + (_v).y*(_v).y + (_v).z*(_v).z;
		
	}

	public static void FLOAT_VECT3_SMUL(FloatVect3 _vo, FloatVect3 _vi, double _s){
		VECT3_SMUL(_vo, _vi, _s);
	}
	public static void FLOAT_VECT3_SMUL(NedCoor_f _vo, NedCoor_f _vi, double _s){
		VECT3_SMUL(_vo, _vi, _s);
	}

	public static void FLOAT_QUAT_NORMALIZE(FloatQuat _q) {        
	    float qnorm = FLOAT_QUAT_NORM(_q);    
	    if (qnorm > FLT_MIN) {                
	      (_q).qi = (_q).qi / qnorm;          
	      (_q).qx = (_q).qx / qnorm;          
	      (_q).qy = (_q).qy / qnorm;          
	      (_q).qz = (_q).qz / qnorm;          
	    }                                     
	  }
	
	public static float FLOAT_QUAT_NORM(FloatQuat _q) {
		return (float) Math.sqrt(SQUARE((_q).qi) + SQUARE((_q).qx)+SQUARE((_q).qy) + SQUARE((_q).qz));
    }
	
	public static void FLOAT_RMAT_VECT3_TRANSP_MUL(FloatVect3 _vout,
			FloatRMat _rmat, FloatVect3 _vin){
		RMAT_VECT3_TRANSP_MUL(_vout, _rmat, _vin);
		
	}
	
	 public static void FLOAT_QUAT_COMP_NORM_SHORTEST(FloatQuat _a2c, FloatQuat _a2b, FloatQuat _b2c) {       
		    FLOAT_QUAT_COMP(_a2c, _a2b, _b2c);                  
		    FLOAT_QUAT_WRAP_SHORTEST(_a2c);                 
		    FLOAT_QUAT_NORMALIZE(_a2c);                     
		  }

	public static void FLOAT_QUAT_WRAP_SHORTEST(FloatQuat _q) {
		// TODO Auto-generated method stub
		
		 if ((_q).qi < 0.)                                   
	      QUAT_EXPLEMENTARY(_q,_q);                     
		
	}

	public static void FLOAT_QUAT_COMP(FloatQuat _a2c, FloatQuat _a2b,
			FloatQuat _b2c) {
		// TODO Auto-generated method stub
		
		(_a2c).qi = (_a2b).qi*(_b2c).qi - (_a2b).qx*(_b2c).qx - (_a2b).qy*(_b2c).qy - (_a2b).qz*(_b2c).qz; 
	    (_a2c).qx = (_a2b).qi*(_b2c).qx + (_a2b).qx*(_b2c).qi + (_a2b).qy*(_b2c).qz - (_a2b).qz*(_b2c).qy; 
	    (_a2c).qy = (_a2b).qi*(_b2c).qy - (_a2b).qx*(_b2c).qz + (_a2b).qy*(_b2c).qi + (_a2b).qz*(_b2c).qx; 
	    (_a2c).qz = (_a2b).qi*(_b2c).qz + (_a2b).qx*(_b2c).qy - (_a2b).qy*(_b2c).qx + (_a2b).qz*(_b2c).qi; 
		
	}
	
	public static void QUAT_EXPLEMENTARY(FloatQuat b, FloatQuat a) {    
	    (b).qi = -(a).qi;               
	    (b).qx = -(a).qx;               
	    (b).qy = -(a).qy;               
	    (b).qz = -(a).qz;               
	  }
	
	public static void FLOAT_QUAT_ZERO(FloatQuat _q){
		QUAT_ASSIGN(_q, 1., 0., 0., 0.);
	}
	
	public static void FLOAT_RATES_ZERO(FloatRates _r) {          
	    RATES_ASSIGN(_r, 0., 0., 0.);       
	  }
	
	public static void FLOAT_QUAT_EXTRACT(FloatVect3 _vo, FloatQuat _qi){ 
		QUAT_EXTRACT_Q(_vo, _qi);
		
	}
	
	public static float FLOAT_VECT3_DOT_PRODUCT(FloatVect3 _v1, FloatVect3 _v2){
		return (_v1).x*(_v2).x + (_v1).y*(_v2).y + (_v1).z*(_v2).z;
	}
	
	public static void FLOAT_VECT3_CROSS_PRODUCT(FloatVect3 _vo, FloatVect3 _v1, FloatVect3 _v2){
		(_vo).x = (_v1).y*(_v2).z - (_v1).z*(_v2).y;            
	    (_vo).y = (_v1).z*(_v2).x - (_v1).x*(_v2).z;            
	    (_vo).z = (_v1).x*(_v2).y - (_v1).y*(_v2).x; 
	}
	
	public static void FLOAT_VECT3_CROSS_PRODUCT(NedCoor_f _vo, FloatVect3 _v1, FloatVect3 _v2){
		(_vo).x = (_v1).y*(_v2).z - (_v1).z*(_v2).y;            
	    (_vo).y = (_v1).z*(_v2).x - (_v1).x*(_v2).z;            
	    (_vo).z = (_v1).x*(_v2).y - (_v1).y*(_v2).x; 
	}
	
	public static void FLOAT_VECT3_ADD(FloatVect3 _a, FloatVect3 _b){
		VECT3_ADD(_a, _b);
	}
	
	public static void FLOAT_VECT3_ADD(NedCoor_f _a, FloatVect3 _b){
		VECT3_ADD(_a, _b);
	}
	
	public static void FLOAT_QUAT_RMAT_N2B(FloatVect3 _n2b, FloatQuat _qi, FloatVect3 _vi){    
        FloatQuat quatinv = new FloatQuat();                     
        FloatVect3 quat3 = new FloatVect3();
        FloatVect3 v1 = new FloatVect3();
        FloatVect3 v2 = new FloatVect3();              
        float qi;                                     
        
        FLOAT_QUAT_INVERT(quatinv, _qi);              
        FLOAT_QUAT_NORMALIZE(quatinv);                
        
        FLOAT_QUAT_EXTRACT(quat3, quatinv);           
        qi = - FLOAT_VECT3_DOT_PRODUCT(quat3, _vi);   
        FLOAT_VECT3_CROSS_PRODUCT(v1, quat3, _vi);    
        FLOAT_VECT3_SMUL(v2, _vi, (quatinv.qi)) ;     
        FLOAT_VECT3_ADD(v2, v1);                      
        
        FLOAT_QUAT_EXTRACT(quat3, _qi);               
        FLOAT_VECT3_CROSS_PRODUCT(_n2b, v2, quat3);   
        FLOAT_VECT3_SMUL(v1, v2, (_qi).qi);           
        FLOAT_VECT3_ADD(_n2b,v1);                     
        FLOAT_VECT3_SMUL(v1, quat3, qi);              
        FLOAT_VECT3_ADD(_n2b,v1);                     
}
	
	public static void FLOAT_QUAT_RMAT_N2B(NedCoor_f _n2b, FloatQuat _qi, FloatVect3 _vi){    
        FloatQuat quatinv = new FloatQuat();                     
        FloatVect3 quat3 = new FloatVect3();
        FloatVect3 v1 = new FloatVect3();
        FloatVect3 v2 = new FloatVect3();              
        float qi;                                     
        
        FLOAT_QUAT_INVERT(quatinv, _qi);              
        FLOAT_QUAT_NORMALIZE(quatinv);                
        
        FLOAT_QUAT_EXTRACT(quat3, quatinv);           
        qi = - FLOAT_VECT3_DOT_PRODUCT(quat3, _vi);   
        FLOAT_VECT3_CROSS_PRODUCT(v1, quat3, _vi);    
        FLOAT_VECT3_SMUL(v2, _vi, (quatinv.qi)) ;     
        FLOAT_VECT3_ADD(v2, v1);                      
        
        FLOAT_QUAT_EXTRACT(quat3, _qi);               
        FLOAT_VECT3_CROSS_PRODUCT(_n2b, v2, quat3);   
        FLOAT_VECT3_SMUL(v1, v2, (_qi).qi);           
        FLOAT_VECT3_ADD(_n2b,v1);                     
        FLOAT_VECT3_SMUL(v1, quat3, qi);              
        FLOAT_VECT3_ADD(_n2b,v1);                     
}
	
	public static void FLOAT_QUAT_RMAT_B2N(FloatVect3 _b2n, FloatQuat _qi, FloatVect3 _vi){  
        FloatQuat _quatinv = new FloatQuat();                
        FLOAT_QUAT_INVERT(_quatinv, _qi);         
        FLOAT_QUAT_NORMALIZE(_quatinv);           
        FLOAT_QUAT_RMAT_N2B(_b2n, _quatinv, _vi); 
        
	}
	
	public static void FLOAT_QUAT_RMAT_B2N(NedCoor_f _b2n, FloatQuat _qi, FloatVect3 _vi){  
        FloatQuat _quatinv = new FloatQuat();                
        FLOAT_QUAT_INVERT(_quatinv, _qi);         
        FLOAT_QUAT_NORMALIZE(_quatinv);           
        FLOAT_QUAT_RMAT_N2B(_b2n, _quatinv, _vi); 
        
	}
	
	public static void FLOAT_QUAT_INVERT(FloatQuat _qo, FloatQuat _qi) 
	{
		QUAT_INVERT(_qo, _qi);
		}
	
	public static void FLOAT_VECT3_DIFF(FloatVect3 _c, FloatVect3 _a, FloatVect3 _b){
		VECT3_DIFF(_c, _a, _b);
	}
	
	public static void FLOAT_VECT3_DIFF(FloatVect3 _c, NedCoor_f _a, NedCoor_f _b){
		VECT3_DIFF(_c, _a, _b);
	}
	
	public static void FLOAT_VECT3_COPY(FloatVect3 _a, FloatVect3 _b){
		VECT3_COPY(_a, _b);
	}
	
	public static void float_vect_zero(inv_state a, int n) {
		a = new inv_state();
	}
	
	public static void FLOAT_VECT3_ASSIGN(FloatVect3 _a, float _x, float _y, float _z){ 
		VECT3_ASSIGN(_a, _x, _y, _z);
	}
	
	public static void FLOAT_QUAT_ASSIGN(FloatQuat _qi, float _i, float _x, float _y, float _z){ 
		QUAT_ASSIGN(_qi, _i, _x, _y, _z);
	}
	
	public static void FLOAT_QUAT_SMUL(FloatQuat _qo, FloatQuat _qi, double _s) {
		QUAT_SMUL(_qo, _qi, _s);
	}
	
	public static void FLOAT_QUAT_VMUL_RIGHT(FloatQuat _mright, FloatQuat _qi, FloatVect3 _vi){ 
        FloatVect3 quat3 = new FloatVect3();
        FloatVect3 v1 = new FloatVect3();
        FloatVect3 v2 = new FloatVect3();
                  
        float qi;                                     
          
        FLOAT_QUAT_EXTRACT(quat3, _qi);               
        qi = - FLOAT_VECT3_DOT_PRODUCT(_vi, quat3);   
        FLOAT_VECT3_CROSS_PRODUCT(v1, _vi, quat3);    
        FLOAT_VECT3_SMUL(v2, _vi, (_qi.qi));          
        FLOAT_VECT3_ADD(v2, v1);                      
        FLOAT_QUAT_ASSIGN(_mright, qi, v2.x, v2.y, v2.z);
	}
	
	public static void FLOAT_QUAT_VMUL_LEFT(FloatQuat _mleft, FloatQuat _qi, FloatVect3 _vi){ 
        FloatVect3 quat3 = new FloatVect3();
        FloatVect3 v1 = new FloatVect3();
        FloatVect3 v2 = new FloatVect3();
               
        float qi;                                   
        FLOAT_QUAT_EXTRACT(quat3, _qi);             
        qi = - FLOAT_VECT3_DOT_PRODUCT(quat3, _vi); 
        FLOAT_VECT3_CROSS_PRODUCT(v1, quat3, _vi);  
        FLOAT_VECT3_SMUL(v2, _vi, (_qi.qi));        
        FLOAT_VECT3_ADD(v2, v1);                    
        FLOAT_QUAT_ASSIGN(_mleft, qi, v2.x, v2.y, v2.z);
	}
	
	public static void FLOAT_QUAT_ADD(FloatQuat _qo, FloatQuat _qi) {QUAT_ADD(_qo, _qi);}
	
	public static void FLOAT_VECT3_SUM(NedCoor_f _a, NedCoor_f _b, FloatVect3 _c) {
		// TODO Auto-generated method stub
		VECT3_SUM(_a, _b, _c);
		
	}
	
	//
	public static void float_vect_smul(inv_state o, inv_state a, double s,
			int n) {
		// TODO Auto-generated method stub
		o = a;//*s;???
		
	}
	
	public static void float_vect_add(inv_state a, inv_state b, double n) {
		// TODO Auto-generated method stub
		a = b;//a+b;
		
	}
	
	public static void float_vect_sum(inv_state o, inv_state a, inv_state s,
			int n) {
		// TODO Auto-generated method stub
		o = a;//*s;???
		
	}
}