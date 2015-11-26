package sw.airborne.math;
import static sw.airborne.math.Pprz_algebra_int.*;

public class Pprz_algebra {
	
	public static float FLT_MIN = 1.1754943508222875e-38F;
	
	public static float RMAT_TRACE(FloatRMat rm){
		return (rm.m[0] + rm.m[4] + rm.m[8]);
	}
	public static long RMAT_TRACE(Int32RMat rm){
		return (rm.m[0] + rm.m[4] + rm.m[8]);
	}
	
	public static int SQUARE(int _a){
		return ((_a)*(_a));
	}
	
	public static float SQUARE(float _a){
		return ((_a)*(_a));
	}

	//
	//
	// Vector algebra
	//
	//


	/*
	 * Dimension 2 vectors
	 */

	/* a =  {x, y} */
	public static void ECEF_BFP_OF_REAL(Int32Vect3 _a, FloatVect3 _b){
	(_b).x = (float) ((float)((_a).x)/1e2);          
	(_b).y = (float) ((float)((_a).y)/1e2); 	
	(_b).z = (float) ((float)((_a).z)/1e2); }
	public static void ECEF_FLOAT_OF_BFP(FloatVect3 _a,Int32Vect3 _b){
		(_b).x = (int) ((float)((_a).x)/1e2);          
		(_b).y = (int) ((float)((_a).y)/1e2); 	
		(_b).z = (int) ((float)((_a).z)/1e2); }
	
	public static void VECT2_ASSIGN(DoubleVect2 _a,int _x,int _y) {		
	    (_a).x = (_x);				
	    (_a).y = (_y);				
	  }
	public static void VECT2_ASSIGN(Int32Vect2 _a,int _x,int _y) {		
		(_a).x =  (_x);				
		(_a).y =  (_y);				
	}
	public static void VECT2_ASSIGN(Int32Vect2 _a,long _x,long _y) {		
		(_a).x =  (_x);				
		(_a).y =  (_y);				
	}
//	public static void VECT2_ASSIGN(Int32Vect2 _a,long _x,long _y) {		
//		(_a).x = (_x);				
//		(_a).y = (_y);				
//	}

	/* a = b */
	public static void VECT2_COPY(DoubleVect2 _a,DoubleVect2 _b) {			
	    (_a).x = (_b).x;				
	    (_a).y = (_b).y;				
	  }
	public static void VECT2_COPY(Int32Vect2 _a,Int32Vect2 _b) {			
		(_a).x = (_b).x;				
		(_a).y = (_b).y;				
	}
	public static void VECT2_COPY(Int32Vect2 _a,NedCoor_i _b) {			
		(_a).x = (_b).x;				
		(_a).y = (_b).y;				
	}
	public static void VECT2_COPY(EnuCoor_i _a, EnuCoor_i _b) {			
		(_a).x = (_b).x;				
		(_a).y = (_b).y;				
	}

	/* a += b */
	public static void VECT2_ADD(DoubleVect2 _a,DoubleVect2 _b) {			
	    (_a).x += (_b).x;				
	    (_a).y += (_b).y;				
	  }
	public static void VECT2_ADD(Int32Vect2 _a,Int32Vect2 _b) {			
		(_a).x += (_b).x;				
		(_a).y += (_b).y;				
	}
	public static void VECT2_ADD(Int64Vect2 _a,Int32Vect2 _b) {			
		(_a).x += (_b).x;				
		(_a).y += (_b).y;				
	}

	/* a -= b */
	public static void VECT2_SUB(DoubleVect2 _a, DoubleVect2 _b) {			
	    (_a).x -= (_b).x;				
	    (_a).y -= (_b).y;				
	  }

	/* c = a + b */
	public static void VECT2_SUM(DoubleVect2 _c, DoubleVect2 _a, DoubleVect2 _b) {			
	    (_c).x = (_a).x + (_b).x;			
	    (_c).y = (_a).y + (_b).y;			
	  }
	public static void VECT2_SUM(EnuCoor_i _c, Int32Vect2 _a, EnuCoor_i _b) {			
	    (_c).x = (_a).x + (_b).x;			
	    (_c).y = (_a).y + (_b).y;			
	  }
	public static void VECT2_SUM(EnuCoor_i _c, EnuCoor_i _a, Int32Vect2 _b) {			
		(_c).x = (_a).x + (_b).x;			
		(_c).y = (_a).y + (_b).y;			
	}
	public static void VECT2_SUM(Int32Vect2 _c, EnuCoor_i _a, Int32Vect2 _b) {			
		(_c).x = (_a).x + (_b).x;			
		(_c).y = (_a).y + (_b).y;			
	}
	public static void VECT2_SUM(Int32Vect2 _c, Int32Vect2 _a, Int32Vect2 _b) {			
		(_c).x = (_a).x + (_b).x;			
		(_c).y = (_a).y + (_b).y;			
	}

	/* c = a - b */
	public static void VECT2_DIFF(DoubleVect2 _c, DoubleVect2 _a, DoubleVect2 _b) {                
	    (_c).x = (_a).x - (_b).x;			
	    (_c).y = (_a).y - (_b).y;			
	  }
	public static void VECT2_DIFF(Int32Vect2 _c, EnuCoor_i _a, EnuCoor_i _b) {                
	    (_c).x = (_a).x - (_b).x;			
	    (_c).y = (_a).y - (_b).y;			
	  }
	public static void VECT2_DIFF(Int32Vect2 _c, EnuCoor_i _a, Int32Vect2 _b) {                
		(_c).x = (_a).x - (_b).x;			
		(_c).y = (_a).y - (_b).y;			
	}
	public static void VECT2_DIFF(Int32Vect2 _c, Int32Vect2 _a, NedCoor_i _b) {                
	    (_c).x = (_a).x - (_b).x;			
	    (_c).y = (_a).y - (_b).y;			
	  }
	public static void VECT2_DIFF(FloatVect2 _c, FloatVect2 _a, EnuCoor_f _b) {                
	    (_c).x = (_a).x - (_b).x;			
	    (_c).y = (_a).y - (_b).y;			
	  }
	public static void VECT2_DIFF(Int32Vect2 _c, Int32Vect2 _a, Int32Vect2 _b) {                
		(_c).x = (_a).x - (_b).x;			
		(_c).y = (_a).y - (_b).y;			
	}

	/* _vo = _vi * _s */
	public static void VECT2_SMUL(DoubleVect2 _vo, DoubleVect2 _vi, int _s) {		
	    (_vo).x =  (_vi).x * (_s);			
	    (_vo).y =  (_vi).y * (_s);			
	  }
	public static void VECT2_SMUL(Int32Vect2 _vo, Int32Vect2 _vi, int _s) {		
		(_vo).x =  (_vi).x * (_s);			
		(_vo).y =  (_vi).y * (_s);			
	}
	public static void VECT2_SMUL(Int32Vect2 _vo, Int32Vect2 _vi, long _s) {		
		(_vo).x =  (_vi).x * (_s);			
		(_vo).y =  (_vi).y * (_s);			
	}
	public static void VECT2_SMUL(Int32Vect2 _vo, EnuCoor_i _vi, int _s) {		
		(_vo).x =  (_vi).x * (_s);			
		(_vo).y =  (_vi).y * (_s);			
	}
	public static void VECT2_SMUL(Int32Vect2 _vo, EnuCoor_i _vi, double _s) {		
		(_vo).x =(int) ( (_vi).x * (_s));			
		(_vo).y =  (int)((_vi).y * (_s));			
	}

	/* _vo =  _vi / _s */
	public static void VECT2_SDIV(Int32Vect2 _vo,Int32Vect2 _vi,int _s) {		
	    (_vo).x =  (_vi).x / (_s);			
	    (_vo).y =  (_vi).y / (_s);			
	  }
	/* _vo =  _vi / _s */
	public static void VECT2_SDIV(Int32Vect2 _vo,Int32Vect2 _vi,long _s) {		
		(_vo).x =  (_vi).x / (_s);			
		(_vo).y =  (_vi).y / (_s);			
	}

	/* _v = Bound(_v, _min, _max) */
	public static void VECT2_STRIM(DoubleVect2 _v,double _min,double _max) {					
	    (_v).x = (_v).x < _min ? _min :( (_v).x > _max ? _max : (_v).x);	
	    (_v).y = (_v).y < _min ? _min : (_v).y > _max ? _max : (_v).y;	
	  }
	public static void VECT2_STRIM(Int32Vect2 _v,int _min,int _max) {					
	    (_v).x = (_v).x < _min ? _min :( (_v).x > _max ? _max : (_v).x);	
	    (_v).y = (_v).y < _min ? _min : (_v).y > _max ? _max : (_v).y;	
	  }
	

	/* _vo=v1*v2 */
	public static void VECT2_DOT_PRODUCT(int _so,DoubleVect2 _v1,DoubleVect2 _v2) {                        
	    (_so) = (int)((_v1).x*(_v2).x + (_v1).y*(_v2).y) ;//+ (_v1).z*(_v2).z;  
	}

	/*
	 * Dimension 3 vectors
	 */

	/* a =  {x, y, z} */
	public static void VECT3_ASSIGN(DoubleVect3 _a,int _x,int _y,int _z) {		
	    (_a).x = (_x);				
	    (_a).y = (_y);				
	    (_a).z = (_z);				
	  }
	public static void VECT3_ASSIGN(DoubleVect3 _a,double _x,double _y,double _z) {		
		(_a).x = (_x);				
		(_a).y = (_y);				
		(_a).z = (_z);				
	}
	
	public static void VECT3_ASSIGN(FloatVect3 _a,double _x,double _y,double _z) {		
		(_a).x = (float) (_x);				
		(_a).y = (float) (_y);				
		(_a).z = (float) (_z);				
	}
	
	public static void VECT3_ASSIGN(NedCoor_f _a,double _x,double _y,double _z) {		
		(_a).x = (float) (_x);				
		(_a).y = (float) (_y);				
		(_a).z = (float) (_z);				
	}
	
	public static void VECT3_ASSIGN(Int32Vect3 _a,int _x,int _y,int _z) {		
		(_a).x = (_x);				
		(_a).y = (_y);				
		(_a).z = (_z);				
	}
	public static void VECT3_ASSIGN(Int32Vect3 _a,long _x,long _y,long _z) {		
		(_a).x = (_x);				
		(_a).y = (_y);				
		(_a).z = (_z);				
	}
	public static void VECT3_ASSIGN(FloatVect3 _a,float _x,float _y,float _z) {		
		(_a).x = (_x);				
		(_a).y = (_y);				
		(_a).z = (_z);				
	}

	/* a = a * b */

	public static void VECT3_MUL(DoubleVect3 _v1,DoubleVect3 _v2){ 
	  (_v1).x = (_v1).x * (_v2).x; 
	  (_v1).y = (_v1).y * (_v2).y; 
	  (_v1).z = (_v1).z * (_v2).z; 
	}

	/* a =  {abs(x), abs(y), abs(z)} */
	public static void VECT3_ASSIGN_ABS(DoubleVect3 _a,int  _x,int _y, int _z) {      
	    (_a).x =Math.abs(_x);                              
	    (_a).y = Math.abs(_y);                              
	    (_a).z = Math.abs(_z);                              
	  }

	/* a = b */
	public static void VECT3_COPY(DoubleVect3 _a,DoubleVect3 _b) {        
	    (_a).x = (_b).x;				
	    (_a).y = (_b).y;				
	    (_a).z = (_b).z;				
	  }
	public static void VECT3_COPY(Int32Vect3 _a,Int32Vect3 _b) {        
	    (_a).x = (_b).x;				
	    (_a).y = (_b).y;				
	    (_a).z = (_b).z;				
	  }

	public static void VECT3_COPY(NedCoor_f _a,NedCoor_d _b) {        
	    (_a).x = (float) (_b).x;				
	    (_a).y = (float) (_b).y;				
	    (_a).z = (float) (_b).z;				
	  }
	public static void VECT3_COPY(EcefCoor_i _a,EcefCoor_i _b) {        
		(_a).x = (_b).x;				
		(_a).y = (_b).y;				
		(_a).z = (_b).z;				
	}

	/* a += b */
	public static void VECT3_ADD(DoubleVect3 _a,DoubleVect3 _b) {			
	    (_a).x += (_b).x;				
	    (_a).y += (_b).y;				
	    (_a).z += (_b).z;				
	  }

	public static void VECT3_ADD(FloatVect3 _a,FloatVect3 _b) {			
	    (_a).x += (_b).x;				
	    (_a).y += (_b).y;				
	    (_a).z += (_b).z;				
	  }
	
	public static void VECT3_ADD(NedCoor_f _a,FloatVect3 _b) {			
	    (_a).x += (_b).x;				
	    (_a).y += (_b).y;				
	    (_a).z += (_b).z;				
	  }
	
	public static void VECT3_ADD(EcefCoor_i _a, EcefCoor_i _b) {			
		(_a).x += (_b).x;				
		(_a).y += (_b).y;				
		(_a).z += (_b).z;				
	}

	/* a -= b */
	public static void VECT3_SUB(DoubleVect3 _a,DoubleVect3 _b) {			
	    (_a).x -= (_b).x;				
	    (_a).y -= (_b).y;				
	    (_a).z -= (_b).z;				
	  }

	/* c = a + b */
	public static void VECT3_SUM(DoubleVect3 _c,DoubleVect3 _a,DoubleVect3 _b) {                 
	    (_c).x = (_a).x + (_b).x;			
	    (_c).y = (_a).y + (_b).y;			
	    (_c).z = (_a).z + (_b).z;			
	  }
	
	public static void VECT3_SUM(NedCoor_f _c,NedCoor_f _a,FloatVect3 _b) {                 
	    (_c).x = (_a).x + (_b).x;			
	    (_c).y = (_a).y + (_b).y;			
	    (_c).z = (_a).z + (_b).z;			
	  }

	/* a += b*s */
	public static void VECT3_ADD_SCALED(DoubleVect3 _a,DoubleVect3 _b,int _s) {			
	    (_a).x += ((_b).x * (_s));				
	    (_a).y += ((_b).y * (_s));				
	    (_a).z += ((_b).z * (_s));				
	  }

	/* c = a + _s * b */
	public static void VECT3_SUM_SCALED(DoubleVect3 _c,DoubleVect3 _a,DoubleVect3 _b,int _s) {		
	    (_c).x = (_a).x + (_s)*(_b).x;			
	    (_c).y = (_a).y + (_s)*(_b).y;			
	    (_c).z = (_a).z + (_s)*(_b).z;			
	  }

	/* c = a - b */
	public static void VECT3_DIFF(DoubleVect3 _c,DoubleVect3 _a,DoubleVect3 _b) {                
	    (_c).x = (_a).x - (_b).x;			
	    (_c).y = (_a).y - (_b).y;			
	    (_c).z = (_a).z - (_b).z;			
	  }
	
	public static void VECT3_DIFF(FloatVect3 _c,FloatVect3 _a,FloatVect3 _b) {                
	    (_c).x = (_a).x - (_b).x;			
	    (_c).y = (_a).y - (_b).y;			
	    (_c).z = (_a).z - (_b).z;			
	  }
	
	public static void VECT3_DIFF(FloatVect3 _c,NedCoor_f _a,NedCoor_f _b) {                
	    (_c).x = (_a).x - (_b).x;			
	    (_c).y = (_a).y - (_b).y;			
	    (_c).z = (_a).z - (_b).z;			
	  }
	
	/* c = a - b */
	public static void VECT3_DIFF(EcefCoor_i _c,EcefCoor_i _a,EcefCoor_i _b) {                
		(_c).x = (_a).x - (_b).x;			
		(_c).y = (_a).y - (_b).y;			
		(_c).z = (_a).z - (_b).z;			
	}
	public static void VECT3_DIFF(EcefCoor_f _c,EcefCoor_f _a,EcefCoor_f _b) {                
		(_c).x = (_a).x - (_b).x;			
		(_c).y = (_a).y - (_b).y;			
		(_c).z = (_a).z - (_b).z;			
	}

	/* _vo = _vi * _s */
	public static void VECT3_SMUL(DoubleVect3 _vo,DoubleVect3 _vi,int _s) {			
	    (_vo).x =  (_vi).x * (_s);				
	    (_vo).y =  (_vi).y * (_s);				
	    (_vo).z =  (_vi).z * (_s);				
	  }
	
	public static void VECT3_SMUL(FloatVect3 _vo,FloatVect3 _vi,double _s) {			
	    (_vo).x =  (float) ((_vi).x * (_s));				
	    (_vo).y =  (float) ((_vi).y * (_s));				
	    (_vo).z =  (float) ((_vi).z * (_s));				
	  }

	public static void VECT3_SMUL(NedCoor_f _vo, NedCoor_f _vi,double _s) {			
	    (_vo).x =  (float) ((_vi).x * (_s));				
	    (_vo).y =  (float) ((_vi).y * (_s));				
	    (_vo).z =  (float) ((_vi).z * (_s));				
	  }

	
	public static void VECT3_SMUL(EnuCoor_i _vo,EnuCoor_i _vi,int _s) {			
	    (_vo).x =  (_vi).x * (_s);				
	    (_vo).y =  (_vi).y * (_s);				
	    (_vo).z =  (_vi).z * (_s);				
	  }

	/* _vo =  _vi / _s */
	public static void VECT3_SDIV(DoubleVect3 _vo,DoubleVect3 _vi,int _s) {			
	    (_vo).x =  (_vi).x / (_s);				
	    (_vo).y =  (_vi).y / (_s);				
	    (_vo).z =  (_vi).z / (_s);				
	  }
	public static void VECT3_SDIV(Int32Vect3 _vo,Int16Vect3 _vi,int _s) {			
		(_vo).x =  (_vi).x / (_s);				
		(_vo).y =  (_vi).y / (_s);				
		(_vo).z =  (_vi).z / (_s);				
	}
	public static void VECT3_SDIV(EnuCoor_i _vo,EnuCoor_i _vi,int _s) {			
		(_vo).x =  (_vi).x / (_s);				
		(_vo).y =  (_vi).y / (_s);				
		(_vo).z =  (_vi).z / (_s);				
	}

	/* _v = Bound(_v, _min, _max) */
	public static void VECT3_STRIM(DoubleVect3 _v,int _min, int _max) {					
	    (_v).x = (_v).x < _min ? _min : (_v).x > _max ? _max : (_v).x;	
	    (_v).y = (_v).y < _min ? _min : (_v).y > _max ? _max : (_v).y;	
	    (_v).z = (_v).z < _min ? _min : (_v).z > _max ? _max : (_v).z;	
	  }

	/*  */
	public static void VECT3_EW_DIV(DoubleVect3 _vo,DoubleVect3 _va,DoubleVect3 _vb) {				
	    (_vo).x =  (_va).x / (_vb).x;				
	    (_vo).y =  (_va).y / (_vb).y;				
	    (_vo).z =  (_va).z / (_vb).z;				
	  }

	/*  */
	public static void VECT3_EW_MUL(DoubleVect3 _vo,DoubleVect3 _va,DoubleVect3 _vb) {				
	    (_vo).x =  (_va).x * (_vb).x;				
	    (_vo).y =  (_va).y * (_vb).y;				
	    (_vo).z =  (_va).z * (_vb).z;				
	  }

	/*  */
	public static void VECT3_BOUND_CUBE(DoubleVect3 _v,int _min,int _max) {				
	    if ((_v).x > (_max)) (_v).x = (_max); else if ((_v).x < (_min)) (_v).x = (_min); 
	    if ((_v).y > (_max)) (_v).y = (_max); else if ((_v).y < (_min)) (_v).y = (_min); 
	    if ((_v).z > (_max)) (_v).z = (_max); else if ((_v).z < (_min)) (_v).z = (_min); 
	  }

	/*  */
	public static void VECT3_BOUND_BOX(DoubleVect3 _v,DoubleVect3 _v_min,DoubleVect3 _v_max) {				
	    if ((_v).x > (_v_max).x) (_v).x = (_v_max).x; else if ((_v).x < (_v_min).x) (_v).x = (_v_min).x; 
	    if ((_v).y > (_v_max).y) (_v).y = (_v_max).y; else if ((_v).y < (_v_min).y) (_v).y = (_v_min).z; 
	    if ((_v).z > (_v_max).y) (_v).z = (_v_max).z; else if ((_v).z < (_v_min).z) (_v).z = (_v_min).z; 
	  }

	/*  */
	public static void VECT3_ABS(DoubleVect3 _vo,DoubleVect3 _vi) { 
	    (_vo).x =Math.abs((_vi).x);   
	    (_vo).y = Math.abs((_vi).y);   
	    (_vo).z = Math.abs((_vi).z);   
	  }

	public static void VECT3_CROSS_PRODUCT(DoubleVect3 _vo,DoubleVect3 _v1,DoubleVect3 _v2) {        
	    (_vo).x = (_v1).y*(_v2).z - (_v1).z*(_v2).y;    
	    (_vo).y = (_v1).z*(_v2).x - (_v1).x*(_v2).z;    
	    (_vo).z = (_v1).x*(_v2).y - (_v1).y*(_v2).x;    
	  }

	public static void VECT3_DOT_PRODUCT(Double _so,DoubleVect3 _v1,DoubleVect3  _v2) {                        
	    (_so) = (_v1).x*(_v2).x + (_v1).y*(_v2).y + (_v1).z*(_v2).z;  
	}

	public static void VECT3_RATES_CROSS_VECT3(DoubleVect3 _vo,DoubleRates _r1,DoubleVect3 _v2) {    
	    (_vo).x = (_r1).q *(_v2).z - ((_r1).r)*(_v2).y;    
	    (_vo).y = (_r1).r *(_v2).x - ((_r1).p)*(_v2).z;    
	    (_vo).z = (_r1).p *(_v2).y - ((_r1).q)*(_v2).x;   } 
	 


	//
	//
	// Euler angles
	//
	//


	public static void EULERS_COPY(DoubleEulers _a ,DoubleEulers _b) {				
	    (_a).phi   = (_b).phi;				
	    (_a).theta = (_b).theta;				
	    (_a).psi   = (_b).psi;				
	  }
	public static void EULERS_COPY(Int32Eulers _a ,Int32Eulers _b) {				
		(_a).phi   = (_b).phi;				
		(_a).theta = (_b).theta;				
		(_a).psi   = (_b).psi;				
	}
	public static void EULERS_COPY(FloatEulers _a ,FloatEulers _b) {				
		(_a).phi   = (_b).phi;				
		(_a).theta = (_b).theta;				
		(_a).psi   = (_b).psi;				
	}

	public static void EULERS_ASSIGN(DoubleEulers _e, double _phi, double _theta, double _psi) {		
	    (_e).phi   = (_phi);                            
	    (_e).theta = (_theta);                          
	    (_e).psi   = (_psi);                            
	  }
	
	public static void EULERS_ASSIGN(FloatEulers _e, float _phi, float _theta, float _psi) {		
		(_e).phi   = (_phi);                            
		(_e).theta = (_theta);                          
		(_e).psi   = (_psi);                            
	}
	
	public static void EULERS_ASSIGN(Int32Eulers _e, int _phi, int _theta, int _psi) {		
		(_e).phi   = (_phi);                            
		(_e).theta = (_theta);                          
		(_e).psi   = (_psi);                            
	}

	/* a += b */
	public static void EULERS_ADD(DoubleEulers _a, DoubleEulers _b) {				
	    (_a).phi   += (_b).phi;				
	    (_a).theta += (_b).theta;				
	    (_a).psi   += (_b).psi;				
	  }
	public static void EULERS_ADD(Int32Eulers _a, Int32Eulers _b) {				
		(_a).phi   += (_b).phi;				
		(_a).theta += (_b).theta;				
		(_a).psi   += (_b).psi;				
	}

	/* a += b */
	public static void EULERS_SUB(DoubleEulers _a, DoubleEulers _b) {				
	    (_a).phi   -= (_b).phi;				
	    (_a).theta -= (_b).theta;				
	    (_a).psi   -= (_b).psi;				
	  }

	/* c = a - b */
	public static void EULERS_DIFF(DoubleEulers _c, DoubleEulers _a, DoubleEulers _b) {		
	    (_c).phi   = (_a).phi   - (_b).phi;		
	    (_c).theta = (_a).theta - (_b).theta;	
	    (_c).psi   = (_a).psi   - (_b).psi;		
	  }
	/* c = a - b */
	public static void EULERS_DIFF(Int32Eulers _c, Int32Eulers _a, Int32Eulers _b) {		
		(_c).phi   = (_a).phi   - (_b).phi;		
		(_c).theta = (_a).theta - (_b).theta;	
		(_c).psi   = (_a).psi   - (_b).psi;		
	}

	/* _vo =  _vi * _s */
	public static void EULERS_SMUL(DoubleEulers _eo, DoubleEulers _ei,double _s) {				
	    (_eo).phi   =  (_ei).phi   * (_s);				
	    (_eo).theta =  (_ei).theta * (_s);				
	    (_eo).psi   =  (_ei).psi   * (_s);				
	  }

	/* _vo =  _vi / _s */
	public static void EULERS_SDIV(DoubleEulers _eo, DoubleEulers _ei,double _s) {				
	    (_eo).phi   =  (_ei).phi   / (_s);				
	    (_eo).theta =  (_ei).theta / (_s);				
	    (_eo).psi   =  (_ei).psi   / (_s);				
	  }

	/* _v = Bound(_v, _min, _max) */
	public static void EULERS_BOUND_CUBE(DoubleEulers _v, double _min, double _max) {				           
	    (_v).phi   = (_v).phi   < (_min) ? (_min) : (_v).phi   > (_max) ? (_max) : (_v).phi; 
	    (_v).theta = (_v).theta < (_min) ? (_min) : (_v).theta > (_max) ? (_max) : (_v).theta; 
	    (_v).psi   = (_v).psi   < (_min) ? (_min) : (_v).psi   > (_max) ? (_max) : (_v).psi;   
	  }
	/* _v = Bound(_v, _min, _max) */
	public static void EULERS_BOUND_CUBE(Int32Eulers _v, int _min, int _max) {				           
		(_v).phi   = (_v).phi   < (_min) ? (_min) : (_v).phi   > (_max) ? (_max) : (_v).phi; 
		(_v).theta = (_v).theta < (_min) ? (_min) : (_v).theta > (_max) ? (_max) : (_v).theta; 
		(_v).psi   = (_v).psi   < (_min) ? (_min) : (_v).psi   > (_max) ? (_max) : (_v).psi;   
	}

	//
	//
	// Rates
	//
	//

	/* ra =  {p, q, r} */
	public static void RATES_ASSIGN(DoubleRates _ra, double _p, double _q, double _r) {		
	    (_ra).p = (_p);				
	    (_ra).q = (_q);				
	    (_ra).r = (_r);				
	  }
	
	public static void RATES_ASSIGN(FloatRates _ra, double _p, double _q, double _r) {		
	    (_ra).p = (float) (_p);				
	    (_ra).q = (float) (_q);				
	    (_ra).r = (float) (_r);				
	  }
	

	public static void RATES_ASSIGN(Int32Vect3 _ra, int _p, int _q, int _r) {		
		(_ra).x = (_p);				
		(_ra).y = (_q);				
		(_ra).z = (_r);				
	}
	
	public static void RATES_ASSIGN(Int32Rates _ra, int _p, int _q, int _r) {		
		(_ra).p = (_p);				
		(_ra).q = (_q);				
		(_ra).r = (_r);				
	}
	
	public static void RATES_ASSIGN(Int32Rates _ra, long _p, long _q, long _r) {		
		(_ra).p = (_p);				
		(_ra).q = (_q);				
		(_ra).r = (_r);				
	}
	

	/* a = b */
	public static void RATES_COPY(DoubleRates _a,DoubleRates _b) {			
	    (_a).p = (_b).p;				
	    (_a).q = (_b).q;				
	    (_a).r = (_b).r;				
	  }
	
	public static void RATES_COPY(FloatRates _a,DoubleRates _b) {			
	    (_a).p = (float) (_b).p;				
	    (_a).q = (float) (_b).q;				
	    (_a).r = (float) (_b).r;				
	  }
	
	public static void RATES_COPY(FloatRates _a,FloatRates _b) {			
		(_a).p = (_b).p;				
		(_a).q = (_b).q;				
		(_a).r = (_b).r;				
	}
	
	public static void RATES_COPY(FloatRates _a,Int32Rates _b) {			
		(_a).p = (_b).p;				
		(_a).q = (_b).q;				
		(_a).r = (_b).r;				
	}
	
	public static void RATES_COPY(Int32Rates _a,Int32Rates _b) {			
		(_a).p = (_b).p;				
		(_a).q = (_b).q;				
		(_a).r = (_b).r;				
	}

	/* a += b */
	public static void RATES_ADD(DoubleRates _a, DoubleRates _b) {			
	    (_a).p += (_b).p;				
	    (_a).q += (_b).q;				
	    (_a).r += (_b).r;				
	  }
	public static void RATES_ADD(Int32Rates _a, Int32Rates _b) {			
		(_a).p += (_b).p;				
		(_a).q += (_b).q;				
		(_a).r += (_b).r;				
	}

	/* a -= b */
	public static void RATES_SUB(DoubleRates _a, DoubleRates _b) {			
	    (_a).p -= (_b).p;				
	    (_a).q -= (_b).q;				
	    (_a).r -= (_b).r;				
	  }

	/* c = a + b */
	public static void RATES_SUM(DoubleRates _c, DoubleRates _a, DoubleRates _b) {			
	    (_c).p = (_a).p + (_b).p;			
	    (_c).q = (_a).q + (_b).q;			
	    (_c).r = (_a).r + (_b).r;			
	  }
	/* c = a + b */
	public static void RATES_SUM(Int32Rates _c, Int32Rates _a, Int32Rates _b) {			
		(_c).p = (_a).p + (_b).p;			
		(_c).q = (_a).q + (_b).q;			
		(_c).r = (_a).r + (_b).r;			
	}

	/* c = a + _s * b */
	public static void RATES_SUM_SCALED(DoubleRates _c, DoubleRates _a, DoubleRates _b, double _s) {		
	    (_c).p = (_a).p + (_s)*(_b).p;			
	    (_c).q = (_a).q + (_s)*(_b).q;			
	    (_c).r = (_a).r + (_s)*(_b).r;			
	  }

	/* c = a - b */
	public static void RATES_DIFF(DoubleRates _c, DoubleRates _a, DoubleRates  _b) {                
	    (_c).p = (_a).p - (_b).p;			
	    (_c).q = (_a).q - (_b).q;			
	    (_c).r = (_a).r - (_b).r;			
	  }
	
	public static void RATES_DIFF(FloatRates _c, FloatRates _a, FloatRates  _b) {                
	    (_c).p = (_a).p - (_b).p;			
	    (_c).q = (_a).q - (_b).q;			
	    (_c).r = (_a).r - (_b).r;			
	  }
	
	public static void RATES_DIFF(Int32Rates _c, Int32Rates _a, Int32Rates  _b) {                
		(_c).p = (_a).p - (_b).p;			
		(_c).q = (_a).q - (_b).q;			
		(_c).r = (_a).r - (_b).r;			
	}

	/* _ro =  _ri * _s */
	public static void RATES_SMUL(DoubleRates _ro, DoubleRates _ri, double _s) {		
	    (_ro).p =  (_ri).p * (_s);			
	    (_ro).q =  (_ri).q * (_s);			
	    (_ro).r =  (_ri).r * (_s);			
	  }
	/* _ro =  _ri * _s */
	public static void RATES_SMUL(FloatRates _ro, FloatRates _ri, double _s) {		
		(_ro).p =(float)(  (_ri).p * (_s));			
		(_ro).q = (float)( (_ri).q * (_s));			
		(_ro).r =(float)  ((_ri).r * (_s));			
	}

	/* _ro =  _ri / _s */
	public static void RATES_SDIV(DoubleRates _ro, DoubleRates _ri, double _s) {		
	    (_ro).p =  (_ri).p / (_s) ;			
	    (_ro).q =  (_ri).q / (_s);			
	    (_ro).r =  (_ri).r / (_s);			
	  }
	public static void RATES_SDIV(Int32Rates _ro, Int32Rates _ri, int _s) {		
		(_ro).p =  (_ri).p / (_s) ;			
		(_ro).q =  (_ri).q / (_s);			
		(_ro).r =  (_ri).r / (_s);			
	}

	/* Element wise vector multiplication */
	public static void RATES_EWMULT_RSHIFT(DoubleRates c, DoubleRates a,DoubleRates b, int _s) {  
	    (c).p = (int)((a).p * (b).p) >> (_s);		
	    (c).q = (int)((a).q * (b).q) >> (_s);		
	    (c).r = (int)((a).r * (b).r) >> (_s);		
	  }//?????????????????????????????????????????
	public static void RATES_EWMULT_RSHIFT(Int32Rates c, Int32Rates a,Int32Rates b, int _s) {  
		(c).p = (int)((a).p * (b).p) >> (_s);		
		(c).q = (int)((a).q * (b).q) >> (_s);		
		(c).r = (int)((a).r * (b).r) >> (_s);		
	}//?????????????????????????????????????????


	/* _v = Bound(_v, _min, _max) */
	public static void RATES_BOUND_CUBE(DoubleRates _v, double _min, double _max) {				
	    (_v).p = (_v).p < (_min) ? (_min) : (_v).p > (_max) ? (_max) : (_v).p;	
	    (_v).q = (_v).q < (_min) ? (_min) : (_v).q > (_max) ? (_max) : (_v).q;	
	    (_v).r = (_v).r < (_min) ? (_min) : (_v).r > (_max) ? (_max) : (_v).r;	
	  }
	public static void RATES_BOUND_CUBE(Int32Rates _v, int _min, int _max) {				
		(_v).p = (_v).p < (_min) ? (_min) : (_v).p > (_max) ? (_max) : (_v).p;	
		(_v).q = (_v).q < (_min) ? (_min) : (_v).q > (_max) ? (_max) : (_v).q;	
		(_v).r = (_v).r < (_min) ? (_min) : (_v).r > (_max) ? (_max) : (_v).r;	
	}

	public static void RATES_BOUND_BOX(DoubleRates _v, DoubleRates _v_min, DoubleRates _v_max) {				
	    if ((_v).p > (_v_max).p) (_v).p = (_v_max).p; else if ((_v).p < (_v_min).p) (_v).p = (_v_min).p; 
	    if ((_v).q > (_v_max).q) (_v).q = (_v_max).q; else if ((_v).q < (_v_min).q) (_v).q = (_v_min).q; 
	    if ((_v).r > (_v_max).r) (_v).r = (_v_max).r; else if ((_v).r < (_v_min).r) (_v).r = (_v_min).r; 
	  }
	public static void RATES_BOUND_BOX(Int32Rates _v, Int32Rates _v_min, Int32Rates _v_max) {				
		if ((_v).p > (_v_max).p) (_v).p = (_v_max).p; else if ((_v).p < (_v_min).p) (_v).p = (_v_min).p; 
		if ((_v).q > (_v_max).q) (_v).q = (_v_max).q; else if ((_v).q < (_v_min).q) (_v).q = (_v_min).q; 
		if ((_v).r > (_v_max).r) (_v).r = (_v_max).r; else if ((_v).r < (_v_min).r) (_v).r = (_v_min).r; 
	}



	//
	//
	// Matrix
	//
	//


	/*
	 * 3x3 matrices
	 */
	/* accessor : row and col range from 0 to 2 */
	public static Double MAT33_ELMT(DoubleMat33 _m, int _row, int _col) {
		return (_m).m[_row*3 +_col];
		} ///??????
	public static float MAT33_ELMT(FloatMat33 _m, int _row, int _col) {
		return (_m).m[_row*3 +_col];
	} ///??????

	public static void MAT33_COPY(DoubleMat33 _mat1,DoubleMat33 _mat2) {			
		(_mat1).m[0*3 +0] = MAT33_ELMT((_mat2),0,0);	
		(_mat1).m[0*3 +1] = MAT33_ELMT((_mat2),0,1);	
		(_mat1).m[0*3 +2] = MAT33_ELMT((_mat2),0,2);	
		(_mat1).m[1*3 +0] = MAT33_ELMT((_mat2),1,0);	
		(_mat1).m[1*3 +1] = MAT33_ELMT((_mat2),1,1);	
		(_mat1).m[1*3 +2] = MAT33_ELMT((_mat2),1,2);	
		(_mat1).m[2*3 +0] = MAT33_ELMT((_mat2),2,0);	
		(_mat1).m[2*3 +1] = MAT33_ELMT((_mat2),2,1);	
		(_mat1).m[2*3 +2] = MAT33_ELMT((_mat2),2,2);	
	}


	/* multiply _vin by _mat, store in _vout */
	public static void MAT33_VECT3_MUL(DoubleVect3 _vout,DoubleMat33 _mat, DoubleVect3 _vin) {		
	    (_vout).x = MAT33_ELMT((_mat), 0, 0) * (_vin).x +	
	                MAT33_ELMT((_mat), 0, 1) * (_vin).y +	
	                MAT33_ELMT((_mat), 0, 2) * (_vin).z;	
	    (_vout).y = MAT33_ELMT((_mat), 1, 0) * (_vin).x +   
	                MAT33_ELMT((_mat), 1, 1) * (_vin).y +   
	                MAT33_ELMT((_mat), 1, 2) * (_vin).z;	
	    (_vout).z = MAT33_ELMT((_mat), 2, 0) * (_vin).x +	
	                MAT33_ELMT((_mat), 2, 1) * (_vin).y +	
	                MAT33_ELMT((_mat), 2, 2) * (_vin).z;	
	  }
	public static void MAT33_VECT3_MUL(EnuCoor_f _vout,FloatMat33 _mat, EcefCoor_f _vin) {		
		(_vout).x = MAT33_ELMT((_mat), 0, 0) * (_vin).x +	
				MAT33_ELMT((_mat), 0, 1) * (_vin).y +	
				MAT33_ELMT((_mat), 0, 2) * (_vin).z;	
		(_vout).y = MAT33_ELMT((_mat), 1, 0) * (_vin).x +   
				MAT33_ELMT((_mat), 1, 1) * (_vin).y +   
				MAT33_ELMT((_mat), 1, 2) * (_vin).z;	
		(_vout).z = MAT33_ELMT((_mat), 2, 0) * (_vin).x +	
				MAT33_ELMT((_mat), 2, 1) * (_vin).y +	
				MAT33_ELMT((_mat), 2, 2) * (_vin).z;	
	}

	/* multiply _vin by transpose of _mat, store in _vout */
	public static void MAT33_VECT3_TRANSP_MUL(DoubleVect3 _vout,DoubleMat33 _mat, DoubleVect3 _vin) {     
	    (_vout).x = MAT33_ELMT((_mat), 0, 0) * (_vin).x +	
	                MAT33_ELMT((_mat), 1, 0) * (_vin).y +	
	                MAT33_ELMT((_mat), 2, 0) * (_vin).z;	
	    (_vout).y = MAT33_ELMT((_mat), 0, 1) * (_vin).x +   
	                MAT33_ELMT((_mat), 1, 1) * (_vin).y +   
	                MAT33_ELMT((_mat), 2, 1) * (_vin).z;	
	    (_vout).z = MAT33_ELMT((_mat), 0, 2) * (_vin).x +   
	                MAT33_ELMT((_mat), 1, 2) * (_vin).y +	
	                MAT33_ELMT((_mat), 2, 2) * (_vin).z;	
	  }
	public static void MAT33_VECT3_TRANSP_MUL(EcefCoor_d _vout,DoubleMat33 _mat, EnuCoor_f _vin) {     
		(_vout).x = MAT33_ELMT((_mat), 0, 0) * (_vin).x +	
				MAT33_ELMT((_mat), 1, 0) * (_vin).y +	
				MAT33_ELMT((_mat), 2, 0) * (_vin).z;	
		(_vout).y = MAT33_ELMT((_mat), 0, 1) * (_vin).x +   
				MAT33_ELMT((_mat), 1, 1) * (_vin).y +   
				MAT33_ELMT((_mat), 2, 1) * (_vin).z;	
		(_vout).z = MAT33_ELMT((_mat), 0, 2) * (_vin).x +   
				MAT33_ELMT((_mat), 1, 2) * (_vin).y +	
				MAT33_ELMT((_mat), 2, 2) * (_vin).z;	
	}

	/* invS = 1/det(S) com(S)' */
	public static void MAT33_INV( DoubleMat33 _minv, DoubleMat33 _m, double FLT_EPSILON) {						
	    final double m00 = MAT33_ELMT((_m),1,1)*MAT33_ELMT((_m),2,2) - MAT33_ELMT((_m),1,2)*MAT33_ELMT((_m),2,1);		
	    final double m10 = MAT33_ELMT((_m),0,1)*MAT33_ELMT((_m),2,2) - MAT33_ELMT((_m),0,2)*MAT33_ELMT((_m),2,1);		
	    final double m20 = MAT33_ELMT((_m),0,1)*MAT33_ELMT((_m),1,2) - MAT33_ELMT((_m),0,2)*MAT33_ELMT((_m),1,1);		
	    final double m01 = MAT33_ELMT((_m),1,0)*MAT33_ELMT((_m),2,2) - MAT33_ELMT((_m),1,2)*MAT33_ELMT((_m),2,0);		
	    final double m11 = MAT33_ELMT((_m),0,0)*MAT33_ELMT((_m),2,2) - MAT33_ELMT((_m),0,2)*MAT33_ELMT((_m),2,0);		
	    final double m21 = MAT33_ELMT((_m),0,0)*MAT33_ELMT((_m),1,2) - MAT33_ELMT((_m),0,2)*MAT33_ELMT((_m),1,0);		
	    final double m02 = MAT33_ELMT((_m),1,0)*MAT33_ELMT((_m),2,1) - MAT33_ELMT((_m),1,1)*MAT33_ELMT((_m),2,0);		
	    final double m12 = MAT33_ELMT((_m),0,0)*MAT33_ELMT((_m),2,1) - MAT33_ELMT((_m),0,1)*MAT33_ELMT((_m),2,0);		
	    final double m22 = MAT33_ELMT((_m),0,0)*MAT33_ELMT((_m),1,1) - MAT33_ELMT((_m),0,1)*MAT33_ELMT((_m),1,0);		
	    final double det = MAT33_ELMT((_m),0,0)*m00 - MAT33_ELMT((_m),1,0)*m10 + MAT33_ELMT((_m),2,0)*m20; 
	    if (det > FLT_EPSILON) {					
	      (_minv).m[0*3 +0] =  m00 / det;						
	      (_minv).m[1*3 +0] = -m01 / det;						
	      (_minv).m[2*3 +0] =  m02 / det;						
	      (_minv).m[0*3 +1] = -m10 / det;						
	      (_minv).m[1*3 +1] =  m11 / det;						
	      (_minv).m[2*3 +1] = -m12 / det;						
	      (_minv).m[0*3 +2] =  m20 / det;						
	      (_minv).m[1*3 +2] = -m21 / det;						
	      (_minv).m[2*3 +2] =  m22 / det;						
	    }									
	  }

	/* set _row of _mat with _vin multiplied by scalar _s */
	public static void MAT33_ROW_VECT3_SMUL(DoubleMat33 _mat, int _row,DoubleVect3 _vin, double _s) {         
	    (_mat).m[_row*3 + 0] = (_vin).x * (_s);           
	    (_mat).m[_row*3 +1] = (_vin).y * (_s);           
	    (_mat).m[_row*3 +2] = (_vin).z * (_s);           
	  }


	//
	//
	// Quaternion algebras
	//
	//

	/* _q = [_i _x _y _z] */
	public static void QUAT_ASSIGN(DoubleQuat _q, double _i,double _x, double _y,double _z) {   
	    (_q).qi = (_i);			    
	    (_q).qx = (_x);			    
	    (_q).qy = (_y);			    
	    (_q).qz = (_z);			    
	  }

	public static void QUAT_ASSIGN(FloatQuat _q, double _i,double _x, double _y,double _z) {   
	    (_q).qi = (float) (_i);			    
	    (_q).qx = (float) (_x);			    
	    (_q).qy = (float) (_y);			    
	    (_q).qz = (float) (_z);			    
	  }
	
	/* _qc = _qa - _qc */
	public static void QUAT_DIFF(DoubleQuat _qc,DoubleQuat  _qa,DoubleQuat _qb) {	    
	    (_qc).qi = (_qa).qi - (_qb).qi;	    
	    (_qc).qx = (_qa).qx - (_qb).qx;	    
	    (_qc).qy = (_qa).qy - (_qb).qy;	    
	    (_qc).qz = (_qa).qz - (_qb).qz;	    
	  }

	/* _qo = _qi */
	public static void QUAT_COPY(DoubleQuat _qo, DoubleQuat _qi) {	    
	    (_qo).qi = (_qi).qi;		    
	    (_qo).qx = (_qi).qx;		    
	    (_qo).qy = (_qi).qy;		    
	    (_qo).qz = (_qi).qz;		    
	  }
	
	public static void QUAT_COPY(FloatQuat _qo, DoubleQuat _qi) {	    
	    (_qo).qi = (float) (_qi).qi;		    
	    (_qo).qx = (float) (_qi).qx;		    
	    (_qo).qy = (float) (_qi).qy;		    
	    (_qo).qz = (float) (_qi).qz;		    
	  }

	public static void QUAT_COPY(FloatQuat _qo, FloatQuat _qi) {	    
		(_qo).qi = (_qi).qi;		    
		(_qo).qx = (_qi).qx;		    
		(_qo).qy = (_qi).qy;		    
		(_qo).qz = (_qi).qz;		    
	}
	public static void QUAT_COPY(Int32Quat _qo, Int32Quat _qi) {	    
		(_qo).qi = (_qi).qi;		    
		(_qo).qx = (_qi).qx;		    
		(_qo).qy = (_qi).qy;		    
		(_qo).qz = (_qi).qz;		    
	}

	public static void QUAT_EXPLEMENTARY(DoubleQuat b,DoubleQuat a) {    
	    (b).qi = -(a).qi;               
	    (b).qx = -(a).qx;               
	    (b).qy = -(a).qy;               
	    (b).qz = -(a).qz;               
	  }
	public static void QUAT_EXPLEMENTARY(Int32Quat b,Int32Quat a) {    
	    (b).qi = -(a).qi;               
	    (b).qx = -(a).qx;               
	    (b).qy = -(a).qy;               
	    (b).qz = -(a).qz;               
	  }

	/* _qo = _qi * _s */
	public static void QUAT_SMUL(DoubleQuat _qo, DoubleQuat _qi, double _s) {	    
	    (_qo).qi = (_qi).qi * (_s);		    
	    (_qo).qx = (_qi).qx * (_s);		    
	    (_qo).qy = (_qi).qy * (_s);		    
	    (_qo).qz = (_qi).qz * (_s);		    
	  }
	
	public static void QUAT_SMUL(FloatQuat _qo, FloatQuat _qi, double _s) {	    
	    (_qo).qi = (float) ((_qi).qi * (_s));		    
	    (_qo).qx = (float) ((_qi).qx * (_s));		    
	    (_qo).qy = (float) ((_qi).qy * (_s));		    
	    (_qo).qz = (float) ((_qi).qz * (_s));		    
	  }

	/* _qo = _qo + _qi */
	public static void QUAT_ADD(DoubleQuat _qo, DoubleQuat _qi) {		    
	    (_qo).qi += (_qi).qi;		    
	    (_qo).qx += (_qi).qx;		    
	    (_qo).qy += (_qi).qy;		    
	    (_qo).qz += (_qi).qz;		    
	  }
	
	public static void QUAT_ADD(FloatQuat _qo, FloatQuat _qi) {		    
	    (_qo).qi += (_qi).qi;		    
	    (_qo).qx += (_qi).qx;		    
	    (_qo).qy += (_qi).qy;		    
	    (_qo).qz += (_qi).qz;		    
	  }

	/* _qo = [qi -qx -qy -qz] */
	public static void QUAT_INVERT(DoubleQuat _qo, DoubleQuat _qi) {		    
	    (_qo).qi =  (_qi).qi;		    
	    (_qo).qx = -(_qi).qx;		    
	    (_qo).qy = -(_qi).qy;		    
	    (_qo).qz = -(_qi).qz;		    
	  }

	public static void QUAT_INVERT(FloatQuat _qo, FloatQuat _qi) {		    
	    (_qo).qi =  (_qi).qi;		    
	    (_qo).qx = -(_qi).qx;		    
	    (_qo).qy = -(_qi).qy;		    
	    (_qo).qz = -(_qi).qz;		    
	  }

	/* _vo=[ qx qy qz] */
	public static void QUAT_EXTRACT_Q(DoubleVect3 _vo, DoubleQuat _qi) {  
	  (_vo).x=(_qi).qx;                 
	  (_vo).y=(_qi).qy;                 
	  (_vo).z=(_qi).qz;                 
	}

	public static void QUAT_EXTRACT_Q(FloatVect3 _vo, FloatQuat _qi) {  
		  (_vo).x=(_qi).qx;                 
		  (_vo).y=(_qi).qy;                 
		  (_vo).z=(_qi).qz;                 
		}

	/* _qo = _qo / _s */
	public static void QUAT_SDIV(DoubleQuat _qo, DoubleQuat _qi, double _s) { 
	    (_qo).qi = (_qi).qi / _s; 
	    (_qo).qx = (_qi).qx / _s; 
	    (_qo).qy = (_qi).qy / _s; 
	    (_qo).qz = (_qi).qz / _s; 
	  }

	//
	//
	// Rotation Matrices
	//
	//


	/* accessor : row and col range from 0 to 2 */
	public static double RMAT_ELMT(DoubleRMat _rm,int _row,int  _col) {return _rm.m [_row * 3 + _col];}
	public static long RMAT_ELMT(Int32RMat _rm,int _row,int  _col) {return _rm.m [_row * 3 + _col];}
	public static float RMAT_ELMT(FloatRMat _rm,int _row,int  _col) {return _rm.m [_row * 3 + _col];}
	public static int RMAT_ELMT(Int32Mat33 _rm,int _row,int  _col) {return _rm.m [_row * 3 + _col];}

	/* trace */
	public static double RMAT_TRACE( DoubleRMat _rm) {return (RMAT_ELMT(_rm, 0, 0)+RMAT_ELMT(_rm, 1, 1)+RMAT_ELMT(_rm, 2, 2));}


	public static void RMAT_DIFF(DoubleRMat _c,DoubleRMat  _a, DoubleRMat _b) {				 
	    (_c).m[0] = (_a).m[0] - (_b).m[0];			 
	    (_c).m[1] = (_a).m[1] - (_b).m[1];			 
	    (_c).m[2] = (_a).m[2] - (_b).m[2];			 
	    (_c).m[3] = (_a).m[3] - (_b).m[3];			 
	    (_c).m[4] = (_a).m[4] - (_b).m[4];			 
	    (_c).m[5] = (_a).m[5] - (_b).m[5];			 
	    (_c).m[6] = (_a).m[6] - (_b).m[6];			 
	    (_c).m[7] = (_a).m[7]- (_b).m[7];			 
	    (_c).m[8] = (_a).m[8] - (_b).m[8];			 
	  }
	public static void RMAT_DIFF(Int32RMat _c,Int32RMat  _a, Int32RMat _b) {				 
	    (_c).m[0] = (_a).m[0] - (_b).m[0];			 
	    (_c).m[1] = (_a).m[1] - (_b).m[1];			 
	    (_c).m[2] = (_a).m[2] - (_b).m[2];			 
	    (_c).m[3] = (_a).m[3] - (_b).m[3];			 
	    (_c).m[4] = (_a).m[4] - (_b).m[4];			 
	    (_c).m[5] = (_a).m[5] - (_b).m[5];			 
	    (_c).m[6] = (_a).m[6] - (_b).m[6];			 
	    (_c).m[7] = (_a).m[7]- (_b).m[7];			 
	    (_c).m[8] = (_a).m[8] - (_b).m[8];			 
	  }

	/* multiply _vin by _rmat, store in _vout */
	public static void RMAT_VECT3_MUL(DoubleVect3 _vout , DoubleRMat _rmat,DoubleVect3 _vin) {		 
	    (_vout).x = RMAT_ELMT((_rmat), 0, 0) * (_vin).x +	 
	                RMAT_ELMT((_rmat), 0, 1) * (_vin).y +	 
	                RMAT_ELMT((_rmat), 0, 2) * (_vin).z;	 
	    (_vout).y = RMAT_ELMT((_rmat), 1, 0) * (_vin).x +    
	                RMAT_ELMT((_rmat), 1, 1) * (_vin).y +    
	                RMAT_ELMT((_rmat), 1, 2) * (_vin).z;	 
	    (_vout).z = RMAT_ELMT((_rmat), 2, 0) * (_vin).x +	 
	                RMAT_ELMT((_rmat), 2, 1) * (_vin).y +	 
	                RMAT_ELMT((_rmat), 2, 2) * (_vin).z;	 
	  }

	public static void RMAT_VECT3_TRANSP_MUL(DoubleVect3 _vout , DoubleRMat _rmat,DoubleVect3 _vin) {     
	    (_vout).x = RMAT_ELMT((_rmat), 0, 0) * (_vin).x +	 
	                RMAT_ELMT((_rmat), 1, 0) * (_vin).y +	 
	                RMAT_ELMT((_rmat), 2, 0) * (_vin).z;	 
	    (_vout).y = RMAT_ELMT((_rmat), 0, 1) * (_vin).x +    
	                RMAT_ELMT((_rmat), 1, 1) * (_vin).y +    
	                RMAT_ELMT((_rmat), 2, 1) * (_vin).z;	 
	    (_vout).z = RMAT_ELMT((_rmat), 0, 2) * (_vin).x +	 
	                RMAT_ELMT((_rmat), 1, 2) * (_vin).y +	 
	                RMAT_ELMT((_rmat), 2, 2) * (_vin).z;	 
	  }
	
	public static void RMAT_VECT3_TRANSP_MUL(FloatVect3 _vout , FloatRMat _rmat,FloatVect3 _vin) {     
	    (_vout).x = RMAT_ELMT((_rmat), 0, 0) * (_vin).x +	 
	                RMAT_ELMT((_rmat), 1, 0) * (_vin).y +	 
	                RMAT_ELMT((_rmat), 2, 0) * (_vin).z;	 
	    (_vout).y = RMAT_ELMT((_rmat), 0, 1) * (_vin).x +    
	                RMAT_ELMT((_rmat), 1, 1) * (_vin).y +    
	                RMAT_ELMT((_rmat), 2, 1) * (_vin).z;	 
	    (_vout).z = RMAT_ELMT((_rmat), 0, 2) * (_vin).x +	 
	                RMAT_ELMT((_rmat), 1, 2) * (_vin).y +	 
	                RMAT_ELMT((_rmat), 2, 2) * (_vin).z;	 
	  }


	public static void RMAT_COPY(DoubleRMat _mat1,DoubleRMat _mat2) { 
	(_mat1).m[0] = RMAT_ELMT((_mat2),0,0);	
	(_mat1).m[1] = RMAT_ELMT((_mat2),0,1);	
	(_mat1).m[2] = RMAT_ELMT((_mat2),0,2);	
	(_mat1).m[3] = RMAT_ELMT((_mat2),1,0);	
	(_mat1).m[4] = RMAT_ELMT((_mat2),1,1);	
	(_mat1).m[5] = RMAT_ELMT((_mat2),1,2);	
	(_mat1).m[6] = RMAT_ELMT((_mat2),2,0);	
	(_mat1).m[7] = RMAT_ELMT((_mat2),2,1);	
	(_mat1).m[8] = RMAT_ELMT((_mat2),2,2);	}
	public static void RMAT_COPY(Int32RMat _mat1,Int32RMat _mat2) { 
		(_mat1).m[0] = RMAT_ELMT((_mat2),0,0);	
		(_mat1).m[1] = RMAT_ELMT((_mat2),0,1);	
		(_mat1).m[2] = RMAT_ELMT((_mat2),0,2);	
		(_mat1).m[3] = RMAT_ELMT((_mat2),1,0);	
		(_mat1).m[4] = RMAT_ELMT((_mat2),1,1);	
		(_mat1).m[5] = RMAT_ELMT((_mat2),1,2);	
		(_mat1).m[6] = RMAT_ELMT((_mat2),2,0);	
		(_mat1).m[7] = RMAT_ELMT((_mat2),2,1);	
		(_mat1).m[8] = RMAT_ELMT((_mat2),2,2);	}
	
	public static void RMAT_COPY(FloatRMat _mat1,FloatRMat _mat2) { 
		(_mat1).m[0] = RMAT_ELMT((_mat2),0,0);	
		(_mat1).m[1] = RMAT_ELMT((_mat2),0,1);	
		(_mat1).m[2] = RMAT_ELMT((_mat2),0,2);	
		(_mat1).m[3] = RMAT_ELMT((_mat2),1,0);	
		(_mat1).m[4] = RMAT_ELMT((_mat2),1,1);	
		(_mat1).m[5] = RMAT_ELMT((_mat2),1,2);	
		(_mat1).m[6] = RMAT_ELMT((_mat2),2,0);	
		(_mat1).m[7] = RMAT_ELMT((_mat2),2,1);	
		(_mat1).m[8] = RMAT_ELMT((_mat2),2,2);	}

	public static void RMAT_COPY(Int32Mat33 _mat1,Int32Mat33 _mat2) { 
		(_mat1).m[0] = RMAT_ELMT((_mat2),0,0);	
		(_mat1).m[1] = RMAT_ELMT((_mat2),0,1);	
		(_mat1).m[2] = RMAT_ELMT((_mat2),0,2);	
		(_mat1).m[3] = RMAT_ELMT((_mat2),1,0);	
		(_mat1).m[4] = RMAT_ELMT((_mat2),1,1);	
		(_mat1).m[5] = RMAT_ELMT((_mat2),1,2);	
		(_mat1).m[6] = RMAT_ELMT((_mat2),2,0);	
		(_mat1).m[7] = RMAT_ELMT((_mat2),2,1);	
		(_mat1).m[8] = RMAT_ELMT((_mat2),2,2);	}




	public static void EULERS_FLOAT_OF_BFP(FloatEulers _ef, Int32Eulers _ei) {			
	    (_ef).phi   = ANGLE_FLOAT_OF_BFP((_ei).phi);	
	    (_ef).theta = ANGLE_FLOAT_OF_BFP((_ei).theta);	
	    (_ef).psi   = ANGLE_FLOAT_OF_BFP((_ei).psi);	
	  }

	public static void EULERS_BFP_OF_REAL(Int32Eulers _ei,FloatEulers _ef) {			
	    (_ei).phi   = ANGLE_BFP_OF_REAL((_ef).phi);		
	    (_ei).theta = ANGLE_BFP_OF_REAL((_ef).theta);	
	    (_ei).psi   = ANGLE_BFP_OF_REAL((_ef).psi);		
	  }

//	public static void RMAT_BFP_OF_REAL(Int32Eulers _ei,FloatEulers _ef) {			
//	    (_ei).m[0][0] = TRIG_BFP_OF_REAL((_ef).m[0]);		
//	    (_ei).m[1] = TRIG_BFP_OF_REAL((_ef).m[1]);		
//	    (_ei).m[2] = TRIG_BFP_OF_REAL((_ef).m[2]);		
//	    (_ei).m[3] = TRIG_BFP_OF_REAL((_ef).m[3]);		
//	    (_ei).m[4] = TRIG_BFP_OF_REAL((_ef).m[4]);		
//	    (_ei).m[5] = TRIG_BFP_OF_REAL((_ef).m[5]);		
//	    (_ei).m[6] = TRIG_BFP_OF_REAL((_ef).m[6]);		
//	    (_ei).m[7] = TRIG_BFP_OF_REAL((_ef).m[7]);		
//	    (_ei).m[8] = TRIG_BFP_OF_REAL((_ef).m[8]);		
//	  }

//	public static void RMAT_FLOAT_OF_BFP(_ef, _ei) {			
//	    (_ef).m[0] = TRIG_FLOAT_OF_BFP((_ei).m[0]);		
//	    (_ef).m[1] = TRIG_FLOAT_OF_BFP((_ei).m[1]);		
//	    (_ef).m[2] = TRIG_FLOAT_OF_BFP((_ei).m[2]);		
//	    (_ef).m[3] = TRIG_FLOAT_OF_BFP((_ei).m[3]);		
//	    (_ef).m[4] = TRIG_FLOAT_OF_BFP((_ei).m[4]);		
//	    (_ef).m[5] = TRIG_FLOAT_OF_BFP((_ei).m[5]);		
//	    (_ef).m[6] = TRIG_FLOAT_OF_BFP((_ei).m[6]);		
//	    (_ef).m[7] = TRIG_FLOAT_OF_BFP((_ei).m[7]);		
//	    (_ef).m[8] = TRIG_FLOAT_OF_BFP((_ei).m[8]);		
//	  }

	public static void QUAT_FLOAT_OF_BFP(FloatQuat _qf,Int32Quat _qi) {			
	    (_qf).qi = QUAT1_FLOAT_OF_BFP((_qi).qi);		
	    (_qf).qx = QUAT1_FLOAT_OF_BFP((_qi).qx);		
	    (_qf).qy = QUAT1_FLOAT_OF_BFP((_qi).qy);		
	    (_qf).qz = QUAT1_FLOAT_OF_BFP((_qi).qz);		
	  }

	public static void QUAT_BFP_OF_REAL(Int32Quat _qi,FloatQuat _qf) {			
	    (_qi).qi = QUAT1_BFP_OF_REAL((_qf).qi);		
	    (_qi).qx = QUAT1_BFP_OF_REAL((_qf).qx);		
	    (_qi).qy = QUAT1_BFP_OF_REAL((_qf).qy);		
	    (_qi).qz = QUAT1_BFP_OF_REAL((_qf).qz);		
	  }

	public static void RATES_FLOAT_OF_BFP(FloatRates _rf, Int32Rates _ri) {			
	    (_rf).p = RATE_FLOAT_OF_BFP((_ri).p);		
	    (_rf).q = RATE_FLOAT_OF_BFP((_ri).q);		
	    (_rf).r = RATE_FLOAT_OF_BFP((_ri).r);		
	  }
	
	public static void RATES_FLOAT_OF_BFP(FloatVect3 _rf, Int32Rates _ri) {			
	    (_rf).x = RATE_FLOAT_OF_BFP((_ri).p);		
	    (_rf).y = RATE_FLOAT_OF_BFP((_ri).q);		
	    (_rf).z = RATE_FLOAT_OF_BFP((_ri).r);		
	  }
	
	public static void RATES_FLOAT_OF_BFP(FloatRates _rf, FloatRates _ri) {			
	    (_rf).p = RATE_FLOAT_OF_BFP((_ri).p);		
	    (_rf).q = RATE_FLOAT_OF_BFP((_ri).q);		
	    (_rf).r = RATE_FLOAT_OF_BFP((_ri).r);		
	  }

	public static void RATES_BFP_OF_REAL(Int32Rates _ri,FloatRates _rf) {			
	    (_ri).p = RATE_BFP_OF_REAL((_rf).p);		
	    (_ri).q = RATE_BFP_OF_REAL((_rf).q);		
	    (_ri).r = RATE_BFP_OF_REAL((_rf).r);		
	  }

	public static void POSITIONS_FLOAT_OF_BFP(FloatVect3 _ef,Int32Vect3 _ei) {      
	    (_ef).x = POS_FLOAT_OF_BFP((_ei).x);		
	    (_ef).y = POS_FLOAT_OF_BFP((_ei).y);		
	    (_ef).z = POS_FLOAT_OF_BFP((_ei).z);		
	  }

	public static void POSITIONS_BFP_OF_REAL(FloatVect3 _ef,Int32Vect3 _ei) {	
	    (_ef).x = POS_BFP_OF_REAL((_ei).x);		
	    (_ef).y = POS_BFP_OF_REAL((_ei).y);		
	    (_ef).z = POS_BFP_OF_REAL((_ei).z);		
	  }

	public static void SPEEDS_FLOAT_OF_BFP(FloatVect3 _ef,Int32Vect3 _ei) {			
	    (_ef).x = SPEED_FLOAT_OF_BFP((_ei).x);		
	    (_ef).y = SPEED_FLOAT_OF_BFP((_ei).y);		
	    (_ef).z = SPEED_FLOAT_OF_BFP((_ei).z);		
	  }
	public static void SPEEDS_FLOAT_OF_BFP(EnuCoor_f _ef,EnuCoor_i _ei) {			
		(_ef).x = SPEED_FLOAT_OF_BFP((_ei).x);		
		(_ef).y = SPEED_FLOAT_OF_BFP((_ei).y);		
		(_ef).z = SPEED_FLOAT_OF_BFP((_ei).z);		
	}
	public static void SPEEDS_FLOAT_OF_BFP(NedCoor_f _ef,NedCoor_i _ei) {			
		(_ef).x = SPEED_FLOAT_OF_BFP((_ei).x);		
		(_ef).y = SPEED_FLOAT_OF_BFP((_ei).y);		
		(_ef).z = SPEED_FLOAT_OF_BFP((_ei).z);		
	}
	public static void SPEEDS_FLOAT_OF_BFP(EcefCoor_f _ef,NedCoor_i _ei) {			
		(_ef).x = SPEED_FLOAT_OF_BFP((_ei).x);		
		(_ef).y = SPEED_FLOAT_OF_BFP((_ei).y);		
		(_ef).z = SPEED_FLOAT_OF_BFP((_ei).z);		
	}

	public static void SPEEDS_BFP_OF_REAL(FloatVect3 _ef,Int32Vect3 _ei) {			
	    (_ef).x = SPEED_BFP_OF_REAL((_ei).x);		
	    (_ef).y = SPEED_BFP_OF_REAL((_ei).y);		
	    (_ef).z = SPEED_BFP_OF_REAL((_ei).z);		
	  }
	public static void SPEEDS_BFP_OF_REAL(EcefCoor_i _ef,EcefCoor_f _ei) {			
		(_ef).x = SPEED_BFP_OF_REAL((_ei).x);		
		(_ef).y = SPEED_BFP_OF_REAL((_ei).y);		
		(_ef).z = SPEED_BFP_OF_REAL((_ei).z);		
	}
	public static void SPEEDS_BFP_OF_REAL(EnuCoor_i _ef,EnuCoor_f _ei) {			
		(_ef).x = SPEED_BFP_OF_REAL((_ei).x);		
		(_ef).y = SPEED_BFP_OF_REAL((_ei).y);		
		(_ef).z = SPEED_BFP_OF_REAL((_ei).z);		
	}
	public static void SPEEDS_BFP_OF_REAL(NedCoor_i  _ef,NedCoor_f _ei) {			
		(_ef).x = SPEED_BFP_OF_REAL((_ei).x);		
		(_ef).y = SPEED_BFP_OF_REAL((_ei).y);		
		(_ef).z = SPEED_BFP_OF_REAL((_ei).z);		
	}

	public static void ACCELS_FLOAT_OF_BFP(FloatVect3 _ef,Int32Vect3 _ei) {			
	    (_ef).x = ACCEL_FLOAT_OF_BFP((_ei).x);		
	    (_ef).y = ACCEL_FLOAT_OF_BFP((_ei).y);		
	    (_ef).z = ACCEL_FLOAT_OF_BFP((_ei).z);		
	  }
	public static void ACCELS_FLOAT_OF_BFP(EcefCoor_f _ef,NedCoor_i _ei) {			
		(_ef).x = ACCEL_FLOAT_OF_BFP((_ei).x);		
		(_ef).y = ACCEL_FLOAT_OF_BFP((_ei).y);		
		(_ef).z = ACCEL_FLOAT_OF_BFP((_ei).z);		
	}
	public static void ACCELS_FLOAT_OF_BFP(NedCoor_f _ef,NedCoor_i _ei) {			
		(_ef).x = ACCEL_FLOAT_OF_BFP((_ei).x);		
		(_ef).y = ACCEL_FLOAT_OF_BFP((_ei).y);		
		(_ef).z = ACCEL_FLOAT_OF_BFP((_ei).z);		
	}

	public static void ACCELS_BFP_OF_REAL(FloatVect3 _ef,Int32Vect3 _ei) {			
	    (_ef).x = ACCEL_BFP_OF_REAL((_ei).x);		
	    (_ef).y = ACCEL_BFP_OF_REAL((_ei).y);		
	    (_ef).z = ACCEL_BFP_OF_REAL((_ei).z);		
	  }
	public static void ACCELS_BFP_OF_REAL(NedCoor_i _ef,NedCoor_f _ei) {			
		(_ef).x =  ACCEL_BFP_OF_REAL((_ei).x);		
		(_ef).y = ACCEL_BFP_OF_REAL((_ei).y);		
		(_ef).z = ACCEL_BFP_OF_REAL((_ei).z);		
	}
	public static void ACCELS_BFP_OF_REAL(EcefCoor_i _ef,EcefCoor_f _ei) {			
		(_ef).x = ACCEL_BFP_OF_REAL((_ei).x);		
		(_ef).y = ACCEL_BFP_OF_REAL((_ei).y);		
		(_ef).z = ACCEL_BFP_OF_REAL((_ei).z);		
	}

	public static void MAGS_FLOAT_OF_BFP(FloatVect3 _ef,Int32Vect3 _ei) {			
	    (_ef).x = MAG_FLOAT_OF_BFP((_ei).x);		
	    (_ef).y = MAG_FLOAT_OF_BFP((_ei).y);		
	    (_ef).z = MAG_FLOAT_OF_BFP((_ei).z);		
	  }

	public static void MAGS_BFP_OF_REAL(FloatVect3 _ef,Int32Vect3 _ei) {			
	    (_ef).x = MAG_BFP_OF_REAL((_ei).x);		
	    (_ef).y = MAG_BFP_OF_REAL((_ei).y);		
	    (_ef).z = MAG_BFP_OF_REAL((_ei).z);		
	  }
	
	public static void INT32_VECT3_ADD(EcefCoor_i _a,EcefCoor_i _b) {
		VECT3_ADD(_a, _b);
	}
	
	public static void RMAT_FLOAT_OF_BFP(FloatRMat _ef,Int32RMat _ei) {			
	    (_ef).m[0] = TRIG_FLOAT_OF_BFP((_ei).m[0]);		
	    (_ef).m[1] = TRIG_FLOAT_OF_BFP((_ei).m[1]);		
	    (_ef).m[2] = TRIG_FLOAT_OF_BFP((_ei).m[2]);		
	    (_ef).m[3] = TRIG_FLOAT_OF_BFP((_ei).m[3]);		
	    (_ef).m[4] = TRIG_FLOAT_OF_BFP((_ei).m[4]);		
	    (_ef).m[5] = TRIG_FLOAT_OF_BFP((_ei).m[5]);		
	    (_ef).m[6] = TRIG_FLOAT_OF_BFP((_ei).m[6]);		
	    (_ef).m[7] = TRIG_FLOAT_OF_BFP((_ei).m[7]);		
	    (_ef).m[8] = TRIG_FLOAT_OF_BFP((_ei).m[8]);		
	  }
	
	public static void RMAT_BFP_OF_REAL(Int32RMat _ei,FloatRMat _ef) {			
	    (_ei).m[0] = TRIG_BFP_OF_REAL((_ef).m[0]);		
	    (_ei).m[1] = TRIG_BFP_OF_REAL((_ef).m[1]);		
	    (_ei).m[2] = TRIG_BFP_OF_REAL((_ef).m[2]);		
	    (_ei).m[3] = TRIG_BFP_OF_REAL((_ef).m[3]);		
	    (_ei).m[4] = TRIG_BFP_OF_REAL((_ef).m[4]);		
	    (_ei).m[5] = TRIG_BFP_OF_REAL((_ef).m[5]);		
	    (_ei).m[6] = TRIG_BFP_OF_REAL((_ef).m[6]);		
	    (_ei).m[7] = TRIG_BFP_OF_REAL((_ef).m[7]);		
	    (_ei).m[8] = TRIG_BFP_OF_REAL((_ef).m[8]);		
	  }
	
}
