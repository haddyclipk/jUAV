package sw.include;

public class Std {
	
	public static final double M_PI = 3.14159265358979323846;
	
	public static final double M_PI_4 = (M_PI/4);
	
	public static final double M_PI_2 = (M_PI/2);
	
//	public static int cbi(int sfr, int bit){
//		return (_SFR_BYTE(sfr) &= ~_BV(bit));
//	}
//	
//	public static int sbi(int sft, int bit){
//		return (_SFR_BYTE(sfr) |= _BV(bit));
//	}
	
	public static boolean bit_is_set(int x, int b){
		return ((x >> b) & 0x1) == 1;
	}
	
	public static double RadOfDeg(int x){
		return ((x) * (M_PI/180.));
	}
	
	public static int _BV(int bit){
		return (1 << (bit));
	}
	
	public static int  SetBit(int a, int n) {
		return a |= (1 << n);
	}
	
	public static int ClearBit(int a, int n){
		return a &= ~(1 << n);
	}
	
	public static void NormRadAngle(int x) { 
	    while (x > M_PI) x -= 2 * M_PI; 
	    while (x < -M_PI) x += 2 * M_PI; 
	  }
	
	public static double DegOfRad(double x){
		return ((x) * (180. / M_PI));
	}
	
	public static double DeciDegOfRad(double x){
		return ((x) * (1800./ M_PI));
	}
	
	public static double RadOfDeg(double x){
		return ((x) * (M_PI/180.));
	}
	
	public static int Min(int x, int y){
		return (x < y ? x : y);
	}
	
	
	public static int Max(int x, int y){
		return (x > y ? x : y);
	}
	public static long Max(long x, int y){
		return (x > y ? x : y);
	}
	
	public static int ABS(int val){
		return ((val) < 0 ? -(val) : (val));
	}
	
	public static void Bound(int _x, int _min, int _max){
		 if (_x > _max) _x = _max; else if (_x < _min) _x = _min;
	}
	public static void Bound(long _x, int _min, int _max){
		if (_x > _max) _x = _max; else if (_x < _min) _x = _min;
	}
	
	public static void Bound(long _x, long _min, long _max){
		if (_x > _max) _x = _max; else if (_x < _min) _x = _min;
	}
	
	public static void BoundInverted(int _x, int _min, int _max){
		if ((_x < _min) && (_x > _max)) {             
			if (ABS(_x - _min) < ABS(_x - _max))        
				_x = _min;                                
			else                                        
				_x = _max;                                
		}      
	}
	
	public static void BoundWrapped(int _x, int _min, int _max){
		if (_max > _min)                              
	      Bound(_x, _min, _max);                       
	    else                                          
	      BoundInverted(_x, _min, _max);
	}
	
	public static void BoundAbs(int _x,int  _max){
		Bound(_x, -(_max), (_max));
	}
	public static void BoundAbs(long  _x,long  _max){
		Bound(_x, -(_max), (_max));
	}
	
	public static int Chop(int _x, int _min, int _max){
		return ( (_x) < (_min) ? (_min) : (_x) > (_max) ? (_max) : (_x) );
	}
	
	public static int ChopAbs(int x, int max){
		return Chop(x, -max, max);
	}
	

	public static void DeadBand(int _x,int _v) {						
		if (_x > _v)							
			_x = _x -_v;							
		else if  (_x < -_v)							
			_x = _x +_v;							
		else								
			_x = 0;								
	}
	
	public static int Blend(int a, int b, int rho){
		return (((rho)*(a))+(1-(rho))*(b));
	}
}
