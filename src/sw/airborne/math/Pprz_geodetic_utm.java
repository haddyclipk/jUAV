package sw.airborne.math;

import static sw.include.Std.*;

public class Pprz_geodetic_utm {
	public static final double E = 0.08181919106;
	public static final double K0 = 0.9996;
	public static final double DELTA_EAST =  500000.;
	public static final double DELTA_NORTH = 0. ;
	public static final double A = 6378137.0;
	public static final double N = (K0*A);
	
	public static double LambdaOfUtmZone(double utm_zone){
		return RadOfDeg((utm_zone -1) * 6 -180+3);
	}
	public static double LambdaOfUtmZone(float utm_zone){
		return RadOfDeg((utm_zone -1) * 6 -180+3);
	}
	
	public static final float[] serie_coeff_proj_mercator = {
		(float)0.99832429842242842444,
		(float)0.00083632803657738403,
		(float) 0.00000075957783563707,
		(float) 0.00000000119563131778,
		(float) 0.00000000000241079916
	};

	public static final float[] serie_coeff_proj_mercator_inverse = {
		(float)0.998324298422428424,
		(float) 0.000837732168742475825,
		(float)  5.90586914811817062e-08,
		(float)  1.6734091890305064e-10,
		(float)  2.13883575853313883e-13
	};
}
