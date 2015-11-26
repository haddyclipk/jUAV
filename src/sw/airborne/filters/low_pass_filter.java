package sw.airborne.filters;

public class low_pass_filter {
	//SecondOrderLowPass_int Butterworth2LowPass_int = new SecondOrderLowPass_int();
	public static long update_second_order_low_pass_int(SecondOrderLowPass_int filter, long value) {
		  long out = filter.b[0] * value
				    + filter.b[1] * filter.i[0]
				    + filter.b[0] * filter.i[1]
				    - filter.a[0] * filter.o[0]
				    - filter.a[1] * filter.o[1];

				  filter.i[1] = filter.i[0];
				  filter.i[0] = value;
				  filter.o[1] = filter.o[0];
				  filter.o[0] = out / (filter.loop_gain);
				  return filter.o[0];
				}
	
	public static long update_butterworth_2_low_pass_int(Butterworth2LowPass_int filter, long value) {
		  return (Long) null;//Commented for compilation's sake
		  					//update_second_order_low_pass_int(filter, value);//?????
	}
}
