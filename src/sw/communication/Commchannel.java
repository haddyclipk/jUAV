package sw.communication;
import java.util.StringTokenizer;

import sw.airborne.State;
import sw.airborne.math.FloatQuat;
import sw.airborne.math.FloatRates;
import sw.airborne.subsystems.Imu;
import fr.dgac.ivy.* ;
import devices.Gps;

public class Commchannel implements IvyMessageListener {
	private static Ivy bus;
	//Add buffer for GPS
	//Add buffer for AHRS
	//Buffer not added values updated directly to state as they come in --Buffer can be added later

	//The below method is not a constructor
	public static void CommChannel() throws IvyException {
		 bus = new Ivy("JUAV","JUAV Ready",null);
		 bus.bindMsg("GPS_CALCULAED_DEVICE(.*)",new IvyMessageListener() {
		      public void receive(IvyClient client, String[] args) {
			// Process GPS
		    	Long[] gpsState = new Long[17];
		    	int i=0;
		    	 StringTokenizer tokenizedString = new StringTokenizer(args[0], " ");
		    	 while (tokenizedString.hasMoreElements()) {
					String currentString = tokenizedString.nextToken();
					gpsState[i] = Long.parseLong(currentString);
					i++;
				}
		    	 Gps.gps_feed_value(gpsState);
		      }
		    });
		 bus.bindMsg("NPS_SENSORS_GYRO_ACCEL(.*)",new IvyMessageListener() {
		      public void receive(IvyClient client, String[] args) {
			// Process AHRS
		    	  long[] gyroUnscaled = new long[6];
		    	  int i=0;
			    	 StringTokenizer tokenizedString = new StringTokenizer(args[0], " ");
			    	 while (tokenizedString.hasMoreElements()) {
						String currentString = tokenizedString.nextToken();
						gyroUnscaled[i] = Long.parseLong(currentString);
						i++;
					}
			    	 Imu.imu_feed_gyro_accel(gyroUnscaled);
		      }
		    });
		 bus.bindMsg("^NPS_AHRS_LTP_ECEF(.*)",new IvyMessageListener() {
		      public void receive(IvyClient client, String[] args) {
			// Process AHRS
		    	  double[] ahrsLtpEcef = new double[7];
		    	  int i=0;
			    	 StringTokenizer tokenizedString = new StringTokenizer(args[0], " ");
			    	 while (tokenizedString.hasMoreElements()) {
						String currentString = tokenizedString.nextToken();
						ahrsLtpEcef[i] = Double.parseDouble(currentString);
						i++;
					}
			    	 FloatQuat quat_f = new FloatQuat();
			    	 quat_f.qi = (float) ahrsLtpEcef[0];
			    	 quat_f.qx = (float) ahrsLtpEcef[1];
			    	 quat_f.qy = (float) ahrsLtpEcef[2];
			    	 quat_f.qz = (float) ahrsLtpEcef[3];
			    	 State.stateSetNedToBodyQuat_f(quat_f);
			    	 FloatRates rates_f = new FloatRates();
			    	 rates_f.p = (float) ahrsLtpEcef[4];
			    	 rates_f.q = (float) ahrsLtpEcef[5];
			    	 rates_f.r = (float) ahrsLtpEcef[6];
			    	 State.stateSetBodyRates_f(rates_f);
		      }
		    });
		 bus.start(null); // starts the bus on the default domain
	}

	private void close(){
		bus.stop();
	}

	protected void finalize() throws Throwable {
	     try {
	         close();        // close open files
	     } finally {
	         super.finalize();
	     }
	 }
	
	@Override
	public void receive(IvyClient arg0, String[] arg1) {
		// TODO Auto-generated method stub
		
	}
}

