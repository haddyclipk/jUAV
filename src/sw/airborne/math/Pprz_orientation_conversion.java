package sw.airborne.math;

import sw.include.Std;
import static sw.airborne.math.Pprz_algebra_int.*;
import static sw.airborne.math.Pprz_algebra.*;
import static sw.include.Std.*;
import static sw.airborne.math.Pprz_geodetic.*;
import static sw.airborne.math.Pprz_algebra_float.*;

public class Pprz_orientation_conversion {
	
	public static final int ORREP_QUAT_I = 0;
	public static final int ORREP_EULER_F = 4;
	public static final int ORREP_EULER_I = 1;
	public static final int ORREP_RMAT_I = 2;
	public static final int ORREP_QUAT_F = 3;
	public static final int ORREP_RMAT_F = 5;
	
	
	public static boolean orientationCheckValid(OrientationReps orientation){
		return orientation.status == 1;
	}

	public static void orientationSetQuat_i( OrientationReps orientation, Int32Quat quat) {
		  QUAT_COPY(orientation.quat_i, quat);
		  /* clear bits for all attitude representations and only set the new one */
		  orientation.status = (1 << ORREP_QUAT_I);
		}
	
	/// Get vehicle body attitude euler angles (float).
	public static FloatEulers orientationGetEulers_f( OrientationReps orientation) {
	  if (!bit_is_set(orientation.status, ORREP_EULER_F))
	    orientationCalcEulers_f(orientation);
	  return orientation.eulers_f;
	}
	
	public static void orientationCalcEulers_f( OrientationReps orientation) {
		  if (bit_is_set(orientation.status, ORREP_EULER_F))
		    return;

		  if (bit_is_set(orientation.status, ORREP_EULER_I)) {
		    EULERS_FLOAT_OF_BFP(orientation.eulers_f, orientation.eulers_i);
		  }
		  else if (bit_is_set(orientation.status, ORREP_RMAT_F)) {
		    FLOAT_EULERS_OF_RMAT(orientation.eulers_f, orientation.rmat_f);
		  }
		  else if (bit_is_set(orientation.status, ORREP_QUAT_F)) {
		    FLOAT_EULERS_OF_QUAT(orientation.eulers_f, orientation.quat_f);
		  }
		  else if (bit_is_set(orientation.status, ORREP_RMAT_I)) {
		    RMAT_FLOAT_OF_BFP(orientation.rmat_f, orientation.rmat_i);
		    SetBit(orientation.status, ORREP_RMAT_F);
		    FLOAT_EULERS_OF_RMAT(orientation.eulers_f, orientation.rmat_f);
		  }
		  else if (bit_is_set(orientation.status, ORREP_QUAT_I)) {
		    QUAT_FLOAT_OF_BFP(orientation.quat_f, orientation.quat_i);
		    SetBit(orientation.status, ORREP_QUAT_F);
		    FLOAT_EULERS_OF_QUAT(orientation.eulers_f, orientation.quat_f);
		  }
		  /* set bit to indicate this representation is computed */
		  SetBit(orientation.status, ORREP_EULER_F);
		}
	
	public static FloatRMat orientationGetRMat_f(OrientationReps orientation) {
		  if (!bit_is_set(orientation.status, ORREP_RMAT_F))
		    orientationCalcRMat_f(orientation);
		  return orientation.rmat_f;
		}
	
	public static void orientationCalcRMat_f( OrientationReps orientation) {
		if (bit_is_set(orientation.status, ORREP_RMAT_F))
			return;

		if (bit_is_set(orientation.status, ORREP_RMAT_I)) {
			RMAT_FLOAT_OF_BFP(orientation.rmat_f, orientation.rmat_i);
		}
		else if (bit_is_set(orientation.status, ORREP_QUAT_F)) {
			FLOAT_RMAT_OF_QUAT(orientation.rmat_f, orientation.quat_f);
		}
		else if (bit_is_set(orientation.status, ORREP_EULER_F)) {
			FLOAT_RMAT_OF_EULERS(orientation.rmat_f, orientation.eulers_f);
		}
		else if (bit_is_set(orientation.status, ORREP_QUAT_I)) {
			QUAT_FLOAT_OF_BFP(orientation.quat_f, orientation.quat_i);
			SetBit(orientation.status, ORREP_QUAT_F);
			FLOAT_RMAT_OF_QUAT(orientation.rmat_f, orientation.quat_f);
		}
		else if (bit_is_set(orientation.status, ORREP_EULER_I)) {
			EULERS_FLOAT_OF_BFP(orientation.eulers_f, orientation.eulers_i);
			SetBit(orientation.status, ORREP_EULER_F);
			FLOAT_RMAT_OF_EULERS(orientation.rmat_f, orientation.eulers_f);
		}
		/* set bit to indicate this representation is computed */
		SetBit(orientation.status, ORREP_RMAT_F);
	}
	
	/// Get vehicle body attitude quaternion (float).
	public static FloatQuat orientationGetQuat_f(OrientationReps orientation) {
	  if (!bit_is_set(orientation.status, ORREP_QUAT_F))
	    orientationCalcQuat_f(orientation);
	  return orientation.quat_f;
	}
	
	public static void orientationCalcQuat_f(OrientationReps orientation) {
		  if (bit_is_set(orientation.status, ORREP_QUAT_F))
		    return;

		  if (bit_is_set(orientation.status, ORREP_QUAT_I)) {
		    QUAT_FLOAT_OF_BFP(orientation.quat_f, orientation.quat_i);
		  }
		  else if (bit_is_set(orientation.status, ORREP_RMAT_F)) {
		    FLOAT_QUAT_OF_RMAT(orientation.quat_f, orientation.rmat_f);
		  }
		  else if (bit_is_set(orientation.status, ORREP_EULER_F)) {
		    FLOAT_QUAT_OF_EULERS(orientation.quat_f, orientation.eulers_f);
		  }
		  else if (bit_is_set(orientation.status, ORREP_RMAT_I)) {
		    RMAT_FLOAT_OF_BFP(orientation.rmat_f, orientation.rmat_i);
		    SetBit(orientation.status, ORREP_RMAT_F);
		    FLOAT_QUAT_OF_RMAT(orientation.quat_f, orientation.rmat_f);
		  }
		  else if (bit_is_set(orientation.status, ORREP_EULER_I)) {
		    EULERS_FLOAT_OF_BFP(orientation.eulers_f, orientation.eulers_i);
		    SetBit(orientation.status, ORREP_EULER_F);
		    FLOAT_QUAT_OF_EULERS(orientation.quat_f, orientation.eulers_f);
		  }
		  /* set bit to indicate this representation is computed */
		  SetBit(orientation.status, ORREP_QUAT_F);
		}
	
	/// Get vehicle body attitude euler angles (int).
	public static  Int32Eulers orientationGetEulers_i( OrientationReps orientation) {
	  if (!bit_is_set(orientation.status, ORREP_EULER_I))
	    orientationCalcEulers_i(orientation);
	  return orientation.eulers_i;
	}
	
	public static void orientationCalcEulers_i( OrientationReps orientation) {
		  if (bit_is_set(orientation.status, ORREP_EULER_I))
		    return;

		  if (bit_is_set(orientation.status, ORREP_EULER_F)) {
		    EULERS_BFP_OF_REAL(orientation.eulers_i, orientation.eulers_f);
		  }
		  else if (bit_is_set(orientation.status, ORREP_RMAT_I)) {
		    INT32_EULERS_OF_RMAT(orientation.eulers_i, orientation.rmat_i);
		  }
		  else if (bit_is_set(orientation.status, ORREP_QUAT_I)) {
		    INT32_EULERS_OF_QUAT(orientation.eulers_i, orientation.quat_i);
		  }
		  else if (bit_is_set(orientation.status, ORREP_RMAT_F)) {
		    RMAT_BFP_OF_REAL(orientation.rmat_i, orientation.rmat_f);
		    SetBit(orientation.status, ORREP_RMAT_I);
		    INT32_EULERS_OF_RMAT(orientation.eulers_i, orientation.rmat_i);
		  }
		  else if (bit_is_set(orientation.status, ORREP_QUAT_F)) {
		    QUAT_BFP_OF_REAL(orientation.quat_i, orientation.quat_f);
		    SetBit(orientation.status, ORREP_QUAT_I);
		    INT32_EULERS_OF_QUAT(orientation.eulers_i, orientation.quat_i);
		  }
		  /* set bit to indicate this representation is computed */
		  SetBit(orientation.status, ORREP_EULER_I);
		}
	
	/// Get vehicle body attitude rotation matrix (int).
	public static Int32RMat orientationGetRMat_i(OrientationReps orientation) {
	  if (!bit_is_set(orientation.status, ORREP_RMAT_I))
	    orientationCalcRMat_i(orientation);
	  return orientation.rmat_i;
	}
	
	public static void orientationCalcRMat_i( OrientationReps orientation) {
		  if (bit_is_set(orientation.status, ORREP_RMAT_I))
		    return;

		  if (bit_is_set(orientation.status, ORREP_RMAT_F)) {
		    RMAT_BFP_OF_REAL(orientation.rmat_i, orientation.rmat_f);
		  }
		  else if (bit_is_set(orientation.status, ORREP_QUAT_I)) {
		    INT32_RMAT_OF_QUAT(orientation.rmat_i, orientation.quat_i);
		  }
		  else if (bit_is_set(orientation.status, ORREP_EULER_I)) {
		    INT32_RMAT_OF_EULERS(orientation.rmat_i, orientation.eulers_i);
		  }
		  else if (bit_is_set(orientation.status, ORREP_QUAT_F)) {
		    QUAT_BFP_OF_REAL(orientation.quat_i, orientation.quat_f);
		    SetBit(orientation.status, ORREP_QUAT_I);
		    INT32_RMAT_OF_QUAT(orientation.rmat_i, orientation.quat_i);
		  }
		  else if (bit_is_set(orientation.status, ORREP_EULER_F)) {
		    EULERS_BFP_OF_REAL(orientation.eulers_i, orientation.eulers_f);
		    SetBit(orientation.status, ORREP_EULER_I);
		    INT32_RMAT_OF_EULERS(orientation.rmat_i, orientation.eulers_i);
		  }
		  /* set bit to indicate this representation is computed */
		  SetBit(orientation.status, ORREP_RMAT_I);
		}
	
	/// Get vehicle body attitude quaternion (int).
	public static Int32Quat orientationGetQuat_i(OrientationReps orientation) {
	  if (!bit_is_set(orientation.status, ORREP_QUAT_I))
	    orientationCalcQuat_i(orientation);
	  return orientation.quat_i;
	}

	public static void orientationCalcQuat_i( OrientationReps orientation) {
		if (bit_is_set(orientation.status, ORREP_QUAT_I))
			return;

		if (bit_is_set(orientation.status, ORREP_QUAT_F)) {
			QUAT_BFP_OF_REAL(orientation.quat_i, orientation.quat_f);
		}
		else if (bit_is_set(orientation.status, ORREP_RMAT_I)) {
			INT32_QUAT_OF_RMAT(orientation.quat_i, orientation.rmat_i);
		}
		else if (bit_is_set(orientation.status, ORREP_EULER_I)) {
			INT32_QUAT_OF_EULERS(orientation.quat_i, orientation.eulers_i);
		}
		else if (bit_is_set(orientation.status, ORREP_RMAT_F)) {
			RMAT_BFP_OF_REAL(orientation.rmat_i, orientation.rmat_f);
			SetBit(orientation.status, ORREP_RMAT_I);
			INT32_QUAT_OF_RMAT(orientation.quat_i, orientation.rmat_i);
		}
		else if (bit_is_set(orientation.status, ORREP_EULER_F)) {
			EULERS_BFP_OF_REAL(orientation.eulers_i, orientation.eulers_f);
			SetBit(orientation.status, ORREP_EULER_I);
			INT32_QUAT_OF_EULERS(orientation.quat_i, orientation.eulers_i);
		}
		/* set bit to indicate this representation is computed */
		SetBit(orientation.status, ORREP_QUAT_I);
	}
	
	
	/// Set vehicle body attitude from euler angles (float).
	public static void orientationSetEulers_f(OrientationReps orientation,  FloatEulers eulers) {
		EULERS_COPY(orientation.eulers_f, eulers);
		/* clear bits for all attitude representations and only set the new one */
		orientation.status = (1 << ORREP_EULER_F);
	}

	/// Set vehicle body attitude from rotation matrix (float).
	public static  void orientationSetRMat_f( OrientationReps orientation,  FloatRMat rmat) {
		RMAT_COPY(orientation.rmat_f, rmat);
		/* clear bits for all attitude representations and only set the new one */
		orientation.status = (1 << ORREP_RMAT_F);
	}
	/// Set vehicle body attitude from rotation matrix (int).
	public static  void orientationSetRMat_i( OrientationReps orientation,  Int32RMat rmat) {
		RMAT_COPY(orientation.rmat_i, rmat);
		/* clear bits for all attitude representations and only set the new one */
		orientation.status = (1 << ORREP_RMAT_I);
	}

	/// Set vehicle body attitude from euler angles (int).
	public static  void orientationSetEulers_i( OrientationReps orientation,  Int32Eulers eulers) {
		EULERS_COPY(orientation.eulers_i, eulers);
		/* clear bits for all attitude representations and only set the new one */
		orientation.status = (1 << ORREP_EULER_I);
	}

	/// Set vehicle body attitude from quaternion (float).
	public static  void orientationSetQuat_f( OrientationReps orientation,  FloatQuat quat) {
		QUAT_COPY(orientation.quat_f, quat);
		/* clear bits for all attitude representations and only set the new one */
		orientation.status = (1 << ORREP_QUAT_F);
	}
}
