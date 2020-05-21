package vehicle_control;

public interface timeLoop extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "vehicle_control/timeLoop";
  static final java.lang.String _DEFINITION = "float64 t";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  double getT();
  void setT(double value);
}
