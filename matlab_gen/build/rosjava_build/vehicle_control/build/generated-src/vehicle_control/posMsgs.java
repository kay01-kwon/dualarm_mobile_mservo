package vehicle_control;

public interface posMsgs extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "vehicle_control/posMsgs";
  static final java.lang.String _DEFINITION = "float64 x_est\nfloat64 y_est\nfloat64 phi_est\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  double getXEst();
  void setXEst(double value);
  double getYEst();
  void setYEst(double value);
  double getPhiEst();
  void setPhiEst(double value);
}
