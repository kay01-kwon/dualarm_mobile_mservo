package sim_control;

public interface desiredMsg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sim_control/desiredMsg";
  static final java.lang.String _DEFINITION = "float64 x_des\nfloat64 y_des\n\nfloat64 vx_des\nfloat64 vy_des\nfloat64 vphi_des\n\nfloat64 phi_des\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  double getXDes();
  void setXDes(double value);
  double getYDes();
  void setYDes(double value);
  double getVxDes();
  void setVxDes(double value);
  double getVyDes();
  void setVyDes(double value);
  double getVphiDes();
  void setVphiDes(double value);
  double getPhiDes();
  void setPhiDes(double value);
}
