package vehicle_control;

public interface gnd extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "vehicle_control/gnd";
  static final java.lang.String _DEFINITION = "float64 x_gnd\nfloat64 y_gnd\nfloat64 q_x\nfloat64 q_y\nfloat64 q_z\nfloat64 q_w\nfloat64 vel_x\nfloat64 vel_y\nfloat64 vel_phi\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  double getXGnd();
  void setXGnd(double value);
  double getYGnd();
  void setYGnd(double value);
  double getQX();
  void setQX(double value);
  double getQY();
  void setQY(double value);
  double getQZ();
  void setQZ(double value);
  double getQW();
  void setQW(double value);
  double getVelX();
  void setVelX(double value);
  double getVelY();
  void setVelY(double value);
  double getVelPhi();
  void setVelPhi(double value);
}
