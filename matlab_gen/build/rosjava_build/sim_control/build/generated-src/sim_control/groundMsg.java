package sim_control;

public interface groundMsg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sim_control/groundMsg";
  static final java.lang.String _DEFINITION = "float64 x_gnd\nfloat64 y_gnd\nfloat64 vx_gnd\nfloat64 vy_gnd\nfloat64 qx_gnd\nfloat64 qy_gnd\nfloat64 qz_gnd\nfloat64 qw_gnd\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  double getXGnd();
  void setXGnd(double value);
  double getYGnd();
  void setYGnd(double value);
  double getVxGnd();
  void setVxGnd(double value);
  double getVyGnd();
  void setVyGnd(double value);
  double getQxGnd();
  void setQxGnd(double value);
  double getQyGnd();
  void setQyGnd(double value);
  double getQzGnd();
  void setQzGnd(double value);
  double getQwGnd();
  void setQwGnd(double value);
}
