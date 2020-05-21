package path_planner;

public interface des_traj extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "path_planner/des_traj";
  static final java.lang.String _DEFINITION = "float64 x_d\nfloat64 y_d\nfloat64 phi_d\n\nfloat64 vx_d\nfloat64 vy_d\nfloat64 vphi_d\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  double getXD();
  void setXD(double value);
  double getYD();
  void setYD(double value);
  double getPhiD();
  void setPhiD(double value);
  double getVxD();
  void setVxD(double value);
  double getVyD();
  void setVyD(double value);
  double getVphiD();
  void setVphiD(double value);
}
