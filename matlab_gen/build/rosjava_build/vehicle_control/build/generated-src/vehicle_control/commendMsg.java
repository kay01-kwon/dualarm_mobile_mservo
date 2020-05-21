package vehicle_control;

public interface commendMsg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "vehicle_control/commendMsg";
  static final java.lang.String _DEFINITION = "\nfloat64 xd\nfloat64 yd\nfloat64 phid\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  double getXd();
  void setXd(double value);
  double getYd();
  void setYd(double value);
  double getPhid();
  void setPhid(double value);
}
