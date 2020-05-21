package vehicle_control;

public interface jointstatesMsg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "vehicle_control/jointstatesMsg";
  static final java.lang.String _DEFINITION = "float64 fl\nfloat64 fr\nfloat64 bl\nfloat64 br\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  double getFl();
  void setFl(double value);
  double getFr();
  void setFr(double value);
  double getBl();
  void setBl(double value);
  double getBr();
  void setBr(double value);
}
