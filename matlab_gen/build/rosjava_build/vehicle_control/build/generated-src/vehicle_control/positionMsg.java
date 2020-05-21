package vehicle_control;

public interface positionMsg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "vehicle_control/positionMsg";
  static final java.lang.String _DEFINITION = "float64 x\nfloat64 y\nfloat64 qx\nfloat64 qy\nfloat64 qz\nfloat64 qw\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  double getX();
  void setX(double value);
  double getY();
  void setY(double value);
  double getQx();
  void setQx(double value);
  double getQy();
  void setQy(double value);
  double getQz();
  void setQz(double value);
  double getQw();
  void setQw(double value);
}
