package vehicle_control;

public interface noise extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "vehicle_control/noise";
  static final java.lang.String _DEFINITION = "float64 noise1\nfloat64 noise2\nfloat64 noise3\nfloat64 noise4\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  double getNoise1();
  void setNoise1(double value);
  double getNoise2();
  void setNoise2(double value);
  double getNoise3();
  void setNoise3(double value);
  double getNoise4();
  void setNoise4(double value);
}
