package sim_control;

public interface motorDynamics extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sim_control/motorDynamics";
  static final java.lang.String _DEFINITION = "int32 omega1\nint32 omega2\nint32 omega3\nint32 omega4\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  int getOmega1();
  void setOmega1(int value);
  int getOmega2();
  void setOmega2(int value);
  int getOmega3();
  void setOmega3(int value);
  int getOmega4();
  void setOmega4(int value);
}
