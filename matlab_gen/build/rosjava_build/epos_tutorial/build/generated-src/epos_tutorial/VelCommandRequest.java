package epos_tutorial;

public interface VelCommandRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "epos_tutorial/VelCommandRequest";
  static final java.lang.String _DEFINITION = "int64 Vel1\nint64 Vel2\nint64 Vel3\nint64 Vel4\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  long getVel1();
  void setVel1(long value);
  long getVel2();
  void setVel2(long value);
  long getVel3();
  void setVel3(long value);
  long getVel4();
  void setVel4(long value);
}
