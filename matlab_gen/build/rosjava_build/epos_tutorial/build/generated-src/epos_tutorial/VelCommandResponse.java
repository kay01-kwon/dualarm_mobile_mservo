package epos_tutorial;

public interface VelCommandResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "epos_tutorial/VelCommandResponse";
  static final java.lang.String _DEFINITION = "int64 setVel1\nint64 setVel2\nint64 setVel3\nint64 setVel4";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  long getSetVel1();
  void setSetVel1(long value);
  long getSetVel2();
  void setSetVel2(long value);
  long getSetVel3();
  void setSetVel3(long value);
  long getSetVel4();
  void setSetVel4(long value);
}
