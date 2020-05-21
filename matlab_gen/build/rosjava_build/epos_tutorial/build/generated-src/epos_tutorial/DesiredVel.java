package epos_tutorial;

public interface DesiredVel extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "epos_tutorial/DesiredVel";
  static final java.lang.String _DEFINITION = "int32 vel1\nint32 vel2\nint32 vel3\nint32 vel4\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  int getVel1();
  void setVel1(int value);
  int getVel2();
  void setVel2(int value);
  int getVel3();
  void setVel3(int value);
  int getVel4();
  void setVel4(int value);
}
