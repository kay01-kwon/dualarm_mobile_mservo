package epos_tutorial;

public interface realVel extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "epos_tutorial/realVel";
  static final java.lang.String _DEFINITION = "int32[4] realVel\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  int[] getRealVel();
  void setRealVel(int[] value);
}
