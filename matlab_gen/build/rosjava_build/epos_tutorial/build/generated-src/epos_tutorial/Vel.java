package epos_tutorial;

public interface Vel extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "epos_tutorial/Vel";
  static final java.lang.String _DEFINITION = "int32 num\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  int getNum();
  void setNum(int value);
}
