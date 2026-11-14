package io.github.frcgokuimu;

/** Desk test of the serial link + parser without WPILib natives: java LinkTest COM6 */
public class LinkTest {
  public static void main(String[] args) throws Exception {
    final Protocol.State[] last = new Protocol.State[1];
    final int[] count = new int[1];
    final byte[][] reply = new byte[1][];
    SerialLink link = new SerialLink(args.length > 0 ? args[0] : "", (t, p) -> {
      if (t == Protocol.MSG_STATE) { last[0] = Protocol.State.parse(p); count[0]++; }
      else { reply[0] = p; System.out.println("reply type " + t + " len " + p.length); }
    }, () -> System.out.println("connected"));
    Thread.sleep(1500);
    System.out.println("frames " + count[0] + " last " + last[0]);
    link.send(Protocol.CMD_SET_YAW, Protocol.le(new byte[4]).putFloat(123.5f).array());
    Thread.sleep(1200);
    System.out.println("yaw after setYaw(123.5): " + last[0].yaw() + " q=" + last[0].qw() + "," + last[0].qz());
    link.send(Protocol.CMD_GET_CONFIG, null);
    Thread.sleep(1200);
    GokuIMUConfiguration c = new GokuIMUConfiguration();
    Protocol.decodeConfig(reply[0], c);
    System.out.println(c);
    link.send(Protocol.CMD_SET_YAW, Protocol.le(new byte[4]).putFloat(0f).array());
    Thread.sleep(1200);
    link.close();
  }
}
