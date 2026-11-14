package io.github.frcgokuimu;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.RandomAccessFile;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.function.BiConsumer;

/**
 * Background USB-CDC link to a GokuIMU. Pure Java: on Linux (Systemcore) it opens the
 * /dev/ttyACM* device directly and puts it in raw mode with stty; on Windows it opens \\.\COMx.
 * Reconnects automatically after unplug or device reboot. All I/O happens on its own daemon
 * thread; sending only queues a frame. The one exception is the static {@link #listDevices()},
 * which reads sysfs (Linux) or runs {@code reg query} (Windows) on the caller's thread.
 */
final class SerialLink implements AutoCloseable {
  static final boolean WINDOWS =
      System.getProperty("os.name", "").toLowerCase(Locale.ROOT).contains("win");
  static final int USB_VID = 0x1209;
  static final int USB_PID = 0x0001;

  private final String m_selector;
  private final BiConsumer<Integer, byte[]> m_handler;
  private final Runnable m_onConnect;
  private final Thread m_thread;
  private volatile boolean m_running = true;
  private volatile RandomAccessFile m_file;
  private volatile String m_openPath;
  // Incremented each time a port is opened. Queued frames carry the generation they were sent
  // for, and the writer drops any frame from an older connection, so a command that robot code
  // was already told failed (on reconnect) can never reach the newly connected IMU.
  private volatile int m_generation;
  private int m_candidateIndex;

  static {
    if (!WINDOWS) {
      ignoreSighup();
    }
  }

  /**
   * Java can't open a tty with O_NOCTTY. If the robot process has no controlling terminal, the
   * IMU's tty can become it, and unplugging the cable would then send SIGHUP and kill the robot
   * program. Ignore SIGHUP (via reflection: sun.misc.Signal is in jdk.unsupported).
   */
  private static void ignoreSighup() {
    try {
      Class<?> sig = Class.forName("sun.misc.Signal");
      Class<?> handler = Class.forName("sun.misc.SignalHandler");
      Object ign = handler.getField("SIG_IGN").get(null);
      Object hup = sig.getConstructor(String.class).newInstance("HUP");
      sig.getMethod("handle", sig, handler).invoke(null, hup, ign);
    } catch (ReflectiveOperationException | RuntimeException e) {
      System.err.println("GokuIMU: could not ignore SIGHUP: " + e);
    }
  }

  /**
   * @param selector null/empty for the first GokuIMU found, a serial number (or suffix of one), or
   *     an explicit device path such as /dev/ttyACM0 or COM6.
   */
  SerialLink(String selector, BiConsumer<Integer, byte[]> handler, Runnable onConnect) {
    m_selector = selector == null ? "" : selector.trim();
    m_handler = handler;
    m_onConnect = onConnect;
    m_thread = new Thread(this::run, "GokuIMU-" + (m_selector.isEmpty() ? "auto" : m_selector));
    m_thread.setDaemon(true);
    m_thread.start();
  }

  String getOpenPath() {
    return m_openPath;
  }

  boolean isOpen() {
    return m_file != null;
  }

  /**
   * Queues one already-built frame for the writer thread. Never blocks: the OS write can stall
   * (for example on Windows while the IMU is busy erasing flash), so it happens off the caller's
   * thread. Returns false if no IMU is open or the queue is full.
   */
  boolean sendFrame(byte[] frame) {
    if (m_file == null) {
      return false;
    }
    startWriter();
    return m_writeQueue.offer(new Tx(m_generation, frame));
  }

  private record Tx(int generation, byte[] frame) {}

  private final java.util.concurrent.LinkedBlockingQueue<Tx> m_writeQueue =
      new java.util.concurrent.LinkedBlockingQueue<>(64);
  private Thread m_writer;

  private synchronized void startWriter() {
    if (m_writer != null) {
      return;
    }
    m_writer = new Thread(this::writeLoop, m_thread.getName() + "-tx");
    m_writer.setDaemon(true);
    m_writer.start();
  }

  private void writeLoop() {
    while (m_running) {
      Tx tx;
      try {
        tx = m_writeQueue.poll(500, TimeUnit.MILLISECONDS);
      } catch (InterruptedException e) {
        return;
      }
      RandomAccessFile f = m_file;
      if (tx == null || f == null || tx.generation() != m_generation) {
        continue; // nothing to send, IMU gone, or queued for an old connection: drop it
      }
      try {
        f.write(tx.frame());
      } catch (IOException e) {
        // port closing; the reader thread handles reconnects
      }
    }
  }

  /**
   * Stops the link without blocking the caller. The port is closed on a short-lived daemon thread,
   * because on Windows closing a COM handle can wait for a read in progress to finish.
   */
  @Override
  public void close() {
    m_running = false;
    RandomAccessFile f = m_file;
    if (f != null) {
      Thread closer =
          new Thread(
              () -> {
                try {
                  f.close();
                } catch (IOException e) {
                  // ignore
                }
              },
              "GokuIMU-close");
      closer.setDaemon(true);
      closer.start();
    }
    m_thread.interrupt();
  }

  private void run() {
    // Windows COM reads block until the buffer is full, so use small reads there to keep reply
    // latency low; Linux raw ttys return as soon as any data arrives.
    byte[] buf = new byte[WINDOWS ? 64 : 4096];
    Parser parser = new Parser();
    while (m_running) {
      String path = nextCandidate();
      if (path == null) {
        sleep(500);
        continue;
      }
      if (!WINDOWS) {
        configureRaw(path); // before opening, so nothing is echoed back to the IMU
      }
      try (RandomAccessFile f = new RandomAccessFile(WINDOWS ? "\\\\.\\" + path : path, "rw")) {
        m_writeQueue.clear(); // drop anything queued for a previous connection
        m_generation++; // and anything the writer already took off the queue
        m_openPath = path;
        m_file = f;
        m_onConnect.run();
        while (m_running) {
          int n = f.read(buf);
          if (n < 0) {
            break;
          }
          parser.feed(buf, n);
        }
      } catch (IOException e) {
        // unplugged, rebooting, or a stale registry entry: rotate to the next candidate
        // (harmless when there is only one)
        m_candidateIndex++;
      } finally {
        m_file = null;
        m_openPath = null;
      }
      sleep(250);
    }
  }

  private static void sleep(long ms) {
    try {
      Thread.sleep(ms);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
  }

  private static void configureRaw(String path) {
    try {
      Process p =
          new ProcessBuilder(
                  "stty", "-F", path, "raw", "-echo", "-echoe", "-echok", "-echoctl", "-echoke",
                  "-ixon", "-ixoff", "-crtscts", "clocal", "115200")
              .redirectErrorStream(true)
              .start();
      if (!p.waitFor(2, TimeUnit.SECONDS)) {
        p.destroyForcibly();
      }
    } catch (IOException | InterruptedException e) {
      System.err.println("GokuIMU: could not set raw mode with stty on " + path + ": " + e);
    }
  }

  // ---------------- device discovery ----------------

  private String nextCandidate() {
    String sel = m_selector;
    if (sel.startsWith("/dev/") || sel.toUpperCase(Locale.ROOT).matches("COM\\d+")) {
      return sel;
    }
    List<String> matches = new ArrayList<>();
    for (String[] d : listDevices()) {
      if (sel.isEmpty() || d[1].toUpperCase(Locale.ROOT).endsWith(sel.toUpperCase(Locale.ROOT))) {
        matches.add(d[0]);
      }
    }
    if (matches.isEmpty()) {
      return null;
    }
    return matches.get(Math.floorMod(m_candidateIndex, matches.size()));
  }

  /**
   * Returns {devicePath, serialNumber} for every GokuIMU the OS knows about. Linux: attached
   * devices from sysfs. Windows: from the registry, which can include unplugged boards (the link
   * skips ports that fail to open). This runs a subprocess on Windows and can take ~100 ms.
   */
  static List<String[]> listDevices() {
    return WINDOWS ? listWindows() : listLinux();
  }

  private static List<String[]> listLinux() {
    List<String[]> out = new ArrayList<>();
    File[] ttys = new File("/sys/class/tty").listFiles((d, n) -> n.startsWith("ttyACM"));
    if (ttys == null) {
      return out;
    }
    Arrays.sort(ttys);
    for (File tty : ttys) {
      // /sys/class/tty/ttyACMn/device -> interface dir; parent is the USB device dir.
      Path usbDev = tty.toPath().resolve("device").resolve("..");
      String product = readSys(usbDev.resolve("product"));
      if (product != null && product.startsWith("GokuIMU")) {
        String serial = readSys(usbDev.resolve("serial"));
        out.add(new String[] {"/dev/" + tty.getName(), serial == null ? "" : serial});
      }
    }
    return out;
  }

  private static String readSys(Path p) {
    try {
      return Files.readString(p.toRealPath(), StandardCharsets.US_ASCII).trim();
    } catch (IOException e) {
      return null;
    }
  }

  // Windows: HKLM\SYSTEM\CurrentControlSet\Enum\USB\VID_1209&PID_0001\<serial> holds
  // ParentIdPrefix; ...\VID_1209&PID_0001&MI_00\<prefix>&0000\Device Parameters holds PortName.
  private static List<String[]> listWindows() {
    String base =
        String.format(
            "HKLM\\SYSTEM\\CurrentControlSet\\Enum\\USB\\VID_%04X&PID_%04X", USB_VID, USB_PID);
    Map<String, String> prefixToSerial = new HashMap<>();
    String key = null;
    for (String line : regQuery(base)) {
      if (line.startsWith("HKEY_")) {
        key = line;
      } else if (key != null && line.trim().startsWith("ParentIdPrefix")) {
        String[] parts = line.trim().split("\\s+");
        String serial = key.substring(key.lastIndexOf('\\') + 1);
        prefixToSerial.put(parts[parts.length - 1].toUpperCase(Locale.ROOT), serial);
      }
    }
    List<String[]> out = new ArrayList<>();
    key = null;
    for (String line : regQuery(base + "&MI_00")) {
      if (line.startsWith("HKEY_")) {
        key = line;
      } else if (key != null && line.trim().startsWith("PortName")) {
        String[] parts = line.trim().split("\\s+");
        String port = parts[parts.length - 1];
        // key ends with ...\<instance>\Device Parameters
        String[] segs = key.split("\\\\");
        String instance = segs.length >= 2 ? segs[segs.length - 2].toUpperCase(Locale.ROOT) : "";
        String serial = "";
        for (var e : prefixToSerial.entrySet()) {
          if (instance.startsWith(e.getKey())) {
            serial = e.getValue();
          }
        }
        out.add(new String[] {port, serial});
      }
    }
    return out;
  }

  /** Runs {@code reg query}; killed after 2 s so a hung reg.exe can never block forever. */
  private static List<String> regQuery(String key) {
    List<String> lines = new ArrayList<>();
    Process p = null;
    try {
      p = new ProcessBuilder("reg", "query", key, "/s").redirectErrorStream(true).start();
      final Process proc = p;
      // Drain output on a helper thread so the timeout below is enforced even if reg hangs.
      var output =
          java.util.concurrent.CompletableFuture.supplyAsync(
              () -> {
                List<String> out = new ArrayList<>();
                try (BufferedReader r =
                    new BufferedReader(
                        new InputStreamReader(proc.getInputStream(), StandardCharsets.UTF_8))) {
                  String line;
                  while ((line = r.readLine()) != null) {
                    out.add(line);
                  }
                } catch (IOException e) {
                  // process killed
                }
                return out;
              });
      if (!p.waitFor(2, TimeUnit.SECONDS)) {
        p.destroyForcibly();
        return lines;
      }
      lines = output.get(1, TimeUnit.SECONDS);
    } catch (Exception e) {
      if (p != null) {
        p.destroyForcibly();
      }
    }
    return lines;
  }

  // ---------------- framing ----------------

  private final class Parser {
    private final byte[] m_buf = new byte[512];
    private int m_len;

    void feed(byte[] data, int n) {
      for (int i = 0; i < n; i++) {
        if (m_len >= m_buf.length) {
          m_len = 0;
        }
        m_buf[m_len++] = data[i];
        process();
      }
    }

    private void process() {
      while (true) {
        if (m_len >= 1 && (m_buf[0] & 0xFF) != Protocol.SYNC1) {
          drop(1);
          continue;
        }
        if (m_len >= 2 && (m_buf[1] & 0xFF) != Protocol.SYNC2) {
          drop(1);
          continue;
        }
        if (m_len < 4) {
          return;
        }
        int len = m_buf[3] & 0xFF;
        if (m_len < len + 6) {
          return;
        }
        int crc = (m_buf[len + 4] & 0xFF) | ((m_buf[len + 5] & 0xFF) << 8);
        if (crc != Protocol.crc16(m_buf, 2, len + 2)) {
          drop(1);
          continue;
        }
        byte[] payload = new byte[len];
        System.arraycopy(m_buf, 4, payload, 0, len);
        int type = m_buf[2] & 0xFF;
        drop(len + 6);
        try {
          m_handler.accept(type, payload);
        } catch (RuntimeException | LinkageError e) {
          // Never let a bad frame or a missing class kill the reader thread.
          e.printStackTrace();
        }
      }
    }

    private void drop(int n) {
      System.arraycopy(m_buf, n, m_buf, 0, m_len - n);
      m_len -= n;
    }
  }
}
