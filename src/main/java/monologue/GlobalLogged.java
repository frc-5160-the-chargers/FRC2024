package monologue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * The GlobalLogged class is a utility class that provides a simple way to use Monologue's
 * logging tooling from any part of your robot code. It provides a set of log methods that
 * allow you to log data to the NetworkTables and DataLog.
 * 
 * @see Monologue
 * @see LogLevel
 */
class GlobalLogged {
  static String ROOT_PATH = "";

  static void setRootPath(String rootPath) {
    ROOT_PATH = NetworkTable.normalizeKey(rootPath, true);
  }

  /**
    * Logs a boolean using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static void log(String entryName, boolean value) {
    log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a boolean using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static void log(String entryName, boolean value, LogLevel level) {
    entryName = NetworkTable.normalizeKey(entryName, true);
    if (!Monologue.isMonologueReady("GlobalLogged: " + entryName) || Monologue.isMonologueDisabled()) return;
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }
  }

  /**
    * Logs a int using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static void log(String entryName, int value) {
    log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a int using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static void log(String entryName, int value, LogLevel level) {
    entryName = NetworkTable.normalizeKey(entryName, true);
    if (!Monologue.isMonologueReady("GlobalLogged: " + entryName) || Monologue.isMonologueDisabled()) return;
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }
  }

  /**
    * Logs a long using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static void log(String entryName, long value) {
    log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a long using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static void log(String entryName, long value, LogLevel level) {
    entryName = NetworkTable.normalizeKey(entryName, true);
    if (!Monologue.isMonologueReady("GlobalLogged: " + entryName) || Monologue.isMonologueDisabled()) return;
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }
  }

  /**
    * Logs a float using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static void log(String entryName, float value) {
    log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a float using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static void log(String entryName, float value, LogLevel level) {
    entryName = NetworkTable.normalizeKey(entryName, true);
    if (!Monologue.isMonologueReady("GlobalLogged: " + entryName) || Monologue.isMonologueDisabled()) return;
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }
  }

  /**
    * Logs a double using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static void log(String entryName, double value) {
    log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a double using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static void log(String entryName, double value, LogLevel level) {
    entryName = NetworkTable.normalizeKey(entryName, true);
    if (!Monologue.isMonologueReady("GlobalLogged: " + entryName) || Monologue.isMonologueDisabled()) return;
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }
  }

  /**
    * Logs a String using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static void log(String entryName, String value) {
    log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a String using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static void log(String entryName, String value, LogLevel level) {
    entryName = NetworkTable.normalizeKey(entryName, true);
    if (!Monologue.isMonologueReady("GlobalLogged: " + entryName) || Monologue.isMonologueDisabled()) return;
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }
  }

  /**
    * Logs a byte[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static void log(String entryName, byte[] value) {
    log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a byte[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static void log(String entryName, byte[] value, LogLevel level) {
    entryName = NetworkTable.normalizeKey(entryName, true);
    if (!Monologue.isMonologueReady("GlobalLogged: " + entryName) || Monologue.isMonologueDisabled()) return;
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }
  }

  /**
    * Logs a boolean[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static void log(String entryName, boolean[] value) {
    log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a boolean[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static void log(String entryName, boolean[] value, LogLevel level) {
    entryName = NetworkTable.normalizeKey(entryName, true);
    if (!Monologue.isMonologueReady("GlobalLogged: " + entryName) || Monologue.isMonologueDisabled()) return;
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }
  }

  /**
    * Logs a int[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static void log(String entryName, int[] value) {
    log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a int[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static void log(String entryName, int[] value, LogLevel level) {
    entryName = NetworkTable.normalizeKey(entryName, true);
    if (!Monologue.isMonologueReady("GlobalLogged: " + entryName) || Monologue.isMonologueDisabled()) return;
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }
  }

  /**
    * Logs a long[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static void log(String entryName, long[] value) {
    log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a long[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static void log(String entryName, long[] value, LogLevel level) {
    entryName = NetworkTable.normalizeKey(entryName, true);
    if (!Monologue.isMonologueReady("GlobalLogged: " + entryName) || Monologue.isMonologueDisabled()) return;
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }
  }

  /**
    * Logs a float[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static void log(String entryName, float[] value) {
    log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a float[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static void log(String entryName, float[] value, LogLevel level) {
    entryName = NetworkTable.normalizeKey(entryName, true);
    if (!Monologue.isMonologueReady("GlobalLogged: " + entryName) || Monologue.isMonologueDisabled()) return;
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }
  }

  /**
    * Logs a double[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static void log(String entryName, double[] value) {
    log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a double[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static void log(String entryName, double[] value, LogLevel level) {
    entryName = NetworkTable.normalizeKey(entryName, true);
    if (!Monologue.isMonologueReady("GlobalLogged: " + entryName) || Monologue.isMonologueDisabled()) return;
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }
  }

  /**
    * Logs a String[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static void log(String entryName, String[] value) {
    log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a String[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static void log(String entryName, String[] value, LogLevel level) {
    entryName = NetworkTable.normalizeKey(entryName, true);
    if (!Monologue.isMonologueReady("GlobalLogged: " + entryName) || Monologue.isMonologueDisabled()) return;
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }
  }


  /**
    * Logs a Serializable Struct using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static <R extends StructSerializable> void log(String entryName, R value) {
    log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a Serializable Struct using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static <R extends StructSerializable> void log(String entryName, R value, LogLevel level) {
    entryName = NetworkTable.normalizeKey(entryName, true);
    if (!Monologue.isMonologueReady("GlobalLogged: " + entryName) || Monologue.isMonologueDisabled()) return;
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }
  }

  /**
    * Logs an array of Serializable Structs using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static <R extends StructSerializable> void log(String entryName, R[] value) {
    log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs an array of Serializable Structs using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public static <R extends StructSerializable> void log(String entryName, R[] value, LogLevel level) {
    entryName = NetworkTable.normalizeKey(entryName, true);
    if (!Monologue.isMonologueReady("GlobalLogged: " + entryName) || Monologue.isMonologueDisabled()) return;
    Monologue.ntLogger.put(entryName, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value, level);
      }
    }
  }

  /**
    * Logs a Serializable Struct using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param struct The struct type to log.
    * @param value The value to log.
    */
  public <R> void log(String entryName, Struct<R> struct, R value) {
    log(entryName, struct, value, LogLevel.DEFAULT);
  }
  /**
    * Logs a Serializable Struct using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param struct The struct type to log.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public <R> void log(String entryName, Struct<R> struct, R value, LogLevel level) {
    entryName = NetworkTable.normalizeKey(entryName, true);
    if (!Monologue.isMonologueReady("GlobalLogged: " + entryName) || Monologue.isMonologueDisabled()) return;
    Monologue.ntLogger.putStruct(entryName, struct, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.putStruct(entryName, struct, value, level);
      }
    }
  }

  /**
    * Logs an array of Serializable Structs using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param struct The struct type to log.
    * @param value The value to log.
    */
  public <R> void log(String entryName, Struct<R> struct, R[] value) {
    log(entryName, struct, value, LogLevel.DEFAULT);
  }
  /**
    * Logs an array of Serializable Structs using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param struct The struct type to log.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public <R> void log(String entryName, Struct<R> struct, R[] value, LogLevel level) {
    entryName = NetworkTable.normalizeKey(entryName, true);
    if (!Monologue.isMonologueReady("GlobalLogged: " + entryName) || Monologue.isMonologueDisabled()) return;
    Monologue.ntLogger.putStructArray(entryName, struct, value, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.putStructArray(entryName, struct, value, level);
      }
    }
  }

  /**
    * Logs an Enum using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public <E extends Enum<?>> void log(String entryName, E value) {
    log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs an Enum using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public <E extends Enum<?>> void log(String entryName, E value, LogLevel level) {
    entryName = NetworkTable.normalizeKey(entryName, true);
    if (!Monologue.isMonologueReady("GlobalLogged: " + entryName) || Monologue.isMonologueDisabled()) return;
    Monologue.ntLogger.put(entryName, value.name(), level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, value.name(), level);
      }
    }
  }

  /**
    * Logs an array of Enums using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public <E extends Enum<?>> void log(String entryName, E[] value) {
    log(entryName, value, LogLevel.DEFAULT);
  }
  /**
    * Logs an array of Enums using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param level The log level to use.
    */
  public <E extends Enum<?>> void log(String entryName, E[] value, LogLevel level) {
    entryName = NetworkTable.normalizeKey(entryName, true);
    if (!Monologue.isMonologueReady("GlobalLogged: " + entryName) || Monologue.isMonologueDisabled()) return;
    String[] names = new String[value.length];
    for (int i = 0; i < value.length; i++) {
      names[i] = value[i].name();
    }
    Monologue.ntLogger.put(entryName, names, level);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      if (!(level == LogLevel.DEFAULT && Monologue.isFileOnly())) {
        Monologue.dataLogger.put(entryName, names, level);
      }
    }
  }

  /**
    * Logs a Sendable using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static void publishSendable(String entryName, Sendable value) {
    entryName = NetworkTable.normalizeKey(entryName, true);
    if (!Monologue.isMonologueReady("GlobalLogged: " + entryName) || Monologue.isMonologueDisabled()) return;
    Monologue.ntLogger.addSendable(entryName, value);

    // The Monologue made NT datalog subscriber only subs to stuff under ROOT_PATH
    // If this is sending data to a different path, we should also log it to the file
    if (!ROOT_PATH.isEmpty() && !entryName.startsWith(ROOT_PATH)) {
      NetworkTablesJNI.startEntryDataLog(
        NetworkTableInstance.getDefault().getHandle(),
        DataLogManager.getLog(),
        entryName,
        Monologue.dataLogger.prefix + entryName
      );
    }
  }
}
