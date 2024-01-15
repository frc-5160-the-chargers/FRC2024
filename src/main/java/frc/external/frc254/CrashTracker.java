package frc.external.frc254;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Date;
import java.util.UUID;

/**
 * Tracks start-up and caught crash events, logging them to a file which dosn't roll over
 */
@SuppressWarnings("ALL")
public class CrashTracker {

    private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();

    public static void logThrowableCrash(Throwable throwable) {
        logMarker("Exception", throwable);
    }

    private static void logMarker(String mark) {
        logMarker(mark, null);
    }

    private static void logMarker(String mark, Throwable nullableException) {

        try (PrintWriter writer = new PrintWriter(new FileWriter("/home/lvuser/crash_tracking.txt", true))) {
            writer.print(RUN_INSTANCE_UUID);
            writer.print(", ");
            writer.print(mark);
            writer.print(", ");
            writer.print(new Date());

            if (nullableException != null) {
                writer.print(", ");
                nullableException.printStackTrace(writer);
            }

            writer.println();

            if (nullableException != null){
                throw new RuntimeException(nullableException);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
