package frc.robot.Utils;

import java.util.concurrent.*;

public class TimeUtils {
    public static void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            return;
        }
    }

    public static void executeWithTimeOut(ExecutorService executor, Runnable command, long timeOutMillis) throws TimeoutException {
        Future<?> future = executor.submit(command);
        try {
            // Wait for the task to complete (timeout: TIMEOUT_MS)
            future.get(timeOutMillis, TimeUnit.MILLISECONDS);
        } catch (InterruptedException | ExecutionException e) {
            // Handle interruption or other exceptions
            e.printStackTrace();
        }
    }
}
