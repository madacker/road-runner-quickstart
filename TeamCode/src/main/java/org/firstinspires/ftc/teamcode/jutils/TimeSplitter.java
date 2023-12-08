package org.firstinspires.ftc.teamcode.jutils;

import android.annotation.SuppressLint;
import android.os.Debug;
import android.util.Log;
import java.util.ArrayList;
import static java.lang.Double.MAX_VALUE;
import static java.lang.System.nanoTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

/**
 * Incremental standard deviation courtesy of
 * <a href="https://stackoverflow.com/questions/1174984/how-to-efficiently-calculate-a-running-standard-deviation">StackOverflow</a>.
 */
class RunningStats {
    int n = 0;
    double old_m = 0;
    double new_m = 0;
    double old_s = 0;
    double new_s = 0;
    public void add(double x) {
        this.n += 1;
        if (this.n == 1) {
            this.old_m = x;
            this.new_m = x;
            this.old_s = 0;
        } else {
            this.new_m = this.old_m + (x - this.old_m) / this.n;
            this.new_s = this.old_s + (x - this.old_m) * (x - this.new_m);
            this.old_m = this.new_m;
            this.old_s = this.new_s;
        }
    }
    public double mean() {
        return (this.n != 0) ? this.new_m : 0.0;
    }
    public double variance() {
        return (this.n > 1) ? this.new_s / this.n : 0.0;
    }
    public double sd() {
        return Math.sqrt(this.variance());
    }
}

// Keep fine-grained split-time statistics.
public class TimeSplitter {
    double nanosToMillis(long x) {
        return x / (1000.0 * 1000.0);
    }
    String description;
    boolean log;
    int n;
    int startGcCount;
    int totalGcCount;
    long startThreadNanos;
    double maxOutlierTime;
    double minOutlierTime;
    double minOutlier = MAX_VALUE;
    double maxOutlier = -1;
    long runtimeStartNanos = nanoTime();
    long startClockNanos = 0;
    RunningStats clockStats = new RunningStats();
    RunningStats threadStats = new RunningStats();
    static ArrayList<TimeSplitter> list = new ArrayList<TimeSplitter>();
    /**
     * Constructor.
     * @param description of this time-split. This is private and for internal use only. Call
     *                    the public (and static) create() function from external code.
     */
    private TimeSplitter(String description, boolean log) {
        this.description = description;
        this.log = log;
    }
    /**
     * Debug.getThreadGcInvocationCount() and Debug.startAllocCounting() were deprecated in API
     * level 23 but since FTC is stuck on level 14, it's safe for us to call. We just need to
     * quiet Lint. Note that startAllocCounting() must be called before calling
     * getThreadGcInvocationCount.
     */
    @SuppressWarnings("deprecation")
    public static void startAllocCounting() {
        Debug.startAllocCounting();
    }
    @SuppressWarnings("deprecation")
    public static int getThreadGcInvocationCount() {
        return Debug.getThreadGcInvocationCount();
    }
    /**
     * Start measuring a split time.
     */
    public void startSplit() {
        startClockNanos = nanoTime();
        startThreadNanos = Debug.threadCpuTimeNanos();
        startGcCount = TimeSplitter.getThreadGcInvocationCount();
    }
    public void endSplit() {
        if (startClockNanos == 0)
            return; // Allow empty 'endSplit()' calls
        long clockNanos = nanoTime();
        double clockMillis = nanosToMillis(clockNanos - startClockNanos);
        startClockNanos = 0;
        n++;
        this.clockStats.add(clockMillis);
        if (clockMillis < minOutlier) {
            minOutlier = clockMillis;
            minOutlierTime = nanosToMillis(clockNanos - runtimeStartNanos);
        }
        if (clockMillis > maxOutlier) {
            maxOutlier = clockMillis;
            maxOutlierTime = nanosToMillis(clockNanos - runtimeStartNanos);
        }
        long threadNanos = Debug.threadCpuTimeNanos();
        double threadMillis = nanosToMillis(threadNanos - startThreadNanos);
        this.threadStats.add(threadMillis);
        totalGcCount += TimeSplitter.getThreadGcInvocationCount() - startGcCount;
    }
    // Get the results as a string:
    @SuppressLint("DefaultLocale")
    public String getResult() {
        if (minOutlier <= maxOutlier) {
            return String.format("x%d: Clock mean %.2fms (SD %.2f), Thread mean %.2fms (SD %.2f), Longest %.2fms (at %.0f), Shortest %.2fms (at %.0f), %d GCs",
                    n, clockStats.mean(), clockStats.sd(), threadStats.mean(), threadStats.sd(), maxOutlier, maxOutlierTime, minOutlier, minOutlierTime, totalGcCount);
        } else {
            return String.format("- Not used");
        }
    }
    // Log the results:
    private void logResult(TelemetryPacket packet) {
        if (log) {
            Log.d("TimeSplitter", this.description + " " + getResult());
            packet.put(this.description, getResult());
        }
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Static methods
    ////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * Create a TimeSplitter object.
     * @param description describes the use of the timer for identification in the log.
     * @return TimeSplitter object.
     */
    synchronized public static TimeSplitter create(String description) {
        return create(description, true);
    }
    synchronized public static TimeSplitter create(String description, boolean log) {
        TimeSplitter self = new TimeSplitter(description, log);
        TimeSplitter.list.add(self);
        return self;
    }
    /**
     * Log the results for *all* TimeSplitter objects that were created.
     */
    synchronized public static void logAllResults() {
        TelemetryPacket packet = new TelemetryPacket();
        for (TimeSplitter t: TimeSplitter.list) {
            t.logResult(packet);
        }
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}

