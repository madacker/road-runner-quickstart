//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package android.util;

import android.annotation.NonNull;
import android.annotation.Nullable;
import androidx.annotation.RecentlyNonNull;
import androidx.annotation.RecentlyNullable;

public final class Log {
    public static final int ASSERT = 7;
    public static final int DEBUG = 3;
    public static final int ERROR = 6;
    public static final int INFO = 4;
    public static final int VERBOSE = 2;
    public static final int WARN = 5;

    Log() {
        throw new RuntimeException("Stub!");
    }

    public static int v(@RecentlyNullable String tag, @RecentlyNonNull String msg) {
        throw new RuntimeException("Stub!");
    }

    public static int v(@RecentlyNullable String tag, @RecentlyNullable String msg, @RecentlyNullable Throwable tr) {
        throw new RuntimeException("Stub!");
    }

    public static int d(@RecentlyNullable String tag, @RecentlyNonNull String msg) {
        throw new RuntimeException("Stub!");
    }

    public static int d(@RecentlyNullable String tag, @RecentlyNullable String msg, @RecentlyNullable Throwable tr) {
        throw new RuntimeException("Stub!");
    }

    public static int i(@RecentlyNullable String tag, @RecentlyNonNull String msg) {
        throw new RuntimeException("Stub!");
    }

    public static int i(@RecentlyNullable String tag, @RecentlyNullable String msg, @RecentlyNullable Throwable tr) {
        throw new RuntimeException("Stub!");
    }

    public static int w(@RecentlyNullable String tag, @RecentlyNonNull String msg) {
        throw new RuntimeException("Stub!");
    }

    public static int w(@RecentlyNullable String tag, @RecentlyNullable String msg, @RecentlyNullable Throwable tr) {
        throw new RuntimeException("Stub!");
    }

    public static native boolean isLoggable(@RecentlyNullable String var0, int var1);

    public static int w(@RecentlyNullable String tag, @RecentlyNullable Throwable tr) {
        throw new RuntimeException("Stub!");
    }

    public static int e(@RecentlyNullable String tag, @RecentlyNonNull String msg) {
        throw new RuntimeException("Stub!");
    }

    public static int e(@RecentlyNullable String tag, @RecentlyNullable String msg, @RecentlyNullable Throwable tr) {
        throw new RuntimeException("Stub!");
    }

    public static int wtf(@RecentlyNullable String tag, @RecentlyNullable String msg) {
        throw new RuntimeException("Stub!");
    }

    public static int wtf(@RecentlyNullable String tag, @RecentlyNonNull Throwable tr) {
        throw new RuntimeException("Stub!");
    }

    public static int wtf(@RecentlyNullable String tag, @RecentlyNullable String msg, @RecentlyNullable Throwable tr) {
        throw new RuntimeException("Stub!");
    }

    @NonNull
    public static String getStackTraceString(@Nullable Throwable tr) {
        throw new RuntimeException("Stub!");
    }

    public static int println(int priority, @RecentlyNullable String tag, @RecentlyNonNull String msg) {
        throw new RuntimeException("Stub!");
    }
}
