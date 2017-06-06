package com.orbbec.DepthViewer.utils;

import android.os.RemoteException;
import android.util.Log;

/**
 * Created by zlh on 2015/7/10.
 */

public class Debug {

    private static final String TAG = "ORBBEC-DEBUG";
    private static int mDebugLevel;

    public Debug() {
    }

    public static int getDebugLevel() {
        return mDebugLevel;
    }

    public static void setDebugLevel(Levels debugLevel) {
        mDebugLevel = debugLevel.ordinal() + 1;
    }

    public static void v(String pMessage) {
        if(mDebugLevel != 0 && mDebugLevel <= 2) {
            Log.v(TAG, pMessage);
        }

    }

    public static void d(String pMessage) {
        if(mDebugLevel != 0 && mDebugLevel <= 3) {
            Log.d(TAG, pMessage);
        }

    }

    public static void i(String pMessage) {
        if(mDebugLevel != 0 && mDebugLevel <= 4) {
            Log.i(TAG, pMessage);
        }

    }

    public static void v(String logTag, String pMessage) {
        if(mDebugLevel != 0 && mDebugLevel <= 2) {
            Log.v(logTag, pMessage);
        }

    }

    public static void d(String logTag, String pMessage) {
        if(mDebugLevel != 0 && mDebugLevel <= 3) {
            Log.d(logTag, pMessage);
        }

    }

    public static void i(String logTag, String pMessage) {
        if(mDebugLevel != 0 && mDebugLevel <= 4) {
            Log.i(logTag, pMessage);
        }

    }

    public static void w(String pMessage) {
        if(mDebugLevel != 0 && mDebugLevel <= 5) {
            Log.w(TAG, pMessage);
        }

    }

    public static void w(Throwable pThrowable) {
        if(mDebugLevel != 0 && mDebugLevel <= 5) {
            Log.w(TAG, pThrowable);
        }

    }

    public static void w(String pMessage, Throwable pThrowable) {
        if(mDebugLevel != 0 && mDebugLevel <= 5) {
            if(pThrowable == null) {
                Log.w(TAG, pMessage);
                (new Exception()).printStackTrace();
            } else {
                Log.w(TAG, pMessage, pThrowable);
            }
        }

    }

    public static void w(String logTag, String pMessage) {
        if(mDebugLevel != 0 && mDebugLevel <= 5) {
            Log.w(logTag, pMessage);
        }

    }

    public static void e(String pMessage) {
        if(mDebugLevel != 0 && mDebugLevel <= 6) {
            Log.e(TAG, pMessage);
        }

    }

    public static void e(String logTag, String pMessage) {
        if(mDebugLevel != 0 && mDebugLevel <= 6) {
            Log.e(logTag, pMessage);
        }

    }

    public static void e(Throwable pThrowable) {
        if(mDebugLevel != 0 && mDebugLevel <= 6) {
            Log.e(TAG, pThrowable.getLocalizedMessage());
        }

    }

    public static void e(String pMessage, Throwable pThrowable) {
        if(mDebugLevel != 0 && mDebugLevel <= 6) {
            if(pThrowable == null) {
                Log.e(TAG, pMessage);
                (new Exception()).printStackTrace();
            } else {
                Log.e(TAG, pMessage, pThrowable);
            }
        }

    }

    public static void e(String logTag, String pMessage, RemoteException e) {
        if(mDebugLevel != 0 && mDebugLevel <= 6) {
            Log.e(logTag, pMessage, e);
        }

    }

    static {
        mDebugLevel = Levels.ERROR.ordinal() + 1;
    }

    public static enum Levels {
        DISABLED,
        VERBOSE,
        DEBUG,
        INFO,
        WARN,
        ERROR,
        ASSERT;

        private Levels() {
        }
    }
}
