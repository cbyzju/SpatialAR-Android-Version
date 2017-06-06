package com.orbbec.NativeNI;

/**
 * Created by netease on 16/10/31.
 */
public class NativeGestureDetect {
    static public native  long nativeInit(String calibjson,String path,boolean setviewDepth);
    static public native  boolean calibration(long nativeCalibrationAdds,long textureMappAdds,int width);
    static public native String getNativeGestureResult(long nativeCalibrationAdds, long textureMappAdds , long depthMapAdds,int adjustX,int adjustY,int flag);
    static public native boolean nativeDestroy(long nativeCalibrationAdds);
    static public native boolean calibrationFixed(long nativeCalibrationAdds);
    static public native boolean reset(long nativeCalibrationAdds);
    static {
        System.loadLibrary("jni-input");
    }
}
