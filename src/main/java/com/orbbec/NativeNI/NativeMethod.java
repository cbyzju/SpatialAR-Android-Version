package com.orbbec.NativeNI;

/**
 * Created by Lin_JX on 2015/7/27.
 */
public class NativeMethod {

    static{
       System.loadLibrary("OpenNIEx.jni");
    }

    public native static int Init();
    public static native int GetDepthData(int[] depthData);
    public native static int GetRGBData(int[] rgbData);
    public native static int GetHandPos3D(Integer x, Integer y, Integer z);
    public native static int GetHandPos2D(Integer x, Integer y, Integer z);
    public native static int GetNV21Data(byte[] nv21Data);
    public native static int NV21ToRGB32(byte[] nv21Data, int[] rgbData);
    public native static int Update();
    public native static int ReleaseSensor();
    public native static int RealWorldToProjective(float rx, float ry, float rz, Float px, Float py, Float pz);
    public native static int ProjectiveToRealWorld(float px, float py, float pz, Float rx, Float ry, Float rz);

    public native static int EnableLog(boolean enable);
}
