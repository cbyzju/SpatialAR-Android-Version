package com.orbbec.DepthViewer.UVC;

import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.util.Log;
import android.view.SurfaceHolder;

import java.io.IOException;
import java.util.List;

/**
 * Created by zlh on 2015/6/24.
 */
public class LiveCamera {

    private static final String TAG = LiveCamera.class.getSimpleName();

    private Camera mCamera = null;
    private byte[] mBuffer;
    private Camera.PreviewCallback mPreviewCb = null;

    public LiveCamera(){

    }

    public int init(int previewWidth, int previewHeight, int fps, Camera.PreviewCallback cb, byte[] buffer){

        Log.d(TAG,"init uvc camera!!");
        mCamera = getCameraInstance();

        if(mCamera == null){
            Log.e(TAG,"UVC camera is null!!");
            return -1;
        }

        config(previewWidth, previewHeight, fps);
        mPreviewCb = cb;
        mBuffer = buffer;

        startPreview();
        
        return 0;
    }

    public void terminate(){
        mCamera.setPreviewCallbackWithBuffer(null);
        mCamera.stopPreview();
        mCamera.release();
        mCamera = null;
    }


    public byte[] GetBuffer()
    {
        return mBuffer;
    }

    private void config(int previewWidth, int previewHeight, int fps){

        Camera.Parameters params = mCamera.getParameters();

        // FPS
        List<int[]> fpsRange = params.getSupportedPreviewFpsRange();

        for(int i = 0; i < fpsRange.size(); i++){
            Log.v(TAG, "FPS range " + fpsRange.get(i)[0] + " " + fpsRange.get(i)[1] + " " + fpsRange.size());
        }

        params.setPreviewFpsRange(fpsRange.get(0)[0], fpsRange.get(0)[1]);

        // Resolution
        List<Camera.Size> previewSizes = params.getSupportedPreviewSizes();
        boolean supportResolution = false;
        for(int i = 0; i < previewSizes.size(); i++){
            int w = previewSizes.get(i).width;
            int h = previewSizes.get(i).height;
            //Log.v(TAG, "Preview size " + w + " " + h);
            if(w == previewWidth && h == previewHeight){
                supportResolution = true;
            }
        }

        if(supportResolution == false){
            Log.e(TAG, "Resolution " + previewWidth + " x " + previewHeight + " is not supported!");
            return;
        }

        params.setPreviewSize(previewWidth, previewHeight);

        // todo: fix it
        // When both depth camera and USB camera are on, sometimes setting parameters will fail.
        try{
            mCamera.setParameters(params);
        }catch (Exception e){
            Log.d(TAG, "setParameters exception");
        }
    }

    public void setPreviewTexture(SurfaceTexture surface){

        try {
            mCamera.setPreviewTexture(surface);
        } catch (IOException ioe) {
            Log.d(TAG, "setPreviewTexture exception");
        }
    }

    public void setPreviewDisplay(SurfaceHolder holder) {

        try {
            //Log.d("TEST","mCamera = " + mCamera);
            mCamera.setPreviewDisplay(holder);
        } catch (IOException ioe) {
            Log.d(TAG, "setPreviewDisplay exception");
        }
    }


    public void startPreview(){

        mCamera.addCallbackBuffer(mBuffer);
        mCamera.setPreviewCallbackWithBuffer(mPreviewCb);
        mCamera.startPreview();
    }


    public static Camera getCameraInstance() {


        Log.d(TAG,"GetCameraInstance!!");
        Camera c = null;


        // the problem:
        // on RK3288, boot the system, plug in camera, Camera.open(0) or Camera.open(1) fails always
        // the fix:
        // call getNumberOfCameras() before calling Camera.open(0) or Camera.open(1)
        // root cause:
        // android's camera service will check plugged in camera only at the first beginning of system boot up.
        // calling getNumberOfCameras will force the camera service to rescan camera devices.
        int numOfCameras = Camera.getNumberOfCameras();
        Log.v(TAG, "Num of Cameras " + numOfCameras);

        try {
            c = Camera.open(0);
        }
        catch (Exception e) {
            Log.d(TAG, "Camera open 0 exception");
        }

        if(null == c){
            try {
                // Error on 0, now try 1
                c = Camera.open(1);
            }
            catch (Exception e) {
                Log.d(TAG, "Camera open 1 exception");
            }
        }
        Log.d(TAG,"Return UVC Camera!!");
        return c;
    }

}
