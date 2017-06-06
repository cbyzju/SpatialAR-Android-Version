package com.orbbec.DepthViewer.UVC;

import android.graphics.ImageFormat;
import android.hardware.Camera;
import android.os.Looper;
import android.util.Log;

import com.orbbec.DepthViewer.utils.FPSMeter;
import com.orbbec.DepthViewer.utils.GlobalDef;


//import com.orbbec.GestureEngine.GestureEngine;

/**
 * Created by zlh on 2015/6/24.
 */
public class LiveStreamProcessor implements Camera.PreviewCallback {

    private static final String TAG = LiveStreamProcessor.class.getSimpleName();

    private byte[] mBuffer;
    private byte[] mRealTimeData;
    private int mSupportPreviewFormat = ImageFormat.NV21;//YUV420
    private FPSMeter mPreviewFPS = new FPSMeter();

    public int getSupportPreviewWidth(){ return GlobalDef.RES_UVC_WIDTH; }
    public int getSupportPreviewHeight(){ return GlobalDef.RES_UVC_HEIGHT; }

    public LiveStreamProcessor(){
        // Prepare buffer
        int bufferSize = GlobalDef.RES_UVC_WIDTH * GlobalDef.RES_UVC_HEIGHT * ImageFormat.getBitsPerPixel(mSupportPreviewFormat) / 8;
        mBuffer = new byte[bufferSize];
        Log.v(TAG, "buffer size " + bufferSize);
    }


    public byte[] getBuffer(){ return mBuffer; }


    public byte[] getRealTimeData(){return mRealTimeData;}

    @Override
    public void onPreviewFrame(byte[] data, Camera camera) {
        if(Looper.myLooper() == Looper.getMainLooper()){
            Log.e(TAG, "Camera preview is running on UI thread!");
            return;
        }

        camera.addCallbackBuffer(mBuffer);

        mPreviewFPS.measure(TAG, 25);

        /*处理*/
        //GestureEngine.getInstance().pushUVCFrame(data);
        mRealTimeData = data;


    }
}
