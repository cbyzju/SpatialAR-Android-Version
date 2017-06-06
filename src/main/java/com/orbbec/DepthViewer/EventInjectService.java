package com.orbbec.DepthViewer;

import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.os.IBinder;
import android.os.Looper;
import android.os.SystemClock;
import android.util.Log;
import android.widget.Toast;

import com.orbbec.DepthViewer.receiver.CoordinateAdjustReceiver;
import com.orbbec.DepthViewer.utils.CalibrationDialog;

import com.orbbec.DepthViewer.utils.GlobalDef;
import com.orbbec.DepthViewer.utils.LogUtil;
import com.orbbec.DepthViewer.utils.RootCommand;

import com.orbbec.DepthViewer.utils.UsbErrorDialogue;
import com.orbbec.DepthViewer.utils.Util;
import com.orbbec.NativeNI.NativeGestureDetect;

import com.orbbec.astrakernel.PermissionCallbacks;
import com.orbbec.permission.PermissionGrant;


import org.json.JSONObject;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.util.HashMap;
import java.util.Iterator;

/**
 * Created by netease on 16/10/9.
 */
public class EventInjectService extends Service {

    String TAG = "obDepth";
    static private OrcManager sOrcManager;
    private  CalibrationDialog calibrationDialog;
    private  CoordinateAdjustReceiver coordinateAdjustReceiver;

    private PermissionCallbacks mCallbacks = new PermissionCallbacks() { //摄像头打开回调
    @Override
    public void onDevicePermissionGranted() {
        Log.d("123","4");
        String apkRoot="chmod 777 "+getPackageCodePath();
        RootCommand.Root(apkRoot);
        apkRoot="chmod 777 /dev/input/event1";
        RootCommand.Root(apkRoot);
        Log.d("root","ok");
        sOrcManager.init();//初始化摄像头数据
        try{
            sOrcManager.mContext.start(); //开始生成摄像头数据
            Log.d("123","5");
        }catch (Exception e1) {
            // TODO Auto-generated catch block
             e1.printStackTrace();
        }

//        calibrationDialog=new CalibrationDialog(getApplicationContext(),R.style.Dialog_Fullscreen);//投标定图
//        calibrationDialog.showCalibrationDialog();

        EventInjectThread eventInjectThread=new EventInjectThread();
        new Thread(eventInjectThread).start();

    }

    @Override
    public void onDevicePermissionDenied() {

    }
};

    @Override
    public IBinder onBind(Intent intent) {
        return null;
    }

    @Override
    public void onCreate() {
        super.onCreate();

        LogUtil.setLevel(LogUtil.DEBUG);
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {


        IntentFilter intentFilter=new IntentFilter();
        intentFilter.addAction("COORDINATE_ADJUST");
        coordinateAdjustReceiver=new CoordinateAdjustReceiver();
        registerReceiver(coordinateAdjustReceiver,intentFilter);
        Log.d("123","1");
        new Thread(new Runnable() {
            @Override
            public void run() {
                Looper.prepare();
                HashMap<String,UsbDevice> deviceHashMap;
                Log.d("123","2");
                while(true){

                    deviceHashMap=getDevInstall();
                    Log.d("123","3");
                    if(deviceHashMap.size()!=0)
                    {
                        try {
                            sOrcManager =new OrcManager(getApplicationContext(), mCallbacks);//生成摄像头管理类

                        } catch (Exception e) {
                            // TODO Auto-generated catch block
                            Log.d("123","10");
                            e.printStackTrace();

                        }
                        break;
                    }

                }
                Looper.loop();
            }
        }).start();


        return super.onStartCommand(intent, flags, startId);
    }

    private HashMap<String, UsbDevice> getDevInstall()
    {
        UsbManager manager = (UsbManager)getApplicationContext().getSystemService(Context.USB_SERVICE);
        HashMap<String, UsbDevice> deviceList = manager.getDeviceList();
        return deviceList;
    }

    class EventInjectThread implements Runnable{
        @Override
        public void run() {
            Log.d("123","7");
            SystemClock.sleep(1000);//解决第一次初始化崩溃的问题
            Mat textureMat=new Mat(sOrcManager.mHeight, sOrcManager.mWidth, CvType.CV_8UC3);
            Mat depthMat=new Mat(sOrcManager.mHeight, sOrcManager.mWidth,CvType.CV_16U);
            String calibjson=getCalibrationParas();
            Log.d("calib",calibjson);
            sOrcManager.mCalibration=NativeGestureDetect.nativeInit(calibjson,calibjson,true);//构建native标定对象

            calibration(textureMat);

            sOrcManager.mDepthData.setViewpoint(sOrcManager.mRgbData); //将彩图和深度图对应


            //手势识别并模拟触摸屏输入事件
            while (!sOrcManager.mExit) {
                try {
                    sOrcManager.mContext.waitforupdate();//更新摄像头数据
                    boolean reCalibration=false;
                    synchronized (GlobalDef.class){
                        if(true == GlobalDef.reCalibration){

//                            calibrationDialog=new CalibrationDialog(getApplicationContext(),R.style.Dialog_Fullscreen);//投标定图
//                            calibrationDialog.showCalibrationDialog();
                            reCalibration=GlobalDef.reCalibration;
                            GlobalDef.reCalibration=false;
                        }
                    }
                    if(true == reCalibration){
                        NativeGestureDetect.reset(sOrcManager.mCalibration);
                        Log.d("RE","OK");
                        calibration(textureMat);
                    }
                    sOrcManager.getDepthMat(depthMat);//读取深度图矩阵
                    int adjustX;
                    int adjustY;
                    synchronized (MyApplication.sInstance){
                        adjustX=MyApplication.sInstance.getAdjustX();
                        adjustY=MyApplication.sInstance.getAdjustY();
                    }
                    String string=null;
                    long endTime1=System.currentTimeMillis();
                    if(Util.getForegroundActivity(getApplicationContext()).equals("com.multimedia.Projector")){
                        Log.d("HelloProjection","ok");
                        string=NativeGestureDetect.getNativeGestureResult(sOrcManager.mCalibration,
                                textureMat.getNativeObjAddr(),depthMat.getNativeObjAddr(),adjustX,adjustY,1);//手势识别
                    }else if(Util.getForegroundActivity(getApplicationContext()).equals("com.netease.arprojectorinair")){
                        string=NativeGestureDetect.getNativeGestureResult(sOrcManager.mCalibration,
                                textureMat.getNativeObjAddr(),depthMat.getNativeObjAddr(),adjustX,adjustY,2);//手势识别
                        Log.d("HelloProjection","ok1");
                    }else{
                        string=NativeGestureDetect.getNativeGestureResult(sOrcManager.mCalibration,
                                textureMat.getNativeObjAddr(),depthMat.getNativeObjAddr(),adjustX,adjustY,0);//手势识别
                        Log.d("HelloProjection","ok2");
                    }
                    long endTime2=System.currentTimeMillis();
                    Log.d("totaltime","all time"+String.valueOf(endTime2-endTime1));
                    Intent intent=new Intent();
                    intent.putExtra("gesture",string);
                    intent.setAction("com.netease.action.RECEIVE_DATA");
                    sendBroadcast(intent);

                } catch (Exception e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }
        }
    }

    private void calibration(Mat textureMat){

        //标定
        while(true) {
            sOrcManager.mContext.waitforupdate(); //更新摄像头数据
            sOrcManager.getTextureMat(textureMat);  //读取彩图矩阵
            boolean flag = NativeGestureDetect.calibration(sOrcManager.mCalibration, textureMat.getNativeObjAddr(), 50);//系统标定
            if (flag) {
//                calibrationDialog.dismiss();
                break;
            }

        }
        Intent intent1=new Intent();
        intent1.setAction("com.orbbec.depth.START_SHOW_LAUNCHER");
        sendBroadcast(intent1);
    }

    private String getCalibrationParas(){
        SharedPreferences sharedPreferences = getSharedPreferences("CalibrationParas", Context.MODE_PRIVATE);
        String string=sharedPreferences.getString("calibjson",null);
        if(null==string){
            try{
                JSONObject jsonObject=new JSONObject();
                jsonObject.put("Rect_X",100);//100
                jsonObject.put("Rect_Y",50);//50
                jsonObject.put("Rect_W",500);//500
                jsonObject.put("Rect_H",300);//
                jsonObject.put("Filt_X",48);//
                jsonObject.put("Filt_Y",35);//
                jsonObject.put("Filt_W",40);//
                jsonObject.put("Filt_H",30);//
                jsonObject.put("Grid_X",0);
                jsonObject.put("Grid_Y",0);
                jsonObject.put("Offset_X",0);
                jsonObject.put("Offset_Y",0);
                string=jsonObject.toString();
                SharedPreferences.Editor editor = sharedPreferences.edit();//获取编辑器
                editor.putString("calibjson", string);
                editor.commit();//提交修改
            }catch (Exception e){
                e.printStackTrace();
            }
        }
        Log.d("calib","out");
        return string;
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        unregisterReceiver(coordinateAdjustReceiver);
        if (sOrcManager.mRgbData != null) {
            sOrcManager.mRgbData.close();
        }
        if (sOrcManager.mLiveCamera != null) {
            Log.d(TAG, "mLiveCamera.terminate()!!!");
            sOrcManager.mLiveCamera.terminate();
        }

        if (sOrcManager.mDepthData != null) {
            sOrcManager.mDepthData.close();
        }

        if(sOrcManager.mContext != null)
        {
            sOrcManager.mContext.close();
        }
        if(sOrcManager.mCalibration!=0){
            NativeGestureDetect.nativeDestroy(sOrcManager.mCalibration);
        }

        Log.i(TAG, "onDestroy!");
    }

    static {
        System.loadLibrary("opencv_java");
        System.loadLibrary("jni-input");
    }
}

