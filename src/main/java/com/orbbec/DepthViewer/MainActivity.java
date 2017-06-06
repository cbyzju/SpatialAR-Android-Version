package com.orbbec.DepthViewer;

import android.app.ActionBar;
import android.app.Activity;
import android.app.IntentService;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.hardware.usb.UsbDevice;
import android.os.Bundle;
import android.os.SystemClock;
import android.util.Log;
import android.view.Gravity;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.ImageView;
import android.widget.PopupWindow;

import com.orbbec.DepthViewer.utils.GlobalDef;
import com.orbbec.DepthViewer.utils.RootCommand;
import com.orbbec.DepthViewer.utils.Util;
import com.orbbec.NativeNI.NativeGestureDetect;
import com.orbbec.astrakernel.PermissionCallbacks;

import org.json.JSONObject;
import org.opencv.core.*;
import org.opencv.objdetect.CascadeClassifier;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.HashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;


public class MainActivity extends Activity {


    String TAG = "obDepth";
    private OrcManager orcManager;
    private PopupWindow mPopupWindow;
    static private int mCalibarationMode = 1;


    private ExecutorService scheduledThreadPool = Executors.newFixedThreadPool(2);
    private Mat textureMat = new Mat(GlobalDef.RES_COLOR_HEIGHT, GlobalDef.RES_COLOR_WIDTH, CvType.CV_8UC3);
    private Mat depthMat = new Mat(GlobalDef.RES_COLOR_HEIGHT, GlobalDef.RES_COLOR_WIDTH, CvType.CV_16U);
    private int adjustX;
    private int adjustY;

//    Runnable gestureCommand=new Runnable() {
//        @Override
//        public void run() {
//            NativeGestureDetect.getNativeGestureResult(orcManager.mCalibration,
//                    textureMat.getNativeObjAddr(),depthMat.getNativeObjAddr(),adjustX,adjustY);//手势识别
//
//        }
//    };
//
//    Runnable inAirGestureCommand=new Runnable() {
//        @Override
//        public void run() {
//            String string= NativeGestureDetect.getNativeGestureResult(orcManager.mCalibration,
//                    textureMat.getNativeObjAddr(),depthMat.getNativeObjAddr(),adjustX,adjustY);//手势识别
//        }
//    };

//    Runnable projectionCommand=new Runnable() {
//        @Override
//        public void run() {
//            String string= NativeGestureDetect.getProjectionResult(depthMat.getNativeObjAddr());
//            Intent intent=new Intent();
//            intent.putExtra("gesture",string);
//            intent.setAction("com.netease.action.RECEIVE_DATA");
//            sendBroadcast(intent);
//        }
//    };

    private PermissionCallbacks mCallbacks = new PermissionCallbacks() { //摄像头回调函数
        @Override
        public void onDevicePermissionGranted() {

            orcManager.init();     //初始化摄像头数据
            try {
                orcManager.mContext.start();  //产生摄像头数据
            } catch (Exception e1) {
                // TODO Auto-generated catch block
                e1.printStackTrace();
            }
            MyThread myThread = new MyThread();
            new Thread(myThread).start();

        }

        @Override
        public void onDevicePermissionDenied() {

        }
    };

    ImageView imageView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
//        requestWindowFeature(Window.FEATURE_NO_TITLE);
//        //取消状态栏
//        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
//                WindowManager.LayoutParams.FLAG_FULLSCREEN);
//
//        View decorView = getWindow().getDecorView();//取消标题栏
//        // Hide both the navigation bar and the status bar.
//        // SYSTEM_UI_FLAG_FULLSCREEN is only available on Android 4.1 and higher, but as
//        // a general rule, you should design your app to hide the status bar whenever you
//        // hide the navigation bar.
//        int uiOptions = View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
//                | View.SYSTEM_UI_FLAG_FULLSCREEN;
//        decorView.setSystemUiVisibility(uiOptions);
        imageView = (ImageView) findViewById(R.id.calibration);

        setContentView(R.layout.main);

//              try {
//                  orcManager=new OrcManager(getApplicationContext(), mCallbacks);//摄像头管理类
////                    GlobalDef.USE_UVC = orcManager.mContext.IsUVC();//选择摄像头模式
////                    if (GlobalDef.USE_UVC) {
////                        Log.i(TAG, "Astra Pro * Detected!");
////                    } else {
////                        Log.i(TAG, "Astra * Detected!");
////                    }
//              } catch (Exception e) {
//                  // TODO Auto-generated catch block
//                  e.printStackTrace();
//              }
        HashMap<String, UsbDevice> deviceHashMap;
        while (true) {

            deviceHashMap = Util.getDevInstall(this);
            Log.d("123", "3");
            if (deviceHashMap.size() != 0) {
                try {
                    orcManager = new OrcManager(getApplicationContext(), mCallbacks);//生成摄像头管理类

                } catch (Exception e) {
                    // TODO Auto-generated catch block
                    Log.d("123", "10");
                    e.printStackTrace();

                }
                break;
            }

        }


    }


    class MyThread implements Runnable {
        @Override
        public void run() {

            SystemClock.sleep(2000);//解决第一次初始化崩溃的问题
            Log.d("123", "11");
            String apkRoot = "chmod 777 " + getPackageCodePath();
            RootCommand.Root(apkRoot);
            apkRoot = "chmod 777 /dev/input/event1";
            RootCommand.Root(apkRoot);
//            Mat textureMat=new Mat(orcManager.mHeight,orcManager.mWidth, CvType.CV_8UC3);
//            Mat depthMat=new Mat(orcManager.mHeight,orcManager.mWidth,CvType.CV_16U);
            String calibjson = getCalibrationParas();
            Log.d("calib", calibjson);
            //Log.d("calibPaht", saveConfigFile());
            orcManager.mCalibration = NativeGestureDetect.nativeInit(calibjson, "ss",true);//构建native标定对象
            if (0 == mCalibarationMode) {
                NativeGestureDetect.calibrationFixed(orcManager.mCalibration);
                Log.d("calib", "ok!");
            } else if (1 == mCalibarationMode) {
                while (true) {  //标定
                    try {
                        orcManager.mContext.waitforupdate(); //更新摄像头数据
                        orcManager.getTextureMat(textureMat);  //读取彩图矩阵
                        boolean flag = NativeGestureDetect.calibration(orcManager.mCalibration, textureMat.getNativeObjAddr(), 50);//系统标定

                        if (flag) {
//                            runOnUiThread(new Runnable() {
//                                @Override
//                                public void run() {
//                                    mPopupWindow.dismiss();
//                                }
//                            });
                            break;
                        }
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
            }

            //orcManager.mDepthData.setViewpoint(orcManager.mRgbData);
            while (!orcManager.mExit) { //手势识别并模拟触摸屏输入事件
                try {
                    long endTime1 = System.currentTimeMillis();
                    //orcManager.mContext.waitforupdate();//更新摄像头数据
                     orcManager.mContext.m_opennicontext.waitOneUpdateAll(orcManager.mDepthData.getGenerator());
//                    orcManager.getTextureMat(textureMat);//读取彩图矩阵
                    orcManager.getDepthMat(depthMat);//读取深度图矩阵
                    long endTime2 = System.currentTimeMillis();
                    Log.d("one", "one wait time" + String.valueOf(endTime2 - endTime1));

//                    int adjustX;
//                    int adjustY;
                    synchronized (MyApplication.sInstance) {
                        adjustX = MyApplication.sInstance.getAdjustX();
                        adjustY = MyApplication.sInstance.getAdjustY();
                    }
                    //Log.d("HelloProjection",String.valueOf(depthMat.height())+"   "+String.valueOf(depthMat.width()));
//                    scheduledThreadPool.execute(gestureCommand);
                    String string = null;
                    if (Util.getForegroundActivity(getApplicationContext()).equals("com.multimedia.Cake")) {
                        Log.d("HelloProjection", "ok");
                        string = NativeGestureDetect.getNativeGestureResult(orcManager.mCalibration,
                                textureMat.getNativeObjAddr(), depthMat.getNativeObjAddr(), adjustX, adjustY, 1);//手势识别
                    }else if (Util.getForegroundActivity(getApplicationContext()).equals("com.multimedia.Water")) {
                        Log.d("HelloProjection", "ok");
                        string = NativeGestureDetect.getNativeGestureResult(orcManager.mCalibration,
                                textureMat.getNativeObjAddr(), depthMat.getNativeObjAddr(), adjustX, adjustY, 2);//手势识别
                    } else if (Util.getForegroundActivity(getApplicationContext()).equals("com.netease.arprojectorinair")) {
                        string = NativeGestureDetect.getNativeGestureResult(orcManager.mCalibration,
                                textureMat.getNativeObjAddr(), depthMat.getNativeObjAddr(), adjustX, adjustY, 3);//手势识别
                        Log.d("HelloProjection", "ok1");
                    } else {
                        string = NativeGestureDetect.getNativeGestureResult(orcManager.mCalibration,
                                textureMat.getNativeObjAddr(), depthMat.getNativeObjAddr(), adjustX, adjustY, 0);//手势识别
                    }
                    //scheduledThreadPool.execute(inAirGestureCommand);
//                    }else if(""==Util.getForegroundActivity(getApplicationContext())){
//                        // scheduledThreadPool.execute(projectionCommand);
//                    }

                    Intent intent = new Intent();
                    intent.putExtra("gesture", string);
                    intent.setAction("com.netease.action.RECEIVE_DATA");
                    sendBroadcast(intent);

                    long endTime3 = System.currentTimeMillis();
                    //Log.d("two","two detection time"+String.valueOf(endTime3-endTime2));

                    //Log.d("mock",string);
                    long endTime4 = System.currentTimeMillis();
                    //Log.d("three","three"+String.valueOf(endTime4-endTime3));
                    Log.d("four", "four all time" + String.valueOf(endTime4 - endTime1));

                } catch (Exception e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }

            }

        }
    }

    private String getCalibrationParas() {
        SharedPreferences sharedPreferences = getSharedPreferences("CalibrationParas", Context.MODE_PRIVATE);
        String string = sharedPreferences.getString("calibjson", null);
        try {
            JSONObject jsonObject = new JSONObject();
            //jsonObject.put("Rect_X", 100);//100
            //jsonObject.put("Rect_Y", 50);//50
            //jsonObject.put("Rect_W", 470);//470
            //jsonObject.put("Rect_H", 300);//300
            jsonObject.put("Rect_X", 35);//100
            jsonObject.put("Rect_Y", 120);//50
            jsonObject.put("Rect_W", 470);//470
            jsonObject.put("Rect_H", 300);//300+

            jsonObject.put("Filt_X", 48);//48
            jsonObject.put("Filt_Y", 35);//35
            jsonObject.put("Filt_W", 40);//40
            jsonObject.put("Filt_H", 30);//30
            jsonObject.put("Grid_X", 25);//25
            jsonObject.put("Grid_Y", 47);//47
            jsonObject.put("Offset_X", 0);
            jsonObject.put("Offset_Y", 0);
            string = jsonObject.toString();
            SharedPreferences.Editor editor = sharedPreferences.edit();//获取编辑器
            editor.putString("calibjson", string);
            editor.commit();//提交修改
        } catch (Exception e) {
            e.printStackTrace();
        }

        return string;
    }

//    @Override
//    public void onWindowFocusChanged(boolean hasFocus) {
//        super.onWindowFocusChanged(hasFocus);
//        View popupView = getLayoutInflater().inflate(R.layout.calibration, null);
//        mPopupWindow = new PopupWindow(popupView, ActionBar.LayoutParams.MATCH_PARENT, ActionBar.LayoutParams.MATCH_PARENT);
//        View rootview = LayoutInflater.from(MainActivity.this).inflate(R.layout.main, null);
//        mPopupWindow.showAtLocation(rootview, Gravity.BOTTOM, 0, 0);
//    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        if (orcManager.mRgbData != null) {
            orcManager.mRgbData.close();
        }
        if (orcManager.mLiveCamera != null) {
            Log.d(TAG, "mLiveCamera.terminate()!!!");
            orcManager.mLiveCamera.terminate();
        }

        if (orcManager.mDepthData != null) {
            orcManager.mDepthData.close();
        }

        if (orcManager.mContext != null) {
            orcManager.mContext.close();
        }

        Log.i(TAG, "onDestroy!");
        System.exit(0);
    }

    static {
        System.loadLibrary("opencv_java");
        System.loadLibrary("jni-input");
    }

    /**
     * 保存raw文件夹下的配置文件
     */
    public String saveConfigFile()
    {
//        try {
//            File cascadeDir;
//            File mCascadeFile;
//            InputStream isCascade = getResources().openRawResource(R.raw.cascade_zfz);
//            cascadeDir = getDir("cascade", Context.MODE_PRIVATE);
//            mCascadeFile = new File(cascadeDir, "cascade_zfz.xml");
//            FileOutputStream osCascade = new FileOutputStream(mCascadeFile);
//            byte[] buffer1 = new byte[4096];
//            int bytesRead1;
//            while ((bytesRead1 = isCascade.read(buffer1)) != -1)
//            {
//                osCascade.write(buffer1, 0, bytesRead1);
//            }
//            isCascade.close();
//            osCascade.close();
//
//            CascadeClassifier mJavaDetector = new CascadeClassifier(mCascadeFile.getAbsolutePath());
//            if (mJavaDetector.empty()) {
//                Log.e(TAG, "Failed to load cascade classifier");
//            } else
//            {
//                Log.i(TAG, "Loaded cascade classifier from " + mCascadeFile.getAbsolutePath());
//            }
//            return mCascadeFile.getAbsolutePath();
//
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
        return null;


    }
}
