package com.orbbec.DepthViewer.utils;

import android.app.ActivityManager;
import android.content.Context;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.util.Log;

import java.util.HashMap;

/**
 * Created by netease on 17/1/3.
 */
public class Util {

    static  public HashMap<String, UsbDevice> getDevInstall(Context context)
    {
        UsbManager manager = (UsbManager)context.getSystemService(Context.USB_SERVICE);
        HashMap<String, UsbDevice> deviceList = manager.getDeviceList();
        return deviceList;
    }


    static public String getForegroundActivity(Context context) {
        ActivityManager mActivityManager =
                (ActivityManager) context.getSystemService(Context.ACTIVITY_SERVICE);
        if (mActivityManager.getRunningTasks(1) == null) {
//            Log.e(TAG, "running task is null, ams is abnormal!!!");
            return null;
        }
        ActivityManager.RunningTaskInfo mRunningTask =
                mActivityManager.getRunningTasks(1).get(0);
        if (mRunningTask == null) {
//            Log.e(TAG, "failed to get RunningTaskInfo");
            return null;
        }

        String pkgName = mRunningTask.topActivity.getPackageName();
        //String activityName =  mRunningTask.topActivity.getClassName();
        return pkgName;
    }
}
