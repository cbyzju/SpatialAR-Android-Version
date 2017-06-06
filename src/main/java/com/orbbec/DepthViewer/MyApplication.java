package com.orbbec.DepthViewer;

import android.app.Application;

import com.orbbec.DepthViewer.utils.CoordAdjustparas;

/**
 * Created by Administrator on 2016/12/1.
 */

public class MyApplication extends Application {
    public static CoordAdjustparas sInstance;
    @Override
    public void onCreate() {
        super.onCreate();
         sInstance=CoordAdjustparas.getInstance();
    }
}
