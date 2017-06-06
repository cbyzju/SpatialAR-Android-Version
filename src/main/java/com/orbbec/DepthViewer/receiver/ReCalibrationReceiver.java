package com.orbbec.DepthViewer.receiver;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.util.Log;

import com.orbbec.DepthViewer.utils.GlobalDef;

/**
 * Created by Administrator on 2016/12/3.
 */

public class ReCalibrationReceiver extends BroadcastReceiver {
    @Override
    public void onReceive(Context context, Intent intent) {
        Log.d("re","get");
        synchronized (GlobalDef.class){
            GlobalDef.reCalibration=true;
        }
    }
}
