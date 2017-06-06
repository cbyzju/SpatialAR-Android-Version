package com.orbbec.DepthViewer.utils;

import android.app.Dialog;
import android.content.Context;
import android.view.Display;
import android.view.View;
import android.view.WindowManager;

import com.orbbec.DepthViewer.R;

import static android.content.Context.WINDOW_SERVICE;

/**
 * Created by Administrator on 2016/11/15.
 */

public class CalibrationDialog extends Dialog{ //投标定图

    private Context mContext;

    public CalibrationDialog(Context context, int themeResId) {
        super(context, themeResId);
        this.mContext = context;

    }

    public void showCalibrationDialog() {  //弹出标定图
    /* create ui */
        View view = View.inflate(mContext,R.layout.calibration, null);
        setContentView(view);
        getWindow().setType(WindowManager.LayoutParams.TYPE_SYSTEM_ALERT);
        /* set size & pos */
        WindowManager.LayoutParams lp = getWindow().getAttributes();
        getWindow().setAttributes(lp);
        show();

    }
}
