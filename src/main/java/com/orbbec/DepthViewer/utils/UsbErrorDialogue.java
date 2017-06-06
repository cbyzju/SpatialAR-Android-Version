package com.orbbec.DepthViewer.utils;


import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.view.WindowManager;

import com.orbbec.permission.PermissionGrant;



import java.lang.reflect.Field;

/**
 * Created by Administrator on 2016/11/30.
 */

public class UsbErrorDialogue  {
   public AlertDialog alertDialog;

    public UsbErrorDialogue(Context context) {
        AlertDialog.Builder builder = new AlertDialog.Builder(context);

        builder.setMessage("检测不到设备，请插入设备!");
        builder.setTitle("提示");

        builder.setPositiveButton("确定", new DialogInterface.OnClickListener()
        {
            public void onClick(DialogInterface dialog, int which)
            {
                dialog.dismiss();
            }
        });
        alertDialog = builder.create();
        alertDialog.getWindow().setType(WindowManager.LayoutParams.TYPE_SYSTEM_ALERT);//设定为系统级警告，关键
    }


}
