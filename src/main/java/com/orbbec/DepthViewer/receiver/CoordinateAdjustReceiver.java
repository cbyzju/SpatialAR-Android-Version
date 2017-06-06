package com.orbbec.DepthViewer.receiver;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.util.Log;

import com.orbbec.DepthViewer.MyApplication;

import org.json.JSONObject;

/**
 * Created by Administrator on 2016/12/1.
 */

public class CoordinateAdjustReceiver extends BroadcastReceiver {
    @Override
    public void onReceive(Context context, Intent intent) {
        synchronized (MyApplication.sInstance){
            MyApplication.sInstance.set(intent.getIntExtra("adjustX",0),intent.getIntExtra("adjustY",0));
        }
        if(true==intent.getBooleanExtra("all_done",false)){
            Log.d("mydemo","saveOffset");
            writeOffset(intent.getIntExtra("adjustX",0),intent.getIntExtra("adjustY",0),context);
        }

    }

    private void writeOffset(int x,int y ,Context context){
        SharedPreferences sharedPreferences = context.getSharedPreferences("CalibrationParas", Context.MODE_PRIVATE);
        String string=sharedPreferences.getString("calibjson",null);
        if(string!=null){
            try {
                JSONObject jsonObject=new JSONObject(string);
                jsonObject.put("Offset_X",x+jsonObject.getInt("Offset_X"));
                jsonObject.put("Offset_Y",y+jsonObject.getInt("Offset_Y"));
                SharedPreferences.Editor editor = sharedPreferences.edit();//获取编辑器
                Log.d("mydemo",jsonObject.toString());
                editor.putString("calibjson", jsonObject.toString());
                editor.commit();//提交修改
            }catch (Exception e){
                e.printStackTrace();
            }
        }
    }
}
