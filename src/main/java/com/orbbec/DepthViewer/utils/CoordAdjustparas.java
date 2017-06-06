package com.orbbec.DepthViewer.utils;

import android.os.ParcelUuid;

/**
 * Created by Administrator on 2016/12/1.
 */

public class CoordAdjustparas {
    public class Point{

    }
    private int adjustX;
    private int adjustY;

    public static CoordAdjustparas sCoordAdjustparas;

    public CoordAdjustparas() {
    }

    public  static synchronized CoordAdjustparas getInstance(){
        return sCoordAdjustparas=new CoordAdjustparas();
    }

    public void set(int x,int y){
        adjustX=x;
        adjustY=y;
    }
    public int getAdjustX(){
        return adjustX;
    }
    public int getAdjustY(){
        return adjustY;
    }
}
