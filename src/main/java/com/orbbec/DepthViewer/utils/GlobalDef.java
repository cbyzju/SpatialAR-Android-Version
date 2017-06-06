package com.orbbec.DepthViewer.utils;

/**
 * Created by zlh on 2015/8/2.
 */
public class GlobalDef {

    public static boolean USE_UVC;//浼氭牴鎹澶囨潵閫夋嫨鏄惁鎵撳紑UVC锛�    
    public static final boolean FPS_ON = false;
    public static String PACKAGE_NAME;

    // Resolution
    public static final int RES_DISPLAY_WIDTH = 640;//褰╄壊鍥炬樉绀哄搴︼紝鍦ㄩ潪UVC涓嬭缃负涓嶳ES_COLOR_WIDTH澶у皬涓�嚧锛屽惁鍒欎笌RES_UVC_WIDTH涓�嚧
    public static final int RES_DISPLAY_HEIGHT = 480;//褰╄壊鍥炬樉绀洪珮搴︼紝鍦ㄩ潪UVC涓嬭缃负涓嶳ES_COLOR_HEIGHT澶у皬涓�嚧锛屽惁鍒欎笌RES_UVC_HEIGHT涓�嚧

    public static final int RES_UVC_WIDTH = 640;//UVC褰╄壊鎽勫儚澶村垎杈ㄧ巼
    public static final int RES_UVC_HEIGHT = 480;
    public static final int RES_COLOR_WIDTH = 640;//鏅�鎽勫儚澶村垎杈ㄧ巼
    public static final int RES_COLOR_HEIGHT = 480;
    public static final int RES_DEPTH_WIDTH = 320;//娣卞害鍥惧垎杈ㄧ巼
    public static final int RES_DEPTH_HEIGHT = 240;

    public static boolean reCalibration=false;

}
