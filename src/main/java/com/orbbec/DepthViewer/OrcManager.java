package com.orbbec.DepthViewer;

import com.orbbec.DepthViewer.UVC.LiveCamera;
import com.orbbec.DepthViewer.UVC.LiveStreamProcessor;
import com.orbbec.DepthViewer.utils.GlobalDef;
import com.orbbec.astrakernel.AstraContext;

import com.orbbec.astrakernel.PermissionCallbacks;
import com.orbbec.astrastartlibs.DepthData;
import com.orbbec.astrastartlibs.RGBData;

import org.opencv.core.Mat;
import org.openni.DepthMap;


import java.nio.ByteBuffer;

/**
 * Created by Administrator on 2016/11/9.
 */

public class OrcManager {

    AstraContext mContext;
    DepthData mDepthData;


    RGBData mRgbData;
    public byte[] mPixels;
    public ByteBuffer mTextureByteBuffer;
    public LiveCamera mLiveCamera;
    LiveStreamProcessor mProcessor = new LiveStreamProcessor();

    short[] mDepthArray;
    DepthMap mGetDepthData;
    public long mCalibration=0;

    boolean mExit = false;
    int mWidth;
    int mHeight;

    OrcManager(android.content.Context context, PermissionCallbacks callbacks){
        mContext=new AstraContext(context,callbacks);
    }

    public void init(){
        try{
            mRgbData = new RGBData(mContext); //初始化彩图
            mRgbData.setMapOutputMode(GlobalDef.RES_COLOR_WIDTH,
                    GlobalDef.RES_COLOR_HEIGHT, 30);
            mRgbData.getGenerator().getMirrorCapability().setMirror(false);// true,false

            mDepthData = new DepthData(mContext); //初始化深度图
            mDepthData.setMapOutputMode(640, 480, 30);
            mDepthData.setMirror(false);// true,false

            mWidth = mDepthData.getDepthMap().getXRes();
            mHeight = mDepthData.getDepthMap().getYRes();

            mDepthArray =new short[mWidth*mHeight];

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void getTextureMat(Mat textureMat){//得到彩图Mat矩阵
//        final ImageMap imageMap = mRgbData.getImageMap();
//        ByteBuffer byteBufferTemp = imageMap
//                .createByteBuffer();
//        mPixels = byteBufferTemp.array();
        if(null==mTextureByteBuffer){
            mTextureByteBuffer = mRgbData.createByteBuffer();
        }
        mRgbData.copyToBuffer(mTextureByteBuffer,mTextureByteBuffer.limit());
        mPixels=mTextureByteBuffer.array();
        textureMat.put(0,0, mPixels);
    }

    public void getDepthMat(Mat depthMat){   //得到深度图Mat矩阵
//        mGetDepthData = mDepthData.getDepthMap();
//        mGetDepthData.createShortBuffer().get(mDepthArray);
        mDepthData.getByteBuffer().asShortBuffer().get(mDepthArray);
        depthMat.put(0,0, mDepthArray);
    }

}
