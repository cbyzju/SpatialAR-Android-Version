package com.orbbec.NativeNI;

/**
 * Created by zlh on 2015/7/29.
 */
public class NIEngine {

	private static final String TAG = NIEngine.class.getSimpleName();

	Thread mDataThread;
	boolean mUpdateData;
	private final int mRGBWidth = 640;
	private final int mRGBHeight = 480;
	private final int mDepthWidth = 160;
	private final int mDepthHeight = 120;

	private int[] mRGBData = new int[mRGBWidth * mRGBHeight];

	private int[] mDepthData = new int[mDepthWidth * mDepthHeight];

	private boolean mEnableRGB = true;

	public interface OnDepthDataUpdateListener {
		public void onDepthDataUpdateListener(int[] data, int width,
											  int height, int handX, int handY);
	}

	public interface OnRGBDataUpdateListener {
		public void onRGBDataUpdateListener(int[] data, int width, int height,
											int handX, int handY);
	}


	private OnDepthDataUpdateListener mOnDepthDataUpdateListener;
	private OnRGBDataUpdateListener mOnRGBDataUpdateListener;


	public void setOnDepthDataUpdateListener(OnDepthDataUpdateListener listener) {
		mOnDepthDataUpdateListener = listener;
	}

	public void setOnRGBDataUpdateListener(OnRGBDataUpdateListener listener) {
		mOnRGBDataUpdateListener = listener;
	}

	private static NIEngine mInstance = null;

	public static NIEngine getInstance() {
		if (null == mInstance) {
			mInstance = new NIEngine();
		}
		return mInstance;
	}

	private NIEngine() {

		mUpdateData = true;

		mDataThread = new Thread() {
			@Override
			public void run() {
				while (mUpdateData) {

					NativeMethod.Update();

					updateData();
				}
			}
		};
	}

	public void enableRGB(boolean enable) {
		mEnableRGB = enable;
	}

	private void updateData(){

        Integer hand2dX = new Integer(0);
        Integer hand2dY = new Integer(0);
        Integer hand2dZ = new Integer(0);
        NativeMethod.GetHandPos2D(hand2dX, hand2dY, hand2dZ);

        // conversionTest();

        int facotrX = mRGBWidth  / mDepthWidth;
        int factorY = mRGBHeight / mDepthHeight;
        int rc = 0;

        if(mOnRGBDataUpdateListener != null && mEnableRGB){
           rc = NativeMethod.GetRGBData(mRGBData);
           if(rc >= 0)
           {
        	   mOnRGBDataUpdateListener.onRGBDataUpdateListener(mRGBData, mRGBWidth, mRGBHeight, hand2dX * facotrX, hand2dY * factorY);
           }
            
        }

        if(mOnDepthDataUpdateListener != null){
            NativeMethod.GetDepthData(mDepthData);
            mOnDepthDataUpdateListener.onDepthDataUpdateListener(mDepthData, mDepthWidth, mDepthHeight, hand2dX, hand2dY);
        }
    }


	public void start() {

		NativeMethod.Init();
		NativeMethod.EnableLog(true);

		mDataThread.start();
	}

	public void stop() {
		try {
			mUpdateData = false;
			mDataThread.join();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		NativeMethod.ReleaseSensor();
	}

}
