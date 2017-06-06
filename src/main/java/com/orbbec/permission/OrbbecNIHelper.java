package com.orbbec.permission;

import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Environment;
import android.util.Log;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.HashMap;
import java.util.Iterator;

/**
 * Created by Administrator on 2015/3/24.
 */


public class OrbbecNIHelper
{
    private Context mAndroidContext;
    private static final String OPENNI_ASSETS_DIR = "openni";
    HashMap<String, UsbDevice> deviceList;
    private String mActionUsbPermission;
    private DevicePermissionListener mDevicePermissionListener;

    static {
        System.loadLibrary("usb");
//        System.loadLibrary("OpenNI");
    }


    public OrbbecNIHelper(Context context)
    {
        this.mAndroidContext = context;
        try
        {

            for (String fileName : this.mAndroidContext.getAssets().list("openni")) {
                Log.v("XtionNIHelper", "fileName = " + fileName);
                retrieveXml(fileName);
            }
        }
        catch (IOException e)
        {
            Log.v("OrbbecNIHelper", "Can not get config Files!!");
            throw new RuntimeException(e);
        }
        this.mActionUsbPermission = (context.getPackageName() + ".USB_PERMISSION");
        Log.v("OrbbecNIHelper", "context.getPackageName() = " + context.getPackageName());
        IntentFilter filter = new IntentFilter(this.mActionUsbPermission);
        this.mAndroidContext.registerReceiver(this.mUsbReceiver, filter);
    }

    public void shutdown()
    {
        this.mAndroidContext.unregisterReceiver(this.mUsbReceiver);
    }

    private final BroadcastReceiver mUsbReceiver = new BroadcastReceiver()
    {
        public void onReceive(Context context, Intent intent)
        {
            String action = intent.getAction();
            if (OrbbecNIHelper.this.mActionUsbPermission.equals(action)) {
                synchronized (this)
                {
                    if (OrbbecNIHelper.this.mDevicePermissionListener == null) {
                        return;
                    }
                    UsbDevice device =
                            (UsbDevice)intent.getParcelableExtra("device");
                    if (device == null) {
                        return;
                    }
                    if (intent.getBooleanExtra("permission", false)) {
                        OrbbecNIHelper.this.mDevicePermissionListener.onDevicePermissionGranted(device);
                    } else {
                        OrbbecNIHelper.this.mDevicePermissionListener.onDevicePermissionDenied(device);
                    }
                }
            }
        }
    };

    public UsbDeviceConnection openDevice(UsbDevice device)
    {
        return
                ((UsbManager)this.mAndroidContext.getSystemService(Context.USB_SERVICE)).openDevice(device);
    }

    public void requestDevicePermission(UsbDevice device, DevicePermissionListener listener)
    {
        PendingIntent permissionIntent = PendingIntent.getBroadcast(this.mAndroidContext, 0, new Intent(
                this.mActionUsbPermission), 0);



        this.mDevicePermissionListener = listener;

                UsbManager manager = (UsbManager)this.mAndroidContext.getSystemService(Context.USB_SERVICE);

        manager.requestPermission(device, permissionIntent);
    }

    public HashMap<String, UsbDevice> getDeviceList()
    {
        return getDevInstall();
    }

    private HashMap<String, UsbDevice> getDevInstall()
    {
        UsbManager manager = (UsbManager)this.mAndroidContext.getSystemService(Context.USB_SERVICE);
        HashMap<String, UsbDevice> deviceList = manager.getDeviceList();
        Iterator<UsbDevice> iterator = deviceList.values().iterator();
        Log.d("TESTUSB",String.valueOf(deviceList.size()));
        while (iterator.hasNext())
        {
            UsbDevice device = (UsbDevice)iterator.next();
            int vendorId = device.getVendorId();
            int productId = device.getProductId();
            Log.d("TESTUSB","VID"+String.valueOf(vendorId)+"  PID"+String.valueOf(productId));

            if((vendorId != 7463 && vendorId != 11205) || ((productId != 1536) && (productId != 1537) &&
                                                          (productId != 1025) && (productId != 1026) &&
                                                          (productId != 1027) && (productId != 1028) &&
                                                          (productId != 1029) && (productId != 1030))){
                iterator.remove();
            }
        }
        return deviceList;
    }

    private void retrieveXml(String filename)
            throws IOException
    {
        InputStream is = this.mAndroidContext.getAssets().open("openni/" + filename);

        this.mAndroidContext.deleteFile(filename);
        OutputStream os = this.mAndroidContext.openFileOutput(filename, 0);

        byte[] buffer = new byte[is.available()];
        is.read(buffer);
        is.close();

        os.write(buffer);
        os.close();
    }

    public static abstract interface DevicePermissionListener
    {
        public abstract void onDevicePermissionGranted(UsbDevice paramUsbDevice);

        public abstract void onDevicePermissionDenied(UsbDevice paramUsbDevice);
    }



    public String getXmlFilePath(Context androidContext)
    {
        File externalConfigFile = new File(
                Environment.getExternalStorageDirectory(), "SamplesConfig.xml");
        if (externalConfigFile.exists()) {
            return externalConfigFile.getPath();
        }
        return androidContext.getFilesDir() + "/" + "SamplesConfig.xml";
    }


}
