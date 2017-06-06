package com.orbbec.permission;

/**
 * Created by Administrator on 2015/3/24.
 */
public abstract interface PermissionCallbacks
{
    public abstract void onDevicePermissionGranted();

    public abstract void onDevicePermissionDenied();
}

