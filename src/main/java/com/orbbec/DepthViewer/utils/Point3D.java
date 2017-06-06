package com.orbbec.DepthViewer.utils;

/**
 * Created by zlh on 2015/1/17.
 */
public class Point3D{

    public float X;
    public float Y;
    public float Z;

    public Point3D(float x, float y, float z)
    {
        this.X = x;
        this.Y = y;
        this.Z = z;
    }

    public void setValue(float x, float y, float z){
        this.X = x;
        this.Y = y;
        this.Z = z;
    }

    public static float Distance(Point3D p1, Point3D p2)
    {
        float xD = Math.abs(p1.X - p2.X);
        float yD = Math.abs(p1.Y - p2.Y);
        float zD = Math.abs(p1.Z - p2.Z);
        return (float) Math.sqrt(Math.pow(xD, 2) + Math.pow(yD, 2) + Math.pow(zD, 2));
    }

    public static float Length(Point3D p)
    {
        return (float) Math.sqrt(Math.pow(p.X, 2) + Math.pow(p.Y, 2) + Math.pow(p.Z, 2));
    }
}
