package com.r1k.utilities.allanvariancesampler;

import android.hardware.SensorEvent;
import android.util.Log;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class SensorSample {

    public float x;
    public float y;
    public float z;
    public long ts;

    public static String arrayToString(ArrayList<SensorSample> a) {
        StringBuilder sb = new StringBuilder();
        for (SensorSample s : a) {
            sb.append(s.toString());
            sb.append("\n");
        }
        return sb.toString();
    }

    public static SensorSample tryCreate(SensorEvent event, int samplingPeriodMs, long lastTimeStamp)
    {
        if (lastTimeStamp == 0) {
            return new SensorSample(event);
        } else if (TimeUnit.MILLISECONDS.convert(event.timestamp - lastTimeStamp, TimeUnit.NANOSECONDS) >= (samplingPeriodMs - 1)) {
            return new SensorSample(event);
        } else {
            return null;
        }
    }

    public static SensorSample tryCreate(float x, float y, float z, long ts, int samplingPeriodMs, long lastTimeStamp)
    {
        if (lastTimeStamp == 0) {
            return new SensorSample(x,y,z,ts);
        } else if (TimeUnit.MILLISECONDS.convert(ts - lastTimeStamp, TimeUnit.NANOSECONDS) >= (samplingPeriodMs - 1)) {
            return new SensorSample(x,y,z,ts);
        } else {
//            Log.d("SensorSample", "tryCreate: " + lastTimeStamp);
//            Log.d("SensorSample", "tryCreate: " + TimeUnit.MILLISECONDS.convert(ts - lastTimeStamp, TimeUnit.NANOSECONDS));
            return null;
        }
    }

    public SensorSample(float x, float y, float z, long ts)
    {
        this.ts = ts;
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public SensorSample(SensorEvent event)
    {
        ts = event.timestamp;
        x = event.values[0];
        y = event.values[1];
        z = event.values[2];
    }

    public String toString() {
        return ts+","+x+","+y+","+z;
    }
}
