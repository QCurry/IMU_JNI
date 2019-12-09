package com.r1k.utilities.allanvariancesampler;

import android.Manifest;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Build;
import android.os.Looper;
import android.support.annotation.RequiresApi;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class MainActivity extends AppCompatActivity implements SensorEventListener {

    static {
        System.loadLibrary("allanVarianceSampler");
    }

    private int samplingPeriodMs = 5;
    private long firstTimestamp = 0;
    private TextView firstSampleText;

    private Sensor acc;
    private long lastAccTimeStamp = 0;
    private ArrayList<SensorSample> accSamples = new ArrayList<SensorSample>();
    private TextView accText;
    private int totalAccSamples = 0;
    private FileOutputStream accF;
    private PrintWriter accPW;
    private long accHzCounterTimestamp = 0;
    private int accHzCounter = 0;
    private int accHz = 0;

    private Sensor gyr;
    private long lastGyrTimestamp = 0;
    private ArrayList<SensorSample> gyrSamples = new ArrayList<SensorSample>();
    private TextView gyrText;
    private int totalGyrSamples = 0;
    private FileOutputStream gyrF;
    private PrintWriter gyrPW;
    private long gyrHzCounterTimestamp = 0;
    private int gyrHzCounter = 0;
    private int gyrHz = 0;

    public static boolean USE_BMI_160_IMU = true;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        //getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        if (ContextCompat.checkSelfPermission(this, Manifest.permission.WRITE_EXTERNAL_STORAGE)
                != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this,
                    new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE}, 100);
        }

        final Button button = findViewById(R.id.startSampling);
        button.setOnClickListener(new View.OnClickListener() {
            @RequiresApi(api = Build.VERSION_CODES.O)
            public void onClick(View v) {
                StartSampling();
            }
        });
        accText = findViewById(R.id.accText);
        gyrText = findViewById(R.id.gyrText);
        firstSampleText = findViewById(R.id.fistSample);

        try {
            accF = openFile("acc");
            gyrF = openFile("gyr");
            accPW = new PrintWriter(accF);
            gyrPW = new PrintWriter(gyrF);

        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        Log.d("Curry", "onCreate: " + Thread.currentThread());
    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public void StartSampling() {
        if (USE_BMI_160_IMU) {
            Log.d("Curry", "StartSampling: " + Thread.currentThread());
            setup();
            return;
        }

        SensorManager sensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
        acc = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED);
        gyr = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED);

        int samplingPeriod = (int)TimeUnit.MICROSECONDS.convert(samplingPeriodMs, TimeUnit.MILLISECONDS);
        int latency = (int)TimeUnit.MICROSECONDS.convert(1, TimeUnit.SECONDS);

        sensorManager.registerListener(this, acc, samplingPeriod, latency);
        sensorManager.registerListener(this, gyr, samplingPeriod, latency);
    }

    @Override
    protected void onDestroy() {
        if (USE_BMI_160_IMU) {
            teardown();
            super.onDestroy();
            return;
        }

        if (acc != null)
        {
            SensorManager sensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
            sensorManager.unregisterListener(this);
        }
        accPW.close();
        gyrPW.close();
        try {
            accF.close();
            gyrF.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

        super.onDestroy();
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (firstTimestamp == 0) {
            firstTimestamp = event.timestamp;
        } else {
            firstSampleText.setText("Sampling for "+TimeUnit.MINUTES.convert(event.timestamp - firstTimestamp, TimeUnit.NANOSECONDS)+" minutes");
        }
        if (event.sensor == acc) {
            if (accHzCounterTimestamp == 0) {
                accHzCounterTimestamp = event.timestamp;
            }
            SensorSample sample = SensorSample.tryCreate(event, samplingPeriodMs, lastAccTimeStamp);
            if (sample != null) {
                accHzCounter++;
                if (TimeUnit.SECONDS.convert(event.timestamp - accHzCounterTimestamp, TimeUnit.NANOSECONDS) > 0) {
                    accHz = accHzCounter;
                    accHzCounterTimestamp = event.timestamp;
                    accHzCounter = 1;
                }
                lastAccTimeStamp = event.timestamp;
                accSamples.add(sample);
                accPW.write(sample.toString()+"\n");
                totalAccSamples++;
                if (accSamples.size() > 100) {
                    accSamples.remove(0);
                }
                accText.setText("ACC ["+totalAccSamples+"] @ "+accHz+" Hz: \n"+SensorSample.arrayToString(accSamples));
            }
        } else if (event.sensor == gyr) {
            if (gyrHzCounterTimestamp == 0) {
                gyrHzCounterTimestamp = event.timestamp;
            }
            SensorSample sample = SensorSample.tryCreate(event, samplingPeriodMs, lastGyrTimestamp);
            if (sample != null) {
                gyrHzCounter++;
                if (TimeUnit.SECONDS.convert(event.timestamp - gyrHzCounterTimestamp, TimeUnit.NANOSECONDS) > 0) {
                    gyrHz = gyrHzCounter;
                    gyrHzCounterTimestamp = event.timestamp;
                    gyrHzCounter = 1;
                }
                lastGyrTimestamp = event.timestamp;
                gyrSamples.add(sample);
                gyrPW.write(sample.toString()+"\n");
                totalGyrSamples++;
                if (gyrSamples.size() > 100) {
                    gyrSamples.remove(0);
                }
                gyrText.setText("GYRO ["+totalGyrSamples+"] @ "+gyrHz+" Hz : \n"+SensorSample.arrayToString(gyrSamples));
            }
        }
    }

    public void onSensorChangedBMI160(float accx, float accy, float accz, float gyrx, float gyry, float gyrz, long ts) {
        Log.d("Main", "onSensorChangedBMI160: " + System.nanoTime());
//        boolean isOnUIThread = (Looper.myLooper() == Looper.getMainLooper());
//        long tid = Thread.currentThread().getId();
//        Log.d("Curry", "onSensorChangedBMI160: " + Thread.currentThread());

        String firstSampleStr = null;
        String accStr = null;
        String gyrStr = null;

        if (firstTimestamp == 0) {
            firstTimestamp = ts;
        } else {
            firstSampleStr = "Sampling for "+TimeUnit.MINUTES.convert(ts - firstTimestamp, TimeUnit.NANOSECONDS)+" minutes";
        }

        if (accHzCounterTimestamp == 0) {
            accHzCounterTimestamp = ts;
        }
        SensorSample accSample = SensorSample.tryCreate(accx,accy, accz,ts, samplingPeriodMs, lastAccTimeStamp);
        if (accSample != null) {
            accHzCounter++;
            if (TimeUnit.SECONDS.convert(ts - accHzCounterTimestamp, TimeUnit.NANOSECONDS) > 0) {
                accHz = accHzCounter;
                accHzCounterTimestamp = ts;
                accHzCounter = 1;
            }
            lastAccTimeStamp = ts;
            accSamples.add(accSample);
            accPW.write(accSample.toString()+"\n");
            totalAccSamples++;
            if (accSamples.size() > 100) {
                accSamples.remove(0);
            }
            accStr = "ACC ["+totalAccSamples+"] @ "+accHz+" Hz: \n"+SensorSample.arrayToString(accSamples);
        }

        if (gyrHzCounterTimestamp == 0) {
            gyrHzCounterTimestamp = ts;
        }
        SensorSample gyrSample = SensorSample.tryCreate(gyrx,gyry,gyrz,ts, samplingPeriodMs, lastGyrTimestamp);
        if (gyrSample != null) {
            gyrHzCounter++;
            if (TimeUnit.SECONDS.convert(ts - gyrHzCounterTimestamp, TimeUnit.NANOSECONDS) > 0) {
                gyrHz = gyrHzCounter;
                gyrHzCounterTimestamp = ts;
                gyrHzCounter = 1;
            }
            lastGyrTimestamp = ts;
            gyrSamples.add(gyrSample);
            gyrPW.write(gyrSample.toString()+"\n");
            totalGyrSamples++;
            if (gyrSamples.size() > 100) {
                gyrSamples.remove(0);
            }
            gyrStr = "GYRO ["+totalGyrSamples+"] @ "+gyrHz+" Hz : \n"+SensorSample.arrayToString(gyrSamples);
        }

//        runOnUiThread(new BMI160EventRunnable(firstSampleStr, accStr, gyrStr) {
//            @Override
//            public void run() {
//                if (firstSampleStr != null) {
//                    firstSampleText.setText(firstSampleStr);
//                }
//                if (accStr != null) {
//                    accText.setText(accStr);
//                }
//                if (gyrStr != null) {
//                    gyrText.setText(gyrStr);
//                }
//            }
//        });
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }

    private FileOutputStream openFile(String name) throws FileNotFoundException {
        File root = android.os.Environment.getExternalStorageDirectory();

        File dir = new File (root.getAbsolutePath() + "/allanvariance");
        dir.mkdirs();
        File file = new File(dir, name+".csv");
        if (file.exists()) {
            file.delete();
        }
        return new FileOutputStream(file);
    }

    public native void setup();

    public native void teardown();

    private abstract class BMI160EventRunnable implements Runnable {
        float accx;
        float accy;
        float accz;
        float gyrx;
        float gyry;
        float gyrz;
        long ts;
        String firstSampleStr;
        String accStr;
        String gyrStr;

        public BMI160EventRunnable(String firstSampleText, String accText, String gyrText) {
            this.firstSampleStr = firstSampleText;
            this.accStr = accText;
            this.gyrStr = gyrText;
        }

        public BMI160EventRunnable(float accx, float accy, float accz, float gyrx, float gyry, float gyrz, long ts) {
            this.accx = accx;
            this.accy = accy;
            this.accz = accz;
            this.gyrx = gyrx;
            this.gyry = gyry;
            this.gyrz = gyrz;
            this.ts = ts;
        }
    }
}
