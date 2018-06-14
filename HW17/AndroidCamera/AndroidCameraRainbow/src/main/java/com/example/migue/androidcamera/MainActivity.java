package com.example.migue.androidcamera;
// libraries

import android.Manifest;
import android.app.Activity;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.os.Bundle;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.TextureView;
import android.view.WindowManager;
import android.widget.TextView;
import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;



import java.io.IOException;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;
import static android.graphics.Color.rgb;


public class MainActivity extends Activity implements TextureView.SurfaceTextureListener {
    private Camera mCamera;
    private TextureView mTextureView;
    private SurfaceView mSurfaceView;
    private SurfaceHolder mSurfaceHolder;
    private Bitmap bmp = Bitmap.createBitmap(640, 480, Bitmap.Config.ARGB_8888);
    private Canvas canvas = new Canvas(bmp);
    private Paint paint1 = new Paint();
    private TextView mTextView;
    SeekBar sensitivityR;
    SeekBar sensitivityG;
    SeekBar sensitivityB;
    SeekBar sensitivityW;


    static long prevtime = 0; // for FPS calculation

    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON); // keeps the screen from turning off
        sensitivityR = (SeekBar)findViewById(R.id.seek1);
        sensitivityG = (SeekBar)findViewById(R.id.seek2);
        sensitivityB = (SeekBar)findViewById(R.id.seek3);
        sensitivityW = (SeekBar)findViewById(R.id.seek4);
        mTextView = (TextView) findViewById(R.id.cameraStatus);
        setMyControlListener1();
        setMyControlListener2();
        setMyControlListener3();
        setMyControlListener4();

        // see if the app has permission to use the camera
        ActivityCompat.requestPermissions(MainActivity.this, new String[]{Manifest.permission.CAMERA}, 1);
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
            mSurfaceView = (SurfaceView) findViewById(R.id.surfaceview);
            mSurfaceHolder = mSurfaceView.getHolder();

            mTextureView = (TextureView) findViewById(R.id.textureview);
            mTextureView.setSurfaceTextureListener(this);

            // set the paintbrush for writing text on the image
            paint1.setColor(0xffff0000); // red
            paint1.setTextSize(24);

            mTextView.setText("started camera");
        } else {
            mTextView.setText("no camera permissions");
        }

    }

    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        mCamera = Camera.open();
        Camera.Parameters parameters = mCamera.getParameters();
        parameters.setPreviewSize(640, 480);
        parameters.setFocusMode(Camera.Parameters.FOCUS_MODE_INFINITY); // no autofocusing
        parameters.setAutoExposureLock(false); // keep the white balance constant
        mCamera.setParameters(parameters);
        mCamera.setDisplayOrientation(90); // rotate to portrait mode

        try {
            mCamera.setPreviewTexture(surface);
            mCamera.startPreview();
        } catch (IOException ioe) {
            // Something bad happened
        }
    }

    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
        // Ignored, Camera does all the work for us
    }

    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        mCamera.stopPreview();
        mCamera.release();
        return true;
    }

    // the important function
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        // every time there is a new Camera preview frame
        mTextureView.getBitmap(bmp);
        int sumx=1;
        int countx=1;
        int sumy=1;
        int county=1;

        final Canvas c = mSurfaceHolder.lockCanvas();
        if (c != null) {
            int thresh1 = sensitivityR.getProgress(); // comparison value
            int thresh2 = sensitivityG.getProgress();
            int thresh3 = sensitivityB.getProgress();
            int thresh4 = sensitivityW.getProgress();
            int[] pixels = new int[bmp.getWidth()]; // pixels[] is the RGBA data
            int lineY = 100; // which row in the bitmap to analyze to read
            sumx=1;
            countx=1;
            sumy=1;
            county=1;
            for (int j = 0; j<bmp.getHeight()/10;j++){
                lineY=j*10;
                bmp.getPixels(pixels, 0, bmp.getWidth(), 0, lineY, bmp.getWidth(), 1);
                // in the row, see if there is more green than red
                for (int i = 0; i < bmp.getWidth(); i++) {
                  //  if ((red(pixels[i])- green(pixels[i]))> thresh1) {
                    //    if ((red(pixels[i])- blue(pixels[i]))> thresh2) {
                    if ((red(pixels[i])>thresh1||green(pixels[i])>thresh2||blue(pixels[i])>thresh3)||(red(pixels[i])-green(pixels[i])>thresh4&&
                            red(pixels[i])-blue(pixels[i])>thresh4&&
                            green(pixels[i])-blue(pixels[i])>thresh4)){

                            pixels[i] = rgb(0, 255, 0); // over write the pixel with pure green
                            sumx=i+sumx;
                            countx++;
                            sumy=j+sumy;
                            county++;
                        //}
                    }
                }
                // update the row
                bmp.setPixels(pixels, 0, bmp.getWidth(), 0, lineY, bmp.getWidth(), 1);
            }





        }
        int posx=sumx/countx;
        int posy=sumy/county*10;
        // draw a circle at some position
        int pos=50;
        canvas.drawCircle(posx,posy, 5, paint1); // x position, y position, diameter, color

        // write the pos as text
        canvas.drawText("posx = " + posx, 10, 200, paint1);
        canvas.drawText("posy = " + posy, 10, 300, paint1);
        c.drawBitmap(bmp, 0, 0, null);
        mSurfaceHolder.unlockCanvasAndPost(c);

        // calculate the FPS to see how fast the code is running
        long nowtime = System.currentTimeMillis();
        long diff = nowtime - prevtime;
        mTextView.setText("FPS " + 1000 / diff);
        prevtime = nowtime;
    }
    private void setMyControlListener1() {
        sensitivityR.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {

            int progressChanged = 0;

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                progressChanged = progress;
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
    }
    private void setMyControlListener2() {
        sensitivityG.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {

            int progressChanged = 0;

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                progressChanged = progress;
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
    }
    private void setMyControlListener3() {
        sensitivityB.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {

            int progressChanged = 0;

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                progressChanged = progress;
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
    }
    private void setMyControlListener4() {
        sensitivityR.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {

            int progressChanged = 0;

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                progressChanged = progress;
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
    }
}