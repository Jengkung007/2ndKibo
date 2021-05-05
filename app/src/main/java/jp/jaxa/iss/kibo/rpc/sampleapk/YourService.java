package jp.jaxa.iss.kibo.rpc.sampleapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.types.PointCloud;

import android.graphics.Bitmap;
import android.os.SystemClock;
import android.util.Log;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.LuminanceSource;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;
import static org.opencv.android.Utils.matToBitmap;

import java.util.ArrayList;
import java.util.List;
import org.json.JSONObject;
/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    
    int CHECKING_MAX = 5;
    double[][] nav_intrinsics = api.getNavCamIntrinsics();

    @Override
    protected void runPlan1(){
        api.startMission();
        
        scanQR(11.21, -9.8, 4.79,0.183f,0.183f,-0.683f,0.683f); //Move to QR
        // qrCodeReader(); //Read QR and return
        // calculatePattern(a_prime);
        // moveWithKOZ();
        // moveToWrapper(10.275,-10.314,4.295,0f,0f,-0.707f,0.707f);
        api.reportMissionCompletion();



    }

    @Override
    protected void runPlan2(){}
    @Override
    protected void runPlan3(){}

    private void move(double x, double y, double z,float xo, float yo, float zo,float wo) {
        Log.d("Move[status]:"," Start");
        Result result = api.moveTo(new Point(x,y,z),new Quaternion(xo,yo,zo,wo),true);
        int count = 0;
        while (!result.hasSucceeded() && count<CHECKING_MAX) {
            result = api.moveTo(new Point(x,y,z),new Quaternion(xo,yo,zo,wo),true);
            count++;
        }
        Log.d("Move[status]:"," Done");
    }

    private void flash(float per) {
        Log.d("Flash[status]:"," Start");
        Result result = api.flashlightControlFront(per);
        int count = 0;
        while (!result.hasSucceeded() && count<CHECKING_MAX) {
            result = api.flashlightControlFront(per);
            count++;
        }
        Log.d("Flash[status]:"," Done");
    }

    private Mat undistort(Mat src) {
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);
        Mat out = new Mat(1280,960,CvType.CV_8UC1);
        cameraMatrix.put(0,0,nav_intrinsics[0]);
        distCoeffs.put(0,0,nav_intrinsics[1]);
        Imgproc.undistort(src,out,cameraMatrix,distCoeffs);
        return out;
    }

    public Rect cropImage(int percent_crop) {
        double ratio = 1280 / 960;
        double percent_row = percent_crop/2;
        double percent_col = percent_row * ratio;
        int offset_row = (int) percent_row * 960 / 100;
        int offset_col = (int) percent_col * 1280 / 100;
        double rows = 960 - (offset_row * 2);
        double cols = 1280 - (offset_col * 2);
        return new Rect(offset_col, offset_row, (int) cols, (int) rows);
    }
    public Bitmap resizeImage(Mat src, int width, int height) {
        Size size = new Size(width, height);
        Imgproc.resize(src, src, size);
        Bitmap bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        matToBitmap(src, bitmap, false);
        return bitmap;
    }

    private void scanQR(double x, double y, double z,float xo, float yo, float zo,float wo) {
        String raw_qr = null;
        int count = 0;
        Log.d("QR[status]:"," Start");
        while (raw_qr==null && count<CHECKING_MAX) {
            move(x,y,z,xo,yo,zo,wo);
            Log.d("QR[status]:"," Scanning");
            flash(0.25f);
            Bitmap img = resizeImage(undistort(api.getMatNavCam()),2000,1500);
            int[] intArray = new int[img.getWidth() * img.getHeight()];
            img.getPixels(intArray, 0, img.getWidth(), 0, 0, img.getWidth(), img.getHeight());
            LuminanceSource source = new RGBLuminanceSource(img.getWidth(), img.getHeight(), intArray);
            BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));
            try {
                com.google.zxing.Result result = new QRCodeReader().decode(bitmap);
                raw_qr = result.getText();
                Log.d("QR[status]:", " Detected :"+raw_qr);
            }
            catch (Exception e)
            {
                Log.d("QR[status]:", " Not detected");
            }
            count++;
        }
        flash(0f);
        api.sendDiscoveredQR(raw_qr);
        Log.d("QR[status]:"," Done");
    }


}

