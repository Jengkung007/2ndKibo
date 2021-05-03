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

    @Override
    protected void runPlan1(){
        // write here your plan 1
        api.startMission();
        moveToWrapper(11.71,-9.53,5.35,0,0,-0.707,0.707);
        double[] a_prime  = qrCodeReader();
        double[] prime_avoid = calculatePattern(a_prime);
        api.takeSnapshot();
        moveToWrapper(10.275,-10.314,4.295,0,0,-0.707,0.707);
        api.reportMissionCompletion();



    }

    @Override
    protected void runPlan2(){}
    @Override
    protected void runPlan3(){}

    // You can add your method
    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w){
        final int LOOP_MAX = 3;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                                                     (float)qua_z, (float)qua_w);
        Result result = api.moveTo(point, quaternion, true);
        int loopCounter = 0;
        while(!result.hasSucceeded() || loopCounter < LOOP_MAX){
            result = api.moveTo(point, quaternion, true);
            ++loopCounter;
        }
    }
    String MODE = "sim"; // mode setting ("sim" or "iss")
    int NAV_MAX_COL = 1280;
    int NAV_MAX_ROW =  960;
    int PointCloud_COL = 224;
    int PointCloud_ROW = 171;
    // carmera constant value
    int max_count = 5, center_range = 6, P1 = 0, P2 = 1;
    // limit value
    float AR_diagonal = 0.07071067812f;
    float ARtoTarget = 0.1414f, y_shift = 0.1328f;

    public Mat undistord(Mat src){
        Mat dst = new Mat(1280, 960, CvType.CV_8UC1);
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);
        int row = 0, col = 0;
        double cameraMatrix_sim[] = {
                344.173397, 0.000000, 630.793795,
                0.000000, 344.277922, 487.033834,
                0.000000, 0.000000, 1.000000};
        double distCoeffs_sim[] = {-0.152963, 0.017530, -0.001107, -0.000210, 0.000000};
        double cameraMatrix_orbit[] = {
                692.827528, 0.000000, 571.399891,
                0.000000, 691.919547, 504.956891,
                0.000000, 0.000000, 1.000000};
        double distCoeffs_orbit[] = {-0.312191, 0.073843, -0.000918, 0.001890, 0.000000};
        if(MODE == "sim") {
            cameraMatrix.put(row, col, cameraMatrix_sim);
            distCoeffs.put(row, col, distCoeffs_sim);
            Log.d("Mode[camera]:"," sim");
        }
        else if(MODE == "iss") {
            cameraMatrix.put(row, col, cameraMatrix_orbit);
            distCoeffs.put(row, col, distCoeffs_orbit);
            Log.d("Mode[camera]:"," iss");
        }
        cameraMatrix.put(row, col, cameraMatrix_orbit);
        distCoeffs.put(row, col, distCoeffs_orbit);
        Imgproc.undistort(src, dst, cameraMatrix, distCoeffs);
        return dst;
    }

    private Rect cropImage(int percent_crop) {
        double ratio = NAV_MAX_COL / NAV_MAX_ROW;
        double percent_row = percent_crop/2;
        double percent_col = percent_row * ratio;
        int offset_row = (int) percent_row * NAV_MAX_ROW / 100;
        int offset_col = (int) percent_col * NAV_MAX_COL / 100;
        double rows = NAV_MAX_ROW - (offset_row * 2);
        double cols = NAV_MAX_COL - (offset_col * 2);
        return new Rect(offset_col, offset_row, (int) cols, (int) rows);
    }

    private Bitmap resizeImage(Mat src, int width, int height) {
        Size size = new Size(width, height);
        Imgproc.resize(src, src, size);
        Bitmap bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        matToBitmap(src, bitmap, false);
        return bitmap;
    }

    private void flash_front(float state) {
        try {
            api.flashlightControlFront(state);
            Thread.sleep(500);
        } catch (Exception e) {}
    }

    private double[] qrCodeReader() {
        double qr_content[] = null;
        int count = 0;
        flash_front(0.8f);
        while (qr_content == null && count < 5) {
            Mat src_mat = new Mat(undistord(api.getMatNavCam()), cropImage(40));
            Bitmap bMap = resizeImage(src_mat, 2000, 1500);
            int[] intArray = new int[bMap.getWidth() * bMap.getHeight()];
            bMap.getPixels(intArray, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());
            LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(), intArray);
            BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));
            try {
                com.google.zxing.Result result = new QRCodeReader().decode(bitmap);
                JSONObject res = new JSONObject(result.getText());
                qr_content = new double[] {res.getDouble("p"),res.getDouble("x"),res.getDouble("y"),res.getDouble("z")};
                api.sendDiscoveredQR(result.getText());
            }
            catch (Exception e)
            {
                Log.d("QR[status]:", " Can't parse JSON");
            }
            count++;
        }
        Log.d("QR[status]:", " Passed");
        flash_front(0f);
        return qr_content;
    }

    private double [] calculatePattern(double[] a_prime) {
        int pattern = (int)a_prime[0];
        double x_prime = a_prime[1];
        double y_prime = a_prime[2];
        double z_prime = a_prime[3];
        double[][][] obstacles = {{{0.075,0.3},{-0.3,-0.075},{-0.075,0},{0,0.075}},{{-0.3,-0.075},{-0.075,0},{0,0.075},{0.075,0.3}}};
        double[] x_avoid = {x_prime+obstacles[0][(int)pattern%4][0],x_prime+obstacles[0][(int)pattern%4][1]};
        double[] z_avoid = {z_prime+obstacles[0][(int)pattern/4][0],z_prime+obstacles[0][(int)pattern/4][1]};
        double[] y_avoid = {y_prime,y_prime+1.785};
        
        return new double[] {};
    }

}

