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

public class YourService extends KiboRpcService {
    
    int CHECKING_MAX = 3;
    
    @Override
    protected void runPlan1(){
        api.startMission();
        double[] qr = scanQR(11.21,-9.8, 4.79,0f,0f,-0.707f,0.707f); //Move to QR

        moverAR((int)qr[0],qr[1],qr[2],qr[3]);
        // laserPewPew(qr[1],qr[2],qr[3]);
        api.laserControl(true);
        api.takeSnapshot();
        api.laserControl(false);
        mission_report((int)qr[0]);
        api.reportMissionCompletion();
    }

    @Override
    protected void runPlan2(){}
    @Override
    protected void runPlan3(){}

    private void move(double x, double y, double z,float xo, float yo, float zo,float wo,boolean navcam) {
        if(navcam) {
            x += 0.1177;
            y -= 0.0422;
            z += 0.0826;
        }
        Log.d("Move[status]:"," Start");
        Result result = api.moveTo(new Point(x,y,z),new Quaternion(xo,yo,zo,wo),true);
        int count = 0;
        while (!result.hasSucceeded() && count<CHECKING_MAX) {
            result = api.moveTo(new Point(x,y,z),new Quaternion(xo,yo,zo,wo),true);
            count++;
        }
        Log.d("Move[status]:"," Done");
    }

    private void moveDeg(double x_org, double y_org, double z_org, double x_des, double y_des, double z_des) {
        double dx = x_des-x_org;
        double dy = y_des-y_org;
        double dz = z_des-z_org;
        double magnitude = Math.sqrt((dx*dx)+(dy*dy)+(dz*dz));
        double x_unit = dx/magnitude;
        double y_unit = dy/magnitude;
        double z_unit = dz/magnitude;
        double matrix[][] ={{1, 0, 0},{x_unit, y_unit, z_unit}};
        double x = matrix[0][1]*matrix[1][2] - matrix[1][1]*matrix[0][2];
        double y = matrix[0][2]*matrix[1][0] - matrix[1][2]*matrix[0][0];
        double z = matrix[0][0]*matrix[1][1] - matrix[1][0]*matrix[0][1];
        double i = matrix[1][0]-matrix[0][0];
        double j = matrix[1][1]-matrix[0][1];
        double k = matrix[1][2]-matrix[0][2];
        double q = Math.sqrt(x*x + y*y + z*z);
        double p = Math.sqrt(i*i + j*j + k*k);
        double theta = Math.acos((2 - p*p) / 2);

        double a = Math.sin(theta/2)*x/q;
        double b = Math.sin(theta/2)*y/q;
        double c = Math.sin(theta/2)*z/q;
        double w = Math.cos(theta/2);

        double pitch = -Math.atan((2 * (a*w + b*c)) / (w*w - a*a - b*b + c*c));
        double roll = -Math.asin(2 * (a*c - b*w));
        double yaw = Math.atan((2 * (c*w + a*b)) / (w*w + a*a - b*b - c*c));
        double sx = (0.103 * Math.cos(roll + 0.279) / Math.cos(1.57080 + yaw));
        double sy = (0.103 * Math.sin(roll + 0.279) / Math.cos(pitch));

        move(x_org - sx, y_org, z_org + sy, (float)a, (float)b, (float)c, (float)w,false);
    }

    private Quaternion pos2Q(double x_org, double y_org, double z_org, double x_des, double y_des, double z_des) {
        double dx = x_des-x_org;
        double dy = y_des-y_org;
        double dz = z_des-z_org;
        double magnitude = Math.sqrt((dx*dx)+(dy*dy)+(dz*dz));
        double x_unit = dx/magnitude;
        double y_unit = dy/magnitude;
        double z_unit = dz/magnitude;
        double matrix[][] ={{1, 0, 0},{x_unit, y_unit, z_unit}};
        double x = matrix[0][1]*matrix[1][2] - matrix[1][1]*matrix[0][2];
        double y = matrix[0][2]*matrix[1][0] - matrix[1][2]*matrix[0][0];
        double z = matrix[0][0]*matrix[1][1] - matrix[1][0]*matrix[0][1];
        double i = matrix[1][0]-matrix[0][0];
        double j = matrix[1][1]-matrix[0][1];
        double k = matrix[1][2]-matrix[0][2];
        double q = Math.sqrt(x*x + y*y + z*z);
        double p = Math.sqrt(i*i + j*j + k*k);
        double theta = Math.acos((2 - p*p) / 2);

        double a = Math.sin(theta/2)*x/q;
        double b = Math.sin(theta/2)*y/q;
        double c = Math.sin(theta/2)*z/q;
        double w = Math.cos(theta/2);

        return new Quaternion((float)a, (float)b, (float)c, (float)w);
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

    private Mat undistord(Mat src) {
        double[][] nav_intrinsics = api.getNavCamIntrinsics();
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);
        Mat out = new Mat(1280,960,CvType.CV_8UC1);
        cameraMatrix.put(0,0,nav_intrinsics[0]);
        distCoeffs.put(0,0,nav_intrinsics[1]);
        Imgproc.undistort(src,out,cameraMatrix,distCoeffs);
        return out;
    }

    private Rect cropImage(int percent_crop) {
        double ratio = 1280 / 960;
        double percent_row = percent_crop/2;
        double percent_col = percent_row * ratio;
        int offset_row = (int) percent_row * 960 / 100;
        int offset_col = (int) percent_col * 1280 / 100;
        double rows = 960 - (offset_row * 2);
        double cols = 1280 - (offset_col * 2);
        return new Rect(offset_col, offset_row, (int) cols, (int) rows);
    }

    private Bitmap resizeImage(Mat src, int width, int height){
        Size size = new Size(width, height);
        Imgproc.resize(src, src, size);
        Bitmap bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        matToBitmap(src, bitmap, false);
        return bitmap;
    }

    private double[] scanQR(double x, double y, double z,float xo, float yo, float zo,float wo) {
        String raw_qr = null;
        int count = 0;
        Log.d("QR[status]:"," Start");
        double pattern=1,a_x=0,a_y=0,a_z=0;
        while (raw_qr==null && count<CHECKING_MAX) {
            count++;    
            move(x,y,z,xo,yo,zo,wo,true);
            Log.d("QR[status]:"," Scanning");
            flash(0.50f);
            Mat src_mat = new Mat(undistord(api.getMatNavCam()), cropImage(20));
            Bitmap bMap = resizeImage(src_mat, 1920,1080 );
            int[] intArray = new int[bMap.getWidth() * bMap.getHeight()];
            bMap.getPixels(intArray, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());
            LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(), intArray);
            BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));
            com.google.zxing.Result qrcode1 = null;
            try {
                Log.d("QR[status]:","Decoding...");
                qrcode1 = new QRCodeReader().decode(bitmap);
                raw_qr = qrcode1.getText();        
                Log.d("QR[status]:","Decoded!! Success");
            }
            catch (Exception e) {
                Log.d("QR[status]:","Decode failed????");
                y -=0.1;
                continue;
            }
        }
        flash(0f);
        api.sendDiscoveredQR(raw_qr);
        Log.d("QR[status]:"," Done");
        double[] r = new double[] {pattern,a_x,a_y,a_z} ;
        return r;
    }

    private void moverAR(int pattern,double x,double y,double z) {
        // top of target move(x,y,z-0.3,0f,0f,-0.707f,0.707f);
        flash(0.50f);
        switch (pattern) {
            case 1:
                move(x+3/16,y,z-3/16,0f,0f,-0.707f,0.707f,false);
                break;

            case 2:
                move(x,y,z-3/16,-0.123f,-0.123f,-0.696f,0.696f,false);
                break;
            case 3:
                move(x-3/16,y,z-6/16,0f,0f,-0.707f,0.707f,false);
                move(x-3/16,y,z-3/16,0f,0f,-0.707f,0.707f,false);
                break;

            case 4:
                move(x,y,z-6/16,0f,0f,-0.707f,0.707f,false);
                move(x-0.150,y,z,0f,0f,-0.707f,0.707f,false);
                break;

            case 5:
                move(x-9/16,y,z-8/16,0f,0f,-0.707f,0.707f,false);
                move(x-3/16,y,z,0f,0f,-0.707f,0.707f,false);
                break;

            case 6:
                move(x-9/16,y,z-8/16,0f,0f,-0.707f,0.707f,false);
                move(x-9/16,y,z+3/16,0f,0f,-0.707f,0.707f,false);
                move(x,y,z+3/16,0f,0f,-0.707f,0.707f,false);
                break;

            case 7:
                move(x+9/16,y,z-8/16,0f,0f,-0.707f,0.707f,false);
                move(x+9/16,y,z+3/16,0f,0f,-0.707f,0.707f,false);
                move(x,y,z+3/16,0f,0f,-0.707f,0.707f,false);
                break;

            case 8:
                move(x,y,z-6/16,0f,0f,-0.707f,0.707f,false);
                move(x+0.150,y,z,0f,0f,-0.707f,0.707f,false);
                break;
        
            default:
                break;
        }
    }

    private void laserPewPew(double ar_x,double ar_y, double ar_z) {
        Point pos_ = api.getRobotKinematics().getPosition();
        double pos_x = pos_.getX();
        double pos_y = pos_.getY();
        double pos_z = pos_.getZ();
        Quaternion q =  pos2Q(pos_x,pos_y,pos_z,ar_x,ar_y,ar_z);
        move(pos_x,pos_y,pos_z,q.getX(),q.getY(),q.getZ(),q.getW(),false);
        api.laserControl(true);
        api.takeSnapshot();
        api.laserControl(false);
        
    }

    private void mission_report(int pattern) {
        Point pos_ = api.getRobotKinematics().getPosition();
        double pos_x = pos_.getX();
        double pos_y = pos_.getY();
        double pos_z = pos_.getZ();
        if(pattern==1) {
            move(pos_x,pos_y,pos_z-3/16,0f,0f,-0.707f,0.707f,false);
            pos_ = api.getRobotKinematics().getPosition();
            pos_z = pos_.getZ();
        }
        if (pattern==7 || pattern==6) {
            move(pos_x,pos_y,pos_z+3/16,0f,0f,-0.707f,0.707f,false);
            pos_ = api.getRobotKinematics().getPosition();
            pos_z = pos_.getZ();
        }

        move(10.5,-8.6,pos_z,0f,0f,-0.707f,0.707f,false);
        move(10.5,-8.4,pos_z,0f,0f,-0.707f,0.707f,false);
        move(10.6, -8.0, 4.5,0f, 0f, -0.707f, 0.707f,false);
    }
    

}

