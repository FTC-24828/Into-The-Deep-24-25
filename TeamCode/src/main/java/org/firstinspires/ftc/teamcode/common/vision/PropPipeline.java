package org.firstinspires.ftc.teamcode.common.vision;

import static org.firstinspires.ftc.teamcode.common.hardware.Global.DEBUG;
import static org.firstinspires.ftc.teamcode.common.hardware.Global.SIDE;
import static org.firstinspires.ftc.teamcode.common.hardware.Global.Side;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import static org.firstinspires.ftc.teamcode.common.hardware.Global.*;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.util.WMath;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class PropPipeline implements VisionProcessor, CameraStreamSource {
    private final AtomicReference<Bitmap> last_frame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    public Global.PropLocation location;

    public int[] blue_range =  {0, 50, 120, 255, 255, 255};
    public int[] red_range = {115, 50, 80, 140, 255, 255};
    public static int[] filter_range = new int[6];

    public double red_threshold = 1.8;
    public double blue_threshold = 1.8;
    public double threshold = 0;

    public double left_white;
    public double center_white;

    Mat mask = new Mat();
    Mat final_mat = new Mat();

    ArrayList<Rect> bounding_box = new ArrayList<>();

    //for displaying with ftc dashboard
    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(last_frame.get()));
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        last_frame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        if (SIDE == Side.BLUE) {
            threshold = blue_threshold;
            filter_range = blue_range.clone();
        } else if (SIDE == Side.RED) {
            threshold = red_threshold;
            filter_range = red_range.clone();
        } else {
            throw new EnumConstantNotPresentException(Global.Side.class, "Global.SIDE not initialized");
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        mask = frame.clone();

        Imgproc.GaussianBlur(mask, mask, new Size(11, 11), 0.0);
        mask = filterColor(mask);

        //clean up image
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(15, 7));
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);

        //get bounding box of contours
        processContours(mask);
        final_mat = mask.clone();

        if (bounding_box.size() > 1) {
            //sort left to right by x value
            bounding_box.sort(Comparator.comparingInt(a -> a.x));
            //get the total amount of pixel in threshold
            left_white = getWhiteSum(mask, bounding_box.get(0)) / 1000000;
            center_white = getWhiteSum(mask, bounding_box.get(1)) / 1000000;
        }

        setPropLocation(left_white, center_white);

        Bitmap b = Bitmap.createBitmap(final_mat.width(), final_mat.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(final_mat, b);
        last_frame.set(b);

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (DEBUG) {
            Bitmap bitmap = Bitmap.createBitmap(final_mat.width(), final_mat.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(final_mat, bitmap);
            android.graphics.Rect ROI = new android.graphics.Rect(0, 0, onscreenWidth, onscreenHeight);
            canvas.drawBitmap(bitmap, null, ROI, null);

            Paint rectPaint = new Paint();
            rectPaint.setColor(Color.RED);
            rectPaint.setStyle(Paint.Style.STROKE);
            rectPaint.setStrokeWidth(scaleCanvasDensity * 4);
            for (Rect rect : bounding_box) {
                canvas.drawRect(makeGraphicsRect(rect, scaleBmpPxToCanvasPx), rectPaint);
                canvas.drawText("box", rect.x, rect.y, new Paint(Color.GREEN));
            }
        }
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);
        return new android.graphics.Rect(left, top, right, bottom);
     }


    public Mat filterColor(Mat src) {
        Mat HSV = new Mat();
        Imgproc.cvtColor(src, HSV, Imgproc.COLOR_BGR2HSV);      //NOTE: BGR2HSV not RGB2HSV
        Scalar lower = new Scalar(filter_range[0], filter_range[1], filter_range[2]);
        Scalar upper = new Scalar(filter_range[3], filter_range[4], filter_range[5]);
        Core.inRange(HSV, lower, upper, HSV);
        return HSV;
    }

    private void processContours(Mat src) {
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(src, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        bounding_box.clear();

        for (MatOfPoint contour : contours) {
            if (Imgproc.contourArea(contour) > 1500) {
                bounding_box.add(Imgproc.boundingRect(contour));
            }
        }
    }

    private double getWhiteSum(Mat src, Rect box) {
        Mat submat;
        submat = src.submat(box);
        Scalar color =  Core.sumElems(submat);
        return color.val[0];
    }

    private void setPropLocation(double left, double center) {
        if (left > threshold) {
            location = PropLocation.LEFT;
        } else if (center > threshold) {
            location = PropLocation.CENTER;
        } else {
            location = PropLocation.RIGHT;
        }
    }

    public PropLocation getPropLocation () { return location; }

    public int getNumberOfDetection() {
        return bounding_box.size();
    }

}
