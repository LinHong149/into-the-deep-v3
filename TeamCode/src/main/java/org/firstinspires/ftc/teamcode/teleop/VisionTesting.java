package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;
import java.util.ArrayList;
import java.util.Collections;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class VisionTesting extends LinearOpMode {
    public double angle = 0;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    OpenCvCamera webcam = null;
    private Servo rotation;
    public double getDetectedAngle(){
        return angle;
    }
    public void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        webcam.setPipeline(new PipeLine());
        rotation = hardwareMap.get(Servo.class, "rotation");
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    @Override
    public void runOpMode() {
        initCamera();
        waitForStart();
        while (!isStopRequested()) {
            dashboardTelemetry.addData("status", "running");
            dashboardTelemetry.update();
        }
    }

    public class PipeLine extends OpenCvPipeline {
        Mat hsv = new Mat();
        Mat mask = new Mat();
        Mat hierarchy = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        public boolean scanned = false;
        @Override
        public Mat processFrame(Mat input) {
            if (scanned){
                return input;
            }
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Scalar lowerRed1 = new Scalar(0, 120, 50);
            Scalar upperRed1 = new Scalar(10, 255, 255);

            Scalar lowerRed2 = new Scalar(170, 120, 50);
            Scalar upperRed2 = new Scalar(180, 255, 255);

            Mat mask1 = new Mat();
            Mat mask2 = new Mat();
            Core.inRange(hsv, lowerRed1, upperRed1, mask1);
            Core.inRange(hsv, lowerRed2, upperRed2, mask2);

            Core.addWeighted(mask1, 1.0, mask2, 1.0, 0.0, mask);
            //yellow hsv
            //Scalar lowerYellow = new Scalar(20, 100, 100);  // Lower bound (H, S, V)
            //    Scalar upperYellow = new Scalar(35, 255, 255);  // Upper bound (H, S, V)
            //
            //    // Threshold the HSV image to get yellow colors
            //    Core.inRange(hsv, lowerYellow, upperYellow, mask);

            contours.clear();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                if (Imgproc.contourArea(contour) < 1000) { //sieve
                    continue;
                }

                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                MatOfPoint2f approxCurve = new MatOfPoint2f();
                double epsilon = 0.05 * Imgproc.arcLength(contour2f, true);
                Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);

                if (approxCurve.total() == 4) {
                    MatOfPoint points = new MatOfPoint(approxCurve.toArray());

                    if (!isRectangle(approxCurve.toArray())){
                        continue;
                    }
                    Imgproc.drawContours(input, Collections.singletonList(points), -1, new Scalar(0, 255, 0), 2);

                    double orientation = calculateOrientation(approxCurve.toArray());
                    angle = (orientation)/180;
                    rotation.setPosition(angle);
                    scanned = true;
                    dashboardTelemetry.addData("angle", orientation);
                }
            }
            dashboardTelemetry.addData("scanned",scanned);
            dashboardTelemetry.update();

            return input;
        }

        private double calculateOrientation(Point[] points) {
            double[] distances = new double[4];
            distances[0] = Math.hypot(points[1].x - points[0].x, points[1].y - points[0].y);
            distances[1] = Math.hypot(points[2].x - points[1].x, points[2].y - points[1].y);
            distances[2] = Math.hypot(points[3].x - points[2].x, points[3].y - points[2].y);
            distances[3] = Math.hypot(points[0].x - points[3].x, points[0].y - points[3].y);

            int l1 = 0;
            int l2 = 1;

            for (int i = 1; i < distances.length; i++) {
                if (distances[i] > distances[l1]) {
                    l2 = l1;
                    l1 = i;
                } else if (distances[i] > distances[l2]) {
                    l2 = i;
                }
            }

            Point start, end;
            if (l1 == 0 || l2 == 2) {
                start = points[0];
                end = points[1];
            } else {
                start = points[1];
                end = points[2];
            }

            double dx = end.x - start.x;
            double dy = end.y - start.y;

            double orientation = Math.atan2(dy, dx) * (180.0 / Math.PI);

            if (orientation < 0) {
                orientation += 360;
            }

            return orientation;
        }

        private boolean isRectangle(Point[] points) {
            double[] angles = new double[4];

            for (int i = 0; i < 4; i++) {
                Point p1 = points[i];
                Point p2 = points[(i + 1) % 4];
                Point p3 = points[(i + 2) % 4];

                double dx1 = p2.x - p1.x;
                double dy1 = p2.y - p1.y;
                double dx2 = p3.x - p2.x;
                double dy2 = p3.y - p2.y;

                double dotProduct = (dx1 * dx2) + (dy1 * dy2);
                double magnitude1 = Math.sqrt(dx1 * dx1 + dy1 * dy1);
                double magnitude2 = Math.sqrt(dx2 * dx2 + dy2 * dy2);

                double angle = Math.acos(dotProduct / (magnitude1 * magnitude2)) * (180.0 / Math.PI);
                angles[i] = angle;
            }

            for (double angle : angles) {
                if (angle < 80 || angle > 100) {
                    return false;
                }
            }
            return true;
        }


    }
}
