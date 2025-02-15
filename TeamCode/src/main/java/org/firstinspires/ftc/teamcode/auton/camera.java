package org.firstinspires.ftc.teamcode.auton;

// RR-specific imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.VisionTesting;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Config
@Autonomous
public class camera extends LinearOpMode{

    public double wristPar = 0.1, wristPerp = 0.62, wristOuttake = 0.82;
    public double clawOpen = 0.27, clawClose = 0.73;
    public double rotationPos = 0.46, rotationSpecial = 0.75;
    public double armDown = 5;
    public double armPar = 100, armUp = 1300, armHang = 700;
    public double slideRest = 300, slideIntaking = 1000;

    //  ARM PID
    PIDFController armPIDF = new PIDFController(0,0,0, 0);
    double armP = 0.008, armI = 0, armD = 0.0009, armF = 0;
    double armTarget = 0.0;

    //  SLIDES PID
    PIDFController slidePIDF = new PIDFController(0,0,0, 0);
    double slideP = 0.005, slideI = 0, slideD = 0.00018, slideF = 0;
    double slidePE = 0.008, slideIE = 0, slideDE = 0.00018, slideFE = 0;
    double slideTarget = 0.0;
    double slidePower = 0.0;
    double angle = 0;

    public enum Mode {
        INTAKING,
        REST,
        OUTTAKING,
        HANG
    }
    public Mode mode = Mode.INTAKING;
    /** Arm and slide*/
    public class ArmSlide {
        private DcMotorEx S1Motor, S2Motor, AMotor;

        public ArmSlide(HardwareMap hardwareMap) {
            S1Motor = hardwareMap.get(DcMotorEx.class, "S1Motor");
            S2Motor = hardwareMap.get(DcMotorEx.class, "S2Motor");
            AMotor = hardwareMap.get(DcMotorEx.class, "AMotor");

            AMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            AMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            AMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            AMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            AMotor.setPower(0);

            S1Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            S1Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            S1Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            S1Motor.setDirection(DcMotorSimple.Direction.REVERSE);
            S1Motor.setPower(0);

            S2Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            S2Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            S2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            S2Motor.setDirection(DcMotorSimple.Direction.REVERSE);
            S2Motor.setPower(0);
        }

        /**
         * To rest
         */
        public class ToRest implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                mode = Mode.REST;
                slideTarget = slideRest;

//                PID
                AMotor.setPower(armPIDF(armTarget, AMotor));
                slidePower = slidePIDF(slideTarget, S1Motor, S2Motor);
                S1Motor.setPower(slidePower);
                S2Motor.setPower(slidePower);

                packet.put("arm", AMotor.getCurrentPosition());
                packet.put("slide", S1Motor.getCurrentPosition());

//                Arm down after slide comes down
                if (Math.abs(S1Motor.getCurrentPosition() - slideTarget) < 600) {
                    armTarget = armPar;
                }

//                Keep repeating until arm Motor gets to target
                if (Math.abs(AMotor.getCurrentPosition() - armTarget) < 50 && armTarget == armPar) {
                    S1Motor.setPower(0);
                    S2Motor.setPower(0);
                    AMotor.setPower(0);
                    return false;
                } else {
                    return true;
                }
            }
        }

        public Action toRest() {
            return new ArmSlide.ToRest();
        }


        /**
         * To intaking
         */
        public class RestToIntaking implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                mode = Mode.INTAKING;
                slideTarget = slideIntaking;
                armTarget = armPar;

//                PID
                AMotor.setPower(armPIDF(armTarget, AMotor));
                slidePower = slidePIDF(slideTarget, S1Motor, S2Motor);
                S1Motor.setPower(slidePower);
                S2Motor.setPower(slidePower);

                packet.put("arm curr", AMotor.getCurrentPosition());
                packet.put("slide curr", S1Motor.getCurrentPosition());
                packet.put("arm tar", armTarget);
                packet.put("slide tar", slideTarget);


//                Keep repeating until arm Motor gets to target
                if (Math.abs(S1Motor.getCurrentPosition() - slideTarget) < 100 && Math.abs(AMotor.getCurrentPosition() - armTarget) < 20) {
                    S1Motor.setPower(0);
                    S2Motor.setPower(0);
                    AMotor.setPower(0);
                    return false;
                } else {
                    return true;
                }
            }
        }

        public Action restToIntaking() {
            return new ArmSlide.RestToIntaking();
        }

        public class IntakingDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                mode = Mode.INTAKING;
                armTarget = armDown;

                AMotor.setPower(armPIDF(armTarget, AMotor));
                if (Math.abs(AMotor.getCurrentPosition() - armTarget) < 20) {
                    return false;
                } else {
                    return true;
                }

            }
        }

        public Action intakingDown() {
            return new ArmSlide.IntakingDown();
        }
    }


    /**
     * Intaking system class
     */
    OpenCvCamera webcam = null;
    PipeLine pipeLine = new PipeLine();
    public void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        webcam.setPipeline(pipeLine);
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
    public class IntakingSystem {
        private Servo claw, wrist, rotation;


        public IntakingSystem(HardwareMap hardwareMap) {
            rotation = hardwareMap.get(Servo.class, "rotation");
            wrist = hardwareMap.get(Servo.class, "wrist");
            claw = hardwareMap.get(Servo.class, "claw");
        }


        /**
         * Open claw intaking
         */
        public class IntakingOpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotation.setPosition(rotationPos);
                wrist.setPosition(wristPar);
                claw.setPosition(clawOpen);

                packet.put("intaking", "intaking open claw");
                return false;
            }
        }

        public Action intakingOpenClaw() {
            return new IntakingSystem.IntakingOpenClaw();
        }


        /**
         * Close claw intaking
         */
        public class IntakingCloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotation.setPosition(rotationPos);
                wrist.setPosition(wristPar);
                claw.setPosition(clawClose);

                packet.put("intaking", "intaking close claw");
                return false;
            }
        }

        public Action intakingCloseClaw() {
            return new IntakingSystem.IntakingCloseClaw();
        }

        /**
         * Close claw rest
         */
        public class RestCloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotation.setPosition(rotationPos);
                wrist.setPosition(wristPerp);
                claw.setPosition(clawClose);

                packet.put("intaking", "intaking close claw");
                return false;
            }
        }

        public Action restCloseClaw() {
            return new IntakingSystem.RestCloseClaw();
        }

        public class RestOpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotation.setPosition(rotationPos);
                wrist.setPosition(wristPerp);
                claw.setPosition(clawOpen);
                return false;
            }
        }

        public Action restOpenClaw() {
            return new IntakingSystem.RestOpenClaw();
        }

        public class VisionRotate implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotation.setPosition(angle);
                return false;
            }
        }
        public Action visionRotate() { return new IntakingSystem.VisionRotate();}

    }



    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-38, -61, Math.toRadians(90));
        ArmSlide armSlide = new ArmSlide(hardwareMap);
        IntakingSystem intakingSystem = new IntakingSystem(hardwareMap);
        initCamera();


        waitForStart();

//        Code Running
        if (isStopRequested()) return;

        if (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                        armSlide.toRest(),
                        new SleepAction(1),
                        armSlide.restToIntaking(),
                        intakingSystem.intakingOpenClaw(),
                        new SleepAction(1),
                        new Action(){
                            @Override
                            public boolean run(@NonNull TelemetryPacket packet){
                                pipeLine.scanned = false;
                                return false;
                            }
                        },
                        intakingSystem.visionRotate(),
                        new SleepAction(1),
                        armSlide.intakingDown(),
                        new SleepAction(1),
                        intakingSystem.intakingCloseClaw(),
                        new SleepAction(1),
                        intakingSystem.restCloseClaw(),
                        armSlide.toRest()




                    )
            );

        }



    }


    public double armPIDF(double target, DcMotorEx motor){
        armPIDF.setPIDF(armP,armI,armD,armF);
        int currentPosition = motor.getCurrentPosition();
        double output = armPIDF.calculate(currentPosition, target);

        return output;
    }

    public double slidePIDF(double target, DcMotorEx motor,DcMotorEx motor2){
        if (mode == Mode.OUTTAKING){
            slidePIDF.setPIDF(slidePE,slideIE,slideDE,slideFE);
        }else {
            slidePIDF.setPIDF(slideP, slideI, slideD, slideF);
        }
        int currentPosition = (motor.getCurrentPosition()+motor2.getCurrentPosition())/2;
        double output = slidePIDF.calculate(currentPosition, target);


        return output;
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
                    scanned = true;
                    telemetry.addData("angle", orientation);
                }
            }
            telemetry.addData("scanned",scanned);
            telemetry.update();

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