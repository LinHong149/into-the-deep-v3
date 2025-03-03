package org.firstinspires.ftc.teamcode.auton;

// RR-specific imports
import androidx.annotation.NonNull;

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

import org.firstinspires.ftc.teamcode.MecanumDrive;


// LL imports
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Config
@Autonomous
public class limelightrr extends LinearOpMode{

    public double wristPar = 0.1, wristPerp = 0.62, wristOuttake = 0.82;
    public double clawOpen = 0.27, clawClose = 0.74;
    public double rotationPos = 0.46, rotationSpecial = 0.75;
    public double armDown = -10;
    public double armPar = 80, armUp = 900, armHang = 500;
    public double slideRest = 300, slideIntaking = 1000, slideOuttaking = 3000, slideHang = 800;
    public double outToRestBuffer = 800, restToOuttake = 1000;
    public double slideInput=0,rotationInput =0.5;
    public double slideInput1 = 0,rotationInput1 = 0.5;

    public boolean inputDone = false;
    public boolean yPrev = false, bPrev = false, xPrev = false, aPrev = false;
    public boolean rightPrev = false, leftPrev = false,upPrev = false,downPrev = false;


    public class IntakingSystem {
        private Servo rotation;
        private Limelight3A limelight;


        public IntakingSystem(HardwareMap hardwareMap) {
            rotation = hardwareMap.get(Servo.class, "rotation");


            // LL
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            telemetry.setMsTransmissionInterval(11);
            limelight.pipelineSwitch(0);
            limelight.start();

        }


        /** Open claw intaking */
        public class LimelightTest implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                packet.put("Status", "Running");
                telemetry.addData("Status", "Running on Driver Hub");
                rotation.setPosition(rotationPos);

                LLStatus status = limelight.getStatus();
                packet.put("Name", status.getName());
                packet.put("Pipeline Index", status.getPipelineIndex());
                packet.put("PipelineType", status.getPipelineType());
                telemetry.addData("Name", "%s",
                        status.getName());
                telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                        status.getTemp(), status.getCpu(),(int)status.getFps());
                telemetry.addData("Pipeline", "Index: %d, Type: %s",
                        status.getPipelineIndex(), status.getPipelineType());

                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    // Access general information
                    Pose3D botpose = result.getBotpose();
                    double captureLatency = result.getCaptureLatency();
                    double targetingLatency = result.getTargetingLatency();
                    double parseLatency = result.getParseLatency();
                    packet.put("LL Latency", captureLatency + targetingLatency);
                    packet.put("Parse Latency", parseLatency);
                    packet.put("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
                    telemetry.addData("LL Latency", captureLatency + targetingLatency);
                    telemetry.addData("Parse Latency", parseLatency);
                    telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));


                    if (result.isValid()) {
                        packet.put("tx", result.getTx());
                        packet.put("txnc", result.getTxNC());
                        packet.put("ty", result.getTy());
                        packet.put("tync", result.getTyNC());
                        packet.put("Botpose", botpose.toString());
                        telemetry.addData("tx", result.getTx());
                        telemetry.addData("txnc", result.getTxNC());
                        telemetry.addData("ty", result.getTy());
                        telemetry.addData("tync", result.getTyNC());
                        telemetry.addData("Botpose", botpose.toString());

                        // Access color results
                        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                        for (LLResultTypes.ColorResult cr : colorResults) {
                            packet.put("TargetXDegrees", cr.getTargetXDegrees());
                            packet.put("TargetYDegrees", cr.getTargetYDegrees());
                            telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                        }
                    }
                } else {
                    packet.put("Limelight", "No data available");
                    telemetry.addData("Limelight", "No data available");
                }
                telemetry.update();
                return true;
            }
        }
        public Action limelightTest() {
            return new LimelightTest();
        }

        /** Close claw intaking */
        public class IntakingCloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotation.setPosition(rotationPos);

                packet.put("intaking", "intaking close claw");
                return false;
            }
        }
        public Action intakingCloseClaw() {
            return new IntakingCloseClaw();
        }


    }


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-38, -61, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        IntakingSystem intakingSystem = new IntakingSystem(hardwareMap);

        TrajectoryActionBuilder dropOffPreload = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(-54, -53, Math.toRadians(45)), Math.toRadians(261))
                .waitSeconds(.2);



        waitForStart();

//        Code Running
        if (isStopRequested()) return;

        if (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            intakingSystem.limelightTest(),
                            dropOffPreload.build()


                    )
            );

        }



    }


}