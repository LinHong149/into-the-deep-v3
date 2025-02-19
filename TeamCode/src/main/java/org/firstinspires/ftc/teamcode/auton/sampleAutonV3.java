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

@Config
@Autonomous
public class sampleAutonV3 extends LinearOpMode{

    public double wristPar = 0.1, wristPerp = 0.62, wristOuttake = 0.82;
    public double clawOpen = 0.27, clawClose = 0.74;
    public double rotationPos = 0.46, rotationSpecial = 0.75;
    public double armDown = 5;
    public double armPar = 125, armUp = 1300, armHang = 700;
    public double slideRest = 300, slideIntaking = 1000, slideOuttaking = 3000, slideHang = 800;
    public double outToRestBuffer = 800, restToOuttake = 1000;
    public double slideInput=0,rotationInput =0.5;
    public double slideInput1 = 0,rotationInput1 = 0.5;

    public boolean inputDone = false;
    public boolean yPrev = false, bPrev = false, xPrev = false, aPrev = false;
    public boolean rightPrev = false, leftPrev = false,upPrev = false,downPrev = false;
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

        public ArmSlide (HardwareMap hardwareMap) {
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

        /** To rest */
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



        /** To intaking */
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

                AMotor.setPower(armPIDF(armTarget,AMotor));
                if (Math.abs(AMotor.getCurrentPosition()-armTarget)<20){
                    return false;
                }else{
                    return true;
                }

            }
        }
        public Action intakingDown(){
            return new ArmSlide.IntakingDown();
        }

        public class IntakingSub implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                mode = Mode.INTAKING;
                armTarget = armPar;
                slideTarget = slideInput;

                AMotor.setPower(armPIDF(armTarget,AMotor));
                slidePower = slidePIDF(slideTarget,S1Motor,S2Motor);
                S1Motor.setPower(slidePower);
                S2Motor.setPower(slidePower);
                packet.put("action","intakingSub");
                if (Math.abs(armTarget-AMotor.getCurrentPosition())<50 && Math.abs(S1Motor.getCurrentPosition() - slideTarget)<200){
                    AMotor.setPower(0);
                    S1Motor.setPower(0);
                    S2Motor.setPower(0);
                    return false;
                }else{
                    return true;
                }
            }
        }
        public Action intakingSub(){ return new ArmSlide.IntakingSub();}

        public class IntakingSub1 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                mode = Mode.INTAKING;
                armTarget = armPar;
                slideTarget = slideInput1;

                AMotor.setPower(armPIDF(armTarget,AMotor));
                slidePower = slidePIDF(slideTarget,S1Motor,S2Motor);
                S1Motor.setPower(slidePower);
                S2Motor.setPower(slidePower);
                packet.put("action","intakingSub");
                if (Math.abs(armTarget-AMotor.getCurrentPosition())<50 && Math.abs(S1Motor.getCurrentPosition() - slideTarget)<200){
                    AMotor.setPower(0);
                    S1Motor.setPower(0);
                    S2Motor.setPower(0);
                    return false;
                }else{
                    return true;
                }
            }
        }
        public Action intakingSub1(){ return new ArmSlide.IntakingSub1();}


        /** To outtaking */
        public class RestToOuttaking implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                mode = Mode.OUTTAKING;
                armTarget = armUp;

//                PID
                AMotor.setPower(armPIDF(armTarget, AMotor));
                slidePower = slidePIDF(slideTarget, S1Motor, S2Motor);
                S1Motor.setPower(slidePower);
                S2Motor.setPower(slidePower);

                packet.put("arm", AMotor.getCurrentPosition());
                packet.put("slide", S1Motor.getCurrentPosition());


//                Arm down after slide comes down
                if (Math.abs(AMotor.getCurrentPosition() - armTarget) < 400) {
                    slideTarget = slideOuttaking;
                }

//                Keep repeating until arm Motor gets to target
                if (Math.abs(S1Motor.getCurrentPosition() - slideTarget) < 200 && slideTarget == slideOuttaking) {
                    S1Motor.setPower(0);
                    S2Motor.setPower(0);
                    AMotor.setPower(0);
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action restToOuttaking() {
            return new ArmSlide.RestToOuttaking();
        }


        /** To hang */
        public class RestToHang implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                mode = Mode.HANG;
                armTarget = armHang;
                slideTarget = slideHang;

//                PID
                AMotor.setPower(armPIDF(armTarget, AMotor));
                slidePower = slidePIDF(slideTarget, S1Motor, S2Motor);
                S1Motor.setPower(slidePower);
                S2Motor.setPower(slidePower);

                packet.put("arm", AMotor.getCurrentPosition());
                packet.put("slide", S1Motor.getCurrentPosition());


//                Keep repeating until arm Motor gets to target
                if (Math.abs(S1Motor.getCurrentPosition() - slideTarget) < 200 && Math.abs(AMotor.getCurrentPosition()-armTarget)<50) {
                    S1Motor.setPower(0);
                    S2Motor.setPower(0);
                    AMotor.setPower(0);
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action restToHang() {
            return new ArmSlide.RestToHang();
        }
    }


    /** Intaking system class */
    public class IntakingSystem {
        private Servo claw, wrist, rotation;


        public IntakingSystem(HardwareMap hardwareMap) {
            rotation = hardwareMap.get(Servo.class, "rotation");
            wrist = hardwareMap.get(Servo.class, "wrist");
            claw = hardwareMap.get(Servo.class, "claw");
        }


        /** Open claw intaking */
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

        public class IntakingOpenSpecial implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packer) {
                rotation.setPosition(rotationSpecial);
                wrist.setPosition(wristPar);
                claw.setPosition(clawOpen);

                return false;
            }
        }
        public Action intakingOpenSpecial (){
            return new IntakingSystem.IntakingOpenSpecial();
        }

        public class IntakingCloseSpecial implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotation.setPosition(rotationSpecial);
                wrist.setPosition(wristPar);
                claw.setPosition(clawClose);

                return false;
            }
        }
        public Action intakingCloseSpecial (){
            return new IntakingSystem.IntakingCloseSpecial();
        }

        /** Close claw intaking */
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

        /** Close claw rest */
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

        /** Open claw outtaking */
        public class OuttakingOpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotation.setPosition(rotationPos);
                wrist.setPosition(wristOuttake);
                claw.setPosition(clawOpen);

                packet.put("outtaking", "outtaking open claw");
                return false;
            }
        }
        public Action outtakingOpenClaw() {
            return new IntakingSystem.OuttakingOpenClaw();
        }

        public class IntakingOpenSub implements Action{
            @Override
            public boolean run (@NonNull TelemetryPacket packet){
                rotation.setPosition(rotationInput);
                wrist.setPosition(wristPar);
                claw.setPosition(clawOpen);
                packet.put("action","intakingOpenSub");
                return false;
            }
        }
        public Action intakingOpenSub() { return new IntakingSystem.IntakingOpenSub();}

        public class IntakingCloseSub implements Action{
            @Override
            public boolean run (@NonNull TelemetryPacket packet){
                rotation.setPosition(rotationInput);
                wrist.setPosition(wristPar);
                claw.setPosition(clawClose);
                packet.put("action","intakingCloseSub");
                return false;
            }
        }
        public Action intakingCloseSub(){return new IntakingSystem.IntakingCloseSub();
        }

        public class IntakingOpenSub1 implements Action{
            @Override
            public boolean run (@NonNull TelemetryPacket packet){
                rotation.setPosition(rotationInput1);
                wrist.setPosition(wristPar);
                claw.setPosition(clawOpen);
                packet.put("action","intakingOpenSub");
                return false;
            }
        }
        public Action intakingOpenSub1() { return new IntakingSystem.IntakingOpenSub1();}

        public class IntakingCloseSub1 implements Action{
            @Override
            public boolean run (@NonNull TelemetryPacket packet){
                rotation.setPosition(rotationInput1);
                wrist.setPosition(wristPar);
                claw.setPosition(clawClose);
                packet.put("action","intakingOpenSub");
                return false;
            }
        }
        public Action intakingCloseSub1() { return new IntakingSystem.IntakingCloseSub1();}

        public class OuttakingCloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                rotation.setPosition(rotationPos);
                wrist.setPosition(wristOuttake);
                claw.setPosition(clawClose);

                packet.put("outtaking","outtaking close claw");
                return false;
            }
        }
        public Action outtakingCloseClaw(){
            return new IntakingSystem.OuttakingCloseClaw();
        }

        public class Hang implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                rotation.setPosition(rotationPos);
                wrist.setPosition(.15);
                claw.setPosition(clawClose);

                packet.put("hang","hang");
                return false;
            }
        }
        public Action hang(){
            return new IntakingSystem.Hang();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-38, -61, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ArmSlide armSlide = new ArmSlide(hardwareMap);
        IntakingSystem intakingSystem = new IntakingSystem(hardwareMap);



        TrajectoryActionBuilder dropOffPreload = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(-55, -54, Math.toRadians(45)), Math.toRadians(261))
                .waitSeconds(.2);
        TrajectoryActionBuilder toFirstSample = drive.actionBuilder(new Pose2d(-55,-54,Math.toRadians(45)))
                .splineTo(new Vector2d(-48.1,-38.5),Math.toRadians(90))
                .waitSeconds(.2);
        TrajectoryActionBuilder dropOffFirst = drive.actionBuilder(new Pose2d(-48.1,-38.5,Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(-53,-54),Math.toRadians(229.5))
                .waitSeconds(.2);
        TrajectoryActionBuilder toSecondSample = drive.actionBuilder(new Pose2d(-53,-54,Math.toRadians(49.5)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-61,-40.5,Math.toRadians(99)),Math.toRadians(107))
                .waitSeconds(.2);
        TrajectoryActionBuilder dropOffSecond = drive.actionBuilder(new Pose2d(-61,-40.5,Math.toRadians(99)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-57.5,-50.5,Math.toRadians(48)),Math.toRadians(232))
                .waitSeconds(.2);
        TrajectoryActionBuilder toThirdSample = drive.actionBuilder(new Pose2d(-57.5,-50.5,Math.toRadians(48)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-62,-40, Math.toRadians(135)), Math.toRadians(55))
                .waitSeconds(.2);
        TrajectoryActionBuilder dropOffThird = drive.actionBuilder(new Pose2d(-62,-40,Math.toRadians(135)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-57.5,-50,Math.toRadians(48)), Math.toRadians(230))
                .waitSeconds(.2);

        TrajectoryActionBuilder sub = drive.actionBuilder(new Pose2d(-57.5,-50,Math.toRadians(48)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-19, -2,Math.toRadians(0)),Math.toRadians(0))
                .waitSeconds(.2);
        TrajectoryActionBuilder dropOffSub = drive.actionBuilder(new Pose2d(-19,-2,Math.toRadians(0)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-59,-53,Math.toRadians(45)),Math.toRadians(225))
                .waitSeconds(.2);

        TrajectoryActionBuilder sub1 = drive.actionBuilder(new Pose2d(-59,-49,Math.toRadians(45)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-19,-5,Math.toRadians(0)),Math.toRadians(0))
                .waitSeconds(.2);
        TrajectoryActionBuilder dropOffSub1 = drive.actionBuilder(new Pose2d(-19,-5,Math.toRadians(0)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-59,-49,Math.toRadians(45)),Math.toRadians(45))
                .waitSeconds(.2);




//        Initialization
        while (!inputDone){
            if (gamepad1.right_bumper || isStopRequested()){
                inputDone = true;
            }

            boolean yCurr = gamepad1.y;
            boolean xCurr = gamepad1.x;
            boolean bCurr = gamepad1.b;
            boolean aCurr = gamepad1.a;
            boolean rightCurr = gamepad1.dpad_right;
            boolean leftCurr = gamepad1.dpad_left;
            boolean upCurr = gamepad1.dpad_up;
            boolean downCurr = gamepad1.dpad_down;

            if (yCurr && ! yPrev){
                slideInput += 100;
            }
            yPrev = yCurr;

            if (xCurr && !xPrev){
                rotationInput-=0.05;
            }
            xPrev = xCurr;

            if (bCurr && !bPrev){
                rotationInput += 0.05;
            }
            bPrev = bCurr;

            if (aCurr && !aPrev){
                slideInput -=100;
            }
            aPrev = aCurr;

            if (upCurr && !upPrev){
                slideInput1 += 100;
            }
            upPrev = upCurr;

            if (leftCurr && !leftPrev){
                rotationInput1 -= 0.05;
            }
            leftPrev = leftCurr;

            if (rightCurr && !rightPrev){
                rotationInput1 += 0.05;
            }
            rightPrev = rightCurr;

            if (downCurr && !downPrev){
                slideInput1 -= 100;
            }
            downPrev = downCurr;

            slideInput = Math.min(2000,Math.max(300,slideInput));
            slideInput1 = Math.min(2000,Math.max(300,slideInput1));
            rotationInput = Math.min(1,Math.max(0,rotationInput));
            rotationInput1 = Math.min(1,Math.max(0,rotationInput1));


            telemetry.addData("slideInput",slideInput);
            telemetry.addData("rotationInput",rotationInput);
            telemetry.addData("slideInput1",slideInput1);
            telemetry.addData("rotationInput1",rotationInput1);
            telemetry.update();



        }
        Actions.runBlocking(
                new SequentialAction(
                        intakingSystem.restCloseClaw(),
                        armSlide.toRest(),
                        new SleepAction(.2),
                        intakingSystem.intakingOpenClaw(),
                        new SleepAction(.2),
                        armSlide.intakingDown(),
                        new SleepAction(.2),
                        intakingSystem.intakingCloseClaw(),
                        new SleepAction(.2),
                        intakingSystem.restCloseClaw(),
                        armSlide.toRest()


                )
        );

        waitForStart();

//        Code Running
        if (isStopRequested()) return;

        if (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    intakingSystem.intakingCloseClaw(),
                                    armSlide.restToOuttaking(),
                                    dropOffPreload.build()
                            ),
                            intakingSystem.outtakingCloseClaw(),
                            new SleepAction(.3),
                            intakingSystem.outtakingOpenClaw(),
                            new SleepAction(.2),
                            intakingSystem.intakingOpenClaw(),
                            new ParallelAction(
                                    armSlide.toRest(),
                                    intakingSystem.intakingOpenClaw(),
                                    toFirstSample.build()
                            ),
                            new SleepAction(.1),
                            armSlide.intakingDown(),
                            new SleepAction(.2),
                            intakingSystem.intakingCloseClaw(),
                            new SleepAction(.2),
                            intakingSystem.restCloseClaw(),
                            armSlide.toRest(),
                            new ParallelAction(
                                    intakingSystem.intakingCloseClaw(),
                                    armSlide.restToOuttaking(),
                                    dropOffFirst.build()
                            ),
                            intakingSystem.outtakingCloseClaw(),
                            new SleepAction(.3),
                            intakingSystem.outtakingOpenClaw(),
                            new SleepAction(.2),
                            intakingSystem.intakingOpenClaw(),
                            new ParallelAction(
                                    armSlide.toRest(),
                                    intakingSystem.intakingOpenClaw(),
                                    toSecondSample.build()
                            ),
                            new SleepAction(.1),
                            armSlide.intakingDown(),
                            new SleepAction(.2),
                            intakingSystem.intakingCloseClaw(),
                            new SleepAction(.2),
                            intakingSystem.restCloseClaw(),
                            armSlide.toRest(),
                            new ParallelAction(
                                    intakingSystem.intakingCloseClaw(),
                                    armSlide.restToOuttaking(),
                                    dropOffSecond.build()
                            ),
                            intakingSystem.outtakingCloseClaw(),
                            new SleepAction(.3),
                            intakingSystem.outtakingOpenClaw(),
                            new SleepAction(.2),
                            intakingSystem.intakingOpenClaw(),
                            armSlide.toRest(),
                            new ParallelAction(
                                    intakingSystem.intakingOpenSpecial(),
                                    toThirdSample.build()
                            ),
                            new SleepAction(.1),
                            armSlide.intakingDown(),
                            new SleepAction(.2),
                            intakingSystem.intakingCloseSpecial(),
                            new SleepAction(.2),
                            intakingSystem.restCloseClaw(),
                            armSlide.toRest(),
                            new ParallelAction(
                                    intakingSystem.intakingCloseClaw(),
                                    armSlide.restToOuttaking(),
                                    dropOffThird.build()
                            ),
                            intakingSystem.outtakingCloseClaw(),
                            new SleepAction(.3),
                            intakingSystem.outtakingOpenClaw(),
                            new SleepAction(.2),
                            intakingSystem.intakingOpenClaw(),

                            new ParallelAction(
                                    armSlide.toRest(),
                                    intakingSystem.intakingOpenClaw(),
                                    sub.build()
                            ),

                            new SleepAction(.1),
                            armSlide.intakingSub(),
                            intakingSystem.intakingOpenSub(),
                            new SleepAction(.2),
                            armSlide.intakingDown(),
                            new SleepAction(.2),
                            intakingSystem.intakingCloseSub(),
                            new SleepAction(.1),
                            intakingSystem.restCloseClaw(),
                            armSlide.toRest(),
                            new ParallelAction(
                                    intakingSystem.intakingCloseClaw(),
                                    armSlide.restToOuttaking(),
                                    dropOffSub.build()
                            ),
                            intakingSystem.outtakingCloseClaw(),
                            new SleepAction(.2),
                            intakingSystem.outtakingOpenClaw(),
                            new SleepAction(.2),
                            intakingSystem.intakingOpenClaw(),
                            new ParallelAction(
                                    armSlide.toRest(),
                                    sub.build()
                            ),


//
//                            new ParallelAction(
//                                    armSlide.toRest(),
//                                    sub1.build()
//                            ),
//                            new SleepAction(.1),
//                            armSlide.intakingSub1(),
//                            intakingSystem.intakingOpenSub1(),
//                            new SleepAction(.2),
//                            armSlide.intakingDown(),
//                            new SleepAction(.2),
//                            intakingSystem.intakingCloseSub1(),
//                            new SleepAction(.1),
//                            intakingSystem.restCloseClaw(),
//                            armSlide.toRest(),
//                            new ParallelAction(
//                                    intakingSystem.intakingCloseClaw(),
//                                    armSlide.restToOuttaking(),
//                                    dropOffSub1.build()
//                            ),
//                            intakingSystem.outtakingCloseClaw(),
//                            new SleepAction(.2),
//                            intakingSystem.outtakingOpenClaw(),
//                            new SleepAction(.2),
//                            intakingSystem.intakingOpenClaw(),
//                            new ParallelAction(
//                                    armSlide.toRest(),
//                                    sub1.build()
//                            ),



                            armSlide.restToIntaking(),
                            new SleepAction(4)

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
}