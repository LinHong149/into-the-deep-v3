package org.firstinspires.ftc.teamcode.auton;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.sample;

@Config
@Autonomous
public class specimenAutonV2 extends LinearOpMode{

    public double wristPar = 0.1, wristPerp = 0.62, wristOuttake = 0.75;
    public double clawOpen = 0.25, clawClose = 0.77;
    public double rotationPos = 0.465;
    public double armDown = 50;
    public double armPar = 125, armOuttakingScore = 545, armBackwardsOuttaking = 1075;
    public double slideRest = 200, slideForwardsOuttaking = 1700, slideOuttakingAim = 720, slideOuttakingScore = 1600;

    //  ARM PID
    PIDFController armPIDF = new PIDFController(0,0,0, 0);
    double armP = 0.007, armI = 0, armD = 0, armF = 0;
    double armTarget = 0.0;

    //  SLIDES PID
    PIDFController slidePIDF = new PIDFController(0,0,0, 0);
    double slideP = 0.0034, slideI = 0, slideD = 0, slideF = 0;
    double slidePE = 0.008, slideIE = 0, slideDE = 0, slideFE = 0;
    double slideTarget = 0.0;
    double slidePower = 0.0;
    public double slideCurr = 0;
    public boolean scoreBackwards = false;

    public enum Mode {
        INTAKING,
        REST,
        AIM,
        SCORE
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
                if (mode == Mode.SCORE) {
                    armTarget = armOuttakingScore + 50;
                }
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
                if (Math.abs(S1Motor.getCurrentPosition() - slideTarget) < 50) {
                    armTarget = armPar;
                }

//                Keep repeating until arm Motor gets to target
                if (Math.abs(AMotor.getCurrentPosition() - armTarget) < 20 && armTarget == armPar) {
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
            return new ToRest();
        }



        /** To outtaking score */
        public class ToOuttakingForwardScore implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                mode = Mode.SCORE;
                armTarget = armOuttakingScore;
                slideTarget = slideForwardsOuttaking;

//                PID
                AMotor.setPower(armPIDF(armTarget, AMotor));
                slidePower = slidePIDF(slideTarget, S1Motor, S2Motor);
                S1Motor.setPower(slidePower);
                S2Motor.setPower(slidePower);

                packet.put("arm", AMotor.getCurrentPosition());
                packet.put("slide", S1Motor.getCurrentPosition());



//                Keep repeating until arm Motor gets to target
                if (Math.abs(AMotor.getCurrentPosition() - armTarget) < 50 && Math.abs(S1Motor.getCurrentPosition()-slideTarget) < 100) {
                    S1Motor.setPower(0);
                    S2Motor.setPower(0);
                    AMotor.setPower(0);
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action toOuttakingForwardScore() {
            return new ToOuttakingForwardScore();
        }





        /** To outtaking aim */
        public class ToOuttakingBackwardAim implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                scoreBackwards = true;
                armTarget = armBackwardsOuttaking;
                slideCurr = S1Motor.getCurrentPosition();

//                PID
                AMotor.setPower(armPIDF(armTarget, AMotor));
                slidePower = slidePIDF(slideTarget, S1Motor, S2Motor);
                S1Motor.setPower(slidePower);
                S2Motor.setPower(slidePower);

                packet.put("arm", AMotor.getCurrentPosition());
                packet.put("slide", S1Motor.getCurrentPosition());

//                Arm down after slide comes down
                if (Math.abs(AMotor.getCurrentPosition() - armTarget) < 400) {
                    slideTarget = slideOuttakingAim;
                }

//                Keep repeating until arm Motor gets to target
                if (Math.abs(S1Motor.getCurrentPosition() - slideTarget) < 150 && slideTarget == slideOuttakingAim) {
                    S1Motor.setPower(0);
                    S2Motor.setPower(0);
                    AMotor.setPower(0);
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action toOuttakingBackwardAim() {
            return new ToOuttakingBackwardAim();
        }



        /** To outtaking score */
        public class ToOuttakingBackwardScore implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armTarget = armBackwardsOuttaking;

//                PID
                AMotor.setPower(armPIDF(armTarget, AMotor));
                slidePower = slidePIDF(slideTarget, S1Motor, S2Motor);
                S1Motor.setPower(slidePower);
                S2Motor.setPower(slidePower);

                packet.put("arm", AMotor.getCurrentPosition());
                packet.put("slide", S1Motor.getCurrentPosition());

//                Arm down after slide comes down
                if (Math.abs(AMotor.getCurrentPosition() - armTarget) < 400) {
                    slideTarget = slideOuttakingScore;
                }

//                Keep repeating until arm Motor gets to target
                if (Math.abs(S1Motor.getCurrentPosition() - slideTarget) < 150 && slideTarget == slideOuttakingScore) {
                    S1Motor.setPower(0);
                    S2Motor.setPower(0);
                    AMotor.setPower(0);
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action toOuttakingBackwardScore() {
            return new ToOuttakingBackwardScore();
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


        /** Open claw  */
        public class PerpOpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotation.setPosition(rotationPos);
                wrist.setPosition(wristPerp);
                claw.setPosition(clawOpen);

                packet.put("intaking", "intaking open claw");
                return false;
            }
        }
        public Action perpOpenClaw() {
            return new PerpOpenClaw();
        }

        /** Close claw  */
        public class PerpCloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotation.setPosition(rotationPos);
                wrist.setPosition(wristPerp);
                claw.setPosition(clawClose);

                packet.put("intaking", "intaking close claw");
                return false;
            }
        }
        public Action perpCloseClaw() {
            return new PerpCloseClaw();
        }



        /** Open claw  */
        public class BackOpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotation.setPosition(rotationPos);
                wrist.setPosition(wristOuttake);
                claw.setPosition(clawOpen);

                packet.put("intaking", "intaking open claw");
                return false;
            }
        }
        public Action backOpenClaw() {
            return new BackOpenClaw();
        }

        /** Close claw  */
        public class BackCloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (slideTarget > 600) {
                    wrist.setPosition(wristOuttake);
                    return false;
                }
                rotation.setPosition(rotationPos);
                claw.setPosition(clawClose);

                packet.put("intaking", "intaking close claw");
                return true;
            }
        }
        public Action backCloseClaw() {
            return new BackCloseClaw();
        }

    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(8, -61, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        IntakingSystem intakingSystem = new IntakingSystem(hardwareMap);
        ArmSlide armSlide = new ArmSlide(hardwareMap);


        TrajectoryActionBuilder spin = drive.actionBuilder(initialPose)
                .turn(Math.toRadians(5));

        TrajectoryActionBuilder dropOffPreloaded = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(0, -31), Math.toRadians(90)); //drop off pre loaded

        TrajectoryActionBuilder pushSamples = drive.actionBuilder(new Pose2d(0, -34, Math.toRadians(90)))
                .setTangent(Math.toRadians(320))
                .splineToConstantHeading(new Vector2d(34, -28), Math.toRadians(90)) //path around submersible
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(44, -11), Math.toRadians(300)) //to first
                .splineToConstantHeading(new Vector2d(44, -55), Math.toRadians(90)) //push observation zone
                .splineToConstantHeading(new Vector2d(57, -11), Math.toRadians(300)) //to second
                .splineToConstantHeading(new Vector2d(46, -53), Math.toRadians(180)) //push observation zone
                .splineToSplineHeading(new Pose2d(36,-50,Math.toRadians(265)),Math.toRadians(270)) //turn
                .splineToConstantHeading(new Vector2d(36,-58.5),Math.toRadians(270)); //intaking

        TrajectoryActionBuilder dropOffFirst = drive.actionBuilder(new Pose2d(36, -58.5, Math.toRadians(260)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(0,-30,Math.toRadians(270)),Math.toRadians(90)); //drop off first
        TrajectoryActionBuilder intakingSecond = drive.actionBuilder(new Pose2d(0, -30, Math.toRadians(275)))
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(36,-56.5,Math.toRadians(270)),Math.toRadians(270)); //intaking
        TrajectoryActionBuilder dropOffSecond = drive.actionBuilder(new Pose2d(36, -56.5, Math.toRadians(270)))
                .setTangent(Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(-2,-26.5,Math.toRadians(270)),Math.toRadians(90)); //drop off second
        TrajectoryActionBuilder intakingThird = drive.actionBuilder(new Pose2d(-2, -26.5, Math.toRadians(270)))
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(36,-58,Math.toRadians(270)),Math.toRadians(270)); //intaking
        TrajectoryActionBuilder dropOffThird = drive.actionBuilder(new Pose2d(36, -58.5, Math.toRadians(270)))
                .setTangent(Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(-3,-33,Math.toRadians(270)),Math.toRadians(90)); //drop off third
        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(-3, -33, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(40,-59,Math.toRadians(270)),Math.toRadians(0)); //park





//        Initialization
        Actions.runBlocking(
                new SequentialAction(
                        armSlide.toRest(),
                        intakingSystem.perpCloseClaw()
                )
        );

        waitForStart();

//        Code Running
        if (isStopRequested()) return;

        if (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
//                            Drops preloaded and pushes samples
                            new ParallelAction(
                                armSlide.toOuttakingForwardScore(),
                                intakingSystem.backCloseClaw(),
                                dropOffPreloaded.build()
                            ),
//                        new SleepAction(0.2),
                            intakingSystem.perpOpenClaw(),
//                        new SleepAction(0.2),
                            new ParallelAction(
                                    armSlide.toRest(),
                                    pushSamples.build()
                            ),

//                        Grabs first sample and scores
                            new SleepAction(0.5),
                            intakingSystem.perpCloseClaw(),
                            new SleepAction(0.3), //needed
                            new ParallelAction (
                                    armSlide.toOuttakingBackwardAim(),
                                    intakingSystem.backCloseClaw(),
                                    dropOffFirst.build()
                            ),
                            armSlide.toOuttakingBackwardScore(),
                            intakingSystem.backOpenClaw(),
                            new SleepAction(0.1),

//                        Grab second sample and score
                            new ParallelAction (
                                    intakingSystem.perpOpenClaw(),
                                    armSlide.toRest(),
                                    intakingSecond.build()
                            ),
                        new SleepAction(0.5),
                        intakingSystem.perpCloseClaw(),
                        new SleepAction(0.3), //needed
                        new ParallelAction (
                                armSlide.toOuttakingBackwardAim(),
                                intakingSystem.backCloseClaw(),
                                dropOffSecond.build()
                        ),
                        armSlide.toOuttakingBackwardScore(),
                        intakingSystem.backOpenClaw(),


//                        Grab second sample and score
                        new ParallelAction (
                                armSlide.toRest(),
                                intakingThird.build()
                        )
//                        new SleepAction(0.5),
//                        intakingSystem.closeClaw(),
//                        new ParallelAction (
//                                armSlide.toOuttakingBackwardAim(),
//                                dropOffThird.build()
//                        ),
//                        armSlide.toOuttakingBackwardScore(),
//                        intakingSystem.openClaw(),
//
//                        park.build()

//                        dropOffSecond.build(),
//                        intakingThird.build(),
//                        dropOffThird.build(),
//                        park.build()


//                        new ParallelAction(
//                            armSlide.toOuttakingAim(),
//                            dropOffPreloaded.build()
//                        ),
//                        new SleepAction(0.5),
//                        armSlide.toOuttakingScore(),
//                        new SleepAction(0.5),
//                        intakingSystem.openClaw(),
//                        new SleepAction(0.2),
//                        armSlide.toRest(),
//                        toFirstSample.build()








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
        if (mode == Mode.AIM || mode == Mode.SCORE){
            slidePIDF.setPIDF(slidePE,slideIE,slideDE,slideFE);
        }else {
            slidePIDF.setPIDF(slideP, slideI, slideD, slideF);
        }
        int currentPosition = (motor.getCurrentPosition()+motor2.getCurrentPosition())/2;
        double output = slidePIDF.calculate(currentPosition, target);


        return output;
    }
}
//MEEEEEEP MEEEP MEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEPPPP

