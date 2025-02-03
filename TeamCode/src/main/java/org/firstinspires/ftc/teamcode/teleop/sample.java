package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp
public class sample extends LinearOpMode{
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    DcMotorEx AMotor, S1Motor, S2Motor, FL, FR, BL, BR = null;
    Servo rotation, wrist, claw,hangL,hangR;

    public double wristPar = 0.1, wristPerp = 0.62, wristOuttake = 0.82;
    public double clawOpen = 0.27, clawClose = 0.77;
    public double rotationPos = 0.46;
    public double armDown = 30;
    public double armPar = 150, armUp = 1200;
    public int slideInterval = 15;
    public double outToRestBuffer = 800, restToOuttake = 1000;

    //  ARM PID
    PIDFController armPIDF = new PIDFController(0,0,0, 0);
    static double armP = 0.007, armI = 0, armD = 0, armF = 0;
    static double armTarget = 0.0;

    //  SLIDES PID
    PIDFController slidePIDF = new PIDFController(0,0,0, 0);
    static double slideP = 0.0034, slideI = 0, slideD = 0, slideF = 0;
    static double slidePE = 0.008, slideIE = 0, slideDE = 0, slideFE = 0;
    static double slideTarget = 0.0;
    double slidePower = 0.0;


    boolean rightBumperPrevState = false;
    boolean switched = false;
    boolean switchPrev = false;
    boolean hangPrev = false;
    boolean hangYPrev = false;
    boolean hangXPrev = false;
    boolean toHang = false;
    boolean hangBack = false;
    boolean hangBPrev = false;
    boolean hanging = false;
    boolean clawPressed = false;
    boolean clawIsOpen = false;
    boolean init = true;
    boolean slideExtended = false;
    boolean retractSlide = false;
    boolean retracted = true;
    boolean slideOuttake = false;
    boolean micro = false;
    boolean intakePrev = false;
    boolean slideRest = false;
    boolean firstRun = true;
    boolean firstRun1 = true;

    double frontLeftPower, frontRightPower, backLeftPower, backRightPower;
    double armTempTarget = armPar;
    double armMax = 1350;
    double slideMax = 2900;

    public enum Mode {
        REST,
        OUTTAKING,
        INTAKING,
        HANG
    }
    Mode mode = Mode.HANG;



    public void initHardware() {
        AMotor = hardwareMap.get(DcMotorEx.class, "AMotor");
        S1Motor = hardwareMap.get(DcMotorEx.class, "S1Motor");
        S2Motor = hardwareMap.get(DcMotorEx.class, "S2Motor");
        FL = hardwareMap.get(DcMotorEx.class,"FL");
        FR = hardwareMap.get(DcMotorEx.class,"FR");
        BL = hardwareMap.get(DcMotorEx.class,"BL");
        BR = hardwareMap.get(DcMotorEx.class,"BR");


        rotation = hardwareMap.get(Servo.class,"rotation");
        wrist = hardwareMap.get(Servo.class,"wrist");
        claw = hardwareMap.get(Servo.class,"claw");
        hangL = hardwareMap.get(Servo.class,"hangL");
        hangR = hardwareMap.get(Servo.class,"hangR");


        FL.setDirection(DcMotorEx.Direction.REVERSE);
        BL.setDirection(DcMotorEx.Direction.REVERSE);
        FR.setDirection(DcMotorEx.Direction.FORWARD);
        BR.setDirection(DcMotorEx.Direction.FORWARD);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

//        wrist.setPosition(wristPar);


        AMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        AMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        AMotor.setPower(0);

        S1Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        S1Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        S1Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        S1Motor.setPower(0);

        S2Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        S2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        S2Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        S2Motor.setPower(0);
//
//        armTarget = 700;
//        slideTarget = 800;
////        Runs to arm pose
//        ElapsedTime timer = new ElapsedTime();
//        while (Math.abs(AMotor.getCurrentPosition() - armTarget) > 10 && timer.seconds() < 3) { // Safety timeout of 3 seconds
//            double armPower = armPIDF(armTarget, AMotor);
//            AMotor.setPower(








//            Power);
//
//            double slidePower = slidePIDF(slideTarget, S1Motor, S2Motor);
//            S1Motor.setPower(slidePower);
//            S2Motor.setPower(slidePower);
//
//            telemetry.addData("Arm Position", AMotor.getCurrentPosition());
//            telemetry.addData("Arm Target", armTarget);
//            telemetry.addData("Slide Position", S1Motor.getCurrentPosition());
//            telemetry.addData("Slide Target", slideTarget);
//            telemetry.update();
//        }
//        AMotor.setPower(0);
//        S1Motor.setPower(0);
//        S2Motor.setPower(0);
    }

//
//                        /^--^\     /^--^\     /^--^\
//                        \____/     \____/     \____/
//                       /      \   /      \   /      \
//                      |        | |        | |        |
//                       \__  __/   \__  __/   \__  __/
//  |^|^|^|^|^|^|^|^|^|^|^|^\ \^|^|^|^/ /^|^|^|^|^\ \^|^|^|^|^|^|^|^|^|^|^|^|
//  | | | | | | | | | | | | |\ \| | |/ /| | | | | | \ \ | | | | | | | | | | |
//  ########################/ /######\ \###########/ /#######################
//  | | | | | | | | | | | | \/| | | | \/| | | | | |\/ | | | | | | | | | | | |
//  |_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|


    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();

        while (opModeIsActive()) {
//  DRIVE
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            if (!micro) {
                double denom = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1);
                frontLeftPower = (y + x + rx) / denom;
                backLeftPower = (y - x + rx) / denom;
                frontRightPower = (y - x - rx) / denom;
                backRightPower = (y + x - rx) / denom;

                FL.setPower(frontLeftPower);
                FR.setPower(frontRightPower);
                BL.setPower(backLeftPower);
                BR.setPower(backRightPower);
            } else {
                //TODO trig calculation for rotation

                frontLeftPower = rx / 3;
                backLeftPower = rx / 3;
                frontRightPower = -rx / 3;
                backRightPower = -rx / 3;

                FL.setPower(frontLeftPower);
                FR.setPower(frontRightPower);
                BL.setPower(backLeftPower);
                BR.setPower(backRightPower);
//                if (x!= 0) {
//                    if (y>=0) {
//                        rotation.setPosition(1 - (Math.acos(x / (Math.pow(Math.pow(x, 2) + Math.pow(y, 2), 0.5))) / Math.PI));
//                    }
//                }
                slideTarget += (y > 0 && slideTarget < slideMax) ? 30 * y / 1.5 : 0;
                slideTarget += (y < 0 && slideTarget > 300) ? 30 * y / 1.5 : 0;
                if (gamepad1.left_trigger > 0 && rotationPos >= 0) {
                    rotationPos -= gamepad1.left_trigger / 60;
                    if (rotationPos < 0) rotationPos = 1; // Ensure upper bound
                }
                if (gamepad1.right_trigger > 0 && rotationPos <= 1) {
                    rotationPos += gamepad1.right_trigger / 60;
                    if (rotationPos > 1) rotationPos = 0; // Ensure lower bound
                }
                rotation.setPosition(rotationPos);
            }

//  ARM & SLIDE PID
            if (armTarget >= 0 && armTarget <= armMax) {
                AMotor.setPower(armPIDF(armTarget, AMotor));
            } else {
                AMotor.setPower(0);
            }
            if (slideTarget >= 200 && slideTarget <= slideMax) {
                slidePower = slidePIDF(slideTarget, S1Motor, S2Motor);
                S1Motor.setPower(slidePower);
                S2Motor.setPower(slidePower);
            } else {
                S1Motor.setPower(0);
                S2Motor.setPower(0);
            }

            if (mode == Mode.INTAKING || micro) {
                slideMax = 2000;
            } else {
                if (switched){
                    slideMax = 1200;
                }else {
                    slideMax = 2900;
                }
            }


//  CLAW
            if (gamepad1.a) {
                if (!clawPressed) {
                    clawPressed = true;
                    clawIsOpen = !clawIsOpen;
                }
            } else {
                clawPressed = false;
            }

            if (!clawIsOpen) {
                claw.setPosition(clawClose);
            } else {
                claw.setPosition(clawOpen);
            }

//  SLIDES
            slideTarget += (gamepad1.dpad_up && slideTarget < slideMax) ? slideInterval : 0;
            slideTarget -= (gamepad1.dpad_down && slideTarget > 500) ? slideInterval : 0;
            slideTarget = Math.min(2900, Math.max(200, slideTarget));

            slideExtended = slideTarget > 300;

//  ARM

            armTempTarget += (gamepad1.left_trigger > 0 && !micro) ? 3 : 0;
            armTempTarget -= (gamepad1.right_trigger > 0 && !micro) ? 3 : 0;
            armTempTarget = Math.min(1350, Math.max(0, armTempTarget));


//             /\_/\
//            ( o.o )
//             > ^ <    Purrrr...


//  MODES
            boolean rightBumperCurrentState = gamepad1.right_bumper;
            if (rightBumperCurrentState && !rightBumperPrevState) {
                if (mode == Mode.REST) {
                    mode = Mode.OUTTAKING;
                    slideInterval = 24;
                    init = true;
                } else if (mode == Mode.OUTTAKING) {
                    wrist.setPosition(wristPar);
                    mode = Mode.REST;
                    init = true;
                } else if (mode == Mode.INTAKING) {
                    wrist.setPosition(wristPerp);
                    micro = false;
                    mode = Mode.REST;
                    init = true;
                }
            }
            rightBumperPrevState = rightBumperCurrentState;


            telemetry.addData("retract", retractSlide);

            boolean switchCurr = gamepad1.right_stick_button;
            if (switchCurr && !switchPrev){
                if (!switched){
                    switched = true;
                }else{
                    switched = false;
                }
            }
            switchPrev = switchCurr;

            boolean hangYCurr = gamepad1.y;
            if (hangYCurr && !hangYPrev){
                toHang = !toHang;
            }
            hangYPrev = hangYCurr;

            boolean hangBCurr = gamepad1.b;
            if (hangBCurr && !hangBPrev){
                hanging = !hanging;
            }
            hangBPrev = hangBCurr;

            boolean hangXCurr = gamepad1.x;
            if (hangXCurr && !hangXPrev){
                hangBack = !hangBack;
            }
            hangXPrev = hangXCurr;

            if (toHang){
                if (hanging){
                    hangL.setPosition(0.52);
                    hangR.setPosition(0.46);
                }else if(hangBack){
                    hangL.setPosition(0);
                    hangR.setPosition(1);
                }else{
                    hangL.setPosition(1);
                    hangR.setPosition(0);
                }
            }else{
                hangL.setPosition(0.5);
                hangR.setPosition(0.5);
            }




            boolean hangCurr = gamepad1.left_stick_button;
            if (hangCurr && !hangPrev) {
                if (mode == Mode.REST) {
                    mode = Mode.HANG;
                } else if (mode == Mode.HANG) {
                    wrist.setPosition(wristPerp);
                    mode = Mode.REST;
                }
                init = true;
            }
            hangPrev = hangCurr;

            if (firstRun) {
                if (firstRun1) {
                    armTarget = 1000;
                    slideTarget = 300;
                    wrist.setPosition(wristPerp);
                    if (S1Motor.getCurrentPosition() < 350 && AMotor.getCurrentPosition() > 700) {
                        armTarget = 200;
                        firstRun1 = false;
                    }
                }
                else if (!firstRun1 && AMotor.getCurrentPosition() < 300) {
                    micro = true;
                    rotationPos = 0.5;
                    slideTarget = 800;
                    mode = Mode.INTAKING;
                    init = true;
                    firstRun = false;
                }
            } else {


                telemetry.addData("mode type", mode);

                switch (mode) {
                    /** REST */
                    case REST:
                        if (init) {
                            slideTarget = 200;
                            rotation.setPosition(0.5);
                            slideRest = true;
                        }
                        init = false;


                        if (slideRest && S1Motor.getCurrentPosition() - 200 < outToRestBuffer) { //distance from slide retracted
                            armTempTarget = armPar;
                            slideRest = false;
                            wrist.setPosition(wristPerp);
                        }

                        // ARM POSITION
                        armTarget = armTempTarget;

                        // CHANGE TO INTAKING


                        boolean intakeCurr = gamepad1.left_bumper;
                        if (intakeCurr && !intakePrev) {
                            micro = true;
                            rotationPos = 0.5;
                            slideTarget = 1000;
                            mode = Mode.INTAKING;
                            init = true;
                        }
                        intakePrev = intakeCurr;

                        break;

                    /** INTAKING */
                    case INTAKING:
                        if (init) {
                            wrist.setPosition(wristPar);
                            clawIsOpen = true;
                            armTempTarget = armPar;
                        }
                        init = false;


                        //  LOWER ARM
                        armTarget = (gamepad1.left_bumper) ? armDown : armTempTarget;


                        break;

                    /** OUTTAKING */
                    case OUTTAKING:
                        if (init) {
                            armTempTarget = armUp;
                            slideOuttake = true;
                            rotation.setPosition(0.5);
                            wrist.setPosition(wristPar);


                        }
                        init = false;
                        if (S1Motor.getCurrentPosition() > 1000) {
                            wrist.setPosition(wristOuttake);
                        }
                        if (slideOuttake && armTempTarget - AMotor.getCurrentPosition() < restToOuttake) {
                            slideTarget = slideMax;
                            slideOuttake = false;
                        }


                        //  ARM
                        armTarget = armTempTarget;

                        break;

                    /** HANG */
                    case HANG:
                        if (init) {
                            clawIsOpen = false;
                            armTarget = 700;
                            slideTarget = 800;
                            wrist.setPosition(wristPar);
                            rotation.setPosition(0.5);
                        }


                        break;
                }
            }

            telemetry.addData("arm current", AMotor.getCurrentPosition());
            telemetry.addData("arm target", armTarget);
            telemetry.addData("slide1 current", S1Motor.getCurrentPosition());
            telemetry.addData("slide2 current", S2Motor.getCurrentPosition());
            telemetry.addData("slide target", slideTarget);

            telemetry.addData("rotation", rotationPos);
            telemetry.addData("init", init);
            telemetry.addData("firstRun ", firstRun);
            telemetry.addData("firstRun1", firstRun1);
            telemetry.addData("hang",hanging);
            telemetry.update();
            dashboardTelemetry.update();

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