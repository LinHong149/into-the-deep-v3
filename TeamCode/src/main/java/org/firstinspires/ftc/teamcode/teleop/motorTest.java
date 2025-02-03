package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class motorTest extends LinearOpMode {
    // Declare motor variables
    private DcMotor motor0;
    private DcMotor motor1;
    private DcMotor motor2;

    public void initHardware() {
        motor0 = hardwareMap.get(DcMotor.class, "S1Motor");
        motor1 = hardwareMap.get(DcMotor.class, "S2Motor");
        motor2 = hardwareMap.get(DcMotor.class, "AMotor");

        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // motor0.setDirection(DcMotor.Direction.REVERSE);
        // motor1.setDirection(DcMotor.Direction.REVERSE);
        // motor2.setDirection(DcMotor.Direction.REVERSE);
        // motor3.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.a) {
                motor0.setPower(-1.0);
                motor1.setPower(-1.0);
            } else if (gamepad1.b) {
                motor0.setPower(1.0);
                motor1.setPower(1.0);
            } else {
                motor0.setPower(0.0);
                motor1.setPower(0.0);
            }

            if (gamepad1.x) {
                motor2.setPower(-1.0);
            } else if (gamepad1.y) {
                motor2.setPower(1.0);
            } else {
                motor2.setPower(0.0);
            }


            telemetry.addData("motor0/1 Power", motor0.getPower()); // motor0 and motor1 will have the same power
            telemetry.addData("motor2 Power", motor2.getPower());
            telemetry.addData("arm pos",motor2.getCurrentPosition());

            telemetry.update();
        }
    }
}