package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class servoTest extends LinearOpMode {
    // Declare servo variables
    private Servo servo0, servo1, servo2;

    private void initHardware() {
        servo0 = hardwareMap.get(Servo.class, "servo0");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
    }

    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();

        while (opModeIsActive()) {
            // Control servo0 with gamepad1 buttons A and B
            if (gamepad1.a) {
                servo0.setPosition(0.0);
            } else if (gamepad1.b) {
                servo0.setPosition(1.0);
            } else {
                servo0.setPosition(0.5);
            }

            // Control servo1 with gamepad1 buttons X and Y
            if (gamepad1.x) {
                servo1.setPosition(0.0);
            } else if (gamepad1.y) {
                servo1.setPosition(1.0);
            } else {
                servo1.setPosition(0.5);
            }

            // Control servo2 with gamepad1 D-Pad
            if (gamepad1.dpad_up) {
                servo2.setPosition(0.0); // Moved within valid range
            } else if (gamepad1.dpad_down) {
                servo2.setPosition(1.0);
            } else {
                servo2.setPosition(0.5);

            }

            // Display servo positions in telemetry
            telemetry.addData("Servo0 Position", servo0.getPosition());
            telemetry.addData("Servo1 Position", servo1.getPosition());
            telemetry.addData("Servo2 Position", servo2.getPosition());
            telemetry.update();
        }
    }
}