package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ArmTest", group = "Test")
public class ArmTest extends LinearOpMode {
    CRServo bottom;
    Servo top;
    DcMotor angle;
    @Override
    public void runOpMode() {
        top = (Servo) hardwareMap.get("servo0");
        bottom = (CRServo) hardwareMap.get("servo3");
        angle = (DcMotor) hardwareMap.get("mot0");
        waitForStart();

        while (opModeIsActive()) {
            angle.setPower(gamepad1.left_stick_x);
        bottom.setPower(-gamepad1.left_stick_y);
        top.setPosition(top.getPosition() - gamepad1.right_stick_y);
        }
    }
}
