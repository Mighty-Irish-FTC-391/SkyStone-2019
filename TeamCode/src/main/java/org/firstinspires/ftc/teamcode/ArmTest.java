package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "ArmTest", group = "Test")
public class ArmTest extends LinearOpMode {
    DcMotor mot0;
    DcMotor mot1;
    @Override
    public void runOpMode() {

        mot0 = hardwareMap.dcMotor.get("mot0");
        mot1 = hardwareMap.dcMotor.get("mot1");

        mot0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mot1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mot0.setDirection(DcMotorSimple.Direction.FORWARD);
        mot1.setDirection(DcMotorSimple.Direction.FORWARD);

        mot0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            mot0.setPower(gamepad1.left_stick_y);
            mot1.setPower(gamepad1.right_stick_y);

        }
    }
}
