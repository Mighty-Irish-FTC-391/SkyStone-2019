package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Benton Solo", group = "drive")
public class BentonSingle extends LinearOpMode {
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor base;
    DcMotor arm;

    CRServo gripBack;
    Servo gripFront1;
    Servo gripFront2;

    Servo gripLeft;
    Servo gripRight;
    CRServo wrench;
    public void runOpMode(){
        leftDrive = hardwareMap.dcMotor.get("left");
        rightDrive = hardwareMap.dcMotor.get("right");

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        base = hardwareMap.dcMotor.get("base");
        arm =  hardwareMap.dcMotor.get("arm");

        base.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        base.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        base.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gripBack = hardwareMap.crservo.get("gripBack");
        gripFront1 = hardwareMap.servo.get("gripFront1");
        gripFront2 = hardwareMap.servo.get("gripFront2");
        gripLeft = hardwareMap.servo.get("gripLeft");
        gripRight = hardwareMap.servo.get("gripRight");
        wrench = hardwareMap.crservo.get("wrench");

        waitForStart();
        while(opModeIsActive()){
            leftDrive.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
            rightDrive.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);

            base.setPower(0.2*gamepad1.right_stick_x);
            arm.setPower(0.5*gamepad1.right_stick_y);

            gripBack.setPower(gamepad1.left_trigger-gamepad1.right_trigger);

            gripFront1.setPosition(gamepad1.a ? 1.0 : gamepad1.b ? 0.0 : gripFront1.getPosition());
            gripFront2.setPosition(gamepad1.a ? 0.0 : gamepad1.b ? 1.0 : gripFront1.getPosition());
            gripLeft.setPosition(gamepad1.x ? (gripLeft.getPosition() > 0.9 ? 0.0 : 1.0) : gripLeft.getPosition());
            gripRight.setPosition(1.0-gripLeft.getPosition());

            wrench.setPower((gamepad1.right_bumper ? 1.0 : 0.0)-(gamepad1.left_bumper ? -1.0 : 0.0));
        }
    }
}
