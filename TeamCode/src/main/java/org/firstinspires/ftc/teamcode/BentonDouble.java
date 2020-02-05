package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Benton OwO", group = "Competition")
public class BentonDouble extends LinearOpMode {
    DcMotor lmot;
    DcMotor rmot;
    DcMotor suemot;
    DcMotor arm;

    CRServo push;

    Servo grip0;
    Servo grip1a;
    Servo grip1b;
    Servo waffle1;
    Servo waffle2;
    public void runOpMode(){

//        String lmotName = new String(new byte[]{'l','m','o','t'});
//        lmot = hardwareMap.dcMotor.get(lmotName);

        lmot = hardwareMap.dcMotor.get("lmot");
        rmot = hardwareMap.dcMotor.get("rmot");

        lmot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rmot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lmot.setDirection(DcMotorSimple.Direction.FORWARD);
        rmot.setDirection(DcMotorSimple.Direction.REVERSE);

        lmot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rmot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        suemot = hardwareMap.dcMotor.get("suemot");
        arm =  hardwareMap.dcMotor.get("arm");

        suemot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        suemot.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        suemot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        push = hardwareMap.crservo.get("push");
        grip1a = hardwareMap.servo.get("grip1");
        grip1b = hardwareMap.servo.get("grip2");
        grip0 = hardwareMap.servo.get("grip0");
        waffle1 = hardwareMap.servo.get("waffle1");
        waffle2 = hardwareMap.servo.get("waffle2");

        grip0.setPosition(0.0);
        grip1a.setPosition(1.0);
        grip1b.setPosition(0.0);

        grip1a.scaleRange(0.0,0.51);
        grip0.scaleRange(0.0,0.75);
        //waffle1.setPosition(1.0);
        //waffle2.setPosition(1.0);

        waitForStart();
        while(opModeIsActive()){
            lmot.setPower(-0.5*gamepad1.right_stick_y);
            rmot.setPower(-0.5*gamepad1.left_stick_y);

            suemot.setPower(0.25*gamepad2.right_stick_x);
            arm.setPower(0.5*-gamepad2.left_stick_y);

            push.setPower(gamepad1.left_trigger-gamepad1.right_trigger);

            grip1a.setPosition(gamepad2.right_bumper ? grip1a.getPosition()-0.005 : gamepad2.left_bumper ? grip1a.getPosition()+0.005 : grip1a.getPosition());
            grip1b.setPosition(1.0-grip1a.getPosition());
            grip0.setPosition(gamepad2.right_trigger > 0.0 ? grip0.getPosition()+(0.005*gamepad2.right_trigger) : gamepad2.left_trigger > 0.0 ? grip0.getPosition()-(0.005*gamepad2.left_trigger) : grip0.getPosition());

            //waffle1.setPosition(gamepad1.y ? 0.0 : 0.0);
            //waffle2.setPosition(gamepad1.y ? 0.5 : 0.0);

            telemetry.addData("gripA", grip1a.getPosition() + " - Stay between 1 and 0");
            telemetry.addData("gripB", grip1b.getPosition() + " - Stay between 1 and 0");
            telemetry.addData("wrist", grip0.getPosition() + " - Stay between 1 and 0");
            telemetry.update();

        }
    }
}
