package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "dAnkbotTest", group = "Competition")
public class dAnkbotTest extends LinearOpMode {



    DcMotor lmott;
    DcMotor lamot;

    DcMotor smot;
    DcMotor amot;

    CRServo gripper;
    Servo fruntwon;
    Servo frunttoo;


    @Override
    public void runOpMode() {
        //INIT START



        lamot = hardwareMap.dcMotor.get("lamot");
        lmott = hardwareMap.dcMotor.get("lmott");
        smot = hardwareMap.dcMotor.get("smot");
        amot = hardwareMap.dcMotor.get("amot");
        gripper = hardwareMap.crservo.get("gripper");
        fruntwon = hardwareMap.servo.get("fruntwon");
        frunttoo = hardwareMap.servo.get("frunttoo");


        lamot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lmott.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        smot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        amot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        lamot.setDirection(DcMotorSimple.Direction.FORWARD);
        lmott.setDirection(DcMotorSimple.Direction.FORWARD);
        smot.setDirection(DcMotorSimple.Direction.FORWARD);
        amot.setDirection(DcMotorSimple.Direction.FORWARD);


        lamot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lmott.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        smot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        amot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //END INIT
        waitForStart();
        //START START
        //END START
        while (opModeIsActive())


            lamot.setPower(gamepad1.left_stick_x);
            lmott.setPower(gamepad1.left_stick_y);

        gripper.setPower(gamepad1.left_trigger-gamepad1.left_trigger);


            //lamot.setPower(gamepad1.left_trigger-gamepad1.right_trigger);

            if(gamepad2.left_bumper) {
            smot.setPower(.15);
            } else if(gamepad2.right_bumper) {
            smot.setPower(-.15);
            } else {
                smot.setPower(0);
            }

            amot.setPower(gamepad2.left_trigger-gamepad2.right_trigger);




    }

}




