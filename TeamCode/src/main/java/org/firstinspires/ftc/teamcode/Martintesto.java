
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.CRServo;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



@TeleOp(name="Martin Testt", group="Test")
public class Martintesto extends LinearOpMode {


    CRServo ser0;

    //BNO055IMU imu;
    @Override
    public void runOpMode() {


        ser0 = hardwareMap.crservo.get("ser0");


        waitForStart();
        //OnStart

        while (opModeIsActive()) {

            ser0.setPower(gamepad1.left_stick_y);

        }
    }


    }

