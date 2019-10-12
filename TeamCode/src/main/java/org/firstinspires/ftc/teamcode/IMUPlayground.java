package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "IMUplaygroud", group = "Test")
public class IMUPlayground extends LinearOpMode {
    BNO055IMU imu;

    @Override
    public void runOpMode() {
        telemetry.addData("imuStatus", "pre-initializing");
        telemetry.update();

        imu = hardwareMap.get(BNO055IMU.class, "nativeIMU");
        //Internal Measurment Unit Parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        //End Internal Measurment Unit Parameters

        imu.initialize(parameters);

        telemetry.addData("imuStatus", "initializing");

        telemetry.update();
        int animate = 0;
        String dots = ".";
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            if (animate % 10 == 0) {
                if (animate % 30 == 0) {
                    dots = ".";
                } else {
                    dots = dots + ".";
                }
            }
            //telemetry.addData("imuStatus",imu.getCalibrationStatus().toString());
            telemetry.addData("imuStatus", "calibrating" + dots);
            telemetry.update();
            sleep(50);
            idle();
            animate++;
        }
        telemetry.addData("imuStatus", imu.getCalibrationStatus().toString());
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            //Loop
            String rotInfo = String.format("x:%f, y:%f, z:%f", imu.getAngularVelocity().xRotationRate, imu.getAngularVelocity().yRotationRate, imu.getAngularVelocity().zRotationRate);
            telemetry.addData("imuStatus", rotInfo);
            telemetry.update();
        }
    }
}
