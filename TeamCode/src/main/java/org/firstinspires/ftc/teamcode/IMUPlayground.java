package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@TeleOp(name = "IMUCalibrate", group = "Test")
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
        parameters.loggingEnabled = true;
        //End Internal Measurment Unit Parameters

        imu.initialize(parameters);

        telemetry.addData("imuStatus", "initializing");

        telemetry.update();
        int animate = 0;
        String dots = ".";
        while (!isStopRequested() && !(imu.isGyroCalibrated() && imu.isAccelerometerCalibrated())) {
            if (animate % 10 == 0) {
                if (animate % 30 == 0) {
                    dots = ".";
                } else {
                    dots = dots + ".";
                }
            }
            //telemetry.addData("imuStatus",imu.getCalibrationStatus().toString());
            telemetry.addData("imuStatus", imu.getCalibrationStatus().toString() + " ["+dots+"] ");
            telemetry.update();
            sleep(50);
            idle();
            animate++;
        }
        telemetry.addData("imuStatus", imu.getCalibrationStatus().toString() + " [READY] ");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            //Loop
            telemetry.addData("imuStatus", imu.getCalibrationStatus().toString() + " STA: " + imu.getSystemStatus().toShortString() + " [WORKING] ");
            String rotInfo = String.format("x:%f, y:%f, z:%f", imu.getAngularOrientation().firstAngle, imu.getAngularOrientation().secondAngle, imu.getAngularOrientation().thirdAngle);
            telemetry.addData("rotInfo", rotInfo);

            String posInfo = String.format("x:%f, y:%f, z:%f", imu.getLinearAcceleration().xAccel, imu.getLinearAcceleration().yAccel, imu.getLinearAcceleration().zAccel);
            telemetry.addData("accInfo", posInfo);
            if (gamepad1.start) {
                BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();
                String filename = "calibration.json";
                File file = AppUtil.getInstance().getSettingsFile(filename);
                ReadWriteFile.writeFile(file, calibrationData.serialize());
                telemetry.log().add("Calibration data saved to '%s'", filename);
                while (gamepad1.start) {
                    telemetry.update();
                    idle();
                }
            }
            telemetry.update();
        }
    }
}
