package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;

@TeleOp(name = "IMUplaygroud", group = "Test")
public class IMUPlayground extends LinearOpMode {
    BNO055IMU imu;

    HashMap<Double, Double> mapX = new HashMap<>();
    HashMap<Double, Double> mapY = new HashMap<>();
    HashMap<Double, Double> mapZ = new HashMap<>();
    HashMap<Double, Double> mapM = new HashMap<>();

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
            Acceleration acc = imu.getLinearAcceleration();
            double xAcc = acc.xAccel;
            double yAcc = acc.yAccel;
            double zAcc = acc.zAccel;
            String posInfo = String.format("x:%f, y:%f, z:%f", xAcc, yAcc, zAcc);
            double mag = Math.sqrt(Math.pow(xAcc,2) + Math.pow(yAcc,2) + Math.pow(zAcc,2));
            mapX.put(time, xAcc);
            mapY.put(time, yAcc);
            mapZ.put(time, zAcc);
            mapM.put(time, mag);
            telemetry.addData("accInfo", posInfo);
            telemetry.addData("accMag", mag);
            if(gamepad1.a){
                File fileX = AppUtil.getInstance().getSettingsFile("accX.csv");
                ReadWriteFile.writeFile(fileX, getCSVstring(mapX, "x acc"));

                File fileY = AppUtil.getInstance().getSettingsFile("accY.csv");
                ReadWriteFile.writeFile(fileY, getCSVstring(mapY, "y acc"));

                File fileZ = AppUtil.getInstance().getSettingsFile("accZ.csv");
                ReadWriteFile.writeFile(fileZ, getCSVstring(mapZ, "z acc"));

                File fileM = AppUtil.getInstance().getSettingsFile("accM.csv");
                ReadWriteFile.writeFile(fileM, getCSVstring(mapM, "mag acc"));
                telemetry.addLine("Wrote to files at " + time + " s");
            }
            telemetry.update();

        }
    }
    public String getCSVstring(HashMap accData, String type){
        String str = "time,"+type+"\n";
        Set<Map.Entry> eSet = accData.entrySet();
        for (Map.Entry<Double, Double> entry : eSet) {
            str = str + entry.getKey().toString() + "," + entry.getValue().toString() + "\n";
        }
        return str;
    }
}
