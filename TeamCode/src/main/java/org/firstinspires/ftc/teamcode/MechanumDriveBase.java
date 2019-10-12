
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="Mechanum Drive Base", group="Test")
public class MechanumDriveBase extends LinearOpMode {
    final double sumSineCosineWheelHypo = 2.0/Math.sqrt(2.0); //Sum of the sine and cosine of the triangle formed by the distance of the wheel to the center of the robot

    DcMotor flmot;
    DcMotor blmot;
    DcMotor frmot;
    DcMotor brmot;

    //BNO055IMU imu;
    @Override
    public void runOpMode() {
        //Init
        /*imu = hardwareMap.get(BNO055IMU.class,"nativeIMU");
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
        while(!isStopRequested() && !imu.isGyroCalibrated()){
            if(animate%10==0){
                if(animate%30==0){
                    dots = ".";
                }else {
                    dots = dots + ".";
                }
            }
            //telemetry.addData("imuStatus",imu.getCalibrationStatus().toString());
            telemetry.addData("imuStatus", "calibrating"+dots);
            telemetry.update();
            sleep(50);
            idle();
            animate++;
        }
        telemetry.addData("imuStatus",imu.getCalibrationStatus().toString());
        telemetry.update();
        */
        flmot = hardwareMap.dcMotor.get("mot0");
        blmot = hardwareMap.dcMotor.get("mot1");
        brmot = hardwareMap.dcMotor.get("mot2");
        frmot = hardwareMap.dcMotor.get("mot3");

        flmot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frmot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blmot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brmot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flmot.setDirection(DcMotorSimple.Direction.FORWARD);
        frmot.setDirection(DcMotorSimple.Direction.FORWARD);
        blmot.setDirection(DcMotorSimple.Direction.REVERSE);
        brmot.setDirection(DcMotorSimple.Direction.REVERSE);

        flmot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frmot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blmot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brmot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        //OnStart

        while (opModeIsActive()) {
            //Loop
            double[] pows = mechanumPower(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
            //String rotInfo = String.format("x:%f, y:%f, z:%f",imu.getAngularVelocity().xRotationRate,imu.getAngularVelocity().yRotationRate,imu.getAngularVelocity().zRotationRate);
            flmot.setPower(pows[0]);
            blmot.setPower(pows[1]);
            brmot.setPower(pows[2]);
            frmot.setPower(pows[3]);

            telemetry.addData("FL",pows[0]);
            telemetry.addData("BL",pows[1]);
            telemetry.addData("BR",pows[2]);
            telemetry.addData("FR",pows[3]);
            //telemetry.addData("imuStatus",rotInfo);
            //telemetry.update();
        }
    }

    public double[] mechanumPower(double x, double y, double rot) {
        //Starting from the front left (0), counterclockwise (1,2,3), ratios for motor powers in range of [-1,1]

        double[] coeffs = new double[4];
        coeffs[0] = y + x - rot*sumSineCosineWheelHypo;
        coeffs[1] = y - x - rot*sumSineCosineWheelHypo;
        coeffs[2] = y + x + rot*sumSineCosineWheelHypo;
        coeffs[3] = y - x + rot*sumSineCosineWheelHypo;

        double largest = Math.max(Math.max(Math.abs(coeffs[0]),Math.abs(coeffs[1])),Math.max(Math.abs(coeffs[2]),Math.abs(coeffs[3])));
        if(largest > 1.0d) {
            coeffs[0] = coeffs[0]/largest;
            coeffs[1] = coeffs[1]/largest;
            coeffs[2] = coeffs[2]/largest;
            coeffs[3] = coeffs[3]/largest;
        }
        return coeffs;
    }
}
