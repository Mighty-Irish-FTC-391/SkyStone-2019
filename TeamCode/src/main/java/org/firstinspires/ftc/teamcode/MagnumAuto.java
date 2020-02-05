package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Magnum Autonomous", group = "*competition*")
public class MagnumAuto extends LinearOpMode {

    //CONSTANTS
    public final double DRIVER_RATIO = 0.3; //ratio of the main driver's control over the wheels to the crane driver's control
    public final double MAX_SPEED_MECH_WHEEL = 1.0; //in degrees per second
    public final double[][] PIDF_MECH = null; //PIDF constants for the mechanum wheels
    public final double SLIDE_MAX_POW = 1.0; //slide servo speed, in ¯\_(ツ)_/¯
    public final double ARM_POW = 0.5;//both arm motor speed, in degrees per second

    //button history trackers
    boolean a1_last = false;
    boolean a2_last = false;

    //motors for mechanum drive go counterclockwise from the bottom right. Should be marked on robot.
    DcMotorEx mech0;
    DcMotorEx mech1;
    DcMotorEx mech2;
    DcMotorEx mech3;

    //Motors for the main arm
    DcMotorEx spoolArm;
    DcMotorEx rightArm;
    DcMotorEx leftArm;

    //Servos for the main arm & pincer
    DcMotorEx slide;
    Servo claw;

    //Servos for grabbing the base plate
    Servo waffleLeft;
    Servo waffleRight;

    //ColorSensor cs;

    @Override
    public void runOpMode(){
        //initialize motors for mechanum drive go counterclockwise from the bottom right. Use motors with encoders for DcMotorEx
        mech0 = (DcMotorEx) hardwareMap.dcMotor.get("mech0");
        mech1 = (DcMotorEx) hardwareMap.dcMotor.get("mech1");
        mech2 = (DcMotorEx) hardwareMap.dcMotor.get("mech2");
        mech3 = (DcMotorEx) hardwareMap.dcMotor.get("mech3");

        mech0.setDirection(DcMotorSimple.Direction.REVERSE);
        mech1.setDirection(DcMotorSimple.Direction.REVERSE);
        mech2.setDirection(DcMotorSimple.Direction.FORWARD);
        mech3.setDirection(DcMotorSimple.Direction.FORWARD);

        mech0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mech1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mech2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mech3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(PIDF_MECH != null){
            mech0.setVelocityPIDFCoefficients(PIDF_MECH[0][1],PIDF_MECH[0][1],PIDF_MECH[0][2],PIDF_MECH[0][3]);
            mech1.setVelocityPIDFCoefficients(PIDF_MECH[1][1],PIDF_MECH[1][1],PIDF_MECH[1][2],PIDF_MECH[1][3]);
            mech2.setVelocityPIDFCoefficients(PIDF_MECH[2][1],PIDF_MECH[2][1],PIDF_MECH[2][2],PIDF_MECH[2][3]);
            mech3.setVelocityPIDFCoefficients(PIDF_MECH[3][1],PIDF_MECH[3][1],PIDF_MECH[3][2],PIDF_MECH[3][3]);
        }

        //Motors for the main arm. Use motors with encoders for DcMotorEx
        spoolArm = (DcMotorEx) hardwareMap.dcMotor.get("spoolArm");
        leftArm = (DcMotorEx) hardwareMap.dcMotor.get ("L_Arm");
        rightArm = (DcMotorEx) hardwareMap.dcMotor.get("R_Arm");

        spoolArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servos for the main arm & pincer
        slide = (DcMotorEx) hardwareMap.dcMotor.get("slide");
        claw = hardwareMap.servo.get("claw");
        //Servos for grabbing the base plate
        waffleLeft = hardwareMap.servo.get("L_waffle");
        waffleRight = hardwareMap.servo.get("R_waffle");

        //sensors
       // cs = hardwareMap.colorSensor.get("colorSensor");
        waitForStart();

        while(opModeIsActive()){
           // telemetry.addData("i2c addr", String.format("@%X", cs.getI2cAddress().get8Bit()));

            //I2cDeviceReader dr = new I2cDeviceReader(cs.getI2cAddress());
            /*telemetry.addLine("<Sensor Dump>");
            String str = "";
            for (int i = 0; i<dump.length; i++){
                if(i%4 == 0){
                    str = str+"\t";
                    if(i%8 == 0){
                        str = str +String.format("\n0x%04X:\t", i);
                    }
                }
                str = str + String.format(" %02X", dump[i]);
            }
            telemetry.addLine(str);
            updateTelemetry(telemetry);*/
        }
        telemetry.addLine("goodbye!");
        updateTelemetry(telemetry);
    }

    public double[] mechanumPower(double x, double y, double rot) {
        //Starting from the front left (0), counterclockwise (1,2,3), ratios for motor powers in range of [-1,1]

        double[] coeffs = new double[4];
        coeffs[0] = y + x - rot;
        coeffs[1] = y - x - rot;
        coeffs[2] = y + x + rot;
        coeffs[3] = y - x + rot;

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