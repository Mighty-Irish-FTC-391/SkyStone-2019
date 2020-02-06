package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
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
    public final double SATURATION_MIN = 50.0;//saturation of the target colors, from 0-100
    public final double VALUE_MIN = 30.0;//brightness of the target colors, from 0-100

    //hues, from 0 to 360 degrees. Think of a color wheel, where red is 0/360
    public final double HUE_BLUE_MAX = 270.0;
    public final double HUE_BLUE_MIN = 180.0;

    public final double HUE_RED_MAX = 30.0;
    public final double HUE_RED_MIN = 330.0;

    //applied power to drive
    public final double MECH_POWER = 0.5;

    //Servo stuff
    public final double[] FLIPPERS_RANGE = {0.0, 0.5}; //flipper servo range (inverse applied for opposite flipper)
    public final double FLIPPER_STAGE_ONE = 0.25;//flipper position for it to clear the foundation in stage 1.
    //ignore
    public final double[][] PIDF_MECH = null; //PIDF constants for the mechanum wheels

    //motors for mechanum drive go counterclockwise from the bottom right. Should be marked on robot.
    DcMotorEx mech0;
    DcMotorEx mech1;
    DcMotorEx mech2;
    DcMotorEx mech3;

    //Servos for grabbing the base plate
    Servo waffleLeft;
    Servo waffleRight;

    //sensors
    LynxI2cColorRangeSensor cs;
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

        //Servos for grabbing the base plate
        waffleLeft = hardwareMap.servo.get("L_waffle");
        waffleRight = hardwareMap.servo.get("R_waffle");
        //sensors
        cs = (LynxI2cColorRangeSensor) hardwareMap.colorSensor.get("colorSensor");


        int phase = 0;
        while (!isStarted()){
            doTelemetry(new String[]{""+phase});
        }


        //phase 1
        int phase1Timer = 0;
        waffleLeft.setPosition(1.0-FLIPPER_STAGE_ONE);
        waffleRight.setPosition(FLIPPER_STAGE_ONE);
        while(opModeIsActive() && phase==0){//approach foundation
            phase1Timer++;
            drive(0,-MECH_POWER,0);
            double[] hsv = getHSV(cs.red()/cs.alpha(), cs.green()/cs.alpha(), cs.blue()/cs.alpha());
            if((hsv[1] > SATURATION_MIN && hsv[2] > VALUE_MIN) && ( (hsv[0] > HUE_RED_MIN || hsv[0] < HUE_RED_MAX) || (hsv[0] > HUE_BLUE_MIN && hsv[0] < HUE_BLUE_MAX) )){ //color check.
                phase++;
                break;
            }
            doTelemetry(new String[]{""+phase});
        }
        //phase 2
        if(opModeIsActive() && phase==1) {//grab foundation
            sleep(500);
            waffleLeft.setPosition(1.0-FLIPPERS_RANGE[0]);
            waffleRight.setPosition(FLIPPERS_RANGE[0]);
            sleep(500);
        }
        while(opModeIsActive() && phase==1){//retreat with foundation
            phase1Timer--;
            drive(0,MECH_POWER,0);
            if(phase1Timer <= 0){
                phase++;
                break;
            }
            doTelemetry(new String[]{""+phase});
        }
        doTelemetry(new String[]{""+phase});
        if(opModeIsActive() && phase==2){//release foundation
            sleep(500);
            waffleLeft.setPosition(1.0-FLIPPERS_RANGE[1]);
            waffleRight.setPosition(FLIPPERS_RANGE[1]);
            sleep(500);
        }
        while(opModeIsActive() && phase==2){//approach bridge
            drive(MECH_POWER,0,0);
            double[] hsv = getHSV(cs.red()/cs.alpha(), cs.green()/cs.alpha(), cs.blue()/cs.alpha());
            if((hsv[1] > SATURATION_MIN && hsv[2] > VALUE_MIN) && ( (hsv[0] > HUE_RED_MIN || hsv[0] < HUE_RED_MAX) || (hsv[0] > HUE_BLUE_MIN && hsv[0] < HUE_BLUE_MAX) )){ //color check.
                phase++;
                break;
            }
            doTelemetry(new String[]{""+phase});
        }
        mech0.setPower(0);
        mech1.setPower(0);
        mech2.setPower(0);
        mech3.setPower(0);
        while(opModeIsActive() && phase == 3){//do nothing, update telemetry

        }

    }
    public void drive(double x, double y, double rot){
        double[] coeffs = mechanumPower(x,y,rot);
        mech0.setPower(coeffs[0]);
        mech1.setPower(coeffs[1]);
        mech2.setPower(coeffs[2]);
        mech3.setPower(coeffs[3]);
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

    public void doTelemetry(String[] args){
        //PLEASE FIX. SOMEONE. ANYONE. I DONT CARE ANYMORE JUST PLEASE FIX THIS MESS
        double r = cs.red();
        double g = cs.green();
        double b = cs.blue();
        double a = cs.alpha();
        double[] hsv = getHSV(r/a,g/a,b/a);
        telemetry.addData("phase", args[0]);
        telemetry.addData("Colour", String.format("r: %f, g: %f, b, %f a, %f", r, g, b, a));
        telemetry.addData("% Colour",  String.format("r: %f, g: %f, b, %f", r/a, g/a, b/a));
        telemetry.addData("HSV", String.format("H: %.2f, S: %.2f, V: %.2f, ",hsv[0],hsv[1],hsv[2]));
        telemetry.addData("Mech Vel", String.format("0: %.2f, 1: %.2f, 2: %.2f, 3: %.2f", mech0.getVelocity(AngleUnit.DEGREES), mech1.getVelocity(AngleUnit.DEGREES), mech2.getVelocity(AngleUnit.DEGREES), mech3.getVelocity(AngleUnit.DEGREES)));
        updateTelemetry(telemetry);
    }

    public double[] getHSV(double r,double g, double b) {
        double hue = 0;
        double sat = 0;
        double cMax = Math.max(Math.max(r,g),b);
        double cMin = Math.min(Math.min(r,g),b);
        if(r == g && g == b) {
            hue = 0.0; //grayscale, hue doesn't matter
        }else if(r > g && r > b){ //red greatest
            hue = (((g-b)/(r - cMin)) % 6.0 + 6.0) % 6.0;
        }else if(g > b){//green greatest
            hue = ((b-r)/(g - cMin)) + 2;
        }else{ //blue greatest
            hue = ((r-g)/(b - cMin)) + 4;
        }
        hue = 60*hue;

        if(cMax != 0) {
            sat = (cMax-cMin)/cMax;
        } else{
            sat = 0;
        }

        return new double[]{hue, sat*100, cMax*100};
    }
}