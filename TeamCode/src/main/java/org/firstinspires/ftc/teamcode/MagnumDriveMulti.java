package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Magnum Drive Muti", group = "*competition*")
public class MagnumDriveMulti extends LinearOpMode {

    //CONSTANTS
    public final double DRIVER_RATIO = 0.3; //ratio of the main driver's control over the wheels to the crane driver's control
    public final double MAX_SPEED_MECH_WHEEL = 1.0; //in degrees per second
    public final double[][] PIDF_MECH = null; //PIDF constants for the mechanum wheels
    public final double[] PIDF_ARM = {20.0, 5.0, 5.0, 1.0}; //PIDF constants for the left/right arm motors
    public final double SLIDE_MAX_POW = 0.5; //slide servo speed, in ¯\_(ツ)_/¯
    public final double ARM_MAX_SPEED = 180.0;//both arm motor speed, in degrees per second
    public final double ARM_SCAN_SPEED = 40.0;//taregt position speed maximum, in encoder ticks per cycle
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

        //Motors for the main arm. Use motors with encoders for DcMotorEx
        spoolArm = (DcMotorEx) hardwareMap.dcMotor.get("spoolArm");
        leftArm = (DcMotorEx) hardwareMap.dcMotor.get ("L_Arm");
        rightArm = (DcMotorEx) hardwareMap.dcMotor.get("R_Arm");

        spoolArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightArm.setTargetPosition(0);
        leftArm.setTargetPosition(0);

        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setTargetPositionTolerance(10);
        leftArm.setTargetPositionTolerance(10);

        if(PIDF_ARM != null){
            rightArm.setVelocityPIDFCoefficients(PIDF_ARM[0],PIDF_ARM[1],PIDF_ARM[2],PIDF_ARM[3]);
            leftArm.setVelocityPIDFCoefficients(PIDF_ARM[0],PIDF_ARM[1],PIDF_ARM[2],PIDF_ARM[3]);
            //rightArm.setPositionPIDFCoefficients();
        }

        //Servos for the main arm & pincer
        slide = (DcMotorEx) hardwareMap.dcMotor.get("slide");
        claw = hardwareMap.servo.get("claw");
        //Servos for grabbing the base plate
        waffleLeft = hardwareMap.servo.get("L_waffle");
        waffleRight = hardwareMap.servo.get("R_waffle");
        //sensors
        cs = (LynxI2cColorRangeSensor) hardwareMap.colorSensor.get("colorSensor");


        double tPos = 0.0;
        while (!isStarted()){
            doTelemetry(new String[]{""+tPos, "NULL"});
        }

        claw.setPosition(1.0);
        rightArm.setPower(0.5);
        leftArm.setPower(0.3);
        waffleLeft.setPosition(0.9);
        waffleRight.setPosition(0.5);
        boolean isHold = false;
        while(opModeIsActive()){
            //get the time step
            //Set the speeds for mechanum wheels
            double[] coeffs = mechanumPower(
                    (1.0-DRIVER_RATIO)*gamepad1.left_stick_x + (gamepad2.dpad_right ? DRIVER_RATIO : (gamepad2.dpad_left ? -DRIVER_RATIO : 0.0)),
                    -(1.0-DRIVER_RATIO)*gamepad1.left_stick_y + (gamepad2.dpad_up ? DRIVER_RATIO : (gamepad2.dpad_down ? -DRIVER_RATIO : 0.0)),
                    -(1.0-DRIVER_RATIO)*gamepad1.right_stick_x + DRIVER_RATIO*(gamepad2.left_trigger - gamepad2.right_trigger));
            mech0.setPower(MAX_SPEED_MECH_WHEEL*coeffs[0]);
            mech1.setPower(MAX_SPEED_MECH_WHEEL*coeffs[1]);
            mech2.setPower(MAX_SPEED_MECH_WHEEL*coeffs[2]);
            mech3.setPower(MAX_SPEED_MECH_WHEEL*coeffs[3]);

            //move arms
            spoolArm.setPower(gamepad2.right_stick_y);
            tPos += -gamepad2.left_stick_y*ARM_SCAN_SPEED;
            int normTPos = (int) tPos;
            leftArm.setTargetPosition(-normTPos);
            rightArm.setTargetPosition(normTPos);

            //move slide
            slide.setPower(gamepad2.x ? SLIDE_MAX_POW : gamepad2.y ? -SLIDE_MAX_POW : 0.0);

            //move claw
            claw.setPosition(gamepad2.a  && !a2_last? (claw.getPosition() > 0.9 ? 0.0 : 1.0) : claw.getPosition());
            //move flipper thingies
            waffleLeft.setPosition(gamepad1.a && !a1_last ? (waffleLeft.getPosition() > 0.99 ? 0.3 : 1.0) : waffleLeft.getPosition());
            waffleRight.setPosition(1.0-waffleLeft.getPosition());

            //update button history
            a1_last = gamepad1.a;
            a2_last = gamepad2.a;

            doTelemetry(new String[]{""+tPos, Boolean.toString(isHold)});
        }
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
        //telemetry.addData("Mech Powers",String.format("0: %.2f, 1: %.2f, 2: %.2f, 3: %.2f",mech0.getPower() ,mech1.getPower() ,mech2.getPower() ,mech3.getPower()));
        telemetry.addData("Arm Velocities", String.format("L: %.2f, R: %.2f, Sp: %.2f", leftArm.getVelocity(AngleUnit.DEGREES), rightArm.getVelocity(AngleUnit.DEGREES), spoolArm.getVelocity(AngleUnit.DEGREES)));
        telemetry.addData("Arm Powers", String.format("L: %f, R %f", leftArm.getPower(), rightArm.getPower() ));
        telemetry.addData("Arm Pos", String.format("L: %d, R %d", leftArm.getCurrentPosition(), rightArm.getCurrentPosition() ));
        telemetry.addData("Target Pos", String.format("L: %d, R %d", leftArm.getTargetPosition(), rightArm.getTargetPosition() ));
        telemetry.addData("Ideal Position", args[0]);
        telemetry.addData("Holding?", args[1]);
        telemetry.addData("PIDF", leftArm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString());
        telemetry.addData("time", time);
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