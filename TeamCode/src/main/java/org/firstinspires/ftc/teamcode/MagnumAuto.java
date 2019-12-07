package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="Magnum Auto")
public class MagnumAuto extends LinearOpMode {

    //CONSTANTS
    public final int BLUE_MIN = 35;//Change at comp

    public final int RED_MIN = 60;//Change at comp
//9in
    public final double WHEEL_RADIUS = 2.0;

    public final double MAX_SPEED_MECH_WHEEL = 360.0; //in degrees per second
    public final double[][] PIDF_MECH = null; //PIDF constants for the mechanum wheels
    public final double SLIDE_SPEED = 1.0; //slide servo speed, in ¯\_(ツ)_/¯
    public final double WRIST_SPEED = 3.5; //wrist rotation speed, in semi-circles per second
    public final double WRIST_GRIP_MIN = 0.0; ////farthest Back position for the Wrist in semi-circles
    public final double WRIST_GRIP_MAX = 1.0; ////farthest Back position for the Wrist in semi-circles
    public final double BACK_GRIP_MIN = 0.4; //farthest Back position for the Back Grip in semi-circles
    public final double BACK_GRIP_MAX = 1.0; //farthest Back position for the Back Grip in semi-circles
    public final double FRONT_GRIP_MIN = 0.0; //farthest Forward position for the Front Grip in semi-circles
    public final double FRONT_GRIP_MAX = 1.0; //farthest Forward position for the Front Grip in semi-circles
    public final double ARM_POW = 0.5;//both arm motor speed, in degrees per second

    //motors for mechanum drive go counterclockwise from the bottom right. Should be marked on robot.
    DcMotorEx mech0;
    DcMotorEx mech1;
    DcMotorEx mech2;
    DcMotorEx mech3;

    //Motors for the main arm
    DcMotorEx lowerArm;
    DcMotorEx upperArm;

    //Servos for the main arm & pincer
    CRServo slide;
    Servo wrist;
    Servo backGrip;
    Servo frontGrip;

    //Servos for grabbing the base plate
    Servo waffleLeft;
    Servo waffleRight;

    //Sensors
    ColorSensor cs;
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
        lowerArm = (DcMotorEx) hardwareMap.dcMotor.get("lowerArm");
        upperArm = (DcMotorEx) hardwareMap.dcMotor.get("upperArm");

        mech2.setDirection(DcMotorSimple.Direction.FORWARD);
        mech3.setDirection(DcMotorSimple.Direction.FORWARD);

        lowerArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upperArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servos for the main arm & pincer
        slide = hardwareMap.crservo.get("slide");
        wrist = hardwareMap.servo.get("wrist");
        backGrip = hardwareMap.servo.get("backGrip");
        frontGrip = hardwareMap.servo.get("frontGrip");

        //Servos for grabbing the base plate
        waffleLeft = hardwareMap.servo.get("waffleLeft");
        waffleRight = hardwareMap.servo.get("waffleRight");

        //Sensors
        cs = hardwareMap.colorSensor.get("color");

        waitForStart();

        backGrip.scaleRange(BACK_GRIP_MIN,BACK_GRIP_MAX);
        frontGrip.scaleRange(FRONT_GRIP_MIN,FRONT_GRIP_MAX);
        wrist.scaleRange(WRIST_GRIP_MIN,WRIST_GRIP_MAX);
        backGrip.setPosition(0.0);
        frontGrip.setPosition(1.0);

        boolean stop = false;
        double startTime = time;
        while(opModeIsActive()){
            if(!stop){
                if(cs.blue() >= BLUE_MIN || cs.red() >= RED_MIN) { //keep moving until under bridge
                    velocity(0.0,0.0,0.0);
                    stop = true;
                }else{
                    velocity(-1.0,0.0,0.0);
                }
            }
            telemetry.addData("found bridge?", stop);
            telemetry.addLine("==========================");
            telemetry.addData("red", cs.red());
            telemetry.addData("blue", cs.blue());
            telemetry.addData("green", cs.green());
            telemetry.update();
        }
    }

    public double[] mechanumPower(double x, double y, double rot) {
        //Starting from the front left (0), counterclockwise (1,2,3), ratios for motor powers in range of [-1,1]

        double[] coeffs = new double[4];
        coeffs[0] = y + x - rot;
        coeffs[1] = y - x - rot;
        coeffs[2] = y + x + rot;
        coeffs[3] = y - x + rot;

        /*double largest = Math.max(Math.max(Math.abs(coeffs[0]),Math.abs(coeffs[1])),Math.max(Math.abs(coeffs[2]),Math.abs(coeffs[3])));
        if(largest > 1.0d) {
            coeffs[0] = coeffs[0]/largest;
            coeffs[1] = coeffs[1]/largest;
            coeffs[2] = coeffs[2]/largest;
            coeffs[3] = coeffs[3]/largest;
        }*/
        return coeffs;
    }

    public void velocity(double forward, double strafe, double rot){
        double[] coeffs = mechanumPower(strafe, forward, rot);
        mech0.setVelocity(MAX_SPEED_MECH_WHEEL*coeffs[0], AngleUnit.DEGREES);
        mech1.setVelocity(MAX_SPEED_MECH_WHEEL*coeffs[1], AngleUnit.DEGREES);
        mech2.setVelocity(MAX_SPEED_MECH_WHEEL*coeffs[2], AngleUnit.DEGREES);
        mech3.setVelocity(MAX_SPEED_MECH_WHEEL*coeffs[3], AngleUnit.DEGREES);

    }

    public void lineMove(double dist){
        double startTime = time;
        double dT = (dist/WHEEL_RADIUS) / MAX_SPEED_MECH_WHEEL;
        double sign = Math.signum(dist);
        while(opModeIsActive() && (startTime + dT) < time){
            velocity(sign*1.0,0.0,0.0);
        }
    }
}
