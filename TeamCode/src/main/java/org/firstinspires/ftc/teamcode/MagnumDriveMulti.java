package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


/* CONTROLS:

Player 1 [DRIVER]:
    Left Stick: move + strafe
    Right Stick x-axis: turn

    A: toggle flippers

 Player 2 [OPERATOR]:
    D-Pad: slowly move + strafe
    (Lower) Triggers: slowly turn

    Left Stick y-axis: rotate the upper arm beam
    Right Stick y-axis: rotate the lower arm beam
    Y/X: move slide out/in, respectively

    (Upper) Shoulders: move wrist
    A: toggle hind grip into/out of position
    B: toggle front grip into/out of position
 */
@TeleOp(name = "Magnum Drive Muti", group = "drive")
public class MagnumDriveMulti extends LinearOpMode {

    //CONSTANTS
    public final double DRIVER_RATIO = 0.3; //ratio of the main driver's control over the wheels to the crane driver's control
    public final double MAX_SPEED_MECH_WHEEL = 720.0; //in degrees per second
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

        waitForStart();

        double preTime = getRuntime();
        double targetWristPos = 1.0;

        boolean a1_last = false;
        boolean a2_last = false;
        boolean b2_last = false;

        backGrip.scaleRange(BACK_GRIP_MIN,BACK_GRIP_MAX);
        frontGrip.scaleRange(FRONT_GRIP_MIN,FRONT_GRIP_MAX);
        wrist.scaleRange(WRIST_GRIP_MIN,WRIST_GRIP_MAX);
        backGrip.setPosition(0.0);
        frontGrip.setPosition(1.0);

        while(opModeIsActive()){
            //get the time step
            double dT = getRuntime() - preTime;

            //Set the speeds for mechanum wheels
            double[] coeffs = mechanumPower(
                    -(1.0-DRIVER_RATIO)*gamepad1.left_stick_x + (gamepad2.dpad_up ? -DRIVER_RATIO : (gamepad2.dpad_down ? DRIVER_RATIO : 0.0)),
                    (1.0-DRIVER_RATIO)*gamepad1.left_stick_y + (gamepad2.dpad_right ? DRIVER_RATIO : (gamepad2.dpad_left ? -DRIVER_RATIO : 0.0)),
                    (1.0-DRIVER_RATIO)*gamepad1.right_stick_x + DRIVER_RATIO*(gamepad2.left_trigger - gamepad2.right_trigger));
            mech0.setVelocity(MAX_SPEED_MECH_WHEEL*coeffs[0], AngleUnit.DEGREES);
            mech1.setVelocity(MAX_SPEED_MECH_WHEEL*coeffs[1], AngleUnit.DEGREES);
            mech2.setVelocity(MAX_SPEED_MECH_WHEEL*coeffs[2], AngleUnit.DEGREES);
            mech3.setVelocity(MAX_SPEED_MECH_WHEEL*coeffs[3], AngleUnit.DEGREES);

            //move arms
            upperArm.setPower(ARM_POW*gamepad2.left_stick_y);
            lowerArm.setPower(ARM_POW*gamepad2.right_stick_y);

            //move slide
            slide.setPower(gamepad2.y ? -SLIDE_SPEED : (gamepad2.x ? SLIDE_SPEED : 0.0));

            //move wrist
            if(gamepad2.right_bumper){
                targetWristPos += WRIST_SPEED*dT;
            }else if(gamepad2.left_bumper){
                targetWristPos -= WRIST_SPEED*dT;
            }
            targetWristPos = targetWristPos < WRIST_GRIP_MIN ? WRIST_GRIP_MIN : targetWristPos > WRIST_GRIP_MAX ? WRIST_GRIP_MAX : targetWristPos; // put targetWristPos between min & max
            wrist.setPosition(targetWristPos);

            //move gripper
            backGrip.setPosition(gamepad2.a && !a2_last ? (backGrip.getPosition() > 0.99 ? 0.0 : 1.0) : backGrip.getPosition());
            frontGrip.setPosition(gamepad2.b && !b2_last ? (frontGrip.getPosition() > 0.99 ? 0.0 : 1.0) : frontGrip.getPosition());

            //move flipper thingies
            waffleLeft.setPosition(gamepad1.a && !a1_last ? (waffleLeft.getPosition() > 0.99 ? 0.0 : 1.0) : waffleLeft.getPosition());
            waffleRight.setPosition(waffleLeft.getPosition());

            //update various stuffs
            preTime = getRuntime();
            a1_last = gamepad1.a;
            a2_last = gamepad2.a;
            b2_last = gamepad2.b;

            //telemetry
            telemetry.addData("Δt", dT);
            telemetry.addData("mech coeffs", String.format("%.2f, %.2f, %.2f, %.2f", coeffs[0], coeffs[1], coeffs[2], coeffs[3]));
            telemetry.addData("wheel speeds [0,1,2,3]", String.format("%.2f, %.2f, %.2f, %.2f", mech0.getVelocity(AngleUnit.DEGREES), mech1.getVelocity(AngleUnit.DEGREES), mech2.getVelocity(AngleUnit.DEGREES), mech3.getVelocity(AngleUnit.DEGREES)));
            telemetry.addData("upArm PIDF", upperArm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString());
            telemetry.addData("lowArm PIDF", lowerArm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString());
            //telemetry.addData("upArm Speed Error", upperArm.getVelocity(AngleUnit.DEGREES)+", "+targUpSpeed+", "+upperArm.getCurrentPosition());
            //telemetry.addData("lowArm Speed Error", lowerArm.getVelocity(AngleUnit.DEGREES)+", "+targDownSpeed+", "+lowerArm.getCurrentPosition());
            telemetry.addData("wristPos", targetWristPos);
            updateTelemetry(telemetry);
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
}