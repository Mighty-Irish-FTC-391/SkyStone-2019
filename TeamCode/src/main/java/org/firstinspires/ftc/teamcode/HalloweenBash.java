package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "Halloween Comp", group = "Competition")
public class HalloweenBash extends LinearOpMode {

    final double sumSineCosineWheelHypo = 2.0/Math.sqrt(2.0); //Sum of the sine and cosine of the triangle formed by the distance of the wheel to the center of the robot
    final double pitchPow = 0.4;
    final int rotSpeed = 1;
    final double servoSpeed = 0.001;

    private int targetArmFront;
    private int targetArmBack;
    DcMotor flmot;
    DcMotor blmot;
    DcMotor frmot;
    DcMotor brmot;

    DcMotor plateM;
    DcMotor pitchM;

    Servo gripFront;
    CRServo gripBack;
    @Override
    public void runOpMode() {
        //INIT START
        //drive motors
        flmot = hardwareMap.dcMotor.get("flmot");
        blmot = hardwareMap.dcMotor.get("blmot");
        brmot = hardwareMap.dcMotor.get("brmot");
        frmot = hardwareMap.dcMotor.get("frmot");

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
        //arm control

        plateM = hardwareMap.dcMotor.get("plate");
        pitchM = hardwareMap.dcMotor.get("pitch");
        gripFront = hardwareMap.servo.get("gripFront");
        gripBack = hardwareMap.crservo.get("gripBack");

        plateM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pitchM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pitchM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //END INIT
        waitForStart();
        //START START
        //END START
        while (opModeIsActive()) {

            if(gamepad1.a){
                pitchM.setPower(pitchPow);
            }else if(gamepad1.b){
                pitchM.setPower(-pitchPow);
            }else{
                pitchM.setPower(0);
            }

            if(gamepad1.x){
                gripFront.setPosition(gripFront.MIN_POSITION);
            }else if(gamepad1.y){
                gripFront.setPosition(gripFront.MAX_POSITION);
            }else{
            }

            if(gamepad1.left_bumper){
                    gripBack.setPower(1.0);
            }else if(gamepad1.right_bumper){
                gripBack.setPower(-1.0);
            }else{
                gripBack.setPower(0.0);
            }

            plateM.setPower((double)(gamepad1.right_trigger-gamepad1.left_trigger));

            double[] pows = mechanumPower(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x); //workaround for now

            flmot.setPower(pows[0]);
            blmot.setPower(pows[1]);
            brmot.setPower(pows[2]);
            frmot.setPower(pows[3]);
            telemetry.addData("servos", gripFront.getPosition() + " frnt; ");
            telemetry.update();
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
