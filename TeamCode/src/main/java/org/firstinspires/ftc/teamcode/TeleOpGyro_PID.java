package org.firstinspires.ftc.teamcode;

import android.media.AudioManager;
import android.media.SoundPool;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;

import java.text.DecimalFormat;


/**
 * Created by Zach on 10/6/2017.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpGyro_PID extends BasicOpMode_Iterative {

    double maxWheelSpeed = .5;

    // Motor Declarations
    DcMotor left_f;
    DcMotor right_f;
    DcMotor left_b;
    DcMotor right_b;
    DcMotor arm;
    DcMotor collection_left;
    DcMotor collection_right;
    Servo gripper;

    // Sound Variables
    public SoundPool mySound;
    public int lionID;
    public int alanID;
    public int hornID;
    public int indianaID;

    // Sensor Declarations
    GyroSensor gyro;

    //Pid Variables
    private final double drivePidKp = 1;    // Tuning variable for PID.
    private final double drivePidTi = 1.0;  // Eliminate integral error in 1 sec.
    private final double drivePidTd = 0.1;  // Account for error in 0.1 sec.
    // Protect against integral windup by limiting integral term.
    private final double drivePidIntMax = maxWheelSpeed;  // Limit to max speed.

    //Misc Pid
    private final double ticksPerRevolution = 1000;  // Get for your motor and gearing.
    private double prevTime;  // The last time loop() was called.
    private int prevLfEncoderPosition;   // Encoder tick at last call to loop().
    private int prevRfEncoderPosition;  // Encoder tick at last call to loop().
    private int prevLbEncoderPosition;  // Encoder tick at last call to loop().
    private int prevRbEncoderPosition;  // Encoder tick at last call to loop().
    private Pid lfPid = null;
    private Pid rfPid = null;
    private Pid lbPid = null;
    private Pid rbPid = null;

    DecimalFormat two_decimal = new DecimalFormat("#.##");

    @Override
    public void init() {
        // Map motors
        left_b = hardwareMap.dcMotor.get("left_b");
        left_f = hardwareMap.dcMotor.get("left_f");
        right_b = hardwareMap.dcMotor.get("right_b");
        right_f = hardwareMap.dcMotor.get("right_f");
        arm = hardwareMap.dcMotor.get("arm");
        gripper = hardwareMap.servo.get("gripper");
        /*collection_left = hardwareMap.dcMotor.get("collection_left");
        collection_right = hardwareMap.dcMotor.get("collection_right");*/

        // Map sensors
        gyro = hardwareMap.gyroSensor.get("gyro");

        // Run with encoders
        left_b.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_f.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_b.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_f.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_b.setDirection(DcMotor.Direction.REVERSE);
        right_f.setDirection(DcMotor.Direction.REVERSE);
        /*collection_left.setDirection(DcMotor.Direction.REVERSE);*/

        gripper.setPosition(1);

        gyro.calibrate();

        // Map sounds
        mySound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0); // PSM
        lionID = mySound.load(hardwareMap.appContext, R.raw.lion, 1); // PSM
        alanID = mySound.load(hardwareMap.appContext, R.raw.alan, 1); // PSM
        hornID = mySound.load(hardwareMap.appContext, R.raw.horn, 1); // PSM
        indianaID = mySound.load(hardwareMap.appContext, R.raw.indiana, 1);

        //Pid
        prevTime = 0;
        prevLfEncoderPosition = left_f.getCurrentPosition();
        prevRfEncoderPosition = right_f.getCurrentPosition();
        prevLbEncoderPosition = left_b.getCurrentPosition();
        prevRbEncoderPosition = right_b.getCurrentPosition();

        lfPid = new Pid(drivePidKp, drivePidTi, drivePidTd,
                -drivePidIntMax, drivePidIntMax);
        rfPid = new Pid(drivePidKp, drivePidTi, drivePidTd,
                -drivePidIntMax, drivePidIntMax);
        lbPid = new Pid(drivePidKp, drivePidTi, drivePidTd,
                -drivePidIntMax, drivePidIntMax);
        rbPid = new Pid(drivePidKp, drivePidTi, drivePidTd,
                -drivePidIntMax, drivePidIntMax);
    }

    @Override
    public void loop() {
        double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double skew = gamepad1.right_stick_x;
        double gyroHeading = gyroRadians(gyro.getHeading());

        setDrivePower(leftJoyAngle() + gyroHeading, speed, skew);
        armPower();
        gripperPower();
        sounds();
    }

    private double gyroRadians(double intHeading) {
        return Math.toRadians(intHeading);
    }

    public double leftJoyAngle() {
        return Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
    }

    public void setDrivePower(Double angleOfTravel, double speed, double skew) {
        double RobotAngle = angleOfTravel - Math.PI/4;

        double lf = -Math.sin(RobotAngle);
        double rb = -Math.sin(RobotAngle);
        double rf = -Math.cos(RobotAngle);
        double lb = -Math.cos(RobotAngle);

        double powers[] = driveSmoothing(lf, rb, rf, lb);

        lf = Range.clip(powers[0] * speed - skew, -maxWheelSpeed, maxWheelSpeed);
        rb = Range.clip(powers[1] * speed + skew, -maxWheelSpeed , maxWheelSpeed);
        rf = Range.clip(powers[2] * speed + skew, -maxWheelSpeed, maxWheelSpeed);
        lb = Range.clip(powers[3] * speed - skew, -maxWheelSpeed, maxWheelSpeed);

        /*telemetry.addData("Left Front/Right Front: ", two_decimal.format(lf) + "/" + two_decimal.format(rf));
        telemetry.addData("Left Back/Right Back: ", two_decimal.format(lb) + "/" + two_decimal.format(rb));*/
        telemetry.addData("lf/rf: ", left_f.getCurrentPosition() + right_f.getCurrentPosition());
        telemetry.update();

        double[] speeds = pid();
        double lfSpeed = speeds[0];
        double rfSpeed = speeds[1];
        double lbSpeed = speeds[2];
        double rbSpeed = speeds[3];
        double deltaTime = speeds[4];
        lf = lfPid.update(lf, lfSpeed, deltaTime);
        rf = rfPid.update(lf, lfSpeed, deltaTime);
        lb = lbPid.update(lf, lfSpeed, deltaTime);
        rb = rbPid.update(lf, lfSpeed, deltaTime);

        left_b.setPower(lb);
        left_f.setPower(lf);
        right_f.setPower(rf);
        right_b.setPower(rb);
    }

    public void armPower() {
        if(gamepad1.left_bumper) {
            arm.setPower(-1);
        }
        else if(gamepad1.right_bumper) {
            arm.setPower(1);
        }
        else {
            arm.setPower(0);
        }
    }

    public double[] driveSmoothing(double lf, double rb, double rf, double lb) {
        double final_lf;
        double final_rb;
        double final_rf;
        double final_lb;

        double math_lf = Math.abs(lf);
        double math_rb = Math.abs(lf);
        double math_rf = Math.abs(lf);
        double math_lb = Math.abs(lf);

        if(math_lf >= math_rb && math_lf >= math_rf && math_lf >= math_lb) {
            final_lf = maxWheelSpeed * (lf/math_lf);
            final_rb = final_lf * (rb/lf);
            final_rf = final_lf * (rf/lf);
            final_lb = final_lf * (lb/lf);
        }

        else if(math_rb >= math_lf && math_rb >= math_rf && math_rb >= math_lb) {
            final_rb = maxWheelSpeed * (rb/math_rb);
            final_lf = final_rb * (lf/rb);
            final_rf = final_rb * (rf/rb);
            final_lb = final_rb * (lb/rb);
        }

        else if(math_rf >= math_rb && math_rf >= math_lf && math_rf >= math_lb) {
            final_rf = maxWheelSpeed * (rf/math_rf);
            final_rb = final_rf * (rb/rf);
            final_lf = final_rf * (lf/rf);
            final_lb = final_rf * (lb/rf);
        }

        else if(math_lb >= math_rb && math_lb >= math_rf && math_lb >= math_lf) {
            final_lb = maxWheelSpeed * (lb/math_lb);
            final_rb = final_lb * (rb/lb);
            final_rf = final_lb * (rf/lb);
            final_lf = final_lb * (lf/lb);
        }

        else {
            final_lf = lf;
            final_rb = rb;
            final_rf = rf;
            final_lb = lb;
        }

        double[] powers = new double[4];

        powers[0] = final_lf;
        powers[1] = final_rb;
        powers[2] = final_rf;
        powers[3] = final_lb;

        int i = 0;

        while(i < 4) {
            if(Double.isNaN(powers[i])) {
                powers[i] = 0;
            }

            i++;
        }

        return powers;
    }

    public void gripperPower() {
        if (gamepad1.a) {
            telemetry.update();
            gripper.setPosition(.5);
        }

        if (gamepad1.b) {
            gripper.setPosition(1);
        }

        /*if (gamepad1.x) {
            collection_left.setPower(1);
            collection_right.setPower(1);
        }

        if(gamepad1.y) {
            collection_left.setPower(-1);
            collection_right.setPower(-1);
        }*/
    }

    public void sounds() {
        if(gamepad2.x) {
            mySound.play(lionID, 1, 1, 1, 0, 1);
        }
        if(gamepad2.y) {
            mySound.play(alanID, 1, 1, 1, 0, 1);
        }
        if(gamepad2.b) {
            mySound.play(hornID, 1, 1, 1, 0, 1);
        }
        if(gamepad2.a) {
            mySound.play(indianaID, 1, 1, 1, 0, 1);
        }
    }

    public double[] pid() {
        double[] speeds = new double[5];

        double deltaTime = time - prevTime;
        double lfSpeed = (left_f.getCurrentPosition() - prevLfEncoderPosition) /
                deltaTime;
        double rfSpeed = (right_f.getCurrentPosition() - prevRfEncoderPosition) /
                deltaTime;
        double lbSpeed = (left_b.getCurrentPosition() - prevLbEncoderPosition) /
                deltaTime;
        double rbSpeed = (right_b.getCurrentPosition() - prevRbEncoderPosition) /
                deltaTime;

        prevTime = time;
        prevLfEncoderPosition = left_f.getCurrentPosition();
        prevRfEncoderPosition = right_f.getCurrentPosition();
        prevLbEncoderPosition = left_b.getCurrentPosition();
        prevRbEncoderPosition = right_b.getCurrentPosition();

        speeds[0] = lfSpeed;
        speeds[1] = rfSpeed;
        speeds[2] = lbSpeed;
        speeds[3] = rbSpeed;
        speeds[4] = deltaTime;

        return speeds;
    }

}
