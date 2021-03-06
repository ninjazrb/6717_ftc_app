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
public class TeleOpGyro extends BasicOpMode_Iterative {

    // Motor Declarations
    private DcMotor left_f;
    private DcMotor right_f;
    private DcMotor left_b;
    private DcMotor right_b;
    private DcMotor arm;
    private DcMotor collection;
    private Servo gripper;
    private Servo hammer;

    // Sound Variables
    private SoundPool mySound;
    private int lionID;
    private int alanID;
    private int hornID;
    private int indianaID;

    // Sensor Declarations
    private GyroSensor gyro;

    //Pid Declarations


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
        collection = hardwareMap.dcMotor.get("collection");
        hammer = hardwareMap.servo.get("hammer");

        // Map sensors
        gyro = hardwareMap.gyroSensor.get("gyro");

        // Run with encoders
        left_b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_f.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_f.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        right_b.setDirection(DcMotor.Direction.REVERSE);
        right_f.setDirection(DcMotor.Direction.REVERSE);

        gripper.setPosition(1);

        gyro.calibrate();

        // Map sounds
        mySound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0); // PSM
        lionID = mySound.load(hardwareMap.appContext, R.raw.lion, 1); // PSM
        alanID = mySound.load(hardwareMap.appContext, R.raw.alan, 1); // PSM
        hornID = mySound.load(hardwareMap.appContext, R.raw.horn, 1); // PSM
        indianaID = mySound.load(hardwareMap.appContext, R.raw.indiana, 1);
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
        hammer.setPosition(.15);
    }

    private double gyroRadians(double intHeading) {
        return Math.toRadians(intHeading);
    }

    private double leftJoyAngle() {
        return Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
    }

    private void setDrivePower(Double angleOfTravel, double speed, double skew) {
        double RobotAngle = angleOfTravel - Math.PI/4;

        double lf = -Math.sin(RobotAngle);
        double rb = -Math.sin(RobotAngle);
        double rf = -Math.cos(RobotAngle);
        double lb = -Math.cos(RobotAngle);

        /*double powers[] = driveSmoothing(lf, rb, rf, lb);*/

        lf = Range.clip(lf * (speed*.5) + (skew*.5), -.5, .5);
        rb = Range.clip(rb * (speed*.5) - (skew*.5), -.5, .5);
        rf = Range.clip(rf * (speed*.5) - (skew*.5), -.5, .5);
        lb = Range.clip(lb * (speed*.5) + (skew*.5), -.5, .5);

        /*telemetry.addData("Left Front/Right Front: ", two_decimal.format(lf) + "/" + two_decimal.format(rf));
        telemetry.addData("Left Back/Right Back: ", two_decimal.format(lb) + "/" + two_decimal.format(rb));*/
        telemetry.addData("lf/rf: ", left_f.getCurrentPosition() + right_f.getCurrentPosition());
        telemetry.update();

        left_b.setPower(lb);
        left_f.setPower(lf);
        right_f.setPower(rf);
        right_b.setPower(rb);
    }

    private void armPower() {
        if(gamepad1.left_bumper) {
            arm.setPower(-.1);
        }
        else if(gamepad1.right_bumper) {
            arm.setPower(1);
        }
        else {
            arm.setPower(0);
        }
    }

    private double[] driveSmoothing(double lf, double rb, double rf, double lb) {
        double final_lf;
        double final_rb;
        double final_rf;
        double final_lb;

        double math_lf = Math.abs(lf);
        double math_rb = Math.abs(lf);
        double math_rf = Math.abs(lf);
        double math_lb = Math.abs(lf);

        if(math_lf >= math_rb && math_lf >= math_rf && math_lf >= math_lb) {
            final_lf = .5 * (lf/math_lf);
            final_rb = final_lf * (rb/lf);
            final_rf = final_lf * (rf/lf);
            final_lb = final_lf * (lb/lf);
        }

        else if(math_rb >= math_lf && math_rb >= math_rf && math_rb >= math_lb) {
            final_rb = .5 * (rb/math_rb);
            final_lf = final_rb * (lf/rb);
            final_rf = final_rb * (rf/rb);
            final_lb = final_rb * (lb/rb);
        }

        else if(math_rf >= math_rb && math_rf >= math_lf && math_rf >= math_lb) {
            final_rf = .5 * (rf/math_rf);
            final_rb = final_rf * (rb/rf);
            final_lf = final_rf * (lf/rf);
            final_lb = final_rf * (lb/rf);
        }

        else if(math_lb >= math_rb && math_lb >= math_rf && math_lb >= math_lf) {
            final_lb = .5 * (lb/math_lb);
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

    private void gripperPower() {
        if (gamepad1.y) {
            telemetry.update();
            gripper.setPosition(.5);
        }

        if (gamepad1.b) {
            gripper.setPosition(1);
        }

        if (gamepad1.a) {
            gripper.setPosition(.7);
        }

        if (gamepad1.left_trigger > 0) {
            collection.setPower(-1);
        }

        if(gamepad1.right_trigger > 0) {
            collection.setPower(1);
        }
        if (gamepad1.x) {
            collection.setPower(0);
        }
    }

    private void sounds() {
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

}

