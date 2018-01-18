package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Zach on 11/14/2017.
 */

@Autonomous
public class Blue_jewel extends LinearOpMode{
    // Motor Declarations
    DcMotor left_f;
    DcMotor right_f;
    DcMotor left_b;
    DcMotor right_b;
    Servo gripper;
    Servo hammer;

    // Sensor Declarations
    ColorSensor color;

    public void runOpMode() throws InterruptedException {

        left_b = hardwareMap.dcMotor.get("left_b");
        left_f = hardwareMap.dcMotor.get("left_f");
        right_b = hardwareMap.dcMotor.get("right_b");
        right_f = hardwareMap.dcMotor.get("right_f");
        gripper = hardwareMap.servo.get("gripper");
        hammer = hardwareMap.servo.get("hammer"); // Drops left

        color = hardwareMap.colorSensor.get("color");

        left_b.setDirection(DcMotor.Direction.REVERSE);
        left_f.setDirection(DcMotor.Direction.REVERSE);

        gripper.setPosition(1.0);
        hammer.setPosition(0.1);

        waitForStart();

        // Drop the hammer
        hammer.setPosition(0.17);
        Thread.sleep(5000);

        // If the sensor sees red
        if(color.red() > color.green() && color.red() > color.blue()) {
            // Rotate towards (Clockwise)
            setDrivePower(0.0, 1.0, 1.0);
            Thread.sleep(500);
            setDrivePower(0.0, 0.0, 0.0);
        }

        // If the sensor sees blue
        if(color.blue() > color.green() && color.blue() > color.red()) {
            // Rotate away (Counter-clockwise)
            setDrivePower(0.0, 1.0, -1.0);
            Thread.sleep(500);
            setDrivePower(0.0, 0.0, 0.0);
        }
    }

    public void setDrivePower(Double angleOfTravel, double speed, double skew) {
        double RobotAngle = angleOfTravel - Math.PI/4;

        double lf = Range.clip(Math.sin(RobotAngle) * speed - skew, -1, 1);
        double rb = Range.clip(Math.sin(RobotAngle) * speed + skew, -1, 1);
        double rf = Range.clip(Math.cos(RobotAngle) * speed + skew, -1, 1);
        double lb = Range.clip(Math.cos(RobotAngle) * speed - skew, -1, 1);

        /*double powers[] = driveSmoothing(lf, rb, rf, lb);

        lf = powers[0];
        rb = powers[1];
        rf = powers[2];
        lb = powers[3];*/

        /*telemetry.addData("Left Front/Right Front: ", two_decimal.format(lf) + "/" + two_decimal.format(rf));
        telemetry.addData("Left Back/Right Back: ", two_decimal.format(lb) + "/" + two_decimal.format(rb));
        telemetry.update();*/

        left_b.setPower(lb);
        left_f.setPower(lf);
        right_f.setPower(rf);
        right_b.setPower(rb);
    }
}
