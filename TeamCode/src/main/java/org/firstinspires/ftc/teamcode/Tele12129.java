package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;

import java.text.DecimalFormat;


/**
 * Created by Zach on 10/6/2017.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class Tele12129 extends BasicOpMode_Iterative {

    DcMotor left_drive;
    DcMotor right_drive;
    DcMotor left_arm;

    @Override
    public void init() {
        left_drive = hardwareMap.dcMotor.get("left_drive");
        right_drive = hardwareMap.dcMotor.get("right_drive");
        left_arm = hardwareMap.dcMotor.get("left_arm");
    }

    @Override
    public void loop() {
        left_drive.setPower(gamepad1.left_stick_y);
        right_drive.setPower(gamepad1.right_stick_y);

        if(gamepad1.left_bumper) {
            left_arm.setPower(-1);
        }
        else if (gamepad1.right_bumper) {
            left_arm.setPower(1);
        }
        else {
            left_arm.setPower(0);
        }
    }

}
