package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.text.DecimalFormat;

/**
 * Created by Zach on 10/17/2017.
 */

@Autonomous
public class Vuforia_test extends LinearOpMode{


    // Motor Declarations
    DcMotor left_f;
    DcMotor right_f;
    DcMotor left_b;
    DcMotor right_b;

    VuforiaLocalizer vuforiaLocalizer;
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables visionTargets;
    VuforiaTrackable target;
    VuforiaTrackableDefaultListener listener;

    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;
    OpenGLMatrix target1Location = createMatrix(0,1000,0,90,0,90);
    OpenGLMatrix target2Location = createMatrix(768,2672,0,0,0,0);

    public static final String VUFORIA_KEY = "AW82ZQb/////AAAAGeoGziaiP05rmQjuI3ITJjRTo58whDfkKhapEDIl1Vx1LdJjR87ORdE+2s5QsFBiH6sk2S0V4K9cmHH65Uo9F4UPxN8lGx+aeZMQ+BuebJvNOGDP09xfn1qMlDl7P2EQyJPaxZeZ7ifIdz9BCQKKRnKiHolFEYvhPHtoT0gjQvL/Vc1XLrP6dLOJpnYibyQQ3SKptEMafHUBEx7UJJjBHLR9P+ZvT+IDRg2nxtkp5LUCFrFJMcscWZP6ovEEjJzCGO28bSJqMYem3H+GutxiNhsLu7GhLCvH8TuCTOslhi3w0Lrm1weu74vi1b+90noRZ/vrrAJ/etyjY4Ya1a/Ezc3CRGoNAefshotCKUDaQBu8";


    float robotx;
    float roboty;
    float robotAngle;
    double angleToTarget;

    DecimalFormat two_decimal = new DecimalFormat("#.##");

    public void runOpMode() throws InterruptedException {

        setupVuforia();
        lastKnownLocation = createMatrix(0,0,0,0,0,0);

        left_b = hardwareMap.dcMotor.get("left_b");
        left_f = hardwareMap.dcMotor.get("left_f");
        right_b = hardwareMap.dcMotor.get("right_b");
        right_f = hardwareMap.dcMotor.get("right_f");

        left_b.setDirection(DcMotor.Direction.REVERSE);
        left_f.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        visionTargets.activate();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(target);

        while(opModeIsActive()) {

            OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

            if(latestLocation != null)
                lastKnownLocation = latestLocation;

            float coordinates[] = lastKnownLocation.getTranslation().getData();

            robotx = coordinates[0];
            roboty = coordinates[1];
            robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            vuMark = RelicRecoveryVuMark.from(target);

            angleToTarget = angleToTargetLocation(lastKnownLocation, target1Location);

            if(vuMark.toString() != "UNKNOWN"){
                setDrivePower(angleToTarget, 1, 0);
            }
            else {
                setDrivePower(angleToTarget, 0, 0);
            }

            telemetry.addData("Visible: ", vuMark);
            telemetry.addData("Robot Location", "x:" + robotx + " y: " + roboty + " angle: " + robotAngle);
            telemetry.update();
        }
    }

    public void setupVuforia() {
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("RelicVuMark");

        target = visionTargets.get(0);
        target.setName("Left_Key");
        target.setLocation(createMatrix(0,1000,0,0,0,0));

        phoneLocation = createMatrix(0,0,0,90,0,90);

        listener = (VuforiaTrackableDefaultListener) target.getListener();
        listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
    }

    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w) {
        return OpenGLMatrix.translation(x,y,z).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC,
                AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    public String formatMatrix(OpenGLMatrix matrix){

        return matrix.formatAsTransform();
    }

    public double angleToTargetLocation(OpenGLMatrix robot, OpenGLMatrix location) {
        float robotCoordinates[] = robot.getTranslation().getData();
        float locationCoordinates[] = location.getTranslation().getData();

        double robotCoordinatesX = robotCoordinates[0];
        double robotCoordinatesY = robotCoordinates[1];
        double robotHeading = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle;

        double locationCoordinatesX = locationCoordinates[0];
        double locationCoordinatesY = locationCoordinates[1];

        double angle = Math.atan2(locationCoordinatesY-robotCoordinatesY, locationCoordinatesX-robotCoordinatesX);
        angle = angle - robotHeading;
        /*while(angle < 0) {
            angle = angle + 360;
        }
        while(angle > 360) {
            angle = angle - 360;
        }*/

        return angle;
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

        left_b.setPower(lb);
        left_f.setPower(lf);
        right_f.setPower(rf);
        right_b.setPower(rb);
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
            final_lf = 1 * (lf/math_lf);
            final_rb = final_lf * (rb/lf);
            final_rf = final_lf * (rf/lf);
            final_lb = final_lf * (lb/lf);
        }

        else if(math_rb >= math_lf && math_rb >= math_rf && math_rb >= math_lb) {
            final_rb = 1 * (rb/math_rb);
            final_lf = final_rb * (lf/rb);
            final_rf = final_rb * (rf/rb);
            final_lb = final_rb * (lb/rb);
        }

        else if(math_rf >= math_rb && math_rf >= math_lf && math_rf >= math_lb) {
            final_rf = 1 * (rf/math_rf);
            final_rb = final_rf * (rb/rf);
            final_lf = final_rf * (lf/rf);
            final_lb = final_rf * (lb/rf);
        }

        else if(math_lb >= math_rb && math_lb >= math_rf && math_lb >= math_lf) {
            final_lb = 1 * (lb/math_lb);
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
}
