package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.Samples.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Utility.PIDController;
import org.firstinspires.ftc.teamcode.Utility.PinPointLocalizer;

import java.util.Locale;
@Config
public class Drivetrain {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    public PinPointLocalizer localizer;
    ElapsedTime time = new ElapsedTime();

    //drivetrain
    public DcMotor rightFrontDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftBackDrive = null;

    PIDController xController;
    PIDController yController;
    PIDController headingController;
    public boolean targetReached = false;

    Pose2D targetPose;

    public static final double HEADING_KP = 0.9;
    public static final double HEADING_KI = 0.0;
    public static final double HEADING_KD = 0.0;
    public static final double DRIVE_KP = 0.01;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0;//0.0003;
    public static final double DRIVE_MAX_OUT = 0.95;

    public Drivetrain(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {

        xController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_MAX_OUT);
        yController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_MAX_OUT);
        headingController = new PIDController(HEADING_KP, HEADING_KI, HEADING_KD, DRIVE_MAX_OUT);
        localizer = new PinPointLocalizer(myOpMode);
        localizer.init();

        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightBackDrive");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        resetEncoders();
        useEncoders();

        myOpMode.telemetry.addData(">", "Drivetrain Initialized");
    }

    public void resetEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void useEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTargetPose(Pose2D newTarget){
        targetPose = newTarget;
        targetReached = false;
    }

    public double angleWrap(double degrees) {

        while (degrees > 180) {
            degrees -= 360;
        }
        while (degrees < -180) {
            degrees += 360;
        }

        // keep in mind that the result is in degrees
        return degrees;
    }

    public void update(){

        //Use PIDs to calculate motor powers based on error to targets
        double xPower = xController.calculate(targetPose.getX(DistanceUnit.INCH), localizer.getX());
        double yPower = yController.calculate(targetPose.getY(DistanceUnit.INCH), localizer.getY());

        double wrappedAngleError = angleWrap(targetPose.getHeading(AngleUnit.DEGREES) - localizer.getHeading());
        double tPower = headingController.calculate(wrappedAngleError);

        double radianHeading = Math.toRadians(localizer.getHeading());

        //rotate the motor powers based on robot heading
        double xPower_rotated = xPower * Math.cos(-radianHeading) - yPower * Math.sin(-radianHeading);
        double yPower_rotated = xPower * Math.sin(-radianHeading) + yPower * Math.cos(-radianHeading);

        // x, y, theta input mixing to deliver motor powers
        leftFrontDrive.setPower(xPower_rotated - yPower_rotated - tPower);
        leftBackDrive.setPower(xPower_rotated + yPower_rotated - tPower);
        rightFrontDrive.setPower(xPower_rotated + yPower_rotated + tPower);
        rightBackDrive.setPower(xPower_rotated - yPower_rotated + tPower);

        //check if drivetrain is still working towards target
        targetReached = (xController.targetReached && yController.targetReached && headingController.targetReached);
        String data = String.format(Locale.US, "{tX: %.3f, tY: %.3f, tH: %.3f}", targetPose.getX(DistanceUnit.INCH), targetPose.getY(DistanceUnit.INCH), targetPose.getHeading(AngleUnit.DEGREES));

        myOpMode.telemetry.addData("Target Position", data);
        myOpMode.telemetry.addData("XReached", xController.targetReached);
        myOpMode.telemetry.addData("YReached", yController.targetReached);
        myOpMode.telemetry.addData("HReached", headingController.targetReached);
        myOpMode.telemetry.addData("targetReached", targetReached);
        myOpMode.telemetry.addData("xPower", xPower);
        myOpMode.telemetry.addData("xPowerRotated", xPower_rotated);
        localizer.update();
    }

    public void teleOp() {
        //drive train
        double max;

        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;

        double drive = -myOpMode.gamepad1.left_stick_y;
        double turn = myOpMode.gamepad1.right_stick_x;
        double strafe = -myOpMode.gamepad1.left_stick_x;

        leftFrontPower = (drive + turn - strafe);
        rightFrontPower = (drive - turn + strafe);
        leftBackPower = (drive + turn + strafe);
        rightBackPower = (drive - turn - strafe);

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        //Slow and Turbo Buttons
        if (myOpMode.gamepad1.right_bumper) {
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
        } else if (myOpMode.gamepad1.left_bumper) {
            leftFrontDrive.setPower(leftFrontPower / 7);
            rightFrontDrive.setPower(rightFrontPower / 7);
            leftBackDrive.setPower(leftBackPower / 7);
            rightBackDrive.setPower(rightBackPower / 7);
        } else {
            leftFrontDrive.setPower(leftFrontPower / 2);
            rightFrontDrive.setPower(rightFrontPower / 2);
            leftBackDrive.setPower(leftBackPower / 2);
            rightBackDrive.setPower(rightBackPower / 2);
        }

    }
/*
    public void driveToPose(double xTarget, double yTarget, double degreeTarget) {
        //double thetaTarget = Math.toRadians(degreeTarget);
        //Use PIDs to calculate motor powers based on error to targets
        double xPower = xPID.calculate(xTarget, localizer.x);
        double yPower = yPID.calculate(yTarget, localizer.y);

        double wrappedAngle = angleWrap(thetaTarget - localizer.heading);
        double tPower = headingPID.calculate(wrappedAngle);

        //rotate the motor powers based on robot heading
        double xPower_rotated = xPower * Math.cos(-localizer.heading) - yPower * Math.sin(-localizer.heading);
        double yPower_rotated = xPower * Math.sin(-localizer.heading) + yPower * Math.cos(-localizer.heading);

        // x, y, theta input mixing
        leftFrontDrive.setPower(xPower_rotated - yPower_rotated - tPower);
        leftBackDrive.setPower(xPower_rotated + yPower_rotated - tPower);
        rightFrontDrive.setPower(xPower_rotated + yPower_rotated + tPower);
        rightBackDrive.setPower(xPower_rotated - yPower_rotated + tPower);

        localizer.update();
        localizer.updateDashboard();
        localizer.telemetry();
        myOpMode.telemetry.update();
    }

 */
}
