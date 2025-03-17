package org.firstinspires.ftc.teamcode.Utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;

import java.util.Locale;
@Config
public class TuningDrivetrain {
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

    MotionProfileRedux motionProfile;

    Pose2D targetPose;
    public boolean targetReached = false;

    public static double HEADING_KP = 0.015;
    public static double HEADING_KI = 0.0;
    public static double HEADING_KD = 0.0;
    public static double DRIVE_KP = 0.03;
    public static double DRIVE_KI = 0.0;
    public static double DRIVE_KD = 0;//0.0003;
    public static double DRIVE_MAX_OUT = 0.7;//0.0003;
    public static double STRAFE_MULTIPLIER = 2;

    //feedforward constant for deceleration
    public static double DRIVE_KF = 0.0;

    public static double DRIVE_MAX_ACC = 52;
    public static double DRIVE_MAX_VEL = 52;
    public static double HEADING_MAX_ACC = 100;
    public static double HEADING_MAX_VEL = 100;

    double instantX = 0;
    double instantY = 0;

    //if the subsystem has explicit states, it can be helpful to use an enum to define them
    public enum DrivetrainMode {
        MANUAL,
        AUTO,
        PROFILE,
        IDLE,
    }

    public TuningDrivetrain.DrivetrainMode drivetrainMode = TuningDrivetrain.DrivetrainMode.MANUAL;

    public TuningDrivetrain(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {

        xController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_MAX_OUT);
        yController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_MAX_OUT);
        headingController = new PIDController(HEADING_KP, HEADING_KI, HEADING_KD, DRIVE_MAX_OUT);

        motionProfile = new MotionProfileRedux(0,0,0,0,DRIVE_MAX_VEL,DRIVE_MAX_ACC);

        targetPose = new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.DEGREES,0);
        targetReached = true;

        localizer = new PinPointLocalizer(myOpMode);
        localizer.init();
        localizer.update();

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
        runWithoutEncoders();

        myOpMode.telemetry.addData(">", "Drivetrain Initialized");
    }

    public void resetEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runWithoutEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void profiledDriveToTarget(double xTarget, double yTarget, double degreeTarget) {
        //check if the target pose is new
        if(targetPose.getX(DistanceUnit.INCH) != xTarget ||
                targetPose.getY(DistanceUnit.INCH) != yTarget ||
                targetPose.getHeading(AngleUnit.DEGREES) != degreeTarget){

            drivetrainMode = DrivetrainMode.PROFILE;

            //if target is new, calculate motion profile time and reset timer, and store original distance
            targetPose = new Pose2D(DistanceUnit.INCH, xTarget,yTarget,AngleUnit.DEGREES,degreeTarget);
            targetReached = false;
            //TODO consider creating profile from last target position, in case robot does not make it to target
            motionProfile = new MotionProfileRedux(localizer.getX(), localizer.getY(), xTarget,yTarget,DRIVE_MAX_VEL,DRIVE_MAX_ACC);
        }

    }

    public void setTargetPose(Pose2D newTarget){
        targetPose = newTarget;
        targetReached = false;
    }

    public void driveToTarget(double xTarget, double yTarget, double degreeTarget){
        if(targetPose.getX(DistanceUnit.INCH) != xTarget ||
                targetPose.getY(DistanceUnit.INCH) != yTarget ||
                targetPose.getHeading(AngleUnit.DEGREES) != degreeTarget) {

            drivetrainMode = DrivetrainMode.AUTO;

            targetPose = new Pose2D(DistanceUnit.INCH, xTarget, yTarget, AngleUnit.DEGREES, degreeTarget);
            targetReached = false;
        }
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
        localizer.update();

        xController.setGains(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_MAX_OUT);
        yController.setGains(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_MAX_OUT);
        headingController.setGains(HEADING_KP, HEADING_KI, HEADING_KD, DRIVE_MAX_OUT);
        double K_decel = DRIVE_KF;

        if(drivetrainMode == DrivetrainMode.MANUAL){
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
        } else if(drivetrainMode == DrivetrainMode.AUTO) {
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
            leftFrontDrive.setPower(xPower_rotated - yPower_rotated * STRAFE_MULTIPLIER - tPower);
            leftBackDrive.setPower(xPower_rotated + yPower_rotated * STRAFE_MULTIPLIER - tPower);
            rightFrontDrive.setPower(xPower_rotated + yPower_rotated * STRAFE_MULTIPLIER + tPower);
            rightBackDrive.setPower(xPower_rotated - yPower_rotated * STRAFE_MULTIPLIER + tPower);

            //TODO Create a function that checks if target is reached based on target pose and current position
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
        }else if(drivetrainMode == DrivetrainMode.PROFILE){
            //double thetaTarget = Math.toRadians(degreeTarget);
            //Use PIDs to calculate motor powers based on error to targets
            double[]instantTarget = motionProfile.getTargetPosition();
            instantX = instantTarget[0];
            instantY = instantTarget[1];

            double xPower = xController.calculate(instantX, localizer.getX());
            double yPower = yController.calculate(instantY, localizer.getY());

            double remainingXDistance = Math.abs(targetPose.getX(DistanceUnit.INCH) - localizer.getX());
            double remainingYDistance = Math.abs(targetPose.getY(DistanceUnit.INCH) - localizer.getY());

            // Prevent division by zero or excessive deceleration force
            double minDistance = 0.1;
            remainingXDistance = Math.max(remainingXDistance, minDistance);
            remainingYDistance = Math.max(remainingYDistance, minDistance);

            // Compute feedforward deceleration
            double feedforwardXDecel = K_decel * (xPower * xPower) / (2 * remainingXDistance);
            double feedforwardYDecel = K_decel * (yPower * yPower) / (2 * remainingYDistance);

            // Apply deceleration in the same direction as pidOutput
            double adjustedXPower = xPower - Math.signum(xPower) * feedforwardXDecel;
            double adjustedYPower = yPower - Math.signum(yPower) * feedforwardYDecel;

            // Clip power to avoid excessive reduction
            adjustedXPower = Range.clip(adjustedXPower, -DRIVE_MAX_OUT, DRIVE_MAX_OUT);
            adjustedYPower = Range.clip(adjustedYPower, -DRIVE_MAX_OUT, DRIVE_MAX_OUT);

            //setMotorPower(adjustedPower);
            double wrappedAngleError = angleWrap(targetPose.getHeading(AngleUnit.DEGREES) - localizer.getHeading());
            double tPower = headingController.calculate(wrappedAngleError);

            double radianHeading = Math.toRadians(localizer.getHeading());

            //rotate the motor powers based on robot heading
            double xPower_rotated = adjustedXPower * Math.cos(-radianHeading) - adjustedYPower * Math.sin(-radianHeading);
            double yPower_rotated = adjustedXPower * Math.sin(-radianHeading) + adjustedYPower * Math.cos(-radianHeading);

            // x, y, theta input mixing to deliver motor powers
            leftFrontDrive.setPower(xPower_rotated - yPower_rotated - tPower);
            leftBackDrive.setPower(xPower_rotated + yPower_rotated - tPower);
            rightFrontDrive.setPower(xPower_rotated + yPower_rotated + tPower);
            rightBackDrive.setPower(xPower_rotated - yPower_rotated + tPower);

            //check if drivetrain is still working towards target
            targetReached = motionProfile.profileComplete() && headingController.targetReached;

            if(targetReached){
                //drivetrainMode = DrivetrainMode.AUTO;
            }

            String data = String.format(Locale.US, "{tX: %.3f, tY: %.3f, tH: %.3f}", targetPose.getX(DistanceUnit.INCH), targetPose.getY(DistanceUnit.INCH), targetPose.getHeading(AngleUnit.DEGREES));

            myOpMode.telemetry.addData("Target Position", data);
            myOpMode.telemetry.addData("Current X", localizer.getX());
            myOpMode.telemetry.addData("Instant X", instantX);
            myOpMode.telemetry.addData("ProfileTimeToMaxVelocity", motionProfile.timeToMaxVelocity);
            myOpMode.telemetry.addData("ProfileElapsedTime", motionProfile.timeElapsed);
            myOpMode.telemetry.addData("ProfileTotalTime", motionProfile.totalTime);
            myOpMode.telemetry.addData("ProfileTotalDistance", motionProfile.totalDistance);
            myOpMode.telemetry.addData("ProfileComplete", motionProfile.profileComplete());
            myOpMode.telemetry.addData("ProfilePhase", motionProfile.phase);
            myOpMode.telemetry.addData("HReached", headingController.targetReached);
            myOpMode.telemetry.addData("targetReached", targetReached);
        }

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        dashboardTelemetry.addData("instantX", instantX);
        dashboardTelemetry.addData("instantY", instantY);
        dashboardTelemetry.addData("currentX", localizer.getX());
        dashboardTelemetry.addData("currentY", localizer.getY());
        dashboardTelemetry.update();

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
