package org.firstinspires.ftc.teamcode.Subsystems;

    //package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utility.PIDController;

@Config
public class Lift {
    /* Declare OpMode members. */
    private final LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.

    //lift motors
    public DcMotor rightLift = null;
    public DcMotor leftLift = null;
    //public TouchSensor touch = null;

    //In Declarations
    PIDController rightLiftPIDController;
    public PIDController leftLiftPIDController;
    public Scoring scoring;

    //Defining Variable Constants
    public static double LIFT_KP = 0.005;
    public static double LIFT_KI = 0.0;
    public static double LIFT_KD = 0.0;
    public static double LIFT_MAX_OUT = 0.9;

    //lift constants
    public static int EXT_HIGH_BASKET= 4000;
    public static int EXT_RETRACTED = 0;
    public static int EXT_LIMIT = 1800;
    //lift for auto
    public static int EXT_HIGH_CHAMBER = 1000;
    public static int EXT_AUTO_2 = 1800;
    public static int EXT_LOW_BASKET = 1170;
    public static final double LIFT_SPEED = 0.5;

    //if the subsystem has explicit states, it can be helpful to use an enum to define them
    public enum LiftMode {
        MANUAL,
        LOW_BASKET,
        HIGH_BASKET,
        HIGH_CHAMBER,
        LOW_CHAMBER,
        EXT_AUTO_2,
        RETRACTED,
        ASCENT,
    }

    public LiftMode liftMode = LiftMode.MANUAL;

    public Lift(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init(Scoring scoringIN) {
        scoring = scoringIN;
        leftLiftPIDController = new PIDController(LIFT_KP,LIFT_KI,LIFT_KD,LIFT_MAX_OUT);
        rightLiftPIDController = new PIDController(LIFT_KP,LIFT_KI,LIFT_KD,LIFT_MAX_OUT);

        rightLift = myOpMode.hardwareMap.get(DcMotor.class, "rightLift");
        leftLift = myOpMode.hardwareMap.get(DcMotor.class, "leftLift");

        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setDirection(DcMotor.Direction.REVERSE);

        // brake and encoders
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        myOpMode.telemetry.addData(">", "Lift Initialized");
    }

    public void teleOp() {
        leftLiftPIDController = new PIDController(LIFT_KP,LIFT_KI,LIFT_KD,LIFT_MAX_OUT);
        rightLiftPIDController = new PIDController(LIFT_KP,LIFT_KI,LIFT_KD,LIFT_MAX_OUT);
        myOpMode.telemetry.addData("leftLiftPosition", leftLift.getCurrentPosition());
        myOpMode.telemetry.addData("rightLiftPosition", rightLift.getCurrentPosition());
        myOpMode.telemetry.addData("liftMode", liftMode);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(liftMode == LiftMode.MANUAL) {
            if (myOpMode.gamepad2.left_trigger > 0.1 && leftLift.getCurrentPosition() > 0) {
                leftLift.setPower(-0.3);
                rightLift.setPower(-0.3);
            } else if (scoring.pivot.getCurrentPosition() > 0 && myOpMode.gamepad2.right_trigger > 0.1 && leftLift.getCurrentPosition() < EXT_HIGH_BASKET) {
                leftLift.setPower(0.7);
                rightLift.setPower(0.7);
            }else if (scoring.pivot.getCurrentPosition() < 0 && myOpMode.gamepad2.right_trigger > 0.1 && leftLift.getCurrentPosition() < EXT_LIMIT) {
                leftLift.setPower(0.7);
                rightLift.setPower(0.7);
            }
                else{
                leftLift.setPower(myOpMode.gamepad2.right_stick_y);
                rightLift.setPower(myOpMode.gamepad2.right_stick_y);
            }
        }else if(liftMode == LiftMode.HIGH_CHAMBER) {
            liftToPositionPIDClass(EXT_HIGH_CHAMBER);
        }else if(liftMode == LiftMode.RETRACTED){
            liftToPositionPIDClass(EXT_RETRACTED);
        } else if (liftMode == LiftMode.HIGH_BASKET) {
            liftToPositionPIDClass(EXT_HIGH_BASKET);
        }else if(liftMode == LiftMode.LOW_BASKET){
            liftToPositionPIDClass(EXT_LOW_BASKET);
        }else if(liftMode == LiftMode.ASCENT){
        }

        //setting lift state
        if(Math.abs(myOpMode.gamepad2.left_trigger) > 0.1 || Math.abs(myOpMode.gamepad2.right_trigger) > 0.1){
            liftMode = LiftMode.MANUAL;
            rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }else if(myOpMode.gamepad2.y){
            liftMode = LiftMode.HIGH_BASKET;
        }else if(myOpMode.gamepad2.x){
            liftMode = LiftMode.HIGH_CHAMBER;
        }else if(myOpMode.gamepad2.b){
            liftMode = LiftMode.RETRACTED;
        }else if(myOpMode.gamepad2.a){
            liftMode = LiftMode.RETRACTED;
        }
    }

    public void update() {
        leftLiftPIDController = new PIDController(LIFT_KP,LIFT_KI,LIFT_KD,LIFT_MAX_OUT);
        rightLiftPIDController = new PIDController(LIFT_KP,LIFT_KI,LIFT_KD,LIFT_MAX_OUT);
        myOpMode.telemetry.addData("leftLiftPosition", leftLift.getCurrentPosition());
        myOpMode.telemetry.addData("rightLiftPosition", rightLift.getCurrentPosition());
        myOpMode.telemetry.addData("liftMode", liftMode);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(liftMode == LiftMode.MANUAL) {
            if (myOpMode.gamepad2.left_trigger > 0.1 && leftLift.getCurrentPosition() > 0) {
                leftLift.setPower(-0.3);
                rightLift.setPower(-0.3);
            } else if (myOpMode.gamepad2.right_trigger > 0.1 && leftLift.getCurrentPosition() < EXT_HIGH_BASKET) {
                leftLift.setPower(0.7);
                rightLift.setPower(0.7);
            } else{
                leftLift.setPower(0);
                rightLift.setPower(0);
            }
        }else if(liftMode == LiftMode.HIGH_CHAMBER) {
            liftToPositionPIDClass(EXT_HIGH_CHAMBER);
        }else if(liftMode == LiftMode.RETRACTED){
            liftToPositionPIDClass(EXT_RETRACTED);
        } else if (liftMode == LiftMode.HIGH_BASKET) {
            liftToPositionPIDClass(EXT_HIGH_BASKET);
        }else if(liftMode == LiftMode.LOW_BASKET){
            liftToPositionPIDClass(EXT_LOW_BASKET);
        }
        //setting lift state
    }


    //sends lift to the target encoder position
    public void liftToPositionPIDClass(double targetPosition) {
        double leftOut = leftLiftPIDController.calculate(targetPosition, leftLift.getCurrentPosition());
        double rightOut = rightLiftPIDController.calculate(targetPosition, rightLift.getCurrentPosition());

        leftLift.setPower(leftOut);
        rightLift.setPower(rightOut);

        myOpMode.telemetry.addData("LiftLeftPower: ", leftOut);
        myOpMode.telemetry.addData("LiftRightPower: ", rightOut);
        myOpMode.telemetry.addData("Running to", targetPosition);
        myOpMode.telemetry.addData("Currently at",  " at %7d :%7d",
               leftLift.getCurrentPosition(), rightLift.getCurrentPosition());
    }


    public void liftToTargetPosition(double speed,
                             int targetPosition) {

        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            rightLift.setTargetPosition(targetPosition);
            leftLift.setTargetPosition(targetPosition);

            // Turn On RUN_TO_POSITION
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start
            leftLift.setPower(Math.abs(speed));
            rightLift.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
                // Display it for the driver.
                myOpMode.telemetry.addData("Running to", targetPosition);
                myOpMode.telemetry.addData("Currently at",  " at %7d :%7d",
                        leftLift.getCurrentPosition(), rightLift.getCurrentPosition());
            }
              // optional pause after each move.
        }


}

