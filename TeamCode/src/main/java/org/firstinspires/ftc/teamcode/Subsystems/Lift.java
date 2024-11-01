package org.firstinspires.ftc.teamcode.Subsystems;

    //package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Lift {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    //lift motors
    public DcMotor rightLift = null;
    public DcMotor leftLift = null;
    //public TouchSensor touch = null;

    //lift constants
    public static final int EXT_HIGH_BASKET= 1;
    public static final int EXT_RETRACTED = 0;
    public static final int EXT_HIGH_CHAMBER = 0;

    //if the subsystem has explicit states, it can be helpful to use an enum to define them
    public enum LiftMode {
        MANUAL,
        LOW_BASKET,
        HIGH_BASKET,
        HIGH_CHAMBER,
        LOW_CHAMBER,
        RETRACTED
    }

    public LiftMode liftMode = LiftMode.MANUAL;

    public Lift(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {

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
        myOpMode.telemetry.addData("leftLiftPosition", leftLift.getCurrentPosition());
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(liftMode == LiftMode.MANUAL) {
            if (myOpMode.gamepad2.left_trigger > 0.1) {
                leftLift.setPower(-0.3);
                rightLift.setPower(-0.3);
            } else if (myOpMode.gamepad2.right_trigger > 0.1) {
                leftLift.setPower(0.7);
                rightLift.setPower(0.7);
            }
        } else if (liftMode == LiftMode.HIGH_BASKET) {
           liftToTargetPosition(0, EXT_HIGH_BASKET);
        }else if(liftMode == LiftMode.HIGH_CHAMBER) {
            liftToTargetPosition(0, EXT_HIGH_CHAMBER);
        }else if(liftMode == LiftMode.RETRACTED){
            liftToTargetPosition(0, EXT_RETRACTED);

        }
    }

    public void liftToTargetPosition(double speed,
                             int targetPosition) {

        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            rightLift.setTargetPosition(targetPosition);

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
                myOpMode.telemetry.update();
            }
              // optional pause after each move.
        }

    public void resetLift(double speed) {
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*if (touch.isPressed() == false) {
            leftLift.setPower(speed);
            rightLift.setPower(speed);
        } else {
            leftLift.setPower(0);
            rightLift.setPower(0);
        }
         */

    }

}

