package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

    public class Scoring {
        /* Declare OpMode members. */
        private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

        //servos
        public Servo claw = null;
       // public Servo extension = null;
        public Servo clawPivot = null;
        public DcMotor pivot = null;

        // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
        public static final double CLAW_UP = 0.3;
        public static final double CLAW_DOWN = 0.05;
        public static final double CLAW_OPEN = 0.5;
        public static final double CLAW_CLOSED = 0;
        public static final int PIVOT_HIGH_BASKET = 300;
        public static final int PIVOT_LOW_BASKET= 0;
        public static final int PIVOT_SUBMERSIBLE = 3;
        public static final double WRIST_OUT = 0.6;
        public static final double WRIST_IN = 0.3;
        public static final double WRIST_MID = 0.4;
        public static final int PIVOT_LOW_LIMIT = -300;
        public static final int PIVOT_HIGH_LIMIT = 300;

        public enum PivotMode {
            PIVOT_SUBMERSIBLE,
            PIVOT_HIGH_BASKET,
            MANUAL
        }

        public PivotMode pivotMode = PivotMode.MANUAL;
        public Scoring(LinearOpMode opmode) {
            myOpMode = opmode;
        }

        public void init() {
            claw = myOpMode.hardwareMap.get(Servo.class, "claw");
           // extension = myOpMode.hardwareMap.get(Servo.class, "extension");
            clawPivot = myOpMode.hardwareMap.get(Servo.class, "clawPivot");
            pivot = myOpMode.hardwareMap.get(DcMotor.class, "pivot");

            //claw.setPosition(CLAW_CLOSED);
            //extension.setPosition(EXTENSION_IN);
            //clawPivot.setPosition(CLAW_UP);

            //extensionPosition = EXTENSION_IN;
            //pivotPosition = CLAW_UP;
            //clawPosition = CLAW_CLOSED;

            myOpMode.telemetry.addData(">", "Extension Initialized");
        }

        public void teleOp() {
            myOpMode.telemetry.addData("pivotPosition", pivot.getCurrentPosition());
            myOpMode.telemetry.addData("pivotMode", pivotMode);
            //send positions
            //claw.setPosition(CLAW_CLOSED);
            //clawPivot.setPosition(CLAW_UP);
            //extension.setPosition(extensionPosition);
            if (pivotMode == PivotMode.MANUAL
                    && pivot.getCurrentPosition() > PIVOT_LOW_LIMIT
                    && pivot.getCurrentPosition() < PIVOT_HIGH_LIMIT)
            {
                pivot.setPower(-myOpMode.gamepad2.left_stick_y);
            } else if (pivotMode == PivotMode.PIVOT_SUBMERSIBLE) {
                pivotToTargetPosition(0.5, PIVOT_SUBMERSIBLE);
            } else if (pivotMode == PivotMode.PIVOT_HIGH_BASKET) {
                pivotToTargetPosition(0.5, PIVOT_HIGH_BASKET);
            }

            //set positions
            if (myOpMode.gamepad2.left_bumper) {
                claw.setPosition(CLAW_OPEN);
            } else if(myOpMode.gamepad2.right_bumper){
                claw.setPosition(CLAW_CLOSED);
            }
/*
            if (myOpMode.gamepad1.) {
                pivot.setPower(PIVOT_HIGH_BASKET);
            }

            if (myOpMode.gamepad1.y) {
                pivot.setPower(PIVOT_LOW_BASKET);
            }
            */

            if(myOpMode.gamepad2.dpad_up){
                clawPivot.setPosition(WRIST_OUT);
            }

            if(myOpMode.gamepad2.dpad_down){
                clawPivot.setPosition(WRIST_IN);
            }

            if(myOpMode.gamepad2.dpad_left){
                clawPivot.setPosition(WRIST_MID);
            }
        }
        public void pivotToTargetPosition(double speed,
                                 int targetPosition){

            // Ensure that the OpMode is still active
            if (myOpMode.opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                pivot.setTargetPosition(targetPosition);

                // Turn On RUN_TO_POSITION
                pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion
                pivot.setPower(Math.abs(speed));

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                    // Display it for the driver.
                    myOpMode.telemetry.addData("Running to", targetPosition);
                    //myOpMode.telemetry.addData("Currently at",  " at %7d :%7d", targetPosition.getCurrentPosition());
                    myOpMode.telemetry.update();
                }

                // Stop all motion;
                pivot.setPower(0);
                pivot.setPower(0);

                // Turn off RUN_TO_POSITION
                pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                // optional pause after each move.
            }
        }
