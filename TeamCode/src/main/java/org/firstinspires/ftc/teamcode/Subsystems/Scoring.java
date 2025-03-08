package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

    public class Scoring {
        /* Declare OpMode members. */
        private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

        //servos
        public Servo claw = null;
        public Servo clawRotate = null;
        public CRServo intakeL = null;
        public CRServo intakeR = null;
        public Servo extension = null;
        public Servo clawPivot = null;
        public DcMotor pivot = null;

        // Define Drive constants.  Make them public so they CAN be used by the calling OpMode

        public static final double CLAW_OPEN = 0.73;
        public static final double CLAW_CLOSED = 0.1;
        public static final int PIVOT_HIGH_BASKET = 1200; //600
        public static final int PIVOT_ZERO= 0;
        public static final int PIVOT_SUBMERSIBLE = -1050;
        public static final double PIVOT_SPEED = 0.9;
        public static final int PIVOT_OBSERVATION_ZONE = 1620;//used to be -1300 //1580
        public static final int PIVOT_HIGH_CHAMBER = 284;
        public static final int PIVOT_UP_AUTO = 692;
        public static final int PIVOT_DOWN_AUTO = -200;
        public double rotateSetPosition;
        public static final double CLAW_ROTATE_SPECIMEN = 0;
        public static final double CLAW_ROTATE_SAMPLE = 0.65;
        //-364
        public static final double WRIST_IN = 0.07;//001
        //0.6
        public static final double WRIST_MID = 0.6;
        public static final double WRIST_SPECIMEN = 0.55;//0.57//0.55
        public static final double WRIST_OUT = 0.67;
        public static final double WRIST_SCORING = 0.56;
        public static final double WRIST_INTAKE = 0.3;
        public static final double WRIST_HOVER = 0.5;
        public static final double ROTATE_HALF = 0.2;
        public static final double ROTATE_FULL = 0.55;


        //mid is low, out is middle, in is high
        //for second specimen scoring
        public static double WRIST_AUTO = 0.42;//was 35
        public static final int PIVOT_LOW_LIMIT = -1200;
        public static final int PIVOT_HIGH_LIMIT = 1200;

        //1774 for high limit
        //-1114 for low limit

        public enum PivotMode {
            PIVOT_SUBMERSIBLE,
            PIVOT_HIGH_BASKET,
            PIVOT_HIGH_CHAMBER,
            OBSERVATION_ZONE,
            PIVOT_UP_AUTO,
            PIVOT_DOWN_AUTO,
            PIVOT_ZERO,
            MANUAL,
            ASCENT
        }

        public PivotMode pivotMode = PivotMode.MANUAL;
        public Scoring(LinearOpMode opmode) {
            myOpMode = opmode;
        }

        public void init() {
            intakeL = myOpMode.hardwareMap.get(CRServo.class, "intakeL");
            intakeR = myOpMode.hardwareMap.get(CRServo.class, "intakeR");
            intakeL.setDirection(CRServo.Direction.FORWARD);
            intakeR.setDirection(CRServo.Direction.REVERSE);
            claw = myOpMode.hardwareMap.get(Servo.class, "claw");
           // extension = myOpMode.hardwareMap.get(Servo.class, "extension");
            clawPivot = myOpMode.hardwareMap.get(Servo.class, "clawPivot");
            clawRotate = myOpMode.hardwareMap.get(Servo.class, "clawRotate");
            pivot = myOpMode.hardwareMap.get(DcMotor.class, "pivot");
            pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            claw.setPosition(CLAW_CLOSED);
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
            if (myOpMode.gamepad1.dpad_right){
                pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            //send positions
            //claw.setPosition(CLAW_CLOSED);
            //clawPivot.setPosition(CLAW_UP);
            //extension.setPosition(extensionPosition);
            if (pivotMode == PivotMode.MANUAL) {
                //pivot.getCurrentPosition() > PIVOT_LOW_LIMIT
                if (-myOpMode.gamepad2.left_stick_y < -.1) {
                    pivot.setPower(-myOpMode.gamepad2.left_stick_y);
                } else if (-myOpMode.gamepad2.left_stick_y > .1 && pivot.getCurrentPosition() < PIVOT_HIGH_LIMIT) {
                    //pivot.getCurrentPosition() < PIVOT_HIGH_LIMIT &&
                    pivot.setPower(-myOpMode.gamepad2.left_stick_y);
                } else {
                    pivot.setPower(0);
                }
            } else if (pivotMode == PivotMode.PIVOT_SUBMERSIBLE) {
                pivotToTargetPosition(PIVOT_SPEED, PIVOT_SUBMERSIBLE);
            } else if (pivotMode == PivotMode.PIVOT_HIGH_BASKET) {
                pivotToTargetPosition(PIVOT_SPEED, PIVOT_HIGH_BASKET);
            } else if (pivotMode == PivotMode.PIVOT_HIGH_CHAMBER) {
                pivotToTargetPosition(PIVOT_SPEED, PIVOT_HIGH_CHAMBER);
            } else if (pivotMode == PivotMode.OBSERVATION_ZONE) {
                pivotToTargetPosition(PIVOT_SPEED, PIVOT_OBSERVATION_ZONE);
            } else if(pivotMode == PivotMode.PIVOT_ZERO){
                pivotToTargetPosition(PIVOT_SPEED, PIVOT_ZERO);
            }
            else if (pivotMode == PivotMode.ASCENT) {
            }

            if (Math.abs(myOpMode.gamepad2.left_stick_y) > 0.1) {
                pivotMode = PivotMode.MANUAL;
                pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else if (myOpMode.gamepad2.y) {
                pivotMode = PivotMode.PIVOT_HIGH_BASKET;
            }//else if(myOpMode.gamepad2.a){
            // pivotMode = PivotMode.PIVOT_SUBMERSIBLE;
            //}
            else if (myOpMode.gamepad2.x) {
                rotateSetPosition = CLAW_ROTATE_SPECIMEN;
                clawPivot.setPosition(WRIST_SCORING);
                pivotMode = PivotMode.PIVOT_ZERO;

            } else if (myOpMode.gamepad2.b) {
                pivotMode = PivotMode.OBSERVATION_ZONE;
                clawPivot.setPosition(WRIST_IN);
                rotateSetPosition = CLAW_ROTATE_SPECIMEN;

                //set posItions with mini claw
                //b is wall position, x is scoring
                //b pivot is 1943, claw opens
                //x pivot 284, lift 215, wrist is WRIST_AUTO

            }


            /* //set positions (ACTIVE INTAKE)
            if (myOpMode.gamepad2.left_bumper) {
                //claw.setPosition(CLAW_OPEN);
                intakeL.setPower(1);
                intakeR.setPower(1);
                //  clawPivot.setPosition(WRIST_MID);
            } else if (myOpMode.gamepad2.right_bumper) {
                intakeL.setPower(-1);
                intakeR.setPower(-1);
                //   clawPivot.setPosition(WRIST_HOVER);
            } else {
                intakeL.setPower(0);
                intakeR.setPower(0);
                //  clawPivot.setPosition(WRIST_HOVER);
            }

             */




            //set positions (CLAW)
            if (myOpMode.gamepad2.left_bumper) {
                claw.setPosition(CLAW_OPEN);
            } else if(myOpMode.gamepad2.right_bumper){
                claw.setPosition(CLAW_CLOSED);
            }
    if(myOpMode.gamepad2.right_trigger > 0.01) {
        rotateSetPosition = CLAW_ROTATE_SAMPLE - myOpMode.gamepad2.right_trigger / 3;
    }
    if(myOpMode.gamepad2.left_trigger > 0.01){
        rotateSetPosition = CLAW_ROTATE_SAMPLE;
    }

            clawRotate.setPosition(rotateSetPosition);


            //360 claw
            /*
            if (myOpMode.gamepad2.left_trigger > 0.1) {
                clawRotate.setPosition(ROTATE_HALF);
            } else if(myOpMode.gamepad2.right_trigger > 0.1){
                clawRotate.setPosition(ROTATE_FULL);
            }

             */





            if (myOpMode.gamepad1.y) {
                pivot.setPower(PIVOT_HIGH_BASKET);
            }

            // if (myOpMode.gamepad1.y) {
            //       pivot.setPower(PIVOT_LOW_BASKET);
            // }


            if (myOpMode.gamepad2.dpad_up) {
                clawPivot.setPosition(WRIST_IN);
                rotateSetPosition = CLAW_ROTATE_SPECIMEN;
            }

            if (myOpMode.gamepad2.dpad_left) {
                clawPivot.setPosition(WRIST_MID);
                rotateSetPosition = CLAW_ROTATE_SAMPLE;
            }

            if (myOpMode.gamepad2.dpad_right) {
                //rotateSetPosition = CLAW_ROTATE_SAMPLE;
                clawPivot.setPosition(WRIST_AUTO);
            }
            if (myOpMode.gamepad2.dpad_down) {
                clawPivot.setPosition(WRIST_OUT);
                rotateSetPosition = CLAW_ROTATE_SAMPLE;

            }
        }

        public void update() {
            myOpMode.telemetry.addData("pivotPosition", pivot.getCurrentPosition());
            myOpMode.telemetry.addData("pivotMode", pivotMode);
            //send positions
            //claw.setPosition(CLAW_CLOSED);
            //clawPivot.setPosition(CLAW_UP);
            //extension.setPosition(extensionPosition);
            if (pivotMode == PivotMode.MANUAL)
            {
                //if(pivot.getCurrentPosition() > PIVOT_LOW_LIMIT && -myOpMode.gamepad2.left_stick_y < -.1) {
                //pivot.setPower(-myOpMode.gamepad2.left_stick_y/2);
                //}else if(pivot.getCurrentPosition() < PIVOT_HIGH_LIMIT && -myOpMode.gamepad2.left_stick_y >.1){
                pivot.setPower(-myOpMode.gamepad2.left_stick_y);
                //}else{
                //pivot.setPower(0);
                //}
            } else if (pivotMode == PivotMode.PIVOT_SUBMERSIBLE) {
                pivotToTargetPosition(PIVOT_SPEED, PIVOT_SUBMERSIBLE);
            } else if (pivotMode == PivotMode.PIVOT_HIGH_BASKET) {
                pivotToTargetPosition(PIVOT_SPEED, PIVOT_HIGH_BASKET);
            } else if(pivotMode == PivotMode.OBSERVATION_ZONE){
                pivotToTargetPosition(PIVOT_SPEED, PIVOT_OBSERVATION_ZONE);
            }else if(pivotMode == PivotMode.PIVOT_DOWN_AUTO){
                pivotToTargetPosition(PIVOT_SPEED, PIVOT_DOWN_AUTO);
            }else if(pivotMode == PivotMode.PIVOT_UP_AUTO){
                pivotToTargetPosition(PIVOT_SPEED, PIVOT_UP_AUTO);
            }else if(pivotMode == PivotMode.PIVOT_ZERO){
                pivotToTargetPosition(PIVOT_SPEED, PIVOT_ZERO);
            }

            if(Math.abs(myOpMode.gamepad2.left_stick_y) > 0.1){
                pivotMode = PivotMode.MANUAL;
                pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }else if(myOpMode.gamepad2.y){
                pivotMode = PivotMode.PIVOT_HIGH_BASKET;
            }else if(myOpMode.gamepad2.a){
                pivotMode = PivotMode.PIVOT_SUBMERSIBLE;
            }
           /* //set positions
            if (myOpMode.gamepad2.left_bumper) {
                //claw.setPosition(CLAW_OPEN);
                intakeL.setPower(1);
                intakeR.setPower(1);
            } else if(myOpMode.gamepad2.right_bumper){
                intakeL.setPower(-1);
                intakeR.setPower(-1);
            }else {
                intakeL.setPower(0);
                intakeR.setPower(0);
            } */

            if (myOpMode.gamepad1.y) {
                pivot.setPower(PIVOT_HIGH_BASKET);
            }

            //if (myOpMode.gamepad1.y) {
            //    pivot.setPower(PIVOT_LOW_BASKET);
            //}


            if(myOpMode.gamepad2.dpad_left){
                clawPivot.setPosition(WRIST_MID);
            }

            if(myOpMode.gamepad2.dpad_right) {
                clawPivot.setPosition(WRIST_AUTO);
            }

            if(myOpMode.gamepad2.dpad_up){
                clawPivot.setPosition(WRIST_IN);
            }

            if(myOpMode.gamepad2.dpad_down){
                clawPivot.setPosition(WRIST_OUT);
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
                    //myOpMode.telemetry.addData("Currently at",  " at %7d :%7d", targetPosition.getCurrentPositio
                }


                // optional pause after each move.
            }
        }

