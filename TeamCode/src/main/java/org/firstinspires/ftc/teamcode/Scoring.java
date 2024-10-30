package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

    public class Scoring {
        /* Declare OpMode members. */
        private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

        //servos
        public Servo claw = null;
       // public Servo extension = null;
        public Servo clawPivot = null;

        // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
        public static final double CLAW_UP = 0.9;
        public static final double CLAW_DOWN = 0.05;
        public static final double CLAW_HOVER = 0.3;
        public static final double CLAW_OPEN = 0.5;
        public static final double CLAW_CLOSED = 0;
        public static final double EXTENSION_IN = 0.6;
        public static final double EXTENSION_MID = 0.4;
        public static final double EXTENSION_OUT = 0.25;

        public double extensionPosition;
        public double pivotPosition;
        public double clawPosition;

        public Scoring(LinearOpMode opmode) {
            myOpMode = opmode;
        }

        public void init() {
            claw = myOpMode.hardwareMap.get(Servo.class, "claw");
           // extension = myOpMode.hardwareMap.get(Servo.class, "extension");
            pivot = myOpMode.hardwareMap.get(Servo.class, "pivot");

            claw.setPosition(CLAW_CLOSED);
            //extension.setPosition(EXTENSION_IN);
            clawPivot.setPosition(CLAW_UP);

            //extensionPosition = EXTENSION_IN;
            //pivotPosition = CLAW_UP;
            //clawPosition = CLAW_CLOSED;

            myOpMode.telemetry.addData(">", "Extension Initialized");
        }

        public void teleOp() {
            //send positions
            claw.setPosition(clawPosition);
            clawPivot.setPosition(pivotPosition);
            //extension.setPosition(extensionPosition);

            myOpMode.telemetry.addData("Extension: ", extensionPosition);

            //set positions
            if (myOpMode.gamepad1.a) {
                clawPosition = CLAW_OPEN;
            } else {
                clawPosition = CLAW_CLOSED;
            }

            if (myOpMode.gamepad1.dpad_up) {
                pivotPosition = CLAW_UP;
            }

            if (myOpMode.gamepad1.dpad_right) {
                pivotPosition = CLAW_HOVER;
            }

            if (myOpMode.gamepad2.dpad_down) {
                pivotPosition = CLAW_DOWN;
            }

            if (myOpMode.gamepad2.x) {
                extensionPosition = EXTENSION_IN;
            }

            if (myOpMode.gamepad2.left_stick_y > 0 && extensionPosition < 0.6) {
                extensionPosition += 0.05;
            }

            if (myOpMode.gamepad2.left_stick_y < 0 && extensionPosition > 0) {
                extensionPosition -= 0.05;
            }
        }
    }
}
