package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;
import org.firstinspires.ftc.teamcode.Utility.TuningDrivetrain;

public class TuningHardware{
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    ElapsedTime runtime = new ElapsedTime();

    public TuningDrivetrain drivetrain;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public TuningHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        drivetrain = new TuningDrivetrain(myOpMode);

        drivetrain.init();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    public void teleOp() {
        drivetrain.teleOp();
    }

    public void update(){
        drivetrain.update();
    }
/*
        public void deliver() {
            scoring.pivot.setPosition(scoring.CLAW_HOVER);
            scoring.extension.setPosition(scoring.EXTENSION_OUT - 0.2);
            myOpMode.sleep(500);
            scoring.claw.setPosition(scoring.CLAW_OPEN);
            myOpMode.sleep(250);
            scoring.extension.setPosition(scoring.EXTENSION_IN);
            myOpMode.sleep(500);
        }

        public void retrieve() {
            scoring.pivot.setPosition(scoring.CLAW_DOWN);
            myOpMode.sleep(250);
            scoring.extension.setPosition(scoring.EXTENSION_OUT - 0.24);
            myOpMode.sleep(500);
            scoring.claw.setPosition(scoring.CLAW_CLOSED);
            myOpMode.sleep(250);
            scoring.extension.setPosition(scoring.EXTENSION_IN);
            myOpMode.sleep(400); //new one
        }
*/
}


