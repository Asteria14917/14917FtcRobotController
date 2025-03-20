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

    public ProfileDrivetrain drivetrain;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public TuningHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        drivetrain = new ProfileDrivetrain(myOpMode);

        drivetrain.init();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }


    public void update(){
        drivetrain.update();
    }

}


