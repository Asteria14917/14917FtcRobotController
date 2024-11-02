package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
public class TeleOp extends LinearOpMode {

    Drivetrain drivetrain;
    Lift lift;
    Scoring scoring;

    @Override
    public void runOpMode() {

        drivetrain = new Drivetrain(this);
        lift = new Lift(this);
        scoring = new Scoring(this);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            drivetrain.teleOp();
            lift.teleOp();
            scoring.teleOp();
            telemetry.update();
        }
    }
}
