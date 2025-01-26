package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Meet 2 TeleOp", group="Linear OpMode")
public class TeleOp extends LinearOpMode {

    Drivetrain drivetrain;
    Lift lift;
    Scoring scoring;


    @Override
    public void runOpMode() {

        drivetrain = new Drivetrain(this);
        lift = new Lift(this);
        scoring = new Scoring(this);

        drivetrain.init();
        scoring.init();
        lift.init(scoring);

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
