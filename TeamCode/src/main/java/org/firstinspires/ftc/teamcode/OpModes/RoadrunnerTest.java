package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Scoring;


@Autonomous(name="RoadRunnerTest", group="Linear OpMode")
@Config
public class RoadrunnerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Scoring scoring = new Scoring(this);
        scoring.init();
        waitForStart();

        Actions.runBlocking(scoring.clawToObservationZone());
        Actions.runBlocking(scoring.clawSubmersible());
    }
}