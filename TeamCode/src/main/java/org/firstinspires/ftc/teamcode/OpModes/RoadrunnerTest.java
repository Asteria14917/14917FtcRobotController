package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Scoring;

public class RoadrunnerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Scoring scoring = new Scoring(this);
        scoring.init();
        waitForStart();

        Actions.runBlocking(scoring.clawToObservationZone());
    }
}