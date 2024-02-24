package org.firstinspires.ftc.teamcode.AprilTag;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AUTO.Recognition.YellowPixelMaster;
import org.firstinspires.ftc.teamcode.AprilTag.ConceptAprilTag;

@Autonomous(name = "AprilTagDetectionTest")
public class AprilTagDetectionTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        ConceptAprilTag test = new ConceptAprilTag();

        waitForStart();

        while (opModeIsActive()) {
            test.runOpMode();
//            telemetry.update();
            sleep(100);
        }

    }
}
