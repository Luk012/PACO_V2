package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.globals.robotMap;

@TeleOp(name = "Test Senzor", group = "OpMode")
public class TestSenzor extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        robotMap r = new robotMap(hardwareMap);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            telemetry.addData("distance", r.back.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

    }
}
