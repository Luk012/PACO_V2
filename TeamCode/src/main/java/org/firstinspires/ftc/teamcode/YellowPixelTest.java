package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AUTO.Recognition.YellowPixelMaster;
import org.firstinspires.ftc.teamcode.AprilTag.ConceptAprilTag;

@Autonomous(name = "YellowPixelTest")
public class YellowPixelTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        YellowPixelMaster yellowTest = new YellowPixelMaster(this);

        yellowTest.observeStick();

        telemetry.addData("Item: ", yellowTest.yellowPixel.getWhichSide());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            
            telemetry.addData("Item: ", yellowTest.yellowPixel.getWhichSide());
            telemetry.addData("imx", yellowTest.yellowPixel.imx);
            telemetry.addData("avg_leftLeft",yellowTest.yellowPixel.avg_leftLeft);
            telemetry.addData("avg_leftRight",yellowTest.yellowPixel.avg_leftRight);
            telemetry.addData("avg_centerLeft",yellowTest.yellowPixel.avg_centerLeft);
            telemetry.addData("avg_centerRight",yellowTest.yellowPixel.avg_centerRight);
            telemetry.addData("avg_rightLeft",yellowTest.yellowPixel.avg_rightLeft);
            telemetry.addData("avg_rightRight",yellowTest.yellowPixel.avg_rightRight);
            telemetry.update();
            sleep(100);
        }

        yellowTest.stopCamera();

    }
}
