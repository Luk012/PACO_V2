package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.globals.robotMap;

@TeleOp(name = "Distance Sensor Calibration", group = "OpMode")
public class DistanceSensorCalibrationOpMode extends LinearOpMode {

    private DistanceSensorCalibrator calibrator;

    @Override
    public void runOpMode() throws InterruptedException {

        robotMap r = new robotMap(hardwareMap);
        LowPassFilter l = new LowPassFilter(0.9, r.right.getDistance(DistanceUnit.CM));

        // Wait for start button
        waitForStart();

        // Create arrays to store calibration data
        double[] rawReadings = {26.3, 25.5, 26, 24.4, 26.6, 25, 26.9, 25.8, 26.1, 26.5, 27.1, 25.6, 26, 26.1, 26.3, 25.9, 26.4, 25.5, 24.7, 26.9};
        double[] actualDistances = {26, 25, 25.5, 24, 26.5, 24.5, 27, 25.4, 25.7, 26.2, 26.6, 24.8, 25.3, 25.9, 26.1, 25.8, 26.3, 25.1, 24.9, 26.8};

        // Perform calibration using collected data
        calibrator = new DistanceSensorCalibrator(rawReadings, actualDistances);

        while (opModeIsActive() && !isStopRequested()) {
            // Get raw sensor reading
            double rawReading = r.right.getDistance(DistanceUnit.CM);

            // Calibrate raw reading
            double calibratedDistance = calibrator.calibrate(rawReading);

            // Display raw and calibrated readings
            telemetry.addData("Raw Reading", rawReading);
            telemetry.addData("Calibrated Distance WITH Filter", l.getValue(calibratedDistance));
            telemetry.addData("Calibrated Distance WITHOUT Filter", calibratedDistance);
            telemetry.addData("Difference", calibratedDistance - rawReading);
            telemetry.update();
        }
    }
}
