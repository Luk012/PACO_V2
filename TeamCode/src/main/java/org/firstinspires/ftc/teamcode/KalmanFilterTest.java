package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.KalmanFilter.KalmanFilter;
import org.firstinspires.ftc.teamcode.globals.robotMap;

@TeleOp(name = "Kalman Filter Test", group = "OpMode")
public class KalmanFilterTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        robotMap r = new robotMap(hardwareMap);

        double Q = 0.5;
        double R = 4;
        int N = 5;
        KalmanFilter filter = new KalmanFilter(Q, R, N);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){

                double currentValue = r.back.getDistance(DistanceUnit.CM);
                double estimate = filter.estimate(currentValue);

                telemetry.addData("Sensor distance: ", r.back.getDistance(DistanceUnit.CM));
                telemetry.addData("Calibrated distance", estimate);
                telemetry.update();

        }

    }
}
