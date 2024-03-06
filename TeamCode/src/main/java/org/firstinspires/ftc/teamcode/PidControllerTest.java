package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.globals.SimplePIDController;
import org.firstinspires.ftc.teamcode.globals.robotMap;


import java.util.List;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "Testers")
public class PidControllerTest extends LinearOpMode {
    public static double DISTANCE = 60; // in

    public static double P1 = 0.007;
    public static double I1 = 0.0001;
    public static double D1 = 0.01;

    public static double P2 = 0.00133;
    public static double I2 = 0;
    public static double D2 = 0;


    public static double maxSpeed = 1;
    public static double plm=0;
    public static double RetractedPosition = 0 , ExtendedPosition = 600;
    int TargetLift = 0;
    ElapsedTime timerPID = new ElapsedTime();

    @Override

    public void runOpMode() throws InterruptedException {
        List <LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        ElapsedTime changePositions = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robotMap robot = new robotMap(hardwareMap);
        SimplePIDController up = new SimplePIDController(P1,I1,D1);
        SimplePIDController down = new SimplePIDController(P2,I2,D2);
        waitForStart();

        if (isStopRequested()) return;


        telemetry.update();
        up.targetValue = RetractedPosition;

        while (!isStopRequested() && opModeIsActive())
        {
            // if(plm==0)
            // SigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;

            int LiftPosition = robot.lift_right.getCurrentPosition();
            double powerLiftUP = up.update(LiftPosition);

            powerLiftUP = Math.max(-1,Math.min(powerLiftUP,1));
            robot.lift_right.setPower(powerLiftUP);
            robot.lift_left.setPower(powerLiftUP);



            if (changePositions.seconds()>3)
            {
                if (up.targetValue == RetractedPosition )
                {
                    up.targetValue = ExtendedPosition;
                    // SigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
                }
                else
                {
                    up.targetValue = RetractedPosition;
                }
                changePositions.reset();
            }



            if (P1!=up.p || D1!=up.d || I1!=up.i || maxSpeed !=up.maxOutput )
            {
                up.p = P1;
                up.d = P2;
                up.i = I2;
                up.maxOutput = maxSpeed;
            }

            telemetry.update();
        }
    }
}
