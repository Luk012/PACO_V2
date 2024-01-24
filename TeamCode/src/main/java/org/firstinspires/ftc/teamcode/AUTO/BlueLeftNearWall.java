package org.firstinspires.ftc.teamcode.AUTO;

import static java.lang.Boolean.FALSE;

import android.sax.TextElementListener;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AUTO_CONTROLLERS.Blue_LEFT;
import org.firstinspires.ftc.teamcode.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.globals.InverseKinematics;
import org.firstinspires.ftc.teamcode.globals.robotMap;
import org.firstinspires.ftc.teamcode.system_controllers.collectAngle_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.fourBar_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.leftLatch_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.lift_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.pto_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.rightLatch_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.storageAngle_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.storage_Controller;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.io.BufferedReader;
import java.util.List;

@Config
@Autonomous(group = "Auto" , name = "BlueLeftNearWall")

public class BlueLeftNearWall extends LinearOpMode {

    enum STROBOT {
        START,
        PURPLE_DROP,
        YELLOW_DROP,
        YELLOW,
        INTER,
        GO_TO_STACK,
        COLLECT,
        VERIF,
        GO_SCORE,
        PREPARE_FOR_SCORE,
        SCORE,
        CHECK_COLLECT,
        PARK,
        PREPARE_COLLECT,
        MICHI_MAUS,
        MICHI_MAUS2,
    }

    public static double x_start = 16, y_start = 62, angle_start = 270;
    public static double x_purple_left = 27, y_purple_left = 36, angle_purple_left = 270;
    public static double x_yellow_left = 47, y_yellow_left = 38.5, angle_yellow_left = 180;
    public static double x_yellow_right = 50, y_yellow_right = 29, angle_yellow_right = 180;
    public static double x_stack = -57.5, y_stack = 30, angle_stack = 180;
    public static double x_prepare_for_stack = 27.5, y_prepare_for_stack = 59, angle_prepare = 180;
    public static double x_lung_de_linie = -26, y_lung_de_linie = 59, angle_lung_de_linie = 180;
    public static double x_park_from_right = 48, y_park_from_right = 62, angle_park_from_right = 180;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robotMap r = new robotMap(hardwareMap);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        double currentVoltage;
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        currentVoltage = batteryVoltageSensor.getVoltage();

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        collectAngle_Controller collectAngle = new collectAngle_Controller();
        outtake_Controller outtake = new outtake_Controller();
        fourBar_Controller fourbar = new fourBar_Controller(r);
        leftLatch_Controller leftLatch = new leftLatch_Controller();
        lift_Controller lift = new lift_Controller();
        pto_Controller pto = new pto_Controller();
        rightLatch_Controller rightLatch = new rightLatch_Controller();
        storage_Controller storage = new storage_Controller();
        storageAngle_Controller storageAngle = new storageAngle_Controller();
        InverseKinematics ik = new InverseKinematics(r.right_fourbar, r.left_fourbar, r.storage_angle, r.back);

        Blue_LEFT blue_left = new Blue_LEFT();

        collectAngle.CS = collectAngle_Controller.collectAngleStatus.GROUND;
        fourbar.CS = fourBar_Controller.fourbarStatus.COLLECT;
        leftLatch.CS = leftLatch_Controller.leftLatchStatus.INITIALIZE;
        rightLatch.CS = rightLatch_Controller.rightLatchStatus.INITIALIZE;
        lift.CS = lift_Controller.liftStatus.DOWN;
        storage.CS = storage_Controller.storageStatus.COLLECT;
        storageAngle.CS = storageAngle_Controller.storageAngleStatus.INITIALIZE;
        pto.CS = pto_Controller.ptoStatus.OFF;
        blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.NOTHING;
        //outtake.CS = outtake_Controller.outtakeStatus.INITIALIZE;

        lift.update(r, 0, currentVoltage);
        fourbar.update(r);
        collectAngle.update(r);
        leftLatch.update(r);
        rightLatch.update(r);
        storage.update(r);
        storageAngle.update(r);
        outtake.update(fourbar, storage, storageAngle, lift);
        blue_left.update(r, lift, ik, fourbar, storage, storageAngle);
        pto.update(r);

        Pose2d start_pose = new Pose2d(x_start, y_start,Math.toRadians(angle_start));
        Pose2d purple_left = new Pose2d(x_purple_left, y_purple_left, Math.toRadians(angle_purple_left));
        Pose2d yellow_left = new Pose2d(x_yellow_left, y_yellow_left, Math.toRadians(angle_yellow_left));
        Pose2d stack = new Pose2d(x_stack, y_stack, Math.toRadians(angle_stack));
        Pose2d prepare_for_stack = new Pose2d(x_prepare_for_stack, y_prepare_for_stack, Math.toRadians(angle_prepare));
        Pose2d lung_de_linie = new Pose2d(x_lung_de_linie, y_lung_de_linie, Math.toRadians(angle_lung_de_linie));
        Pose2d yellow_right = new Pose2d(x_yellow_right, y_yellow_right, Math.toRadians(angle_yellow_right));
        Pose2d park_from_right = new Pose2d(x_park_from_right, y_park_from_right, Math.toRadians(angle_park_from_right));

        TrajectorySequence PURPLE_LEFT = drive.trajectorySequenceBuilder(start_pose)
                .lineToLinearHeading(purple_left)
                .build();

        TrajectorySequence YELLOW_LEFT = drive.trajectorySequenceBuilder(purple_left)
                .lineToLinearHeading(yellow_left)
                .build();

        TrajectorySequence GO_STACK = drive.trajectorySequenceBuilder(yellow_left)
                .lineToLinearHeading(prepare_for_stack)
                .lineToLinearHeading(lung_de_linie)
                .splineToLinearHeading(stack,Math.toRadians(180))
                .build();

        TrajectorySequence GO_STACK_2 = drive.trajectorySequenceBuilder(yellow_right)
                .lineToLinearHeading(prepare_for_stack)
                .lineToLinearHeading(lung_de_linie)
                .splineToLinearHeading(stack,Math.toRadians(180))
                .build();

        TrajectorySequence SCORE_LEFT = drive.trajectorySequenceBuilder(stack)
                .lineToLinearHeading(lung_de_linie)
                .splineToLinearHeading(prepare_for_stack, Math.toRadians(180))
                .splineToLinearHeading(yellow_right, Math.toRadians(180))
                .build();

        TrajectorySequence PARK_FROM_RIGHT = drive.trajectorySequenceBuilder(yellow_right)
                .lineToLinearHeading(park_from_right)
                .build();

        drive.setPoseEstimate(start_pose);
        STROBOT status = STROBOT.START;

        ElapsedTime collect = new ElapsedTime();
        ElapsedTime score = new ElapsedTime();
        ElapsedTime preload = new ElapsedTime();
        ElapsedTime preload2 = new ElapsedTime();
        ElapsedTime collect2= new ElapsedTime();

        double nrcicluri = 0;
        collectAngle.stack_level = 3;

        while (!isStarted() && !isStopRequested()) {

            sleep(20);

        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            int position = r.lift.getCurrentPosition();


            switch (status) {

                case START: {
                    drive.followTrajectorySequenceAsync(PURPLE_LEFT);
                    preload.reset();
                    status = STROBOT.PURPLE_DROP;
                    break;
                }

                case PURPLE_DROP: {
                    if (!drive.isBusy() || preload.seconds() > 0.65) {
                        collectAngle_Controller.CS = collectAngle_Controller.collectAngleStatus.LIFTED;
                        drive.followTrajectorySequenceAsync(YELLOW_LEFT);
                        preload2.reset();;
                        status = STROBOT.YELLOW;
                    }
                    break;
                }


                case YELLOW: {
                    if (!drive.isBusy() || preload2.seconds() > 0.85) {
                        blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.SCORE_PRELOAD;
                        status = STROBOT.YELLOW_DROP;
                    }
                    break;
                }

                case YELLOW_DROP: {
                    if (blue_left.CurrentStatus == Blue_LEFT.autoControllerStatus.SCORE_PRELOAD_DONE) {
                        leftLatch_Controller.CS = leftLatch_Controller.leftLatchStatus.OPEN;
                        rightLatch_Controller.CS = rightLatch_Controller.rightLatchStatus.OPEN;
                        score.reset();
                        status = status.GO_TO_STACK;
                    }
                    break;
                }

                case GO_TO_STACK: {
                    if(score.seconds() > 0.25)
                    {  drive.followTrajectorySequenceAsync(GO_STACK);
                        collect.reset();
                        status = STROBOT.PREPARE_COLLECT;}
                    break;
                }

                case MICHI_MAUS:
                {
                    if(score.seconds() > 0.01)
                    {  drive.followTrajectorySequenceAsync(GO_STACK_2);
                        collect.reset();
                        status = STROBOT.MICHI_MAUS2;}
                    break;
                }

                case MICHI_MAUS2:
                {
                    if(collect.seconds() > 3)
                    {
                      //  blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.COLLECT;
                        status = STROBOT.COLLECT;
                    }
                    break;
                }

                case PREPARE_COLLECT:
                {
                     if(collect.seconds() > 0.9)
                     {
                         blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.COLLECT;
                         collect2.reset();
                         status = STROBOT.COLLECT;
                     }
                     break;
                }

                case COLLECT: {
                    if(collect.seconds() > 0.01)
                    {

                        collectAngle.CS = collectAngle_Controller.collectAngleStatus.LIFTED;
                    }
                    if (!drive.isBusy() || collect2.seconds() > 4.5) {
                                 r.collect.setPower(1);
                                 collectAngle.CS = collectAngle_Controller.collectAngleStatus.STACK;
                                 collect.reset();
                                 status = STROBOT.VERIF;
                    }
                    break;
                }

                case VERIF:
                {
                    if(r.left_pixel.getState() == FALSE && leftLatch.CS == leftLatch_Controller.leftLatchStatus.OPEN)
                    {
                        leftLatch.CS = leftLatch_Controller.leftLatchStatus.CLOSE;
                    }

                    if(r.right_pixel.getState() == FALSE && rightLatch.CS == rightLatch_Controller.rightLatchStatus.OPEN)
                    {
                        rightLatch.CS = rightLatch_Controller.rightLatchStatus.CLOSE;
                    }

                    if(leftLatch.CS == leftLatch_Controller.leftLatchStatus.CLOSE_DONE && rightLatch.CS == rightLatch_Controller.rightLatchStatus.CLOSE_DONE || collect.seconds() > 2)
                    {
                      //  outtake.CS = outtake_Controller.outtakeStatus.INTER;
                        status = STROBOT.GO_SCORE;
                    }
                    break;
                }

                case GO_SCORE:
                {
                    r.collect.setPower(-1);
                    drive.followTrajectorySequenceAsync(SCORE_LEFT);
                    score.reset();
                    status = STROBOT.PREPARE_FOR_SCORE;
                    break;
                }

                case PREPARE_FOR_SCORE:
                {
                    if(!drive.isBusy() || score.seconds() > 6.6)

                    { r.collect.setPower(0);
                        blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.SCORE;
                    status = STROBOT.SCORE;}
                    break;
                }

                case SCORE:
                {
                    if(blue_left.CurrentStatus == Blue_LEFT.autoControllerStatus.SCORE_DONE)
                    {
                        leftLatch.CS = leftLatch_Controller.leftLatchStatus.OPEN;
                        rightLatch.CS = rightLatch_Controller.rightLatchStatus.OPEN;
                        status = STROBOT.CHECK_COLLECT;
                    }
                    break;
                }

                case CHECK_COLLECT:
                {
                    if(nrcicluri < 1)
                    {
                        nrcicluri += 1;
                        collectAngle.stack_level -= nrcicluri*2;
                        outtake.CS = outtake_Controller.outtakeStatus.INITIALIZE;
                        score.reset();
                        status = STROBOT.GO_TO_STACK;
                    } else
                    {
                        collect.reset();
                        status = STROBOT.PARK;
                    }
                    break;
                }

                case PARK:
                {
                    if(collect.seconds() > 0.05)
                    {
                        blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.COLLECT;
                    }
                    if(collect.seconds() > 0.65)
                    {
                        drive.followTrajectorySequenceAsync(PARK_FROM_RIGHT);
                    }
                    break;
                }

            }



            lift.update(r, position, currentVoltage);
            fourbar.update(r);
            collectAngle.update(r);
            leftLatch.update(r);
            rightLatch.update(r);
            storage.update(r);
            storageAngle.update(r);
            outtake.update(fourbar, storage, storageAngle, lift);
            blue_left.update(r, lift, ik, fourbar, storage, storageAngle);
            pto.update(r);
            drive.update();

            telemetry.addData("status", status);
            telemetry.addData("autostatus", blue_left.CurrentStatus);
            telemetry.addData("left_latch", leftLatch.CS);
            telemetry.addData("ik", fourbar.inverse_kinematics);
            telemetry.addData("4bar", fourbar.CS);
            telemetry.addData("left_pixel", r.left_pixel.getState());
            telemetry.update();

        }

    }
}

