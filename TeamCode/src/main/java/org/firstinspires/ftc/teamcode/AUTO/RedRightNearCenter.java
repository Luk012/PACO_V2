package org.firstinspires.ftc.teamcode.AUTO;

import static org.firstinspires.ftc.teamcode.AUTO.BlueLeftNearCenter.STROBOT.NOTHING;
import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AUTO.Recognition.BlueOpenCVMaster;
import org.firstinspires.ftc.teamcode.AUTO.Recognition.OpenCVMaster;
import org.firstinspires.ftc.teamcode.AUTO_CONTROLLERS.Blue_LEFT;

import org.firstinspires.ftc.teamcode.RoadRunner.DriveConstants;
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

import java.util.List;

@Config
@Autonomous(group = "Auto" , name = "RedRightNearCenter")

public class RedRightNearCenter extends LinearOpMode {

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
        NOTHING,
        SYSTEMS,
    }

    public static double x_start = 16, y_start = -62, angle_start = 90;
    public static double x_purple_left = 25.5, y_purple_left = -39, angle_purple_left = 90;
    public static double x_purple_center = 17, y_purple_center = -31.5, angle_purple_center = 90;
    public static double x_purple_right = 11.2, y_purple_right = -27.5, angle_purple_right = 180;
    public static double x_yellow_left = 45.5, y_yellow_left = -41, angle_yellow_left = 180;
    public static double x_yellow_center = 46, y_yellow_center = -28, angle_yellow_center = 180;
    public static double x_yellow_right = 47.5, y_yellow_right = -28, angle_yellow_right = 180;
    public static double x_yellow_right2 = 48, y_yellow_right2 = -35.5, angle_yellow_right2 = 180;

    public static double x_stack = -59.5, y_stack = -4, angle_stack = 180;
    public static double x_interstack = -5, y_inetrstack = -4 , angle_interstack = 180;
    public static double x_prepare_for_stack = 27.5, y_prepare_for_stack = -4, angle_prepare = 180;
    public static double x_lung_de_linie = -25, y_lung_de_linie = -59, angle_lung_de_linie = 180;
    public static double x_park_from_right = 48, y_park_from_right = -62, angle_park_from_right = 180;

    int caz = 0;
    boolean ok  = FALSE;
    boolean ok2 = FALSE;

    @Override
    public void runOpMode() throws InterruptedException {

        OpenCVMaster redRight = new OpenCVMaster(this);
        redRight.observeStick();

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
        fourBar_Controller fourbar = new fourBar_Controller();
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
        lift.CS = lift_Controller.liftStatus.DOWN_AUTO;
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
        outtake.update( fourbar, storage, storageAngle, lift, rightLatch,leftLatch);
        blue_left.update(r, lift, ik, fourbar, storage, storageAngle);
        pto.update(r);

        Pose2d start_pose = new Pose2d(x_start, y_start,Math.toRadians(angle_start));
        Pose2d purple_left = new Pose2d(x_purple_left-2, y_purple_left, Math.toRadians(angle_purple_left));
        Pose2d purple_center = new Pose2d(x_purple_center, y_purple_center - 1.3, Math.toRadians(angle_purple_center));
        Pose2d purple_right = new Pose2d(x_purple_right -1, y_purple_right, Math.toRadians(angle_purple_right));
        Pose2d yellow_left = new Pose2d(x_yellow_left +1, y_yellow_left, Math.toRadians(angle_yellow_left));
        Pose2d stack = new Pose2d(x_stack, y_stack - 1.5, Math.toRadians(angle_stack));
        Pose2d stackright = new Pose2d(x_stack+0.25, y_stack-2, Math.toRadians(angle_stack));

        Pose2d inter = new Pose2d(x_interstack, y_inetrstack, Math.toRadians(angle_interstack));
        Pose2d interright = new Pose2d(x_interstack, y_inetrstack, Math.toRadians(angle_interstack));


        Pose2d prepare_for_stack = new Pose2d(x_prepare_for_stack, y_prepare_for_stack-2, Math.toRadians(angle_prepare));
        Pose2d prepare_for_stackright = new Pose2d(x_prepare_for_stack, y_prepare_for_stack-2, Math.toRadians(angle_prepare));

        Pose2d lung_de_linie = new Pose2d(x_lung_de_linie - 4, y_lung_de_linie-5.5, Math.toRadians(angle_lung_de_linie));

        Pose2d yellow_right = new Pose2d(x_yellow_right -1, y_yellow_right, Math.toRadians(angle_yellow_right));
        Pose2d yellow_right2 = new Pose2d(x_yellow_right2 -5, y_yellow_right2+1, Math.toRadians(angle_yellow_right2));
        Pose2d yellow_right3 = new Pose2d(x_yellow_right2 -3.5, y_yellow_right2+1, Math.toRadians(angle_yellow_right2));
        Pose2d yellow_right4 = new Pose2d(x_yellow_right2-3.5 , y_yellow_right2+3, Math.toRadians(angle_yellow_right2));
        Pose2d yellow_right5 = new Pose2d(x_yellow_right2-4 , y_yellow_right2+3, Math.toRadians(angle_yellow_right2));
        Pose2d yellow_right6 = new Pose2d(x_yellow_right -2, y_yellow_right-8, Math.toRadians(angle_yellow_right));


        Pose2d park_from_right = new Pose2d(x_park_from_right, y_park_from_right, Math.toRadians(angle_park_from_right));
        Pose2d yellow_center = new Pose2d(x_yellow_center +1, y_yellow_center-5, Math.toRadians(angle_yellow_center));
        Pose2d park_from_left = new Pose2d(x_park_from_right, y_park_from_right, Math.toRadians(angle_park_from_right));

        Pose2d prepare_for_stack_score = new Pose2d(x_prepare_for_stack - 25, y_prepare_for_stack, Math.toRadians(angle_prepare));
        Pose2d prepare_for_stack_scoreright = new Pose2d(x_prepare_for_stack - 25, y_prepare_for_stack-2, Math.toRadians(angle_prepare));

        Pose2d lung_de_linie_score = new Pose2d(x_lung_de_linie-4, y_lung_de_linie+5.5, Math.toRadians(angle_lung_de_linie));

        Pose2d lung_de_linie_2 = new Pose2d(x_lung_de_linie -4.5, y_lung_de_linie-2.5, Math.toRadians(angle_lung_de_linie));
        Pose2d prepare_for_stack_2 = new Pose2d(x_prepare_for_stack, y_prepare_for_stack, Math.toRadians(angle_prepare));

        TrajectorySequence PURPLE_LEFT = drive.trajectorySequenceBuilder(start_pose)
                .lineToLinearHeading(purple_left)
                .build();

        TrajectorySequence PURPLE_CENTER = drive.trajectorySequenceBuilder(start_pose)
                .lineToLinearHeading(purple_center)
                .build();

        TrajectorySequence PURPLE_RIGHT = drive.trajectorySequenceBuilder(start_pose)
                .splineToLinearHeading(purple_right, Math.toRadians(180))
                .build();

        TrajectorySequence YELLOW_LEFT = drive.trajectorySequenceBuilder(purple_left)
                .lineToLinearHeading(yellow_left)
                .build();

        TrajectorySequence YELLOW_CENTER = drive.trajectorySequenceBuilder(purple_center)
                .lineToLinearHeading(yellow_center)
                .build();

        TrajectorySequence YELLOW_RIGHT = drive.trajectorySequenceBuilder(purple_right)
                .lineToLinearHeading(yellow_right)
                .build();

        TrajectorySequence GO_STACK_LEFT = drive.trajectorySequenceBuilder(yellow_left)
                .setTangent(90)
                .splineToLinearHeading(prepare_for_stack, Math.toRadians(180))
                .lineToLinearHeading(inter)
                //.lineToLinearHeading(lung_de_linie)
                .lineToLinearHeading(

                        stack,
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )

                //.lineToLinearHeading(stack)
//                .lineToLinearHeading(stack)
//                .lineToLinearHeading(prepare_for_stack)
//                .lineToLinearHeading(lung_de_linie)
//                .splineToLinearHeading(stack,Math.toRadians(180))
//                .setTangent(Math.toRadians(0))
                .build();

        TrajectorySequence GO_STACK_CENTER = drive.trajectorySequenceBuilder(yellow_center)
                .setTangent(90)
                .splineToLinearHeading(prepare_for_stack, Math.toRadians(180))
                .lineToLinearHeading(inter)
                //.lineToLinearHeading(lung_de_linie)
                .lineToLinearHeading(

                        stack,
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
//                .lineToLinearHeading(prepare_for_stack)
//                .lineToLinearHeading(lung_de_linie)
//                .splineToLinearHeading(stack, Math.toRadians(180))
                .build();

        TrajectorySequence GO_STACK_CENTER_2 = drive.trajectorySequenceBuilder(yellow_right)
                .setTangent(90)
                .splineToLinearHeading(prepare_for_stack_2, Math.toRadians(180))
                //.lineToLinearHeading(lung_de_linie_2)
                .lineToLinearHeading(stack)
//                .lineToLinearHeading(prepare_for_stack)
//                .lineToLinearHeading(lung_de_linie)
//                .splineToLinearHeading(stack, Math.toRadians(180))
                .build();

        TrajectorySequence GO_STACK_RIGHT = drive.trajectorySequenceBuilder(yellow_right)
                .setTangent(90)
                .splineToLinearHeading(prepare_for_stackright, Math.toRadians(180))
                .lineToLinearHeading(interright)
                //.lineToLinearHeading(lung_de_linie)
                .lineToLinearHeading(

                        stackright,
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
//                .lineToLinearHeading(prepare_for_stack)
//                .lineToLinearHeading(lung_de_linie)
//                .splineToLinearHeading(stack,Math.toRadians(180))
                .build();

        TrajectorySequence SCORE_LEFT = drive.trajectorySequenceBuilder(stack)

//                .splineToLinearHeading(lung_de_linie,Math.toRadians(180))
//                .lineToLinearHeading(prepare_for_stack)
//                .lineToLinearHeading(yellow_right)
//                .setTangent(Math.toRadians(0))
                .setTangent(70)
                //.splineToLinearHeading(lung_de_linie_score, Math.toRadians(0))
                .lineToLinearHeading(prepare_for_stack_score)
                .splineToLinearHeading(yellow_right4, Math.toRadians(0))
                //.splineToLinearHeading(yellow_right, Math.toRadians(270))
//                .lineToLinearHeading(lung_de_linie)
//                .splineToLinearHeading(prepare_for_stack, Math.toRadians(180))
//                .splineToLinearHeading(yellow_right, Math.toRadians(180))
                //.setTangent(Math.toRadians(0))
                .build();

        TrajectorySequence SCORE_CENTER = drive.trajectorySequenceBuilder(stack)

//                .splineToLinearHeading(lung_de_linie,Math.toRadians(180))
//                .lineToLinearHeading(prepare_for_stack)
//                .lineToLinearHeading(yellow_right)
//                .setTangent(Math.toRadians(0))
                .setTangent(70)
                //.splineToLinearHeading(lung_de_linie_score, Math.toRadians(0))
                .lineToLinearHeading(prepare_for_stack_score)
                .splineToLinearHeading(yellow_right6, Math.toRadians(0))
                //.splineToLinearHeading(yellow_right, Math.toRadians(270))
//                .lineToLinearHeading(lung_de_linie)
//                .splineToLinearHeading(prepare_for_stack, Math.toRadians(180))
//                .splineToLinearHeading(yellow_right, Math.toRadians(180))
                //.setTangent(Math.toRadians(0))
                .build();

        TrajectorySequence SCORE_RIGHT = drive.trajectorySequenceBuilder(stackright)
                .setTangent(70)
                // .splineToLinearHeading(lung_de_linie_score, Math.toRadians(0))
                .lineToLinearHeading(prepare_for_stack_scoreright)
                .splineToLinearHeading(yellow_right3, Math.toRadians(0))
                //.splineToLinearHeading(yellow_right, Math.toRadians(270))
                .build();

        TrajectorySequence PARK_FROM_RIGHT = drive.trajectorySequenceBuilder(yellow_right)
                .forward(4)
                .lineToLinearHeading(park_from_right)
                .build();

        TrajectorySequence PARK_FROM_LEFT = drive.trajectorySequenceBuilder(yellow_left)
                .forward(4)
                .lineToLinearHeading(park_from_left)
                .build();

        drive.setPoseEstimate(start_pose);
        STROBOT status = STROBOT.START;

        ElapsedTime collect = new ElapsedTime();
        ElapsedTime score = new ElapsedTime();
        ElapsedTime preload = new ElapsedTime();
        ElapsedTime preload2 = new ElapsedTime();
        ElapsedTime collect2= new ElapsedTime();

        double nrcicluri = 0;
        collectAngle.stack_level = 4;
        lift.upCnt = 0;

        while (!isStarted() && !isStopRequested()) {

            sleep(20);
            if(redRight.opencv.getWhichSide() == "left"){
                caz = 0;
            } else if (redRight.opencv.getWhichSide() == "center") {
                caz = 1;
            } else {
                caz = 2;
            }
            telemetry.addData("case", redRight.opencv.getWhichSide());
            telemetry.update();
            sleep(50);
        }

        waitForStart();

        String redRightCase = redRight.opencv.getWhichSide();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            int position = r.lift.getCurrentPosition();


            switch (status) {

                case START: {
                    if(redRightCase == "left"){
                        collectAngle_Controller.CS = collectAngle_Controller.collectAngleStatus.INITIALIZE;
                        drive.followTrajectorySequenceAsync(PURPLE_RIGHT);
                    } else if(redRightCase == "center"){
                        collectAngle_Controller.CS = collectAngle_Controller.collectAngleStatus.INITIALIZE;
                        drive.followTrajectorySequenceAsync(PURPLE_CENTER);
                    } else {
                        collectAngle_Controller.CS = collectAngle_Controller.collectAngleStatus.INITIALIZE;
                        drive.followTrajectorySequenceAsync(PURPLE_LEFT);
                    }
                    preload.reset();
                    status = STROBOT.PURPLE_DROP;
                    break;
                }

                case PURPLE_DROP: {
                    if (!drive.isBusy() /*|| preload.seconds() > 0.85*/) {
                        collectAngle_Controller.CS = collectAngle_Controller.collectAngleStatus.LIFTED;
                        if(redRightCase == "left"){
                            drive.followTrajectorySequenceAsync(YELLOW_RIGHT);
                        } else if(redRightCase == "center"){
                            drive.followTrajectorySequenceAsync(YELLOW_CENTER);
                        } else {
                            drive.followTrajectorySequenceAsync(YELLOW_LEFT);
                        }
                        blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.INTER;
                        preload2.reset();
                        status = STROBOT.YELLOW;
                    }
                    break;
                }

                case YELLOW: {
                    if (!drive.isBusy() /*preload2.seconds() > 1.05*/) {
                        blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.SCORE_PRELOAD;
                        if(redRightCase == "left"){
                            storageAngle.CS = storageAngle_Controller.storageAngleStatus.ROTATION;
                            storageAngle.rotation_i = 0;
                        } else if(redRightCase == "center"){
                            storageAngle.CS = storageAngle_Controller.storageAngleStatus.ROTATION;
                            storageAngle.rotation_i = 2;
                        } else {
                            storageAngle.CS = storageAngle_Controller.storageAngleStatus.ROTATION;
                            storageAngle.rotation_i = 1;
                        }
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
                    if(score.seconds() > 0.25) {
                        if(redRightCase == "left"){
                            drive.followTrajectorySequenceAsync(GO_STACK_LEFT);
                        } else if (redRightCase == "center") {
                            drive.followTrajectorySequenceAsync(GO_STACK_CENTER);
                        } else {
                            drive.followTrajectorySequenceAsync(GO_STACK_RIGHT);
                        }
                        collect.reset();
                        status = STROBOT.PREPARE_COLLECT;
                    }
                    break;
                }




                case PREPARE_COLLECT:
                {
                    if(collect.seconds() > 0.35)
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
                        ok = FALSE;
                        ok2 = FALSE;
                        status = STROBOT.VERIF;
                    }
                    break;
                }

                case VERIF:
                {
                    if(ok == FALSE && (r.right_pixel.getState() == FALSE || r.left_pixel.getState() == FALSE))
                    {
                        collectAngle.stack_level = Math.max(0, collectAngle.stack_level-1);
                        ok = TRUE;
                    }
                    if(collect.seconds() > 1.2 && (r.right_pixel.getState() == TRUE || r.left_pixel.getState() == TRUE) && ok2 == FALSE)
                    {
                        collectAngle.stack_level = Math.max(0, collectAngle.stack_level-1);
                        ok2 = TRUE;
                    }
                    if(r.left_pixel.getState() == FALSE && leftLatch.CS == leftLatch_Controller.leftLatchStatus.OPEN)
                    {
                        leftLatch.CS = leftLatch_Controller.leftLatchStatus.CLOSE;
                    }

                    if(r.right_pixel.getState() == FALSE && rightLatch.CS == rightLatch_Controller.rightLatchStatus.OPEN)
                    {
                        rightLatch.CS = rightLatch_Controller.rightLatchStatus.CLOSE;
                    }

                    if(nrcicluri <2)
                    { if(leftLatch.CS == leftLatch_Controller.leftLatchStatus.CLOSE_DONE && rightLatch.CS == rightLatch_Controller.rightLatchStatus.CLOSE_DONE || collect.seconds() > 2)
                    {
                        r.collect.setPower(-0.5);
                        //  outtake.CS = outtake_Controller.outtakeStatus.INTER;
                        status = STROBOT.GO_SCORE;
                    }} else
                    {
                        if(leftLatch.CS == leftLatch_Controller.leftLatchStatus.CLOSE_DONE || rightLatch.CS == rightLatch_Controller.rightLatchStatus.CLOSE_DONE || collect.seconds() > 1.1)
                        {
                            r.collect.setPower(-0.5);
                            //  outtake.CS = outtake_Controller.outtakeStatus.INTER;
                            status = STROBOT.GO_SCORE;
                        }
                    }
                    break;
                }

                case GO_SCORE:
                {
                    r.collect.setPower(-0.5);
                    if(redRightCase == "left"){
                        drive.followTrajectorySequenceAsync(SCORE_LEFT);
                        lift.upCnt += 2;
                    } else if(redRightCase == "center"){
                        drive.followTrajectorySequenceAsync(SCORE_CENTER);
                        lift.upCnt += 2;
                    } else {
                        drive.followTrajectorySequenceAsync(SCORE_RIGHT);
                        lift.upCnt += 2;
                    }

                    collectAngle.CS = collectAngle_Controller.collectAngleStatus.LIFTED;
                    collectAngle.stack_level -=1;
                    score.reset();
                    status = STROBOT.SYSTEMS;
                    break;
                }

                case SYSTEMS:
                {
                    if(score.seconds() > 1.45)
                    {
                        blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.SCORE;
                        status = STROBOT.PREPARE_FOR_SCORE;
                    }
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
                        //collectAngle.stack_level -= nrcicluri*2;
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
                    if(collect.seconds() > 0.3)
                    {
                        if(redRightCase == "left"){
                            drive.followTrajectorySequenceAsync(PARK_FROM_RIGHT);
                        } else if(redRightCase == "center"){
                            drive.followTrajectorySequenceAsync(PARK_FROM_RIGHT);
                        } else {
                            drive.followTrajectorySequenceAsync(PARK_FROM_LEFT);
                        }
                    }
                    if(collect.seconds() > 0.4)
                    {
                        fourbar.CS= fourBar_Controller.fourbarStatus.INTER;
                        storage.CS = storage_Controller.storageStatus.INTER;
                        lift.pid =0 ;
                        lift.CS = lift_Controller.liftStatus.DOWN;
                        status = STROBOT.NOTHING;

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
            outtake.update( fourbar, storage, storageAngle, lift, rightLatch,leftLatch);
            blue_left.update(r, lift, ik, fourbar, storage, storageAngle);
            pto.update(r);
            drive.update();

            telemetry.addData("status", status);
            telemetry.addData("autostatus", blue_left.CurrentStatus);
            telemetry.addData("left_latch", leftLatch.CS);
            telemetry.addData("ik", fourbar.inverse_kinematics);
            telemetry.addData("4bar", fourbar.CS);
            telemetry.addData("stack", collectAngle.stack_level);
            telemetry.addData("left_pixel", r.left_pixel.getState());
            telemetry.update();

        }

    }
}
