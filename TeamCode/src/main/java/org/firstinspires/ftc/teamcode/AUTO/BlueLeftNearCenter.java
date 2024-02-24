package org.firstinspires.ftc.teamcode.AUTO;

import static org.firstinspires.ftc.teamcode.AUTO.BlueLeftNearCenter.STROBOT.NOTHING;
import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AUTO.Recognition.BlueOpenCVMaster;
import org.firstinspires.ftc.teamcode.AUTO_CONTROLLERS.Blue_LEFT;

import org.firstinspires.ftc.teamcode.DistanceSensorCalibrator;
import org.firstinspires.ftc.teamcode.LowPassFilter;
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
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;

import java.util.List;

@Config
@Autonomous(group = "Auto" , name = "lucanomia")

public class BlueLeftNearCenter extends LinearOpMode {

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
        GO_STACK_2,
        SYSTEMS,
        SYNC_PATH,
        SYSTEMS_PRELOAD,
        SYNC_PATH_PRELOAD,
        PREPARE_TO_DROP_YELLOW,
        NOTHING,
    }

    public static double x_start = 16, y_start = 62, angle_start = 270;
    public static double x_purple_left = 27, y_purple_left = 39, angle_purple_left = 270;
    public static double x_purple_center = 13.5, y_purple_center = 34, angle_purple_center = 270;
    public static double x_purple_right = 9.5, y_purple_right = 27.5, angle_purple_right = 180;
    public static double x_yellow_left = 35, y_yellow_left = 38.5, angle_yellow_left = 180;
    public static double x_yellow_center = 35, y_yellow_center = 38, angle_yellow_center = 180;
    public static double x_yellow_right = 35, y_yellow_right = 32, angle_yellow_right = 180;
    public static double x_stack = -58.5, y_stack = 7, angle_stack = 180;
    public static double x_interstack = -25, y_inetrstack = 7 , angle_interstack = 180;
    public static double x_prepare_for_stack = 35, y_prepare_for_stack = 7, angle_prepare = 180;
    public static double x_lung_de_linie = -25, y_lung_de_linie = 59, angle_lung_de_linie = 180;
    public static double x_park_from_right = 48, y_park_from_right = 62, angle_park_from_right = 180;

    int caz = 0;
    boolean ok  = FALSE;
    boolean ok2 = FALSE;
    double loopTime;

    @Override
    public void runOpMode() throws InterruptedException {

        BlueOpenCVMaster blueLeft = new BlueOpenCVMaster(this);
        blueLeft.observeStick();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DistanceSensorCalibrator calibrator;
        robotMap r = new robotMap(hardwareMap);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        LowPassFilter l = new LowPassFilter(0.9, r.back.getDistance(DistanceUnit.CM));

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
        blue_left.update(r, lift, ik, fourbar, storage, storageAngle, leftLatch, rightLatch);
        pto.update(r);

        Pose2d start_pose = new Pose2d(x_start, y_start,Math.toRadians(angle_start));
        Pose2d purple_left = new Pose2d(x_purple_left-2, y_purple_left, Math.toRadians(angle_purple_left));
        Pose2d purple_left_senzor = new Pose2d(x_purple_left+3, y_purple_left, Math.toRadians(180));
        Pose2d purple_center = new Pose2d(x_purple_center, y_purple_center - 1, Math.toRadians(angle_purple_center));
        Pose2d purple_center_senzor = new Pose2d(x_purple_center+12, y_purple_center - 1, Math.toRadians(180));
        Pose2d purple_right = new Pose2d(x_purple_right +1, y_purple_right, Math.toRadians(angle_purple_right));
        Pose2d purple_right_senzor = new Pose2d(x_purple_right +15, y_purple_right, Math.toRadians(180));
        Pose2d yellow_left = new Pose2d(x_yellow_left + 3, y_yellow_left+0.5, Math.toRadians(angle_yellow_left));

        Pose2d stack = new Pose2d(x_stack, y_stack, Math.toRadians(angle_stack));


        Pose2d inter = new Pose2d(x_interstack, y_inetrstack, Math.toRadians(angle_interstack));

        Pose2d prepare_for_stack = new Pose2d(x_prepare_for_stack, y_prepare_for_stack, Math.toRadians(angle_prepare));
        Pose2d lung_de_linie = new Pose2d(x_lung_de_linie - 4, y_lung_de_linie-5.5, Math.toRadians(angle_lung_de_linie));

        Pose2d yellow_right = new Pose2d(x_yellow_right, y_yellow_right -2.5, Math.toRadians(angle_yellow_right));
        Pose2d yellow_right2 = new Pose2d(x_yellow_right, y_yellow_right , Math.toRadians(angle_yellow_right));
        Pose2d yellow_right3 = new Pose2d(x_yellow_right, y_yellow_right, Math.toRadians(angle_yellow_right));
        Pose2d yellow_right4 = new Pose2d(x_yellow_right, y_yellow_right +1, Math.toRadians(angle_yellow_right));


        Pose2d park_from_right = new Pose2d(x_park_from_right, y_park_from_right, Math.toRadians(angle_park_from_right));
        Pose2d yellow_center = new Pose2d(x_yellow_center, y_yellow_center-8, Math.toRadians(angle_yellow_center));
        Pose2d park_from_left = new Pose2d(x_park_from_right, y_park_from_right, Math.toRadians(angle_park_from_right));

        Pose2d prepare_for_stack_score = new Pose2d(x_prepare_for_stack, y_prepare_for_stack, Math.toRadians(angle_prepare));
        Pose2d lung_de_linie_score = new Pose2d(x_lung_de_linie-4, y_lung_de_linie+5.5, Math.toRadians(angle_lung_de_linie));

        Pose2d lung_de_linie_2 = new Pose2d(x_lung_de_linie -4.5, y_lung_de_linie-2.5, Math.toRadians(angle_lung_de_linie));
        Pose2d prepare_for_stack_2 = new Pose2d(x_prepare_for_stack, y_prepare_for_stack, Math.toRadians(angle_prepare));


        TrajectorySequence PURPLE_LEFT = drive.trajectorySequenceBuilder(start_pose)
                .lineToLinearHeading(purple_left)
                .build();

        TrajectorySequence TRAJ = drive.trajectorySequenceBuilder(yellow_right3)
                .back(30,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence PRELOAD_LEFT_SENSOR = drive.trajectorySequenceBuilder(purple_left_senzor)
                .back(30,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence PRELOAD_CENTER_SENSOR = drive.trajectorySequenceBuilder(purple_center_senzor)
                .back(30,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence PRELOAD_RIGHT_SENSOR = drive.trajectorySequenceBuilder(purple_right_senzor)
                .back(30,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence TRAJ1 = drive.trajectorySequenceBuilder(yellow_right3)
                .back(20,
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
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
                .lineToLinearHeading(prepare_for_stack)
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
                .lineToLinearHeading(prepare_for_stack)
                .lineToLinearHeading(inter)
                //.lineToLinearHeading(lung_ de_linie)
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
                .lineToLinearHeading(prepare_for_stack_2)
                //.lineToLinearHeading(lung_de_linie_2)
                .lineToLinearHeading(stack)
//                .lineToLinearHeading(prepare_for_stack)
//                .lineToLinearHeading(lung_de_linie)
//                .splineToLinearHeading(stack, Math.toRadians(180))
                .build();

        TrajectorySequence GO_STACK_RIGHT = drive.trajectorySequenceBuilder(yellow_right)
                .lineToLinearHeading(prepare_for_stack)
                .lineToLinearHeading(inter)
                //.lineToLinearHeading(lung_de_linie)
                .lineToLinearHeading(

                        stack,
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
//                .lineToLinearHeading(prepare_for_stack)
//                .lineToLinearHeading(lung_de_linie)
//                .splineToLinearHeading(stack,Math.toRadians(180))
                .build();

        TrajectorySequence SCORE_CENTER = drive.trajectorySequenceBuilder(stack)

//                .splineToLinearHeading(lung_de_linie,Math.toRadians(180))
//                .lineToLinearHeading(prepare_for_stack)
//                .lineToLinearHeading(yellow_right)
//                .setTangent(Math.toRadians(0))
                //.splineToLinearHeading(lung_de_linie_score, Math.toRadians(0))
                .lineToLinearHeading(prepare_for_stack_score)
                .lineToLinearHeading(yellow_right2)
                //.splineToLinearHeading(yellow_right, Math.toRadians(270))
//                .lineToLinearHeading(lung_de_linie)
//                .splineToLinearHeading(prepare_for_stack, Math.toRadians(180))
//                .splineToLinearHeading(yellow_right, Math.toRadians(180))
                //.setTangent(Math.toRadians(0))
                .build();

        TrajectorySequence SCORE_LEFT = drive.trajectorySequenceBuilder(stack)

//
                .lineToLinearHeading(prepare_for_stack_score)
                .lineToLinearHeading(yellow_right3)

                .build();

        TrajectorySequence SCORE_RIGHT = drive.trajectorySequenceBuilder(stack)
                // .splineToLinearHeading(lung_de_linie_score, Math.toRadians(0))
                .lineToLinearHeading(prepare_for_stack_score)
                .lineToLinearHeading(yellow_right4)
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
        ElapsedTime preload_sensor = new ElapsedTime();

        double nrcicluri = 0;
        collectAngle.stack_level = 4;
        lift.upCnt = 0;

        while (!isStarted() && !isStopRequested()) {

            sleep(20);
            if(blueLeft.opencv2.getWhichSide() == "left"){
                caz = 0;
            } else if (blueLeft.opencv2.getWhichSide() == "center") {
                caz = 1;
            } else {
                caz = 2;
            }
            telemetry.addData("case", blueLeft.opencv2.getWhichSide());
            telemetry.update();
            sleep(50);
        }

        waitForStart();

        String blueLeftCase = blueLeft.opencv2.getWhichSide();
        double[] rawReadings = {26.3, 25.5, 26, 24.4, 26.6, 25, 26.9, 25.8, 26.1, 26.5, 27.1, 25.6, 26, 26.1, 26.3, 25.9, 26.4, 25.5, 24.7, 26.9};
        double[] actualDistances = {26, 25, 25.5, 24, 26.5, 24.5, 27, 25.4, 25.7, 26.2, 26.6, 24.8, 25.3, 25.9, 26.1, 25.8, 26.3, 25.1, 24.9, 26.8};
        calibrator = new DistanceSensorCalibrator(rawReadings, actualDistances);

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            int position = r.lift.getCurrentPosition();


            switch (status) {

                case START: {
                    if(blueLeftCase == "left"){
                        collectAngle_Controller.CS = collectAngle_Controller.collectAngleStatus.INITIALIZE;
                        drive.followTrajectorySequenceAsync(PURPLE_LEFT);
                    } else if(blueLeftCase == "center"){
                        collectAngle_Controller.CS = collectAngle_Controller.collectAngleStatus.INITIALIZE;
                        drive.followTrajectorySequenceAsync(PURPLE_CENTER);
                    } else {
                        collectAngle_Controller.CS = collectAngle_Controller.collectAngleStatus.INITIALIZE;
                        drive.followTrajectorySequenceAsync(PURPLE_RIGHT);
                    }
                    preload.reset();
                    status = STROBOT.PURPLE_DROP;
                    break;
                }

                case SYNC_PATH_PRELOAD:
                {
                    if(blueLeftCase == "left"){
                        drive.followTrajectorySequenceAsync(PRELOAD_LEFT_SENSOR);
                    } else if(blueLeftCase == "center") {
                        drive.followTrajectorySequenceAsync(PRELOAD_CENTER_SENSOR);
                    } else {
                        drive.followTrajectorySequenceAsync(PRELOAD_RIGHT_SENSOR);
                    }
                    status = STROBOT.PREPARE_TO_DROP_YELLOW;
                    break;
                }

                case SYSTEMS_PRELOAD:
                {
                    if(!drive.isBusy())
                    {
                        blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.SCORE_PRELOAD;
                        if(blueLeftCase == "left"){
                            storageAngle.CS = storageAngle_Controller.storageAngleStatus.ROTATION;
                            storageAngle.rotation_i = 0;
                        } else if(blueLeftCase == "center"){
                            storageAngle.CS = storageAngle_Controller.storageAngleStatus.ROTATION;
                            storageAngle.rotation_i = 0; //2
                        } else {
                            storageAngle.CS = storageAngle_Controller.storageAngleStatus.ROTATION;
                            storageAngle.rotation_i = 2; //4
                        }
                        status = STROBOT.SYNC_PATH_PRELOAD;
                    }
                    break;
                }

                case PREPARE_TO_DROP_YELLOW:
                {
                    double rawReading = r.back.getDistance(DistanceUnit.CM);
                    double calibratedDistance = calibrator.calibrate(rawReading);

                    if(calibratedDistance <= 21.5) {
                        status = STROBOT.YELLOW_DROP;
                    }
                    break;
                }

                case PURPLE_DROP: {
                    if (!drive.isBusy() /*|| preload.seconds() > 0.85*/) {
                        collectAngle_Controller.CS = collectAngle_Controller.collectAngleStatus.LIFTED;
                        if(blueLeftCase == "left"){
                            drive.followTrajectorySequenceAsync(YELLOW_LEFT);
                        } else if(blueLeftCase == "center"){
                            drive.followTrajectorySequenceAsync(YELLOW_CENTER);
                        } else {
                            drive.followTrajectorySequenceAsync(YELLOW_RIGHT);
                        }
                        blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.INTER;
                        preload2.reset();
                        preload_sensor.reset();
                        status = STROBOT.SYSTEMS_PRELOAD;
                    }
                    break;
                }

                case YELLOW: {
                    if (!drive.isBusy() /*preload2.seconds() > 1.05*/) {
                        blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.SCORE_PRELOAD;
                        if(blueLeftCase == "left"){
                            storageAngle.CS = storageAngle_Controller.storageAngleStatus.ROTATION;
                            storageAngle.rotation_i = 0;
                        } else if(blueLeftCase == "center"){
                            storageAngle.CS = storageAngle_Controller.storageAngleStatus.ROTATION;
                            storageAngle.rotation_i = 2;
                        } else {
                            storageAngle.CS = storageAngle_Controller.storageAngleStatus.ROTATION;
                            storageAngle.rotation_i = 4;
                        }
                        status = STROBOT.YELLOW_DROP;
                    }
                    break;
                }

                case YELLOW_DROP: {

                    leftLatch_Controller.CS = leftLatch_Controller.leftLatchStatus.OPEN;
                    rightLatch_Controller.CS = rightLatch_Controller.rightLatchStatus.OPEN;
                    score.reset();
                    status = status.GO_TO_STACK;

                    break;
                }

                case GO_TO_STACK: {

                    if(blueLeftCase == "left"){
                        drive.followTrajectorySequenceAsync(GO_STACK_LEFT);
                    } else if (blueLeftCase == "center") {
                        drive.followTrajectorySequenceAsync(GO_STACK_CENTER);
                    } else {
                        drive.followTrajectorySequenceAsync(GO_STACK_RIGHT);
                    }
                    collect.reset();
                    status = STROBOT.PREPARE_COLLECT;
                    break;
                }

                case GO_STACK_2: {
                    if(score.seconds() > 0.01) {
                        if(blueLeftCase == "left"){
                            drive.followTrajectorySequenceAsync(GO_STACK_LEFT);
                        } else if (blueLeftCase == "center") {
                            drive.followTrajectorySequenceAsync(GO_STACK_CENTER_2);
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
                    if(collect.seconds() > 0.4)
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
                    if (/*!drive.isBusy() || 4.5, 2.3*/ collect2.seconds() > 3.5) {
                        r.collect.setPower(1);
                        collectAngle.CS = collectAngle_Controller.collectAngleStatus.STACK;
                        collect.reset();
                        ok = FALSE;
                        ok2 = FALSE;
                        if(nrcicluri == 0)
                        { collectAngle.stack_level =3;} else { collectAngle.stack_level =1;}
                        status = STROBOT.VERIF;
                    }
                    break;
                }

                case VERIF:
                {

                    if(r.left_pixel.getState() == FALSE && leftLatch.CS == leftLatch_Controller.leftLatchStatus.OPEN)
                    {
                        leftLatch.CS = leftLatch_Controller.leftLatchStatus.CLOSE_AUTO;
                    }

                    if(r.right_pixel.getState() == FALSE && rightLatch.CS == rightLatch_Controller.rightLatchStatus.OPEN)
                    {
                        rightLatch.CS = rightLatch_Controller.rightLatchStatus.CLOSE_AUTO;
                    }

                    if(nrcicluri <2)
                    { if(leftLatch.CS == leftLatch_Controller.leftLatchStatus.CLOSE_DONE && rightLatch.CS == rightLatch_Controller.rightLatchStatus.CLOSE_DONE || collect.seconds() > 4)
                    {
                        r.collect.setPower(-1);
                        //  outtake.CS = outtake_Controller.outtakeStatus.INTER;
                        status = STROBOT.GO_SCORE;
                    }} else
                    {
                        if(leftLatch.CS == leftLatch_Controller.leftLatchStatus.CLOSE_DONE || rightLatch.CS == rightLatch_Controller.rightLatchStatus.CLOSE_DONE || collect.seconds() > 3.5)
                        {
                            r.collect.setPower(-1);
                            //  outtake.CS = outtake_Controller.outtakeStatus.INTER;
                            status = STROBOT.GO_SCORE;
                        }
                    }
                    break;
                }

                case GO_SCORE:
                {
                    r.collect.setPower(-1);
                    if(blueLeftCase == "left"){
                        lift.upCnt += 2;
                        drive.followTrajectorySequenceAsync(SCORE_LEFT);
                    } else if(blueLeftCase == "center"){
                        lift.upCnt += 2;
                        drive.followTrajectorySequenceAsync(SCORE_CENTER);
                    } else {
                        lift.upCnt += 3;
                        drive.followTrajectorySequenceAsync(SCORE_RIGHT);
                    }

                    collectAngle.CS = collectAngle_Controller.collectAngleStatus.LIFTED;

                    score.reset();

                    status = STROBOT.SYSTEMS;
                    break;
                }

                case SYSTEMS:
                {
                    if(score.seconds() > 1.35)
                    {
                        blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.SCORE;
                        status = STROBOT.SYNC_PATH;
                    }
                    break;
                }

                case PREPARE_FOR_SCORE:
                {
                    double rawReading = r.back.getDistance(DistanceUnit.CM);
                    double calibratedDistance = calibrator.calibrate(rawReading);


                    if(nrcicluri==0){
                        if(calibratedDistance <= 23.6) {
                            r.collect.setPower(0);
                            status = STROBOT.SCORE;
                        }
                    }
                    if(nrcicluri>0) {
                        if (calibratedDistance <= 23.6) {
                            r.collect.setPower(0);
                            status = STROBOT.SCORE;
                        }
                    }
                    break;
                }

                case SYNC_PATH:
                {
                    if(!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(TRAJ);
                        status = STROBOT.PREPARE_FOR_SCORE;
                    }
                    break;
                }

                case SCORE:
                {
                    leftLatch.CS = leftLatch_Controller.leftLatchStatus.OPEN;
                    rightLatch.CS = rightLatch_Controller.rightLatchStatus.OPEN;
                    status = STROBOT.CHECK_COLLECT;
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
                    if(collect.seconds() > 0.15)
                    {
                        fourbar.CS = fourBar_Controller.fourbarStatus.COLLECT;
                    }
                    if(collect.seconds() > 0.25)
                    {
                        storage.CS = storage_Controller.storageStatus.COLLECT;
                    }
                    if(collect.seconds() > 0.01)
                    {
                        if(blueLeftCase == "left"){
                            drive.followTrajectorySequenceAsync(PARK_FROM_RIGHT);
                        } else if(blueLeftCase == "center"){
                            drive.followTrajectorySequenceAsync(PARK_FROM_RIGHT);
                        } else {
                            drive.followTrajectorySequenceAsync(PARK_FROM_LEFT);
                        }
                    }
                    if(collect.seconds() > 0.45)
                    {
                        lift.pid =0 ;
                        lift.CS = lift_Controller.liftStatus.DOWN;
                        status = NOTHING;

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
            blue_left.update(r, lift, ik, fourbar, storage, storageAngle, leftLatch, rightLatch);
            pto.update(r);
            drive.update();

            double loop = System.nanoTime();
            telemetry.addData("hz", 1000000000 / (loop - loopTime));
            telemetry.addData("status", status);
            telemetry.addData("autostatus", blue_left.CurrentStatus);
            telemetry.addData("left_latch", leftLatch.CS);
            telemetry.addData("stack level", collectAngle.stack_level);
            telemetry.addData("4bar", fourbar.CS);
            telemetry.addData("stack", collectAngle.stack_level);
            telemetry.addData("left_pixel", r.left_pixel.getState());
            telemetry.addData("distance", calibrator.calibrate(r.back.getDistance(DistanceUnit.CM)));
            loopTime = loop;
            telemetry.update();

        }

    }
}