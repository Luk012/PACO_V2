package org.firstinspires.ftc.teamcode.AUTO;
import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AUTO.Recognition.BluePipelineStackMaster;
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
@Autonomous(group = "Auto" , name = "alilbitoftrolling")

public class alilbitoftrolling extends LinearOpMode {

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
        GO_SCORE_YELLOW_PRELOAD_AND_PIXEL_FROM_STACK,
        INTERMEDIARY,
        NOTHING,
        INTERMEDIARY_2,
    }

    public static double x_start = -37, y_start = 62, angle_start = 270;
    public static double x_purple_left = 27, y_purple_left = 39, angle_purple_left = 270;
    public static double x_purple_center = 13.5, y_purple_center = 34, angle_purple_center = 270;
    public static double x_purple_right = 8, y_purple_right = 27.5, angle_purple_right = 180;

    public static double x_yellow_left = 46, y_yellow_left = 38.5, angle_yellow_left = 180;
    public static double x_yellow_center = 47, y_yellow_center = 38, angle_yellow_center = 180;
    public static double x_yellow_right = 47, y_yellow_right = 32, angle_yellow_right = 180;

    public static double x_score_right_ciclu_1 = -5, y_score_right_ciclu1 = 4.5, angle_score_right_ciculu1 = 180;
    public static double x_score_right_ciclu_2 = -5, y_score_right_ciclu2 = 4.5, angle_score_right_ciculu2 = 180;

    public static double x_score_center_ciclu_1 = -5, y_score_center_ciclu1 = -4.5, angle_score_center_ciculu1 = 180;
    public static double x_score_center_ciclu_2 = -5, y_score_center_ciclu2 = -4.5, angle_score_center_ciculu2 = 180;

    public static double x_score_left_ciclu_1 = -5, y_score_left_ciclu1 = -4.5, angle_score_left_ciculu1 = 180;
    public static double x_score_left_ciclu_2 = -5, y_score_left_ciclu2 = -4.5, angle_score_left_ciculu2 = 180;

    public static double x_collect_right_ciclu_1 = -25, y_collect_right_ciclu1 = 59, angle_collect_right_ciculu1 = 180;
    public static double x_collect_right_ciclu_2 = -25, y_collect_right_ciclu2 = 59, angle_collect_right_ciculu2 = 180;

    public static double x_collect_center_ciclu_1 = -25, y_collect_center_ciclu1 = 59, angle_collect_center_ciculu1 = 180;
    public static double x_collect_center_ciclu_2 = -25, y_collect_center_ciclu2 = 59, angle_collect_center_ciculu2 = 180;

    public static double x_collect_left_ciclu_1 = -25, y_collect_left_ciclu1 = 59, angle_collect_left_ciculu1 = 180;
    public static double x_collect_left_ciclu_2 = -25, y_collect_left_ciclu2 = 59, angle_collect_left_ciculu2 = 180;

    public static double x_inter_score_left_ciclu_1 = 27.5, y_inter_score_left_ciclu_1 = 59, angle_inter_score_left_ciclu_1 = 180;
    public static double x_inter_score_left_ciclu_2 = 27.5, y_inter_score_left_ciclu_2 = 59, angle_inter_score_left_ciclu_2 = 180;

    public static double x_inter_score_center_ciclu_1 = 27.5, y_inter_score_center_ciclu_1 = 59, angle_inter_score_center_ciclu_1 = 180;
    public static double x_inter_score_center_ciclu_2 = 27.5, y_inter_score_center_ciclu_2 = 59, angle_inter_score_center_ciclu_2 = 180;

    public static double x_inter_score_right_ciclu_1 = 27.5, y_inter_score_right_ciclu_1 = 59, angle_inter_score_right_ciclu_1 = 180;
    public static double x_inter_score_right_ciclu_2 = 27.5, y_inter_score_right_ciclu_2 = 59, angle_inter_score_right_ciclu_2 = 180;

    public static double x_inter_collect_left_ciclu_1 = 27.5, y_inter_collect_left_ciclu_1 = 59, angle_inter_collect_left_ciclu_1 = 180;
    public static double x_inter_collect_left_ciclu_2 = 27.5, y_inter_collect_left_ciclu_2 = 59, angle_inter_collect_left_ciclu_2 = 180;

    public static double x_inter_collect_center_ciclu_1 = 27.5, y_inter_collect_center_ciclu_1 = 59, angle_inter_collect_center_ciclu_1 = 180;
    public static double x_inter_collect_center_ciclu_2 = 27.5, y_inter_collect_center_ciclu_2 = 59, angle_inter_collect_center_ciclu_2 = 180;

    public static double x_inter_collect_right_ciclu_1 = 27.5, y_inter_collect_right_ciclu_1 = 59, angle_inter_collect_right_ciclu_1 = 180;
    public static double x_inter_collect_right_ciclu_2 = 27.5, y_inter_collect_right_ciclu_2 = 59, angle_inter_collect_right_ciclu_2 = 180;


    public static double x_slowed_left_ciclu_1 = -25, y_slowed_left_ciclu_1 = 59, angle_slowed_left_ciclu_1 = 180;
    public static double x_slowed_left_ciclu_2 = -25, y_slowed_left_ciclu_2 = 59, angle_slowed_left_ciclu_2 = 180;

    public static double x_slowed_center_ciclu_1 = -25, y_slowed_center_ciclu_1 = 59, angle_slowed_center_ciclu_1 = 180;
    public static double x_slowed_center_ciclu_2 = -25, y_slowed_center_ciclu_2 = 59, angle_slowed_center_ciclu_2 = 180;

    public static double x_slowed_right_ciclu_1 = -25, y_slowed_right_ciclu_1 = 59, angle_slowed_right_ciclu_1 = 180;
    public static double x_slowed_right_ciclu_2 = -25, y_slowed_right_ciclu_2 = 59, angle_slowed_right_ciclu_2 = 180;


    public static double x_stack = -59.5, y_stack = 4.5, angle_stack = 180;

    int caz = 0;
    boolean ok  = FALSE;
    boolean ok2 = FALSE;

    @Override
    public void runOpMode() throws InterruptedException {

        BluePipelineStackMaster blueRight = new BluePipelineStackMaster(this);
        blueRight.observeStick();

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
        lift.CS = lift_Controller.liftStatus.BASE;
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

        Pose2d purple_left = new Pose2d(x_purple_left, y_purple_left, Math.toRadians(angle_purple_left));
        Pose2d purple_center = new Pose2d(x_purple_center, y_purple_center, Math.toRadians(angle_purple_center));
        Pose2d purple_right = new Pose2d(x_purple_right, y_purple_right, Math.toRadians(angle_purple_right));

        Pose2d yellow_left = new Pose2d(x_yellow_left, y_yellow_left, Math.toRadians(angle_yellow_left));
        Pose2d yellow_center = new Pose2d(x_yellow_center, y_yellow_center, Math.toRadians(angle_yellow_center));
        Pose2d yellow_right = new Pose2d(x_yellow_right, y_yellow_right, Math.toRadians(angle_yellow_right));

        Pose2d score_left_ciclu1 = new Pose2d(x_score_left_ciclu_1, y_score_left_ciclu1, Math.toRadians(angle_score_left_ciculu1));
        Pose2d score_left_ciclu2 = new Pose2d(x_score_left_ciclu_2, y_score_left_ciclu2, Math.toRadians(angle_score_left_ciculu2));

        Pose2d score_center_ciclu1 = new Pose2d(x_score_center_ciclu_1, y_score_center_ciclu1, Math.toRadians(angle_score_center_ciculu1));
        Pose2d score_center_ciclu2 = new Pose2d(x_score_center_ciclu_2, y_score_center_ciclu2, Math.toRadians(angle_score_center_ciculu2));

        Pose2d score_right_ciclu1 = new Pose2d(x_score_right_ciclu_1, y_score_right_ciclu1, Math.toRadians(angle_score_right_ciculu1));
        Pose2d score_right_ciclu2 = new Pose2d(x_score_right_ciclu_2, y_score_right_ciclu2, Math.toRadians(angle_score_right_ciculu2));

        Pose2d collect_left_ciclu1 = new Pose2d(x_collect_left_ciclu_1, y_collect_left_ciclu1, Math.toRadians(angle_collect_left_ciculu1));
        Pose2d collect_left_ciclu2 = new Pose2d(x_collect_left_ciclu_2, y_collect_left_ciclu2, Math.toRadians(angle_collect_left_ciculu2));

        Pose2d collect_center_ciclu1 = new Pose2d(x_collect_center_ciclu_1, y_collect_center_ciclu1, Math.toRadians(angle_collect_center_ciculu1));
        Pose2d collect_center_ciclu2 = new Pose2d(x_collect_center_ciclu_2, y_collect_center_ciclu2, Math.toRadians(angle_collect_center_ciculu2));

        Pose2d collect_right_ciclu1 = new Pose2d(x_collect_right_ciclu_1, y_collect_right_ciclu1, Math.toRadians(angle_collect_right_ciculu1));
        Pose2d collect_right_ciclu2 = new Pose2d(x_collect_right_ciclu_2, y_collect_right_ciclu2, Math.toRadians(angle_collect_right_ciculu2));

        Pose2d inter_score_right_ciclu1 = new Pose2d(x_inter_score_right_ciclu_1, y_inter_score_right_ciclu_1, Math.toRadians(angle_inter_score_right_ciclu_1));
        Pose2d inter_score_right_ciclu2 = new Pose2d(x_inter_score_right_ciclu_2, y_inter_score_right_ciclu_2, Math.toRadians(angle_inter_score_right_ciclu_2));

        Pose2d inter_score_left_ciclu1 = new Pose2d(x_inter_score_left_ciclu_1, y_inter_score_left_ciclu_1, Math.toRadians(angle_inter_score_left_ciclu_1));
        Pose2d inter_score_left_ciclu2 = new Pose2d(x_inter_score_left_ciclu_2, y_inter_score_left_ciclu_2, Math.toRadians(angle_inter_score_left_ciclu_2));

        Pose2d inter_score_center_ciclu1 = new Pose2d(x_inter_score_center_ciclu_1, y_inter_score_center_ciclu_1, Math.toRadians(angle_inter_score_center_ciclu_1));
        Pose2d inter_score_center_ciclu2 = new Pose2d(x_inter_score_center_ciclu_2, y_inter_score_center_ciclu_2, Math.toRadians(angle_inter_score_center_ciclu_2));


        Pose2d inter_collect_right_ciclu1 = new Pose2d(x_inter_collect_right_ciclu_1, y_inter_collect_right_ciclu_1, Math.toRadians(angle_inter_collect_right_ciclu_1));
        Pose2d inter_collect_right_ciclu2 = new Pose2d(x_inter_collect_right_ciclu_2, y_inter_collect_right_ciclu_2, Math.toRadians(angle_inter_collect_right_ciclu_2));

        Pose2d inter_collect_left_ciclu1 = new Pose2d(x_inter_collect_left_ciclu_1, y_inter_collect_left_ciclu_1, Math.toRadians(angle_inter_collect_left_ciclu_1));
        Pose2d inter_collect_left_ciclu2 = new Pose2d(x_inter_collect_left_ciclu_2, y_inter_collect_left_ciclu_2, Math.toRadians(angle_inter_collect_left_ciclu_2));

        Pose2d inter_collect_center_ciclu1 = new Pose2d(x_inter_collect_center_ciclu_1, y_inter_collect_center_ciclu_1, Math.toRadians(angle_inter_collect_center_ciclu_1));
        Pose2d inter_collect_center_ciclu2 = new Pose2d(x_inter_collect_center_ciclu_2, y_inter_collect_center_ciclu_2, Math.toRadians(angle_inter_collect_center_ciclu_2));

        Pose2d slowed_left_ciclu1 = new Pose2d(x_slowed_left_ciclu_1, y_slowed_left_ciclu_1, Math.toRadians(angle_slowed_left_ciclu_1));
        Pose2d slowed_left_ciclu2 = new Pose2d(x_slowed_left_ciclu_2, y_slowed_left_ciclu_2, Math.toRadians(angle_slowed_left_ciclu_2));

        Pose2d slowed_center_ciclu1 = new Pose2d(x_slowed_center_ciclu_1, y_slowed_center_ciclu_1, Math.toRadians(angle_slowed_center_ciclu_1));
        Pose2d slowed_center_ciclu2 = new Pose2d(x_slowed_center_ciclu_2, y_slowed_center_ciclu_2, Math.toRadians(angle_slowed_center_ciclu_2));

        Pose2d slowed_right_ciclu1 = new Pose2d(x_slowed_right_ciclu_1, y_slowed_right_ciclu_1, Math.toRadians(angle_slowed_right_ciclu_1));
        Pose2d slowed_right_ciclu2 = new Pose2d(x_slowed_right_ciclu_2, y_slowed_right_ciclu_2, Math.toRadians(angle_slowed_right_ciclu_2));


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

        TrajectorySequence COLLECT_LEFT_CICLU_1 = drive.trajectorySequenceBuilder(yellow_left)
                .lineToLinearHeading(inter_collect_left_ciclu1)
                .lineToLinearHeading(collect_left_ciclu1)
                .lineToLinearHeading(
                        slowed_left_ciclu1,
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
                .build();

        TrajectorySequence COLLECT_LEFT_CICLU_2 = drive.trajectorySequenceBuilder(score_left_ciclu1)
                .lineToLinearHeading(inter_collect_left_ciclu2)
                .lineToLinearHeading(collect_left_ciclu2)
                .lineToLinearHeading(
                        slowed_left_ciclu2,
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
                .build();

        TrajectorySequence COLLECT_CENTER_CICLU_1 = drive.trajectorySequenceBuilder(yellow_center)
                .lineToLinearHeading(inter_collect_center_ciclu1)
                .lineToLinearHeading(collect_center_ciclu1)
                .lineToLinearHeading(
                        slowed_center_ciclu1,
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
                .build();

        TrajectorySequence COLLECT_CENTER_CICLU_2 = drive.trajectorySequenceBuilder(score_center_ciclu1)
                .lineToLinearHeading(inter_collect_center_ciclu2)
                .lineToLinearHeading(collect_center_ciclu2)
                .lineToLinearHeading(
                        slowed_center_ciclu2,
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
                .build();

        TrajectorySequence COLLECT_RIGHT_CICLU_1 = drive.trajectorySequenceBuilder(yellow_right)
                .lineToLinearHeading(inter_collect_right_ciclu1)
                .lineToLinearHeading(collect_right_ciclu1)
                .lineToLinearHeading(
                        slowed_right_ciclu1,
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
                .build();

        TrajectorySequence COLLECT_RIGHT_CICLU_2 = drive.trajectorySequenceBuilder(score_right_ciclu1)
                .lineToLinearHeading(inter_collect_right_ciclu2)
                .lineToLinearHeading(collect_right_ciclu2)
                .lineToLinearHeading(
                        slowed_right_ciclu2,
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
                .build();

        TrajectorySequence SCORE_CENTER_CICLU_1 = drive.trajectorySequenceBuilder(slowed_center_ciclu1)
                .lineToLinearHeading(inter_score_center_ciclu1)
                .splineToLinearHeading(score_center_ciclu1, Math.toRadians(0))
                .build();

        TrajectorySequence SCORE_CENTER_CICLU_2 = drive.trajectorySequenceBuilder(slowed_center_ciclu2)
                .lineToLinearHeading(inter_score_center_ciclu2)
                .splineToLinearHeading(score_center_ciclu2, Math.toRadians(0))
                .build();

        TrajectorySequence SCORE_LEFT_CICLU_1 = drive.trajectorySequenceBuilder(slowed_left_ciclu1)
                .lineToLinearHeading(inter_score_left_ciclu1)
                .splineToLinearHeading(score_left_ciclu1, Math.toRadians(0))
                .build();

        TrajectorySequence SCORE_LEFT_CICLU_2 = drive.trajectorySequenceBuilder(slowed_left_ciclu2)
                .lineToLinearHeading(inter_score_left_ciclu2)
                .splineToLinearHeading(score_left_ciclu2, Math.toRadians(0))
                .build();

        TrajectorySequence SCORE_RIGHT_CICLU_1 = drive.trajectorySequenceBuilder(slowed_right_ciclu1)
                .lineToLinearHeading(inter_score_right_ciclu1)
                .splineToLinearHeading(score_right_ciclu1, Math.toRadians(0))
                .build();

        TrajectorySequence SCORE_RIGHT_CICLU_2 = drive.trajectorySequenceBuilder(slowed_right_ciclu2)
                .lineToLinearHeading(inter_score_right_ciclu2)
                .splineToLinearHeading(score_right_ciclu2, Math.toRadians(0))
                .build();

        drive.setPoseEstimate(start_pose);
        STROBOT status = STROBOT.START;

        ElapsedTime collect = new ElapsedTime();
        ElapsedTime score = new ElapsedTime();
        ElapsedTime preload = new ElapsedTime();
        ElapsedTime preload2 = new ElapsedTime();
        ElapsedTime collect2= new ElapsedTime();

        double nrcicluri = 0;
        collectAngle.stack_level = 4; //4
        lift.upCnt = 0;

        while (!isStarted() && !isStopRequested()) {

            sleep(20);
            if(blueRight.opencvstack.getWhichSide() == "left"){
                caz = 0;
            } else if (blueRight.opencvstack.getWhichSide() == "center") {
                caz = 1;
            } else {
                caz = 2;
            }
            telemetry.addData("case", blueRight.opencvstack.getWhichSide());
            telemetry.update();
            sleep(50);
        }

        waitForStart();

        String blueRightCase = blueRight.opencvstack.getWhichSide();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            int position = r.lift.getCurrentPosition();


            switch (status) {

                case START: {
                    if(blueRightCase == "left"){
                        drive.followTrajectorySequenceAsync(PURPLE_LEFT);
                    } else if(blueRightCase == "center"){
                        drive.followTrajectorySequenceAsync(PURPLE_CENTER);
                    } else {
                        drive.followTrajectorySequenceAsync(PURPLE_RIGHT);
                    }
                    leftLatch.CS = leftLatch_Controller.leftLatchStatus.CLOSE;
                    rightLatch.CS = rightLatch_Controller.rightLatchStatus.CLOSE;
                    preload.reset();
                    status = STROBOT.PURPLE_DROP;
                    break;
                }

                case PURPLE_DROP: {
                    if(preload.seconds() >0.75)
                    {
                        blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.PRELOAD_GROUND;
                        status = STROBOT.INTERMEDIARY_2;
                    }
                    break;
                }

                case INTERMEDIARY_2:
                {
                    if (!drive.isBusy() && blue_left.CurrentStatus == Blue_LEFT.autoControllerStatus.PRELOADGROUNDDONE) {
                        leftLatch.CS = leftLatch_Controller.leftLatchStatus.OPEN;
                        preload2.reset();
                        status = STROBOT.INTERMEDIARY;
                    }
                    break;
                }

                case INTERMEDIARY:
                {
                    if(preload2.seconds() > 0.7) {
                        if (blueRightCase == "left") {
                            drive.followTrajectorySequenceAsync(COLLECT_LEFT_CICLU_1);
                        } else if (blueRightCase == "center") {
                            drive.followTrajectorySequenceAsync(COLLECT_CENTER_CICLU_1);
                        } else {
                            drive.followTrajectorySequenceAsync(COLLECT_RIGHT_CICLU_1);
                        }
                        collect.reset();
                        status = STROBOT.PREPARE_COLLECT;

                    }
                    break;
                }

                case PREPARE_COLLECT:
                {
                    if(collect.seconds() > 0.2) //0.7
                    {
                        blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.COLLECT;
                        collect2.reset();
                        status = STROBOT.COLLECT;
                    }
                    break;
                }

                case COLLECT: {
                    if (!drive.isBusy()) {
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
                    if(ok == FALSE && (r.right_pixel.getState() == FALSE || r.left_pixel.getState() == FALSE) && collectAngle.stack_level > 0 && nrcicluri>0)
                    {
                        collectAngle.stack_level = Math.max(0, collectAngle.stack_level - 1);
                        ok = TRUE;
                    }
                    if(collect.seconds() > 1.5 && (r.right_pixel.getState() == TRUE || r.left_pixel.getState() == TRUE) && ok2 == FALSE && nrcicluri>0 && collectAngle.stack_level > 0)
                    {
                        collectAngle.stack_level = Math.max(0, collectAngle.stack_level - 1);
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

                    if(leftLatch.CS == leftLatch_Controller.leftLatchStatus.CLOSE_DONE && rightLatch.CS == rightLatch_Controller.rightLatchStatus.CLOSE_DONE || collect.seconds() > 2)
                    {
                        r.collect.setPower(-1);
                        //  outtake.CS = outtake_Controller.outtakeStatus.INTER;
                        status = STROBOT.GO_SCORE_YELLOW_PRELOAD_AND_PIXEL_FROM_STACK;
                    }
                    break;
                }

                case GO_SCORE_YELLOW_PRELOAD_AND_PIXEL_FROM_STACK:{
                    r.collect.setPower(-1);
                    if(blueRightCase == "left"){
                        drive.followTrajectorySequenceAsync(SCORE_LEFT_CICLU_1);
                    } else if(blueRightCase == "center"){
                        drive.followTrajectorySequenceAsync(SCORE_CENTER_CICLU_1);
                    } else {
                        drive.followTrajectorySequenceAsync(SCORE_RIGHT_CICLU_1);
                    }

                    collectAngle.CS = collectAngle_Controller.collectAngleStatus.LIFTED;
                    collectAngle.stack_level = Math.max(0, collectAngle.stack_level -1);
                    score.reset();
                    status = STROBOT.PREPARE_FOR_SCORE;
                    break;
                }

                case PREPARE_FOR_SCORE:
                {
                    if(score.seconds() > 2.5)
                    {
                        blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.INTER;
                    }
                    if(!drive.isBusy() || score.seconds() > 6.6 && nrcicluri==0)
                    {
                        r.collect.setPower(0);
                        blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.SCORE_2;
                        status = STROBOT.SCORE;
                    }
                    if(!drive.isBusy() || score.seconds() > 6.6 && nrcicluri>0)
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
                    if(nrcicluri < 2) //1
                    {
                        if(nrcicluri >0)
                        {lift.upCnt += 2;}
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

                case GO_TO_STACK: {
                    if(score.seconds() > 0.25) {
                        if(blueRightCase == "left" && nrcicluri>0){
                                drive.followTrajectorySequenceAsync(COLLECT_LEFT_CICLU_2);
                        } else if (blueRightCase == "center") {
                            drive.followTrajectorySequenceAsync(COLLECT_CENTER_CICLU_2);
                        } else {
                            drive.followTrajectorySequenceAsync(COLLECT_RIGHT_CICLU_2);
                        }
                        nrcicluri += 1;
                        collect.reset();
                        status = STROBOT.PREPARE_COLLECT;
                    }
                    break;
                }

                case PARK:
                {
                    if(collect.seconds() > 0.3)
                    {
                        blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.COLLECT;

                    }
//                    if(collect.seconds() > 0.65)
//                    {
//                        if(blueRightCase == "left"){
//                            drive.followTrajectorySequenceAsync(PARK_FROM_RIGHT);
//                        } else if(blueRightCase == "center"){
//                            drive.followTrajectorySequenceAsync(PARK_FROM_RIGHT);
//                        } else {
//                            drive.followTrajectorySequenceAsync(PARK_FROM_LEFT);
//                        }
//                    }
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
