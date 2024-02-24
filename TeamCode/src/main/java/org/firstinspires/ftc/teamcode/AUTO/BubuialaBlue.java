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
@Autonomous(group = "Auto" , name = "bubuialablue")

public class BubuialaBlue extends LinearOpMode {

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
        NOTHING,
    }

    public static double x_start = 16, y_start = 62, angle_start = 270;

    public static double x_purple_left = 25, y_purple_left = 39, angle_purple_left = 270;
    public static double x_purple_center = 13.5, y_purple_center = 32, angle_purple_center = 270;
    public static double x_purple_right = 9, y_purple_right = 27.5, angle_purple_right = 180;

    public static double x_yellow_left = 46, y_yellow_left = 38.5, angle_yellow_left = 180;
    public static double x_yellow_center = 47, y_yellow_center = 38, angle_yellow_center = 180;
    public static double x_yellow_right = 45, y_yellow_right = 29.5, angle_yellow_right = 180;

    public static double x_score_right_ciclu_1 = 45, y_score_right_ciclu1 = 38, angle_score_right_ciculu1 = 180;
    public static double x_score_right_ciclu_2 = 45, y_score_right_ciclu2 = 38, angle_score_right_ciculu2 = 180;

    public static double x_score_center_ciclu_1 = -5, y_score_center_ciclu1 = -4.5, angle_score_center_ciculu1 = 180;
    public static double x_score_center_ciclu_2 = -5, y_score_center_ciclu2 = -4.5, angle_score_center_ciculu2 = 180;

    public static double x_score_left_ciclu_1 = -5, y_score_left_ciclu1 = -4.5, angle_score_left_ciculu1 = 180;
    public static double x_score_left_ciclu_2 = -5, y_score_left_ciclu2 = -4.5, angle_score_left_ciculu2 = 180;

    public static double x_collect_right_ciclu_1 = -5, y_collect_right_ciclu1 = -4.5, angle_collect_right_ciculu1 = 180;
    public static double x_collect_right_ciclu_2 = -5, y_collect_right_ciclu2 = -4.5, angle_collect_right_ciculu2 = 180;

    public static double x_collect_center_ciclu_1 = -5, y_collect_center_ciclu1 = -4.5, angle_collect_center_ciculu1 = 180;
    public static double x_collect_center_ciclu_2 = -5, y_collect_center_ciclu2 = -4.5, angle_collect_center_ciculu2 = 180;

    public static double x_collect_left_ciclu_1 = -5, y_collect_left_ciclu1 = -4.5, angle_collect_left_ciculu1 = 180;
    public static double x_collect_left_ciclu_2 = -5, y_collect_left_ciclu2 = -4.5, angle_collect_left_ciculu2 = 180;
    
    public static double x_inter_score_left_ciclu_1 = 2, y_inter_score_left_ciclu_1 = 4.5, angle_inter_score_left_ciclu_1 = 180;
    public static double x_inter_score_left_ciclu_2 = 2, y_inter_score_left_ciclu_2 = 4.5, angle_inter_score_left_ciclu_2 = 180;

    public static double x_inter_score_center_ciclu_1 = 2, y_inter_score_center_ciclu_1 = 4.5, angle_inter_score_center_ciclu_1 = 180;
    public static double x_inter_score_center_ciclu_2 = 2, y_inter_score_center_ciclu_2 = 4.5, angle_inter_score_center_ciclu_2 = 180;

    public static double x_inter_score_right_ciclu_1 = 2, y_inter_score_right_ciclu_1 = 4.5, angle_inter_score_right_ciclu_1 = 180;
    public static double x_inter_score_right_ciclu_2 = 2, y_inter_score_right_ciclu_2 = 4.5, angle_inter_score_right_ciclu_2 = 180;

    public static double x_inter_collect_left_ciclu_1 = 2, y_inter_collect_left_ciclu_1 = 4.5, angle_inter_collect_left_ciclu_1 = 180;
    public static double x_inter_collect_left_ciclu_2 = 2, y_inter_collect_left_ciclu_2 = 4.5, angle_inter_collect_left_ciclu_2 = 180;

    public static double x_inter_collect_center_ciclu_1 = 2, y_inter_collect_center_ciclu_1 = 4.5, angle_inter_collect_center_ciclu_1 = 180;
    public static double x_inter_collect_center_ciclu_2 = 2, y_inter_collect_center_ciclu_2 = 4.5, angle_inter_collect_center_ciclu_2 = 180;

    public static double x_inter_collect_right_ciclu_1 = 2, y_inter_collect_right_ciclu_1 = 4.5, angle_inter_collect_right_ciclu_1 = 180;
    public static double x_inter_collect_right_ciclu_2 = 2, y_inter_collect_right_ciclu_2 = 4.5, angle_inter_collect_right_ciclu_2 = 180;


    public static double x_slowed_left_ciclu_1 = -59.5, y_slowed_left_ciclu_1 = 4.5, angle_slowed_left_ciclu_1 = 180;
    public static double x_slowed_left_ciclu_2 = -59.5, y_slowed_left_ciclu_2 = 4.5, angle_slowed_left_ciclu_2 = 180;

    public static double x_slowed_center_ciclu_1 = -59.5, y_slowed_center_ciclu_1 = 4.5, angle_slowed_center_ciclu_1 = 180;
    public static double x_slowed_center_ciclu_2 = -59.5, y_slowed_center_ciclu_2 = 4.5, angle_slowed_center_ciclu_2 = 180;

    public static double x_slowed_right_ciclu_1 = -59.5, y_slowed_right_ciclu_1 = 4.5, angle_slowed_right_ciclu_1 = 180;
    public static double x_slowed_right_ciclu_2 = -59.5, y_slowed_right_ciclu_2 = 4.5, angle_slowed_right_ciclu_2 = 180;

    int caz = 0;
    boolean ok  = FALSE;
    boolean ok2 = FALSE;

    @Override
    public void runOpMode() throws InterruptedException {

        BlueOpenCVMaster blueLeft = new BlueOpenCVMaster(this);
        blueLeft.observeStick();

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
        blue_left.update(r, lift, ik, fourbar, storage, storageAngle, leftLatch, rightLatch);
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
                        status = STROBOT.YELLOW;
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
                    if (blue_left.CurrentStatus == Blue_LEFT.autoControllerStatus.SCORE_PRELOAD_DONE) {
                        leftLatch_Controller.CS = leftLatch_Controller.leftLatchStatus.OPEN;
                        rightLatch_Controller.CS = rightLatch_Controller.rightLatchStatus.OPEN;
                        score.reset();
                        nrcicluri +=1;
                        status = status.GO_TO_STACK;
                    }
                    break;
                }

                case GO_TO_STACK: {
                    if(score.seconds() > 0.25) {
                        if(nrcicluri == 1)
                        {
                        if(blueLeftCase == "left"){
                            drive.followTrajectorySequenceAsync(COLLECT_LEFT_CICLU_1);
                        } else if (blueLeftCase == "center") {
                            drive.followTrajectorySequenceAsync(COLLECT_CENTER_CICLU_1);
                        } else {
                            drive.followTrajectorySequenceAsync(COLLECT_RIGHT_CICLU_1);
                        }
                        }
                        else
                        {
                            if(blueLeftCase == "left"){
                                drive.followTrajectorySequenceAsync(COLLECT_LEFT_CICLU_2);
                            } else if (blueLeftCase == "center") {
                                drive.followTrajectorySequenceAsync(COLLECT_CENTER_CICLU_2);
                            } else {
                                drive.followTrajectorySequenceAsync(COLLECT_RIGHT_CICLU_2);
                            }
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
                        lift.upCnt +=2;
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
                        r.collect.setPower(-1);
                        //  outtake.CS = outtake_Controller.outtakeStatus.INTER;
                        status = STROBOT.GO_SCORE;
                    }} else
                    {
                        if(leftLatch.CS == leftLatch_Controller.leftLatchStatus.CLOSE_DONE || rightLatch.CS == rightLatch_Controller.rightLatchStatus.CLOSE_DONE || collect.seconds() > 1.1)
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
                    if(nrcicluri == 1)
                    {
                        if(blueLeftCase == "left"){
                            drive.followTrajectorySequenceAsync(SCORE_LEFT_CICLU_1);
                        } else if(blueLeftCase == "center"){
                            drive.followTrajectorySequenceAsync(SCORE_CENTER_CICLU_1);
                        } else {
                            drive.followTrajectorySequenceAsync(SCORE_RIGHT_CICLU_1);
                        }
                    }
                    else
                    {
                        if(blueLeftCase == "left"){
                            drive.followTrajectorySequenceAsync(SCORE_LEFT_CICLU_2);
                        } else if(blueLeftCase == "center"){
                            drive.followTrajectorySequenceAsync(SCORE_CENTER_CICLU_2);
                        } else {
                            drive.followTrajectorySequenceAsync(SCORE_RIGHT_CICLU_2);
                        }
                    }

                    collectAngle.CS = collectAngle_Controller.collectAngleStatus.LIFTED;
                    collectAngle.stack_level -=1;
                    score.reset();
                    status = STROBOT.PREPARE_FOR_SCORE;
                    break;
                }

                case PREPARE_FOR_SCORE:
                {
                    if(score.seconds() > 2.2)
                    {

                        blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.SCORE;
                    }
                    if(!drive.isBusy())

                    { r.collect.setPower(0);
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
                    if(nrcicluri < 2)
                    {
                        lift.upCnt += 2;
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
                    if(collect.seconds() > 0.2)
                    {
                        fourbar.CS = fourBar_Controller.fourbarStatus.COLLECT;
                    }
                    if(collect.seconds() > 0.3)
                    {
                        storage.CS = storage_Controller.storageStatus.COLLECT;
                    }
//                    if(collect.seconds() > 0.4)
//                    {
//                        if(blueLeftCase == "left"){
//                            drive.followTrajectorySequenceAsync(PARK_FROM_RIGHT);
//                        } else if(blueLeftCase == "center"){
//                            drive.followTrajectorySequenceAsync(PARK_FROM_RIGHT);
//                        } else {
//                            drive.followTrajectorySequenceAsync(PARK_FROM_LEFT);
//                        }
//                    }
                    if(collect.seconds() > 0.45)
                    {
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
            blue_left.update(r, lift, ik, fourbar, storage, storageAngle, leftLatch, rightLatch);
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
