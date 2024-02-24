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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AUTO.Recognition.BluePipelineStackMaster;
import org.firstinspires.ftc.teamcode.AUTO_CONTROLLERS.Blue_LEFT;

import org.firstinspires.ftc.teamcode.DistanceSensorCalibrator;
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
@Autonomous(group = "Auto" , name = "BlueRightNearCenter")

public class BlueRightNearCenter extends LinearOpMode {

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
        SYSTEMS,
        SYNC_PATH,
        PREPARE_COLLECT2,
        GO_SCORE_YELLOW_PRELOAD_AND_PIXEL_FROM_STACK2,
        SYNC_PATH2,
        GO_DROP_WHITE_PIXELS,
        GO_DROP_WHITE_PIXELS_SYSTEMS,
    }

    public static double x_start = -37, y_start = 62, angle_start = 270;
    public static double x_purple_left = -24, y_purple_left = 27, angle_purple_left = 270;
    public static double x_purple_center = -38, y_purple_center = 29, angle_purple_center = 270;
    public static double x_purple_right = -45.3, y_purple_right = 40, angle_purple_right = 270;
    public static double x_yellow_left = 38, y_yellow_left = 39, angle_yellow_left = 180;
    public static double x_yellow_center = 38, y_yellow_center = 36, angle_yellow_center = 180;
    public static double x_yellow_right = 38, y_yellow_right = 30, angle_yellow_right = 180;
    public static double x_stack = -49, y_stack = 33, angle_stack = 180;
    public static double stack_2_x = -51.3 , stack_2_y = 9.5, stack_2_angle = 180;
    public static double inter_x = -35.5, inter_y = 48, inter_angle = 270;
    public static double x_interstack = -25, y_inetrstack = 9.5 , angle_interstack = 180;
    public static double x_prepare_for_stack = -47.5, y_prepare_for_stack = 9.5, angle_prepare = 180;

    public static double x_prepare_for_stack2 = 30, y_prepare_for_stack2 = 9.5, angle_prepare2 = 180;
    public static double x_lung_de_linie = -25, y_lung_de_linie = 59, angle_lung_de_linie = 180;
    public static double x_park_from_right = 48, y_park_from_right = 62, angle_park_from_right = 180;
    public static double x_inter_score = 40, y_inter_score=9.5, angle_interscore=180;

    int caz = 0;
    boolean ok  = FALSE;
    boolean ok2 = FALSE;

    @Override
    public void runOpMode() throws InterruptedException {

        BluePipelineStackMaster blueRight = new BluePipelineStackMaster(this);
        blueRight.observeStick();

        DistanceSensorCalibrator calibrator;

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
        leftLatch.CS = leftLatch_Controller.leftLatchStatus.OPEN;
        rightLatch.CS = rightLatch_Controller.rightLatchStatus.OPEN;
        lift.CS = lift_Controller.liftStatus.BASE;
        storage.CS = storage_Controller.storageStatus.COLLECT;
        storageAngle.CS = storageAngle_Controller.storageAngleStatus.ROTATION;
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
        Pose2d purple_center = new Pose2d(x_purple_center, y_purple_center +6, Math.toRadians(angle_purple_center));
        Pose2d purple_right = new Pose2d(x_purple_right +1, y_purple_right, Math.toRadians(angle_purple_right));
        Pose2d yellow_left = new Pose2d(x_yellow_left, y_yellow_left, Math.toRadians(angle_yellow_left));
        Pose2d stack = new Pose2d(x_stack - 6, y_stack, Math.toRadians(angle_stack));
        Pose2d stack2 = new Pose2d(x_stack - 6, y_stack, Math.toRadians(angle_stack));

        Pose2d stack_michi = new Pose2d(stack_2_x, stack_2_y, Math.toRadians(stack_2_angle));

        Pose2d inetr = new Pose2d(inter_x, inter_y, Math.toRadians(inter_angle));
        Pose2d inetr_2 = new Pose2d(inter_x, inter_y-3, Math.toRadians(inter_angle));

        Pose2d stack_center = new Pose2d(x_stack - 6, y_stack, Math.toRadians(angle_stack));

        Pose2d prepare_for_stack = new Pose2d(x_prepare_for_stack, y_prepare_for_stack, Math.toRadians(angle_prepare));
        Pose2d prepare_for_stack2 = new Pose2d(x_prepare_for_stack2, y_prepare_for_stack2, Math.toRadians(angle_prepare2));
        Pose2d lung_de_linie = new Pose2d(x_lung_de_linie - 2, y_lung_de_linie-7.5, Math.toRadians(angle_lung_de_linie));

        Pose2d yellow_right = new Pose2d(x_yellow_right, y_yellow_right, Math.toRadians(angle_yellow_right));
        Pose2d park_from_right = new Pose2d(x_park_from_right, y_park_from_right, Math.toRadians(angle_park_from_right));
        Pose2d yellow_center = new Pose2d(x_yellow_center, y_yellow_center+3, Math.toRadians(angle_yellow_center));
        Pose2d park_from_left = new Pose2d(x_park_from_right, y_park_from_right, Math.toRadians(angle_park_from_right));
        Pose2d inter = new Pose2d(x_interstack, y_inetrstack, Math.toRadians(angle_interstack));

        Pose2d prepare_for_stack_score_0 = new Pose2d(x_prepare_for_stack, y_prepare_for_stack-4, Math.toRadians(angle_prepare));
        Pose2d interscore_0 =new Pose2d(x_inter_score-20,y_inter_score-4,Math.toRadians(angle_interscore));

        Pose2d prepare_for_stack_score = new Pose2d(x_prepare_for_stack, y_prepare_for_stack, Math.toRadians(angle_prepare));
        Pose2d lung_de_linie_score = new Pose2d(x_lung_de_linie-4, y_lung_de_linie+5.5, Math.toRadians(angle_lung_de_linie));
        Pose2d interscore =new Pose2d(x_inter_score,y_inter_score,Math.toRadians(angle_interscore));
        Pose2d lung_de_linie_2 = new Pose2d(x_lung_de_linie -3, y_lung_de_linie-2.5, Math.toRadians(angle_lung_de_linie));
        Pose2d prepare_for_stack_2 = new Pose2d(x_prepare_for_stack+1, y_prepare_for_stack-2.5, Math.toRadians(angle_prepare));


        TrajectorySequence PURPLE_LEFT = drive.trajectorySequenceBuilder(start_pose)
                .lineToLinearHeading(new Pose2d(-36 , 27))
                .lineToLinearHeading(purple_left)
                .build();

        TrajectorySequence PURPLE_CENTER = drive.trajectorySequenceBuilder(start_pose)
                .lineToLinearHeading(purple_center)
                .build();

        TrajectorySequence PURPLE_RIGHT = drive.trajectorySequenceBuilder(start_pose)
                .splineToLinearHeading(purple_right, Math.toRadians(180))
                .build();

        TrajectorySequence SCORE_YELLOW_AND_WHITE_RIGHT = drive.trajectorySequenceBuilder(stack)

                //.splineToLinearHeading(lung_de_linie_score, Math.toRadians(0))
                .lineToLinearHeading(prepare_for_stack_score_0)
                .lineToLinearHeading(interscore_0)
                .lineToLinearHeading(yellow_right)
                //.splineToLinearHeading(yellow_right, Math.toRadians(270))
//                .lineToLinearHeading(lung_de_linie)
//                .splineToLinearHeading(prepare_for_stack, Math.toRadians(180))
//                .splineToLinearHeading(yellow_right, Math.toRadians(180))
                //.setTangent(Math.toRadians(0))
                .build();

        TrajectorySequence SCORE_YELLOW_AND_WHITE_CENTER = drive.trajectorySequenceBuilder(stack)

                //.splineToLinearHeading(lung_de_linie_score, Math.toRadians(0))
                .lineToLinearHeading(prepare_for_stack_score_0)
                .lineToLinearHeading(interscore_0)
                .splineToLinearHeading(yellow_center, Math.toRadians(0))
                //.splineToLinearHeading(yellow_right, Math.toRadians(270))
//                .lineToLinearHeading(lung_de_linie)
//                .splineToLinearHeading(prepare_for_stack, Math.toRadians(180))
//                .splineToLinearHeading(yellow_right, Math.toRadians(180))
                //.setTangent(Math.toRadians(0))
                .build();

        TrajectorySequence SCORE_YELLOW_AND_WHITE_LEFT = drive.trajectorySequenceBuilder(stack)

                //.splineToLinearHeading(lung_de_linie_score, Math.toRadians(0))
                .lineToLinearHeading(prepare_for_stack_score_0)
                .lineToLinearHeading(interscore_0)
                .splineToLinearHeading(yellow_left, Math.toRadians(0))
                //.splineToLinearHeading(yellow_right, Math.toRadians(270))
//                .lineToLinearHeading(lung_de_linie)
//                .splineToLinearHeading(prepare_for_stack, Math.toRadians(180))
//                .splineToLinearHeading(yellow_right, Math.toRadians(180))
                //.setTangent(Math.toRadians(0))
                .build();

        TrajectorySequence SCORE_YELLOW_AND_WHITE_RIGHT22 = drive.trajectorySequenceBuilder(stack_michi)

                //.splineToLinearHeading(lung_de_linie_score, Math.toRadians(0))
                .lineToLinearHeading(prepare_for_stack_score)
                .lineToLinearHeading(interscore)
                .lineToLinearHeading(yellow_left)
                //.splineToLinearHeading(yellow_right, Math.toRadians(270))
//                .lineToLinearHeading(lung_de_linie)
//                .splineToLinearHeading(prepare_for_stack, Math.toRadians(180))
//                .splineToLinearHeading(yellow_right, Math.toRadians(180))
                //.setTangent(Math.toRadians(0))
                .build();

        TrajectorySequence SCORE_YELLOW_AND_WHITE_CENTER22 = drive.trajectorySequenceBuilder(stack_michi)

                //.splineToLinearHeading(lung_de_linie_score, Math.toRadians(0))
                .lineToLinearHeading(prepare_for_stack_score)
                .lineToLinearHeading(interscore)
                .lineToLinearHeading(yellow_center)
                //.splineToLinearHeading(yellow_right, Math.toRadians(270))
//                .lineToLinearHeading(lung_de_linie)
//                .splineToLinearHeading(prepare_for_stack, Math.toRadians(180))
//                .splineToLinearHeading(yellow_right, Math.toRadians(180))
                //.setTangent(Math.toRadians(0))
                .build();


        TrajectorySequence DROP_WHITE_PIXELS = drive.trajectorySequenceBuilder(stack_michi)
                .lineToLinearHeading(prepare_for_stack_score)
                .lineToLinearHeading(interscore)
                .build();

        TrajectorySequence SCORE_YELLOW_AND_WHITE_LEFT22 = drive.trajectorySequenceBuilder(stack_michi)

                //.splineToLinearHeading(lung_de_linie_score, Math.toRadians(0))
                .lineToLinearHeading(prepare_for_stack_score)
                .lineToLinearHeading(interscore)
                .lineToLinearHeading(yellow_left)
                //.splineToLinearHeading(yellow_right, Math.toRadians(270))
//                .lineToLinearHeading(lung_de_linie)
//                .splineToLinearHeading(prepare_for_stack, Math.toRadians(180))
//                .splineToLinearHeading(yellow_right, Math.toRadians(180))
                //.setTangent(Math.toRadians(0))
                .build();

        TrajectorySequence PRELOAD_LEFT_SENSOR = drive.trajectorySequenceBuilder(yellow_left)
                .back(30,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence PRELOAD_CENTER_SENSOR = drive.trajectorySequenceBuilder(yellow_center)
                .back(30,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence PRELOAD_RIGHT_SENSOR = drive.trajectorySequenceBuilder(yellow_right)
                .back(30,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence PRELOAD_LEFT_SENSOR2 = drive.trajectorySequenceBuilder(yellow_left)
                .back(30,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence PRELOAD_CENTER_SENSOR2 = drive.trajectorySequenceBuilder(yellow_center)
                .back(30,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence PRELOAD_RIGHT_SENSOR2 = drive.trajectorySequenceBuilder(yellow_left)
                .back(30,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
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

        TrajectorySequence GO_STACK_WITH_PRELOAD_RIGHT = drive.trajectorySequenceBuilder(purple_right)
                .lineToLinearHeading(inetr)
                .lineToLinearHeading(stack)
                .build();

        TrajectorySequence GO_STACK_WITH_PRELOAD_CENTER = drive.trajectorySequenceBuilder(purple_center)
                .lineToLinearHeading(inetr_2)
                .lineToLinearHeading(stack_center)
                .build();

        TrajectorySequence GO_STACK_WITH_PRELOAD_LEFT = drive.trajectorySequenceBuilder(purple_left)
                .lineToLinearHeading(inetr)
                .lineToLinearHeading(stack)
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

        TrajectorySequence GO_STACK_LEFT_CICLU_1 = drive.trajectorySequenceBuilder(yellow_left)
                .setTangent(90)
                .splineToLinearHeading(prepare_for_stack, Math.toRadians(180))
                .lineToLinearHeading(inter)
                //.lineToLinearHeading(lung_de_linie)
                .lineToLinearHeading(

                        stack_michi,
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
                .lineToLinearHeading(prepare_for_stack2)
                .lineToLinearHeading(inter)
                //.lineToLinearHeading(lung_de_linie)
                .lineToLinearHeading(

                        stack_michi,
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
//                .lineToLinearHeading(prepare_for_stack)
//                .lineToLinearHeading(lung_de_linie)
//                .splineToLinearHeading(stack,Math.toRadians(180))
                .build();

        TrajectorySequence GO_STACK_CENTER_2 = drive.trajectorySequenceBuilder(yellow_right)
                .setTangent(90)
                .splineToLinearHeading(prepare_for_stack_2, Math.toRadians(180))
                .lineToLinearHeading(lung_de_linie_2)
                .lineToLinearHeading(stack)
//                .lineToLinearHeading(prepare_for_stack)
//                .lineToLinearHeading(lung_de_linie)
//                .splineToLinearHeading(stack, Math.toRadians(180))
                .build();

        TrajectorySequence GO_STACK_RIGHT = drive.trajectorySequenceBuilder(yellow_right)
                .lineToLinearHeading(prepare_for_stack2)
                .lineToLinearHeading(inter)
                //.lineToLinearHeading(lung_de_linie)
                .lineToLinearHeading(

                        stack_michi,
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
                .splineToLinearHeading(lung_de_linie_score, Math.toRadians(0))
                .lineToLinearHeading(prepare_for_stack_score)
                .splineToLinearHeading(yellow_right, Math.toRadians(0))
                //.splineToLinearHeading(yellow_right, Math.toRadians(270))
//                .lineToLinearHeading(lung_de_linie)
//                .splineToLinearHeading(prepare_for_stack, Math.toRadians(180))
//                .splineToLinearHeading(yellow_right, Math.toRadians(180))
                //.setTangent(Math.toRadians(0))
                .build();

        TrajectorySequence SCORE_RIGHT = drive.trajectorySequenceBuilder(stack)
                .setTangent(70)
                .splineToLinearHeading(lung_de_linie_score, Math.toRadians(0))
                .lineToLinearHeading(prepare_for_stack_score)
                .splineToLinearHeading(yellow_right, Math.toRadians(0))
                //.splineToLinearHeading(yellow_right, Math.toRadians(270))
                .build();

        TrajectorySequence PARK_FROM_RIGHT = drive.trajectorySequenceBuilder(yellow_right)
                .lineToLinearHeading(park_from_right)
                .build();

        TrajectorySequence PARK_FROM_LEFT = drive.trajectorySequenceBuilder(yellow_left)
                .lineToLinearHeading(park_from_left)
                .build();

        drive.setPoseEstimate(start_pose);
        STROBOT status = STROBOT.START;

        ElapsedTime collect = new ElapsedTime();
        ElapsedTime score = new ElapsedTime();
        ElapsedTime preload = new ElapsedTime();
        ElapsedTime preload2 = new ElapsedTime();
        ElapsedTime collect2= new ElapsedTime();
        ElapsedTime drop = new ElapsedTime();
        ElapsedTime drop2 = new ElapsedTime();

        double nrcicluri = 0;
        collectAngle.stack_level = 4; //4
        storageAngle.rotation_i = 2;
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
        double[] rawReadings = {26.3, 25.5, 26, 24.4, 26.6, 25, 26.9, 25.8, 26.1, 26.5, 27.1, 25.6, 26, 26.1, 26.3, 25.9, 26.4, 25.5, 24.7, 26.9};
        double[] actualDistances = {26, 25, 25.5, 24, 26.5, 24.5, 27, 25.4, 25.7, 26.2, 26.6, 24.8, 25.3, 25.9, 26.1, 25.8, 26.3, 25.1, 24.9, 26.8};
        calibrator = new DistanceSensorCalibrator(rawReadings, actualDistances);

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            int position = r.lift.getCurrentPosition();


            switch (status) {

                case START: {
                    if(blueRightCase == "left"){
                        collectAngle_Controller.CS = collectAngle_Controller.collectAngleStatus.INITIALIZE;
                        drive.followTrajectorySequenceAsync(PURPLE_LEFT);
                    } else if(blueRightCase == "center"){
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
                    if(!drive.isBusy())
                    {
                        collectAngle.CS = collectAngle_Controller.collectAngleStatus.LIFTED ;
                        status = STROBOT.INTERMEDIARY;
                    }
                    break;
                }



                case INTERMEDIARY:
                {
                    if(preload2.seconds() > 0.7) {
                        if (blueRightCase == "left") {
                            drive.followTrajectorySequenceAsync(GO_STACK_WITH_PRELOAD_LEFT);
                        } else if (blueRightCase == "center") {
                            drive.followTrajectorySequenceAsync(GO_STACK_WITH_PRELOAD_CENTER);
                        } else {
                            drive.followTrajectorySequenceAsync(GO_STACK_WITH_PRELOAD_RIGHT);
                        }

                        collect.reset();
                        status = STROBOT.PREPARE_COLLECT;

                    }
                    break;
                }

                case PREPARE_COLLECT:
                {
                    if(nrcicluri == 0){
                        if(collect.seconds() > 0.2) //0.7
                        {
                            r.collect.setPower(1);
                            collectAngle.CS = collectAngle_Controller.collectAngleStatus.STACK;
                            //  blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.COLLECT;
                            collect2.reset();
                            status = STROBOT.COLLECT;
                        }
                    }
                    if(nrcicluri > 0)
                    {
                        if(collect.seconds() > 0.4)
                        {
                            blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.COLLECT;
                            collect2.reset();
                            status = STROBOT.COLLECT;
                        }
                    }
                    break;
                }

                case PREPARE_COLLECT2:
                {
                    if(nrcicluri == 0){
                        if(collect.seconds() > 0.2) //0.7
                        {
                            r.collect.setPower(1);
                            collectAngle.CS = collectAngle_Controller.collectAngleStatus.STACK;
                            blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.COLLECT;
                            collect2.reset();
                            status = STROBOT.COLLECT;
                        }
                    }
                    if(nrcicluri > 0)
                    {
                        if(collect.seconds() > 0.4)
                        {
                            blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.COLLECT;
                            collect2.reset();
                            status = STROBOT.COLLECT;
                        }
                    }
                    break;
                }

                case COLLECT: {
                    if(nrcicluri == 0){
                        if (collect2.seconds() > 0.35) {
                            collect.reset();
                            collectAngle.stack_level =4;
                            status = STROBOT.VERIF;
                        }
                    }
                    if(nrcicluri > 0){
                        if(collect2.seconds() > 0.01)
                        {
                            collectAngle.CS = collectAngle_Controller.collectAngleStatus.LIFTED;
                        }
                        if (/*!drive.isBusy() || 4.5, 2.3*/ collect2.seconds() > 3.5) {
                            r.collect.setPower(1);
                            collectAngle.CS = collectAngle_Controller.collectAngleStatus.STACK;
                            collect.reset();
                            ok = FALSE;
                            ok2 = FALSE;
                            if(nrcicluri == 1)
                            { collectAngle.stack_level =3;}
                            else {
                                collectAngle.stack_level =1;
                                if(collect2.seconds() > 3.65){
                                    collectAngle.stack_level = 0;
                                }
                            }
                            status = STROBOT.VERIF;
                        }
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

                    if((leftLatch.CS == leftLatch_Controller.leftLatchStatus.CLOSE_DONE && rightLatch.CS == rightLatch_Controller.rightLatchStatus.CLOSE_DONE) || collect.seconds() > 3)
                    {
                        //  r.collect.setPower(-1);
                        //  outtake.CS = outtake_Controller.outtakeStatus.INTER;
                        if(nrcicluri == 0)
                        {status = STROBOT.GO_SCORE_YELLOW_PRELOAD_AND_PIXEL_FROM_STACK;}
                        else if(nrcicluri == 1)
                        {
                            status = STROBOT.GO_SCORE_YELLOW_PRELOAD_AND_PIXEL_FROM_STACK2;
                        }
                        else{
                            drop2.reset();
                            status = STROBOT.GO_DROP_WHITE_PIXELS;
                        }
                    }
                    break;
                }

                case GO_SCORE_YELLOW_PRELOAD_AND_PIXEL_FROM_STACK:{
                    r.collect.setPower(-1);
                    if(blueRightCase == "left"){
                        drive.followTrajectorySequenceAsync(SCORE_YELLOW_AND_WHITE_LEFT);
                    } else if(blueRightCase == "center"){
                        drive.followTrajectorySequenceAsync(SCORE_YELLOW_AND_WHITE_CENTER);
                    } else {
                        drive.followTrajectorySequenceAsync(SCORE_YELLOW_AND_WHITE_RIGHT);
                    }

                    collectAngle.CS = collectAngle_Controller.collectAngleStatus.LIFTED;
                    score.reset();
                    status = STROBOT.SYSTEMS;
                    break;
                }

                case GO_DROP_WHITE_PIXELS:{
                    r.collect.setPower(-1);
                    drive.followTrajectorySequenceAsync(DROP_WHITE_PIXELS);
                    drop.reset();
                    status = STROBOT.GO_DROP_WHITE_PIXELS_SYSTEMS;
                    break;
                }

                case GO_DROP_WHITE_PIXELS_SYSTEMS:{
                    blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.SCORE_26;
                    status = STROBOT.NOTHING;
                    break;
                }

                case GO_SCORE_YELLOW_PRELOAD_AND_PIXEL_FROM_STACK2:{
                    r.collect.setPower(-1);
                    if(blueRightCase == "left"){
                        drive.followTrajectorySequenceAsync(SCORE_YELLOW_AND_WHITE_LEFT22);
                    } else if(blueRightCase == "center"){
                        drive.followTrajectorySequenceAsync(SCORE_YELLOW_AND_WHITE_CENTER22);
                    } else {
                        drive.followTrajectorySequenceAsync(SCORE_YELLOW_AND_WHITE_RIGHT22);
                    }

                    collectAngle.CS = collectAngle_Controller.collectAngleStatus.LIFTED;
                    score.reset();
                    status = STROBOT.SYSTEMS;
                    break;
                }

                case SYSTEMS:
                {
                    if(score.seconds() > 3)
                    {
                        if(nrcicluri ==0) {
                            lift.upCnt += 1;
                            blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.SCORE_PRELOAD_AUTO;
                            status = STROBOT.SYNC_PATH;
                        }
                        else
                        {
                            lift.upCnt += 1;
                            blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.SCORE_PRELOAD_AUTO;
                            status = STROBOT.SYNC_PATH2;
                        }
                    }
                    break;
                }

                case SYNC_PATH:
                {
                    if(!drive.isBusy()) {
                        if(blueRightCase == "left"){
                            drive.followTrajectorySequenceAsync(PRELOAD_LEFT_SENSOR);
                        } else if(blueRightCase == "center"){
                            drive.followTrajectorySequenceAsync(PRELOAD_CENTER_SENSOR);
                        } else {
                            drive.followTrajectorySequenceAsync(PRELOAD_RIGHT_SENSOR);
                        }
                        status = STROBOT.PREPARE_FOR_SCORE;
                    }
                    break;
                }

                case SYNC_PATH2:
                {
                    if(!drive.isBusy()) {
                        if(blueRightCase == "left"){
                            drive.followTrajectorySequenceAsync(PRELOAD_LEFT_SENSOR2);
                        } else if(blueRightCase == "center"){
                            drive.followTrajectorySequenceAsync(PRELOAD_CENTER_SENSOR2);
                        } else {
                            drive.followTrajectorySequenceAsync(PRELOAD_RIGHT_SENSOR2);
                        }
                        status = STROBOT.PREPARE_FOR_SCORE;
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

                case SCORE:
                {
                    leftLatch.CS = leftLatch_Controller.leftLatchStatus.OPEN;
                    rightLatch.CS = rightLatch_Controller.rightLatchStatus.OPEN;
                    status = STROBOT.CHECK_COLLECT;
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
                    if(blueRightCase == "left" && nrcicluri>0){
                        drive.followTrajectorySequenceAsync(GO_STACK_LEFT_CICLU_1);
                    } else if (blueRightCase == "center") {
                        drive.followTrajectorySequenceAsync(GO_STACK_CENTER);
                    } else {
                        drive.followTrajectorySequenceAsync(GO_STACK_RIGHT);
                    }
                    nrcicluri += 1;
                    collect.reset();
                    status = STROBOT.PREPARE_COLLECT;
                    break;
                }

                case PARK:
                {
                    if(collect.seconds() > 0.3)
                    {
                        blue_left.CurrentStatus = Blue_LEFT.autoControllerStatus.COLLECT;

                    }
                    if(collect.seconds() > 0.65)
                    {
                        if(blueRightCase == "left"){
                            drive.followTrajectorySequenceAsync(PARK_FROM_RIGHT);
                        } else if(blueRightCase == "center"){
                            drive.followTrajectorySequenceAsync(PARK_FROM_RIGHT);
                        } else {
                            drive.followTrajectorySequenceAsync(PARK_FROM_LEFT);
                        }
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
            telemetry.addData("storage", storage.CS);
            telemetry.addData("stack", collectAngle.stack_level);
            telemetry.addData("left_pixel", r.left_pixel.getState());
            telemetry.addData("nrcicluri", nrcicluri);
            telemetry.update();

        }

    }
}