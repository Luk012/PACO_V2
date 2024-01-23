//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//
//import org.firstinspires.ftc.teamcode.RoadRunner.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.globals.InverseKinematics;
//import org.firstinspires.ftc.teamcode.globals.robotMap;
//import org.firstinspires.ftc.teamcode.system_controllers.collectAngle_Controller;
//import org.firstinspires.ftc.teamcode.system_controllers.drone_Controller;
//import org.firstinspires.ftc.teamcode.system_controllers.fourBar_Controller;
//import org.firstinspires.ftc.teamcode.system_controllers.leftLatch_Controller;
//import org.firstinspires.ftc.teamcode.system_controllers.lift_Controller;
//import org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller;
//import org.firstinspires.ftc.teamcode.system_controllers.pto_Controller;
//import org.firstinspires.ftc.teamcode.system_controllers.rightLatch_Controller;
//import org.firstinspires.ftc.teamcode.system_controllers.storageAngle_Controller;
//import org.firstinspires.ftc.teamcode.system_controllers.storage_Controller;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//
//import java.util.List;
//
//@Config
//@Autonomous(group = "Auto" , name = "BlueLeft")
//
//public class BlueLeft extends LinearOpMode {
//
//    enum STROBOT{
//        START,
//        PURPLE_DROP,
//        GO_TO_YELLOW_POS,
//        YELLOW_DROP,
//        DROP,
//        GO_TO_STACK,
//        COLLECT,
//        RETURN,
//    }
//
//    public static double x_start = 16, y_start = 66.5, angle_start = 270;
//    public static double x_purple_left = 23, y_purple_left = 33.5, angle_purple_left = 270;
//    public static double x_yellow_left = 47, y_yellow_left = 41, angle_yellow_left = 180;
//    public static double x_stack = -65, y_stack = 8, angle_stack = 180;
//    public static double x_prepare_for_stack = 34.5, y_prepare_for_stack = 14, angle_prepare = 180;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        robotMap robot = new robotMap(hardwareMap);
//        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//
//        double currentVoltage;
//        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
//        currentVoltage = batteryVoltageSensor.getVoltage();
//
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }
//
//        collectAngle_Controller angleController = new collectAngle_Controller();
//        outtake_Controller outtakeController = new outtake_Controller();
//        fourBar_Controller fourBarController = new fourBar_Controller();
//        leftLatch_Controller leftLatchController = new leftLatch_Controller();
//        lift_Controller liftController = new lift_Controller();
//        pto_Controller ptoController = new pto_Controller();
//        rightLatch_Controller rightLatchController = new rightLatch_Controller();
//        storage_Controller storageController = new storage_Controller();
//        storageAngle_Controller storageAngleController = new storageAngle_Controller();
//        InverseKinematics inverseKinematics = new InverseKinematics(robot.right_fourbar, robot.left_fourbar, robot.storage_angle, robot.back);
//
//        collectAngle_Controller.CS = collectAngle_Controller.collectAngleStatus.INITIALIZE;
//        fourBar_Controller.CS = fourBar_Controller.fourbarStatus.INITIALIZE;
//        leftLatch_Controller.CS = leftLatch_Controller.leftLatchStatus.INITIALIZE;
//        rightLatch_Controller.CS =  rightLatch_Controller.rightLatchStatus.INITIALIZE;
//        lift_Controller.CS = lift_Controller.liftStatus.INITIALIZE;
//        storage_Controller.CS = storage_Controller.storageStatus.INITIALZIE;
//        storageAngle_Controller.CS = storageAngle_Controller.storageAngleStatus.INITIALIZE;
//        ptoController.CS = pto_Controller.ptoStatus.INITIALIZE;
//        outtakeController.CS = outtake_Controller.outtakeStatus.COLLECT;
//
//        liftController.update(robot, 0, currentVoltage);
//        fourBarController.update(robot);
//        angleController.update(robot);
//        leftLatchController.update(robot);
//        rightLatchController.update(robot);
//        storageController.update(robot);
//        storageAngleController.update(robot);
//        outtakeController.update(fourBarController, storageController, storageAngleController, liftController);
//
//        Pose2d start_pose = new Pose2d(x_start, y_start, angle_start);
//        Pose2d purple_left = new Pose2d(x_purple_left, y_purple_left, angle_purple_left);
//        Pose2d yellow_left = new Pose2d(x_yellow_left, y_yellow_left, angle_yellow_left);
//        Pose2d stack = new Pose2d(x_stack, y_stack, angle_stack);
//        Pose2d prepare_for_stack = new Pose2d(x_prepare_for_stack, y_prepare_for_stack, angle_prepare);
//
//        TrajectorySequence PURPLE_LEFT = drive.trajectorySequenceBuilder(start_pose)
//                .lineToLinearHeading(purple_left)
//                .build();
//
//        TrajectorySequence YELLOW_LEFT = drive.trajectorySequenceBuilder(purple_left)
//                .lineToLinearHeading(yellow_left)
//                .build();
//
//        TrajectorySequence PREPARE_FOR_STACK = drive.trajectorySequenceBuilder(yellow_left)
//                .lineToLinearHeading(prepare_for_stack)
//                .build();
//
//        TrajectorySequence GO_STACK = drive.trajectorySequenceBuilder(prepare_for_stack)
//                .lineToLinearHeading(stack)
//                .build();
//
//        drive.setPoseEstimate(start_pose);
//        STROBOT status = STROBOT.START;
//
//        while(!isStarted() && !isStopRequested()){
//
//            sleep(20);
//
//        }
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        while (opModeIsActive() && !isStopRequested()){
//
//            switch (status){
//
//                case START:{
//                    collectAngle_Controller.CS = collectAngle_Controller.collectAngleStatus.GROUND;
//                    drive.followTrajectorySequenceAsync(PURPLE_LEFT);
//                    status = STROBOT.PURPLE_DROP;
//                    break;
//                }
//
//                case PURPLE_DROP:{
//                    if(!drive.isBusy()){
//                        collectAngle_Controller.CS = collectAngle_Controller.collectAngleStatus.LIFTED;
//                        status = STROBOT.GO_TO_YELLOW_POS;
//                    }
//                    break;
//                }
//
//                case GO_TO_YELLOW_POS:{
//                    drive.followTrajectorySequenceAsync(YELLOW_LEFT);
//                    status = STROBOT.YELLOW_DROP;
//                    break;
//                }
//
//                case YELLOW_DROP:{
//                    if(!drive.isBusy()){
//                        outtake_Controller.CS = outtake_Controller.outtakeStatus.SCORE;
//                        fourBar_Controller.CS = fourBar_Controller.fourbarStatus.SCORE;
//                        storage_Controller.CS = storage_Controller.storageStatus.SCORE;
//                        status = status.DROP;
//                    }
//                    break;
//                }
//
//                case DROP:{
//                    if(!drive.isBusy()){
//                        leftLatch_Controller.CS = leftLatch_Controller.leftLatchStatus.OPEN;
//                        rightLatch_Controller.CS = rightLatch_Controller.rightLatchStatus.OPEN;
//                        status = status.GO_TO_STACK;
//                    }
//                    break;
//                }
//
//                case GO_TO_STACK:{
//                    drive.followTrajectorySequenceAsync(PREPARE_FOR_STACK);
//                    drive.followTrajectorySequenceAsync(GO_STACK);
//                    status = status.COLLECT;
//                    break;
//                }
//
//                case COLLECT:{
//                    if(!drive.isBusy()){
//                        collectAngle_Controller.CS = collectAngle_Controller.collectAngleStatus.STACK;
//                        robot.collect.setPower(1);
//                        if(robot.left_pixel.getState() && robot.right_pixel.getState()){
//                            leftLatch_Controller.CS = leftLatch_Controller.leftLatchStatus.CLOSE;
//                            rightLatchController.CS = rightLatch_Controller.rightLatchStatus.CLOSE;
//                            robot.collect.setPower(-0.5);
//                            status = status.RETURN;
//                        }
//                    }
//                    break;
//                }
//
//            }
//
//        }
//
//    }
//}
