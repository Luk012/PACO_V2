package org.firstinspires.ftc.teamcode.globals;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.BuiltInConfigurationTypeJsonAdapter;

import org.firstinspires.ftc.teamcode.globals.CoolServo;
import org.firstinspires.ftc.teamcode.system_controllers.fourBar_Controller;

public class robotMap{


    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;

    public DcMotor lift_left = null;
    public DcMotor lift_right  = null;

    public DcMotor collect = null;

    public Servo left_fourbar = null;
    public Servo right_fourbar = null;

    public Servo storage = null;

    public Servo right_latch = null;
    public Servo left_latch = null;

    public Servo collect_angle = null;

   public Servo drone = null;


    //public IMU imu;


    public robotMap(HardwareMap Init)
    {

        rightFront = Init.get(DcMotor.class,"rightFront");
        leftFront = Init.get(DcMotor.class,"leftFront");
        rightBack = Init.get(DcMotor.class,"rightBack");
        leftBack = Init.get(DcMotor.class,"leftBack");

        lift_left = Init.get(DcMotor.class, "lift_left");
        lift_right = Init.get(DcMotor.class, "lift_right");
        collect = Init.get(DcMotor.class, "collect");

        left_fourbar = Init.get(Servo.class, "left_fourbar");
        right_fourbar = Init.get(Servo.class, "right_fourbar");

        storage = Init.get(Servo.class, "storage");

        right_latch = Init.get(Servo.class, "right_latch");
        left_latch = Init.get(Servo.class, "left_latch");

        collect_angle = Init.get(Servo.class, "collect_angle");

       drone = Init.get(Servo.class, "drone");


        //imu = Init.get(IMU.class, "imu");

//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
//        ));
//        imu.initialize(parameters);
//
//        imu.resetYaw();


        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift_left.setDirection(DcMotorSimple.Direction.FORWARD);
        lift_right.setDirection(DcMotorSimple.Direction.REVERSE);
        collect.setDirection(DcMotorSimple.Direction.REVERSE);



        collect_angle.setDirection(Servo.Direction.FORWARD);

       storage.setDirection(Servo.Direction.FORWARD);

       left_latch.setDirection(Servo.Direction.FORWARD);
       right_latch.setDirection(Servo.Direction.FORWARD);

       right_fourbar.setDirection(Servo.Direction.REVERSE);

      drone.setDirection(Servo.Direction.FORWARD);
//
//        ((CRServoImplEx) left_fourbar).setPwmRange(new PwmControl.PwmRange(500, 2500, 5000));
//        ((CRServoImplEx) right_fourbar).setPwmRange(new PwmControl.PwmRange(500, 2500, 5000));
//
//


    }


}


