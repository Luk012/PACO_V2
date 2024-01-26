package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.system_controllers.pto_Controller.ptoStatus.OFF;
import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.globals.InverseKinematics;
import org.firstinspires.ftc.teamcode.globals.robotMap;
import org.firstinspires.ftc.teamcode.system_controllers.collectAngle_Controller;
//import org.firstinspires.ftc.teamcode.system_controllers.drone_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.fourBar_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.leftLatch_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.lift_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.pto_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.rightLatch_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.storageAngle_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.storage_Controller;


@TeleOp(name="ik", group="OpMode")
public class TestOpMode extends LinearOpMode {



    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() {
        robotMap r = new robotMap(hardwareMap);

        collectAngle_Controller collectAngle = new collectAngle_Controller();
       // drone_Controller drone = new drone_Controller();
      fourBar_Controller fourBar = new fourBar_Controller(r);
       // leftLatch_Controller leftLatch = new leftLatch_Controller();
       //pto_Controller pto = new pto_Controller();
      //  rightLatch_Controller rightLatch = new rightLatch_Controller();
        storage_Controller storage = new storage_Controller();
      //  storageAngle_Controller storageAngle = new storageAngle_Controller();
       lift_Controller lift = new lift_Controller();
       // outtake_Controller outtake = new outtake_Controller();
        InverseKinematics ik = new InverseKinematics(r.right_fourbar, r.left_fourbar, r.storage,r.back);
//lift.CS = lift_Controller.liftStatus.DOWN;


        double voltage;
        double loopTime = 0;
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltage = batteryVoltageSensor.getVoltage();


//storageAngle.CS = storageAngle_Controller.storageAngleStatus.ROTATION;

       collectAngle.update(r);
       //drone.update(r);
       fourBar.update(r);
      // leftLatch.update(r);
        //pto.update(r);
       //rightLatch.update(r);
       // storageAngle.update(r);
       storage.update(r);
       lift.update(r, 0, voltage);
       // outtake.update( fourBar, storage, storageAngle, lift);



        boolean StrafesOn = true;
        boolean stack = false;

        //lift.upCnt = 0;
       // storageAngle.rotation_i = 0;



        double SpeedLimit = 1;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        double collect_power = 0;

        waitForStart();



        while (opModeIsActive() && !isStopRequested()) {

            int position = r.lift.getCurrentPosition();



            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

              double power = gamepad2.right_trigger - gamepad2.left_trigger;

              r.collect.setPower(power);

            if(!previousGamepad2.cross && currentGamepad2.cross)
            {
                    stack = !stack;
            }


                storage.CS = storage_Controller.storageStatus.IK;
                fourBar.CS = fourBar_Controller.fourbarStatus.IK;
                ik.updateServoPositions();


//            if(!previousGamepad2.circle && currentGamepad2.circle){
//                if(rightLatch.CS != rightLatch_Controller.rightLatchStatus.OPEN)
//                {
//                    rightLatch.CS = rightLatch_Controller.rightLatchStatus.OPEN;
//                } else
//                {
//                    rightLatch.CS = rightLatch_Controller.rightLatchStatus.CLOSE;
//                }
//            }

           collectAngle.update(r);
            //drone.update(r);
           fourBar.update(r);
          // leftLatch.update(r);
          // pto.update(r);
           // rightLatch.update(r);
           //storageAngle.update(r);
          storage.update(r);
           lift.update(r, position, voltage);
          // ik.updateServoPositions();
            //outtake.update(fourBar, storage, storageAngle, lift);

            double loop = System.nanoTime();

            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            telemetry.addData("status", collectAngle.CS);

            loopTime = loop;
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());


            telemetry.update();
        }
    }
}

