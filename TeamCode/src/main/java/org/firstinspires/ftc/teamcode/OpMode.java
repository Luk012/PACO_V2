package org.firstinspires.ftc.teamcode;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.globals.robotMap;
import org.firstinspires.ftc.teamcode.system_controllers.collectAngle_Controller;
//import org.firstinspires.ftc.teamcode.system_controllers.drone_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.drone_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.fourBar_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.leftLatch_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.lift_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.rightLatch_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.storage_Controller;


@TeleOp(name="TeleOP", group="OpMode")
public class OpMode extends LinearOpMode {


    public static double  PrecisionDenominatorTranslational = 1, PrecisionDenominatorAngle = 1;

    public void robotCentricDrive(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack, double  SpeedLimit, boolean StrafesOn , double LeftTrigger, double RightTrigger)
    {
        double y = -gamepad1.right_stick_y; // Remember, this is reversed!
        double x = gamepad1.right_stick_x*1.1;
        if (StrafesOn == false)
        {
            x=0;
        }
        double rx = gamepad1.left_stick_x*1 - LeftTrigger + RightTrigger;

        rx/=PrecisionDenominatorAngle;
        x/=PrecisionDenominatorTranslational;
        y/=PrecisionDenominatorTranslational;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftPower = Clip(frontLeftPower,SpeedLimit);
        backLeftPower = Clip(backLeftPower,SpeedLimit);
        frontRightPower = Clip(frontRightPower,SpeedLimit);
        backRightPower = Clip(backRightPower,SpeedLimit);

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }

    double Clip(double Speed, double lim)
    {
        return Math.max(Math.min(Speed,lim), -lim);
    }

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() {
        robotMap r = new robotMap(hardwareMap);

        collectAngle_Controller collectAngle = new collectAngle_Controller();
   // drone_Controller drone = new drone_Controller();
        fourBar_Controller fourBar = new fourBar_Controller(r);
leftLatch_Controller leftLatch = new leftLatch_Controller();
   rightLatch_Controller rightLatch = new rightLatch_Controller();
  storage_Controller storage = new storage_Controller();
 lift_Controller lift = new lift_Controller();
        outtake_Controller outtake = new outtake_Controller();

        ElapsedTime scuipa = new ElapsedTime();
        ElapsedTime timer = new ElapsedTime();
      //  ElapsedTime collect_timer = new ElapsedTime();

        double voltage;
        double loopTime = 0;
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
       voltage = batteryVoltageSensor.getVoltage();

      collectAngle.CS = collectAngle_Controller.collectAngleStatus.GROUND;
//      drone.CS = drone_Controller.droneStatus.INITIALIZE;
   fourBar.CS = fourBar_Controller.fourbarStatus.COLLECT;
     leftLatch.CS = leftLatch_Controller.leftLatchStatus.OPEN;
       rightLatch.CS = rightLatch_Controller.rightLatchStatus.OPEN;
   storage.CS = storage_Controller.storageStatus.COLLECT;
    lift.CS = lift_Controller.liftStatus.DOWN;
        outtake.CS = outtake_Controller.outtakeStatus.INITIALIZE;



   collectAngle.update(r);
//       drone.update(r);
  fourBar.update(r);
   leftLatch.update(r);
      rightLatch.update(r);
  storage.update(r);
       lift.update(r, 0, voltage);
        outtake.update( fourBar, storage, lift, rightLatch,leftLatch);



        boolean StrafesOn = true;
        boolean stack = true;
        boolean one_pixel = false;

//        lift.upCnt = 0;
//        collectAngle.stack_level = 0;




        double SpeedLimit = 1;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        telemetry.addData("Status", "Initialized");
        telemetry.update();



        ElapsedTime collect_timer = new ElapsedTime();
        double collect_power = 0;
        lift.upCnt = 0;
        waitForStart();

timer.reset();

        while (opModeIsActive() && !isStopRequested()) {


            int position = r.lift_right.getCurrentPosition();

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (!previousGamepad1.touchpad && currentGamepad1.touchpad) {
                StrafesOn = !StrafesOn;
            }

            robotCentricDrive(r.leftFront, r.leftBack, r.rightFront, r.rightBack, SpeedLimit , StrafesOn , 0,0);


            double collect_input = currentGamepad2.right_trigger - currentGamepad2.left_trigger;

            if(collect_input!=0) {
                collect_timer.reset();
                collect_power = currentGamepad2.right_trigger - currentGamepad2.left_trigger;
            }
            if(collect_timer.seconds() >= 1 || (collect_power < 0  && collect_input == 0)){
                collect_power = 0;
            }

            r.collect.setPower(collect_power);

            if(lift.CS == lift_Controller.liftStatus.DOWN)
            {
                if(collect_input > 0 )
                {
                    leftLatch.CS = leftLatch_Controller.leftLatchStatus.OPEN;
                    rightLatch.CS = rightLatch_Controller.rightLatchStatus.OPEN;
                    collectAngle.CS = collectAngle_Controller.collectAngleStatus.GROUND;
                }

                else if(collect_input <= 0)
                {
                    leftLatch.CS = leftLatch_Controller.leftLatchStatus.CLOSE;
                    rightLatch.CS = rightLatch_Controller.rightLatchStatus.CLOSE;
                    collectAngle.CS = collectAngle_Controller.collectAngleStatus.LIFTED;
                }
            }

            if(!previousGamepad2.cross && currentGamepad2.cross)
            {
                        if(outtake.CS != outtake_Controller.outtakeStatus.SCORE)
                        {
                            outtake.CS = outtake_Controller.outtakeStatus.SCORE;
                        } else
                        {
                            outtake.CS = outtake_Controller.outtakeStatus.COLLECT;
                        }
            }

            if(!previousGamepad2.dpad_up && currentGamepad2.dpad_up)
            {
                lift.upCnt = Math.min(15, lift.upCnt+1);
            }

            if(!previousGamepad2.dpad_down && currentGamepad2.dpad_down)
            {
                lift.upCnt = Math.max(0, lift.upCnt-1);
            }

            if(!previousGamepad2.triangle && currentGamepad2.triangle)
            {
                leftLatch.CS = leftLatch_Controller.leftLatchStatus.OPEN;
                rightLatch.CS = rightLatch_Controller.rightLatchStatus.OPEN;
            }

            if(!previousGamepad2.circle && currentGamepad2.circle)
            {
                if(leftLatch.CS != leftLatch_Controller.leftLatchStatus.OPEN)
                {
                    leftLatch.CS = leftLatch_Controller.leftLatchStatus.OPEN;
                } else
                {
                    rightLatch.CS = rightLatch_Controller.rightLatchStatus.OPEN;
                }
            }



        collectAngle.update(r);
         //  drone.update(r);
            fourBar.update(r);
          leftLatch.update(r);
            rightLatch.update(r);
           storage.update(r);
            lift.update(r, position, voltage);
            outtake.update( fourBar, storage, lift, rightLatch,leftLatch);


            double loop = System.nanoTime();

            telemetry.addData("hz ", 1000000000 / (loop - loopTime));

            loopTime = loop;
//            telemetry.addData( "lift status", lift.CS);
//
//            telemetry.addData("collecy angle", collectAngle.CS);
//                    telemetry.addData("stack lever", collectAngle.stack_level);
//                    telemetry.addData("stack" ,stack);
            telemetry.addData("fourbar", fourBar.CS);
            telemetry.addData("collect_input", collect_input);
            telemetry.addData("collect_power", collect_power);
            telemetry.addData("angle", storage.CS);


//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());


            telemetry.update();
        }
    }
}

