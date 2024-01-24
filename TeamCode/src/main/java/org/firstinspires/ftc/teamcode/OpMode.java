package org.firstinspires.ftc.teamcode;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;

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
      //  drone_Controller drone = new drone_Controller();
        fourBar_Controller fourBar = new fourBar_Controller(r);
        leftLatch_Controller leftLatch = new leftLatch_Controller();
        pto_Controller pto = new pto_Controller();
        rightLatch_Controller rightLatch = new rightLatch_Controller();
        storage_Controller storage = new storage_Controller();
        storageAngle_Controller storageAngle = new storageAngle_Controller();
        lift_Controller lift = new lift_Controller();
        outtake_Controller outtake = new outtake_Controller();


        double voltage;
        double loopTime = 0;
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
       voltage = batteryVoltageSensor.getVoltage();

       collectAngle.CS = collectAngle_Controller.collectAngleStatus.GROUND;
      // drone.CS = drone_Controller.droneStatus.INITIALIZE;
       fourBar.CS = fourBar_Controller.fourbarStatus.COLLECT;
       leftLatch.CS = leftLatch_Controller.leftLatchStatus.OPEN;
       pto.CS = pto_Controller.ptoStatus.OFF;
       rightLatch.CS = rightLatch_Controller.rightLatchStatus.OPEN;
       storageAngle.CS = storageAngle_Controller.storageAngleStatus.INITIALIZE;
       storage.CS = storage_Controller.storageStatus.COLLECT;
       lift.CS = lift_Controller.liftStatus.DOWN;
       outtake.CS = outtake_Controller.outtakeStatus.INITIALIZE;



       collectAngle.update(r);
       //drone.update(r);
       fourBar.update(r);
       leftLatch.update(r);
       pto.update(r);
       rightLatch.update(r);
       storageAngle.update(r);
       storage.update(r);
       lift.update(r, 0, voltage);
       outtake.update( fourBar, storage, storageAngle, lift);



        boolean StrafesOn = true;
        boolean stack = true;

        lift.upCnt = 0;
        storageAngle.rotation_i = 2;
        collectAngle.stack_level = 0;




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

            if (!previousGamepad1.touchpad && currentGamepad1.touchpad) {
                StrafesOn = !StrafesOn;
            }

            robotCentricDrive(r.leftFront, r.leftBack, r.rightFront, r.rightBack, SpeedLimit , StrafesOn , 0,0);

          collect_power = gamepad2.right_trigger- gamepad2.left_trigger;

              r.collect.setPower(collect_power);

          if(!previousGamepad2.square && currentGamepad2.square)
          {
              stack = !stack;
          }

          if(stack)
          {
              if(collect_power > 0)
              {
                  collectAngle.CS = collectAngle_Controller.collectAngleStatus.STACK;
              }
              else
              {
                  collectAngle.CS = collectAngle_Controller.collectAngleStatus.LIFTED;
              }
          } else
          {
              if(collect_power > 0)
              {
                  collectAngle.CS = collectAngle_Controller.collectAngleStatus.GROUND;
              }
              else
              {
                  collectAngle.CS = collectAngle_Controller.collectAngleStatus.GROUND;
              }
          }


            if(lift.CS == lift_Controller.liftStatus.DOWN) {

                if (r.left_pixel.getState() == FALSE && leftLatch.CS == leftLatch_Controller.leftLatchStatus.OPEN) {
                        leftLatch.CS = leftLatch_Controller.leftLatchStatus.CLOSE;
                }
                if(r.right_pixel.getState() == FALSE && rightLatch.CS == rightLatch_Controller.rightLatchStatus.OPEN)
                {
                    rightLatch.CS = rightLatch_Controller.rightLatchStatus.CLOSE;
                }
            }

          if(lift.CS == lift_Controller.liftStatus.DOWN && leftLatch.CS == leftLatch_Controller.leftLatchStatus.CLOSE_DONE && rightLatch.CS == rightLatch_Controller.rightLatchStatus.CLOSE_DONE && outtake.CS != outtake_Controller.outtakeStatus.INTER && r.right_pixel.getState() == FALSE && r.left_pixel.getState() == FALSE )
          {
              outtake.CS = outtake_Controller.outtakeStatus.INTER;
          }


          if(r.left_pixel.getState() == FALSE && r.right_pixel.getState() == FALSE && collect_power > 0)
          {
              gamepad1.rumble(100);
              gamepad2.rumble(100);
          }

          if(!previousGamepad2.cross && currentGamepad2.cross)
          {
              if(lift.CS != lift_Controller.liftStatus.DOWN)
              {
                  storageAngle.rotation_i = 0;
                  outtake.CS = outtake_Controller.outtakeStatus.COLLECT;
              }
              else
              {
                  storageAngle.CS = storageAngle_Controller.storageAngleStatus.ROTATION;
                  outtake.CS = outtake_Controller.outtakeStatus.SCORE;
              }
          }

            if(!previousGamepad2.dpad_up && currentGamepad2.dpad_up)
            {
                lift.upCnt = Math.min(20, lift.upCnt+1);
            }

            if(!previousGamepad2.dpad_down && currentGamepad2.dpad_down)
            {
                lift.upCnt = Math.max(0, lift.upCnt-1);
            }


            if(!previousGamepad2.right_bumper && currentGamepad2.right_bumper && lift.CS == lift_Controller.liftStatus.UP)
            {

                    storageAngle.rotation_i = Math.min(4, storageAngle.rotation_i+1);
            }

            if(!previousGamepad2.left_bumper && currentGamepad2.left_bumper && lift.CS == lift_Controller.liftStatus.UP)
            {
                  storageAngle.rotation_i = Math.max(0, storageAngle.rotation_i-1);
            }

            if(!previousGamepad2.dpad_right && currentGamepad2.dpad_right)
            {
                if(storageAngle.rotation_i == 4)
                {
                    storageAngle.rotation_i = 0;
                } else
                if(storageAngle.rotation_i == 3)
                {
                    storageAngle.rotation_i = 1;
                } else
                if(storageAngle.rotation_i == 1)
                {
                    storageAngle.rotation_i = 3;
                } else
                if(storageAngle.rotation_i == 0)
                {
                    storageAngle.rotation_i = 4;
                }
            }

            if(!previousGamepad2.circle && currentGamepad2.circle)
            {
                if(lift.CS == lift_Controller.liftStatus.UP)
                {
                    if(storageAngle.rotation_i == 2 || storageAngle.rotation_i == 3 || storageAngle.rotation_i == 4)
                    {
                            if(leftLatch.CS == leftLatch_Controller.leftLatchStatus.CLOSE)
                            {
                                leftLatch.CS = leftLatch_Controller.leftLatchStatus.OPEN;
                            } else
                            {
                                rightLatch.CS = rightLatch_Controller.rightLatchStatus.OPEN;
                            }
                    } else
                    {
                        if(rightLatch.CS == rightLatch_Controller.rightLatchStatus.CLOSE)
                        {
                            rightLatch.CS = rightLatch_Controller.rightLatchStatus.OPEN;
                        } else
                        {
                           leftLatch.CS = leftLatch_Controller.leftLatchStatus.OPEN;
                        }
                    }
                }
            }

            if(!previousGamepad2.triangle && currentGamepad2.triangle)
            {
                leftLatch.CS = leftLatch_Controller.leftLatchStatus.OPEN;
                rightLatch.CS = rightLatch_Controller.rightLatchStatus.OPEN;
            }

            if(!previousGamepad1.dpad_left && currentGamepad1.dpad_left)
            {
                if(pto.CS != pto_Controller.ptoStatus.ON)
                {
                    pto.CS = pto_Controller.ptoStatus.ON;
                } else
                    pto.CS = pto_Controller.ptoStatus.OFF;
            }

            if(!previousGamepad1.dpad_up && currentGamepad1.dpad_up)
            {
                collectAngle.stack_level = Math.min(4, collectAngle.stack_level+1);
            }

            if(!previousGamepad1.dpad_down && currentGamepad1.dpad_down)
            {
                collectAngle.stack_level = Math.max(0, collectAngle.stack_level-1);
            }

//            if(!previousGamepad2.dpad_left && currentGamepad2.dpad_left)
//            {
//                drone.CS = drone_Controller.droneStatus.RELEASED;
//            }

            double hang_power = gamepad1.right_trigger - gamepad1.left_trigger;

            if(pto.CS == pto_Controller.ptoStatus.ON)
            {
                collectAngle.stack_level =3;
                collectAngle.CS = collectAngle_Controller.collectAngleStatus.STACK;
                r.collect.setPower(hang_power);
            }

            collectAngle.update(r);
           // drone.update(r);
            fourBar.update(r);
            leftLatch.update(r);
            pto.update(r);
            rightLatch.update(r);
            storageAngle.update(r);
            storage.update(r);
            lift.update(r, position, voltage);
            outtake.update(fourBar, storage, storageAngle, lift);

            double loop = System.nanoTime();

            telemetry.addData("hz ", 1000000000 / (loop - loopTime));

            loopTime = loop;
            telemetry.addData( "lift status", lift.CS);
            telemetry.addData("outtake", outtake.CS);
            telemetry.addData("left_pixel", r.left_pixel.getState());
            telemetry.addData("right_pixel", r.right_pixel.getState());
            telemetry.addData("collecy angle", collectAngle.CS);
                    telemetry.addData("stack lever", collectAngle.stack_level);
                    telemetry.addData("stack" ,stack);
            telemetry.addData("fourbar", fourBar.CS);


//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());


            telemetry.update();
        }
    }
}

