package org.firstinspires.ftc.teamcode.AUTO_CONTROLLERS;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.globals.InverseKinematics;
import org.firstinspires.ftc.teamcode.globals.robotMap;
import org.firstinspires.ftc.teamcode.system_controllers.fourBar_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.leftLatch_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.lift_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.rightLatch_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.storageAngle_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.storage_Controller;

import java.io.BufferedReader;


public class Blue_LEFT {
    public enum autoControllerStatus
    {
        NOTHING,
        SCORE_PRELOAD,
        SCORE_PRELOAD_LIFT,
        SCORE_PRELOAD_SYSTEMS,
        INTER_COLLECT,
        SCORE_PRELOAD_DONE,
        COLLECT,
        COLLECT_SYSTEMS,
        COLLECT_DONE,
        SCORE,
        INTER_SCORE,
        INTER_SCORE_PRELAOD,
        SCORE_LIFT,
        SCORE_SYSTEMS,
        SCORE_DONE,
        INTER,

        PRELOAD_GROUND,
        PRELOAD_GROUND_2,
        PRELOAD_GROUND_3,
       PRELOADGROUNDDONE,
        SCORE_LIFT_2,
        SCORE_2,
        SCORE_PRELOAD_AUTO,
        SCORE_PRELOAD_LIFT_2,
        SCORE_PRELOAD_SYSTEMS_2,
        SCORE_26,
        SCORE_262,
        RETRACT_SYSTEMS,

    }
    public static autoControllerStatus CurrentStatus = autoControllerStatus.NOTHING, PreviousStatus = autoControllerStatus.NOTHING;

    ElapsedTime pruple_prelaod = new ElapsedTime();
    ElapsedTime yellow_preload = new ElapsedTime();
    ElapsedTime scorev2 = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime inter = new ElapsedTime();
    ElapsedTime slow = new ElapsedTime();
    ElapsedTime slow2 = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();
    ElapsedTime retract = new ElapsedTime();

    public void update(robotMap r, lift_Controller lift, InverseKinematics ik, fourBar_Controller fourbar, storage_Controller storage, storageAngle_Controller storageAngle, leftLatch_Controller leftLatch, rightLatch_Controller rightLatch)
    {
        switch (CurrentStatus)
        {

            case INTER:
            {
                fourbar.CS = fourBar_Controller.fourbarStatus.INTER_AUTO;
                storage.CS = storage_Controller.storageStatus.INTER;
                CurrentStatus = autoControllerStatus.NOTHING;
                break;
            }

            case SCORE_PRELOAD:
            {
                ik.updateServoPositions();
                inter.reset();
                CurrentStatus = autoControllerStatus.SCORE_PRELOAD_LIFT;
                break;

            }

            case INTER_SCORE_PRELAOD:
        {
            fourbar.CS = fourBar_Controller.fourbarStatus.INTER;
            storage.CS = storage_Controller.storageStatus.INTER;
            inter.reset();
            CurrentStatus = autoControllerStatus.SCORE_PRELOAD_LIFT;
            break;
        }

            case SCORE_PRELOAD_LIFT:
            {
                if(inter.seconds() > 0.1)
                {
                    lift.pid =1;
                    lift.CS = lift_Controller.liftStatus.UP;
                    scorev2.reset();
                    CurrentStatus = autoControllerStatus.SCORE_PRELOAD_SYSTEMS;
                }
                break;
            }


            case SCORE_26:{
                timer2.reset();
                CurrentStatus = autoControllerStatus.SCORE_262;
                break;
            }

            case SCORE_262:{
                if(timer2.seconds() > 1.45)
                {
                    fourbar.CS = fourBar_Controller.fourbarStatus.SCORE;
                }
                if(timer2.seconds() > 1.55)
                {
                    storage.CS = storage_Controller.storageStatus.SCORE;
                }
                if(timer2.seconds() > 2.1)
                {
                    rightLatch.CS = rightLatch_Controller.rightLatchStatus.OPEN;
                    leftLatch.CS = leftLatch_Controller.leftLatchStatus.OPEN;
                }
                if(timer2.seconds() > 2.3){
                    fourbar.CS = fourBar_Controller.fourbarStatus.COLLECT;
                }
                if(timer2.seconds() > 2.4){
                    storage.CS = storage_Controller.storageStatus.COLLECT;
                }
                break;
            }

            case SCORE_PRELOAD_SYSTEMS:
            {
                if(scorev2.seconds() > 0.15)
                {
                    storage.CS = storage_Controller.storageStatus.SCORE;

                }
                if(scorev2.seconds() > 0.35)
                {
                    fourbar.CS = fourBar_Controller.fourbarStatus.SCORE;
                }
                if(scorev2.seconds() > 1)
                {
                    CurrentStatus = autoControllerStatus.SCORE_PRELOAD_DONE;
                }
                break;
            }

            case COLLECT:
            {
                storageAngle.rotation_i = 2;
                lift.pid= 0;
                lift.CS = lift_Controller.liftStatus.DOWN;
                scorev2.reset();
                CurrentStatus = autoControllerStatus.INTER_COLLECT;
                break;
            }

            case INTER_COLLECT:
            {
                if(scorev2.seconds() > 0.1)
            {
                fourbar.CS = fourBar_Controller.fourbarStatus.INTER;
            }
                if(scorev2.seconds() > 0.3)
                {
                storage.CS = storage_Controller.storageStatus.INTER;
                inter.reset();
                CurrentStatus = autoControllerStatus.COLLECT_SYSTEMS;}

                break;
            }

            case PRELOAD_GROUND:
            {
                fourbar.CS = fourBar_Controller.fourbarStatus.INTER;
                storage.CS = storage_Controller.storageStatus.INTER;
                timer.reset();
                CurrentStatus =autoControllerStatus.PRELOAD_GROUND_2;
                break;
            }

            case PRELOAD_GROUND_2:
            {
               if(timer.seconds() > 0.2)
               {
                   fourbar.CS = fourBar_Controller.fourbarStatus.PRELOAD_AUTO;
               }

               if(timer.seconds() > 0.45)
               {
                   storage.CS = storage_Controller.storageStatus.PRELOAD_AUTO;
                   pruple_prelaod.reset();
                   CurrentStatus = autoControllerStatus.PRELOAD_GROUND_3;
               }

                break;
            }

            case PRELOAD_GROUND_3:
            {
                if(pruple_prelaod.seconds() > 0.3)
                {
                    CurrentStatus = autoControllerStatus.PRELOADGROUNDDONE;
                }
                break;
            }



    case COLLECT_SYSTEMS:
    {
        if(inter.seconds() > 0.35) {
            fourbar.CS = fourBar_Controller.fourbarStatus.COLLECT;
            storage.CS = storage_Controller.storageStatus.COLLECT;
        }

        if(inter.seconds() > 0.7)
        {
            CurrentStatus = autoControllerStatus.COLLECT_DONE;
        }
        break;
    }

            case SCORE:
            {
                ik.updateServoPositions();
                inter.reset();
                CurrentStatus = autoControllerStatus.SCORE_LIFT;
                break;
            }

            case SCORE_PRELOAD_AUTO:
            {
                ik.updateServoPositions();
                slow.reset();
                CurrentStatus = autoControllerStatus.SCORE_PRELOAD_LIFT_2;
                break;
            }

            case SCORE_2:{
                ik.updateServoPositions();
                inter.reset();
                CurrentStatus = autoControllerStatus.SCORE_LIFT_2;
                break;
            }
            case SCORE_LIFT_2:{
                if(inter.seconds() > 0.1)
                {
//                    lift.pid =1;
//                    lift.CurrentPosition = 0;
//                    lift.CS = lift_Controller.liftStatus.UP;
//                    scorev2.reset();
                    CurrentStatus = autoControllerStatus.SCORE_SYSTEMS;
                }
                break;
            }

            case INTER_SCORE:
            {fourbar.CS = fourBar_Controller.fourbarStatus.INTER;
                storage.CS = storage_Controller.storageStatus.INTER;
                inter.reset();
                CurrentStatus = autoControllerStatus.SCORE_LIFT;
                break;
            }

            case SCORE_LIFT:
            {
                if(inter.seconds() > 0.1)
                {
                    lift.pid =1;
                    lift.CS = lift_Controller.liftStatus.UP;
                    scorev2.reset();
                    CurrentStatus = autoControllerStatus.SCORE_SYSTEMS;
                }
                break;
            }

            case SCORE_PRELOAD_LIFT_2:
            {
                if(slow.seconds() > 0.1)
                {
                    lift.pid = 1;
                    lift.CS = lift_Controller.liftStatus.UP;
                    slow2.reset();
                    CurrentStatus = autoControllerStatus.SCORE_PRELOAD_SYSTEMS_2;
                }
                break;
            }

            case SCORE_SYSTEMS:
            {
                if(scorev2.seconds() > 0.15)
                {
                    storage.CS = storage_Controller.storageStatus.SCORE;

                }
                if(scorev2.seconds() > 0.35)
                {
                    fourbar.CS = fourBar_Controller.fourbarStatus.SCORE;
                }
                if(scorev2.seconds() > 1.5)
                {
                    CurrentStatus = autoControllerStatus.SCORE_DONE;
                }
                break;
            }

            case SCORE_PRELOAD_SYSTEMS_2:
            {
                if(slow2.seconds() > 0.15)
                {
                    fourbar.CS = fourBar_Controller.fourbarStatus.SCORE;
                }
                if(slow2.seconds() > 0.25)
                {
                    storage.CS = storage_Controller.storageStatus.SCORE;
                }
                if(slow2.seconds() > 1.5)
                {
                    CurrentStatus = autoControllerStatus.SCORE_DONE;
                }
                break;
            }

            case RETRACT_SYSTEMS:{
                storageAngle.rotation_i = 2;
                if(retract.seconds() > 1.1)
                {
                    fourbar.CS = fourBar_Controller.fourbarStatus.INTER;
                }
                if(retract.seconds() > 1.3)
                {
                    storage.CS = storage_Controller.storageStatus.INTER;
                }
                break;
            }

        }
        PreviousStatus = CurrentStatus;
    }
}

