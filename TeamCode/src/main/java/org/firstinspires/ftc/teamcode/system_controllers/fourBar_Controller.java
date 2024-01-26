package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.fourBar_Controller.fourbarStatus.INITIALIZE;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.globals.CoolServo;
import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class fourBar_Controller {

    public final CoolServo left,right;

    public static boolean reversedLeftServo = false, reversedRightServo = true;
    public static double profileMaxVelocity = 10, profileAcceleration = 3, profileDeceleration = 15;


    public enum fourbarStatus
    {
        INITIALIZE,
        SCORE,
        COLLECT,
        INTER,
        IK,
        INTER_AUTO,
        PRELOAD_AUTO,
        FOR_LATCHES,
    }

    public fourBar_Controller(robotMap r){

        left = new CoolServo(r.left_fourbar, reversedLeftServo, profileMaxVelocity, profileAcceleration, profileDeceleration, 1);
        right = new CoolServo(r.right_fourbar, reversedRightServo, profileMaxVelocity, profileAcceleration, profileDeceleration, 1);
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static fourbarStatus CS = INITIALIZE, PS = INITIALIZE;

    public static double init = 0.865;
    public static double inter = 0.79;
    public static double up = 0.19;
    public static double collect = 0.865;
    public static double inverse_kinematics = 0.15;
    public static double interauto = 0.5;
    public static double score_preload_auto = 0.045;

    public void update(robotMap r)
    {
        left.update();
        right.update();

        if(CS == fourbarStatus.IK)
        {
            right.setPosition(inverse_kinematics);
            left.setPosition(inverse_kinematics);
        }

        if(CS != PS || CS == INITIALIZE)
        {

            switch(CS)
            {
                case INITIALIZE:
                {
                    right.setPosition(init);
                    left.setPosition(init);
                    break;
                }

                case INTER:
                {
                    right.setPosition(inter);
                    left.setPosition(inter);
                    break;
                }

                case SCORE:
                {
                    right.setPosition(up);
                    left.setPosition(up);
                    break;
                }

                case COLLECT:
                {
                    right.setPosition(collect);
                    left.setPosition(collect);
                    break;
                }

                case IK:
                {
                    right.setPosition(inverse_kinematics);
                    left.setPosition(inverse_kinematics);
                    break;
                }

                case INTER_AUTO:
                {
                    right.setPosition(interauto);
                    left.setPosition(interauto);
                    break;
                }

                case PRELOAD_AUTO:
                {
                    right.setPosition(score_preload_auto);
                    left.setPosition(score_preload_auto);
                    break;
                }

            }
        }

        PS = CS;
    }

}
