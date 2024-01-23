package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.fourBar_Controller.fourbarStatus.INITIALIZE;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.globals.CoolServo;
import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class fourBar_Controller {

    public final CoolServo left,right;

    public static boolean reversedLeftServo = false, reversedRightServo = true;
    public static double profileMaxVelocity = 16, profileAcceleration = 8, profileDeceleration = 8;


    public enum fourbarStatus
    {
        INITIALIZE,
        SCORE,
        COLLECT,
        INTER,
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

    public void update(robotMap r)
    {
        left.update();
        right.update();

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
            }
        }

        PS = CS;
    }

}
