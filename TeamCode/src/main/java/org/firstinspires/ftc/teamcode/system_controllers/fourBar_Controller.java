package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.fourBar_Controller.fourbarStatus.COLLECT;
import static org.firstinspires.ftc.teamcode.system_controllers.fourBar_Controller.fourbarStatus.IK;
import static org.firstinspires.ftc.teamcode.system_controllers.fourBar_Controller.fourbarStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.fourBar_Controller.fourbarStatus.INTER;
import static org.firstinspires.ftc.teamcode.system_controllers.fourBar_Controller.fourbarStatus.INTER_AUTO;
import static org.firstinspires.ftc.teamcode.system_controllers.fourBar_Controller.fourbarStatus.PRELOAD_AUTO;
import static org.firstinspires.ftc.teamcode.system_controllers.fourBar_Controller.fourbarStatus.SCORE;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.globals.CoolServo;
import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class fourBar_Controller {

    
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

    public fourBar_Controller(){
        CS = fourbarStatus.INITIALIZE;
        PS = fourbarStatus.INITIALIZE;
    }

    public static fourbarStatus CS = fourbarStatus.INITIALIZE, PS = fourbarStatus.INITIALIZE;

    public static double init = 0.865;
    public static double inter = 0.79;
    public static double up = 0.19;
    public static double collect = 0.865;
    public static double inverse_kinematics = 0.15;
    public static double interauto = 0.5;
    public static double score_preload_auto = 0.045;

    public void update(robotMap r)
    {
       

        if(CS == fourbarStatus.IK)
        {
            r.right_fourbar.setPosition(inverse_kinematics);
            r.left_fourbar.setPosition(inverse_kinematics);
        }

        if(CS != PS || CS == INITIALIZE || CS == INTER || CS == SCORE || CS == COLLECT || CS == IK || CS == INTER_AUTO || CS == PRELOAD_AUTO)
        {

            switch(CS)
            {
                case INITIALIZE:
                {
                    r.right_fourbar.setPosition(init);
                    r.left_fourbar.setPosition(init);
                    break;
                }

                case INTER:
                {
                    r.right_fourbar.setPosition(inter);
                    r.left_fourbar.setPosition(inter);
                    break;
                }

                case SCORE:
                {
                    r.right_fourbar.setPosition(up);
                    r.left_fourbar.setPosition(up);
                    break;
                }

                case COLLECT:
                {
                    r.right_fourbar.setPosition(collect);
                    r.left_fourbar.setPosition(collect);
                    break;
                }

                case IK:
                {
                    r.right_fourbar.setPosition(inverse_kinematics);
                    r.left_fourbar.setPosition(inverse_kinematics);
                    break;
                }

                case INTER_AUTO:
                {
                    r.right_fourbar.setPosition(interauto);
                    r.left_fourbar.setPosition(interauto);
                    break;
                }

                case PRELOAD_AUTO:
                {
                    r.right_fourbar.setPosition(score_preload_auto);
                    r.left_fourbar.setPosition(score_preload_auto);
                    break;
                }

            }
        }

        PS = CS;
    }

}
