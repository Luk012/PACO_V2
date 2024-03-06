package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.fourBar_Controller.fourbarStatus.INITIALIZE;

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
    }

    public fourBar_Controller(robotMap r){
        
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static fourbarStatus CS = INITIALIZE, PS = INITIALIZE;

    public static double init = 0.2;
    public static double up = 0.88;
    public static double inter = 0.4;
    public static double collect = 0.2;

    public void update(robotMap r)
    {
      

        if(CS != PS || CS == INITIALIZE)
        {

            switch(CS)
            {

                case INTER:
                {
                    r.right_fourbar.setPosition(inter);
                    r.left_fourbar.setPosition(inter);
                    break;
                }
                case INITIALIZE:
                {
                    r.right_fourbar.setPosition(init);
                    r.left_fourbar.setPosition(init);
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
            }
        }

        PS = CS;
    }

}
