package org.firstinspires.ftc.teamcode.system_controllers;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class fourBar_Controller {

    public enum fourbarStatus
    {
        INITIALIZE,
        SCORE,
        COLLECT,
    }

    public fourBar_Controller(){
        CS = fourbarStatus.INITIALIZE;
        PS = fourbarStatus.INITIALIZE;
    }

    public static fourbarStatus CS = fourbarStatus.INITIALIZE, PS = fourbarStatus.INITIALIZE;

    public static double init = 0;
    public static double up = 1;
    public static double collect = 0;

    public void update(robotMap r)
    {
        if(CS != PS || CS == fourbarStatus.INITIALIZE)
        {
            switch(CS)
            {
                case INITIALIZE:
                {
                    r.right.setPosition(init);
                    r.left.setPosition(init);
                    break;
                }

                case SCORE:
                {
                    r.right.setPosition(up);
                    r.left.setPosition(up);
                    break;
                }

                case COLLECT:
                {
                    r.right.setPosition(collect);
                    r.left.setPosition(collect);
                    break;
                }
            }
        }

        PS = CS;
    }

}
