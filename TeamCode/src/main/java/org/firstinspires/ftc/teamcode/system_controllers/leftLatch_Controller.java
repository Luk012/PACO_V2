package org.firstinspires.ftc.teamcode.system_controllers;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class leftLatch_Controller {

    public enum leftLatchStatus
    {
        INITIALIZE,
        OPEN,
        CLOSE,
    }

    public leftLatch_Controller(){
        CS = leftLatchStatus.INITIALIZE;
        PS = leftLatchStatus.INITIALIZE;
    }

    public static leftLatchStatus CS = leftLatchStatus.INITIALIZE, PS = leftLatchStatus.INITIALIZE;

    public static double init = 0;
    public static double open = 1;
    public static double close = 0;

    public void update(robotMap r)
    {
        if(CS != PS || CS == leftLatchStatus.INITIALIZE)
        {
            switch (CS)
            {
                case INITIALIZE:
                {
                    r.left_latch.setPosition(init);
                    break;
                }

                case OPEN:
                {
                    r.left_latch.setPosition(open);
                    break;
                }

                case CLOSE:
                {
                    r.left_latch.setPosition(close);
                    break;
                }
            }
        }

        PS = CS;
    }

}
