package org.firstinspires.ftc.teamcode.system_controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class leftLatch_Controller {

    public enum leftLatchStatus
    {
        INITIALIZE,
        OPEN,
        CLOSE,
        CLOSE_2,
        CLOSE_DONE,
    }

    public leftLatch_Controller(){
        CS = leftLatchStatus.INITIALIZE;
        PS = leftLatchStatus.INITIALIZE;
    }

    public static leftLatchStatus CS = leftLatchStatus.INITIALIZE, PS = leftLatchStatus.INITIALIZE;

    public static double init = 0.665;
    public static double open = 0.215;
    public static double close = 0.665;

    ElapsedTime latch = new ElapsedTime();

    public void update(robotMap r)
    {
        if(CS != PS || CS == leftLatchStatus.INITIALIZE || CS == leftLatchStatus.CLOSE || CS == leftLatchStatus.CLOSE_2 || CS == leftLatchStatus.CLOSE_DONE)
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
                    latch.reset();
                    CS = leftLatchStatus.CLOSE_2;
                    break;
                }

                case CLOSE_2:
                {
                    if(latch.seconds() > 0.75)
                    {
                        r.left_latch.setPosition(close);

                    }
                    if (latch.seconds() > 0.95)
                    {
                        CS = leftLatchStatus.CLOSE_DONE;
                    }
                    break;
                }
            }
        }

        PS = CS;
    }

}
