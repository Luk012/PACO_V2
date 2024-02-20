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
        CLOSE_AUTO,
        CLOSE_AUTO_2,
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
    ElapsedTime latch_auto = new ElapsedTime();

    public void update(robotMap r)
    {
        if(CS != PS || CS == leftLatchStatus.INITIALIZE || CS == leftLatchStatus.CLOSE || CS == leftLatchStatus.CLOSE_2 || CS == leftLatchStatus.CLOSE_DONE || CS == leftLatchStatus.CLOSE_AUTO || CS == leftLatchStatus.CLOSE_AUTO_2)
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

                case CLOSE_AUTO:
                {
                    latch_auto.reset();
                    CS = leftLatchStatus.CLOSE_AUTO_2;
                    break;
                }

                case CLOSE_AUTO_2:
                {
                    if(latch_auto.seconds() > 0.4){
                        r.left_latch.setPosition(close);
                    }
                    if(latch_auto.seconds() > 0.55){
                        CS = leftLatchStatus.CLOSE_DONE;
                    }
                    break;
                }

            }
        }

        PS = CS;
    }

}
