package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.rightLatch_Controller.rightLatchStatus.CLOSE;
import static org.firstinspires.ftc.teamcode.system_controllers.rightLatch_Controller.rightLatchStatus.CLOSE_2;
import static org.firstinspires.ftc.teamcode.system_controllers.rightLatch_Controller.rightLatchStatus.CLOSE_DONE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class rightLatch_Controller{

    public enum rightLatchStatus
    {
        INITIALIZE,
        OPEN,
        CLOSE,
        CLOSE_2,
        CLOSE_DONE,
    }

    public rightLatch_Controller()
    {
        CS = rightLatchStatus.INITIALIZE;
                PS = rightLatchStatus.INITIALIZE;
    }

    public static rightLatchStatus CS = rightLatchStatus.INITIALIZE, PS = rightLatchStatus.INITIALIZE;

    public static double init = 0.07;
    public static double open = 0.55;
    public static double close = 0.07;

    ElapsedTime latch = new ElapsedTime();

    public void update(robotMap r)
    {
        if(CS != PS || CS == rightLatchStatus.INITIALIZE || CS == CLOSE_2 || CS == CLOSE || CS == CLOSE_DONE)
        {
            switch (CS)
            {
                case INITIALIZE:
                {
                    r.right_latch.setPosition(init);
                    break;
                }

                case OPEN:
                {
                    r.right_latch.setPosition(open);
                    break;
                }


                case CLOSE:
                {
                    latch.reset();
                    CS = CLOSE_2;
                    break;
                }

                case CLOSE_2:
                {
                    if(latch.seconds() > 0.75)
                    {
                        r.right_latch.setPosition(close);

                    }
                    if (latch.seconds() > 0.95)
                    {
                        CS = CLOSE_DONE;
                    }
                    break;
                }
            }

            PS = CS;
        }
    }
}
