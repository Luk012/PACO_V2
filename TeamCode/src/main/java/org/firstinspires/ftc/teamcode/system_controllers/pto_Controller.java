package org.firstinspires.ftc.teamcode.system_controllers;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class pto_Controller {

    public enum ptoStatus{
        INITIALIZE,
        ON,
        OFF,
    }

    public pto_Controller()
    {
        CS = ptoStatus.INITIALIZE;
        PS = ptoStatus.INITIALIZE;
    }

    public static ptoStatus CS = ptoStatus.INITIALIZE, PS = ptoStatus.INITIALIZE;

    public static double init = 0;
    public static double on = 1;
    public static double off = 0;

    public void update(robotMap r)
    {
        if(CS != PS || CS == ptoStatus.INITIALIZE)
        {
            switch (CS)
            {
                case INITIALIZE:
                {
                    r.pto.setPosition(init);
                    break;
                }

                case ON:
                {
                    r.pto.setPosition(on);
                    break;
                }

                case OFF:
                {
                    r.pto.setPosition(off);
                    break;
                }
            }
        }

        PS = CS;
    }

}
