package org.firstinspires.ftc.teamcode.system_controllers;

import org.firstinspires.ftc.teamcode.globals.robotMap;

public class storageAngle_Controller {

    public enum storageAngleStatus{
        INITIALIZE,
        ROTATION,

    }

    public storageAngle_Controller()
    {
        CS = storageAngleStatus.INITIALIZE;
        PS = storageAngleStatus.INITIALIZE;
    }

    public static storageAngleStatus CS = storageAngleStatus.INITIALIZE, PS = storageAngleStatus.INITIALIZE;

    public static double init = 0;
    public static double rotation[] = {0, 0, 0, 0, 0};
    public static int rotation_i = 0;

    public void update(robotMap r)
    {
        if(CS != PS || CS == storageAngleStatus.INITIALIZE)
        {
            switch (CS){

                case INITIALIZE:
                {
                    r.storage_angle.setPosition(init);
                    break;
                }



                case ROTATION:
                {
                    r.storage_angle.setPosition(rotation[rotation_i]);
                    break;
                }



            }
        }

        PS = CS;
    }
}
