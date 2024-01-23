package org.firstinspires.ftc.teamcode.system_controllers;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
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

    public static double init = 0.485;
    public static double rotation[] = {0.03, 0.26, 0.485, 0.72, 0.9235};
    public static int rotation_i = 2;
    public static double pos = 0.485;

    public void update(robotMap r)
    {

        if(CS == storageAngleStatus.ROTATION)
        {
            r.storage_angle.setPosition(rotation[rotation_i]);
        }

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
