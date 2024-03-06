package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.collectAngle_Controller.collectAngleStatus.INITIALIZE;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class collectAngle_Controller {

    public enum collectAngleStatus
    {
        INITIALIZE,
        LIFTED,
        GROUND,
    }

    public collectAngle_Controller(){
       CS = INITIALIZE;
       PS = INITIALIZE;

    }

    public static collectAngleStatus CS = INITIALIZE, PS = INITIALIZE;

    public static double init = 0;
    public static double lifted = 1;
    public static double ground = 0.55;

    public void update(robotMap r)
    {

               if(PS != CS || CS == INITIALIZE)
               {
                   switch(CS)
                   {
                       case INITIALIZE:
                       {
                           r.collect_angle.setPosition(init);
                           break;
                       }

                       case LIFTED:
                       {
                           r.collect_angle.setPosition(lifted);
                           break;
                       }

                       case GROUND:
                       {
                           r.collect_angle.setPosition(ground);
                           break;
                       }
                   }
               }
               PS = CS;
    }

}
