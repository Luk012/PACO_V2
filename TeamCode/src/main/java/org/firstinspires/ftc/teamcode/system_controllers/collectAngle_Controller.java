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
        STACK,
        GROUND,
    }

    public collectAngle_Controller(){
       CS = INITIALIZE;
       PS = INITIALIZE;

    }

    public static collectAngleStatus CS = INITIALIZE, PS = INITIALIZE;

    public static double init = 0;
    public static double lifted = 0.6;
    public static double ground = 0.1;

    public static double stack[] = {0.1, 0.2, 0.28, 0.35, 0.44};

    public int stack_level = 0;

    public void update(robotMap r)
    {
        if(CS == collectAngleStatus.STACK)
        {
            r.collect_angle.setPosition(stack[stack_level]);
        }

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

                       case STACK:
                       {
                           r.collect_angle.setPosition(stack[stack_level]);
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
