//package org.firstinspires.ftc.teamcode.system_controllers;
//
//import com.acmerobotics.dashboard.config.Config;
//
//import org.firstinspires.ftc.teamcode.globals.robotMap;
//
//@Config
//public class drone_Controller{
//    public enum droneStatus
//    {
//        INITIALIZE,
//        RELEASED,
//    }
//
//    public drone_Controller(){
//        CS = droneStatus.INITIALIZE;
//        PS = droneStatus.INITIALIZE;
//    }
//
//    public static droneStatus CS = droneStatus.INITIALIZE, PS = droneStatus.INITIALIZE;
//
//    public static double init = 0;
//    public static double released = 0;
//
//    public void update(robotMap r)
//    {
//        if(PS != CS && CS == droneStatus.INITIALIZE)
//        {
//            switch (CS)
//            {
//                case INITIALIZE:
//                {
//                    r.drone.setPosition(init);
//                    break;
//                }
//
//                case RELEASED:
//                {
//                    r.drone.setPosition(released);
//                    break;
//                }
//            }
//        }
//
//        PS = CS;
//    }
//
//}
