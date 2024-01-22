package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.lift_Controller.liftStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.lift_Controller.liftStatus.UP;

import org.firstinspires.ftc.teamcode.globals.SimplePIDController;
import org.firstinspires.ftc.teamcode.globals.robotMap;

public class lift_Controller {

    public enum liftStatus
    {
        INITIALIZE,
        UP,
        DOWN,
    }

    public double CurrentSpeed=0;
    public static double P1 = 0;
    public static double I1 = 0;
    public static double D1 = 0;

    public static double P2 = 0;
    public static double I2 = 0;
    public static double D2 = 0;

    public double pid = 0;

    public double Kg = 0;
    public double maxSpeedUp = 1;

    public static liftStatus CS = liftStatus.INITIALIZE, PS = liftStatus.INITIALIZE;

    SimplePIDController LiftPIDUP = null;
    SimplePIDController LiftPIDDOWN = null;

    int base = 0;
    int highest = 600;

    public static double up = 225;
    public static int upCnt = 0;
    public static double MultiplicationIndex = 55;

    public int CurrentPosition = 0;

    public lift_Controller()
    {
        LiftPIDUP = new SimplePIDController(P1,I1,D1);
        LiftPIDUP.targetValue = base;
        LiftPIDUP.maxOutput = maxSpeedUp;

        LiftPIDDOWN = new SimplePIDController(P2,I2,D2);
        LiftPIDDOWN.targetValue = base;
        LiftPIDDOWN.maxOutput = maxSpeedUp;
    }

    public void update(robotMap r, int position, double voltage)
    {

        CurrentPosition = position;

        if(CS == UP) LiftPIDUP.targetValue = up + upCnt * MultiplicationIndex;

        double powerLiftUp = LiftPIDUP.update(position) + Kg;
        powerLiftUp = Math.max(-1,Math.min(powerLiftUp* 14 / voltage,1));
        CurrentSpeed=powerLiftUp;

        double powerLiftDown = LiftPIDDOWN.update(position) + Kg;
        powerLiftDown = Math.max(-1,Math.min(powerLiftDown* 14 / voltage,1));
        CurrentSpeed=powerLiftDown;


        double powerLiftFinal;

        powerLiftFinal = (pid == 0) ? powerLiftDown : powerLiftUp;

        r.lift.setPower(powerLiftFinal);

        if(CS != PS || CS == INITIALIZE)
        {
            switch (CS)
            {
                case INITIALIZE:
                {
                        LiftPIDUP.targetValue = base;
                        break;
                }

                case UP:
                {
                    LiftPIDUP.targetValue = up + upCnt * MultiplicationIndex;
                    break;
                }

                case DOWN:
                {
                    LiftPIDDOWN.targetValue = base;
                    break;
                }
            }
        }

PS = CS;
    }

}
