package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.COLLECT;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.COLLECT2;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.COLLECTDONE;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.INTER;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.INTER2;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.INTER3;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.INTER4;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.SCORE;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.SCORE2;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.SCOREDONE;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedReader;

public class outtake_Controller {
    public enum outtakeStatus
    {
        INITIALIZE,
        SCORE,
        SCORE2,
        SCOREDONE,
        COLLECT,
        COLLECT2,
        COLLECTDONE,
        INTER,
        INTER2,
        INTER3,
        INTER4,
    }

    public outtake_Controller()
    {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static outtakeStatus CS = INITIALIZE, PS = INITIALIZE;

    ElapsedTime score = new ElapsedTime();
    ElapsedTime collect = new ElapsedTime();
    ElapsedTime inter = new ElapsedTime();

    public void update(fourBar_Controller fourbar, storage_Controller storage, storageAngle_Controller storageAngle, lift_Controller lift)
    {


            if(CS != PS || CS == INITIALIZE || CS == SCOREDONE || CS == COLLECTDONE || CS == SCORE || CS == SCORE2 || CS == COLLECT || CS == COLLECT2 || CS == INTER || CS == INTER2 || CS == INTER3 || CS == INTER4)
            {
                switch (CS)
                {

                    case INTER:
                    {
                        fourbar.CS = fourBar_Controller.fourbarStatus.INTER;
                        storage.CS = storage_Controller.storageStatus.INTER;
                        break;
                    }

                    case SCORE:
                    {
                        if(fourbar.CS == fourBar_Controller.fourbarStatus.INTER)
                        {lift.pid = 1;
                        lift.CS = lift_Controller.liftStatus.UP;
                        score.reset();
                       CS = SCORE2;} else
                        {
                            CS = outtakeStatus.INTER2;
                        }
                        break;
                    }

                    case INTER2:
                    {
                        fourbar.CS = fourBar_Controller.fourbarStatus.INTER;
                        storage.CS = storage_Controller.storageStatus.INTER;
                        inter.reset();
                        CS = INTER3;
                        break;
                    }

                    case INTER3:
                    {
                        if (inter.seconds() > 0.2)
                        {
                            CS = SCORE;
                        }
                        break;
                    }

                    case SCORE2:
                    {
                        if(score.seconds() > 0.2)
                        {
                            fourbar.CS = fourBar_Controller.fourbarStatus.SCORE;
                        }
                        if (score.seconds() > 0.3)
                        {
                            storage.CS = storage_Controller.storageStatus.SCORE;
                            CS = SCOREDONE;
                        }

                        break;
                    }

                    case COLLECT:
                    {
                        storageAngle.rotation_i = 2;
                        fourbar.CS = fourBar_Controller.fourbarStatus.INTER;
                        storage.CS = storage_Controller.storageStatus.INTER;
                        collect.reset();
                        CS = INTER4;
                            break;
                    }

                    case COLLECT2:
                    {
                        if(collect.seconds() > 0.2)
                        {
                            fourbar.CS = fourBar_Controller.fourbarStatus.COLLECT;
                            storage.CS = storage_Controller.storageStatus.COLLECT;
                            CS = COLLECTDONE;
                        }
                        break;
                    }

                    case INTER4:
                    {
                        if(collect.seconds() > 0.2)
                        {
                            lift.pid = 0;
                            lift.CS = lift_Controller.liftStatus.DOWN;
                            ;
                        CS = COLLECT2;}
                        break;
                    }
                }
            }

           PS = CS;
    }

}
