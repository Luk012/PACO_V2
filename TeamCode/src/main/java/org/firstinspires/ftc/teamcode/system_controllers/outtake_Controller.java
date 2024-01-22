package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.COLLECT2;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.COLLECTDONE;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.SCORE;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.SCORE2;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.SCOREDONE;

import com.qualcomm.robotcore.util.ElapsedTime;

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
    }

    public outtake_Controller()
    {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static outtakeStatus CS = INITIALIZE, PS = INITIALIZE;

    ElapsedTime score = new ElapsedTime();
    ElapsedTime collect = new ElapsedTime();

    public void update(fourBar_Controller fourbar, storage_Controller storage, storageAngle_Controller storageAngle, lift_Controller lift)
    {
            if(CS != PS || CS == INITIALIZE || CS == SCOREDONE || CS == COLLECTDONE)
            {
                switch (CS)
                {

                    case SCORE:
                    {
                        lift.pid = 1;
                        lift.CS = lift_Controller.liftStatus.UP;
                        score.reset();
                       CS = COLLECT2;
                        break;
                    }

                    case SCORE2:
                    {
                        if(score.seconds() > 0.3)
                        {
                            fourbar.CS = fourBar_Controller.fourbarStatus.SCORE;
                            storage.CS = storage_Controller.storageStatus.SCORE;
                            CS = SCOREDONE;
                        }

                        break;
                    }

                    case COLLECT:
                    {
                        lift.pid = 0;
                        lift.CS = lift_Controller.liftStatus.DOWN;
                        collect.reset();
                        CS = COLLECT2;
                            break;
                    }

                    case COLLECT2:
                    {
                        if(collect.seconds() > 0.3)
                        {
                            storageAngle.rotation_i = 2;
                        }
                        if(collect.seconds() > 0.5)
                        {
                            fourbar.CS = fourBar_Controller.fourbarStatus.COLLECT;
                            storage.CS = storage_Controller.storageStatus.COLLECT;
                            CS = COLLECTDONE;
                        }
                        break;
                    }
                }
            }

           PS = CS;
    }

}
