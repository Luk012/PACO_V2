package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.COLLECT;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.COLLECT2;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.COLLECTDONE;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.INTER;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.INTER2;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.INTER3;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.INTER4;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.INTER_INTER;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.SCORE;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.SCORE2;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.SCOREDONE;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.SECURE_LATCHES_FOR_2_PIXELS2;
import static org.firstinspires.ftc.teamcode.system_controllers.outtake_Controller.outtakeStatus.SECURE_LATCHES_FOR_2_PIXELS_DONE;

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
        INTER,
        INTER2,
        INTER3,
        INTER4,
        SECURE_LATCHES_FOR_2_PIXELS,
        SECURE_LATCHES_FOR_2_PIXELS2,
        SECURE_LATCHES_FOR_2_PIXELS_DONE,
        INTER_INTER,
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
    ElapsedTime latches = new ElapsedTime();

    public void update(fourBar_Controller fourbar, storage_Controller storage, lift_Controller lift, rightLatch_Controller right_latch, leftLatch_Controller left_latch)
    {


            if(CS != PS || CS == INITIALIZE || CS == SCOREDONE || CS == COLLECTDONE || CS == SCORE || CS == SCORE2 || CS == COLLECT || CS == COLLECT2 || CS == INTER || CS == INTER2 || CS == INTER3 || CS == INTER4 || CS == INTER_INTER)
            {
                switch (CS)
                {

                    case INTER:
                    {
                        fourbar.CS = fourBar_Controller.fourbarStatus.INTER;
                        latches.reset();
                        CS = INTER_INTER;
                        break;
                    }

                    case INTER_INTER:
                    {
                        if(latches.seconds() > 0.2)
                        {
                            lift.pid = 1;
                            lift.CS = lift_Controller.liftStatus.UP;
                            score.reset();
                            CS = SCORE;
                        }
                        break;
                    }

                    case SCORE:
                    {
                        if(score.seconds() > 0.2)
                        {
                            fourbar.CS = fourBar_Controller.fourbarStatus.SCORE;
                            storage.CS = storage_Controller.storageStatus.SCORE;
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
                        if(collect.seconds() > 0.2)
                        {
                            fourbar.CS = fourBar_Controller.fourbarStatus.COLLECT;
                            storage.CS = storage_Controller.storageStatus.COLLECT;
                        }
                        break;
                    }


                }
            }

           PS = CS;
    }

}
