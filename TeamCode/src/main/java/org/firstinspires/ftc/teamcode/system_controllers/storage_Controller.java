package org.firstinspires.ftc.teamcode.system_controllers;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class storage_Controller {

    public enum storageStatus
    {
        INITIALZIE,
        COLLECT,
        SCORE,
        INTER,
    }

    public storage_Controller()
    {
        CS= storageStatus.INITIALZIE;
        PS = storageStatus.INITIALZIE;
    }

    public static storageStatus CS = storageStatus.INITIALZIE, PS = storageStatus.INITIALZIE;

    public static double init = 0.048;
    public static double inter = 0.01;
    public static double collect = 0.048;
    public static double score = 0.33;

    public void update(robotMap r)
    {
        if(PS != CS || CS == storageStatus.INITIALZIE)
        {
            switch (CS)
            {
                case INITIALZIE:
                {
                    r.storage.setPosition(init);
                    break;
                }

                case INTER:
                {
                    r.storage.setPosition(inter);
                    break;
                }

                case SCORE:
                {
                    r.storage.setPosition(score);
                    break;
                }
                case COLLECT:
                {
                    r.storage.setPosition(collect);
                    break;
                }
            }

            PS = CS;
        }
    }

}
