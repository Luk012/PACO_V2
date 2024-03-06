package org.firstinspires.ftc.teamcode.system_controllers;

import android.support.v4.app.INotificationSideChannel;
import android.telecom.CallAudioState;

import com.acmerobotics.dashboard.config.Config;

import org.apache.commons.math3.distribution.AbstractRealDistribution;
import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class storage_Controller {

    public enum storageStatus
    {
        INITIALZIE,
        COLLECT,
        SCORE,

    }

    public storage_Controller()
    {
        CS= storageStatus.INITIALZIE;
        PS = storageStatus.INITIALZIE;
    }

    public static storageStatus CS = storageStatus.INITIALZIE, PS = storageStatus.INITIALZIE;

    public static double init = 0.85;
    public static double collect = 0.85;
    public static double score = 0.14;

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
