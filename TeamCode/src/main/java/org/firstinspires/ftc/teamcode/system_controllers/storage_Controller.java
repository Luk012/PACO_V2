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
        INTER,
        IK,
        INTER_AUTO,
        PRELOAD_AUTO,
    }

    public storage_Controller()
    {
        CS= storageStatus.INITIALZIE;
        PS = storageStatus.INITIALZIE;
    }

    public static storageStatus CS = storageStatus.INITIALZIE, PS = storageStatus.INITIALZIE;

    public static double init = 0.35;
    public static double inter = 0.28;
    public static double collect = 0.35;
    public static double score = 0.64;
    public static double inverse_kinematics = 0.15;
    public static double score_auto_preload = 0.6;

    public void update(robotMap r)
    {
        if(CS == storageStatus.IK)
        {
            r.storage.setPosition(inverse_kinematics);
        }

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
                case IK:
                {
                    r.storage.setPosition(inverse_kinematics);
                    break;
                }

                case PRELOAD_AUTO:
                {
                    r.storage.setPosition(score_auto_preload);
                    break;
                }
            }

            PS = CS;
        }
    }

}
