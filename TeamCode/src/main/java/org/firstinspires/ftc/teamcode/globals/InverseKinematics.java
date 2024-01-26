package org.firstinspires.ftc.teamcode.globals;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.system_controllers.fourBar_Controller;
import org.firstinspires.ftc.teamcode.system_controllers.storage_Controller;

@Config
public class InverseKinematics {

    private Servo right4Bar;
    private Servo left4Bar;
    private Servo angleServo;
    private DistanceSensor distanceSensor;

    // Constants for distance range and servo
    public static double MAX_DISTANCE = 200;
    public static double MIN_DISTANCE = 10;
    public static double FOUR_BAR_LENGTH = 235;
    public static double distance_offset = -80;

    public static double fourbar_offset = 0;
    public static double angle_offset = 0.65;



    public InverseKinematics(Servo right4Bar, Servo left4Bar,Servo angleServo, DistanceSensor distanceSensor) {
        this.right4Bar = right4Bar;
        this.left4Bar = left4Bar;
        this.angleServo = angleServo;
        this.distanceSensor = distanceSensor;
    }

    public void updateServoPositions() {
        double distance = distanceSensor.getDistance(DistanceUnit.MM);
        distance += distance_offset;

        distance = Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, distance));

        double[] targetAngles = calculateJointAngles(distance);

        if (targetAngles != null) {

 fourBar_Controller.inverse_kinematics = targetAngles[0];
 storage_Controller.inverse_kinematics = targetAngles[1];
        }
    }

    public double[] calculateJointAngles(double distance) {

        double fourbarTarget = fourbar_offset + ((Math.toDegrees(Math.acos(distance / FOUR_BAR_LENGTH))) / ((355.0 / 48.0) * 40.0));
        double AngleTarget = angle_offset - (60-(Math.toDegrees(Math.acos(distance / FOUR_BAR_LENGTH)))) / ((355.0*40.0)/48.0*24.0/16.0);

       double normalizedForbarTarget = Math.min(1, Math.max(fourbarTarget, 0));
        double normalizedAngleTarget = Math.min(1, Math.max(AngleTarget, 0));

        return new double[] { normalizedForbarTarget, normalizedAngleTarget };
    }


    public void updateServoPosition(Servo servo, double targetAngle) {

        double servoPosition = convertAngleToServoPosition(targetAngle);
        servo.setPosition(servoPosition);
    }

    private double convertAngleToServoPosition(double angle) {
        return angle;
    }
}
