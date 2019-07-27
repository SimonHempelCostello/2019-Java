/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class RobotStats {
    public static double robotBaseDistance = 2.5;
    public static double robotMaxAccertion= 8;
    public static double robotMaxVelocity = 14;
    public static double encoderTicsPerWheelRotation = 800.0;
    public static double wheelDiam = 6.5;
    public static double wheelCircum = wheelDiam*Math.PI;
    public static double joyStickDeadZone = 0.015;
    public static double triggerDeadZone = 0.1;
    public static int driveMotorContinuousCurrentHighGear = 20;
    public static int driveMotorPeakCurrentHighGear = 30;
    public static int driveMotorPeakCurrentDurationHighGear = 100;
    public static double armTicksToAngleConversion=0.02470588;
    public static double armAngleToTicksConversion =1/armTicksToAngleConversion;
	public static double armUpAngle =106;
    public static double armRestingAngle = -1;
    public static double armOutTakeAngle = 70;
    public static double armKfFactor = 0.0;

}
