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
    public static double robotMaxVelocity = 8;
    public static double encoderTicsPerWheelRotation = 4096;
    public static double wheelDiam = 7.25;
    public static double wheelCircum = wheelDiam*Math.PI;
    public static double joyStickDeadZone = 0.015;
    public static double triggerDeadZone = 0.1;
    public static int driveMotorContinuousCurrentHighGear = 30;
    public static int driveMotorPeakCurrentHighGear = 40;
    public static int intakePeakCurrent = 40;
    public static int intakeContinuousCurrent = 20;
    public static int driveMotorPeakCurrentDurationHighGear = 50;
    public static double armTicksToAngleConversion=0.02470588;
    public static double armAngleToTicksConversion =1/armTicksToAngleConversion;
	public static double armUpAngle =105;
    public static double armRestingAngle = 0;
    public static double armOutTakeAngle = 70;
    public static double armKfFactor = 0.0;

}
