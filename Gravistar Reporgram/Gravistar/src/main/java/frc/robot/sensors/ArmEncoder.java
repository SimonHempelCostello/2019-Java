/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Talon;
import frc.robot.RobotConfig;
import frc.robot.RobotStats;

/**
 * Add your docs here.
 */
public class ArmEncoder {
    private TalonSRX talon;
    private double startPosition;
    private double AngleConversion;
    public ArmEncoder(TalonSRX armTalon){
        talon = armTalon;
    }
    public double getRawPosition(){
        return talon.getSelectedSensorPosition();
    }
    public double getAngle(){
        return talon.getSelectedSensorPosition()*RobotStats.armTicksToAngleConversion; 
    }
    public double convertAngleToEncoderTics(double angle){
        return Math.round(RobotStats.armAngleToTicksConversion * angle);
    }
    public double convertTicsToAngle(double tics){
        return RobotStats.armTicksToAngleConversion * tics;
    }
    public void setForwardLimitSwitchAngle(){
        talon.setSelectedSensorPosition((int)(convertAngleToEncoderTics(RobotStats.armUpAngle)));
    }
    public void setReverseLimitSwitchAngle(){
        talon.setSelectedSensorPosition((int)(convertAngleToEncoderTics(RobotStats.armRestingAngle)));
    }
}
