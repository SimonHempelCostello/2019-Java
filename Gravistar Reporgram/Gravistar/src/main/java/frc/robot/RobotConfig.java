/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * Add your docs here.
 */
public class RobotConfig {
    public void setUPFollowerTalon(TalonSRX leadTalon, TalonSRX followerTallon){
        followerTallon.set(ControlMode.Follower, leadTalon.getDeviceID());
    }
    public void setStartingConfig(){
        RobotMap.shifters.set(RobotMap.highGear);
        for(TalonSRX talon:RobotMap.allMotors){
            talon.configFactoryDefault();
        }
        RobotConfig.setAllMotorsBrake();
        RobotConfig.setDriveMotorsCoast();
        
        RobotMap.rightDriveLead.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
		RobotMap.leftDriveLead.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
        RobotMap.armMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
        
        RobotMap.rightDriveFollowerOne.set(ControlMode.Follower, RobotMap.rightMasterTalonID);
        RobotMap.leftDriveFollowerOne.set(ControlMode.Follower, RobotMap.leftMasterTalonID);
        RobotMap.armFollower.set(ControlMode.Follower, RobotMap.armMasterID);
        
        RobotMap.rightDriveLead.setInverted(true);
        RobotMap.rightDriveFollowerOne.setInverted(InvertType.FollowMaster);
        
    	RobotMap.leftDriveLead.setInverted(false);
        RobotMap.leftDriveFollowerOne.setInverted(InvertType.FollowMaster);
        		
		RobotMap.armMaster.setInverted(false);
        RobotMap.armFollower.setInverted(InvertType.FollowMaster);
        
        RobotMap.intakeMotor.setInverted(false);
        
    	RobotMap.leftDriveLead.setSelectedSensorPosition(0, 0,0);
        RobotMap.rightDriveLead.setSelectedSensorPosition(0, 0, 0);
        RobotMap.armMaster.setSelectedSensorPosition((int)RobotMap.arm.mainArmEncoder.convertAngleToEncoderTics(RobotStats.armUpAngle));
        RobotMap.armMaster.setSensorPhase(true);
        
    	for(TalonSRX talon:RobotMap.allMotorLeads) {
    		talon.configContinuousCurrentLimit(RobotStats.driveMotorContinuousCurrentHighGear);
    		talon.configPeakCurrentLimit(RobotStats.driveMotorPeakCurrentHighGear);
            talon.configPeakCurrentDuration(RobotStats.driveMotorPeakCurrentDurationHighGear);
            talon.enableCurrentLimit(false);
		}

    }
    public void setTeleopConfig(){
    }
    public void setAutoConfig(){

    }
    public static void setAllMotorsBrake() {
		for(TalonSRX talon:RobotMap.allMotors){
            talon.setNeutralMode(NeutralMode.Brake);
        }
	}
	public static void setDriveMotorsCoast() {
		for(TalonSRX talon:RobotMap.driveMotors){
            talon.setNeutralMode(NeutralMode.Coast);
        }
	}

	public static void setDriveMotorsBrake() {
		for(TalonSRX talon:RobotMap.driveMotors){
            talon.setNeutralMode(NeutralMode.Brake);
        }
	}
}
