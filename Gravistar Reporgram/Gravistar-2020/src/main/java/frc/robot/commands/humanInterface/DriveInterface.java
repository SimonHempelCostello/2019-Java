/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*---------------------------------------------------------------------------*/

package frc.robot.commands.humanInterface;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.RobotMap;
import frc.robot.RobotStats;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ButtonMap;

public class DriveInterface extends Command {
	private boolean shouldFinish;
	public DriveInterface() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(RobotMap.drive);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		shouldFinish = false;
		RobotMap.drive.initVelocityPIDs();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if(ButtonMap.testVelocity()){
			RobotMap.visionRelay1.set(Value.kForward);
			RobotMap.drive.trackVisionTape();
		}
		else{
			RobotMap.visionRelay1.set(Value.kReverse);
		 	RobotMap.visionRelay1.set(RobotMap.lightRingOff);
		 	RobotMap.drive.arcadeDrive();
		}

	}
	public void forceEnd(){
		shouldFinish = true;
	}
	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return RobotState.isDisabled()||shouldFinish;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		RobotMap.drive.stopDriveTrainMotors();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		this.end();
	}
}
