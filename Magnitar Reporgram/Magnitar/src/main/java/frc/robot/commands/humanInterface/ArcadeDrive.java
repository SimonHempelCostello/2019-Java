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

public class ArcadeDrive extends Command {
	private double deadZone = 0.00;
	private double turn =0;
	private double throttel = 0;
	private double povValue;
	private double ratio = 0;
	private double sensitivity;
	private double leftPower;
	private double rightPower;
	public ArcadeDrive() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(RobotMap.drive);

	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {

		throttel = ButtonMap.getDriveThrottle()*0.9; 
		ratio = Math.abs(throttel);
		povValue = ButtonMap.getPOV();
		turn = ButtonMap.getRotation();
			
		if(Math.abs(throttel)>RobotStats.joyStickDeadZone){
			leftPower = (throttel - (sensitivity*turn*ratio));
			rightPower = (throttel + (sensitivity*turn*ratio));
			RobotConfig.setDriveMotorsCoast();
		}
		else{
			leftPower = (-turn)*sensitivity;
			rightPower = (turn)*sensitivity; 
		}
		if(ButtonMap.quickTurn()) {
			leftPower = throttel +(-turn);
			rightPower= throttel +(turn);
		}
		if(Math.abs(leftPower)>1) {
			leftPower = (leftPower/Math.abs(leftPower));
			rightPower = Math.abs(rightPower/leftPower)*(rightPower/Math.abs(rightPower));
		}
		else if(Math.abs(rightPower)>1) {
			rightPower = (rightPower/Math.abs(rightPower));
			leftPower = Math.abs(leftPower/rightPower)*(leftPower/Math.abs(leftPower));
		}
		RobotMap.leftDriveLead.set(ControlMode.PercentOutput, leftPower);
		RobotMap.rightDriveLead.set(ControlMode.PercentOutput, rightPower);
		if(ButtonMap.shiftDown()){
			System.out.println("down");
			RobotMap.drive.setLowGear();
		}
		else if(ButtonMap.shiftUp()) {
			System.out.println("up");
			RobotMap.drive.setHighGear();
		}
		if(RobotMap.shifters.get() == RobotMap.highGear) {
				sensitivity =1.25;
		}
		else if(RobotMap.shifters.get() == RobotMap.lowGear) {
				sensitivity =0.95;
		}

	}
	
	
	 
	

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return RobotState.isDisabled();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.stopMotors.stopDriveTrainMotors();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		this.end();
	}
}
