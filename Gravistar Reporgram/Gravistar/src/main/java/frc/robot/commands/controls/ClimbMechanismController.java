/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.controls;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ButtonMap;
import frc.robot.OI;
import frc.robot.RobotMap;

public class ClimbMechanismController extends Command {

  public ClimbMechanismController() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putNumber("climbingmechPosition", RobotMap.climbingMechLeadTalon.getSelectedSensorPosition());
    SmartDashboard.putBoolean("climbingMechControllerActicve",true);
    if(Math.abs(ButtonMap.climbingMotorPower())>0.1){
      RobotMap.climbingMechLeadTalon.set(ControlMode.PercentOutput, ButtonMap.climbingMotorPower());
    }
    else{
      RobotMap.climbingMechLeadTalon.set(ControlMode.PercentOutput, 0);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
