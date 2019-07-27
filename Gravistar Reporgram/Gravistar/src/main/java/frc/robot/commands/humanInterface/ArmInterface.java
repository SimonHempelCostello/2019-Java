/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.humanInterface;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.ButtonMap;
import frc.robot.RobotMap;
import frc.robot.RobotStats;

public class ArmInterface extends Command {
  private boolean shouldFinish;
  private double desiredAngle  = 105;
  public ArmInterface() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    shouldFinish = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    RobotMap.arm.initializeArmControl();
    if(RobotMap.armMaster.getSensorCollection().isFwdLimitSwitchClosed()){
      RobotMap.arm.mainArmEncoder.setForwardLimitSwitchAngle();
    }
    else if(RobotMap.armMaster.getSensorCollection().isRevLimitSwitchClosed()){
      RobotMap.arm.mainArmEncoder.setReverseLimitSwitchAngle();
    }

    if(ButtonMap.armOuttake()){
      desiredAngle = RobotStats.armOutTakeAngle;
    }
    else if(ButtonMap.armResting()){
      desiredAngle = RobotStats.armRestingAngle;
    }
    else if(ButtonMap.armUp()){
      desiredAngle = RobotStats.armUpAngle;
    }
    RobotMap.arm.setArmPostion(desiredAngle);
    


  }

  public void forceEnd(){
    shouldFinish = true;
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(shouldFinish||RobotState.isDisabled()){
      return true;
    }
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
