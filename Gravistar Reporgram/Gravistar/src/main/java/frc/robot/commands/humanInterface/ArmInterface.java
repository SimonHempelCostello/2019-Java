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
import frc.robot.commands.controls.GrabHatch;
import frc.robot.commands.controls.PlaceHatch;
import frc.robot.commands.controls.GrabHatch;

public class ArmInterface extends Command {
  private boolean shouldFinish;
  private double desiredAngle  = 105;
  private GrabHatch grabHatch;
  private PlaceHatch placeHatch;
  public ArmInterface() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    grabHatch = new GrabHatch();
    placeHatch = new PlaceHatch();
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
    if(Math.abs(ButtonMap.armManualControlValue())>0.1){
      RobotMap.arm.setArmPercentPower(ButtonMap.armManualControlValue()*0.5);
      desiredAngle = RobotMap.arm.mainArmEncoder.getAngle();
    }
    else{
      RobotMap.arm.setArmPostion(desiredAngle);
    }
    if(ButtonMap.releaseHatch()){
      placeHatch.start();
    }
    else if(ButtonMap.grabHatch()){
      grabHatch.start();
    }
    if(ButtonMap.outTakeBall()){
      RobotMap.arm.outTakeBall();
    }
    else if(ButtonMap.inTakeBall()){
      RobotMap.arm.inTakeBall();
    }
    else{
      RobotMap.arm.intakeRest();
    }
    


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
