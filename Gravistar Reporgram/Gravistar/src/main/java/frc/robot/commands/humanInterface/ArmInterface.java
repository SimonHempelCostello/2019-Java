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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ButtonMap;
import frc.robot.RobotMap;
import frc.robot.RobotStats;
import frc.robot.commands.controls.GrabHatch;
import frc.robot.commands.controls.PlaceHatch;
import frc.robot.commands.controls.GrabHatch;
import frc.robot.commands.controls.ClimbMechanismController;

public class ArmInterface extends Command {
  private boolean shouldFinish;
  private double desiredAngle  = 105;
  private GrabHatch grabHatch;
  private PlaceHatch placeHatch;
  private ClimbMechanismController climbMechanismController;
  public ArmInterface() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    grabHatch = new GrabHatch();
    placeHatch = new PlaceHatch();
    climbMechanismController = new ClimbMechanismController();
    RobotMap.arm.setHatchMechIn();
    desiredAngle = RobotMap.arm.getArmAngle();
    RobotMap.arm.setArmPostion(desiredAngle);
    RobotMap.arm.tenseHatchGrabbers();
    shouldFinish = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putNumber("positioner", RobotMap.arm.mainArmEncoder.getAngle());
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
    else if(ButtonMap.armUpOuttake()){
      desiredAngle = RobotStats.armUpOutTakeAngle;
    }
    else if(Math.abs(ButtonMap.armManualControlValue())>0.2){
      desiredAngle = RobotMap.arm.getArmAngle();
    }
    if(Math.abs(ButtonMap.armManualControlValue())>0.2){
      RobotMap.arm.setArmPercentPower(ButtonMap.armManualControlValue());
    }
    else if(RobotMap.arm.getArmAngle()<10&&desiredAngle == RobotStats.armRestingAngle){
      RobotMap.arm.setArmPercentPower(-0.1);
    }
    else{
      RobotMap.arm.setArmPostion(desiredAngle);
    }
    if(ButtonMap.releaseHatch()&&!grabHatch.isRunning()&&!placeHatch.isRunning()){
      placeHatch.start();
    }
    else if(ButtonMap.grabHatch()&&!placeHatch.isRunning()&&!grabHatch.isRunning()){
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

    if(ButtonMap.enableClimber()){
      climbMechanismController.start();
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
