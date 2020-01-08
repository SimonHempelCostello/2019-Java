/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.controls;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.ButtonMap;
import frc.robot.RobotMap;

public class GrabHatch extends Command {
  private double startTime;
  private boolean startTimer;
  public GrabHatch() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTimer = true;
    RobotMap.arm.setHatchMechOut();
    RobotMap.arm.releaseHatchGrabbers();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(!ButtonMap.grabHatch()){
      if(startTimer){
        startTime = Timer.getFPGATimestamp();
        startTimer = false;
        RobotMap.arm.tenseHatchGrabbers();
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !ButtonMap.grabHatch()&&(Timer.getFPGATimestamp()-startTime>0.45);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    RobotMap.arm.setHatchMechIn();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
