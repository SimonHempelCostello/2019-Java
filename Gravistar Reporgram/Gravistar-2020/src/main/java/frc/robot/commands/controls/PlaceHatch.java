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
import edu.wpi.first.wpilibj.command.WaitCommand;

public class PlaceHatch extends Command {
  private WaitCommand wait1;
  private WaitCommand wait2;
  private double startTime;
  private boolean startTimer;
  public PlaceHatch() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    RobotMap.arm.setHatchMechOut();
    startTimer = true;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(!ButtonMap.releaseHatch()){
      if(startTimer){
        RobotMap.arm.releaseHatchGrabbers();
        startTime = Timer.getFPGATimestamp();
        startTimer = false;
      }
      else if(Timer.getFPGATimestamp()-startTime>0.35){
        RobotMap.arm.setHatchMechIn();
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !ButtonMap.releaseHatch()&&(Timer.getFPGATimestamp()-startTime)>0.75;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    RobotMap.arm.tenseHatchGrabbers();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
