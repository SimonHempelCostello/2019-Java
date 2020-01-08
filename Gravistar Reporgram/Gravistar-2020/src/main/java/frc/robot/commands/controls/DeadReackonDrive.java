/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.controls;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;

public class DeadReackonDrive extends Command {
  private double startTime;
  private double period;
  private double driveSpeed;
  public DeadReackonDrive(double time, double speed) {
    period = time;
    driveSpeed = speed;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTime = Timer.getFPGATimestamp();

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    RobotMap.drive.setLeftPercent(driveSpeed);
    RobotMap.drive.setRightPercent(driveSpeed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(Timer.getFPGATimestamp()-startTime)>=period;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    RobotMap.drive.setLeftPercent(0);
    RobotMap.drive.setRightPercent(0);
    System.out.println(driveSpeed);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
