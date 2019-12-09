/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.controls;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;

public class SetRobotOdometryPostion extends Command {
  private double x;
  private double y;
  private double heading;
  public SetRobotOdometryPostion(double desiredX, double desiredY, double desiredHeading) {
    x = desiredX;
    y = desiredY;
    heading = desiredHeading;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    RobotMap.drive.setDriveTrainX(x);
    RobotMap.drive.setDriveTrainY(y);
    RobotMap.drive.setDriveTrainHeading(heading);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
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
