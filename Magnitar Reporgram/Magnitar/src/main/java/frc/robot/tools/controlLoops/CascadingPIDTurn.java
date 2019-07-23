/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.tools.controlLoops;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.RobotMap;
import frc.robot.RobotStats;
import frc.robot.sensors.DriveEncoder;
import frc.robot.sensors.Navx;

public class CascadingPIDTurn extends Command {
  private Navx navx;
  private VelocityPID leftDriveTrainVelocityPID;
  private VelocityPID rightDriveTrainVelocityPID;
  private PID turnPID;
  private double desiredAngle;
  private DriveEncoder leftSideDriveEncoder;
  private DriveEncoder rightSideDriveEncoder;
  private double p;
  private double i;
  private double d;
  private boolean shouldEnd;
  public CascadingPIDTurn(double Angle, double kp, double ki, double kd) {
    desiredAngle = Angle;
    p = kp;
    i = ki;
    d = kd;
    //requires(RobotMap.drive);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    shouldEnd = false;
    leftDriveTrainVelocityPID = new VelocityPID(0, RobotMap.leftDriveLead, 1, 0.33230122, 0.7, 0.008, 18.0);
    rightDriveTrainVelocityPID = new VelocityPID(0, RobotMap.rightDriveLead, 1, 0.33230122, 0.7, 0.008, 18.0);
    turnPID =  new PID(p,i,d );
    navx = new Navx(RobotMap.navx);
    leftSideDriveEncoder = new DriveEncoder(RobotMap.leftDriveLead, RobotMap.leftDriveLead.getSelectedSensorPosition(0));
    rightSideDriveEncoder = new DriveEncoder(RobotMap.rightDriveLead, RobotMap.rightDriveLead.getSelectedSensorPosition(0));
    turnPID.setMaxOutput(RobotStats.robotMaxVelocity);
    turnPID.setMinOutput(-RobotStats.robotMaxVelocity);
    turnPID.setSetPoint(desiredAngle);
    leftDriveTrainVelocityPID.start();
    rightDriveTrainVelocityPID.start();
    navx.softResetYaw();
  }
  public void setTarget(double target){
    turnPID.setSetPoint(target);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    turnPID.updatePID(navx.currentAngle());
    leftDriveTrainVelocityPID.changeDesiredSpeed(turnPID.getResult());
    rightDriveTrainVelocityPID.changeDesiredSpeed(-turnPID.getResult());
  }
  public void forceFinish(){
    shouldEnd = true;
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(Math.abs(navx.currentAngle()-desiredAngle)<0.5){
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    leftDriveTrainVelocityPID.cancel();
    rightDriveTrainVelocityPID.cancel();
    Robot.stopMotors.stopDriveTrainMotors();

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  }
}
