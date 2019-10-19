/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.tools.controlLoops;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.tools.math.Point;
import frc.robot.tools.math.Vector;
import Jama.Matrix;

public class CubicInterpolationFollower extends Command {
  private double xPI;
  private double yPI;
  private double xPF;
  private double yPF;
  private double xVI;
  private double yVI;
  private double xVF;
  private double yVF;
  private Point desiredPoint;
  private Vector desiredVelocity;
  private double finalTime;
  private double initialTime;
  private double deltaT;
  private Matrix xPositionAndVelocityMatrix;
  private Matrix yPositionAndVelocityMatrix;
  private Matrix timeFunctionMatrix;
  private Matrix xFunctionMatrix;
  private Matrix yFunctionMatrix;

  public CubicInterpolationFollower(double initialXPot, double initialYPot, double finalXPot, double finalYPot, double initialXVel, double initialYVel, double finalXVel, double finalYVel, double timeSpan) {
    xPI = initialXPot;
    yPI = initialYPot;
    xPF = finalXPot;
    yPF = finalYPot;
    xVI = initialXVel;
    yVI = initialYVel;
    xVF = finalXVel;
    yVF = finalYVel;
    deltaT = timeSpan;

    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {  
    initialTime = Timer.getFPGATimestamp();
    finalTime = initialTime + deltaT;
    

  }
  public void createPathFunction(double initialTime, double finalTime){
    xPositionAndVelocityMatrix = new Matrix(4, 1);
    xPositionAndVelocityMatrix.set(0, 0, xPF);
    xPositionAndVelocityMatrix.set(1, 0, xVF);
    xPositionAndVelocityMatrix.set(2, 0, xPI);
    xPositionAndVelocityMatrix.set(3, 0, xVI);
    yPositionAndVelocityMatrix = new Matrix(4, 1);
    yPositionAndVelocityMatrix.set(0, 0, yPF);
    yPositionAndVelocityMatrix.set(1, 0, yVF);
    yPositionAndVelocityMatrix.set(2, 0, yPI);
    yPositionAndVelocityMatrix.set(3, 0, yVI);
    timeFunctionMatrix = new Matrix(4, 4);
    timeFunctionMatrix.set(0, 0, 1);
    timeFunctionMatrix.set(1, 0, 0);
    timeFunctionMatrix.set(2, 0, 1);
    timeFunctionMatrix.set(3, 0, 0);
    timeFunctionMatrix.set(0, 1, finalTime);
    timeFunctionMatrix.set(1, 1, 1);
    timeFunctionMatrix.set(2, 1, initialTime);
    timeFunctionMatrix.set(3, 1, 1);
    timeFunctionMatrix.set(0, 2, Math.pow(finalTime, 2));
    timeFunctionMatrix.set(1, 2, 2*finalTime);
    timeFunctionMatrix.set(2, 2, Math.pow(initialTime,2));
    timeFunctionMatrix.set(3, 2, 2*initialTime);
    timeFunctionMatrix.set(0, 3, Math.pow(finalTime, 3));
    timeFunctionMatrix.set(1, 3, 3*Math.pow(finalTime, 2));
    timeFunctionMatrix.set(2, 3, Math.pow(initialTime,3));
    timeFunctionMatrix.set(3, 3, 3*Math.pow(initialTime, 2));
    xFunctionMatrix = timeFunctionMatrix.inverse().times(xPositionAndVelocityMatrix);
    yFunctionMatrix = timeFunctionMatrix.inverse().times(yPositionAndVelocityMatrix);
  }
  public Point getDesiredPosition(double time){
    double xPoint;
    double yPoint;
    if(time<initialTime){
      desiredPoint = new Point(xPI, yPI);
      return desiredPoint;
    }
    else if(time>finalTime){
      desiredPoint = new Point(xPF, yPF);
      return desiredPoint;
    }
    else{
      xPoint = xFunctionMatrix.get(0, 0) + xFunctionMatrix.get(1,0)*time + xFunctionMatrix.get(2,0)*Math.pow(time, 2) + xFunctionMatrix.get(3,0)*Math.pow(time, 3);
      yPoint = yFunctionMatrix.get(0, 0) + yFunctionMatrix.get(1,0)*time + yFunctionMatrix.get(2,0)*Math.pow(time, 2) + yFunctionMatrix.get(3,0)*Math.pow(time, 3);
      desiredPoint = new Point(xPoint,yPoint);
      return desiredPoint;
    }
  }
  public Vector getDesiredVelocity(double time){
    double xVelocity;
    double yVelocity;
    if(time<initialTime){
      desiredVelocity = new Vector(xVI, yVI);
      return desiredVelocity;
    }
    else if(time>finalTime){
      desiredVelocity = new Vector(xVF, yVF);
      return desiredVelocity;
    }
    else{
      xVelocity = xFunctionMatrix.get(1,0)+ 2*xFunctionMatrix.get(2,0)*time + 3*xFunctionMatrix.get(3,0)*Math.pow(time, 2);
      yVelocity =  yFunctionMatrix.get(1,0) + 2*yFunctionMatrix.get(2,0)*time + 3*yFunctionMatrix.get(3,0)*Math.pow(time, 2);
      desiredVelocity = new Vector(xVelocity,yVelocity);
      return desiredVelocity;
    }
  }
  private void findRobotCurvature(){
		double a = -Math.tan(Math.toRadians(RobotMap.drive.getDriveTrainHeading()));
		double b = 1;
		double c = Math.tan(Math.toRadians(RobotMap.drive.getDriveTrainHeading())) * RobotMap.drive.getDriveTrainX() - RobotMap.drive.getDriveTrainY();
		double x = Math.abs( a * getDesiredPosition(Timer.getFPGATimestamp()+0.1).getXPos()+ b * getDesiredPosition(Timer.getFPGATimestamp()+0.1).getYPos() + c) /Math.sqrt(Math.pow(a, 2)+Math.pow(b, 2));
		double side = Math.signum(Math.sin(Math.toRadians(RobotMap.drive.getDriveTrainHeading()) * (getDesiredPosition(Timer.getFPGATimestamp()+0.1).getXPos()-RobotMap.drive.getDriveTrainX()-Math.cos(Math.toRadians(odometry.gettheta()))*(lookAheadPoint.getYPos()-odometry.getY())); 
		double curvature = ((2*x)/Math.pow(lookAheadDistance,2))*side;
		desiredRobotCurvature = curvature;
    }
  private void setWheelVelocities(double targetVelocity, double curvature){
      double leftVelocity;
      double rightVelocity;
      double v;
      else{
        v = targetVelocity;
      }
      
      double c = curvature;
      if(chosenPath.getReversed()){
        v = -v;
        c = -c;
      }
      leftVelocity = v*(2+(c*RobotStats.robotBaseDistance))/2;
      rightVelocity = v*(2-(c*RobotStats.robotBaseDistance))/2;

      RobotMap.drive.setLeftSpeed(leftVelocity);
      RobotMap.drive.setRightSpeed(rightVelocity);
      
    }
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    findRobotCurvature();
		curveAdjustedVelocity = Math.min(Math.abs(k/desiredRobotCurvature),chosenPath.getMainPath().get(closestSegment).velocity);
		setWheelVelocities(curveAdjustedVelocity, desiredRobotCurvature);
		endThetaError = Pathfinder.boundHalfDegrees((Math.toDegrees(chosenPath.getMainPath().get(chosenPath.getMainPath().length()-1).heading)-odometry.gettheta()));
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
