/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.tools.controlLoops;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.RobotStats;
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
  private double desiredRobotCurvature;
  private Point midPoint; 
  private Point lookAheadPoint;
  private double pathDistance;
  private double velocity;
  private Vector distToMidPoint;
  private Vector distToEndPoint;
  private double lookAheadDistance;
  private boolean shouldRunAlgorithm;
  private Vector traveledDistanceVector;
  private double lastTestValue;
  private boolean isReversed;

  public CubicInterpolationFollower(double initialXPot, double initialYPot, double finalXPot, double finalYPot, double initialXVel, double initialYVel, double finalXVel, double finalYVel, double timeSpan, double laDistance, boolean reversePath) {
    xPI = initialXPot;
    yPI = initialYPot;
    xPF = finalXPot;
    yPF = finalYPot;
    xVI = initialXVel;
    yVI = initialYVel;
    xVF = finalXVel;
    yVF = finalYVel;
    deltaT = timeSpan;
    lookAheadDistance = laDistance;
    isReversed = reversePath;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {  
    lastTestValue = 1;
    shouldRunAlgorithm = false;
    initialTime = Timer.getFPGATimestamp();
    finalTime = initialTime + deltaT;
    createPathFunction(0, deltaT,xPI, yPI, xPF, yPF);
    midPoint = new Point((xPF+xPI)/2, (yPF+yPI)/2);
    distToEndPoint = new Vector(xPF-RobotMap.drive.getDriveTrainX(), yPF - RobotMap.drive.getDriveTrainY());
    distToMidPoint = new Vector(midPoint.getXPos()-RobotMap.drive.getDriveTrainX(), midPoint.getYPos()-RobotMap.drive.getDriveTrainY());
    pathDistance = Math.sqrt(Math.pow(xPF-xPI,2)+Math.pow(yPF-yPI,2));
    traveledDistanceVector = new Vector(0,0);
    lookAheadPoint = new Point(0,0);
    shouldRunAlgorithm = false;
    System.out.println( xFunctionMatrix.get(0, 0) +" + " + xFunctionMatrix.get(1,0)+"t + " + xFunctionMatrix.get(2,0)+"t^2 + " + xFunctionMatrix.get(3,0)+"t^3");
    System.out.println( yFunctionMatrix.get(0, 0) +" + " + yFunctionMatrix.get(1,0)+"t + " + yFunctionMatrix.get(2,0)+"t^2 + " + yFunctionMatrix.get(3,0)+"t^3");
  }
  public void createPathFunction(double initialTime, double finalTime, double initialXPot, double initialYPot, double finalXPot, double finalYPot){
    xPositionAndVelocityMatrix = new Matrix(4, 1);
    xPositionAndVelocityMatrix.set(0, 0, finalXPot);
    xPositionAndVelocityMatrix.set(1, 0, xVF);
    xPositionAndVelocityMatrix.set(2, 0, initialXPot);
    xPositionAndVelocityMatrix.set(3, 0, xVI);
    yPositionAndVelocityMatrix = new Matrix(4, 1);
    yPositionAndVelocityMatrix.set(0, 0, finalYPot);
    yPositionAndVelocityMatrix.set(1, 0, yVF);
    yPositionAndVelocityMatrix.set(2, 0, initialYPot);
    yPositionAndVelocityMatrix.set(3, 0, yVI);
    timeFunctionMatrix = new Matrix(4, 4);
    timeFunctionMatrix.set(0, 0, 1);
    timeFunctionMatrix.set(1, 0, 0);
    timeFunctionMatrix.set(2, 0, 1);
    timeFunctionMatrix.set(3, 0, 0);
    timeFunctionMatrix.set(0, 1, deltaT);
    timeFunctionMatrix.set(1, 1, 1);
    timeFunctionMatrix.set(2, 1, 0);
    timeFunctionMatrix.set(3, 1, 1);
    timeFunctionMatrix.set(0, 2, Math.pow(deltaT, 2));
    timeFunctionMatrix.set(1, 2, 2*finalTime);
    timeFunctionMatrix.set(2, 2, Math.pow(0,2));
    timeFunctionMatrix.set(3, 2, 2*0);
    timeFunctionMatrix.set(0, 3, Math.pow(deltaT, 3));
    timeFunctionMatrix.set(1, 3, 3*Math.pow(deltaT, 2));
    timeFunctionMatrix.set(2, 3, Math.pow(0,3));
    timeFunctionMatrix.set(3, 3, 3*Math.pow(0, 2));
    xFunctionMatrix = timeFunctionMatrix.inverse().times(xPositionAndVelocityMatrix);
    yFunctionMatrix = timeFunctionMatrix.inverse().times(yPositionAndVelocityMatrix);
  }
  private Point getDesiredPosition(double time){
    double xPoint;
    double yPoint;
    if(time<0){
      desiredPoint = new Point(xPI, yPI);
      return desiredPoint;
    }
    else if(time>deltaT){
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
  private Vector getDesiredVelocity(double time){
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
  private double getdesiredXPosition(double time){
    return xFunctionMatrix.get(0, 0) + xFunctionMatrix.get(1,0)*time + xFunctionMatrix.get(2,0)*Math.pow(time, 2) + xFunctionMatrix.get(3,0)*Math.pow(time, 3);
  }
  private double getdesiredYPosition(double time){
    return yFunctionMatrix.get(0, 0) + yFunctionMatrix.get(1,0)*time + yFunctionMatrix.get(2,0)*Math.pow(time, 2) + yFunctionMatrix.get(3,0)*Math.pow(time, 3);
  }
  private double getdeisredXVelocity(double time){
      return  xFunctionMatrix.get(1,0)+ 2*xFunctionMatrix.get(2,0)*time + 3*xFunctionMatrix.get(3,0)*Math.pow(time, 2);
  }
  private double getdesiredYVelocity(double time){
      return  yFunctionMatrix.get(1,0) + 2*yFunctionMatrix.get(2,0)*time + 3*yFunctionMatrix.get(3,0)*Math.pow(time, 2);
  }
	private void findRobotCurvature(){
		double a = -Math.tan(Math.toRadians(RobotMap.drive.getDriveTrainHeading()));
		double b = 1;
		double c = Math.tan(Math.toRadians(RobotMap.drive.getDriveTrainHeading())) * RobotMap.drive.getDriveTrainX() - RobotMap.drive.getDriveTrainY();
		double x = Math.abs( a * lookAheadPoint.getXPos()+ b * lookAheadPoint.getYPos() + c) /Math.sqrt(Math.pow(a, 2)+Math.pow(b, 2));
		double side = Math.signum(Math.sin(Math.toRadians(RobotMap.drive.getDriveTrainHeading())) * (lookAheadPoint.getXPos()-RobotMap.drive.getDriveTrainX())-Math.cos(Math.toRadians(RobotMap.drive.getDriveTrainHeading()))*(lookAheadPoint.getYPos()-RobotMap.drive.getDriveTrainY())); 
		double curvature = ((2*x)/Math.pow(lookAheadDistance,2))*side;
		desiredRobotCurvature = curvature;
  }
  private double findLookAheadPoint(){
    double testValue = lastTestValue;
    double numerator;
    double denominator;
    for(int i = 0; i<8; i++){
      numerator = (Math.pow(RobotMap.drive.getDriveTrainX()-getdesiredXPosition(testValue),2)+Math.pow(RobotMap.drive.getDriveTrainY()-getdesiredYPosition(testValue),2)-Math.pow(lookAheadDistance, 2));
      denominator = (2*(RobotMap.drive.getDriveTrainX()-getdesiredXPosition(testValue))*(-getdeisredXVelocity(testValue))+2*(RobotMap.drive.getDriveTrainY()-getdesiredYPosition(testValue))*(-getdesiredYVelocity(testValue)));
      testValue = testValue -(numerator/denominator);
    }
    if(testValue<0){
      testValue = 0;
    }
    if(testValue>deltaT){
      testValue = deltaT;
    }
    lastTestValue = testValue;
    return testValue;

  }
  private double getCurrentTime(){
    return Timer.getFPGATimestamp()-initialTime;
  }

  private void setWheelVelocities(double targetVelocity, double curvature){
      double leftVelocity;
      double rightVelocity;
      double v;
      double c; 
      if(isReversed){
        v = -targetVelocity;
        c = -curvature;
      }
      else{
        v = targetVelocity;
        c = curvature;
      }
      leftVelocity = v*(2-(c*RobotStats.robotBaseDistance))/2;
      rightVelocity = v*(2+(c*RobotStats.robotBaseDistance))/2;
      if(isReversed){
        RobotMap.drive.setLeftSpeed(leftVelocity);
        RobotMap.drive.setRightSpeed(rightVelocity);
      }
      else{
        RobotMap.drive.setLeftSpeed(rightVelocity);
        RobotMap.drive.setRightSpeed(leftVelocity);
      }
      
    }
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    distToMidPoint.setX(midPoint.getXPos()-RobotMap.drive.getDriveTrainX());
    distToMidPoint.setY(midPoint.getYPos()-RobotMap.drive.getDriveTrainY());
    traveledDistanceVector.setX(RobotMap.drive.getDriveTrainX()-xPI); 
    traveledDistanceVector.setY(RobotMap.drive.getDriveTrainY()-yPI);
    distToEndPoint.setX(xPF-RobotMap.drive.getDriveTrainX());
    distToEndPoint.setY(yPF - RobotMap.drive.getDriveTrainY());
    velocity = -1*(distToMidPoint.length()-pathDistance/2)*(distToMidPoint.length()+pathDistance/2);
    if(traveledDistanceVector.length()<0.5){
      velocity = 1.2;
    }
    else if(velocity>2.0){
      velocity = 2.0;
    }
    SmartDashboard.putNumber("distToEnd",distToEndPoint.length());
    lookAheadPoint = getDesiredPosition(findLookAheadPoint());
    findRobotCurvature();
    setWheelVelocities(velocity, desiredRobotCurvature);

  }
  public void forceEnd(){
    shouldRunAlgorithm = true;
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return distToEndPoint.length()<0.2||shouldRunAlgorithm;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    RobotMap.drive.setLeftPercent(0);
    RobotMap.drive.setRightPercent(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
