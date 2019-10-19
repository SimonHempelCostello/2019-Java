/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.tools.controlLoops;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.RobotStats;
import frc.robot.tools.pathTools.PathSetup;
import frc.robot.tools.pathTools.Odometry;
import frc.robot.tools.math.Point;
import frc.robot.tools.math.Vector;
import jaci.pathfinder.Pathfinder;

public class PurePursuitController extends Command {
	private PathSetup chosenPath;
	private Odometry odometry;
	private int closestSegment;
	private Point lookAheadPoint;
	private Point lastLookAheadPoint;
	private Notifier pathNotifier;
	private int startingNumber;
	private double deltaX;
	private double deltaY;
	private double distToPoint;
	private double minDistanceToPoint;
	private Point closestPoint;
	private double lookAheadDistance;
	private double desiredRobotCurvature;
	private Point startingPointOfLineSegment;
	private boolean firstLookAheadFound;
	private int startingNumberLA;
	private Vector lineSegVector;
	private Point endPointOfLineSegment;
	private Point robotPos;
	private Vector robotPosVector;
	private double lookAheadIndexT1;
	private double lookAheadIndexT2;
	private double partialPointIndex;
	private double lastPointIndex;
	private Vector distToEndVector;
	private double curveAdjustedVelocity;
	private double k;
	private boolean shouldRunAlgorithm;
	public double endThetaError;
	private boolean useOutsideOdometry;
	private boolean shouldEnd;
	private boolean odometryDirection;
  	//no carried over position information
  	public PurePursuitController(PathSetup path, double lookAhead, double kValue){
		chosenPath = path;
		lookAheadDistance = lookAhead;  
		k = kValue;  
		requires(RobotMap.drive);
		useOutsideOdometry = false;
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
  	}
  	//for carried over angle
  	public PurePursuitController(PathSetup path, double lookAhead, double kValue, boolean outsideOdometry, boolean robotAbsoluteDirection){
		chosenPath = path;
		lookAheadDistance = lookAhead;  
		k = kValue;  
		useOutsideOdometry = outsideOdometry;
		//robot absolute direction describes the direction of robot movemnt for which x increases positively,
		//if robotAbsoluteDirection is true, assuming a theta of 0, negative drivetrain values on both side will lead to x increasing
		//if robotAbsoluteDirection is false, assuming a theta of 0, positive drivetrain values on both sides will lead to x increasing,
		//this way the robot can run any path either forward or reversed, as long as robotAbsoluteDirection remains constant.
		odometryDirection = robotAbsoluteDirection;
		requires(RobotMap.drive);

		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
 	}


  	// Called just before this Command runs the first time
  	@Override
  	protected void initialize() {
		shouldEnd = false;
		odometry = new Odometry(odometryDirection);
		RobotMap.drive.setOdometryReversed(odometryDirection);

		odometry.zero();
		odometry.start();
		if(useOutsideOdometry){
			odometry.setX(RobotMap.drive.getDriveTrainX());
			odometry.setY(RobotMap.drive.getDriveTrainY());
			odometry.setTheta(RobotMap.drive.getDriveTrainHeading());
		}
		else{
			odometry.setX(chosenPath.getMainPath().get(0).x);
			odometry.setY(chosenPath.getMainPath().get(0).y);
			odometry.setTheta(chosenPath.getMainPath().get(0).heading);
		}
		
		distToEndVector = new Vector(12,12);
		lookAheadPoint = new Point(0, 0);
		closestPoint = new Point(0,0);
		robotPos = new Point(0,0);
		endPointOfLineSegment = new Point(0,0);
		startingPointOfLineSegment = new Point(0,0);
		lineSegVector = new Vector(0,0);
		robotPosVector = new Vector(0,0);
		lastLookAheadPoint = new Point(0,0);
		closestSegment = 0;
		minDistanceToPoint = 10000;
		startingNumber = 0;
		startingNumberLA  = 0;
		lastPointIndex = 0;
		partialPointIndex = 0;
		lookAheadIndexT1 = 0;
		lookAheadIndexT2 = 0;
		desiredRobotCurvature = 0;
		minDistanceToPoint = 0;
		distToPoint = 0;
		deltaX = 0;
		deltaY = 0;
		shouldRunAlgorithm = true;
		curveAdjustedVelocity = 0;
		pathNotifier = new Notifier(new PathRunnable());
		pathNotifier.startPeriodic(0.001);
    }
 	private class PathRunnable implements Runnable{
		public void run(){
			if(shouldRunAlgorithm){
			purePursuitAlgorithm();
			}
			else{
			pathNotifier.stop();
			}
		}
  	}
  	private void purePursuitAlgorithm(){
		odometry.setX(RobotMap.drive.getDriveTrainX());

		odometry.setY(RobotMap.drive.getDriveTrainY());
		for(int i = startingNumber; i<chosenPath.getMainPath().length()-1;i++){        
			deltaX = chosenPath.getMainPath().get(i).x-odometry.getX();
			deltaY = chosenPath.getMainPath().get(i).y-odometry.getY();
			distToPoint = Math.sqrt(Math.pow(deltaX, 2)+Math.pow(deltaY, 2));
			if(distToPoint<minDistanceToPoint){
				minDistanceToPoint = distToPoint;
				closestSegment = i;
				closestPoint.setLocation(chosenPath.getMainPath().get(i).x, chosenPath.getMainPath().get(i).y);
			}
		}
		if(minDistanceToPoint<lookAheadDistance){

		}
		startingNumber = closestSegment;
		minDistanceToPoint = 100;
		firstLookAheadFound = false;
		for(int i = startingNumberLA; i<chosenPath.getMainPath().length()-1;i++){
			startingPointOfLineSegment.setLocation(chosenPath.getMainPath().get(i).x, chosenPath.getMainPath().get(i).y);
			endPointOfLineSegment.setLocation(chosenPath.getMainPath().get(i+1).x, chosenPath.getMainPath().get(i+1).y); 
			robotPos.setLocation(odometry.getX(), odometry.getY());
			lineSegVector.setX(endPointOfLineSegment.getXPos()-startingPointOfLineSegment.getXPos());
			lineSegVector.setY(endPointOfLineSegment.getYPos()-startingPointOfLineSegment.getYPos());
			robotPosVector.setX(startingPointOfLineSegment.getXPos()-robotPos.getXPos());
			robotPosVector.setY(startingPointOfLineSegment.getYPos()-robotPos.getYPos());
			double a = lineSegVector.dot(lineSegVector);
			double b = 2*robotPosVector.dot(lineSegVector);
			double c = robotPosVector.dot(robotPosVector)-lookAheadDistance*lookAheadDistance;
			double discriminant = b*b - 4*a*c;
			if(discriminant<0){
				lookAheadPoint.setLocation(lastLookAheadPoint.getXPos(), lastLookAheadPoint.getYPos()); 
			}
			else{
				discriminant = Math.sqrt(discriminant);
				lookAheadIndexT1 = (-b-discriminant)/(2*a);
				lookAheadIndexT2 = (-b+discriminant)/(2*a);
				if(lookAheadIndexT1>=0&&lookAheadIndexT1<=1){
					partialPointIndex = i+lookAheadIndexT1;
					if(partialPointIndex>lastPointIndex){
					lookAheadPoint.setLocation(startingPointOfLineSegment.getXPos()+ lookAheadIndexT1*lineSegVector.getxVec() , startingPointOfLineSegment.getYPos() + lookAheadIndexT1*lineSegVector.getyVec());
					firstLookAheadFound = true;
					}
				}
				
				else if(lookAheadIndexT2>=0&&lookAheadIndexT2<=1){
					partialPointIndex = i+lookAheadIndexT2;
					if(partialPointIndex>lastPointIndex){
					lookAheadPoint.setLocation(startingPointOfLineSegment.getXPos() + lookAheadIndexT2*lineSegVector.getxVec() , startingPointOfLineSegment.getYPos() + lookAheadIndexT2*lineSegVector.getyVec());
					firstLookAheadFound = true;

					}
				}
			}
			if(firstLookAheadFound){
				i = chosenPath.getMainPath().length();
			}
			else if(!firstLookAheadFound && i==chosenPath.getMainPath().length()-1){
				lookAheadPoint.setLocation(lastLookAheadPoint.getXPos(), lastLookAheadPoint.getYPos());
			}
		}
		lastLookAheadPoint.setLocation(lookAheadPoint.getXPos(), lookAheadPoint.getYPos());
		if(partialPointIndex>lastPointIndex){
			lastPointIndex = partialPointIndex;
		}
		distToEndVector.setX(chosenPath.getMainPath().get(chosenPath.getMainPath().length()-1).x-odometry.getX());
		distToEndVector.setY(chosenPath.getMainPath().get(chosenPath.getMainPath().length()-1).y-odometry.getY());
		SmartDashboard.putNumber("distoend", distToEndVector.length());
		SmartDashboard.putNumber("x", odometry.getX());
		SmartDashboard.putNumber("closestSegment", chosenPath.getMainPath().length()-closestSegment);
		SmartDashboard.putNumber("y",odometry.getY());
		SmartDashboard.putNumber("theta", odometry.gettheta());
		startingNumberLA = (int)partialPointIndex;
		lastLookAheadPoint = lookAheadPoint;
		findRobotCurvature();
		curveAdjustedVelocity = Math.min(Math.abs(k/desiredRobotCurvature),chosenPath.getMainPath().get(closestSegment).velocity);
		setWheelVelocities(curveAdjustedVelocity, desiredRobotCurvature);
		endThetaError = Pathfinder.boundHalfDegrees((Math.toDegrees(chosenPath.getMainPath().get(chosenPath.getMainPath().length()-1).heading)-odometry.gettheta()));
  	} 
  	public void setOdometryX(double X){
		double desiredX= X;
		odometry.setX(desiredX);
  	}
  	public void setOdometryY(double Y){
		double desiredY= Y;
		odometry.setY(desiredY);
  	}
  	public void setOdometryTheta(double Theta){
		double desiredTheta= Theta;
		odometry.setTheta(desiredTheta);
  	}
  	public double getX(){
		return odometry.getX();
 	}
  	public double getY(){
		return odometry.getY();
  	}
  	public double getTheta(){
		return odometry.gettheta();
  	}
  	private void setWheelVelocities(double targetVelocity, double curvature){
		double leftVelocity;
		double rightVelocity;
		double v;
		if(closestSegment <10){
			v = 1.2;
		}
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
		SmartDashboard.putNumber("left", leftVelocity);
		SmartDashboard.putNumber("right", rightVelocity);

		if(chosenPath.getReversed()){
			RobotMap.drive.setLeftSpeed(rightVelocity);
			RobotMap.drive.setRightSpeed(leftVelocity);

		}
		else{
			RobotMap.drive.setLeftSpeed(leftVelocity);
			RobotMap.drive.setRightSpeed(rightVelocity);
		}
	}
	private void findRobotCurvature(){
		double a = -Math.tan(Math.toRadians(odometry.gettheta()));
		double b = 1;
		double c = Math.tan(Math.toRadians(odometry.gettheta())) * odometry.getX() - odometry.getY();
		double x = Math.abs( a * lookAheadPoint.getXPos()+ b * lookAheadPoint.getYPos() + c) /Math.sqrt(Math.pow(a, 2)+Math.pow(b, 2));
		double side = Math.signum(Math.sin(Math.toRadians(odometry.gettheta())) * (lookAheadPoint.getXPos()-odometry.getX())-Math.cos(Math.toRadians(odometry.gettheta()))*(lookAheadPoint.getYPos()-odometry.getY())); 
		double curvature = ((2*x)/Math.pow(lookAheadDistance,2))*side;
		desiredRobotCurvature = curvature;
    }

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
	}
	public void forceFinish(){
		shouldEnd = true;
	}
	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		if(chosenPath.getMainPath().length()-closestSegment<=2){
			return true;
		} 
		else{
			return shouldEnd;
		}   
		
	}
	// Called once after isFinished returns true
	@Override
	protected void end() {
		System.out.println(RobotMap.drive.getDriveTrainX());
		pathNotifier.stop();
		shouldRunAlgorithm = false;
		odometry.endOdmetry();
		RobotMap.drive.setLeftSpeed(0);
		RobotMap.drive.setRightSpeed(0);
		odometry.cancel();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		this.end();
	}
}
