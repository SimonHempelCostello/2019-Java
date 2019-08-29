package frc.robot.tools.pathTools;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;


import frc.robot.RobotConfig;
import frc.robot.RobotStats;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class PathSetup {
	private double velocity;
	private DistanceFollower rightFollower;
	private DistanceFollower leftFollower;
	private boolean isReversed;
	private Trajectory mainPath;
	private Waypoint[] points;
	public PathSetup(Waypoint[] pathpoints, double pathspeed, boolean reverse){
		points = pathpoints;
		velocity = pathspeed;
		mainPath = generateMainPath();
		rightFollower = generateRightPathFollower();
		leftFollower = generateLeftPathFollower();
		isReversed = reverse;
	}
	public PathSetup(File file, boolean reverse) {
		try {
			mainPath = Pathfinder.readFromCSV(file);
		} catch (IOException e) {
		}
		rightFollower = generateRightPathFollower();
		leftFollower = generateLeftPathFollower();
		isReversed = reverse;
	}
	public Trajectory generateMainPath() {
		// all units are in feet, cause MURICA!, basically the path calculations are assuming 1/20th of a second between updates, and a max velcoity of v ft/sec, a max acceleration of a ft/sec, and a max jerk of 75 feet/sec^3
		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_FAST, 0.05,velocity, RobotStats.robotMaxAccertion, 75.0);
		Trajectory trajectory = Pathfinder.generate(points, config);
		return trajectory;
	}
	public DistanceFollower generateLeftPathFollower(){
		//this isn't measured wheel base, instead it is effective wheel base which is found by rotating the robot around x times 
		//then C=2Pir and solve for r or r=C/(2xPi) then double I believe
		TankModifier modifier = new TankModifier(mainPath).modify(RobotStats.robotBaseDistance);
		Trajectory left= modifier.getLeftTrajectory();
		DistanceFollower leftFollower = new DistanceFollower(left);
		// this section of code is to create an distance follower which is basically a fancier version of our PID class and then to 
		//modify that for Tank drive 
		
		return leftFollower;
	}
	public DistanceFollower generateRightPathFollower(){
		//check comments for generateLeftPathFollower() basically same thing
		TankModifier modifier = new TankModifier(mainPath).modify(RobotStats.robotBaseDistance);
		Trajectory right= modifier.getRightTrajectory();
		DistanceFollower rightfollower = new DistanceFollower(right);
		//this is a way to print out what pathfinder expects the robot to do and how that is supposed to happen
		return rightfollower;
	}
	public void pathdata(){
		  for(int i = 0; i< mainPath.length();i++){
			jaci.pathfinder.Trajectory.Segment seg = mainPath.get(i);
			System.out.println( "%f,%f,%f,%f,%f,%f,%f,%f\n"+ 
			seg.dt +" dt " + seg.x+" dx "+ seg.y+" dy "+ seg.position+" dpos "+ seg.velocity+" dvel "+
				seg.acceleration+" dacc "+ seg.jerk+" dj "+ seg.heading+" dhead ");

		}
		
	}
	public DistanceFollower getRightFollower(){
		return rightFollower;
	}
	public DistanceFollower getLeftFollower(){
		return leftFollower;
	}
	//this is to reset the path after running it once, it seems necessary but further bug testing is necessary
	public void resetPath(){
		mainPath = generateMainPath();
		leftFollower = generateLeftPathFollower();
		rightFollower = generateRightPathFollower();
	}
	public double getVelocity(){
		return velocity;
	}
	public boolean getReversed(){
		return isReversed;
	}
	public Trajectory getMainPath(){
		return mainPath;
	}
	public void generateCurvature(){

	}
	   
}


