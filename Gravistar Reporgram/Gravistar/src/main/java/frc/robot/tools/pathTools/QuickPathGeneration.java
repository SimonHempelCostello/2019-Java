/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.tools.pathTools;

import frc.robot.RobotConfig;
import jaci.pathfinder.Waypoint;

/**
 * Add your docs here.
 */
public class QuickPathGeneration {
  private double xpos;
  private double ypos; 
  private double heading;
  private PathSetup returnPath;
  private boolean isReversed;
  private Waypoint[] returnPathPoints;
  //for small quick paths with short distances. Useful for variable paths
  public QuickPathGeneration(double xdist, double ydist, double targetAngle, boolean reverse){
    xpos = xdist;
    ypos = ydist;
    heading = targetAngle;
    isReversed = reverse;
  }
  public PathSetup GeneratePath(){
    returnPathPoints = new Waypoint[] {
      new Waypoint(xpos, ypos, 0),
      new Waypoint(0,0, heading), 
    };

    returnPath = new PathSetup(returnPathPoints, 4, isReversed);
    returnPath.generateMainPath();
    
    return returnPath;

  }
}
