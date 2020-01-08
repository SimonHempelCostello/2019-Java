/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.tools.pathTools;

import java.io.File;

public class PathList {
  private File demoAutoFile2 = new File("/home/lvuser/deploy/demoAuto4.pf1.csv");
  private File demoAutoFile3 = new File("/home/lvuser/deploy/demoAuto3.pf1.csv");



  public PathSetup multiHatchAutoPath1;
  public PathSetup multiHatchAutoPath2;
  public PathSetup multiHatchAutoPath3;
  public PathSetup demoAuto1;
  public PathSetup demoAuto2;
  public PathSetup demoAuto3;



  public PathList() {
    demoAuto3 = new PathSetup(demoAutoFile3, false);
    demoAuto2 = new PathSetup(demoAutoFile2, true);





	}
    
  
  public void resetAllPaths(){
  }
}
 
