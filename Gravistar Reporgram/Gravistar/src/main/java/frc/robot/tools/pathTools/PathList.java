/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.tools.pathTools;

import java.io.File;

public class PathList {
  private File multiHatchAutoFile1 = new File("/home/lvuser/deploy/2HatchAuto1.pf1.csv");
  private File multiHatchAutoFile2 = new File("/home/lvuser/deploy/2HatchAuto2.pf1.csv");
  private File multiHatchAutoFile3 = new File("/home/lvuser/deploy/2HatchAuto3.pf1.csv");
  private File demoAutoFile1 = new File("/home/lvuser/deploy/demoAuto.pf1.csv");


  public PathSetup multiHatchAutoPath1;
  public PathSetup multiHatchAutoPath2;
  public PathSetup multiHatchAutoPath3;
  public PathSetup demoAuto1;


  public PathList() {
    multiHatchAutoPath1 = new PathSetup(multiHatchAutoFile1, true);
    multiHatchAutoPath2 = new PathSetup(multiHatchAutoFile2, true);
    multiHatchAutoPath3 = new PathSetup(multiHatchAutoFile3, false);
    demoAuto1 = new PathSetup(demoAutoFile1, false);



	}
    
  
  public void resetAllPaths(){
  }
}
 
