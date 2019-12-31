/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.tools.controlLoops.PurePursuitController;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class DemoAuto extends CommandGroup {
  /**
   * Add your docs here.
   */
  public DemoAuto() {
    addSequential(new PurePursuitController(Robot.pathlist.demoAuto1,1.2,3.8,true,false));
    addSequential(new PurePursuitController(Robot.pathlist.demoAuto2,1.2,3.8,true,false));
    addSequential(new PurePursuitController(Robot.pathlist.demoAuto3,2,3.8,true,false));


  }
}
