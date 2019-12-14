/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.controls.DeadReackonDrive;
import frc.robot.commands.controls.SetRobotOdometryPostion;
import frc.robot.tools.controlLoops.CascadingPIDTurn;
import frc.robot.tools.controlLoops.PurePursuitController;

public class testAuto extends CommandGroup {

  public testAuto() {
    addSequential(new PurePursuitController(Robot.pathlist.test1Path, 1.5, 4.8, true, true));
    addSequential(new CascadingPIDTurn(120, 0.4, 0,0));
    addSequential(new SetRobotOdometryPostion(11,0, 120, true));
    addSequential(new PurePursuitController(Robot.pathlist.test2Path, 1.5, 4.8, true, true));
  }
}
