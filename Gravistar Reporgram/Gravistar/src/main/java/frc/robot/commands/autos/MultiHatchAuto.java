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
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.commands.controls.DeadReackonDrive;
import frc.robot.commands.controls.SetRobotOdometryPostion;
import frc.robot.tools.controlLoops.CascadingPIDTurn;
import frc.robot.tools.controlLoops.PurePursuitController;
public class MultiHatchAuto extends CommandGroup {

  public MultiHatchAuto() {
    addSequential(new PurePursuitController(Robot.pathlist.multiHatchAutoPath1, 0.8, 4.8, true, true));
    addSequential(new CascadingPIDTurn(120, 0.7, 0.02, 0));
    addSequential(new WaitCommand(0.25));
    addSequential(new PurePursuitController(Robot.pathlist.multiHatchAutoPath2, 1.2, 4.8, true, true));
    addSequential(new PurePursuitController(Robot.pathlist.multiHatchAutoPath3, 1.2, 4.0, true, true));
    addSequential(new WaitCommand(0.25));
    addSequential(new CascadingPIDTurn(0, 0.7, 0.02, 0));
    addSequential(new CascadingPIDTurn(120, 0.7, 0.02, 0));
    addSequential(new PurePursuitController(Robot.pathlist.multiHatchAutoPath2, 1.2, 4.8, true, true));
  }
}
