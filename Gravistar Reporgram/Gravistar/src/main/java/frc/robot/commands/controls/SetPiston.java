/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.controls;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Add your docs here.
 */
public class SetPiston extends InstantCommand {
  /**
   * Add your docs here.
   */
  private DoubleSolenoid piston;
  private DoubleSolenoid.Value position;
  public SetPiston(DoubleSolenoid solenoid, DoubleSolenoid.Value value) {
    super();
    piston = solenoid;
    position = value;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    piston.set(position);
  }

}
