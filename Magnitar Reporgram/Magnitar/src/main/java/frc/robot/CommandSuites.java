/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.humanInterface.ArcadeDrive;

/**
 * Add your docs here.
 */
public class CommandSuites {
    public ArcadeDrive arcadeDrive;
    public CommandSuites(){
        arcadeDrive = new ArcadeDrive();
    }
    public void startAutoCommands(){

    }
    public void endAutoCommands(){

    }
    public void startTeleopCommands(){
        arcadeDrive.start();
    }
    public void endTeleopCommands(){

    }
}
