/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.autos.testAuto;
import frc.robot.commands.humanInterface.ArmInterface;
import frc.robot.commands.humanInterface.DriveInterface;
import frc.robot.tools.controlLoops.CubicInterpolationFollower;
import frc.robot.tools.controlLoops.VelocityPID;

/**
 * Add your docs here.
 */
public class CommandSuites {
    public DriveInterface driveInterface;
    public ArmInterface armInterface;
    public testAuto basicTestAuto;
    private CubicInterpolationFollower cubicFollower;
    public CommandSuites(){
        armInterface = new ArmInterface();
        driveInterface = new DriveInterface();
    }
    public void startAutoCommands(){

        basicTestAuto = new testAuto();
        basicTestAuto.start();
    }
    public void endAutoCommands(){

    }
    public void startTeleopCommands(){
        armInterface.start();
        driveInterface.start();
    }
    public void endTeleopCommands(){

    }
}
