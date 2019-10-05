/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.VisionCamera;
import frc.robot.tools.pathTools.Odometry;
import frc.robot.tools.pathTools.PathList;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI m_oi;
  Command m_autonomousCommand;
  public static PathList pathlist = new PathList();
  private CommandSuites commandSuites;
  private RobotConfig robotConfig;
  public static VisionCamera visionCamera;
  public static SerialPort serialPort1 = new SerialPort(115200, Port.kUSB);

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    visionCamera = new VisionCamera(serialPort1);
    commandSuites = new CommandSuites();
    robotConfig = new RobotConfig();
    robotConfig.setStartingConfig();
    RobotMap.drive.initVelocityPIDs();
    RobotMap.drive.initAlignmentPID();
    m_oi = new OI();
  
  }
  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    if(RobotMap.armMaster.getSensorCollection().isFwdLimitSwitchClosed()){
      SmartDashboard.putBoolean("fwdLimitSwitch", true);
      SmartDashboard.putBoolean("revLimitSwitch", false);
    }
    else if(RobotMap.armMaster.getSensorCollection().isRevLimitSwitchClosed()){
      SmartDashboard.putBoolean("fwdLimitSwitch", false);
      SmartDashboard.putBoolean("revLimitSwitch", true);
    }
    else{
      SmartDashboard.putBoolean("fwdLimitSwitch", false);
      SmartDashboard.putBoolean("revLimitSwitch", false);
    }

    SmartDashboard.putString("CameraString", visionCamera.getString());
    //SmartDashboard.putNumber("armSpinnyBoy",RobotMap.arm.mainArmEncoder.getRawPosition());
    //SmartDashboard.putNumber("armSpinnyBoy1",RobotMap.arm.mainArmEncoder.getAngle());
  }
  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }
  @Override
  public void disabledPeriodic() {

    Scheduler.getInstance().run();
  }
  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    commandSuites.startAutoCommands();
    RobotMap.drive.startAutoOdometry(0,2,0);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }
  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("Robotx", RobotMap.drive.getDriveTrainX());
    SmartDashboard.putNumber("Roboty", RobotMap.drive.getDriveTrainY());
    Scheduler.getInstance().run();
  }
  @Override
  public void teleopInit() {
    commandSuites.startTeleopCommands();


    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    

    Scheduler.getInstance().run();
  }
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
