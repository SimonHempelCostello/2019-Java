/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.subsystems.DriveTrain;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  public static DoubleSolenoid shifters = new DoubleSolenoid(0, 7);
  public static Value lowGear = DoubleSolenoid.Value.kForward;
  public static Value highGear = DoubleSolenoid.Value.kReverse;

  public static AHRS navx = new AHRS(Port.kMXP);
  
	public static DoubleSolenoid intake = new DoubleSolenoid(2, 5);
	public static Value intakeIn = DoubleSolenoid.Value.kForward;
  public static Value intakeOut = DoubleSolenoid.Value.kReverse;
  
	public static DoubleSolenoid catapultRelease = new DoubleSolenoid(3, 4);
	public static Value releaseClosed = DoubleSolenoid.Value.kForward;
  public static Value releaseOpen = DoubleSolenoid.Value.kReverse;
  
  public static DoubleSolenoid catapult = new DoubleSolenoid(1, 6);
	public static Value catapultResting = DoubleSolenoid.Value.kReverse;
	public static Value catapultSet = DoubleSolenoid.Value.kForward;

	public static AnalogInput pressureSensor = new AnalogInput(0);

	public static int leftMasterTalonID = 1;
	public static int leftFollowerTalonID = 2;
	public static int rightMasterTalonID = 3;
	public static int rightFollowerTalonID = 4;
	public static int intakeMotorID = 5;
	public static int pincherTalonID = 6;
	public static TalonSRX leftDriveLead = new TalonSRX(leftMasterTalonID); // blue encoder
	public static TalonSRX leftDriveFollowerOne = new TalonSRX(leftFollowerTalonID); // yellow
	public static TalonSRX rightDriveLead = new TalonSRX(rightMasterTalonID); // red
	public static TalonSRX rightDriveFollowerOne = new TalonSRX(rightFollowerTalonID);// green encoder
	public static TalonSRX intakeMotor = new TalonSRX(intakeMotorID);
  public static TalonSRX pincher = new TalonSRX(pincherTalonID);

  
  public static TalonSRX driveMotors[] = {
    RobotMap.leftDriveLead,
    RobotMap.rightDriveLead,
    RobotMap.leftDriveFollowerOne,
    RobotMap.rightDriveFollowerOne,
  };
  public static TalonSRX driveMotorLeads[] = {
    RobotMap.leftDriveLead,
    RobotMap.rightDriveLead,
  };
  public static TalonSRX allMotorLeads[] = {
    RobotMap.leftDriveLead,
    RobotMap.rightDriveLead,
    RobotMap.intakeMotor,
    RobotMap.pincher
  };
  public static TalonSRX allMotors[] = {
    RobotMap.leftDriveLead,
    RobotMap.rightDriveLead,
    RobotMap.leftDriveFollowerOne,
    RobotMap.rightDriveFollowerOne,
    RobotMap.intakeMotor,
    RobotMap.pincher
  };

  public static DriveTrain drive = new DriveTrain();

  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
