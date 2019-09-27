/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.sensors.ArmEncoder;
import frc.robot.sensors.DriveEncoder;
import frc.robot.sensors.PWMUltraSonicSensor;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  public static AHRS navx = new AHRS(Port.kMXP);

  public static DoubleSolenoid shifters = new DoubleSolenoid(0, 1);
	public static DoubleSolenoid.Value lowGear = DoubleSolenoid.Value.kReverse;//TODO directions must be assigned
  public static DoubleSolenoid.Value highGear = DoubleSolenoid.Value.kForward;//TODO directions mut be assinged

  public static DoubleSolenoid hatchPushOutPiston = new DoubleSolenoid(2,3);
  public static DoubleSolenoid.Value hatchMechOut = DoubleSolenoid.Value.kForward;
  public static DoubleSolenoid.Value hatchMechIn = DoubleSolenoid.Value.kReverse;
  
	public static DoubleSolenoid hatchGrabberPiston = new DoubleSolenoid(4,5);
	public static DoubleSolenoid.Value hatchMechRelease = DoubleSolenoid.Value.kForward;
  public static DoubleSolenoid.Value hatchMechGrab = DoubleSolenoid.Value.kReverse;
  
  public static PowerDistributionPanel pdp = new PowerDistributionPanel();
  
	public static Relay visionRelay1 = new Relay(0);

  
	public static int rightMasterTalonID = 3;
  public static int leftMasterTalonID = 1;

  public static int rightFollowerTalonID = 4;
  public static int leftFollowerTalonID = 2;

  public static int armMasterID = 5;
  public static int armFollowerID = 6;
  
  public static int intakeMotorID = 7;
  
  public static int climbingMechLeadTalonID = 8;
	public static int climbingMechFollowerTalonID = 9;
  
  public static TalonSRX leftDriveLead = new TalonSRX(leftMasterTalonID); // blue encoder
  public static TalonSRX rightDriveLead = new TalonSRX(rightMasterTalonID); // red

	public static TalonSRX leftDriveFollowerOne = new TalonSRX(leftFollowerTalonID); // yellow
  public static TalonSRX rightDriveFollowerOne = new TalonSRX(rightFollowerTalonID);// green encoder
  
  public static TalonSRX armMaster = new TalonSRX(armMasterID);
  public static TalonSRX armFollower = new TalonSRX(armFollowerID);

  public static TalonSRX intakeMotor = new TalonSRX(intakeMotorID);
  
  public static TalonSRX climbingMechLeadTalon = new TalonSRX(climbingMechLeadTalonID);
	public static TalonSRX climbingMechFollowerTalon = new TalonSRX(climbingMechFollowerTalonID);
  
  public static Relay.Value lightRingOn = Relay.Value.kForward;
  public static Relay.Value lightRingOff = Relay.Value.kReverse;

  public static AnalogInput preassureSensor = new AnalogInput(1);
  
	public static Counter ultraSonic1 = new Counter(0);
	public static Counter ultraSonic2 = new Counter(1);
	public static Counter ultraSonic3 = new Counter(2);
  public static Counter ultraSonic4 = new Counter(3);

  public static PWMUltraSonicSensor mainUltrasonicSensor1=new PWMUltraSonicSensor(ultraSonic1);
	public static PWMUltraSonicSensor mainUltrasonicSensor2= new PWMUltraSonicSensor(ultraSonic2);
	public static PWMUltraSonicSensor mainUltrasonicSensor3=new PWMUltraSonicSensor(ultraSonic3);
	public static PWMUltraSonicSensor mainUltrasonicSensor4= new PWMUltraSonicSensor(ultraSonic4);
  
  public static CANifier canifier1 = new CANifier(0);
  public static CANifier canifier2 = new CANifier(1);
  
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
    RobotMap.armMaster
  };
  public static TalonSRX allMotors[] = {
    RobotMap.leftDriveLead,
    RobotMap.rightDriveLead,
    RobotMap.leftDriveFollowerOne,
    RobotMap.rightDriveFollowerOne,
    RobotMap.armMaster,
    RobotMap.armFollower,
    RobotMap.intakeMotor
  };

  public static DriveTrain drive = new DriveTrain();
  public static Arm arm = new Arm();
  
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
