/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.controls.ChangeLightColor;
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
	private UsbCamera camera;
	private UsbCamera camera2;
	private VideoSink server;
	public static boolean hasCamera = false;
	private boolean cameraBoolean = false;
	public static ChangeLightColor changeLightColor = new ChangeLightColor(1,0, 0, RobotMap.canifier1);
	public static ChangeLightColor changeLightColor1 = new ChangeLightColor(0,0, 0, RobotMap.canifier2);
	public static VisionCamera visionCamera;
	public static SerialPort jevois1;
	private double byteCount;
	private boolean ableToSwitch;
	private int runCounter = 0;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    commandSuites = new CommandSuites();
    robotConfig = new RobotConfig();
    RobotMap.drive.startAutoOdometry();
    robotConfig.setStartingConfig();
    //RobotMap.drive.initVelocityPIDs();
    RobotMap.drive.initAlignmentPID();
    try {
			jevois1 = new SerialPort(115200, Port.kUSB);
			if(jevois1.getBytesReceived()>2){
				hasCamera = true;
			}
			else{
				hasCamera = false;
			}
		} catch (Exception e) {
			hasCamera = false;
		}
		visionCamera= new VisionCamera(Robot.jevois1);
		ableToSwitch = true;
		
		robotConfig.setStartingConfig();
		camera = CameraServer.getInstance().startAutomaticCapture("VisionCamera1", "/dev/video0");
		camera.setResolution(320, 240);
		camera.setFPS(15);

		camera2 = CameraServer.getInstance().startAutomaticCapture("VisionCamera2", "/dev/video1");
		camera2.setResolution(320, 240);
		camera2.setFPS(15);
		RobotMap.visionRelay1.set(Value.kOn);
	

		server = CameraServer.getInstance().addSwitchedCamera("driverVisionCameras");
		server.setSource(camera);
		Shuffleboard.update();
		SmartDashboard.updateValues(); 
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
    runCounter++;
		if(runCounter%10==0){
			visionCamera.updateVision();
			SmartDashboard.putNumber("visionAngle", visionCamera.getAngle());
		}
		if(runCounter%100==0){
			double pressure = ((250*RobotMap.preassureSensor.getAverageVoltage())/4.53)-25;
      SmartDashboard.putNumber("pressure", pressure);
      SmartDashboard.putString("visionString", visionCamera.getString());
      SmartDashboard.putNumber("ultraSonic2", RobotMap.mainUltrasonicSensor2.getDistance());
			SmartDashboard.putBoolean("hasNavx", RobotMap.navx.isConnected());
			SmartDashboard.putNumber("getX",RobotMap.drive.getDriveTrainX());
			SmartDashboard.putNumber("getY",RobotMap.drive.getDriveTrainY());
			SmartDashboard.putBoolean("hasCamera", hasCamera);
			SmartDashboard.putNumber("armPosit", RobotMap.arm.getArmAngle());
			if(byteCount>0){
				hasCamera = true;
				changeLightColor1.changeLedColor(0, 1,0);
			}
			else{
				changeLightColor1.changeLedColor(0, 0,1);
				hasCamera = true;
			}
			byteCount = 0;
		}
		try{
			byteCount = byteCount + jevois1.getBytesReceived();
		}
		catch(Exception e){
			hasCamera = false;
		}
		if(ButtonMap.switchCamera()&&ableToSwitch){
			if(cameraBoolean){
				server.setSource(camera2);
				cameraBoolean = false;
			}
			else if(!cameraBoolean){
				server.setSource(camera);
				cameraBoolean = true;
			}
			ableToSwitch = false;
		}
		else if(!ButtonMap.switchCamera()){
			ableToSwitch = true;
		}
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
    RobotMap.drive.startAutoOdometry();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }
  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
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
