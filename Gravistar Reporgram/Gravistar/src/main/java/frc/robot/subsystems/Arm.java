/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.RobotStats;
import frc.robot.commands.controls.SwitchPiston;
import frc.robot.sensors.ArmEncoder;

/**
 * Add your docs here.
 */
public class Arm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private double kp = 2.7;
  private double ki = 0.03;
  private double kd = 30;
  private SwitchPiston tenseHatchGrabber;
  private SwitchPiston releaseHatchGrabber;
  private SwitchPiston pushHatchMechOut;
  private SwitchPiston pullHatchMechIn;
  private double kf = 1.274;
  private int acceleration = 1250;
  private int velocity = 1250;
  
  private double compensationFactor = 0.07;
  public static ArmEncoder mainArmEncoder = new ArmEncoder(RobotMap.armMaster);
  public Arm(){
    pushHatchMechOut = new SwitchPiston(RobotMap.hatchPushOutPiston, RobotMap.hatchMechOut);
    tenseHatchGrabber = new SwitchPiston(RobotMap.hatchGrabberPiston, RobotMap.hatchMechGrab);
    releaseHatchGrabber = new SwitchPiston(RobotMap.hatchGrabberPiston, RobotMap.hatchMechRelease);
    pullHatchMechIn = new SwitchPiston(RobotMap.hatchPushOutPiston, RobotMap.hatchMechIn);

  }
  @Override
  public void initDefaultCommand() {
  }
  public void initializeArmControl(){
    RobotMap.armMaster.configMotionCruiseVelocity(velocity);
    RobotMap.armMaster.configMotionAcceleration(acceleration);
    RobotMap.armMaster.config_kP(0, kp);
    RobotMap.armMaster.config_kI(0, ki);
    RobotMap.armMaster.config_kD(0, kd);
    RobotMap.armMaster.config_kF(0, kf);
    RobotMap.armMaster.selectProfileSlot(0, 0);

  }
  public void setArmPercentPower(double value){
    RobotMap.armMaster.set(ControlMode.PercentOutput,value);
  }
  public void setArmPostion(double angle){
    RobotMap.armMaster.set(ControlMode.MotionMagic, RobotMap.arm.mainArmEncoder.convertAngleToEncoderTics(angle));
    SmartDashboard.putNumber("error", RobotMap.armMaster.getClosedLoopError(0));
    SmartDashboard.putNumber("output", RobotMap.armMaster.getMotorOutputPercent());
    SmartDashboard.putNumber("desiredAnlge", RobotMap.armMaster.getClosedLoopTarget(0));

  }
  public void setHatchMechOut(){
    pushHatchMechOut.start();
  }
  public void setHatchMechIn(){
    pullHatchMechIn.start();
  }
  public void tenseHatchGrabbers(){
    tenseHatchGrabber.start();
  }
  public void releaseHatchGrabbers(){
    releaseHatchGrabber.start();
  }
  
  public void outTakeBall(){
    RobotMap.intakeMotor.set(ControlMode.PercentOutput, 1.0);
  }
  public void inTakeBall(){
    RobotMap.intakeMotor.set(ControlMode.PercentOutput, -1.0);
  }
  public void intakeRest(){
    RobotMap.intakeMotor.set(ControlMode.PercentOutput, 0);
  }
  public double getArmAngle(){
    return mainArmEncoder.getAngle();
  }
  public void zeroArmUP(){
    mainArmEncoder.setForwardLimitSwitchAngle();
  }
  public void zeroArmDown(){
    mainArmEncoder.setReverseLimitSwitchAngle();
  }

}
