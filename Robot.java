/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */



private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(1);
private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(2);
private final WPI_VictorSPX leftSlave = new WPI_VictorSPX(5);
private final WPI_VictorSPX rightSlave = new WPI_VictorSPX(3);

private final DifferentialDrive robotDrive = new DifferentialDrive(leftMaster, rightMaster);

private final WPI_TalonSRX turretControl = new WPI_TalonSRX(11);
private final WPI_TalonSRX mainShooter = new WPI_TalonSRX(7);
private final WPI_TalonSRX topShooter = new WPI_TalonSRX(8);

private final WPI_VictorSPX feederWheel = new WPI_VictorSPX(13);
private final WPI_VictorSPX intakeSystem = new WPI_VictorSPX(17);

private final WPI_TalonSRX elevatorMaster = new WPI_TalonSRX(9);
private final WPI_VictorSPX elevatorSlave = new WPI_VictorSPX(10);

private final Compressor compressor =  new Compressor();

private final DoubleSolenoid intakeArms = new DoubleSolenoid(5, 7);
private final DoubleSolenoid ratchetArm = new DoubleSolenoid(4, 1);

private final Joystick driverJoystick = new Joystick(0);
private final Joystick manipJoystick = new Joystick(1);
CameraServer server;

@Override
  public void robotInit() {
    server = CameraServer.getInstance();
    server.startAutomaticCapture(0);
    
    //interted settings
    leftMaster.setInverted(true);
    rightMaster.setInverted(true);

    //slave setups
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    elevatorSlave.follow(elevatorMaster);

    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);

    //init encoders
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    mainShooter.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    topShooter.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    turretControl.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

    leftMaster.clearStickyFaults(10);
    rightMaster.clearStickyFaults(10);
    mainShooter.clearStickyFaults(10);
    topShooter.clearStickyFaults(10);
    turretControl.clearStickyFaults(10);

    //config PIDs
    mainShooter.config_kF(0, 0.02764865, 10);
    mainShooter.config_kP(0, 1.0, 10);
    mainShooter.config_kI(0, 0, 10);
    mainShooter.config_kD(0, 0, 10);

    topShooter.config_kF(0, 0.02764865, 10);
    topShooter.config_kP(0, 0.05, 10);
    topShooter.config_kI(0, 0, 10);
    topShooter.config_kD(0, 0, 10);

    turretControl.config_kF(0, .5, 10);
    turretControl.config_kP(0, 0, 10);
    turretControl.config_kI(0, 0, 10);
    turretControl.config_kD(0, 0, 10);

    leftMaster.config_kF(0, 0, 10);
    leftMaster.config_kF(0, 0, 10);
    leftMaster.config_kF(0, 0, 10);
    leftMaster.config_kF(0, 0, 10);

    rightMaster.config_kF(0, 0, 10);
    rightMaster.config_kF(0, 0, 10);
    rightMaster.config_kF(0, 0, 10);
    rightMaster.config_kF(0, 0, 10);

    //reset encoders
    leftMaster.setSelectedSensorPosition(0, 0, 10);
    rightMaster.setSelectedSensorPosition(0, 0, 10);
    mainShooter.setSelectedSensorPosition(0, 0, 10);
    topShooter.setSelectedSensorPosition(0, 0, 10);
    turretControl.setSelectedSensorPosition(0, 0, 10);
    turretControl.setSensorPhase(true);

    //set configs
    turretControl.configMotionAcceleration(800, 10);
    turretControl.configMotionCruiseVelocity(800, 10);
    turretControl.setNeutralMode(NeutralMode.Coast);
    
    mainShooter.setNeutralMode(NeutralMode.Coast);
    topShooter.setNeutralMode(NeutralMode.Coast);


    //start compressor
    compressor.start();

  }

  

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    final double leftTravelled = leftMaster.getSelectedSensorPosition();
    final double rightTravelled = rightMaster.getSelectedSensorPosition();

    while (leftTravelled < 2000 && rightTravelled > -2000){
      leftMaster.set(ControlMode.MotionMagic, 2000);
      rightMaster.set(ControlMode.MotionMagic, -2000);

      
    }
  }

  @Override
  public void teleopInit() {
    
  }

  @Override
  public void teleopPeriodic() {
    //networktables
    final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    final NetworkTableEntry tx = table.getEntry("tx");
    final NetworkTableEntry ty = table.getEntry("ty");
    final NetworkTableEntry ta = table.getEntry("ta");
    final NetworkTableEntry ledMode = table.getEntry("ledMode");
    
    //read values periodically
    final double x = tx.getDouble(0.0);
    final double y = ty.getDouble(0.0);
    final double area = ta.getDouble(0.0);
    
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    robotDrive.arcadeDrive(driverJoystick.getY(), driverJoystick.getX()* -1);

      double shooterSpeed = 0;
      mainShooter.set(ControlMode.PercentOutput, shooterSpeed);
    if (manipJoystick.getRawButton(7)==true){
      shooterSpeed = 20300;
      mainShooter.set(ControlMode.Velocity, shooterSpeed);
    } 
    if (manipJoystick.getRawButton(9)==true){
      shooterSpeed = 23000;
      mainShooter.set(ControlMode.Velocity, shooterSpeed);
    } 
    if (manipJoystick.getRawButton(11)==true){
      shooterSpeed = 25500;
      mainShooter.set(ControlMode.Velocity, shooterSpeed);
    }
      
    topShooter.set(ControlMode.PercentOutput, shooterSpeed);
    if (manipJoystick.getRawButton(7)==true){
      topShooter.set(ControlMode.Velocity, shooterSpeed * -1);
    } 
    if (manipJoystick.getRawButton(9)==true){
      topShooter.set(ControlMode.Velocity, shooterSpeed * -1);
    } 
    if (manipJoystick.getRawButton(11)==true){
      topShooter.set(ControlMode.Velocity, shooterSpeed * -1);
    }

    final double kError2Deg = 4096 / 360;
    final double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    final double trackConstant = x * kError2Deg;
    double currentPosition = turretControl.getSelectedSensorPosition();

    if (tv < 1.0) {
  turretControl.set(ControlMode.MotionMagic, currentPosition);
      return;
    }

      double turretPosition = 0; ////
      ledMode.setNumber(1);
      if (manipJoystick.getRawButton(3)==true){
        turretPosition = 4000; //forward
        turretControl.set(ControlMode.MotionMagic, turretPosition);
      } 
      if (manipJoystick.getRawButton(4)==true){
        turretPosition = -1; //backwards
        turretControl.set(ControlMode.MotionMagic, turretPosition);
      } 
      if (manipJoystick.getRawButton(7)==true){
        if (x < 0){
          turretPosition = currentPosition + trackConstant);
          turretControl.set(ControlMode.MotionMagic, turretPosition);
        }if (x > 0){
          turretPosition = currentPosition - trackConstant);
          turretControl.set(ControlMode.MotionMagic, turretPosition);
        }
        ledMode.setNumber(0);
      } 
      if (manipJoystick.getRawButton(9)==true){
        if (x < 0){
          turretControl.set(ControlMode.MotionMagic, currentPosition + trackConstant);
        }if (x > 0){
          turretControl.set(ControlMode.MotionMagic, currentPosition - trackConstant);
        }
        ledMode.setNumber(0);
      } 
      if (manipJoystick.getRawButton(11)==true){
          if (x < 0){
        turretControl.set(ControlMode.MotionMagic, currentPosition + trackConstant);
      }   if (x > 0){
        turretControl.set(ControlMode.MotionMagic, currentPosition - trackConstant);
      }
        ledMode.setNumber(0);

      }
      turretControl.set(ControlMode.MotionMagic, turretPosition);

    double feederSpeed = 0; ////
      if (manipJoystick.getRawButton(2)==true){
        feederSpeed = -0.75;
      }
      feederWheel.set(ControlMode.PercentOutput, feederSpeed);

      double intakeSpeed = 0; ////
      if (manipJoystick.getRawButton(2)==true){
        intakeSpeed = -0.7;
      }
      if (manipJoystick.getRawButton(1)==true){
        intakeSpeed = -0.6;
      }
      intakeSystem.set(ControlMode.PercentOutput, intakeSpeed);

    double elevatorSpeed = 0; ////
      if (manipJoystick.getRawButton(10)==true){
        elevatorSpeed = 1;
      } else if (manipJoystick.getRawButton(12)==true){
        elevatorSpeed = -1;
      }
      elevatorMaster.set(ControlMode.PercentOutput, elevatorSpeed);

    intakeArms.set(Value.kOff);
    if (driverJoystick.getRawButton(1)==true){
      intakeArms.set(Value.kForward);;
    }
    if (driverJoystick.getRawButton(2)==true){
        intakeArms.set(Value.kReverse);;
    }

    ratchetArm.set(Value.kOff);
    if (manipJoystick.getRawButton(10)==true){
        ratchetArm.set(Value.kReverse);
    }    
    if (manipJoystick.getRawButton(12)==true){
        ratchetArm.set(Value.kForward);
    }
  }
      
    
    

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
  

      }
    
