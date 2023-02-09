// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
// This code is adapted from Sean Sun's FRC 0 to Autonomous Swerve Drive code
// and 4201 Vitruvian Bots https://github.com/4201VitruvianBots/2023SwerveSim/tree/main/2023RevSwerve/src/main/java/frc/robot
// all we changed was removing deprecated stuff and the constants, and combined the Vitruvian Bots code with Sean Sun's code to update it to 2023
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  private final CANSparkMax driveMotor;
  private final WPI_TalonSRX turningMotor;

  private final RelativeEncoder driveEncoder;
  private final Encoder turningEncoder;

  //Moves the angle motor - Liam
  private final PIDController turningPidController;

  //Connected to analog ports on roborio - L
  //Abs encoder might differ from acc angle in first boot
  private final AnalogInput absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  //This stores the difference between the initial abs encoder value and the wheel direction
  //Used later in the code to compensate for difference
  private final double absoluteEncoderOffsetRad;
  
  /** Creates a new SwerveModule. */ 
  //Asks for the port #'s of motors, sensors, and whether or not they are reversed
  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
      int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed, int encChannelA, int encChannelB) {
    
    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new AnalogInput(absoluteEncoderId);
      
    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    turningMotor = new WPI_TalonSRX(turningMotorId);
  
    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = new Encoder(encChannelA, encChannelB);
    
    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    turningEncoder.setDistancePerPulse(ModuleConstants.kTurnEncoderDistPerPulse);

    turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
    //Tells the PID controller that our system is a circle and that 180 and -180 are beside eachother
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
    }

  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public double getTurningPosition() {
    return turningEncoder.getDistance();
    
  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getTurningVelocity() {
    return turningEncoder.getRate();
  }

  public double getAbsoluteEncoderRad() {
    //gives the percentage of a full rotation by dividing abs encoder voltage by supplied voltage
    double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    //Converts percentage rotation to radians
    angle *= 2.0 * Math.PI;
    //Adjusts radians by radian offset
    angle -= absoluteEncoderOffsetRad;
    //Reverses the angle if the abs encoder has been reversed
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }
  //Resets drive enc, and changes turning encoder to the value of the abs encoder
  //Called during boot
  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turningEncoder.reset();
  
  }
  //WPI libraries request encoder values in the format of "swerve module state"
  //This method returns the encoder values in these formats
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  //Takes in the desired state of the swerve module
  //state contains desired angle and speed of the module
  public void setDesiredState (SwerveModuleState state) {
    //If the robot receives no input, the method will automatically reset the wheels to 0 degrees
    //This checks if no substantial speed has been input, in which case it will end the method
    //without resetting the wheels
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
      }

    //"state" is desired angle and "getState().angle" is the current angle
    //if the desired angle has a +90 degree difference from the current angle
    //it will turn the other way and reverse the motor
    //state = SwerveModuleState.optimize(state, getState().angle);

    //Sets motor value from 1 to -1, depending on max
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    //PID controller calculates what to set the turning motor to
    //I don't understand how it does this
    turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
  }
  public SwerveModulePosition getSMPosition(){
    return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getHeadingDegrees()));
  }
  public double getHeadingDegrees() {
    return getTurningPosition() * Constants.ModuleConstants.kTurnEncoderDistPerPulse;
  }
  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }
}  