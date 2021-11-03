/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.ModuleConstants.*;

public class SwerveModule extends SubsystemBase
{
  int mModuleNumber;

  public final TalonFX mTurningMotor;
  public final TalonFX mDriveMotor;
  double mZeroOffset;
  boolean mInverted;

  static double kF;
  static double kP;
  static double kI;
  static double kD;
  int kI_Zone = 900;
  int kMaxIAccum = 1000000;
  int kErrorBand = 50;

  int kCruiseVelocity = 14000;
  int kMotionAcceleration = kCruiseVelocity * 10;

  private static final long STALL_TIMEOUT = 2000;
  private long mStallTimeBegin = Long.MAX_VALUE;

  private double m_turnOutput;
  private double m_driveOutput;

  private final PIDController m_drivePIDController =
    new PIDController(kPModuleDriveController, 0, 0);

  private final ProfiledPIDController m_turningPIDController =
    new ProfiledPIDController(
      kPModuleTurningController, 0, 0,
      new TrapezoidProfile.Constraints(kMaxModuleAngularSpeedRadiansPerSecond,
        kMaxModuleAngularAccelerationRadiansPerSecondSquared)
    );

  // Gains are example only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward =
    new SimpleMotorFeedforward(0.587, 2.3, 0.0917);
  private final SimpleMotorFeedforward m_turnFeedforward =
    new SimpleMotorFeedforward(1, 0.5);

  private double simTurnEncoderDistance;
  private double simThrottleEncoderDistance;

  private Encoder simulationTurnEncoder;
  private Encoder simulationThrottleEncoder;

  Pose2d swerveModulePose = new Pose2d();

  public SwerveModule(int moduleNumber, TalonFX TurningMotor,
    TalonFX driveMotor, double zeroOffset, boolean invertTurn,
    boolean invertThrottle)
  {
    mModuleNumber = moduleNumber;
    mTurningMotor = TurningMotor;
    mDriveMotor = driveMotor;
    mZeroOffset = zeroOffset;

    mTurningMotor.configFactoryDefault();
    mTurningMotor.configOpenloopRamp(0.1);
    mTurningMotor.configClosedloopRamp(0.1);

    mTurningMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    mDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    mTurningMotor.setInverted(invertTurn);
    mDriveMotor.setInverted(invertThrottle);
    mTurningMotor.setSelectedSensorPosition(0);
    mDriveMotor.setSelectedSensorPosition(0);

    mTurningMotor.config_kF(0,kF);
    mTurningMotor.config_kP(0,kP);
    mTurningMotor.config_kI(0,kI);
    mTurningMotor.config_IntegralZone(0, kI_Zone);
    mTurningMotor.configMaxIntegralAccumulator(0, kMaxIAccum);
    mTurningMotor.config_kD(0,kD);
    mTurningMotor.configMotionCruiseVelocity(kCruiseVelocity);
    mTurningMotor.configMotionAcceleration(kMotionAcceleration);
    mTurningMotor.configAllowableClosedloopError(0, kErrorBand);

    mTurningMotor.setNeutralMode(NeutralMode.Brake);
    mDriveMotor.setNeutralMode(NeutralMode.Brake);

    // Limit the PID Controller input range -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Zeros all the SwerveModule encoders.
  public void resetEncoders() {
    mTurningMotor.setSelectedSensorPosition(0);
    mDriveMotor.setSelectedSensorPosition(0);
    simulationTurnEncoder.reset();
    simulationThrottleEncoder.reset();
  }

  public Rotation2d getHeading() {
    return new Rotation2d(getTurningRadians());
  }

  // Returns the current angle of the module.
  public double getTurningRadians() {
    if(RobotBase.isReal())
      return mTurningMotor.getSelectedSensorPosition() * Constants.ModuleConstants.kTurningEncoderDistancePerPulse;
    else
      return simulationTurnEncoder.getDistance();
  }

  public double getTurnAngle() {
    return Units.radiansToDegrees(getTurningRadians());
  }


  // Returns the current velocity of the module.
  public double getVelocity() {
    if(RobotBase.isReal())
      return mDriveMotor.getSelectedSensorVelocity() * Constants.ModuleConstants.kDriveEncoderDistancePerPulse * 10;
    else
      return simulationThrottleEncoder.getRate();
  }

  // Returns the current state of the module.
  public SwerveModuleState getState()
  {
    return new SwerveModuleState(getVelocity(),
      new Rotation2d(getTurningRadians()));
  }

  // Sets the desired state for the module.
  public void setDesiredState(SwerveModuleState state)
  {
    SwerveModuleState outputState =
      SwerveModuleState.optimize(state, new Rotation2d(getTurningRadians()));

    // Calculate the drive output from the drive PID controller.
    m_driveOutput =
      m_drivePIDController.calculate( getVelocity(),
        outputState.speedMetersPerSecond);

    double driveFeedforward =
      m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    m_turnOutput =
      m_turningPIDController.calculate(getTurningRadians(),
        outputState.angle.getRadians());

    double turnFeedforward =
    m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    mDriveMotor.set(ControlMode.PercentOutput,
      m_driveOutput + driveFeedforward);
    mTurningMotor.set(ControlMode.PercentOutput, m_turnOutput);
  }

  public void setPercentOutput(double speed)
  {
    mDriveMotor.set(ControlMode.PercentOutput, speed);
  }

  // True is brake, false is coast
  public void setBrakeMode(boolean mode)
  {
    DriveMotor.setNeutralMode(mode ? NeutralMode.Brake : NeutralMode.Coast);
    mTurningMotor.setNeutralMode(NeutralMode.Brake);
  }

  public Pose2d getPose()
  {
    return swerveModulePose;
  }

  public void setPose(Pose2d pose)
  {
    swerveModulePose = pose;
  }

  private void updateSmartDashboard()
  {
    //SmartDashboardTab.putNumber("SwerveDrive","Turning PID " + mModuleNumber,
    //  turnOutput);
  }

  @Override
  public void periodic()
  {
    updateSmartDashboard();
  }

}
