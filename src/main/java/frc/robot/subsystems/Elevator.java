package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Ports;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMax leaderMotor;
  private final SparkMax followerMotor;
  private final DigitalInput bottomLimitSwitch;

  // private final CANSparkMax leaderMotor = new CANSparkMax(Ports.ElevatorPorts.LEADER_MOTOR, MotorType.kBrushless);
  // private final CANSparkMax followerMotor = new CANSparkMax(Ports.ElevatorPorts.FOLLOWER_MOTOR, MotorType.kBrushless);

  private final SparkRelativeEncoder relativeEncoder;
  private final SparkAbsoluteEncoder absoluteEncoder;
  // private final RelativeEncoder RelativeEncoder = leaderMotor.getEncoder();

  private final PIDController pid;
  private final ElevatorFeedforward ff;

  public Elevator() {
    leaderMotor = new SparkMax(Ports.ElevatorPorts.LEADER_MOTOR, MotorType.kBrushless);
    followerMotor = new SparkMax(Ports.ElevatorPorts.FOLLOWER_MOTOR, MotorType.kBrushless);

    relativeEncoder = (SparkRelativeEncoder) leaderMotor.getEncoder();
    absoluteEncoder = leaderMotor.getAbsoluteEncoder();
    
    bottomLimitSwitch = new DigitalInput(Ports.ElevatorPorts.BOTTOM_LIMIT_SWITCH_PORT);
    pid = new PIDController(Constants.ElevatorPIDValues.kP, Constants.ElevatorPIDValues.kI, Constants.ElevatorPIDValues.kD);
    pid.setTolerance(0.1);
    ff = new ElevatorFeedforward(Constants.ElevatorFeedforwardConstants.kS, Constants.ElevatorFeedforwardConstants.kG, Constants.ElevatorFeedforwardConstants.kV, Constants.ElevatorFeedforwardConstants.kA);

    SparkMaxConfig leaderConfig = new SparkMaxConfig();
      leaderConfig
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .smartCurrentLimit(Constants.ElevatorConstants.CURRENT_LIMIT);

    leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig followerConfig = new SparkMaxConfig();
      followerConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(Constants.ElevatorConstants.CURRENT_LIMIT)
      .follow(leaderMotor, true);

    followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // for(SparkMax motor : List.of(leaderMotor, followerMotor)) {
    //   motor.restoreFactoryDefaults();
    //   motor.setIdleMode(IdleMode.kBrake);
    // }
    // followerMotor.follow(leaderMotor, true);

  }

  private void elevatorPID(double current, double setpoint) {
    leaderMotor.setVoltage(pid.calculate(current, setpoint));
  }

  public Command setVoltageCmd(double voltage) {
    return run(() -> leaderMotor.setVoltage(voltage));
  }

  public double getHeight() {
    return relativeEncoder.getPosition();
  }

  public Command moveToPosition(double setpoint) {
    return run(() -> {
        elevatorPID(relativeEncoder.getPosition(), setpoint);
    });
  }

  public Command reverseMotor() {
    return run(() -> leaderMotor.set(-Constants.ElevatorSpeeds.MOTOR_SPEED));
  }
  
  public Command runElevatorMotorCmd() {
    return run(() -> leaderMotor.set(Constants.ElevatorSpeeds.MOTOR_SPEED));
  }

  public Command stopElevatorMotorCmd() {
    return runOnce(() -> leaderMotor.set(0));
  }

  public boolean hitBottomLimit() {
    return !bottomLimitSwitch.get();
  }

  public Command resetEncoder() {
    return runOnce(() -> relativeEncoder.setPosition(0));
  }

  public Command setLevel(double setpoint) {
    return run(() -> elevatorPID(relativeEncoder.getPosition(), setpoint));
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}