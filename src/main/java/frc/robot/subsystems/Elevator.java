package frc.robot.subsystems;

import java.util.List;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Ports;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final CANSparkMax leaderMotor;
  private final CANSparkMax followerMotor;
  private final DigitalInput bottomLimitSwitch;

  // private final CANSparkMax leaderMotor = new CANSparkMax(Ports.ElevatorPorts.LEADER_MOTOR, MotorType.kBrushless);
  // private final CANSparkMax followerMotor = new CANSparkMax(Ports.ElevatorPorts.FOLLOWER_MOTOR, MotorType.kBrushless);

  private final RelativeEncoder encoder;
  // private final RelativeEncoder RelativeEncoder = leaderMotor.getEncoder();

  private final Encoder encoder;
  private final PIDController pid;
  private final ElevatorFeedforward ff;

  public Elevator() {
    leaderMotor = new CANSparkMax(Ports.ElevatorPorts.LEADER_MOTOR, MotorType.kBrushless);
    followerMotor = new CANSparkMax(Ports.ElevatorPorts.FOLLOWER_MOTOR, MotorType.kBrushless);
    followerMotor.follow(leaderMotor, false);
    encoder = leaderMotor.getEncoder();
    bottomLimitSwitch = new DigitalInput(Ports.ElevatorPorts.BOTTOM_LIMIT_SWITCH_PORT);
    pid = new PIDController(Constants.ElevatorPIDConstants.kP, Constants.ElevatorPIDConstants.kI, Constants.ElevatorPIDConstants.kD);
    pid.setTolerance(0.1);

    for(CANSparkMax motor : List.of(leaderMotor, followerMotor)) {
      motor.restoreFactoryDefaults();
      motor.setIdleMode(IdleMode.kBrake);
    }
    followerMotor.follow(leaderMotor, true);

  }

  private void setSpeed(double speed) {
    leaderMotor.set(speed);
  }

  public Command move(DoubleSupplier speed) {
    return run(() -> setSpeed(speed.getAsDouble()));
  }

  public double getHeight() {
    return encoder.getDistance();
  }

  public Command goTo(State setpoint) {
    return run(() -> updateSetpoint(setpoint));
  }

  public void updateSetpoint(State setpoint) {
    double ffOutput = ff.calculate(setpoint.velocity);
    double pidOutput = pid.calculate(getHeight(), setpoint.position);

    motor.setVoltage(ffOutput + pidOutput);
  }

  public Command moveToPosition(double setpoint) {
    return run(() -> {
        elevatorPID(encoder.getPosition(), setpoint);
    });
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
    return runOnce(() -> encoder.setPosition(0));
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