package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Ports;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import java.util.List;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import java.util.function.DoubleSupplier;

public class Elevator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  // private final SparkMax elevatorMotor;
  private final CANSparkMax leaderMotor = new CANSparkMax(Ports.ElevatorPorts.LEADER_MOTOR, MotorType.kBrushless);
  private final CANSparkMax followerMotor = new CANSparkMax(Ports.ElevatorPorts.FOLLOWER_MOTOR, MotorType.kBrushless);

  private final RelativeEncoder elevatorEncoder = leaderMotor.getEncoder();

  private final CANSparkMax motor;

  private final Encoder encoder;

  private final PIDController pid;
  private final ElevatorFeedforward ff;

  public Elevator() {
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