package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Ports;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Elevator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMax elevatorMotor;
  private final CANSparkMax leaderMotor = new CanSparkMax(Ports.ElevatorPorts.LEADER_MOTOR, MotorType.kBrushless);
  private final CANSparkMax followerMotor = new CanSparkMax(Ports.ElevatorPorts.FOLLOWER_MOTOR, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
  private final RelativeEncoder rightEncoder = rightLeader.getEncoder();

  public Elevator() {
    for(CANSparkMax motor : List.of(leaderMotor, followerMotor)) {
      motor.restoreFactoryDefaults();
      motor.setIdleMode(IdleMode.kBreak)
    }

  }

  private void setSpeed(double speed) {
    leaderMotor.set(speed);
  }

  public Command move(DoubleSupplier speed) {
    return run(() -> setSpeed(speed.getAsDouble()));
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