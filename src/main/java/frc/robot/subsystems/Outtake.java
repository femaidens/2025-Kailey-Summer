package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Ports;
import frc.robot.Ports.OuttakePorts;
import frc.robot.Constants;

public class Outtake extends SubsystemBase {
    private final SparkMax outtakeMotor;
    private final DigitalInput frontBeamBreak;
    private final DigitalInput middleBeamBreak;

    public Outtake() {
        outtakeMotor = new SparkMax(Ports.OuttakePorts.OUTTAKE_MOTOR, MotorType.kBrushless);
        frontBeamBreak = new DigitalInput(OuttakePorts.FRONT_BEAM_BREAK);
        middleBeamBreak = new DigitalInput(OuttakePorts.MIDDLE_BEAM_BREAK);

        SparkMaxConfig config = new SparkMaxConfig();
            config
            .idleMode(IdleMode.kBrake);
            
        outtakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command runFoward() {
        return run(() -> outtakeMotor.set(Constants.OuttakeSpeeds.MOTOR_SPEED));
    }

    public Command runReverse() {
        return run(() -> outtakeMotor.set(-Constants.OuttakeSpeeds.MOTOR_SPEED));
    }

    public Command stopMotor() {
        return runOnce(() -> outtakeMotor.set(0));
    }

    public boolean frontBeamBreakBroken() {
        return !frontBeamBreak.get();
    }

    public boolean middleBeamBreakBroken() {
        return !middleBeamBreak.get();
    }

    public boolean isCoralInPosition() {
        return !middleBeamBreakBroken() && frontBeamBreakBroken();
    }

    public Command setVoltageCmd(double voltage) {
        return run(() -> outtakeMotor.setVoltage(voltage));
    }

}
