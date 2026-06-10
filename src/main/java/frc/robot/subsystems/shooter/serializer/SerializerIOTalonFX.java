package frc.robot.subsystems.shooter.serializer;

import static frc.robot.subsystems.shooter.serializer.SerializerConstants.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersConfiguration;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOTalonFX;

public class SerializerIOTalonFX extends GenericRollersIOTalonFX {
  private TalonFX talon2 = new TalonFX(SERIALIZER_CONFIG.motorID2());

  public SerializerIOTalonFX() {
    //   super(SERIALIZER_CONFIG.motorID(), CURRENT_LIMIT_AMPS, SERIALIZER_CONFIG.inverted(),
    // SERIALIZER_CONFIG.brake(), SERIALIZER_CONFIG.reduction());
    super(
        new GenericRollersConfiguration()
            .withID(SERIALIZER_CONFIG.motorID())
            .withMotorDirection(
                SERIALIZER_CONFIG.inverted()
                    ? InvertedValue.CounterClockwise_Positive
                    : InvertedValue.Clockwise_Positive)
            .withSupplyCurrentLimit(CURRENT_LIMIT_AMPS)
            .withReduction(SERIALIZER_CONFIG.reduction())
            .withUpperVoltageLimit(UPPER_VOLT_LIMIT)
            .withLowerVoltageLimit(LOWER_VOLT_LIMIT)
            .withNeutralMode(SERIALIZER_CONFIG.brake())
            .withStatorCurrentLimit(STATOR_CURRENT_LIMIT));
    talon2.getConfigurator().apply(config);
    talon2
        .getConfigurator()
        .apply(
            new MotorOutputConfigs()
                .withInverted(
                    SERIALIZER_CONFIG.inverted() ^ SERIALIZER_CONFIG.opposeMotor()
                        ? InvertedValue.CounterClockwise_Positive
                        : InvertedValue.Clockwise_Positive)
                .withNeutralMode(
                    SERIALIZER_CONFIG.brake() ? NeutralModeValue.Brake : NeutralModeValue.Coast));
    setSlot0(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
  }

  @Override
  public void runVelocity(double velocity) {
    super.runVelocity(velocity);
    talon2.setControl(velocityControl.withVelocity(velocity));
  }

  @Override
  public void stop() {
    super.stop();
    talon2.setControl(super.neutralOutput);
  }

  @Override
  public void setSlot0(double kP, double kI, double kD, double kS, double kV, double kA) {
    super.setSlot0(kP, kI, kD, kS, kV, kA);
    talon2.getConfigurator().apply(super.gainsConfig);
  }

  @Override
  public void setSupplyCurrentLimit(double amps) {
    if (Math.abs(config.CurrentLimits.SupplyCurrentLimit - amps) > 0.01) {
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
      config.CurrentLimits.SupplyCurrentLimit = amps;
      config.withSlot0(gainsConfig);
      talon2.getConfigurator().apply(config);
      talon2
          .getConfigurator()
          .apply(
              new MotorOutputConfigs()
                  .withInverted(
                      SERIALIZER_CONFIG.inverted() ^ SERIALIZER_CONFIG.opposeMotor()
                          ? InvertedValue.CounterClockwise_Positive
                          : InvertedValue.Clockwise_Positive)
                  .withNeutralMode(
                      SERIALIZER_CONFIG.brake() ? NeutralModeValue.Brake : NeutralModeValue.Coast));
    }
  }
}
