package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utilities.Constants.HardwareID;

public class LauncherSubsystem extends SubsystemBase{

    TalonFX bottomSpinner, middleSpinner, topSpinner;
    
    public LauncherSubsystem() {
        bottomSpinner = new TalonFX(HardwareID.bottomSpinnerMotorCANId);
        middleSpinner = new TalonFX(HardwareID.middleSpinnerMotorCANId);
        topSpinner = new TalonFX(HardwareID.topSpinnerMotorCANId);

        bottomSpinner.setNeutralMode(NeutralModeValue.Brake);
        middleSpinner.setNeutralMode(NeutralModeValue.Brake);
        topSpinner.setNeutralMode(NeutralModeValue.Brake);
    }

    public void spinLowerSpinners() {
        bottomSpinner.set(0.5);
        middleSpinner.set(0.5);
    }

    public void spinLowerSpinners(double speed) {
        bottomSpinner.set(speed);
        middleSpinner.set(speed);
    }

    public void spinUpperSpinners() {
        middleSpinner.set(-0.3);
        topSpinner.set(-0.3);
    }

    public void spinUpperSpinners(double speed) {
    middleSpinner.set(-speed);
    topSpinner.set(-speed);
    }

    public void stopSpinners() {
        bottomSpinner.set(0.0);
        middleSpinner.set(0.0);
        topSpinner.set(0.0);
    }

    @Override
    public void periodic() {
        telemetry();
    }

    public void telemetry() {
        SmartDashboard.putNumber("Bottom Spinner Speed", bottomSpinner.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Middle Spinner Speed", middleSpinner.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Top Spinner Speed", topSpinner.getVelocity().getValueAsDouble());
    }
}
