package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utilities.Constants.HardwareID;
import frc.lib.utilities.Constants.Keys;
import frc.lib.utilities.Constants.TuningValues;
import frc.lib.utilities.Constants.VoltageVelocityValues;

import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.ColorSensorV3;

public class IndexerSubsystem extends SubsystemBase{

    TalonFX indexerMotor, intakeMotor;
    ColorSensorV3 noteDetector;
    PositionVoltage returnPosition = new PositionVoltage(0);

    public IndexerSubsystem() {
        indexerMotor = new TalonFX(HardwareID.indexerMotorCANId);
        intakeMotor = new TalonFX(HardwareID.intakeMotorCANId);
        noteDetector = new ColorSensorV3(I2C.Port.kOnboard);

        configureMotors();

    }

    public void stopIndexer() {
        indexerMotor.setVoltage(0.0);
    }

    public void intakeNote() {
        indexerMotor.setVoltage(VoltageVelocityValues.intakeVolt);
    }

    public void outtakeNote() {
        indexerMotor.setVoltage(-VoltageVelocityValues.intakeVolt);
    }
    
    public void indexNote(boolean ampShot) {
        double voltage = ampShot ? VoltageVelocityValues.indexAmpVolt : VoltageVelocityValues.indexVolt;
        indexerMotor.setVoltage(voltage);
    }

    public double setTargetPosition() {
        return indexerMotor.getPosition().getValueAsDouble() - 0.4;
    }

    public void correctNotePosition(double targetPosition) {
        indexerMotor.setControl(returnPosition.withPosition(targetPosition));
        SmartDashboard.putNumber("Indexer Motor Target Position", targetPosition);
    }
    
    public boolean isNoteCorrectlyPositioned(double targetPosition) {
        if (Math.abs(indexerMotor.getPosition().getValueAsDouble() - targetPosition) < 0.1 ) {
            return true;
        }
        return false;
    }

    public boolean isNoteLoaded() {
        int noteProximity = noteDetector.getProximity();
        if (noteProximity > Preferences.getDouble(Keys.minimumNoteProximityKey, 500)) {
            return true;
        }
        return false;
    }

    
    @Override
    public void periodic() {
        // telemetry();
    }

    private void telemetry() {
        SmartDashboard.putNumber("Indexer Motor Velocity", indexerMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake Motor Velocity", intakeMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Note Proximity", noteDetector.getProximity());
        SmartDashboard.putNumber("Indexer Motor Position", indexerMotor.getPosition().getValueAsDouble());
    }

    private void configureMotors() {
        intakeMotor.setControl(new Follower(HardwareID.indexerMotorCANId, true));
        
        indexerMotor.setNeutralMode(NeutralModeValue.Brake);
        indexerMotor.setInverted(true);

        var config = new Slot0Configs();
        config.kP = TuningValues.indexerkP;
        config.kI = TuningValues.indexerkI;
        config.kD = TuningValues.indexerkD;
        
        var voltageRamp = new OpenLoopRampsConfigs();
        voltageRamp.VoltageOpenLoopRampPeriod = 0.5;
        
        indexerMotor.getConfigurator().apply(config);
        indexerMotor.getConfigurator().apply(voltageRamp);
    }
}
