package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utilities.LimelightHelpers;
import frc.lib.utilities.Constants.HardwareID;
import frc.lib.utilities.Constants.Keys;
import frc.lib.utilities.Constants.TuningValues;

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
    double targetPosition;

    public IndexerSubsystem() {
        indexerMotor = new TalonFX(HardwareID.indexerMotorCANId);
        intakeMotor = new TalonFX(HardwareID.intakeMotorCANId);
        intakeMotor.setControl(new Follower(HardwareID.indexerMotorCANId, true));
        
        indexerMotor.setNeutralMode(NeutralModeValue.Brake);
        indexerMotor.setInverted(true);

        var config = new Slot0Configs();
        config.kP = TuningValues.indexerkP;
        config.kI = TuningValues.indexerkI;
        config.kD = TuningValues.indexerkD;
        indexerMotor.getConfigurator().apply(config);

        targetPosition = indexerMotor.getPosition().getValueAsDouble();
        
        noteDetector = new ColorSensorV3(I2C.Port.kOnboard);
    }

    public void stopIndexer() {
        indexerMotor.setVoltage(0.0);
    }

    public void intakeNote() {
        indexerMotor.setVoltage(Preferences.getDouble(Keys.intakeVoltKey, 3.0));
    }

    public void outtakeNote() {
        indexerMotor.setVoltage(-Preferences.getDouble(Keys.intakeVoltKey, 3.0));
    }

    public double setTargetPosition() {
        return indexerMotor.getPosition().getValueAsDouble() - Preferences.getDouble(Keys.correctNotePositionKey, 0.85);
    }

    public void correctNotePosition(double targetPosition) {
        indexerMotor.setControl(returnPosition.withPosition(targetPosition));

        SmartDashboard.putNumber("Indexer Motor Target Position", targetPosition);
    }
    
    public boolean isNoteCorrectlyPositioned() {
        if (Math.abs(indexerMotor.getPosition().getValueAsDouble() - targetPosition) < 0.5 ) {
            return true;
        }
        return false;
    }

    public void indexNote(boolean ampShot) {
        double voltage = ampShot ? Preferences.getDouble(Keys.indexAmpVoltKey, 3.0) : Preferences.getDouble(Keys.indexVoltKey, 6.0);
        indexerMotor.setVoltage(voltage);
    }
    
    public boolean isNoteLoaded() {
        int noteProximity = noteDetector.getProximity();
        if (noteProximity > Preferences.getDouble(Keys.minimumNoteProximityKey, 500)) {
            LimelightHelpers.setLEDMode_ForceBlink("limelight");
            return true;
        }
        return false;
    }

    
    @Override
    public void periodic() {
        telemetry();
    }

    public void telemetry() {
        //SmartDashboard.putNumber("Indexer Motor Velocity", indexerMotor.getVelocity().getValueAsDouble());
        //SmartDashboard.putNumber("Intake Motor Velocity", intakeMotor.getVelocity().getValueAsDouble());
        //SmartDashboard.putNumber("Note Proximity", noteDetector.getProximity());
        SmartDashboard.putNumber("Indexer Motor Position", indexerMotor.getPosition().getValueAsDouble());
    }
}
