package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utilities.Constants.HardwareID;

public class PneumaticsSubsystem extends SubsystemBase{

    public DoubleSolenoid leftClimber, rightClimber, noteAimer;
    boolean allSolenoidsDisabled = false;
    Compressor compressor;


    public PneumaticsSubsystem() {
        leftClimber = new DoubleSolenoid(PneumaticsModuleType.REVPH, HardwareID.leftClimberForwardChannel, HardwareID.leftClimberReverseChannel);
        rightClimber = new DoubleSolenoid(PneumaticsModuleType.REVPH, HardwareID.rightClimberForwardChannel, HardwareID.rightClimberReverseChannel);
        noteAimer = new DoubleSolenoid(PneumaticsModuleType.REVPH, HardwareID.noteAimerForwardChannel, HardwareID.noteAimerReverseChannel);


        compressor = new Compressor(PneumaticsModuleType.REVPH);
        compressor.enableDigital();

        setAllSolenoidsToReverse();
    }

    
    //Individual Solenoid Controls (Take Solenoid as Input)
    public void setSolenoidToReverse(DoubleSolenoid solenoid) {
        solenoid.set(Value.kReverse);
    }
    
    public void setSolenoidToForward(DoubleSolenoid solenoid) {
        solenoid.set(Value.kForward);
    }

    public void turnSolenoidOff(DoubleSolenoid solenoid) {
        solenoid.set(Value.kOff);
    }

    public void toggleSolenoid(DoubleSolenoid solenoid) {
        solenoid.toggle();
    }

    //Sets all solenoids on robot to specfic position.
    public void setAllSolenoidsToReverse() {
        setSolenoidToReverse(leftClimber);
        setSolenoidToReverse(rightClimber);
        setSolenoidToReverse(noteAimer);
    }
    
    public void setAllSolenoidsToForward() {
        setSolenoidToForward(leftClimber);
        setSolenoidToForward(rightClimber);
        setSolenoidToForward(noteAimer);
    }
    
    public void turnAllSolenoidsOff() {
        turnSolenoidOff(leftClimber);
        turnSolenoidOff(rightClimber);
        turnSolenoidOff(noteAimer);
        allSolenoidsDisabled = true;
    }
    
    public void toggleAllSolenoids() {
        leftClimber.toggle();
        rightClimber.toggle();
        noteAimer.toggle();
    }
    
    //Not reccommended to use unless necessary, compressor should always be enabled if one intends to use pneumatics.
    public void toggleCompressor() {
        if (compressor.isEnabled()) {
            compressor.disable();
            return;
        }
        compressor.enableDigital();
    }
    
    public boolean isForwardShorted() {
        if (leftClimber.isFwdSolenoidDisabled() || rightClimber.isFwdSolenoidDisabled() || noteAimer.isFwdSolenoidDisabled()) {
            return true;
        }
        return false;
    }
    
    public boolean isReverseShorted() {
        if (leftClimber.isRevSolenoidDisabled() || rightClimber.isRevSolenoidDisabled() || noteAimer.isRevSolenoidDisabled()) {
            return true;
        }
        return false;
    }
    
    private void telemetry() {
        SmartDashboard.putBoolean("All Solenoids Disabled", allSolenoidsDisabled);
    }
    
    @Override
    public void periodic() {
        //Turns pneumatics off if an electical short occurs in either the forward or reverse solenoids.
        if (isForwardShorted() || isReverseShorted()) {
            turnAllSolenoidsOff();
            compressor.disable();
        }
        
        telemetry();
    }
}
