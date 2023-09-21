package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.commands.State.*;

//import org.firstinspires.ftc.robotcore.external.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.State;

public class Robot {

    public Drivetrain drivetrain;
    private State state;
    Telemetry telemetry;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        drivetrain = new Drivetrain(hardwareMap);
    }

    public void setPosition(State state){
        //set robot positions based on state (later)
    }

    public void setState(State state) {
        this.state = state;
    }
    public State getState() {
        return state;
    }
}
