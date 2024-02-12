package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import static java.lang.Math.*;

import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

//
@Autonomous(name = "\uD83D\uDDFF Spike Auto \uD83D\uDDFF", group = "Final")
public class SpikeAuto extends Spike {

    public void build(){

    }

    @Override
    public void setCameraPosition(){
        webcamName = "Webcam 1";
    }
}