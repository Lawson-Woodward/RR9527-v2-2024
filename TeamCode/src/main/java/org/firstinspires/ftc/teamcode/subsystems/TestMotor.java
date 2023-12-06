package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestMotor {
    private DcMotorEx test;

    public TestMotor(HardwareMap hardwareMap) {
        test = (DcMotorEx) hardwareMap.dcMotor.get("test");
        test.setPower(0.5);
    }
}
