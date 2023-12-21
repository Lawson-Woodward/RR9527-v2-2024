package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private Servo leftLift;
    private Servo rightLift;
    private CRServo compliant;
    private Servo leftBrush;     //    0.5 is intaking maybe 1 position is push position
    private Servo rightBrush;    //    0.5 is intaking maybe 0.0 position is push position
    private DcMotorEx carWash;

    public Intake(HardwareMap hardwareMap)
    {
        leftLift = hardwareMap.servo.get("leftLift");
        rightLift = hardwareMap.servo.get("rightLift"); //direction reversed
        compliant = hardwareMap.crservo.get("compliant");
        leftBrush = hardwareMap.servo.get("leftBrush"); //check these names
        rightBrush = hardwareMap.servo.get("rightBrush");  //direction reversed

        leftLift.setPosition(0);  //needs to be set to above ground
        rightLift.setPosition(0); //needs to be set to above ground
        compliant.setPower(0);    //just needs to be set to no power <-|
        leftBrush.setPosition(0);    // set to hold pixel
        rightBrush.setPosition(0);   // set to hold pixel

        carWash = (DcMotorEx) hardwareMap.dcMotor.get("carWash");
        carWash.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void intaking() {
        this.liftDown();
        this.brushDown();
        compliant.setPower(1);
        carWash.setPower(1);
    }

    public void resting() {
        this.liftUp();
        this.spitOut();
        this.brushUp();
        compliant.setPower(0);
        carWash.setPower(0);
    }

    public void spitOut() {
        // need to run a quick timer for this one idk how to do that
        compliant.setPower(-1);
        carWash.setPower(-1);
    }

    public void liftUp() {
        leftLift.setPosition(0.1);
        rightLift.setPosition(-0.1);
    }

    public void liftDown() {
        leftLift.setPosition(0);
        rightLift.setPosition(0);
    }

    public void brushUp() {
        leftBrush.setPosition(0);
        rightBrush.setPosition(0);
    }

    public void brushDown() {
        leftBrush.setPosition(-0.1);
        rightBrush.setPosition(-0.1);
    }

    public void initIntake() {
        leftBrush.setPosition(0.4);
        rightBrush.setPosition(-0.4);
        //timer here
        brushDown();
    }

    public double getLeftLiftPosition()
    {
        return leftLift.getPosition();
    }
    public double getRightLiftPosition()
    {
        return rightLift.getPosition();
    }
    public double getCompliantPower()
    {
        //return compliant.getPower;
        return 0.0;
    }
    public double getLeftBrushPosition()
    {
        return leftBrush.getPosition();
    }
    public double getRightBrushPosition()
    {
        return rightBrush.getPosition();
    }

    public double getCarWashPosition()
    {
        return carWash.getCurrentPosition();
    }


    public void letDownSlowly() {
        //put pixel down slowly, just a nicer spitOut() for auto
    }






    public CRServo getCompliant() {
        return compliant;
    }

    public DcMotorEx getCarWash() {
        return carWash;
    }

    public Servo getLeftBrush() {
        return leftBrush;
    }

    public Servo getRightBrush() {
        return rightBrush;
    }

    public Servo getLeftLift() {
        return leftLift;
    }

    public Servo getRightLift() {
        return rightLift;
    }

}
