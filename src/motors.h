//FOC
MagneticSensorSPI sensor1 = MagneticSensorSPI(17, 14, 0x3FFF);
MagneticSensorSPI sensor2 = MagneticSensorSPI(16, 14, 0x3FFF);

StepperMotor motor1 = StepperMotor(50);
StepperMotor motor2 = StepperMotor(50);

StepperDriver4PWM driver1 = StepperDriver4PWM(32, 33, 25, 26);
StepperDriver4PWM driver2 = StepperDriver4PWM(27, 2, 12, 13);

//FOC SETUP//

void focSetup() {
    // initialise magnetic sensor hardware
    sensor1.init();
    sensor2.init();
    // link the motor to the sensor
    motor1.linkSensor(&sensor1);
    motor2.linkSensor(&sensor2);

    // driver config
    // power supply voltage [V]
    driver1.voltage_power_supply = 16;
    driver1.init();
    driver2.voltage_power_supply = 16;
    driver2.init();
    // link the motor and the driver
    motor1.linkDriver(&driver1);
    motor2.linkDriver(&driver2);

    // choose FOC modulation (optional)
    motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor2.foc_modulation = FOCModulationType::SpaceVectorPWM;

    // set motion control loop to be used
    motor1.controller = ControlType::velocity;
    motor2.controller = ControlType::velocity;

    // contoller configuration
    // default parameters in defaults.h

    // velocity PI controller parameters
    motor1.PID_velocity.P = 2.3;
    motor1.PID_velocity.I = 22;
    motor1.PID_velocity.D = 0;
    // maximal voltage to be set to the motor
    motor1.voltage_limit = 15;

    // velocity low pass filtering time constant
    // the lower the less filtered
    motor1.LPF_velocity.Tf = 0.05;

    // angle P controller
    motor1.P_angle.P = 20;
    // maximal velocity of the position control
    motor1.velocity_limit = 60;

    // velocity PI controller parameters
    motor2.PID_velocity.P = 2.3;
    motor2.PID_velocity.I = 22;
    // maximal voltage to be set to the motor
    motor2.voltage_limit = 15;

    // velocity low pass filtering time constant
    // the lower the less filtered
    motor2.LPF_velocity.Tf = 0.05;

    // angle P controller
    motor2.P_angle.P = 20;
    // maximal velocity of the position control
    motor2.velocity_limit = 60;
    
    motor1.PID_velocity.output_ramp = 10000;
    motor2.PID_velocity.output_ramp = 10000;
    // use monitoring with serial
    // comment out if not needed
    motor2.useMonitoring(Serial);

    // initialize motor
    motor1.init();
    motor2.init();

    // align sensor and start FOC
    motor1.initFOC(2.91,CCW);
    motor2.initFOC(3.33,CW);
    Serial.println(motor1.zero_electric_angle);
    Serial.println(sensor1.natural_direction);
    Serial.println(motor2.zero_electric_angle);
    Serial.println(sensor2.natural_direction);

    
    
    Serial.println("Motor ready.");


}

