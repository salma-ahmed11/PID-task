#define _STM32F1_

#define outputA PA0
#define outputB PA1

const int enA = PA3;
const int in1 = PA4;
const int in2 = PA5;

const float COUNTS_PER_REVOLUTION = 360.0;
const float WHEEL_DIAMETER = 10.0;   // cm
const float WHEEL_CIRCUMFERENCE = 3.14159 * WHEEL_DIAMETER;

volatile long counter = 0;

long lastCounter = 0;
bool aLastState;

unsigned long lastTime = 0;

float velocity = 0.0;
float distancePerCount = 0.0;

// PID
float setP = 20.0;      // desired speed (cm/s)

float kp = 1.0;
float ki = 0.1;
float kd = 1.0;

float integration = 0;
float previousError = 0;

void encoderISR()
{
    bool a = digitalRead(outputA);
    bool b = digitalRead(outputB);

    if (a != aLastState)
    {
        if (b != a)
            counter++;
        else
            counter--;
    }

    aLastState = a;
}

// PID FUNCTION

float Pid(float dt, float currentVelocity)

{
    float error = setP - currentVelocity;

    // Derivative
    float derivative =
        (error - previousError) / dt;

    // PID without integral first
    float pTerm = kp * error;
    float dTerm = kd * derivative;
    float iTerm = ki * integration;

    float output = pTerm + iTerm + dTerm;

    // Anti-Windup
    bool saturatedHigh = (output > 255);
    bool saturatedLow  = (output < 0);

    if (!(saturatedHigh && error > 0) &&
        !(saturatedLow  && error < 0))
    {
        integration += error * dt;
    }

    // Optional safety limit
    integration = constrain(integration, -100, 100);

    // Recalculate with updated integral
    iTerm = ki * integration;

    output = pTerm + iTerm + dTerm;

    output = constrain(output, 0, 255);

    previousError = error;

    return output;
}

void setup()
{
    pinMode(outputA, INPUT_PULLUP);
    pinMode(outputB, INPUT_PULLUP);

    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    Serial.begin(9600);

    attachInterrupt(
        digitalPinToInterrupt(outputA),
        encoderISR,
        CHANGE);

    aLastState = digitalRead(outputA);

    distancePerCount =
        WHEEL_CIRCUMFERENCE /
        COUNTS_PER_REVOLUTION;

    Serial.println("Encoder PID Ready");

    lastTime = millis();
}


void loop()
{
    unsigned long currentTime = millis();

    // update every 100 ms
    if (currentTime - lastTime >= 100)
    {
        float deltaTime =
            (currentTime - lastTime) / 1000.0;

        noInterrupts();
        long currentCount = counter;
        interrupts();

        long deltaCount =
            currentCount - lastCounter;

        float distanceTraveled =
            deltaCount * distancePerCount;

        velocity =
            distanceTraveled / deltaTime;

        float outputPWM =
            Pid(deltaTime, velocity);

        // Motor Forward
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);

        analogWrite(enA, (int)outputPWM);

        Serial.print("Velocity = ");
        Serial.print(velocity);
        Serial.print(" cm/s");

        Serial.print(" | PWM = ");
        Serial.println(outputPWM);

        lastCounter = currentCount;
        lastTime = currentTime;
    }
}
