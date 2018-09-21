// pragma region bruges til at gøre vores kode mere læsbar og struktureret. Det gør
// så man kan kollapse koden i Visual Studio. Når koden bliver compiled ser compileren
// bort fra pragma region ligesom den gør ved kommentarer
#pragma region Libraries, Objects and ENUMs

// biblioteker
#include "MeMegaPi.h"
#include "Wire.h"

// Motorer UDEN encoder (bruges til manuel styring)
MeMegaPiDCMotor mL(PORT1B);     // DC-motor til venstre larvefodsmotor
MeMegaPiDCMotor mR(PORT2B);     // DC-motor til højre larvefodsmotor
MeMegaPiDCMotor mArm(PORT3B);   // DC-motor til arm
MeMegaPiDCMotor mClamp(PORT4B); // DC-motor til klo

// Motorer MED encoder (bruges til automatisk program)
MeEncoderOnBoard EmL(SLOT_1);   // Encodermotor til venstre larvefodsmotor
MeEncoderOnBoard EmR(SLOT_2);   // Encodermotor til højre larvefodsmotor
MeEncoderOnBoard EmArm(SLOT_3); // Encodermotor til arm

// De andre sensorer på robotten
MeLineFollower LFollower(PORT_5);       // Line follower
MeUltrasonicSensor USensor(PORT_6);     // Ultralydssensor
MeGyro Gyro;                            // Gyroskop

// ENUM så det er nemmere at læse koden vi bruger til automatisk program
enum AutoState { OPGAVE_1, OPGAVE_2, OPGAVE_3, OPGAVE_4, OPGAVE_5, OPGAVE_6, NOTHING };

AutoState aState; // enum til automatisk program

#pragma endregion

#pragma region Classes and Functions

// class, som har funktioner vedrørende dataoverførsler igennem Bluetooth
class DataTransfer {
private:
    byte ReceiveDataBuffer[16]; // modtage data fra Appen gennem Bluetooth

    void Decode(); // erklærer, at vi har en funktion defineret længere nede
public:
    int8_t TransmitBuffer[16];  // sende data til Appen

    // funktion der har mulighed for at modtage data fra appen
    void isReceived() {
        uint8_t cnt = 0; // looper igennem hver byte modtaget

        // looper igennem hvert modtaget byte og skriver ind i bufferen
        while (Serial3.available() != (int)0) {

            ReceiveDataBuffer[cnt] = Serial3.read();

            cnt++;

            delay(1);
        }

        Serial.print("Received: ");
        Serial.print(cnt);
        Serial.print(" bytes. ");

        Serial.print("The first byte is: ");
        Serial.print(ReceiveDataBuffer[0]);
        Serial.println(".");

        Decode();
    }

    void Send(uint8_t nofData); // erklærer, at vi har en funktion defineret længere nede
};

// class, som har funktioner til robottens sensorer: ultralydssensor, gyro, linefollower
class Sensor {
private:
    uint8_t fart_ = 120; // hvor hurtigt motorene skal køre med
public:
    // ser om ultralydssensor detekterer noget indenfor en bestemt afstand
    // length = afstand til objekt målt i cm (centimeter), max er 400cm
    bool CheckUltrasonicDistance(uint16_t length) {
        return USensor.distanceCm() < length;
    }

    // kan dreje hele robotten præcist ved brug af gyroskop
    // degrees = antal positive grader, der skal drejes i bestemt retning, max = 180
    // direction = enten 'L' når man vil dreje til venstre eller 'R' til højre
    void Turn(uint8_t degrees, char direction) {

        // TODO: Ændre denne til et while(true) loop?
        for (int i = 0; i < degrees / 2; i++) {
            Gyro.update(); // Gyro opdateres så Gyro.getAngleZ() får ny værdi

            //TODO: Sæt denne if-statement uden for loopet, men IKKE den nested switch statement
            if (degrees <= 180) {
                switch (direction) {
                case 'L':
                    mL.run(-fart_); // da motorene vender forskellige veje, så holder
                    mR.run(-fart_); // robotten stille og drejer ved disse værdier

                    Serial.print("Turning: ");
                    Serial.print(degrees);
                    Serial.println(" degrees to the left.");

                    // når robotten har drejet bestemt antal grader, så stop motorene
                    if (Gyro.getAngleZ() > degrees /* -1 */) {
                        mL.stop();
                        mR.stop();
                        break;
                    }
                    break;

                case 'R':
                    mL.run(fart_);
                    mR.run(fart_);

                    Serial.print("Turning: ");
                    Serial.print(degrees);
                    Serial.println(" degrees to the right.");

                    if (Gyro.getAngleZ() < -degrees /* +1 */) {
                        mL.stop();
                        mR.stop();
                        break;
                    }
                    break;
                }
            }
            delay(100);
        }
    }

    // funktion der følger den sort-hvid-sorte linje på banen
    void FollowLine() {

    }
};

// class, som har funktionerne, der bruges, når robotten er i manuel tilstand
// dvs. DC-motorene UDEN encoder
class ManualControl {
private:
    uint8_t vent_ = 200; // bruges når der bruges delay, dvs. vent 200 milisekunder

    uint8_t fart_ = 150; // angiver minimum hastighed til motorene 

    // i ManualControl class skal der bruges funktioner
    // fra DataTransfer classen
    class DataTransfer data;

    // funktion til at stoppe motorene
    void Stop() {
        mL.stop();
        mR.stop();
        mArm.stop();
        mClamp.stop();
    }
public:
    // funktion til at styre larveføddernes hastighed
    // mLSpeed = venstre motor hastighed fra Appen
    // mRSpeed = højre motor hastighed fra Appen
    void Drive(byte mLSpeed, byte mRSpeed) {

        // konverter fra unsigned int (byte) til signed int
        int8_t MLSpeed = mLSpeed - 127;
        int8_t MRSpeed = -(mRSpeed - 127); // modsat fortegn, da motor vender modsat vej

        // der lægges/trækkes fart til/fra pga. det er minimumshastighed
        // som robotten vil køre med på bord/gulv. Alt tættere på 0 vil robot ikke køre med
        if (MLSpeed < -25) {
            mL.run(MLSpeed - fart_);
        }
        else if (MLSpeed > 25) {
            mL.run(MLSpeed + fart_);
        }

        if (MRSpeed < -25) {
            mR.run(MRSpeed - fart_);
        }
        else if (MRSpeed > 25) {
            mR.run(MRSpeed + fart_);
        }

        // sender data via Bluetooth til Appen jf. protokoldesignet
        data.TransmitBuffer[0] = 4;
        data.TransmitBuffer[1] = MLSpeed;
        data.TransmitBuffer[2] = MRSpeed;
        data.Send(3);

        delay(vent_);
        // alle motorer stoppes, og det skyldes tilfælde af fejl. Så er man sikker på,
        // at robotten ikke kan gøre noget fuldkommen forkert
        Stop();
    }

    // funktion til at styre armens hastighed
    // saenkArm = hvis den er lig 0, så skal armen hæves, hvis den er lig 1 skal armen sænkes
    void Swing(byte saenkArm) {
        if (saenkArm == 0) {
            mArm.run(fart_);
        }
        else if (saenkArm == 1) {
            mArm.run(-fart_);
        }
        delay(vent_);
        Stop();
    }

    // funktion til at styre kloens hastighed
    // lukClamp = hvis den er lig 0, så skal kloen åbnes, hvis den er lig 1 skal kloen lukkes
    void Squash(byte lukClamp) {
        if (lukClamp == 0) {
            mClamp.run(-fart_);
        }
        else if (lukClamp == 1) {
            mClamp.run(fart_);
        }
        delay(vent_);
        Stop();
    }
};

//class, som styrer decodermotorer på larvefødder
class eFremdrift {
private:
    float length_;      // afstand man ønsker at køre i cm
    int8_t motorSpeed_; // antal RPM (rounds per minute) motoren drejer rundt med

    // bruges til at omregne fra ticks (vinkel) til centimeter
    float CalculateDistance() { return 2.2 * 360 * length_ / (6.3 * PI); }

    // bruges til at beregne tiden, som enconderen bruger på at køre bestemt antal ticks
    float CalculateTime() { return 136.5 * length_ / motorSpeed_; }
public:

    void static isr_process_encoder1(void)
    {
        if (digitalRead(EmL.getPortB()) == 0)
        {
            EmL.pulsePosMinus();
        }
        else
        {
            EmL.pulsePosPlus();
        }
    }

    void static isr_process_encoder2(void)
    {
        if (digitalRead(EmR.getPortB()) == 0)
        {
            EmR.pulsePosMinus();
        }
        else
        {
            EmR.pulsePosPlus();
        }
    }

    // kørefunktion med encodere
    // length = længde man ønsker at køre i cm
    // motorSpeed = hvor hurtigt man ønsker motoren at køre, målt i RPM
    void Drive(float length, int8_t motorSpeed) {
        EmL.reset(SLOT_1);
        EmR.reset(SLOT_2);

        length_ = length;
        motorSpeed_ = motorSpeed;

        Serial.print("Driving: ");
        Serial.print(length_);
        Serial.println(" centimeters.");

        EmL.moveTo(CalculateDistance(), motorSpeed_);
        EmR.moveTo(-CalculateDistance(), motorSpeed_);

        for (int i = 0; i < CalculateTime(); i++) {
            EmL.loop();
            EmR.loop();
            delay(100);
        }
    }
};

// class med funktioner omhandlende decodermotor til armen
class eArm {

};

DataTransfer data;      // class med dataoverførselsfunktioner
Sensor sensor;          // bruges til at tilgå funktioner vedrørende øvrige sensorer på robotten
ManualControl manuel;   // class med kørefunktioner til larvefødderne, armen og kloen UDEN encoder
eFremdrift fremdrift;   // class med funktioner til encodermotor på larvefødder
eArm arm;               // class med funktioner til encoderarmen

void CallState() {
    Serial.print("Changing state to: ");
    Serial.print(aState);
    Serial.println(".");
}

void AutomaticMode() {
    while (true) {
        switch (aState) {
        case OPGAVE_1:
            CallState();

            fremdrift.Drive(50, 100);
            sensor.Turn(90, 'R');
            sensor.Turn(90, 'L');
            fremdrift.Drive(-50, 100);

            aState = OPGAVE_2;
            break;
        case OPGAVE_2:
            CallState();

            aState = OPGAVE_3;
            break;
        case OPGAVE_3:
            CallState();

            aState = OPGAVE_4;
            break;
        case OPGAVE_4:
            CallState();

            aState = OPGAVE_5;
            break;
        case OPGAVE_5:
            CallState();

            aState = OPGAVE_6;
            break;
        case OPGAVE_6:
            CallState();

            aState = NOTHING;
            break;
        }
        if (aState == NOTHING) {
            Serial.println("Exiting automatic mode.");
            aState = OPGAVE_1;
            break;
        }
        if (Serial3.read() == 3) {
            Serial.println("Exiting automatic mode.");
            break;
        }
        delay(100);
    }
}

// bruges til at "læse" data fra appen og gøre bestemte ting. Tilhører
// DataTransfer-classen
void DataTransfer::Decode() {
    // tager udgangspunkt i vores protokol
    switch (ReceiveDataBuffer[0])
    {
    case 0:
        manuel.Drive(ReceiveDataBuffer[1], ReceiveDataBuffer[2]);
        break;
    case 1:
        manuel.Squash(ReceiveDataBuffer[1]);
        break;
    case 2:
        manuel.Swing(ReceiveDataBuffer[1]);
        break;
    case 3:
        AutomaticMode();
        break;
    }
}

// bruges til at sende data til appen
// nofData = antallet af data, der sendes
void DataTransfer::Send(uint8_t nofData) {
    Serial.print("Sending: ");
    Serial.print(nofData);
    Serial.println(" bytes.");

    for (int i = 0; i < nofData; i++) {
        Serial3.write(TransmitBuffer[i]);
    }
    delay(1);
}

#pragma endregion

#pragma region Setup()- and Loop()-functions

// The setup() function runs once each time the micro-controller starts
void setup() {
    aState = OPGAVE_1;
    Gyro.begin();

    Serial.begin(115200);
    Serial3.begin(115200);
    
    // alt nedenstående er til encodere
    attachInterrupt(EmL.getIntNum(), eFremdrift::isr_process_encoder1, RISING);
    attachInterrupt(EmR.getIntNum(), eFremdrift::isr_process_encoder2, RISING);

    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(CS11) | _BV(WGM12);

    TCCR2A = _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS21);

    EmL.setPulse(7);
    EmR.setPulse(7);

    EmL.setRatio(26.9);
    EmR.setRatio(26.9);

    EmL.setPosPid(1.8, 0, 1.2);
    EmR.setPosPid(1.8, 0, 1.2);

    EmL.setSpeedPid(0.18, 0, 0);
    EmR.setSpeedPid(0.18, 0, 0);
}

// Add the main program code into the continuous loop() function
void loop() {
    // hvis der ikke modtages noget data, så bare delay, så MegaPi'en ikke ristes
    if (Serial3.available() == int(0)) {
        delay(100);
    }
    else {
        data.isReceived();
    }
}

#pragma endregion
