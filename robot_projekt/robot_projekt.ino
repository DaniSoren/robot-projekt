// pragma bruges til at gøre vores kode mere læsbar og struktureret. Det gør
// så man kan kollapse koden i Visual Studio. Når koden bliver compiled ser compileren
// bort fra pragma ligesom den gør ved kommentarer
#pragma region Libraries, Objects and ENUMs

// bibliotek
#include "MeMegaPi.h"

// Motorer UDEN encoder (bruges til manuel styring)
MeMegaPiDCMotor mL(PORT1B);     // DC-motor til venstre larvefodsmotor
MeMegaPiDCMotor mR(PORT2B);     // DC-motor til højre larvefodsmotor
MeMegaPiDCMotor mArm(PORT3B);   // DC-motor til arm
MeMegaPiDCMotor mClamp(PORT4B); // DC-motor til klo

// Motorer MED encoder (bruges til automatisk program)
MeEncoderOnBoard EmL(SLOT_1);   // Encodermotor til venstre larvefodsmotor
MeEncoderOnBoard EmR(SLOT_2);   // Encodermotor til højre larvefodsmotor
MeEncoderOnBoard EmArm(SLOT_3); // Encodermotor til arm

// De andre sensorer på robotten TODO: ANGIV PORTE
MeUltrasonicSensor USSensor;    // Ultralydssensor
MeGyro Gyro;                    // Gyroskop og accelerometer
MeLineFollower LFollower;       // Line follower

// ENUM så det er nemmere at læse koden vi bruger til automatisk program
enum AUTOSTATE { OPGAVE_1, OPGAVE_2, OPGAVE_3, OPGAVE_4, OPGAVE_5, OPGAVE_6, NOTHING };

AUTOSTATE aState; // vores enum bruges til automatisk program

#pragma endregion

#pragma region Classes and Functions

// siger vi har en function som er uddybbet længere nede
void AutomaticMode();
class DataTransfer;

// class, som har funktioner til robottens sensorer: ultralydssensor, gyro, linefollower
class Sensor {
private:
public:
};

Sensor sensor; // bruges til at tilgå funktioner vedrørende øvrige sensorer på robotten

// class, som har funktionerne, der bruges, når robotten er i manuel tilstand
// dvs. DC-motorene UDEN encoder
class ManualControl {
private:
    uint8_t vent = 200; // bruges når der bruges delay, dvs. vent 200 milisekunder
    uint8_t fart = 150; // angiver hastighed til kloen og armen.

    // funktion til at stoppe motorene
    void Stop() {
        mL.stop();
        mR.stop();
        mArm.stop();
        mClamp.stop();
    }
public:
    // funktion til at styre larveføddernes hastighed
    // mLSpeed = venstre motor hastighed
    // mRSpeed = højre motor hastighed
    void Drive(byte mLSpeed, byte mRSpeed) {

        // konverterer fra unsigned int (byte) til signed int
        int8_t MLSpeed = mLSpeed - 127;
        int8_t MRSpeed = mRSpeed - 127;

        // fordi robotten ikke skal køre motoren hvis den får data fra appen som er tæt på 0.
        // Lægger 150 til eller fra, fordi det er minimum hastighed for at robotten
        // overhovedet vil bevæge sig. MLSpeed og MRSpeed kan højest være på 80 eller -80 pga.
        // appens opbygning.
        if (MLSpeed < -25) {
            mL.run(MLSpeed - 150);
        }
        else if (MLSpeed > 25) {
            mL.run(MLSpeed + 150);
        }

        if (MRSpeed < -25) {
            mL.run(MRSpeed - 150);
        }
        else if (MRSpeed > 25) {
            mL.run(MRSpeed + 150);
        }

        //DataTransfer::TransmitBuffer[0] = 4;
        //DataTransfer::TransmitBuffer[1] = MLSpeed;
        //DataTransfer::TransmitBuffer[2] = MRSpeed;
        //DataTransfer::Send(3);

        delay(vent);
        Stop();
    }

    // funktion til at styre armens hastighed
    // saenkArm = hvis den er lig 0, så skal armen hæves, hvis den er lig 1 skal armen sænkes
    void Swing(byte saenkArm) {
        if (saenkArm == 0) {
            mArm.run(fart);
        }
        else if (saenkArm == 1) {
            mArm.run(-fart);
        }
        delay(vent);
        Stop();
    }

    // funktion til at styre kloens hastighed
    // lukClamp = hvis den er lig 0, så skal kloen åbnes, hvis den er lig 1 skal kloen lukkes
    void Squash(byte lukClamp) {
        if (lukClamp == 0) {
            mClamp.run(fart);
        }
        else if (lukClamp == 1) {
            mClamp.run(-fart);
        }
        delay(vent);
        Stop();
    }
};

ManualControl manuel; // class med kørefunktioner til larvefødderne, armen og kloen UDEN encoder

// class, som har funktioner vedrørende dataoverførsler igennem Bluetooth
class DataTransfer {
private:
    byte ReceiveDataBuffer[16]; // modtage data fra Appen

    // bruges til at "læse" data fra appen og gøre bestemte ting
    // nofData = antallet af fundne data
    void Decode(byte nofData) {
        for (uint8_t i = 0; i < nofData; i++) {
            //TransmitBuffer[i] = ReceiveDataBuffer[i];

            switch (ReceiveDataBuffer[i])
            {
            case 0:
                manuel.Drive(ReceiveDataBuffer[++i], ReceiveDataBuffer[++i]);
                break;
            case 1:
                manuel.Squash(ReceiveDataBuffer[++i]);
                break;
            case 2:
                manuel.Swing(ReceiveDataBuffer[++i]);
                break;
            case 3:
                AutomaticMode();
                break;
            default:
                break;
            }
        }
        //Send(nofData);
    }
public:
    byte static TransmitBuffer[16];    // sende data til Appen

    // funktion der konstant bliver kørt og har mulighed for at modtage data fra appen
    void isReceived() {
        //byte readData = 0; // læser den nuværende data
        byte nof_data = 0; // antallet af fundne data
        uint8_t cnt = 0; // looper igennem hver byte modtaget

        if (Serial3.available() != (int)0)
        {
            Serial.println("Data recieved");

            while (Serial3.available() != (int)0)
            {
                Serial.println(Serial3.available());

                //readData = Serial3.read();
                //ReceiveDataBuffer[cnt] = readData;
                ReceiveDataBuffer[cnt] = Serial3.read();   // hvorfor gør man ikke det her?

                cnt++;
                nof_data++;

                delay(1);
            }

            Decode(nof_data);
        }
    }

    // bruges til at sende data til appen
    // nofData = antallet af data
    //void static Send(int nofData) {
    //    for (int i = 0; i < nofData; i++) {
    //        Serial3.write(TransmitBuffer[i]);
    //    }
    //    delay(1);
    //}
};
DataTransfer data;      // class med dataoverførselsfunktioner

//class, som styrer decodermotorer på larvefødder
class eFremdrift {

};

eFremdrift fremdrift;   // class med funktioner til encodermotor på larvefødder

// class med funktioner omhandlende decodermotor til armen
class eArm {
    // i denne class skal der være funktioner til armen (obviously lol)
};

eArm arm;               // class med funktioner til encoderarmen

void AutomaticMode() {
    while (true) {
        switch (aState) {
        case OPGAVE_1:

            aState = OPGAVE_2;
            break;
        case OPGAVE_2:

            aState = OPGAVE_3;
            break;
        case OPGAVE_3:

            aState = OPGAVE_4;
            break;
        case OPGAVE_4:

            aState = OPGAVE_5;
            break;
        case OPGAVE_5:

            aState = OPGAVE_6;
            break;
        case OPGAVE_6:

            aState = NOTHING;
            break;
        }
        if (aState == NOTHING) {
            aState = OPGAVE_1;
            break;
        }
        if (Serial3.read() == 3)
            break;
    }
}

#pragma endregion

#pragma region Setup()- and Loop()-functions

// The setup() function runs once each time the micro-controller starts
void setup() {
    aState = OPGAVE_1;
    Serial.begin(115200);
    Serial3.begin(115200);
    //Serial3.write(0);
}

// Add the main program code into the continuous loop() function
void loop() {
    data.isReceived();
    delay(100);
}
