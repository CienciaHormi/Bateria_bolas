/* Control de 7 servomotores con MIDI de entrada para dejar caer bolas sobre instrumentos. El golpe es recogido por un piezoeléctrico y devuelto como señal MIDI.
   Programado sobre Teensy 3.6
   Autor: Javier Vargas. El Hormiguero 11/04/2018
   https://creativecommons.org/licenses/by-sa/4.0/
*/

//#define Debug_Retardo //Las bolas caen automaticamente cada 10 segundos

/////////AJUSTES DEL PROGRAMA///////////
#define Nservos 7 //Numero de servos
//Offset de cada servomotor
const int Offset[] = { -24, -37, -12, -21, -59, -40, 1};
const int OffsetG = 0; //Offset global
//Angulo de apertura
const int IncrementoAbierto[] = {8, 6, 8, 9, 8, 9, 10};

//Umbral de sensibilidad del piezoeléctrico
const int RangoPiezo[] = {700, 800, 500, 1000, 500, 500, 500};

//Rango de tension de ON
const int TiempoPiezoOn[] = {100, 100, 120, 100, 100, 100, 100}; //Tiempo para evitar rebotes en la lectura del piezo eléctrico (ms)

//Angulo donde el tubo esta totalmente obstaculizado
const int IncrCerrado = 20; //**Inutilizado

#define AnguloCerrado 90 //Angulo de servo cerrado
#define TiempoPaso 70 //Tiempo que esta el servo abierto (ms)
#define TiempoReposo 10 //Segundos para que los servos entren en roposo **Inutilizado

//Notas de entrada
const int NotaServoIN[Nservos] = {0, 12, 24, 36, 48, 60, 72};
//Notas de salida
const int NotaPiezoOUT[Nservos] = {0, 12, 24, 36, 48, 60, 72};
//Pines de entrada analogica para lecutra de piezoeléctricos
const int pinPiezo[Nservos] = {A20, A9, A8, A7, A6, A1, A0};

//Servo angulo PWM (PCA9685)
#define SERVOMIN  150 // Minima cuenta de PWM (sobre 4096), angulo de servo a 0 grados.
#define SERVOMAX  675 // Maxima cuenta de PWM (sobre 4096), angulo de servo a 180 grados.
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int ledPin = 13;
boolean estadoServo[Nservos]; //1 abierto, 0 cerrado
boolean estadoPiezo[Nservos]; //1 activado, 0 desactivado
unsigned long m[Nservos];
unsigned long mils[Nservos];
unsigned long m0 = 0;
unsigned long m1 = 0;
boolean Cierre = 0;


void setup() {
#ifdef Debug_Retardo
  Serial.begin(115200);
  delay(1000);
  Serial.println("DEBUG RETARDO");
#endif

  pinMode(ledPin, OUTPUT);

  for (int i = 0; i < Nservos; i++) {
    estadoServo[i] = 1; //Inicio de servos en estado abierto
    estadoPiezo[i] = 0; //Piezoeléctricos en desactivados
  }

  usbMIDI.setHandleNoteOn(myNoteOn);
  usbMIDI.setHandleNoteOff(myNoteOff);
  usbMIDI.setHandleControlChange(myControlChange);

  pwm.begin();
  pwm.setPWMFreq(60);  // Frecuencia de servos analógicos
  delay(10);
}

/////////////////////////////////////////
///////////////////LOOP//////////////////
/////////////////////////////////////////

void loop() {
#ifdef  Debug_Retardo
  static unsigned long mm = 0;
  if (mm != millis() / 5000) {
    mm = millis() / 5000;
    NotaServo(0);
    NotaServo(1);
    NotaServo(2);
    NotaServo(3);
    NotaServo(4);
    NotaServo(5);
    NotaServo(6);
    m0 = millis();
    Serial.println("");
  }
#endif
  usbMIDI.read();
  lecturaPiezo();
  MuestreoServos();
  //Reposo();
}

/////////////////////////////////////////
/////////////////////////////////////////
/////////////////////////////////////////

void Reposo() {
  if (millis() > m1 && Cierre == 0) {
    for (int s = 0; s < Nservos; s++) {
      SetServo(s, AnguloCerrado + IncrCerrado); //Angulo cerrado de reposo
    }
    Cierre = 1;
  }
}

void myNoteOn(byte channel, byte note, byte velocity) {
  // When using MIDIx4 or MIDIx16, usbMIDI.getCable() can be used
  // to read which of the virtual MIDI cables received this message.
  digitalWrite(ledPin, HIGH);

  for (int i = 0; i < Nservos; i++) {
    if (note == NotaServoIN[i]) NotaServo(i);
  }

}

void myNoteOff(byte channel, byte note, byte velocity) {
  digitalWrite(ledPin, LOW);
}

void myControlChange(byte channel, byte control, byte value) {

}

void NotaServo(byte s) {
  m[s] = millis() + TiempoPaso; //Tiempo de cierre
  SetServo(s, AnguloCerrado - IncrementoAbierto[s]); //Angulo abierto
  estadoServo[s] = 1; //estadoServo abierto
  m1 = millis() + TiempoReposo * 1000; //Tiempo de reposo
  Cierre = 0;
}

void MuestreoServos() {
  for (int s = 0; s < Nservos; s++) {
    if (estadoServo[s] == 1) { //Servomotor abierto
      if (millis() > m[s]) { //Cierre de los servomotores al pasar el tiempo de abierto
        SetServo(s, AnguloCerrado); //Angulo cerrado
        estadoServo[s] = 0; //estadoServo cerrado
      }
    }
  }
}

//Servo 0 - 180º
void SetServo(byte servo, int angulo) {
  angulo = constrain(angulo, 0, 180);
  pwm.setPWM(servo, 0, map(angulo + Offset[servo] + OffsetG, 0, 180, SERVOMIN, SERVOMAX));
}

void lecturaPiezo() {
  for (int i = 0; i < Nservos; i++) {
    if (estadoPiezo[i] == 0) { //estado desactivado
      if (analogRead(pinPiezo[i]) >= RangoPiezo[i]) { //Si leemos un un flanco de subida
        usbMIDI.sendNoteOn(NotaPiezoOUT[i], 99, 1); //Envío de la nota
        mils[i] = millis() + TiempoPiezoOn[i]; //Actualización del tiempo antirebotes
        estadoPiezo[i] = 1;
        digitalWrite(ledPin, HIGH);
#ifdef  Debug_Retardo
        Serial.print("Nota " );
        Serial.print(i);
        Serial.print(": ");
        Serial.print(millis() - m0);
        Serial.println("ms");
#endif
      }
    }
    else { //estado activado
      if (millis() > mils[i]) { //Tiempo de activado pasado
        usbMIDI.sendNoteOn(NotaPiezoOUT[i], 0, 1); //Nota off
        estadoPiezo[i] = 0;
        digitalWrite(ledPin, LOW);
      }
    }
  }
}




