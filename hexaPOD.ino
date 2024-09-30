#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial miBT(10, 11);

// var control
char DATO = 0;

bool calibrate = false;

String array_paramiters[6];

char clave = 'z';
String paramiter;

bool speed_control= false;
bool sleep_hex = false;

// Calibracion
int indice = 0;

// Servo 17-18
Servo servo_one;
Servo servo_two;

// Adafruit_PWMServoDriver
Adafruit_PWMServoDriver servo_placa = Adafruit_PWMServoDriver(0x40);

int duty1;

// { PMIN, PMAX }
int matrizPulsos[18][2] = {
  { 80, 525 }, 
  { 120, 600 },
  { 120, 600 },

  { 140, 600 },
  { 80, 580 },
  { 90, 620 },

  { 125, 620 },
  { 155, 625 },
  { 125, 620 },

  { 150, 640 },
  { 140, 620 },
  { 150, 640 },

  { 80, 680 },
  { 120, 620 },
  { 75, 565 },

  { 140, 620 },
  { 450, 6750 },
  { 590, 6650 }
};

// Dimension
double ejeX = 119.88;
double ejeY = 36.68;
double ejeZ = 70;

// Angulos
double gamma;

double alfa;
double alfa1;
double alfa2;

double beta;

double hipoXY;
double lineaAlfa;

// Medidas
double COXA = 28;
double FEMUR = 55;
double TIBIA = 80;

// Control Movimiento
double max_percent = 100;  // porcentaje maximo
int patas = 6;             // numero de patas involucradas
double avance_general = 5; // avance por grados - menos pierde velocidad (7)

// Distancia de movimientos
double altura = 75;
double distancia = 18;
int velocidad = 5;

double desZ = 40;
double desY = 40;

double dis_x = 70;

double X = distancia;
double Z;
double Y;
double Z2;
double Y2;

void setup() {
  // Bluetooth
  miBT.begin(38400);

  // Monitor Serial
  Serial.begin(9600);

  // PWMServoDriver
  servo_placa.begin();
  servo_placa.setPWMFreq(60);  // Frecuencia PWM de 60Hz o T=16,66ms

  // Inicialization of two servos free
  servo_one.attach(8, matrizPulsos[16][0], matrizPulsos[16][1]);
  servo_two.attach(9, matrizPulsos[17][0], matrizPulsos[17][1]);

  delay(1000);
}

void SetServoMovGeneral(uint8_t servo, double angulo) {
  duty1 = map(angulo, 0, 180, matrizPulsos[servo][0], matrizPulsos[servo][1]);
  servo_placa.setPWM(servo, 0, duty1);
}

void SetCalibrateServo(uint8_t n_servo1, int angulo1, int pMin, int pMax) {
  //duty1 = map(angulo1, 0, 180, 140, 620);

  // Servo 17 y 18 no se incluye en modulo PWM servo driver
  if (n_servo1 == 17) {
    servo_one.attach(8, pMin, pMax);
    servo_one.write(angulo1);
  } else if (n_servo1 == 18) {  //590 6650
    servo_two.attach(9, pMin, pMax);
    servo_two.write(angulo1);
  // Servos que si cuentan con PWM servo driver
  } else {
    duty1 = map(angulo1, 0, 180, pMin, pMax);
    servo_placa.setPWM(n_servo1, 0, duty1);
  }
}

void ServoCalibrate() {
  calibrate = (DATO == clave) ? !calibrate : calibrate;

  if (calibrate) {
      if (DATO != '-') {
        paramiter += String(DATO);
      }else {
        array_paramiters[indice] = paramiter;
        paramiter = "";
        indice++;
      }
  }
  delay(10);
  if (!calibrate && indice > 0) {
    Serial.println("");
    Serial.println("[+] SERVO: " + String(array_paramiters[1]));
    Serial.println("[+] ANGULO: " + String(array_paramiters[2]));
    Serial.println("[+] PMIN: " + String(array_paramiters[3]));
    Serial.println("[+] PMAX: " + String(array_paramiters[4]));
  
    SetCalibrateServo(array_paramiters[1].toInt(), array_paramiters[2].toInt(), array_paramiters[3].toInt(), array_paramiters[4].toInt());
    delay(100);

    indice = 0;
  }
}

// Funciones para hacer los calculos
double HipoXY(double ejeX, double ejeY) {
  return sqrt(pow(ejeX, 2) + pow(ejeY, 2));
}

double LineaA(double ejeZ, double hipotenusaXY) {
  return sqrt(pow(ejeZ, 2) + pow(hipotenusaXY - COXA, 2));
}

double Gamma(double ejeY, double ejeX) {
  return atan(ejeY / ejeX) * RAD_TO_DEG;
}

double Alfa(double ejeZ, double lineaAlfa) {
  alfa1 = acos(ejeZ / lineaAlfa) * RAD_TO_DEG;
  alfa2 = acos((pow(TIBIA, 2) - pow(FEMUR, 2) - pow(lineaAlfa, 2)) / (-2 * FEMUR * lineaAlfa)) * RAD_TO_DEG;
  return alfa1 + alfa2;
}

double Beta(double lineaAlfa) {
  return acos((pow(lineaAlfa, 2) - pow(TIBIA, 2) - pow(FEMUR, 2)) / (-2 * TIBIA * FEMUR)) * RAD_TO_DEG;
}

void CinematicaInversa(double avance, double InicialPosition[][3], double FinalPosition[][3], double FinalPosition2[][3]) {
  double ProcessPosition[6][3] = {};
  double ProcessPosition2[6][3] = {};

  while (true) {

    for (double percent = 1; percent <= max_percent; percent += avance_general) {  // iteraciones generales hasta que este en 100%
      //-------------------------------------------------------
      for (int i = 0; i < patas; i++) {
        for (int j = 0; j < 3; j++) {
          if (j == 2) {  // si estoy en la columna z
            if (percent * 2 > max_percent) {
              ProcessPosition[i][j] = (InicialPosition[i][j] + (FinalPosition[i][j] * (max_percent / 100))) - FinalPosition[i][j] * (((percent - (max_percent / 2)) * 2) / 100);
            } else {
              ProcessPosition[i][j] = InicialPosition[i][j] + FinalPosition[i][j] * ((percent * 2) / 100);
            }
          } else {
            ProcessPosition[i][j] = InicialPosition[i][j] + FinalPosition[i][j] * (percent / 100);
          }
        }
      }

      //-------------------------------------------------------
      for (int pos = 0; pos < patas; pos++) {
        //-----------------------------
        ejeX = ProcessPosition[pos][0];
        ejeY = ProcessPosition[pos][1];
        ejeZ = ProcessPosition[pos][2];
        //-----------------------------
        hipoXY = HipoXY(ejeX, ejeY);
        lineaAlfa = LineaA(ejeZ, hipoXY);
        gamma = Gamma(ejeY, ejeX);
        alfa = Alfa(ejeZ, lineaAlfa);
        beta = Beta(lineaAlfa);
        //-----------------------------
        for (int srv = pos * 3; srv < pos * 3 + 1; srv++) {  //itera por cada servo; desde el 0 hasta el 17 segun la iteracion "pos"
          if (srv + 2 <= 8) {
            SetServoMovGeneral(srv, abs(90 - gamma));
            SetServoMovGeneral(srv + 1, alfa);
            SetServoMovGeneral(srv + 2, abs(beta - 180));
          } else if (srv + 2 >= 16) {
            SetServoMovGeneral(srv, abs(90 - gamma));
            servo_one.write(abs(180 - alfa));
            servo_two.write(abs(beta));
          } else {
            SetServoMovGeneral(srv, abs(90 - gamma));
            SetServoMovGeneral(srv + 1, abs(180 - alfa));
            SetServoMovGeneral(srv + 2, abs(beta));
          }
        }
      }
      delay(velocidad);
      //-------------------------------------------------------
    }

    //-------------------------------------------------------------------------------------------------------------------------------

    for (double percent = 1; percent <= max_percent; percent += avance_general) {  // iteraciones generales hasta que este en 100%
      //-------------------------------------------------------
      for (int i = 0; i < patas; i++) {
        for (int j = 0; j < 3; j++) {

          if (j == 2) {  // si estoy en la columna z
            if (percent * 2 > max_percent) {
              ProcessPosition2[i][j] = (ProcessPosition[i][j] + (FinalPosition2[i][j] * (max_percent / 100))) - FinalPosition2[i][j] * (((percent - (max_percent / 2)) * 2) / 100);
            } else {
              ProcessPosition2[i][j] = ProcessPosition[i][j] + FinalPosition2[i][j] * ((percent * 2) / 100);
            }
          } else {
            ProcessPosition2[i][j] = ProcessPosition[i][j] + FinalPosition2[i][j] * (percent / 100);
          }
        }
      }

      //-------------------------------------------------------
      for (int pos = 0; pos < patas; pos++) {
        //-----------------------------
        ejeX = ProcessPosition2[pos][0];
        ejeY = ProcessPosition2[pos][1];
        ejeZ = ProcessPosition2[pos][2];
        //-----------------------------
        hipoXY = HipoXY(ejeX, ejeY);
        lineaAlfa = LineaA(ejeZ, hipoXY);
        gamma = Gamma(ejeY, ejeX);
        alfa = Alfa(ejeZ, lineaAlfa);
        beta = Beta(lineaAlfa);
        //-----------------------------
        for (int srv = pos * 3; srv < pos * 3 + 1; srv++) {  //itera por cada servo; desde el 0 hasta el 17 segun la iteracion "pos"
          if (srv + 2 <= 8) {
            SetServoMovGeneral(srv, abs(90 - gamma));
            SetServoMovGeneral(srv + 1, alfa);
            SetServoMovGeneral(srv + 2, abs(beta - 180));
          } else if (srv + 2 >= 16) {
            SetServoMovGeneral(srv, abs(90 - gamma));
            servo_one.write(abs(180 - alfa));
            servo_two.write(abs(beta));
          } else {
            SetServoMovGeneral(srv, abs(90 - gamma));
            SetServoMovGeneral(srv + 1, abs(180 - alfa));
            SetServoMovGeneral(srv + 2, abs(beta));
          }
        }
      }
      delay(velocidad);
      //-------------------------------------------------------
    }

    if (miBT.available()) {
      break;
    }
  }
}

void BasicTraslation(double Inicial[][3], double Traslation[][3], bool hex_status) {
  avance_general = 3;

  // matrices temporales
  double ProcessPosition[6][3] = {};
  bool tmp = false;

  if (hex_status) {
    sleep_hex = true;
  }else if (!hex_status) {
    sleep_hex = false;
  }else {
    sleep_hex = !sleep_hex;
  }
  
  for (double percent = 1; percent <= max_percent; percent += avance_general) {  // iteraciones generales hasta que este en 100%
      //-------------------------------------------------------
      if (sleep_hex) {
        for (int i = 0; i < patas; i++) {
          for (int j = 0; j < 3; j++) {     
              ProcessPosition[i][j] = Inicial[i][j] + Traslation[i][j] * ((percent * 2) / 100);
             // Serial.print(String(ProcessPosition[i][j]) + " - ");
          }
         // Serial.println("");
        }
        tmp = false;
      }else {
        //Serial.print("000000000000000000000\n");
        for (int i = 0; i < patas; i++) {
          for (int j = 0; j < 3; j++) {
              ProcessPosition[i][j] = (Inicial[i][j] + Traslation[i][j]) - Traslation[i][j] * (((percent - (max_percent / 2)) * 2) / 100);
              //Serial.print(String(ProcessPosition[i][j]) + " - ");
              //ProcessPosition[i][j] = Inicial[i][j] + Traslation[i][j] * ((percent * 2) / 100);
          }
          tmp = true;
          //Serial.println("");
        }
      }
      //-------------------------------------------------------
      
      for (int pos = 0; pos < patas; pos++) {
        //-----------------------------
        ejeX = ProcessPosition[pos][0];
        ejeY = ProcessPosition[pos][1];
        ejeZ = ProcessPosition[pos][2];
        //-----------------------------
        hipoXY = HipoXY(ejeX, ejeY);
        lineaAlfa = LineaA(ejeZ, hipoXY);
        gamma = Gamma(ejeY, ejeX);
        alfa = Alfa(ejeZ, lineaAlfa);
        beta = Beta(lineaAlfa);
        //-----------------------------
        for (int srv = pos * 3; srv < pos * 3 + 1; srv++) {  //itera por cada servo; desde el 0 hasta el 17 segun la iteracion "pos"
          if (srv + 2 <= 8) {
            SetServoMovGeneral(srv, abs(90 - gamma));
            SetServoMovGeneral(srv + 1, alfa);
            SetServoMovGeneral(srv + 2, abs(beta - 180));
          } else if (srv + 2 >= 16) {
            SetServoMovGeneral(srv, abs(90 - gamma));
            servo_one.write(abs(180 - alfa));
            servo_two.write(abs(beta));
          } else {
            SetServoMovGeneral(srv, abs(90 - gamma));
            SetServoMovGeneral(srv + 1, abs(180 - alfa));
            SetServoMovGeneral(srv + 2, abs(beta));
          }
        }
      }
      delay(velocidad);
      //-------------------------------------------------------
  }
  sleep_hex = tmp;
}

void movUP() {
  Z = altura;
  Y = distancia;

  Z2 = desZ;
  Y2 = desY;
  int x_des = 0;
  

  double InicialPosition[][3] = {
    //x   y     z
    { dis_x, Y2, Z },
    { 85, 0, Z },
    { dis_x, -Y2, Z },

    { dis_x, Y2, Z },
    { 85, 0, Z },
    { dis_x, -Y2, Z }
  };

  double FinalPosition[][3] = {
    //x   y    z                  No es una posicion final, es el avance que va tener.
    { 0, -Y, -Z2 },
    { 0, Y, 0 },
    { 0, -Y, -Z2 },

    { 0, -Y, 0 },
    { 0, Y, -Z2 },
    { 0, -Y, 0 }
  };

  double FinalPosition2[][3] = {
    //x   y    z                  No es una posicion final, es el avance que va tener.
    { 0, Y, 0 },
    { 0, -Y, -Z2 },
    { 0, Y, 0 },

    { 0, Y, -Z2 },
    { 0, -Y, 0 },
    { 0, Y, -Z2 }
  };

  CinematicaInversa(8, InicialPosition, FinalPosition, FinalPosition2);
}

void movDW() {
  Z = altura;
  Y = distancia;
  Z2 = desZ;
  Y2 = desY;

  double retro[][3] = {
    //x   y     z
    { dis_x, Y2, Z },
    { 85, 0, Z },
    { dis_x, -Y2, Z },

    { dis_x, Y2, Z },
    { 85, 0, Z },
    { dis_x, -Y2, Z }
  };

  double fretro[][3] = {
    //x   y    z                  No es una posicion final, es el avance que va tener.
    { 0, Y, -Z2 },
    { 0, -Y, 0 },
    { 0, Y, -Z2 },

    { 0, Y, 0 },
    { 0, -Y, -Z2 },
    { 0, Y, 0 }
  };

  double fretro2[][3] = {
    //x   y    z                  No es una posicion final, es el avance que va tener.
    { 0, -Y, 0 },
    { 0, Y, -Z2 },
    { 0, -Y, 0 },

    { 0, -Y, -Z2 },
    { 0, Y, 0 },
    { 0, -Y, -Z2 }
  };

  CinematicaInversa(6, retro, fretro, fretro2);
}

void movLF() {
  Z = altura;
  Y = distancia-5;
  Z2 = desZ;
  Y2 = desY;

  X = 10;

  double left[][3] = {
    //x   y     z
    { dis_x, Y2, Z },
    { 85, 0, Z },
    { 70, -Y2, Z },

    { dis_x, Y2, Z },
    { 85, 0, Z },
    { dis_x, -Y2, Z }
  };

  double fleft1[][3] = {
    //x   y    z                  No es una posicion final, es el avance que va tener.
    { X, -Y, -Z2 },
    { 0, Y, 0 },
    { -X, -Y, -Z2 },

    { -X, Y, 0 },
    { 0, -Y, -Z2 },
    { X, Y, 0 }
  };

  double fleft2[][3] = {
    //x   y    z                  No es una posicion final, es el avance que va tener.
    { -X, Y, 0 },
    { 0, -Y, -Z2 },
    { X, Y, 0 },

    { X, -Y, -Z2 },
    { 0, Y, 0 },
    { -X, -Y, -Z2 }
  };

  CinematicaInversa(5, left, fleft1, fleft2);
}

void movRG() {
  Z = altura;
  Y = distancia-5;
  Z2 = desZ;
  Y2 = desY;

   X = 10;

  double right[][3] = {
    //x   y     z
    { dis_x, Y2, Z },
    { 85, 0, Z },
    { dis_x, -Y2, Z },

    { dis_x, Y2, Z },
    { 85, 0, Z },
    { dis_x, -Y2, Z }
  };

  double fright1[][3] = {
    //x   y    z                  No es una posicion final, es el avance que va tener.
    { -X, Y, -Z2 },
    { 0, -Y, 0 },
    { X, Y, -Z2 },

    { X, -Y, 0 },
    { 0, Y, -Z2 },
    { -X, -Y, 0 }
  };

  double fright2[][3] = {
    //x   y    z                  No es una posicion final, es el avance que va tener.
    { X, -Y, 0 },
    { 0, Y, -Z2 },
    { -X, -Y, 0 },

    { -X, Y, -Z2 },
    { 0, -Y, 0 },
    { X, Y, -Z2 }
  };

  CinematicaInversa(5, right, fright1, fright2);
}

void CustomA(bool hex_status) {
  Z = altura;
  Y = distancia;

  Z2 = desZ - 15;
  Y2 = desY;

  double InicialPosition[][3] = {
    //x   y     z
    { dis_x, Y2, Z },
    { 85, 0, Z - 4 },
    { dis_x, -Y2, Z },

    { dis_x, Y2, Z },
    { 85, 0, Z - 4 },
    { dis_x, -30, Z }
  };

  double Sleep[][3] = {
    //x   y    z                  No es una posicion final, es el avance que va tener.
    { 0, 0, -Z2 },
    { 0, 0, -Z2},
    { 0, 0, -Z2 },

    { 0, 0, -Z2 },
    { 0, 0, -Z2},
    { 0, 0, -Z2 }
  };

  BasicTraslation(InicialPosition, Sleep, hex_status);
}

void Velocidad() {
  if (DATO == 'v') {
    speed_control = !speed_control;
    return;
  }

  if (speed_control) {
    avance_general = String(DATO).toInt();
    Serial.println("Avance: " + String(avance_general));
    speed_control = false;
  }
}

void loop() {
  if (miBT.available()) {
    DATO = miBT.read();

    //Serial.println(DATO);
    ServoCalibrate();
    Velocidad();

    switch (DATO) {
      case 'i':
        //CustomA(true);
        break;
      case 'u':
        movUP();
        break;
      case 'd':
        movDW();
        break;
      case 'l':
        movLF();
        break;
      case 'r':
        movRG();
        break;
    }
  }
  delay(30);
}
