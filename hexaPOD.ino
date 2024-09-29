#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
#include <SoftwareSerial.h>

//=====================================================================
//                               bluetooth
//=====================================================================
SoftwareSerial miBT(10, 11);

char DATO = 0;
bool comandos = false;
char clave = 'z';

String command_servo;
String command_pMin;
String command_pMax;
String command_angulo;

int indice = 0;
int cont = 0;
int len = 40;

String array_comandos[40];
//=====================================================================
//                                Servo.h
//=====================================================================
Servo servo_one;
Servo servo_two;
//=====================================================================
//                      Adafruit_PWMServoDriver
//=====================================================================
Adafruit_PWMServoDriver servo_placa = Adafruit_PWMServoDriver(0x40);

int duty1;
//=====================================================================
//                      Pos Iniciales  =1=2=3>
//=====================================================================
int velocidad = 50;
int salto = 5;

int angulo_mov = 30;
int angulo_inclinacion = 20;
int angulo_3 = 20;
int angulo_apertura = 30;
int ini_noventa = 90;


int matrisServoControl[][6] = {
  //pMin, pMax, incial position, angulo Min, angulo Max
  { 80, 525, ini_noventa - angulo_apertura, 0, 120 },
  { 120, 600, ini_noventa + angulo_inclinacion, 60, 180 },
  { 120, 600, ini_noventa + angulo_inclinacion, 0, 160 },

  { 140, 600, ini_noventa, 70, 110 },
  { 80, 580, ini_noventa + angulo_inclinacion, 70, 180 },
  { 90, 620, ini_noventa + angulo_3, 0, 150 },

  { 125, 620, ini_noventa + angulo_apertura, 70, 110 },
  { 155, 625, ini_noventa + angulo_inclinacion, 70, 180 },
  { 125, 620, ini_noventa + angulo_inclinacion, 0, 150 },

  { 150, 640, ini_noventa - angulo_apertura, 70, 110 },
  { 140, 620, ini_noventa - angulo_inclinacion, 70, 180 },
  { 150, 640, ini_noventa - angulo_inclinacion, 0, 150 },

  { 100, 535, ini_noventa, 70, 110 },
  { 120, 620, ini_noventa - angulo_inclinacion, 70, 180 },
  { 75, 565, ini_noventa - angulo_inclinacion, 0, 150 },

  { 110, 580, ini_noventa, 70, 110 },
  { 450, 6750, ini_noventa, 70, 180 },
  { 590, 6650, ini_noventa, 0, 150 }
};

int vect_triangulo_b[] = { 3, 4, 5, 9, 10, 11, 15, 16, 17 };
int vect_triangulo_a[] = { 0, 1, 2, 6, 7, 8, 12, 13, 14 };

//--bool--
bool desplazamiento = false;

void setup() {
  //-----------------------------bluetooth----------------------------
  miBT.begin(38400);
  //--------------------------MonitorSerial---------------------------
  Serial.begin(9600);
  //--------------------------PWMServoDriver--------------------------
  servo_placa.begin();
  servo_placa.setPWMFreq(60);  //Frecuencia PWM de 60Hz o T=16,66ms

  servo_one.attach(8, 450, 6750);
  servo_two.attach(9, 590, 6650);

  //-------------------------inicial position-------------------------

  //InicialPosition();

  //------------------------------demora------------------------------
  delay(1000);
}
void InicialPosition() {
  //--------one----------
  /* SetIncialPosition(0);
  SetIncialPosition(1);
  SetIncialPosition(2);
  //--------two----------
  SetIncialPosition(3);
  SetIncialPosition(4);
  SetIncialPosition(5);
  //--------three---------
  SetIncialPosition(6);
  SetIncialPosition(7);
  SetIncialPosition(8);
  //--------four----------
  SetIncialPosition(9);
  SetIncialPosition(10);
  SetIncialPosition(11);
  //--------five----------
  SetIncialPosition(12);
  SetIncialPosition(13);
  SetIncialPosition(14);*/
  //--------six----------
}

void SetIncialPosition(uint8_t servo) {
  duty1 = map(matrisServoControl[servo][2], 0, 180, matrisServoControl[servo][0], matrisServoControl[servo][1]);
  servo_placa.setPWM(servo, 0, duty1);
}

void SetServoMovGeneral(uint8_t servo, double angulo) {
  duty1 = map(angulo, 0, 180, matrisServoControl[servo][0], matrisServoControl[servo][1]);
  servo_placa.setPWM(servo, 0, duty1);
}

void SetCalibrateServo(uint8_t n_servo1, int angulo1, int pMin, int pMax) {
  int duty1;
  //duty1 = map(angulo1, 0, 180, 140, 620);
  if (n_servo1 == 17) {
    servo_one.attach(8, pMin, pMax);
    servo_one.write(angulo1);

  } else if (n_servo1 == 18) {  //590 6650
    servo_two.attach(9, pMin, pMax);
    servo_two.write(angulo1);

  } else {
    duty1 = map(angulo1, 0, 180, pMin, pMax);
    servo_placa.setPWM(n_servo1, 0, duty1);
  }
}

void CalibrarServoMotor() {
  if (DATO == clave) {
    comandos = !comandos;
  }

  if (comandos) {
    if (indice <= len && DATO != clave) {
      Serial.print(String(DATO) + " ");
      array_comandos[indice] = String(DATO);
      indice++;
    }
    delay(10);
  }

  if (!comandos && DATO == clave) {
    for (int i = 0; i <= indice - 1; i++) {
      if (cont == 0) {
        command_servo += array_comandos[i];
      } else if (cont == 1) {
        command_angulo += array_comandos[i];
      } else if (cont == 2) {
        command_pMin += array_comandos[i];
      } else if (cont == 3) {
        command_pMax += array_comandos[i];
      }

      if (array_comandos[i] == "-") {
        cont++;
      }
    }

    Serial.println("");
    Serial.println("servo: " + String(command_servo.toInt()));
    Serial.println("angulo: " + String(command_angulo.toInt()));
    Serial.println("pmin: " + String(command_pMin.toInt()));
    Serial.println("pmax: " + String(command_pMax.toInt()));

    SetCalibrateServo(command_servo.toInt(), command_angulo.toInt(), command_pMin.toInt(), command_pMax.toInt());
    delay(100);

    indice = 0;
    cont = 0;

    command_servo = "";
    command_angulo = "";
    command_pMin = "";
    command_pMax = "";
    delay(100);
  }
}
double ejeX = 119.88;
double ejeY = 36.68;
double ejeZ = 70;

double gamma;

double alfa1;
double alfa2;
double alfa;

double beta;

double hipoXY;
double lineaAlfa;



double COXA = 28;
double FEMUR = 55;
double TIBIA = 80;

double max_percent = 100;  // porcentaje maximo
int patas = 6;             // numero de patas involucradas


// Funciones
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
  double avance_percent = avance;  // salto en porcentaje
  double velocidad = 10;





  while (true) {

    double ProcessPosition[6][3] = {};
    double ProcessPosition2[6][3] = {};

    for (double percent = 1; percent <= max_percent; percent += avance_percent) {  // iteraciones generales hasta que este en 100%
      //-------------------------------------------------------
      //Serial.println(" ");
      //Serial.println("Iteracion al " + String(percent) + "%");
      for (int i = 0; i < patas; i++) {
        for (int j = 0; j < 3; j++) {

          if (j == 2) {  // si estoy en la columna z
            if (percent * 2 > max_percent) {
              ProcessPosition[i][j] = (InicialPosition[i][j] + (FinalPosition[i][j] * (max_percent / 100))) - FinalPosition[i][j] * (((percent - (max_percent / 2)) * 2) / 100);
              //Serial.print(" B2|" + String(ProcessPosition[i][j]) + "| " + String(FinalPosition[i][j] * (((percent - (max_percent / 2)) * 2) / 100)));
            } else {
              ProcessPosition[i][j] = InicialPosition[i][j] + FinalPosition[i][j] * ((percent * 2) / 100);
              //Serial.print(" B1|" + String(ProcessPosition[i][j]) + "| " + String(FinalPosition[i][j] * ((percent * 2) / 100)));
            }
          } else {
            ProcessPosition[i][j] = InicialPosition[i][j] + FinalPosition[i][j] * (percent / 100);
            //Serial.print(" A|" + String(ProcessPosition[i][j]) + "| " + String(FinalPosition[i][j] * (percent / 100)));
          }
          //delay(1);
        }
        //Serial.println("");
      }

      //-------------------------------------------------------
      //Serial.println(" ");
      //Serial.println("Positions");
      for (int pos = 0; pos < patas; pos++) {

        //Serial.println("x: " + String(ProcessPosition[pos][0]) + " y: " + String(ProcessPosition[pos][1]) + " z: " + String(ProcessPosition[pos][2]));
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



    for (double percent = 1; percent <= max_percent; percent += avance_percent) {  // iteraciones generales hasta que este en 100%
      //-------------------------------------------------------
      //Serial.println(" ");
      //Serial.println("Iteracion al " + String(percent) + "%");
      for (int i = 0; i < patas; i++) {
        for (int j = 0; j < 3; j++) {

          if (j == 2) {  // si estoy en la columna z
            if (percent * 2 > max_percent) {
              ProcessPosition2[i][j] = (ProcessPosition[i][j] + (FinalPosition2[i][j] * (max_percent / 100))) - FinalPosition2[i][j] * (((percent - (max_percent / 2)) * 2) / 100);
              //Serial.print(" B2|" + String(ProcessPosition[i][j]) + "| " + String(FinalPosition[i][j] * (((percent - (max_percent / 2)) * 2) / 100)));
            } else {
              ProcessPosition2[i][j] = ProcessPosition[i][j] + FinalPosition2[i][j] * ((percent * 2) / 100);
              //Serial.print(" B1|" + String(ProcessPosition[i][j]) + "| " + String(FinalPosition[i][j] * ((percent * 2) / 100)));
            }
          } else {
            ProcessPosition2[i][j] = ProcessPosition[i][j] + FinalPosition2[i][j] * (percent / 100);
            //Serial.print(" A|" + String(ProcessPosition[i][j]) + "| " + String(FinalPosition[i][j] * (percent / 100)));
          }
          //delay(1);
        }
        //Serial.println("");
      }

      //-------------------------------------------------------
      //Serial.println(" ");
      //Serial.println("Positions");
      for (int pos = 0; pos < patas; pos++) {
        //Serial.println("x: " + String(ProcessPosition[pos][0]) + " y: " + String(ProcessPosition[pos][1]) + " z: " + String(ProcessPosition[pos][2]));
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
      //DATO = miBT.read();

      break;
    }
  }
  /*
  Serial.println(" ");
  Serial.println("Matriz Final A");
  for (int i = 0; i < patas; i++) {
    for (int j = 0; j < 3; j++) {
      Serial.print(" |" + String(ProcessPosition[i][j]) + "| ");
      delay(100);
    }
    Serial.println("");
  }

  Serial.println(" ");
  Serial.println("Matriz Final B");
  for (int m = 0; m < patas; m++) {
    for (int n = 0; n < 3; n++) {
      Serial.print(" |" + String(ProcessPosition2[m][n]) + "| ");
      delay(100);
    }
    Serial.println("");
  }*/

  /*for (int a = 0; a < 2; a++) {  //itera por los moviemintos finales

    double media = FinalPosition[a][1] / 2;
    double porcentaje;

    for (int y = 0; y >= FinalPosition[a][1]; y -= salto) {  //itera por cada punto final en el eje Y

      ///Serial.println("-------------------- CINEMATIC ---------------------");
      //itera por las posiciones iniciales (repetido)

      //====================== Variacion X Y Z ===================================

      ejeX = InicialPosition[a][0] + FinalPosition[a][0];
      ejeY = InicialPosition[a][1] + y;

      if (y >= media) {

        porcentaje = y / media;
        ejeZ = InicialPosition[a][2] + (porcentaje * FinalPosition[a][2]);
        //Serial.println("Z = " + String(ejeZ));
      } else {
        //Serial.println("Bajada: " + String(y - media));
        porcentaje = (y - media) / media;
        ejeZ = (InicialPosition[a][2] + FinalPosition[a][2]) - (porcentaje * FinalPosition[a][2]);
        // Serial.println("Z00000 = " + String(ejeZ));
      }

      //====================== Calculos ===================================

      hipoXY = HipoXY(ejeX, ejeY);
      lineaAlfa = LineaA(ejeZ, hipoXY, COXA);
      gamma = Gamma(ejeY, ejeX);
      alfa = Alfa(ejeZ, lineaAlfa, TIBIA, FEMUR);
      beta = Beta(lineaAlfa, TIBIA, FEMUR);

      //====================== Movimientos ===================================
      for (int j = a * 3; j < a * 3 + 1; j++) {  //itera por cada servo; desde el 0 hasta el 17 segun la iteracion "a"
        if (j + 2 <= 8) {
          SetServoMovGeneral(j, abs(90 - gamma));
          SetServoMovGeneral(j + 1, alfa);
          SetServoMovGeneral(j + 2, abs(beta - 180));
        } else if (j + 2 >= 16) {
          SetServoMovGeneral(j, abs(90 - gamma));
          servo_one.write(abs(180 - alfa));
          servo_two.write(abs(beta));
        } else {
          SetServoMovGeneral(j, abs(90 - gamma));
          SetServoMovGeneral(j + 1, abs(180 - alfa));
          SetServoMovGeneral(j + 2, abs(beta));
        }
      }

      //Serial.println("[" + String(i + 1) + "]" + "  ~Gamma: " + String(gamma) + "   ~Alfa: " + String(alfa) + "   ~Beta: " + String(beta));
      delay(10);

      //delay(2);
    }

    for (int y = 0; y <= InicialPosition[0][1]; y += salto) {

      //Serial.println("-------------------- CINEMATIC ---------------------");
      for (int i = 0; i < 1; i++) {

        ejeX = InicialPosition[i][0];
        ejeY = (InicialPosition[i][1] + FinalPosition[a][1]) + y;
        ejeZ = InicialPosition[i][2];

        hipoXY = HipoXY(ejeX, ejeY);
        lineaAlfa = LineaA(ejeZ, hipoXY, COXA);
        gamma = Gamma(ejeY, ejeX);
        alfa = Alfa(ejeZ, lineaAlfa, TIBIA, FEMUR);
        beta = Beta(lineaAlfa, TIBIA, FEMUR);

        for (int j = i * 3; j < i * 3 + 1; j++) {
          if (j + 2 <= 8) {
            SetServoMovGeneral(j, abs(90 - gamma));
            SetServoMovGeneral(j + 1, alfa);
            SetServoMovGeneral(j + 2, abs(beta - 180));
          } else if (j + 2 >= 16) {
            SetServoMovGeneral(j, abs(90 - gamma));
            servo_one.write(abs(180 - alfa));
            servo_two.write(abs(beta));
          } else {
            SetServoMovGeneral(j, abs(90 - gamma));
            SetServoMovGeneral(j + 1, abs(180 - alfa));
            SetServoMovGeneral(j + 2, abs(beta));
          }
        }

        //Serial.println("[" + String(i + 1) + "]" + "  ~Gamma: " + String(gamma) + "   ~Alfa: " + String(alfa) + "   ~Beta: " + String(beta));
        delay(2);
      }
      //delay(2);
    }
  }*/
}

double Z;
double Y;
double Z2;
double Y2;
void movUP() {
  Z = 90;
  Y = 30;
  Z2 = 50;
  Y2 = 50;

  double InicialPosition[][3] = {
    //x   y     z
    { 60, Y2, Z },
    { 85, 0, Z },
    { 70, -Y2, Z },

    { 70, Y2, Z },
    { 85, 0, Z },
    { 60, -30, Z }
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
  //TestMov();
  CinematicaInversa(8, InicialPosition, FinalPosition, FinalPosition2);
}

void movDW() {
  Z = 90;
  Y = 30;
  Z2 = 50;
  Y2 = 50;

  double retro[][3] = {
    //x   y     z
    { 60, Y2, Z },
    { 85, 0, Z },
    { 70, -Y2, Z },

    { 70, Y2, Z },
    { 85, 0, Z },
    { 60, -Y2, Z }
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
  Z = 80;
  Y = 30;
  Z2 = 50;
  Y2 = 50;

  double left[][3] = {
    //x   y     z
    { 60, Y2, Z },
    { 85, 0, Z },
    { 70, -Y2, Z },

    { 70, Y2, Z },
    { 85, 0, Z },
    { 60, -Y2, Z }
  };

  double fleft1[][3] = {
    //x   y    z                  No es una posicion final, es el avance que va tener.
    { 0, -Y, -Z2 },
    { 0, Y, 0 },
    { 0, -Y, -Z2 },

    { 0, Y, 0 },
    { 0, -Y, -Z2 },
    { 0, Y, 0 }
  };

  double fleft2[][3] = {
    //x   y    z                  No es una posicion final, es el avance que va tener.
    { 0, Y, 0 },
    { 0, -Y, -Z2 },
    { 0, Y, 0 },

    { 0, -Y, -Z2 },
    { 0, Y, 0 },
    { 0, -Y, -Z2 }
  };

  CinematicaInversa(4, left, fleft1, fleft2);
}
void movRG() {
  Z = 80;
  Y = 30;
  Z2 = 50;
  Y2 = 50;

  double right[][3] = {
    //x   y     z
    { 60, Y2, Z },
    { 85, 0, Z },
    { 70, -Y2, Z },

    { 70, Y2, Z },
    { 85, 0, Z },
    { 60, -Y2, Z }
  };

  double fright1[][3] = {
    //x   y    z                  No es una posicion final, es el avance que va tener.
    { 0, Y, -Z2 },
    { 0, -Y, 0 },
    { 0, Y, -Z2 },

    { 0, -Y, 0 },
    { 0, Y, -Z2 },
    { 0, -Y, 0 }
  };

  double fright2[][3] = {
    //x   y    z                  No es una posicion final, es el avance que va tener.
    { 0, -Y, 0 },
    { 0, Y, -Z2 },
    { 0, -Y, 0 },

    { 0, Y, -Z2 },
    { 0, -Y, 0 },
    { 0, Y, -Z2 }
  };

  CinematicaInversa(4, right, fright1, fright2);
}
void loop() {
  if (miBT.available()) {
    DATO = miBT.read();
    
    Serial.println(DATO);
    CalibrarServoMotor();

    switch (DATO) {
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
