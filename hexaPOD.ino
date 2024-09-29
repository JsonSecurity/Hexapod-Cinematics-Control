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
//--pata 1 delante--
int ini_one_1 = ini_noventa - angulo_apertura;     //30
int ini_one_2 = ini_noventa + angulo_inclinacion;  //40 60
int ini_one_3 = ini_noventa + angulo_inclinacion;
//--pata 2 medio--


int matrisServoControl[][6] = {
  //pMin, pMax, incial position, angulo Min, angulo Max
  { 80, 525, ini_noventa - angulo_apertura, 0, 120 },
  { 120, 600, ini_noventa + angulo_inclinacion, 60, 180 },
  { 120, 600, ini_noventa + angulo_inclinacion, 0, 160 },

  { 140, 600, ini_noventa, 70, 110 },
  { 50, 620, ini_noventa + angulo_inclinacion, 70, 180 },
  { 90, 620, ini_noventa + angulo_3, 0, 150 },

  { 125, 620, ini_noventa + angulo_apertura, 70, 110 },
  { 155, 625, ini_noventa + angulo_inclinacion, 70, 180 },
  { 125, 620, ini_noventa + angulo_inclinacion, 0, 150 },

  { 110, 600, ini_noventa - angulo_apertura, 70, 110 },
  { 140, 620, ini_noventa - angulo_inclinacion, 70, 180 },
  { 150, 640, ini_noventa - angulo_inclinacion, 0, 150 },

  { 100, 500, ini_noventa, 70, 110 },
  { 120, 620, ini_noventa - angulo_inclinacion, 70, 180 },
  { 75, 565, ini_noventa - angulo_inclinacion, 0, 150 }
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

  /*servo_one.attach(8, 300, 6300);
    servo_two.attach(9, pMin, pMax);*/

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
  duty1 = map(angulo1, 0, 180, pMin, pMax);
  servo_placa.setPWM(n_servo1, 0, duty1);
}

void MovUp() {
  angulo_mov = 30;
  velocidad = 60;


  int matris_use_servo_a[][4] = {
    { 0, 1, 2, 1 },
    { 6, 7, 8, 1 },
    { 12, 13, 14, 0 }
  };

  int h = 0;
  for (int j = 0; j < 3; j++) {


    for (float i = 0; i <= angulo_mov; i += salto) {

      h = matris_use_servo_a[j][3] == 1 ? 1 : -1;

      Serial.println("H: " + String(matris_use_servo_a[j][3]));
      SetServoMovGeneral(matris_use_servo_a[j][0], matrisServoControl[matris_use_servo_a[j][0]][2] + (i * h));
      //Serial.println("A: " + String(matrisServoControl[0][2] + (i * h)));
      if (i <= angulo_mov / 2) {
        SetServoMovGeneral(matris_use_servo_a[j][1], matrisServoControl[matris_use_servo_a[j][1]][2] + ((i * 2) * h));
        SetServoMovGeneral(matris_use_servo_a[j][2], matrisServoControl[matris_use_servo_a[j][2]][2] + ((i * 2) * h));
        Serial.println("One: " + String(matrisServoControl[matris_use_servo_a[j][1]][2] + ((i * 2) * h)));
      } else {
        SetServoMovGeneral(matris_use_servo_a[j][1], (matrisServoControl[matris_use_servo_a[j][1]][2] + angulo_mov) - ((i - (angulo_mov / 2)) * 2));
        SetServoMovGeneral(matris_use_servo_a[j][2], (matrisServoControl[matris_use_servo_a[j][2]][2] + angulo_mov) - ((i - (angulo_mov / 2)) * 2));
        Serial.println("Two: " + String((matrisServoControl[matris_use_servo_a[j][1]][2] + angulo_mov) - ((i - (angulo_mov / 2)) * 2)));
      }
      delay(velocidad);
    }
  }
}
void TestMov() {
  angulo_apertura = 30;
  angulo_inclinacion = 20;

  ini_one_1 = ini_noventa - angulo_apertura - 20;     //40
  ini_one_2 = ini_noventa + angulo_inclinacion;       //110
  ini_one_3 = ini_noventa + angulo_inclinacion + 10;  //120 MAX ANGULO DE MOV  160

  angulo_mov = 50;
  velocidad = 60;
  int salto = 5;

  while (true) {

    for (float i = 0; i <= angulo_mov; i += salto) {
      SetServoMovGeneral(0, ini_one_1 + i);

      if (i <= angulo_mov / 2) {
        SetServoMovGeneral(1, ini_one_2 + (i * 2));
        SetServoMovGeneral(2, ini_one_3 + (i * 2));
        Serial.println("One: " + String(ini_one_2 + (i * 2)));
      } else {
        SetServoMovGeneral(1, (ini_one_2 + angulo_mov) - ((i - (angulo_mov / 2)) * 2));
        SetServoMovGeneral(2, (ini_one_3 + angulo_mov) - ((i - (angulo_mov / 2)) * 2));
        Serial.println("Two: " + String((ini_one_2 + angulo_mov) - (i / 2)) + " i:" + String((i - 15) * 2));
      }

      delay(velocidad);
    }

    int vuel = 1;
    for (int i = 0; i <= angulo_mov - salto; i += salto) {
      SetServoMovGeneral(0, (ini_one_1 + angulo_mov) - i);

      if (i >= angulo_mov / 3) {
        SetServoMovGeneral(2, ini_one_3 - (vuel));
        vuel += 2;
        Serial.println("Vuelta: " + String(vuel));
      }

      delay(velocidad);
    }

    DATO = miBT.read();
    if (DATO == 'l') {
      break;
    }
  }
}

void HexUp() {
  velocidad = 60;
  salto = 5;



  while (true) {
    for (float i = 0; i <= angulo_mov; i += salto) {
      SetServoMovGeneral(0, ini_one_1 + i);

      if (i <= angulo_mov / 2) {
        SetServoMovGeneral(1, ini_one_2 + (i * 2) + 20);
        SetServoMovGeneral(2, ini_one_3 + (i * 2));
        Serial.println("One: " + String(ini_one_2 + (i * 2)));
      } else {
        SetServoMovGeneral(1, (ini_one_2 + angulo_mov) - ((i - (angulo_mov / 2)) * 2));
        SetServoMovGeneral(2, (ini_one_3 + angulo_mov) - ((i - (angulo_mov / 2)) * 2));
        Serial.println("Two: " + String((ini_one_2 + angulo_mov) - (i / 2)) + " i:" + String((i - 15) * 2));
      }

      delay(velocidad);
    }

    int vuel = 1;
    for (int i = 0; i <= angulo_mov - salto; i += salto) {
      SetServoMovGeneral(0, (ini_one_1 + angulo_mov) - i);

      if (i >= angulo_mov / 3) {
        SetServoMovGeneral(2, ini_one_3 - (vuel));
        vuel += 2;
        Serial.println("Vuelta: " + String(vuel));
      }

      delay(velocidad);
    }

    DATO = miBT.read();
    if (DATO == 'l') {
      break;
    }
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
double ejeZ = 0;

double gamma;

double alfa1;
double alfa2;
double alfa;

double beta;

double hipoXY;
double lineaAlfa;

double desfaceZ = 70;

double COXA = 28;
double FEMUR = 55;
double TIBIA = 80;

// Funciones
double HipoXY(double ejeX, double ejeY) {
  return sqrt(pow(ejeX, 2) + pow(ejeY, 2));
}

double LineaA(double desfaceZ, double hipotenusaXY, double COXA) {
  return sqrt(pow(desfaceZ, 2) + pow(hipotenusaXY - COXA, 2));
}

double Gamma(double ejeY, double ejeX) {
  return abs(90 - (atan(ejeY / ejeX) * RAD_TO_DEG));
}

double Alfa(double desfaceZ, double lineaAlfa, double TIBIA, double FEMUR) {
  alfa1 = acos(desfaceZ / lineaAlfa) * RAD_TO_DEG;
  alfa2 = acos((pow(TIBIA, 2) - pow(FEMUR, 2) - pow(lineaAlfa, 2)) / (-2 * FEMUR * lineaAlfa)) * RAD_TO_DEG;
  return alfa1 + alfa2;
}

double Beta(double lineaAlfa, double TIBIA, double FEMUR) {
  return abs((acos((pow(lineaAlfa, 2) - pow(TIBIA, 2) - pow(FEMUR, 2)) / (-2 * TIBIA * FEMUR)) * RAD_TO_DEG)-180);
}

void CinematicaInversa() {
  double CinematicServo[][2] = {
    //ejeX, ejeY
    { 77, 50 },
    { 77, 0 },
    { 77, -20 },

    { 77, 20 },
    { 77, 0 },
    { 77, 20 }
  };

  desfaceZ = 60;
  Serial.println("--Cinematic--");
  for (int i = 0; i < 3; i++) {
    ejeX = CinematicServo[i][0];
    ejeY = CinematicServo[i][1];

    hipoXY = HipoXY(ejeX, ejeY);
    lineaAlfa = LineaA(desfaceZ, hipoXY, COXA);
    gamma = Gamma(ejeY, ejeX);
    alfa = Alfa(desfaceZ, lineaAlfa, TIBIA, FEMUR);
    beta = Beta(lineaAlfa, TIBIA, FEMUR);

    for (int j = i * 3; j < i * 3 + 1; j++) {
      SetServoMovGeneral(j, gamma);
      SetServoMovGeneral(j + 1, alfa);
      SetServoMovGeneral(j + 2, beta);
    }

    Serial.println("Gamma:" + String(gamma) + " --  Alfa:" + String(alfa) + "  -- Beta:" + String(beta));
    delay(1000);
  }
  /*
  double ini = -30;
  double fin = 60;
  velocidad = 10;
  salto = 10;

  while (true) {
    for (ejeY = fin; ejeY >= ini; ejeY -= salto) {
      hipoXY = HipoXY(ejeX, ejeY);
      lineaAlfa = LineaA(desfaceZ, hipoXY, COXA);
      gamma = Gamma(ejeY, ejeX);
      alfa = Alfa(desfaceZ, lineaAlfa, TIBIA, FEMUR);
      beta = Beta(lineaAlfa, TIBIA, FEMUR);

      Serial.println("Gamma:" + String(gamma) + " ang:" + String(abs(90 - gamma)) + " --  Alfa:" + String(alfa) + "  -- Beta:" + String(beta) + " ang:" + String(abs(beta - 180)));

      SetServoMovGeneral(0, abs(90 - gamma));
      SetServoMovGeneral(1, alfa);
      SetServoMovGeneral(2, abs(beta - 180));

      delay(velocidad);
    }

    for (ejeY = ini; ejeY <= fin; ejeY += salto) {
      hipoXY = HipoXY(ejeX, ejeY);
      lineaAlfa = LineaA(desfaceZ, hipoXY, COXA);
      gamma = Gamma(ejeY, ejeX);
      alfa = Alfa(desfaceZ, lineaAlfa, TIBIA, FEMUR);
      beta = Beta(lineaAlfa, TIBIA, FEMUR);

      Serial.println("Gamma:" + String(gamma) + " ang:" + String(abs(90 - gamma)) + " --  Alfa:" + String(alfa) + "  -- Beta:" + String(beta) + " ang:" + String(abs(beta - 180)));

      SetServoMovGeneral(0, abs(90 - gamma));
      SetServoMovGeneral(1, alfa);
      SetServoMovGeneral(2, abs(beta - 180));

      delay(velocidad);
    }
  }*/
}

void loop() {
  CinematicaInversa();
  //delay(2000);
  /*

  if (miBT.available()) {
    DATO = miBT.read();

    CalibrarServoMotor();

    switch (DATO) {
      case 'd':
        //TestMov();
        CinematicaInversa();
        break;
      case 'l':
        InicialPosition();
        break;
    }
  }
  delay(30);*/
}
