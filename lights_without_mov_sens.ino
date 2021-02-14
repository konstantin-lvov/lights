
//остановился на функции lightsOn. надо переписать блок со счетчиком counter

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#define ph_resistor_pin A6
#define battery_volt_pin A3
#define rele_pin 2
#define movement_sensor A0

RF24 radio(9, 10); // "создать" модуль на пинах 9 и 10 Для Уно

const byte TABLE_LIGHTS_MOSFET = 3;
const byte KITCHEN_LIGHTS_MOSFET = 5;
const byte HALL_LIGHTS_MOSFET = 6;

const byte MAX_LIGHT = 255;
const byte MIDDLE_LIGHT = 180;
const byte LOW_LIGHT = 100;
const byte TWILIGHT = 4;
const byte LOWER_LIMIT = 0;

const int DAY_LIGHT_TRESHOLD = 800; //parrots from photo resistor
const int EVENING_LIGHT_TRESHOLD = 400; //parrots from photo resistor
const int TWILIGHT_TRESHOLD = 2; //parrots from photo resistor


const int MOVEMENT_TRGGER = 500; // trashhold for action


const byte STEP = 1;
const byte MOVEMENT_NIGH_LIMIT = 4; // light level if night
const byte MOVEMENT_BRIGHT_TRIGGER = 2;
long TIME_FOR_MOVE = 3e5;
//const int TIME_FOR_ON_OR_OFF = 600; //millisec

const float MIN_BAT_VOLTAGE = 3.5;
const float MAX_BAT_VOLTAGE = 4.15;
const float COEF = 1;
const byte MULTIPLIER = 8;

bool lightIsOn = false;
bool powerIsOn = false;
bool isCharging = false;
bool lightsByMove = false;
bool onByMovementSensor = false;
bool nightLightsOn = false;
bool dedTime = false;
bool allLightsAction = false;
bool tableLightsAction = false;
bool kitchenLightsAction = false;


byte current_light_levels[] = {LOWER_LIMIT, LOWER_LIMIT, LOWER_LIMIT}; //текущие уровни освещения

byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; //возможные номера труб
byte mosfets[3] = {TABLE_LIGHTS_MOSFET, KITCHEN_LIGHTS_MOSFET, HALL_LIGHTS_MOSFET}; // номера пинов для управления мосфетами
byte pipeNo, gotByte, oldMaxLightLevel = 0, calculatedMaxLightLevel;

int delay_for_changing_lights = 10;
int photo_resistor = 0;
int counter = 0;

float battery_voltage = 0;

double maxLightLevel = 0;

unsigned long millisOnStart;
unsigned long lastMoveTime;
unsigned long currentTime;

void setup() {

  // Пины D5 и D6 - 7.8 кГц
  TCCR0B = 0b00000010;  // x8
  TCCR0A = 0b00000011;
  // Пины D3 и D11 - 8 кГц
  TCCR2B = 0b00000010;  // x8
  TCCR2A = 0b00000011;  // fast pwm


  //Serial.begin(9600); //открываем порт для связи с ПК
  radio.begin(); //активировать модуль
  radio.setAutoAck(1);         //режим подтверждения приёма, 1 вкл 0 выкл
  radio.setRetries(0, 15);    //(время между попыткой достучаться, число попыток)
  radio.enableAckPayload();    //разрешить отсылку данных в ответ на входящий сигнал
  radio.setPayloadSize(32);     //размер пакета, в байтах

  radio.openReadingPipe(1, address[0]);     //хотим слушать трубу 0
  radio.setChannel(0x60);  //выбираем канал (в котором нет шумов!)

  radio.setPALevel (RF24_PA_MAX); //уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.setDataRate (RF24_250KBPS); //скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
  //должна быть одинакова на приёмнике и передатчике!
  //при самой низкой скорости имеем самую высокую чувствительность и дальность!!

  radio.powerUp(); //начать работу
  radio.startListening();  //начинаем слушать эфир, мы приёмный модуль

  pinMode(rele_pin, OUTPUT);
  digitalWrite(rele_pin, HIGH);

  for (byte x = 0; x < sizeof(mosfets); x++) {
    analogWrite(mosfets[x], LOWER_LIMIT);
  }
  millisOnStart = millis();

//  Serial.begin(9600);
  photo_resistor = analogRead(ph_resistor_pin);
  Serial.print("photo resistor: "); Serial.println(photo_resistor);
  define_max_light_level();
  Serial.print("maxlightlevel: "); Serial.println(maxLightLevel);
  allLightsAction = true;
  lightsOn(false);
  allLightsAction = false;
}

void loop() {
  while (radio.available(&pipeNo)) {  // слушаем эфир со всех труб
    radio.read( &gotByte, sizeof(gotByte) );         // чиатем входящий сигнал
  }

  currentTime = millis();

  /*
     пока есть движение - будет задержка
     видимо борьба с ложными срабатываениями
  */
  //  if (onByMovementSensor) {
  //    if (analogRead(movement_sensor) > MOVEMENT_TRGGER) {
  //      lastMoveTime  = millis();
  //      while (!dedTime) {
  //        dedTime = true;
  //        for (int x = 0; x < 50; x++) {
  //          if (analogRead(movement_sensor) > MOVEMENT_TRGGER) {
  //            dedTime = false;
  //          }
  //          delay(2);
  //        }
  //      }
  //      dedTime = false;
  //    }

  /*
     Выставляем продолжительность включения света для дня и ночи
  */
  //    if (nightLightsOn) {
  //      TIME_FOR_MOVE = 1.8e5 * MULTIPLIER;
  //    } else {
  //      TIME_FOR_MOVE = 6.2e5 * MULTIPLIER;
  //    }

  /*
     Если время свечения истекло то выключаем свет
  */
  //    if (currentTime > lastMoveTime &&
  //        currentTime - lastMoveTime > TIME_FOR_MOVE) {
  //      Serial.println("DISABLE LIGHTS BECAUSE TIME IS UP");
  //      lightsOff();
  //      onByMovementSensor = false;
  //    }
  //  }

  if (currentTime - millisOnStart > 8000) { // раз в секунду проверяем несколько доп условий

    // читаем яркость с фоторезистора
    photo_resistor = analogRead(ph_resistor_pin);
    Serial.print("photo resistor: "); Serial.println(photo_resistor);

    define_max_light_level();
    Serial.print("maxlightlevel: "); Serial.println(maxLightLevel);

    // снимаем показания с батареи
    battery_voltage = (float)analogRead(battery_volt_pin) / 1023 * 5.0;
    Serial.print("battery voltage: "); Serial.println(battery_voltage);
    //в зависимости от напряжения включаем бп или выключаем или помечаем что зарядка закончилась если свет вкл
    if (battery_voltage < MIN_BAT_VOLTAGE && !isCharging) {
      digitalWrite(rele_pin, LOW);
      isCharging = true;
    } else if (battery_voltage >= MAX_BAT_VOLTAGE && isCharging) {
      isCharging = false;
      if (!lightIsOn) {
        digitalWrite(rele_pin, HIGH);
      }
    }

    switch (gotByte) {
      case 1:
        Serial.print("Recieved: "); Serial.println(gotByte);
        f_allLightsAction();
        gotByte = 0;
        break;
      case 2:
        Serial.print("Recieved: "); Serial.println(gotByte);
        f_kitchenLightsAction();
        gotByte = 0;
        break;
      case 3:
        Serial.print("Recieved: "); Serial.println(gotByte);
        f_tableLightsAction();
        gotByte = 0;
        break;
    }
    delay(50);

    postAdjustment();

    //    if (!lightIsOn) {
    //      if (analogRead(movement_sensor) > MOVEMENT_TRGGER) {
    //        Serial.println("MOVEMENT");
    //        movementSensorHendler();
    //      }
    //    }


    millisOnStart = millis();

    Serial.print("Charging: "); Serial.println(isCharging);
    Serial.println("-----------------------------");

  }


}

void f_allLightsAction() {

  if (kitchenLightsAction || tableLightsAction) {
    lightsOff(true);
  }

  allLightsAction = true;
  tableLightsAction = false;
  kitchenLightsAction = false;

  if (!lightIsOn) {
    Serial.println("lets lights on");
    lightsOn(false);
  } else {
    Serial.println("lets lights off");
    lightsOff(false);
  }
}

void f_tableLightsAction() {

  if (kitchenLightsAction || allLightsAction) {
    lightsOff(true);
  }

  allLightsAction = false;
  tableLightsAction = true;
  kitchenLightsAction = false;

  if (!lightIsOn) {
    lightsOn(false);
  } else {
    lightsOff(false);
  }
}

void f_kitchenLightsAction() {

  if (tableLightsAction || allLightsAction) {
    lightsOff(true);
  }

  allLightsAction = false;
  tableLightsAction = false;
  kitchenLightsAction = true;

  if (!lightIsOn) {
    lightsOn(false);
  } else {
    lightsOff(false);
  }
}

//###################
//#                 #
//# ВКЛЮЧЕНИЕ СВЕТА #
//#                 #
//###################

void lightsOn(bool byAnotherCall) {
  Serial.println("lights on!");
  Serial.print("max light level: "); Serial.println(maxLightLevel);
  delay_for_changing_lights = delay_for_changing_lights; // * MULTIPLIER;
  Serial.print("delay_for_changing_lights: ");
  Serial.println(delay_for_changing_lights);
  calculatedMaxLightLevel = maxLightLevel;
  digitalWrite(rele_pin, LOW);
  /*
     Пока свет не включится до максимального уровня, включаем на всех потребителях
  */
  while (!lightIsOn) {
    Serial.println("in begin while");
    Serial.print("calculatedMaxLightLevel: "); Serial.println(calculatedMaxLightLevel);

    //if (current_light_levels[0] < calculatedMaxLightLevel) {
    //            current_light_levels[0] += STEP;
    //            analogWrite(mosfets[0], current_light_levels[x]);
    //          }

    if (allLightsAction) {
      for (byte x = 0; x < sizeof(mosfets); x++) {
        if (current_light_levels[x] < calculatedMaxLightLevel) {
          current_light_levels[x] += STEP;
          analogWrite(mosfets[x], current_light_levels[x]);
        }
      }
    }

    if (tableLightsAction) {
      if (current_light_levels[0] < calculatedMaxLightLevel) {
        current_light_levels[0] += STEP;
        analogWrite(mosfets[0], current_light_levels[0]);
      }
    }

    if (kitchenLightsAction) {
      if (current_light_levels[1] < calculatedMaxLightLevel) {
        current_light_levels[1] += STEP;
        analogWrite(mosfets[1], current_light_levels[1]);
      }
    }

    //    Serial.print(x); Serial.print(" current_light_level -"); Serial.println(current_light_levels[x]);
    /*
             Если уровень яркости всех потребителей на максимуме значит свет включен
    */
    if (allLightsAction) {
      for (byte x = 0; x < sizeof(mosfets); x++) {
        counter += current_light_levels[x];
      }

      Serial.print("counter: "); Serial.println(counter);
      if (counter == calculatedMaxLightLevel * sizeof(mosfets)) {
        Serial.println("Condition true - all lights on");
        lightIsOn = true;
        counter = 0;
        break;
      }
      counter = 0;
      delay(delay_for_changing_lights);
    }

    if (tableLightsAction || kitchenLightsAction) {

      if (tableLightsAction) {
        counter = current_light_levels[0];
      }
      if (kitchenLightsAction) {
        counter = current_light_levels[1];
      }

      Serial.print("counter: "); Serial.println(counter);
      if (counter == calculatedMaxLightLevel) {
        Serial.println("Condition true - all lights on");
        lightIsOn = true;
        counter = 0;
        break;
      }
      counter = 0;
      delay(delay_for_changing_lights);

    }
    Serial.println("in the end of while");
  }

  Serial.println("after while");
  oldMaxLightLevel = calculatedMaxLightLevel;
}

//####################
//#                  #
//# ВЫКЛЮЧЕНИЕ СВЕТА #
//#                  #
//####################

void lightsOff(bool byAnotherCall) {
  Serial.println("lights off!");
  Serial.print("max light level: "); Serial.println(maxLightLevel);
  delay_for_changing_lights = delay_for_changing_lights;// * MULTIPLIER;
  Serial.print("delay_for_changing_lights: ");
  Serial.println(delay_for_changing_lights);
  calculatedMaxLightLevel = maxLightLevel;
  /*
     Пока свет не выключится, гасим его на всех потребителях
  */
  while (lightIsOn) {
    if (allLightsAction) {
      for (byte x = 0; x < sizeof(mosfets); x++) {
        if (current_light_levels[x] > LOWER_LIMIT) {
          current_light_levels[x] -= STEP;
          Serial.print("decrease all lights levels to:");
          Serial.println(current_light_levels[x]);
          analogWrite(mosfets[x], current_light_levels[x]);
        }
      }
    }

    if (tableLightsAction) {
      if (current_light_levels[0] > LOWER_LIMIT) {
        current_light_levels[0] -= STEP;
        analogWrite(mosfets[0], current_light_levels[0]);
      }
    }

    if (kitchenLightsAction) {
      if (current_light_levels[1] > LOWER_LIMIT) {
        current_light_levels[1] -= STEP;
        analogWrite(mosfets[1], current_light_levels[1]);
      }
    }
    /*
       Если уровень яркости всех потребителей на минимальном значении значит свет выключен
    */
    //    for (byte x = 0; x < sizeof(mosfets); x++) {
    //      counter += current_light_levels[x];
    //    }
    //    Serial.print("counter: "); Serial.println(counter);
    //    if (counter == LOWER_LIMIT) {
    //      Serial.println("Condition true - all lights off");
    //      lightIsOn = false;
    //      counter = 0;
    //      break;
    //    }
    //    counter = 0;
    //    delay(delay_for_changing_lights);

    if (allLightsAction) {
      for (byte x = 0; x < sizeof(mosfets); x++) {
        counter += current_light_levels[x];
      }

      Serial.print("counter: "); Serial.println(counter);
      if (counter == LOWER_LIMIT) {
        Serial.println("Condition true - all lights off");
        lightIsOn = false;
        counter = 0;
        break;
      }
      counter = 0;
      delay(delay_for_changing_lights);
    }

    if (tableLightsAction || kitchenLightsAction) {

      if (tableLightsAction) {
        counter = current_light_levels[0];
      }
      if (kitchenLightsAction) {
        counter = current_light_levels[1];
      }

      Serial.print("counter: "); Serial.println(counter);
      if (counter == LOWER_LIMIT) {
        Serial.println("Condition true - all lights off");
        lightIsOn = false;
        counter = 0;
        break;
      }
      counter = 0;
      delay(delay_for_changing_lights);

    }

  }
  /*
     Если батарея сейчас не заряжается то можно выключить
  */
  if (!isCharging || !byAnotherCall) {
    digitalWrite(rele_pin, HIGH);
  }

  onByMovementSensor = false;
  nightLightsOn = false;
}

//#######################
//#                     #
//# КОРРЕКТИРОВКА СВЕТА #
//#                     #
//#######################

void postAdjustment() {
  delay_for_changing_lights = delay_for_changing_lights;// * MULTIPLIER;
  if (lightIsOn && !nightLightsOn) {

    maxLightLevel = 0.004 * pow((photo_resistor * COEF), 2);

    if (maxLightLevel < LOW_LIGHT) {
      maxLightLevel = LOW_LIGHT;
    }
    if (maxLightLevel > 250) {
      maxLightLevel = 250;
    }
    calculatedMaxLightLevel = maxLightLevel;

    Serial.print("OLD max light level: "); Serial.println(oldMaxLightLevel);
    Serial.print("max light level: "); Serial.println(calculatedMaxLightLevel);

    if (calculatedMaxLightLevel > oldMaxLightLevel
        && calculatedMaxLightLevel - oldMaxLightLevel > 10) {
      Serial.println(calculatedMaxLightLevel - oldMaxLightLevel);
      if (allLightsAction) {
        for (byte x = 0; x < sizeof(mosfets); x++) {

          current_light_levels[x] += STEP;
          analogWrite(mosfets[x], current_light_levels[x]);
          Serial.print("UP mosfet № "); Serial.print(x);
          Serial.print(". Current light level - "); Serial.println(current_light_levels[x]);
          delay(delay_for_changing_lights);
        }
      }
      if (tableLightsAction) {
        current_light_levels[0] += STEP;
        analogWrite(mosfets[0], current_light_levels[0]);
        Serial.print("UP mosfet № "); Serial.print(0);
        Serial.print(". Current light level - "); Serial.println(current_light_levels[0]);
        delay(delay_for_changing_lights);
      }
      if (kitchenLightsAction) {
        current_light_levels[1] += STEP;
        analogWrite(mosfets[1], current_light_levels[1]);
        Serial.print("UP mosfet № "); Serial.print(1);
        Serial.print(". Current light level - "); Serial.println(current_light_levels[1]);
        delay(delay_for_changing_lights);
      }

      oldMaxLightLevel = current_light_levels[0];

    } else if (calculatedMaxLightLevel < oldMaxLightLevel
               && oldMaxLightLevel - calculatedMaxLightLevel > 10) {
      Serial.println(calculatedMaxLightLevel - oldMaxLightLevel);
      if (allLightsAction) {
        for (byte x = 0; x < sizeof(mosfets); x++) {

          current_light_levels[x] -= STEP;
          analogWrite(mosfets[x], current_light_levels[x]);
          Serial.print("DOWN mosfet № "); Serial.print(x);
          Serial.print(". Current light level - "); Serial.println(current_light_levels[x]);
          delay(delay_for_changing_lights);
          oldMaxLightLevel = current_light_levels[0];
        }
      }
      if (tableLightsAction) {
        current_light_levels[0] -= STEP;
        analogWrite(mosfets[0], current_light_levels[0]);
        Serial.print("DOWN mosfet № "); Serial.print(0);
        Serial.print(". Current light level - "); Serial.println(current_light_levels[0]);
        delay(delay_for_changing_lights);
        oldMaxLightLevel = current_light_levels[0];
      }
      if (kitchenLightsAction) {
        current_light_levels[1] -= STEP;
        analogWrite(mosfets[1], current_light_levels[1]);
        Serial.print("DOWN mosfet № "); Serial.print(1);
        Serial.print(". Current light level - "); Serial.println(current_light_levels[1]);
        delay(delay_for_changing_lights);
        oldMaxLightLevel = current_light_levels[1];
      }



    }
  }

}

//const byte MAX_LIGHT = 255;
//const byte MIDDLE_LIGHT = 180;
//const byte LOW_LIGHT = 100;
//const byte TWILIGHT = 4;
//const byte LOWER_LIMIT = 0;

//const int DAY_LIGHT_TRESHOLD = 800; //parrots from photo resistor
//const int EVENING_LIGHT_TRESHOLD = 400; //parrots from photo resistor
//const int TWILIGHT_TRESHOLD = 2; //parrots from photo resistor
void define_max_light_level() {
  if (photo_resistor  <= TWILIGHT_TRESHOLD) {
    maxLightLevel = TWILIGHT;
  }
  if (photo_resistor > TWILIGHT_TRESHOLD) {
    maxLightLevel = LOW_LIGHT;
  }
  if (photo_resistor > EVENING_LIGHT_TRESHOLD) {
    maxLightLevel = MIDDLE_LIGHT;
  }
  if (photo_resistor > DAY_LIGHT_TRESHOLD) {
    maxLightLevel = MAX_LIGHT;
  }


}
