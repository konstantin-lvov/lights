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
const byte STEP = 1;
const byte MAX_LIGHT = 255;
const byte MIDDLE_LIGHT = 150;
const byte LOW_LIGHT = 15;
const byte LOWER_LIMIT = 0;
const int MOVEMENT_TRGGER = 500; // trashhold for action
const byte MOVEMENT_NIGH_LIMIT = 5; // light level if night
const byte MOVEMENT_BRIGHT_TRIGGER = 20;
const long TIME_FOR_MOVE = 3e5;
const int TIME_FOR_ON_OR_OFF = 1000; //millisec
const float MIN_BAT_VOLTAGE = 3.5;
const float MAX_BAT_VOLTAGE = 4.15;
const float COEF = 1;

bool lightIsOn = false;
bool powerIsOn = false;
bool isCharging = false;
bool lightsByMove = false;
bool onByMovementSensor = false;
bool nightLightsOn = false;

byte current_light_levels[3] = {LOWER_LIMIT, LOWER_LIMIT, LOWER_LIMIT}; //текущие уровни освещения

byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; //возможные номера труб
byte mosfets[3] = {TABLE_LIGHTS_MOSFET, KITCHEN_LIGHTS_MOSFET, HALL_LIGHTS_MOSFET}; // номера пинов для управления мосфетами
byte delay_for_changing_lights = 3;
byte pipeNo, gotByte, oldMaxLightLevel = 0, calculatedMaxLightLevel;

int photo_resistor = 0;

float battery_voltage = 0;

double maxLightLevel = 0;

long millisOnStart;
unsigned long lastMoveTime;
unsigned long currentTime;

void setup() {
  // Пины D5 и D6 - 7.8 кГц
  TCCR0B = 0b00000010;  // x8
  TCCR0A = 0b00000011;
  // Пины D5 и D6 - 7.8 кГц
  TCCR0B = 0b00000010;  // x8
  TCCR0A = 0b00000011;

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


}

void loop() {

  while (radio.available(&pipeNo)) {  // слушаем эфир со всех труб
    radio.read( &gotByte, sizeof(gotByte) );         // чиатем входящий сигнал
  }

  currentTime = millis();

  if (onByMovementSensor) {
    if (analogRead(movement_sensor) > MOVEMENT_TRGGER) {
      lastMoveTime  = millis();
    }
    if (currentTime - lastMoveTime > TIME_FOR_MOVE) {
      lightsOff();
      onByMovementSensor = false;
    }
  }

  if (currentTime - millisOnStart > 1000) { // раз в секунду проверяем несколько доп условий

    // коррректируем яркость по фоторезистору
    photo_resistor = analogRead(ph_resistor_pin);
    Serial.print("photo resistor: "); Serial.println(photo_resistor);

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

    if (gotByte > 0 && gotByte < 255) {
      Serial.print("Recieved: "); Serial.println(gotByte);
      applyCommand(gotByte);
      gotByte = 0;
    }
    delay(100);

    postAdjustment();

    if (!lightIsOn) {
      if (analogRead(movement_sensor) > MOVEMENT_TRGGER) {
        movementSensorHendler();
      }
    }


    millisOnStart = millis();

    Serial.print("Charging: "); Serial.println(isCharging);
    Serial.println("-----------------------------");

  }


}

void applyCommand(byte lightningOptions) {
  if (lightningOptions > 0 && lightningOptions < 4) {
    maxLightLevel = 0.004 * pow((photo_resistor * COEF), 2); //0 - 150 = 150
    if (maxLightLevel < LOW_LIGHT) {
      maxLightLevel = LOW_LIGHT;
    }
    if (maxLightLevel > 250) {
      maxLightLevel = 250;
    }
    if (lightIsOn) {
      lightsOff();
    } else {
      lightsOn();
    }
  }
}

void lightsOn() {
  Serial.println("lights on!");
  Serial.print("max light level: "); Serial.println(maxLightLevel);
  delay_for_changing_lights = TIME_FOR_ON_OR_OFF / maxLightLevel;
  Serial.print("delay_for_changing_lights: ");
  Serial.println(delay_for_changing_lights);
  digitalWrite(rele_pin, LOW);
  /*
     Пока свет не включится, включаем на всех потребителях
  */
  while (!lightIsOn) {
    for (byte x = 0; x < sizeof(mosfets); x++) {
      if (current_light_levels[x] < maxLightLevel) {
        current_light_levels[x] += STEP;
        analogWrite(mosfets[x], current_light_levels[x]);
      }
      /*
               Если уровень яркости всех потребителей на максимуме значит свет включен
      */
      if (current_light_levels[TABLE_LIGHTS_MOSFET] == maxLightLevel &&
          current_light_levels[KITCHEN_LIGHTS_MOSFET] == maxLightLevel &&
          current_light_levels[HALL_LIGHTS_MOSFET] == maxLightLevel) {
        lightIsOn = true;
      }
      delay(delay_for_changing_lights);
    }
  }
  oldMaxLightLevel - maxLightLevel;
}

void lightsOff() {
  Serial.println("lights off!");
  Serial.print("max light level: "); Serial.println(maxLightLevel);
  delay_for_changing_lights = TIME_FOR_ON_OR_OFF / maxLightLevel;
  Serial.print("delay_for_changing_lights: ");
  Serial.println(delay_for_changing_lights);
  /*
     Пока свет не выключится, гасим его на всех потребителях
  */
  while (lightIsOn) {
    for (byte x = 0; x < sizeof(mosfets); x++) {
      if (current_light_levels[x] > LOWER_LIMIT) {
        current_light_levels[x] -= STEP;
        analogWrite(mosfets[x], current_light_levels[x]);
      }
      /*
         Если уровень яркости всех потребителей на минимальном значении значит свет выключен
      */
      if (current_light_levels[TABLE_LIGHTS_MOSFET] == LOWER_LIMIT &&
          current_light_levels[KITCHEN_LIGHTS_MOSFET] == LOWER_LIMIT &&
          current_light_levels[HALL_LIGHTS_MOSFET] == LOWER_LIMIT) {
        lightIsOn = false;
      }
      delay(delay_for_changing_lights);
    }
  }
  /*
     Если батарея сейчас не заряжается то можно выключить
  */
  if (!isCharging) {
    digitalWrite(rele_pin, HIGH);
  }
  delay(1000);
}

void postAdjustment() {
  if (lightIsOn) {

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
      for (byte x = 0; x < sizeof(mosfets); x++) {

        current_light_levels[x] += STEP;
        analogWrite(mosfets[x], current_light_levels[x]);
        Serial.print("UP mosfet № "); Serial.print(x);
        Serial.print(". Current light level - "); Serial.println(current_light_levels[x]);
        delay(delay_for_changing_lights);
      }
      oldMaxLightLevel = current_light_levels[0];

    } else if (calculatedMaxLightLevel < oldMaxLightLevel
               && oldMaxLightLevel - calculatedMaxLightLevel > 10) {
      Serial.println(calculatedMaxLightLevel - oldMaxLightLevel);
      for (byte x = 0; x < sizeof(mosfets); x++) {

        current_light_levels[x] -= STEP;
        analogWrite(mosfets[x], current_light_levels[x]);
        Serial.print("DOWN mosfet № "); Serial.print(x);
        Serial.print(". Current light level - "); Serial.println(current_light_levels[x]);
        delay(delay_for_changing_lights);
      }
      oldMaxLightLevel = current_light_levels[0];

    }
  }

}

void movementSensorHendler () {
  if (photo_resistor < MOVEMENT_BRIGHT_TRIGGER) {
    analogWrite(TABLE_LIGHTS_MOSFET, 0);
    analogWrite(KITCHEN_LIGHTS_MOSFET, 0);
    analogWrite(HALL_LIGHTS_MOSFET, 0);
    delay_for_changing_lights = TIME_FOR_ON_OR_OFF / MOVEMENT_NIGH_LIMIT;
    digitalWrite(rele_pin, LOW);
    while (current_light_levels[HALL_LIGHTS_MOSFET] < MOVEMENT_NIGH_LIMIT) {
      current_light_levels[HALL_LIGHTS_MOSFET] += STEP;
      analogWrite(mosfets[HALL_LIGHTS_MOSFET], current_light_levels[HALL_LIGHTS_MOSFET]);
      delay(delay_for_changing_lights);
    }
    lightIsOn = true;
    nightLightsOn = true;
  } else {
    lightsOn();
  }
  onByMovementSensor = true;
}
