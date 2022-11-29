#define BLINKER_BLE

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Adafruit_Fingerprint.h>
#include <Blinker.h>

// Light: 若有 stk500_getsync() attempt 1 of 10 错误, 需要检查板子上的 RX/TX 是否被占用
/* Light:
 *  == LEDs ==
 *  Red LED -> 2
 *  Green LED -> 3
 *  == UltraSonic ==
 *  Trigger -> 7
 *  Echo -> 6
 *  == Finger ==
 *  RX -> 0
 *  TX -> 1
 *  == Blinker ==
 *  RX -> 0
 *  TX -> 1
 */

SoftwareSerial BT(0,1);

int Red_LED = 2;
int Green_LED = 3;
int UltraSonic_Trigger = 7;
int UltraSonic_Echo = 6;
// 定义超声波运算距离
float distance;

// 定义 指纹传感器软串口
#if (defined(__AVR__) || defined(ESP8266)) && !defined(__AVR_ATmega2560__)
SoftwareSerial mySerial(8, 9);
#else
#endif
// 实例化 指纹传感器
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

// Light: Blinker 初始化
BlinkerButton Button1("btn-red");
BlinkerButton Button2("btn-green");
BlinkerButton Button3("btn-enroll");
BlinkerNumber Number1("num-abc");
int counter;

// Light: Blinker 传输未识别的数据时的处理
void dataRead(const String & data)
{
    BLINKER_LOG("Detected other Component: ", data);
    counter++;
    Number1.print(counter);
}

// Light: 按钮回调函数
void Button1_callback(const String & state)
{
    /* Light: 未知原因, 在 state 为 TAP 且 Monitor 显示 connected 时
     *  若 LED 开启, 则按下按钮会失效 */
    if (state == BLINKER_CMD_BUTTON_PRESS)
    {
        // 长按让 LED 熄灭
        digitalWrite(Red_LED, LOW);
    }
    if (state == BLINKER_CMD_BUTTON_TAP or state == BLINKER_CMD_CONNECTED)
    {
        digitalWrite(Red_LED, (!digitalRead(Red_LED)));
        Blinker.print("RedLED", digitalRead(Red_LED));
        if (digitalRead(Red_LED) == 0)
        {
            Blinker.print("RedLED OFF");
        }
        else
        {
            Blinker.print("RedLED ON");
        }
    }
}

void Button2_callback(const String & state)
{
    if (state == BLINKER_CMD_BUTTON_PRESS)
    {
        // 长按让 LED 熄灭
        digitalWrite(Green_LED, LOW);
    }
    if (state == BLINKER_CMD_BUTTON_TAP or state == BLINKER_CMD_CONNECTED)
    {
        digitalWrite(Green_LED, (!digitalRead(Green_LED)));
        Blinker.print("GreenLED", digitalRead(Green_LED));
        if (digitalRead(Green_LED) == 0)
        {
            Blinker.print("GreenLED OFF");
        }
        else
        {
            Blinker.print("GreenLED ON");
        }
    }
}

void Button3_callback(const String & state)
{

}

// Light: LED
void LED_Init()
{
    // 所有初始化 LED 均为熄灭状态
    pinMode(Red_LED, OUTPUT);
    pinMode(Green_LED, OUTPUT);
    digitalWrite(Red_LED, LOW);
    digitalWrite(Green_LED, LOW);
}

void LED_LoopBody()
{
    digitalWrite(Red_LED, HIGH);
    delay(100);
    digitalWrite(Red_LED, LOW);
    delay(100);
}

// Light: 超声波
void UltraSonic_Init()
{
    pinMode(UltraSonic_Trigger, OUTPUT);
    pinMode(UltraSonic_Echo, INPUT);
}

void UltraSonic_LoopBody()
{
    // 发送 10ms 高脉冲触发 TriggerPin
    digitalWrite(UltraSonic_Trigger, LOW);
    delayMicroseconds(2);
    digitalWrite(UltraSonic_Trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(UltraSonic_Trigger, LOW);

    // 计算
    distance = pulseIn(UltraSonic_Echo, HIGH) / 58.0;
    distance = (int(distance * 100.0)) / 100.0;     // 保留两位小数

    if (distance < 0)
    {
        distance = 0.00;
        Blinker.print("distance", distance, " cm, too Close");

        Serial.print("Now distance is: ");
        Serial.print(distance);
        Serial.println(" cm, Maybe you're too close!");

    }
    else if (distance <= 10 && distance >= 0)
    {
        Blinker.print("distance", distance, " cm, can Unlock");

        Serial.print("Now distance is: ");
        Serial.print(distance);
        Serial.print(" cm, ");
        Serial.println("You can Unlock the door!");

        LED_LoopBody();
    }
    else
    {
        Blinker.print("distance", distance, " cm");

        Serial.print("Now distance is: ");
        Serial.print(distance);
        Serial.println(" cm, Waiting...");
    }
    delay(200);
}

// Light: 指纹
void FingerPrint_Init()
{
    finger.begin(57600);   // TODO: 是否冲突存疑

    // 指纹传感器存在性检测
    delay(500);
    if (finger.verifyPassword())
    {
//        Blinker.print("FingerPrint", "Founded!");
        Serial.println("FingerPrintSensor Founded!");
    }
    else
    {
//        Blinker.print("FingerPrint", "Not Found...");
        Serial.println("FingerPrintSensor Not Found!");
    }
    Serial.println(F("Reading sensor parameters"));
    finger.getParameters();
    Serial.print(F("Status: 0x")); Serial.println(finger.status_reg, HEX);
    Serial.print(F("Sys ID: 0x")); Serial.println(finger.system_id, HEX);
    Serial.print(F("Capacity: ")); Serial.println(finger.capacity);
    Serial.print(F("Security level: ")); Serial.println(finger.security_level);
    Serial.print(F("Device address: ")); Serial.println(finger.device_addr, HEX);
    Serial.print(F("Packet len: ")); Serial.println(finger.packet_len);
    Serial.print(F("Baud rate: ")); Serial.println(finger.baud_rate);

    finger.getTemplateCount();

    if (finger.templateCount == 0) {
        Serial.println("Sensor doesn't contain any fingerprint data. Please run the 'enroll' example.");
    }
    else {
        Serial.println("Waiting for valid finger...");
        Serial.print("Sensor contains "); Serial.print(finger.templateCount); Serial.println(" templates");
    }
}

// Light: 获取指纹 ID
uint8_t getFingerprintID() {
    // TODO: 尝试把UltraSonic 塞到这里orz
    UltraSonic_LoopBody();

    uint8_t p = finger.getImage();
    switch (p) {
        case FINGERPRINT_OK:
            Serial.println("Image taken");
            break;
        case FINGERPRINT_NOFINGER:
            Serial.println("No finger detected");
            return p;
        case FINGERPRINT_PACKETRECIEVEERR:
            Serial.println("Communication error");
            return p;
        case FINGERPRINT_IMAGEFAIL:
            Serial.println("Imaging error");
            return p;
        default:
            Serial.println("Unknown error");
            return p;
    }

    // OK success!

    p = finger.image2Tz();
    switch (p) {
        case FINGERPRINT_OK:
            Serial.println("Image converted");
            break;
        case FINGERPRINT_IMAGEMESS:
            Serial.println("Image too messy");
            return p;
        case FINGERPRINT_PACKETRECIEVEERR:
            Serial.println("Communication error");
            return p;
        case FINGERPRINT_FEATUREFAIL:
            Serial.println("Could not find fingerprint features");
            return p;
        case FINGERPRINT_INVALIDIMAGE:
            Serial.println("Could not find fingerprint features");
            return p;
        default:
            Serial.println("Unknown error");
            return p;
    }

    // OK converted!
    p = finger.fingerSearch();
    if (p == FINGERPRINT_OK) {
        Serial.println("Found a print match!");
    } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
        Serial.println("Communication error");
        return p;
    } else if (p == FINGERPRINT_NOTFOUND) {
        Serial.println("Did not find a match");
        return p;
    } else {
        Serial.println("Unknown error");
        return p;
    }

    // found a match!
    UltraSonic_LoopBody();
    Serial.print("Found ID #"); Serial.print(finger.fingerID);
    Serial.print(" with confidence of "); Serial.println(finger.confidence);
    digitalWrite(Green_LED, HIGH);
    delay(100);
    digitalWrite(Green_LED, LOW);

    return finger.fingerID;
}

// returns -1 if failed, otherwise returns ID #
//int getFingerprintIDez() {
//    uint8_t p = finger.getImage();
//    if (p != FINGERPRINT_OK)  return -1;
//
//    p = finger.image2Tz();
//    if (p != FINGERPRINT_OK)  return -1;
//
//    p = finger.fingerFastSearch();
//    if (p != FINGERPRINT_OK)  return -1;
//
//    // found a match!
//    Serial.print("Found ID #"); Serial.print(finger.fingerID);
//    Serial.print(" with confidence of "); Serial.println(finger.confidence);
//    return finger.fingerID;
//}

void FingerPrint_LoopBody()
{
//    UltraSonic_LoopBody();
    getFingerprintID();
    delay(100);
}

void setup()
{
    Blinker.begin(0,1, 115200);
    Blinker.attachData(dataRead);
    Button1.attach(Button1_callback);
    Button2.attach(Button2_callback);
    Button3.attach(Button3_callback);

    BT.begin(115200);

    LED_Init();
    UltraSonic_Init();
    FingerPrint_Init();
}

void loop()
{
//    if (BT.available())
//    {
//        BT.println(BT.read());
//    }
    FingerPrint_LoopBody();
    UltraSonic_LoopBody();
    Blinker.run();
}