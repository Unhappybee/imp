#include <ezButton.h>  // the library to use for SW pin
#include <BluetoothSerial.h>
#include <esp_wifi.h> //modem sleep
#include <driver/rtc_io.h> //deep sleep

//Code for rotary encoder was taken from https://esp32io.com/tutorials/esp32-rotary-encoder#google_vignette

#define CLK_PIN 25 // ESP32 pin GPIO25 connected to the rotary encoder's CLK pin
#define DT_PIN  26 // ESP32 pin GPIO26 connected to the rotary encoder's DT pin
#define SW_PIN  27 // ESP32 pin GPIO27 connected to the rotary encoder's SW pin

#define DIRECTION_CW  0   // clockwise direction
#define DIRECTION_CCW 1  // counter-clockwise direction

#define LED_PIN 13 //pin for LED 
#if CONFIG_IDF_TARGET_ESP32
#define THRESHOLD 25// Greater the value, more the sensitivity
#else // ESP32-S2 and ESP32-S3 + default for other chips (to be adjusted)
#define THRESHOLD 5000 // Lower the value, more the sensitivity
#endif

#define WAKEUP_GPIO  GPIO_NUM_2

volatile int counter = 0;
volatile int direction = DIRECTION_CW;
volatile unsigned long last_time; 
int prev_counter;
unsigned long lastTouchTime = 0; 
unsigned long debounceTime = 500;
RTC_DATA_ATTR int  wasInDeepSleep = false; //counter for deep sleep
RTC_DATA_ATTR int bootCount = 0; //counter for deep sleep
bool fromLightSleep = false;

ezButton button(SW_PIN);  // create ezButton object that attach to pin 27;
BluetoothSerial bt;

enum PowerMode {  ACTIVE, MODEM_SLEEP, LIGHT_SLEEP, DEEP_SLEEP };
PowerMode currentMode = ACTIVE;


void IRAM_ATTR ISR_encoder(); 
void print_wakeup_reason();

void setup() {
  Serial.begin(115200);

  PowerMode currentMode = ACTIVE;
  if (wasInDeepSleep) {
    Serial.println("Woke up from DEEP SLEEP");
    wasInDeepSleep = false; // Reset the flag
    print_wakeup_reason();
  } else {
    Serial.println("Fresh boot or wakeup from LIGHT SLEEP");
  }
  
  bootCount++;
  Serial.printf("Boot count: %d\n", bootCount);


  //rotary encoder:
  // configure encoder pins as inputs
  pinMode(CLK_PIN, INPUT);
  pinMode(DT_PIN, INPUT);
  button.setDebounceTime(50);  // set debounce time to 50 milliseconds

  // read the initial state of the rotary encoder's CLK pin
  attachInterrupt(digitalPinToInterrupt(CLK_PIN), ISR_encoder, RISING);

  //Led
  pinMode(LED_PIN, OUTPUT);

  //bluetooth setup
   bt.begin("MY ESP32 PROJECT");
   bt.setTimeout(50);

   esp_sleep_enable_ext0_wakeup(WAKEUP_GPIO, 1);
   rtc_gpio_pullup_dis(WAKEUP_GPIO);
   rtc_gpio_pulldown_en(WAKEUP_GPIO);

  delay(100);
}

void loop() {
  button.loop();  

  // read the current state of the rotary encoder's CLK pin
  if (prev_counter != counter) {
    Serial.print("Rotary Encoder:: direction: ");
    if (direction == DIRECTION_CW)
      Serial.print("CLOCKWISE");
    else
      Serial.print("ANTICLOCKWISE");

    Serial.print(" - count: ");
    Serial.println(counter*counter);

    prev_counter = counter;
  }

  if (button.isPressed()) {
    Serial.println("The button is pressed");
  }

  if (fromLightSleep) {
    Serial.println("Woke up from LIGHT SLEEP");
    fromLightSleep = false;
    lastTouchTime = millis();
    currentMode = ACTIVE;
  }

  if (bt.available()) {
    String input = bt.readString();
    input.trim();

    if (input == "led_on") {
      digitalWrite(LED_PIN, HIGH);
    } else if (input == "led_off") {
      digitalWrite(LED_PIN, LOW);
    }else if (input == "Go_to_modem_sleep"){
       if (currentMode != MODEM_SLEEP){
        Serial.println("Go to modem sleep");
        esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
        currentMode = MODEM_SLEEP;
       }else {
        Serial.println("Device is already in modem sleep");
       }
    }else if (input == "Wake_up_from_modem_sleep"){
      if (currentMode == MODEM_SLEEP){
        Serial.println("Wake up from modem sleep");
        esp_wifi_set_ps(WIFI_PS_NONE);
        currentMode = ACTIVE;
      } else {
        Serial.println("Device is not it modem sleep mode");
      }
    } else if (input == "Go_to_light_sleep") {
      if (currentMode == ACTIVE) {
        fromLightSleep = true;
        Serial.println("Go to light sleep mode");
        delay(100);  
        currentMode = LIGHT_SLEEP;
        esp_light_sleep_start(); 
      } else {
        Serial.println("Already in Light Sleep.");
      }
    }
     else if (input == "go_to_sleep"){
      wasInDeepSleep = true;
      Serial.println("Going to sleep now");
      delay(1000);
      Serial.flush(); 
      //going to sleep right after this function is called. Any code after that wont be run 
      esp_deep_sleep_start();
     }
     
  }

  //modem sleep 
  int touchValue12 = touchRead(12);
  int touchValue14 = touchRead(14);
  if(touchValue12 < THRESHOLD && touchValue14 > THRESHOLD){
    if (millis() - lastTouchTime > debounceTime){
      lastTouchTime = millis();
      if(currentMode == MODEM_SLEEP){
        Serial.println("Wake up from modem sleep");
        esp_wifi_set_ps(WIFI_PS_NONE);
        currentMode = ACTIVE;
      } else if(currentMode == ACTIVE){
        Serial.println("Go to modem sleep");
        esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
        currentMode = MODEM_SLEEP;
      }
    }    
  }
  //light sleep
   
  if(touchValue14 < THRESHOLD && touchValue12 > THRESHOLD){
    if (millis() - lastTouchTime > debounceTime){
      lastTouchTime = millis();

       if(currentMode == ACTIVE){
        fromLightSleep = true;
        Serial.println("Go to light sleep mode...");
        delay(1000);  
        currentMode = LIGHT_SLEEP;
        esp_light_sleep_start();
       }

    }
  }
  

  //deep sleep
  if(touchValue12 < THRESHOLD && touchValue14 < THRESHOLD){
    if (millis() - lastTouchTime > debounceTime){
      lastTouchTime = millis();
      wasInDeepSleep = true;
      Serial.println("Going to deep sleep now");
      delay(1000);
      Serial.flush(); 
      //going to sleep right after this function is called. Any code after that wont be run 
      esp_deep_sleep_start();
    }
  }


}

void IRAM_ATTR ISR_encoder() {
  if ((millis() - last_time) < 50)  // debounce time is 50ms
    return;

  if (digitalRead(DT_PIN) == HIGH) {
    // the encoder is rotating in counter-clockwise direction => decrease the counter
    counter--;
    direction = DIRECTION_CCW;
  } else {
    // the encoder is rotating in clockwise direction => increase the counter
    counter++;
    direction = DIRECTION_CW;
  }

  last_time = millis();
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:     Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1:     Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER:    Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP:      Serial.println("Wakeup caused by ULP program"); break;
    default:                        Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}
