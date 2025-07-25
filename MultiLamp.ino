#include <user_interface.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266SSDP.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <IRremoteESP8266.h>
#include <UDPLogger.h>
#include <ArduinoJson.h>
#include "FS.h" 
#include "MCP.h"
#include "DHTx.h"
#include "IRSender.h"
#include "Light.h"

#define ESP8266_PIN_SDA   0
#define ESP8266_PIN_SCL   2
#define ESP8266_PIN_IRR   1
#define ESP8266_PIN_DHT   3

#define PIN_IR_SENDER     1
#define PIN_LED_GREEN_1   2
#define PIN_LED_RED_1     3
#define PIN_LED_BLUE_1    4
#define PIN_LED_GREEN_2   5
#define PIN_LED_RED_2     6
#define PIN_LED_BLUE_2    7

#define PIN_LED_STAT  8
#define PIN_RELAY_1   9
#define PIN_RELAY_2   10

#define WIFI_CONNECT_TIMEOUT   30000

#define arg_InfraRedTest  "irt"   // http://10.1.0.53/api?irt=1 http://10.1.0.53/api?irt=0
#define arg_Relay1        "r1"    // http://10.1.0.53/api?r1=1 http://10.1.0.53/api?r1=0
#define arg_Relay2        "r2"    // http://10.1.0.53/api?r2=1 http://10.1.0.53/api?r2=0
// http://10.1.0.53/api?lr1=0519&lg1=00050514&lb1=000A050F&lr2=000F050A&lg2=00140505&lb2=001905
// http://10.1.0.53/api?lr1=0001&lg1=0001&lb1=0001&lr2=0001&lg2=0001&lb2=0001
#define arg_LightRed1     "lr1"   // http://10.1.0.53/api?lr1=0100
#define arg_LightGreen1   "lg1"   // http://10.1.0.53/api?lg1=0100
#define arg_LightBlue1    "lb1"   // http://10.1.0.53/api?lb1=0100
#define arg_LightRed2     "lr2"   // http://10.1.0.53/api?lr2=0100
#define arg_LightGreen2   "lg2"   // http://10.1.0.53/api?lg2=0100
#define arg_LightBlue2    "lb2"   // http://10.1.0.53/api?lb2=0100
// OnOff http://10.1.0.53/api?ir=0080403434404034344040400B0B6C6C0B0B6C6C
// CancelTimer http://10.1.0.53/api?ir=00C020FEFE1C1C20FDFD1C1C20FEFEBCBC20FDFDBCBC2064642C2C20C9C92C2C
#define arg_IRSender      "ir"
// Turn on AC http://10.1.0.53/api?ac_power=1&ac_mode=3&ac_temp=26&ac_swing=1&ac_fan=3
// Turn off AC http://10.1.0.53/api?ac_power=0&ac_mode=3&ac_temp=26&ac_swing=1&ac_fan=3
// e-Ion AC http://10.1.0.53/api?ac_tgl=1
#define arg_AC_Power      "ac_power"
#define arg_AC_Mode       "ac_mode"
#define arg_AC_Temp       "ac_temp"
#define arg_AC_Swing      "ac_swing"
#define arg_AC_FanSpeed   "ac_fan"
#define arg_AC_Toggle     "ac_tgl"
#define arg_SaveConfig    "save"  // http://10.1.0.53/api?save=1
#define arg_FormatFS      "fmt"   // http://10.1.0.53/api?fmt=1
#define arg_SoftReset     "rst"   // http://10.1.0.53/api?rst=1
#define arg_LogIP         "logip"   // http://10.1.0.53/api?logip=10.1.0.4
#define arg_LogPort       "logport"   // http://10.1.0.53/api?logport=8280
#define arg_wifi_ssid     "wifissid"  // http://10.1.0.53/api?wifissid=Milky+Way
#define arg_wifi_pwd      "wifipwd"   // http://10.1.0.53/api?wifipwd=d5b2mg0K1udD

const char *def_otahostname = "myesp8266";
const char *def_wifi_ssid = "Milky Way";
const char *def_wifi_pwd = "d5b2mg0K1udD";
const char *def_apwifi_ssid = "lamputidur";
const char *def_apwifi_pwd = "mediacontrol";
const char *def_logHost = "10.1.0.4";

char apwifi_ssid[32];
char apwifi_pwd[32];

char wifi_ssid[32];
char wifi_pwd[32];

int otaport = 8266;
char otahostname[16];

char logHost[16];
int logPort = 8280;

// common
UDPLogger udpLog(logHost, logPort);
uint8_t pinTX = ESP8266_PIN_IRR;
uint8_t pinRX = ESP8266_PIN_DHT;
MCP mcp(ESP8266_PIN_SDA, ESP8266_PIN_SCL);
uint32_t loop_lastTrapTime;
uint32_t loop_cycle = 0;
uint16_t loop_led_on = 0;

uint8_t ota_update = 0;
int ota_progress;

// wifi
bool isWifiConnected = false;
bool isAPWifi = false;
uint32_t wifiTimeout = 0;

// webserver
ESP8266WebServer webserver(80);
//holds the current upload
File fsUploadFile;

// DHT
DHTx dht(ESP8266_PIN_DHT);

// ir receiver and sender
IRrecv irrecv(ESP8266_PIN_IRR);
IRSender irSender(&mcp, ESP8266_PIN_IRR);
volatile boolean irTestEnable = false;
uint32_t milisNextPanasonicSignal = 0;

// led stat
static os_timer_t led_timer;
volatile boolean led_state;
uint8_t nec_light = 0;
String nec_light_on = "0100";
String nec_light_off = "0001";

// sleepy light
Light light_Red_1(&mcp, PIN_LED_RED_1);
Light light_Green_1(&mcp, PIN_LED_GREEN_1);
Light light_Blue_1(&mcp, PIN_LED_BLUE_1);
Light light_Red_2(&mcp, PIN_LED_RED_2);
Light light_Green_2(&mcp, PIN_LED_GREEN_2);
Light light_Blue_2(&mcp, PIN_LED_BLUE_2);

// relay
volatile boolean relay_portDirty = true;
volatile boolean relay_portState_1 = LOW;
volatile boolean relay_portState_2 = LOW;

void ir_sendHex(String *strHex);
void led_init(unsigned long duration);
void light_init();
void light_setHexSignal(Light *light, String *strHex);
void reset_timer(unsigned long duration);

bool loadConfig() {
  bool use_default = true;

  File configFile = SPIFFS.open("/config.json", "r");
  if (configFile) {
    size_t size = configFile.size();
    std::unique_ptr<char[]> buf(new char[size]);
    configFile.readBytes(buf.get(), size);

    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& json = jsonBuffer.parseObject(buf.get());
    if (json.success()) {
      const char *str_config;
      str_config = json["otahostname"];
      strcpy(otahostname, str_config);
      str_config = json["wifi_ssid"];
      strcpy(wifi_ssid, str_config);
      str_config = json["wifi_pwd"];
      strcpy(wifi_pwd, str_config);
      str_config = json["apwifi_ssid"];
      strcpy(apwifi_ssid, str_config);
      str_config = json["apwifi_pwd"];
      strcpy(apwifi_pwd, str_config);
      str_config = json["logHost"];
      strcpy(logHost, str_config);
      logPort = json["logPort"];
      relay_portState_1 = json["relay_p1"];
      relay_portState_2 = json["relay_p2"];
      use_default = false;
    }
    configFile.close();
  }
  if (use_default) {
    strcpy(otahostname, def_otahostname);
    strcpy(wifi_ssid, def_wifi_ssid);
    strcpy(wifi_pwd, def_wifi_pwd);
    strcpy(apwifi_ssid, def_apwifi_ssid);
    strcpy(apwifi_pwd, def_apwifi_pwd);
    strcpy(logHost, def_logHost);
    return false;
  }
  return true;
}

bool saveConfig() {
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();

  json["otahostname"] = (const char *) &otahostname;
  json["wifi_ssid"] = (const char *) &wifi_ssid;
  json["wifi_pwd"] = (const char *) &wifi_pwd;
  json["apwifi_ssid"] = (const char *) &apwifi_ssid;
  json["apwifi_pwd"] = (const char *) &apwifi_pwd;
  json["logHost"] = (const char *) &logHost;
  json["logPort"] = logPort;
  json["relay_p1"] = relay_portState_1;
  json["relay_p2"] = relay_portState_2;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    return false;
  }

  json.printTo(configFile);
  configFile.close();
  return true;
}

void welcomeMessage() {
  udpLog.beginPrint();
  udpLog.print("\n{ Status: \"ESP8266 Ready, wifi has been connected.\"");
  udpLog.print(", FlashID: 0x");
  udpLog.print(ESP.getFlashChipId(), HEX);
  udpLog.print(", FlashSize: ");
  udpLog.print(ESP.getFlashChipRealSize());
  udpLog.print(" }");
  udpLog.endPrint();
}

void disableDevices() {
  irrecv.disableIRIn();
  light_init();
  mcp.reset();
}

void wifi_ConnectOnce() {
  isWifiConnected = true;
  MDNS.begin(otahostname);
  SSDP.begin();
  ArduinoOTA.begin();
  webserver.begin();
  welcomeMessage();
}

void wifi_ConnectLoop() {
  ArduinoOTA.handle();
  webserver.handleClient();
}

void checkWifi() {
  if (isAPWifi) {
    wifi_ConnectLoop();
    return;
  }
  if (WiFi.status() == WL_CONNECTED) {
    if (isWifiConnected) {
      wifi_ConnectLoop();
    } else {
      wifi_ConnectOnce();
      wifiTimeout = 0;
    }
  } else {
    if (isWifiConnected) {
      light_init();
    }
    isWifiConnected = false;
    if (wifiTimeout == 0) {
      wifiTimeout = millis() + WIFI_CONNECT_TIMEOUT;
      WiFi.softAPdisconnect(false);
      WiFi.begin(wifi_ssid, wifi_pwd);
    } else if (wifiTimeout < millis()) {
      // start access point
      WiFi.softAP(apwifi_ssid, apwifi_pwd);
      wifi_ConnectOnce();
      isAPWifi = true;
    }
  }
}

void reset_timerCallback(void *pArg) {
  os_timer_disarm(&led_timer);
  ESP.reset();
}

void reset_timer(unsigned long duration) {
  os_timer_disarm(&led_timer);
  os_timer_setfn(&led_timer, reset_timerCallback, NULL);
  os_timer_arm(&led_timer, duration, true);
}

void led_timerCallback(void *pArg) {
  if (led_state == HIGH) {
    led_state = LOW;
    mcp.digitalWrite(PIN_LED_STAT, LOW);
  } else {
    led_state = HIGH;
    mcp.digitalWrite(PIN_LED_STAT, HIGH);
  }
}

void led_init(unsigned long duration) {
  os_timer_disarm(&led_timer);
  if (duration == 0) {
    led_state = LOW;
    mcp.digitalWrite(PIN_LED_STAT, LOW);
  } else if (duration == 1) {
    led_state = HIGH;
    mcp.digitalWrite(PIN_LED_STAT, HIGH);
  } else {
    led_state = LOW;
    mcp.digitalWrite(PIN_LED_STAT, LOW);
    os_timer_setfn(&led_timer, led_timerCallback, NULL);
    os_timer_arm(&led_timer, duration, true);
  }
}

void light_init() {
  uint8_t light_Red_1_seq[] = {0, 255};
  uint8_t light_Green_1_seq[] = {0, 255};
  uint8_t light_Blue_1_seq[] = {0, 255};
  uint8_t light_Red_2_seq[] = {0, 255};
  uint8_t light_Green_2_seq[] = {0, 255};
  uint8_t light_Blue_2_seq[] = {0, 255};

  light_Red_1.setSignal(light_Red_1_seq, 2);
  light_Green_1.setSignal(light_Green_1_seq, 2);
  light_Blue_1.setSignal(light_Blue_1_seq, 2);
  light_Red_2.setSignal(light_Red_2_seq, 2);
  light_Green_2.setSignal(light_Green_2_seq, 2);
  light_Blue_2.setSignal(light_Blue_2_seq, 2);
}

void ssdp_init() {
  SSDP.setSchemaURL("description.xml");
  SSDP.setHTTPPort(80);
  SSDP.setName("Media Control Center");
  SSDP.setSerialNumber("00103-00652418");
  SSDP.setURL("index.html");
  SSDP.setModelName("Media Control Device");
  SSDP.setModelNumber("MCC-103");
  SSDP.setModelURL("http://www.arelosoft.com/mcc");
  SSDP.setManufacturer("Arelosoft Inovasi Dunia");
  SSDP.setManufacturerURL("http://www.arelosoft.com");
}

String getContentType(String filename) {
  if (filename.endsWith(".htm")) return "text/html";
  else if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".json")) return "application/json";
  else if (filename.endsWith(".png")) return "image/png";
  else if (filename.endsWith(".gif")) return "image/gif";
  else if (filename.endsWith(".jpg")) return "image/jpeg";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  else if (filename.endsWith(".xml")) return "text/xml";
  return "";
}

void webserver_handleDoc() {
  bool success = false;
  String path = webserver.uri();
  if (path.endsWith("/")) path += "index.html";
  String contentType = getContentType(path);
  if (contentType.length()) {
    if (SPIFFS.exists(path)) {
      File file = SPIFFS.open(path, "r");
      size_t sent = webserver.streamFile(file, contentType);
      file.close();
      success = true;
    }
  }
  if (!success) {
    String s = "";
    s += "<!DOCTYPE HTML>\r\n<html>\r\n";
    s += "<h1>Server Error</h1>\r\n";
    s += "<h2>404 - Document not found.</h2>\r\n";
    s += "</html>\r\n";
    webserver.send(404, "text/html", s);
  }
}

void webserver_handleDocManage() {
  String s = "";

  s += "<!DOCTYPE HTML>\r\n<html>\r\n";
  s += "<head><title>File Manager</title></head>\r\n<body>";
  s += "<form method=\"POST\" enctype=\"multipart/form-data\" ><input type=\"file\" name=\"filename\" /><input type=\"submit\" value=\"upload\" /></form><br />";
  s += "<table border=\"1\" width=\"600px\"><thead><th>Filename</th><th>Size</th></thead><tbody>";
  Dir dir = SPIFFS.openDir("/");
  while (dir.next()) {
    s += "<tr><td>";
    s += dir.fileName();
    s += "</td><td>";
    s += dir.fileSize();
    s += "</td></tr>";
  }
  s += "</tbody></table>";
  s += "</body></html>\r\n";
  webserver.send(200, "text/html", s);
}

void webserver_handleDocUpload() {
  if (webserver.uri() != "/upload") return;
  HTTPUpload& upload = webserver.upload();
  if (upload.status == UPLOAD_FILE_START) {
    String filename = upload.filename;
    if (!filename.startsWith("/")) filename = "/" + filename;
    fsUploadFile = SPIFFS.open(filename, "w");
    filename = String();
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (fsUploadFile) {
      fsUploadFile.close();
    }
  }
}

void webserver_handleApi() {
  bool success = false;
  String rv;

  // wifi SSID
  if (webserver.hasArg(arg_wifi_ssid)) {
    rv = webserver.arg(arg_wifi_ssid);
    if (rv.length() > 0) {
      strcpy(wifi_ssid, rv.c_str());
      success = true;
    }
  }

  // wifi password
  if (webserver.hasArg(arg_wifi_pwd)) {
    rv = webserver.arg(arg_wifi_pwd);
    if (rv.length() > 0) {
      strcpy(wifi_pwd, rv.c_str());
      success = true;
    }
  }

  // log host
  if (webserver.hasArg(arg_LogIP)) {
    rv = webserver.arg(arg_LogIP);
    if (rv.length() > 0) {
      strcpy(logHost, rv.c_str());
      success = true;
    }
  }

  // log port
  if (webserver.hasArg(arg_LogPort)) {
    rv = webserver.arg(arg_LogPort);
    if (rv.length() > 0) {
      logPort = rv.toInt();
      success = true;
    }
  }
      
  // infrared Test
  if (webserver.hasArg(arg_InfraRedTest)) {
    rv = webserver.arg(arg_InfraRedTest);
    if (rv.length() > 0) {
      irTestEnable = (rv == "1");
      success = true;
    }
  }
  // relay 1
  if (webserver.hasArg(arg_Relay1)) {
    rv = webserver.arg(arg_Relay1);
    if (rv.length() > 0) {
      relay_portState_1 = (rv == "1" ? HIGH : LOW);
      relay_portDirty = true;
      success = true;
    }
  }
  // relay 2
  if (webserver.hasArg(arg_Relay2)) {
    rv = webserver.arg(arg_Relay2);
    if (rv.length() > 0) {
      relay_portState_2 = (rv == "1" ? HIGH : LOW);
      relay_portDirty = true;
      success = true;
    }
  }
  // light Red 1
  if (webserver.hasArg(arg_LightRed1)) {
    rv = webserver.arg(arg_LightRed1);
    if ((rv.length() > 2) && (rv.length() <= 16) && (rv.length() % 2 == 0)) {
      light_setHexSignal(&light_Red_1, &rv);
      success = true;
    }
  }
  // light Green 1
  if (webserver.hasArg(arg_LightGreen1)) {
    rv = webserver.arg(arg_LightGreen1);
    if ((rv.length() > 2) && (rv.length() <= 16) && (rv.length() % 2 == 0)) {
      light_setHexSignal(&light_Green_1, &rv);
      success = true;
    }
  }
  // light Blue 1
  if (webserver.hasArg(arg_LightBlue1)) {
    rv = webserver.arg(arg_LightBlue1);
    if ((rv.length() > 2) && (rv.length() <= 16) && (rv.length() % 2 == 0)) {
      light_setHexSignal(&light_Blue_1, &rv);
      success = true;
    }
  }
  // light Red 2
  if (webserver.hasArg(arg_LightRed2)) {
    rv = webserver.arg(arg_LightRed2);
    if ((rv.length() > 2) && (rv.length() <= 16) && (rv.length() % 2 == 0)) {
      light_setHexSignal(&light_Red_2, &rv);
      success = true;
    }
  }
  // light Green 2
  if (webserver.hasArg(arg_LightGreen2)) {
    rv = webserver.arg(arg_LightGreen2);
    if ((rv.length() > 2) && (rv.length() <= 16) && (rv.length() % 2 == 0)) {
      light_setHexSignal(&light_Green_2, &rv);
      success = true;
    }
  }
  // light Blue 2
  if (webserver.hasArg(arg_LightBlue2)) {
    rv = webserver.arg(arg_LightBlue2);
    if ((rv.length() > 2) && (rv.length() <= 16) && (rv.length() % 2 == 0)) {
      light_setHexSignal(&light_Blue_2, &rv);
      success = true;
    }
  }
  // irSender
  if (webserver.hasArg(arg_IRSender)) {
    rv = webserver.arg(arg_IRSender);
    if ((rv.length() > 6) && (rv.length() < 512) && (rv.length() % 2 == 0)) {
      ir_sendHex(&rv);
      success = true;
    }
  }

  // AC Panasonic2
  if (webserver.hasArg(arg_AC_Power)) {
    uint8_t iPower = webserver.arg(arg_AC_Power).toInt();
    uint8_t iMode = webserver.arg(arg_AC_Mode).toInt();
    uint8_t iTemp = webserver.arg(arg_AC_Temp).toInt();
    uint8_t iSwing = webserver.arg(arg_AC_Swing).toInt();
    uint8_t iFanSpeed = webserver.arg(arg_AC_FanSpeed).toInt();
    irrecv.disableIRIn();
    irSender.sendACPanasonic2(iPower,  iMode,  iTemp,  iSwing,  iFanSpeed);
    irrecv.enableIRIn();
    success = true;
  }

  // AC Panasonic2 Toggle Button
  if (webserver.hasArg(arg_AC_Toggle)) {
    uint8_t iToggleButton = webserver.arg(arg_AC_Power).toInt();
    irrecv.disableIRIn();
    irSender.sendACPanasonic2Toggle(iToggleButton);
    irrecv.enableIRIn();
  }

  // save config
  if (webserver.hasArg(arg_SaveConfig)) {
    rv = webserver.arg(arg_SaveConfig);
    if (rv.length() > 0) {
      if (rv == "1") {
        success = saveConfig();
      }
    }
  }
  // formats the file system
  if (webserver.hasArg(arg_FormatFS)) {
    rv = webserver.arg(arg_FormatFS);
    if (rv.length() > 0) {
      if (rv == "1") {
        SPIFFS.format();
        success = true;
      }
    }
  }
  // Soft Reset
  if (webserver.hasArg(arg_SoftReset)) {
    rv = webserver.arg(arg_SoftReset);
    if (rv.length() > 0) {
      if (rv == "1") {
        udpLog.print("\n{ Status: \"Soft Reset\" }\n");
        disableDevices();
        reset_timer(3000);
        success = true;
      }
    }
  }

  String s = "";
  if (success) {
    s += "<!DOCTYPE HTML>\r\n<html>\r\n";
    s += "ESP8266:OK\r\n";
    s += "</html>\r\n";
    webserver.send(200, "text/html", s);
  } else {
    s += "<!DOCTYPE HTML>\r\n<html>\r\n";
    s += "ESP8266:Error\r\n";
    s += "</html>\r\n";
    webserver.send(500, "text/html", s);
  }
}

void webserver_init() {
  webserver.on("/api", HTTP_GET, webserver_handleApi);
  webserver.on("/description.xml", HTTP_GET, []() {
    SSDP.schema(webserver.client());
  });
  webserver.on("/upload", HTTP_GET, webserver_handleDocManage);
  webserver.on("/upload", HTTP_POST, webserver_handleDocManage, webserver_handleDocUpload);
  webserver.onNotFound(webserver_handleDoc);
}

void arduinoOTA_init() {
  ArduinoOTA.setPort(otaport);
  ArduinoOTA.setHostname(otahostname);
  //   ArduinoOTA.setPassword((const char *)"123"); // No authentication by default
  ArduinoOTA.onStart([]() {
    udpLog.print("\nLoading firmware\r");
    disableDevices();
    led_init(100UL);
    ota_progress = 0;
    ota_update = 1;
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    ota_progress++;
    if (ota_progress == 9) {
      udpLog.print(".\r");
      ota_progress = 0;
    }
  });
  ArduinoOTA.onEnd([]() {
    led_init(1UL);
    udpLog.print("done. Restarting...");
    delay(1000);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    led_init(0);
    udpLog.print("Error occured.");
    if (error == OTA_AUTH_ERROR) {
      udpLog.print("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      udpLog.print("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      udpLog.print("Connect Failed");
    }    else if (error == OTA_RECEIVE_ERROR) {
      udpLog.print("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      udpLog.print("End Failed");
    }
  });
}

void setup() {
  pinMode(pinRX, OUTPUT);
  pinMode(pinTX, OUTPUT);
  digitalWrite(pinRX, LOW);
  digitalWrite(pinTX, LOW);
  isAPWifi = false;
  isWifiConnected = false;
  wifiTimeout = 0;

  loop_led_on = 2;
  disableDevices();
  if (SPIFFS.begin()) {
    //    SPIFFS.format();
    loadConfig();
  }
  ssdp_init();
  arduinoOTA_init();
  webserver_init();
  WiFi.mode(WIFI_STA);

  mcp.begin();
  irrecv.enableIRIn();  // Start the receiver
  dht.begin();
  led_init(200UL);
  mcp.digitalWrite(PIN_RELAY_1, relay_portState_1);
  mcp.digitalWrite(PIN_RELAY_2, relay_portState_2);
  loop_lastTrapTime = millis();
}

void ir_send(int nbits, unsigned char *dataBuff, int dataBuff_len) {
  irrecv.disableIRIn();
  irSender.sendPack(nbits, dataBuff, dataBuff_len);
  irrecv.enableIRIn();
}

uint8_t parseCharHex(char ch1, char ch2) {
  uint8_t rv = 0;

  if ((ch1 >= '0') && (ch1 <= '9')) {
    rv = ch1 - '0';
  } else {
    rv = 10 + ((ch1 - 'A') & 0x0F);
  }
  rv <<= 4;
  if ((ch2 >= '0') && (ch2 <= '9')) {
    rv |= ch2 - '0';
  } else {
    rv |= 10 + ((ch2 - 'A') & 0x0F);
  }
  return rv;
}

void ir_sendHex(String * strHex) {
  int nbits;
  uint8_t dataBuff[512];
  int dataBuff_len;
  unsigned char ch1, ch2;

  dataBuff_len = (strHex->length() / 2) - 2;
  ch1 = strHex->charAt(0);
  ch2 = strHex->charAt(1);
  nbits = parseCharHex(ch1, ch2);
  nbits <<= 8;
  ch1 = strHex->charAt(2);
  ch2 = strHex->charAt(3);
  nbits |= parseCharHex(ch1, ch2);

  int j = 0;
  for (int i = 4; i < strHex->length(); i += 2) {
    ch1 = strHex->charAt(i);
    ch2 = strHex->charAt(i + 1);
    dataBuff[j] = parseCharHex(ch1, ch2);
    j++;
  }
  irrecv.disableIRIn();
  irSender.sendPack(nbits, (unsigned char *) &dataBuff, dataBuff_len);
  irrecv.enableIRIn();
}

bool arrayMatch(uint8_t arr1[], uint8_t arr2[], int arrLen) {
  bool result = true;

  for (int i = 0; i < arrLen; i++) {
    if (arr1[i] != arr2[i]) {
      result = false;
      break;
    }
  }
  return result;
}

void ir_action_print(decode_results * results) {
  udpLog.beginPrint();
  udpLog.print("{ IRBits: ");
  udpLog.print(results->bits);
  udpLog.print(", IRCode: \"");

  uint8_t b;
  int chunkSize = 0;
  int idxByte = 0;
  int bitNo = 0;
  while ((bitNo < results->bits) && (idxByte < results->rawlen)) {
    b = results->acPanasonicValue[idxByte];
    if (chunkSize == 0) {
      chunkSize = b;
      if (bitNo > 0) {
        udpLog.print("-");
      }
      if (chunkSize < 16) {
        udpLog.print("0");
      }
      udpLog.print((unsigned int) chunkSize, HEX);
      udpLog.print(":");
    } else {
      if (b < 16) {
        udpLog.print("0");
      }
      udpLog.print((unsigned int) b, HEX);
      if (chunkSize >= 8) {
        bitNo += 8;
        chunkSize -= 8;
      } else {
        bitNo += chunkSize;
        chunkSize = 0;
      }
    }
    idxByte++;
  }
  udpLog.print("\" }");
  udpLog.endPrint();
}

bool ir_action_acpanasonic2(decode_results *results) {
  bool result = false;
  uint8_t preSignal[] = { 0x40, 0x04, 0x07, 0x20, 0x00, 0x00, 0x00, 0x60};
  uint8_t signalEion[] = { 0x40, 0x04, 0x07, 0x20, 0x01, 0x86, 0xCC, 0x58};
  uint8_t signalPatrol[] = { 0x40, 0x04, 0x07, 0x20, 0x01, 0xC6, 0xCC, 0x38};
  uint8_t signalPowerfullQuite[] = { 0x40, 0x04, 0x07, 0x20, 0x01, 0x39, 0x4C, 0x2A};

  if (results->bits == 64) {
    if (arrayMatch(preSignal, &results->acPanasonicValue[1], 8)) {
      milisNextPanasonicSignal = millis() + 1500;
      return result;
    }
  }
  if (milisNextPanasonicSignal > millis()) {
    ir_action_print(results);
    if (results->bits == 64) {
      if (arrayMatch(signalEion, &results->acPanasonicValue[1], 8)) {
        udpLog.print("{ IRButton: \"Toggle E-ion\" }");
        result = true;
      } else if (arrayMatch(signalPatrol, &results->acPanasonicValue[1], 8)) {
        udpLog.print("{ IRButton: \"Toggle Patrol\" }");
        result = true;
      } else if (arrayMatch(signalPowerfullQuite, &results->acPanasonicValue[1], 8)) {
        udpLog.print("{ IRButton: \"Toggle Quit/Powerfull\" }");
        result = true;
      }
    } else if (results->bits == 152) {
      udpLog.beginPrint();
      uint8_t b;
      b = irSender.reverse_byte(results->acPanasonicValue[6]);
      udpLog.print("{ Power: "); // 1 or 0
      udpLog.print(b & B00000001);
      udpLog.print(", Mode: ");  // 2 (dry), 3 (cool), 0 (auto)
      udpLog.print((b & B11110000) >> 4);
      b = irSender.reverse_byte(results->acPanasonicValue[7]);
      udpLog.print(", Temp: ");  // 16 - 30
      udpLog.print((b & B00111110) >> 1);
      b = irSender.reverse_byte(results->acPanasonicValue[9]);
      udpLog.print(", Swing: ");  // 1, 2, 3, 4, 5, 15 (auto)
      udpLog.print(b & B00001111);
      udpLog.print(", FanSpeed: ");  // 3, 5, 7, 10 (auto)
      udpLog.print((b & B11110000) >> 4);
      udpLog.print(" } ");
      udpLog.endPrint();
      result = true;
    }
  }
  return result;
}

void ir_action_nec_light_anim() {
  nec_light |= B00111111;
  uint8_t light_Red_1_seq[] = {0x05, 0x19};
  uint8_t light_Green_1_seq[] = {0x00, 0x05, 0x05, 0x14};
  uint8_t light_Blue_1_seq[] = {0x00, 0x0A, 0x05, 0x0F};
  uint8_t light_Red_2_seq[] = {0x00, 0x0F, 0x05, 0x0A};
  uint8_t light_Green_2_seq[] = {0x00, 0x14, 0x05, 0x05};
  uint8_t light_Blue_2_seq[] = {0x00, 0x19, 0x05};

  light_Red_1.setSignal(light_Red_1_seq, 2);
  light_Green_1.setSignal(light_Green_1_seq, 4);
  light_Blue_1.setSignal(light_Blue_1_seq, 4);
  light_Red_2.setSignal(light_Red_2_seq, 4);
  light_Green_2.setSignal(light_Green_2_seq, 4);
  light_Blue_2.setSignal(light_Blue_2_seq, 3);
}

void ir_action_nec(uint32_t value) {
  if (value == 0xFFFFFFFF) {
    return;
  }
  udpLog.beginPrint();
  udpLog.print("{ necValue: 0x");
  udpLog.print(value, HEX);
  udpLog.print(", action: \"");

  switch (value) {
    case 0xFFB24D: // power
      udpLog.print("power on/off Relay 1");
      relay_portState_1 = (relay_portState_1 == LOW ? HIGH : LOW);
      relay_portDirty = true;
      break;
    case 0xFF2AD5: // source
      udpLog.print("power on/off Relay 2");
      relay_portState_2 = (relay_portState_2 == LOW ? HIGH : LOW);
      relay_portDirty = true;
      break;
    case 0xFF6897: // sound
      udpLog.print("send IR power on/off AC Panasonic");
      irrecv.disableIRIn();
      if (nec_light & B10000000) {
        nec_light &= B01111111;
        irSender.sendACPanasonic2(0, 3, 26, 1, 3);
      } else {
        nec_light |= B10000000;
        irSender.sendACPanasonic2(1, 3, 26, 1, 3);
      }
      irrecv.enableIRIn();
      break;
    case 0xFF32CD: // record
      udpLog.print("save state power on/off Relay");
      saveConfig();
      break;
    case 0xFFA05F:  // CH+
      udpLog.print("test IR AC Panasonic");
      irrecv.disableIRIn();
      irSender.sendTestCancelTimer2();
      irrecv.enableIRIn();
      break;
    case 0xFF30CF: // time shift
      udpLog.print("Soft Reset");
      disableDevices();
      os_timer_disarm(&led_timer);
      ESP.reset();
      break;
    case 0xFF48B7: // 0
      udpLog.print("turn off all light");
      nec_light &= B11000000;
      light_setHexSignal(&light_Red_1, &nec_light_off);
      light_setHexSignal(&light_Green_1, &nec_light_off);
      light_setHexSignal(&light_Blue_1, &nec_light_off);
      light_setHexSignal(&light_Red_2, &nec_light_off);
      light_setHexSignal(&light_Green_2, &nec_light_off);
      light_setHexSignal(&light_Blue_2, &nec_light_off);
      break;
    case 0xFF38C7: // recall
      udpLog.print("turn on animation light");
      ir_action_nec_light_anim();
      break;
    case 0xFF906F: // 1
      udpLog.print("turn on/blink/off light Red 1");
      if (nec_light & B00000001) {
        nec_light &= B11111110;
        light_setHexSignal(&light_Red_1, &nec_light_off);
      } else {
        nec_light |= B00000001;
        light_setHexSignal(&light_Red_1, &nec_light_on);
      }
      break;
    case 0xFFB847: // 2
      udpLog.print("turn on/blink/off light Green 1");
      if (nec_light & B00000010) {
        nec_light &= B11111101;
        light_setHexSignal(&light_Green_1, &nec_light_off);
      } else {
        nec_light |= B00000010;
        light_setHexSignal(&light_Green_1, &nec_light_on);
      }
      break;
    case 0xFFF807: // 3
      udpLog.print("turn on/blink/off light Blue 1");
      if (nec_light & B00000100) {
        nec_light &= B11111011;
        light_setHexSignal(&light_Blue_1, &nec_light_off);
      } else {
        nec_light |= B00000100;
        light_setHexSignal(&light_Blue_1, &nec_light_on);
      }
      break;
    case 0xFFB04F: // 4
      udpLog.print("turn on/blink/off light Red 2");
      if (nec_light & B00001000) {
        nec_light &= B11110111;
        light_setHexSignal(&light_Red_2, &nec_light_off);
      } else {
        nec_light |= B00001000;
        light_setHexSignal(&light_Red_2, &nec_light_on);
      }
      break;
    case 0xFF9867: // 5
      udpLog.print("turn on/blink/off light Green 2");
      if (nec_light & B00010000) {
        nec_light &= B11101111;
        light_setHexSignal(&light_Green_2, &nec_light_off);
      } else {
        nec_light |= B00010000;
        light_setHexSignal(&light_Green_2, &nec_light_on);
      }
      break;
    case 0xFFD827: // 6
      udpLog.print("turn on/blink/off light Blue 2");
      if (nec_light & B00100000) {
        nec_light &= B11011111;
        light_setHexSignal(&light_Blue_2, &nec_light_off);
      } else {
        nec_light |= B00100000;
        light_setHexSignal(&light_Blue_2, &nec_light_on);
      }
      break;
    case 0xFFA857: // 8
      udpLog.print("turn on/off AC Panasonic and blue 1 & 2 light");
      irrecv.disableIRIn();
      if (nec_light & B10000000) {
        nec_light &= B01011011;
        irSender.sendACPanasonic2(0, 3, 26, 1, 3);
        light_setHexSignal(&light_Blue_1, &nec_light_off);
        light_setHexSignal(&light_Blue_2, &nec_light_off);
      } else {
        nec_light |= B10100100;
        irSender.sendACPanasonic2(1, 3, 26, 1, 3);
        light_setHexSignal(&light_Blue_1, &nec_light_on);
        light_setHexSignal(&light_Blue_2, &nec_light_on);
      }
      irrecv.enableIRIn();
      break;
    default:
      udpLog.print("none");
  }
  udpLog.print("\" }");
  udpLog.endPrint();
}

uint8_t ir_action(decode_results * results) {
  uint8_t result = 0;

  if (results->decode_type == ACPANASONIC) {
    ir_action_print(results);
    result = 1;
  } else if (results->decode_type == ACPANASONIC2) {
    if (ir_action_acpanasonic2(results)) {
      result = 1;
    }
  } else if (results->decode_type == NEC) {
    ir_action_nec(results->value);
    result = 1;
  } else if (results->decode_type > 0) {
    udpLog.beginPrint();
    udpLog.print("{ Unknown: ");
    udpLog.print(results->decode_type);
    udpLog.print(", len: ");
    udpLog.print(results->rawlen);
    udpLog.print(", value: \"");
    udpLog.print(results->value, HEX);
    //    dump unkonwn IR Code
    //    for (int i = 0; i < results->rawlen; i++) {
    //      if (i % 64 == 0) {
    //        udpLog.print("\" }");
    //        udpLog.endPrint();
    //        udpLog.beginPrint();
    //        udpLog.print("{ IRCode: \"");
    //      }
    //      udpLog.print((unsigned long) results->rawbuf[i]*USECPERTICK);
    //      udpLog.print("-");
    //    }
    udpLog.print("\" }");
    udpLog.endPrint();
  }
  return result;
}

void report_loop() {
  // report to central
  udpLog.beginPrint();
  udpLog.print("{ ");
  udpLog.print("R_1: ");
  udpLog.print((char) (relay_portState_1 + '0'));
  udpLog.print(", R_2: ");
  udpLog.print((char) (relay_portState_2 + '0'));

  float t =  dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(h)) {
    udpLog.print(", Hum: ");
    udpLog.print(h, 2);
  }
  if (!isnan(t)) {
    udpLog.print(", Temp: ");
    udpLog.print(t, 2);
  }
  if (!(isnan(t) || isnan(h))) {
    float hic = dht.computeHeatIndex(t, h);
    udpLog.print(", HIdx: ");
    udpLog.print(hic, 2);
  }
  if (irTestEnable) {
    udpLog.print(", IRTest: 1");
  }
  udpLog.print(" }");
  udpLog.endPrint();
}

void relay_loop() {
  if (relay_portDirty) {
    relay_portDirty = false;
    mcp.digitalWrite(PIN_RELAY_1, relay_portState_1);
    mcp.digitalWrite(PIN_RELAY_2, relay_portState_2);
  }
}

void light_loop() {
  light_Red_1.doAction();
  light_Green_1.doAction();
  light_Blue_1.doAction();
  light_Red_2.doAction();
  light_Green_2.doAction();
  light_Blue_2.doAction();
}

void light_setHexSignal(Light * light, String * strHex) {
  uint8_t dataBuff[16];
  int dataBuff_len;
  unsigned char ch1, ch2;

  dataBuff_len = 0;
  for (int i = 0; i < strHex->length(); i += 2) {
    ch1 = strHex->charAt(i);
    ch2 = strHex->charAt(i + 1);
    dataBuff[dataBuff_len] = parseCharHex(ch1, ch2);
    dataBuff_len++;
  }
  light->setSignal(dataBuff, dataBuff_len);
}

void lightTest() {
  uint8_t light_Red_1_seq[] = {5, 25};
  uint8_t light_Green_1_seq[] = {0, 5, 5, 20};
  uint8_t light_Blue_1_seq[] = {0, 10, 5, 15};
  uint8_t light_Red_2_seq[] = {0, 15, 5, 10};
  uint8_t light_Green_2_seq[] = {0, 20, 5, 5};
  uint8_t light_Blue_2_seq[] = {0, 25, 5};

  light_Red_1.setSignal(light_Red_1_seq, 2);
  light_Green_1.setSignal(light_Green_1_seq, 4);
  light_Blue_1.setSignal(light_Blue_1_seq, 4);
  light_Red_2.setSignal(light_Red_2_seq, 4);
  light_Green_2.setSignal(light_Green_2_seq, 4);
  light_Blue_2.setSignal(light_Blue_2_seq, 3);
}

void irTest() {
  irrecv.disableIRIn();
  irSender.sendTestCancelTimer2();
  irrecv.enableIRIn();
}

void loop100ms() {
  relay_loop();
  light_loop();
  if (loop_cycle % 50 == 0) {
    report_loop();
    if (irTestEnable) {
      irTest();
    }
  }
}

void loop() {
  if (ota_update == 1) {
    yield;
    return;
  }

  checkWifi();

  if (loop_lastTrapTime < millis() - 100) {
    loop_lastTrapTime = millis();
    if (isWifiConnected) {
      loop_cycle++;
      if (loop_led_on > 0) {
        if (loop_led_on == 1) {
          led_init(1UL);
        }
        loop_led_on--;
      }
      loop100ms();
    }
  }

  decode_results  ir_results;
  if (irrecv.decode(&ir_results)) {
    if (loop_led_on == 0) {
      if (ir_action(&ir_results)) {
        led_init(0);
        loop_led_on = 5;
      }
    }
    irrecv.resume();
  }

  yield();
}

