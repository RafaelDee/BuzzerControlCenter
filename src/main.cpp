/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#include <espnow.h>

#define BOARD "ESP8266"
// ESP8266 specific defines can go here
#elif defined(ESP32)
#include <WiFi.h>
#include <esp_now.h>
#include "nvs.h"
#include "nvs_flash.h"
#include <Preferences.h>
#define BOARD "ESP32"
// ESP32 specific defines can go here
#else
#error "This code is intended for ESP8266 or ESP32 boards only!"
#endif

#include <../../commonlib/networking.h>
#include <map>
#include <sstream>
#include <clock.h>
#include <serialQueueManager.h>
// Create a struct_message called myData
typedef struct PodiumData
{
  int dnrCount = 0;
  int position = -1;
  bool isButtonPressed = false;
  /**what rank the button got pressed */
  int buttonPlacement = -1;
  uint8_t battLevel = 0;
  uint16_t battVoltage = 0;
  bool isCharging = false;
  DnrSeverity dnrSeverity = DnrSeverity::ok;
  PodiumData() {};
  PodiumData(int position) : position(position) {};
} PodiumData;
const unsigned int dnrMaxcount = 2;
boolean paired = false;
Preferences preferences;
// cahce it! cpu is bad at dividing
const unsigned int dnrHalfcount = dnrMaxcount / 2;
DnrSeverity getPodiumDnrSeverity(PodiumData podium)
{
  if (podium.dnrCount > dnrMaxcount)
  {
    return DnrSeverity::disconnected;
  }
  else if (podium.dnrCount > dnrHalfcount)
  {
    return DnrSeverity::slow;
  }
  return DnrSeverity::ok;
};
String arrayToString(const std::array<uint8_t, 6> &arr)
{
  char buffer[18]; // "FF:AA:BB:CC:DD:EE\0" (17 chars + null terminator)
  sprintf(buffer, "%02X:%02X:%02X:%02X:%02X:%02X",
          arr[0], arr[1], arr[2], arr[3], arr[4], arr[5]);
  return String(buffer);
}
String pairKey = "c8e89336-1cbb-4028-b1fe-0079c00d6b4f";
std::map<std::array<uint8_t, 6>, PodiumData> podiumsMap;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t podiumBrightnessFce = 175;
uint8_t podiumBrightnessBtn = 175;
enum class GameState
{
  // used when nothing is happening, resets the podium interface
  Idle,
  // use to lock podium actions, podium light status is kept
  Lock,
  // ready for podium actions
  QuizReady,
  // button as been pressed, spotlights the podium
  QuizAnswered,
  CorrectAns,
  WrongAns,
  SuspenseAns,
  OffAllPodium,
  Spotlight
};

Networking networking;
uint8_t buttonPlacement = 0;
SerialQueueManager serialManager;
unsigned long dnrTickDuration = Networking::heartbeatDuration * 2;
int podiumAnsweredFirst = -1;
template <typename T>
void sendToAll(T packet)
{
  for (const auto& [macAddr, podium] : podiumsMap) {
      networking.sendPacket(macAddr.data(), packet);
      delay(1); // Small delay to prevent packet collisions
  }
}

void SendToUI(serialSendCommands command, uint32_t responseToken)
{
  serialManager.queueMessage({static_cast<int>(command)}, responseToken);
}

template <typename... Args>
void SendToUI(serialSendCommands command, uint32_t responseToken, Args... args)
{
  if (!paired && !(command == serialSendCommands::Pair || command == serialSendCommands::Ready))
    return;

  String result;
  bool first = true;
  ((result += (first ? "" : ","),
    result += (args),
    first = false),
   ...);
  Serial.print("Sending:");
  Serial.println(result);
  serialManager.queueMessage(static_cast<int>(command), responseToken, result);
}
uint8_t spottedPodium = NO_PIN;
GameState currentGameState = GameState::Idle;
void setCurrentGameState(GameState state)
{
  currentGameState = state;
  int castedState = static_cast<int>(currentGameState);
  if (castedState == NAN)
    return;
  Serial.print("GAME STATE: ");
  Serial.print(castedState);
  SendToUI(serialSendCommands::GameState, NO_TOKEN, castedState);
}

void suspenseAnswer()
{
  Serial.println("SUSPENSE");
  sendToAll(SuspenseAnsPacket());
  /* for (std::map<std::array<uint8_t, 6>, PodiumData>::iterator it = podiumsMap.begin(); it != podiumsMap.end(); ++it)
  {
    //if (it->second.dnrSeverity == DnrSeverity::disconnected)
    //  continue;
    try
    {
      networking.sendPacket(it->first.data(), sns);
    }
    catch (const std::exception &e)
    {
      Serial.print("Send failed: ");
      Serial.println(e.what());
    }
  } */
}
void addPeer(const std::array<uint8_t, 6> mac_addr, bool catchUpServer, bool saveToMem = true);

void initializeClient(const std::array<uint8_t, 6> mac_addr, bool saveToMem = true)
{
  // get and find what position is this client
  // TODO: send button game state to catch up
  Serial.println("Initializing Client");
  auto it = podiumsMap.find(mac_addr);
  if (it == podiumsMap.end())
  {
    Serial.println("CLIENT INITIALIZING NOT ADDED");
    addPeer(mac_addr, false, saveToMem);
    initializeClient(mac_addr, saveToMem);
    return;
  }
  Serial.print("Client Initialized with podium position:");
  Serial.println(it->second.position);
  auto macAddr = mac_addr.data();
  SendToUI(serialSendCommands::PodiumAdded, NO_TOKEN, it->second.position, static_cast<int>(it->second.dnrSeverity), arrayToString(it->first));
  networking.sendPacket<InitPacket>(macAddr, InitPacket(it->second.position, it->second.isButtonPressed));
  networking.sendPacket<LedBrightnessPacket>(macAddr, LedBrightnessPacket(podiumBrightnessFce, podiumBrightnessBtn));
  Serial.print("SENDING GAME STATE:");
  Serial.println((int)currentGameState);
  switch (currentGameState)
  {
  case GameState::SuspenseAns:
    suspenseAnswer();
    networking.sendPacket(macAddr, SpotlightPacket(podiumAnsweredFirst, false));
    /* code */
    break;
  case GameState::QuizAnswered:
    networking.sendPacket(macAddr, SpotlightPacket(podiumAnsweredFirst, false));
    /* code */
    break;
  case GameState::Spotlight:
    networking.sendPacket(macAddr, SpotlightPacket(spottedPodium, false));
    /* code */
    break;
  case GameState::CorrectAns:
    networking.sendPacket(macAddr, CorrectAnsPacket());
    break;
  case GameState::WrongAns:
    networking.sendPacket(macAddr, WrongAnsPacket());
    break;
  case GameState::OffAllPodium:
    networking.sendPacket(macAddr, LedBrightnessPacket(0, 0));
    break;
  default:
    break;
  }
}
void RunDnr()
{
  for (auto &[macAddr, podium] : podiumsMap)
  {
    DnrSeverity dnrSeverity = getPodiumDnrSeverity(podium);
    if (dnrSeverity != podium.dnrSeverity)
    {
      podium.dnrSeverity = dnrSeverity;
      SendToUI(serialSendCommands::Dnr, NO_TOKEN, podium.position, dnrSeverity);
    }

    if (podium.dnrCount <= dnrMaxcount)
    {
      podium.dnrCount += 1;
    }
  }
}

void savePodiumOrder()
{
  String podiumOrder = "";
  bool first = true;
  std::array<uint8_t, 6U> test[podiumsMap.size()];
  for (const auto &[macAddr, podium] : podiumsMap)
  {
    test[podium.position] = macAddr;
  }
  for (const std::array<uint8_t, 6U> podMacAddr : test)
  {
    podiumOrder += (first ? "" : ",");
    podiumOrder += arrayToString(podMacAddr);
    first = false;
  }
  Serial.print("SAVING: ");
  Serial.println(podiumOrder);
  preferences.putString("order", podiumOrder);
}
void addPeer(const std::array<uint8_t, 6> mac_addr, bool catchUpServer, bool saveToMem)
{
  if (podiumsMap.count(mac_addr) > 0)
    return;
  Serial.println("ADDING PEER TO NETWORK");
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac_addr.data(), 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  int index = podiumsMap.size();
  podiumsMap[mac_addr] = PodiumData(index);
  if (saveToMem)
    savePodiumOrder();
  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  // catch up when Server rebooted
  if (catchUpServer)
    initializeClient(mac_addr);
  SendToUI(serialSendCommands::Dnr, NO_TOKEN, podiumsMap[mac_addr].position, (int)DnrSeverity::ok);
  Serial.print("Sucessfully added peer with position: ");
  Serial.println(podiumsMap[mac_addr].position);
}

void DeviceResponded(std::array<uint8_t, 6> mac_addr)
{
  if (podiumsMap.count(mac_addr) <= 0)
    return;
  int respCount = (podiumsMap[mac_addr]).dnrCount - 1;
  // check if device came back from DNR and set count to half the dnrMaxcount;
  if (respCount <= 0)
    respCount = 0;
  // ik that it will negative if given a chance, but unlikely
  if (respCount >= dnrHalfcount)
  {
    Serial.println("Device Came Back!");
    respCount = dnrHalfcount;
    podiumsMap[mac_addr].dnrSeverity = DnrSeverity::ok;
    SendToUI(serialSendCommands::Dnr, NO_TOKEN, podiumsMap[mac_addr].position, (int)DnrSeverity::ok);
  }
  podiumsMap[mac_addr].dnrCount = respCount;
}
void spotLightPodium(uint8_t index, boolean flash)
{
  if (index < 0 || index >= podiumsMap.size())
    return;
  // send to all
  Serial.print("sending spotlight to: ");
  Serial.println(index);
  spottedPodium = index;
  sendToAll(SpotlightPacket(index, flash));
}
// callback function that will be executed when data is received

void buttonPressed(std::array<uint8_t, 6> mac_addr)
{
  if (!(currentGameState == GameState::QuizReady || currentGameState == GameState::QuizAnswered))
    return;

  int btnIndex = podiumsMap[mac_addr].position;
  if (podiumsMap[mac_addr].isButtonPressed)
    return;
  SendToUI(serialSendCommands::PodiumPlacement, NO_TOKEN, btnIndex, buttonPlacement);
  if (buttonPlacement == 0 && currentGameState == GameState::QuizReady)
  {
    spotLightPodium(btnIndex, true);
    podiumAnsweredFirst = btnIndex;
  }
  podiumsMap[mac_addr].isButtonPressed = true;
  podiumsMap[mac_addr].buttonPlacement = buttonPlacement;
  // record button order
  setCurrentGameState(GameState::QuizAnswered);
  buttonPlacement++;
}
void correctAnswer(uint32_t responseToken)
{
  sendToAll(CorrectAnsPacket());
  SendToUI(serialSendCommands::Acknowledge, responseToken);
}
void movePodium(uint32_t responseToken, String payload)
{
  SendToUI(serialSendCommands::Acknowledge, responseToken);
  int firstComma = payload.indexOf(',');
  int currentIndex = payload.substring(0, firstComma).toInt();
  int targetIndex = payload.substring(firstComma + 1).toInt();
  // shift left, decrement if true
  bool shiftLeft = currentIndex < targetIndex;
  if (currentIndex < 0 || targetIndex < 0 || currentIndex >= podiumsMap.size() || targetIndex >= podiumsMap.size())
    return;
  // move spotlight when focused
  if (spottedPodium == currentIndex)
    spottedPodium = targetIndex;
  /* std::array<uint8_t, 6> macA;
  for (auto &[macAddr, podium] : podiumsMap)
  {
    if (podium.position == currentIndex)
      macA = macAddr;
  } */
  for (std::map<std::array<uint8_t, 6>, PodiumData>::iterator it = podiumsMap.begin(); it != podiumsMap.end(); ++it)
  {
    // check if podium position is equal to targetIndex

    // increment position of all podiums that are greater than targetIndex
    auto macAddr = it->first;
    if (it->second.position == currentIndex)
    {
      it->second.position = targetIndex;
    }
    else if (!shiftLeft && it->second.position >= targetIndex && it->second.position < currentIndex)
    {
      it->second.position++;
    }
    else if (shiftLeft && it->second.position <= targetIndex && it->second.position > currentIndex)
    {
      it->second.position--;
    }
  }
  for (std::map<std::array<uint8_t, 6>, PodiumData>::iterator it = podiumsMap.begin(); it != podiumsMap.end(); ++it)
  {
    networking.sendPacket(it->first.data(), InitPacket(it->second.position, it->second.isButtonPressed));
    SendToUI(serialSendCommands::PodiumAdded, NO_TOKEN, it->second.position, static_cast<int>(it->second.dnrSeverity), arrayToString(it->first));
  }
  savePodiumOrder();

  // Notify the UI about the swap
}

void wrongAnswer(uint32_t responseToken)
{
  sendToAll(WrongAnsPacket());
  SendToUI(serialSendCommands::Acknowledge, responseToken);
}
void updatePodiumBattery(std::array<uint8_t, 6> &mac_addr, BatteryStatPacket &bsp)
{
  PodiumData &podium = podiumsMap[mac_addr];
  podium.battLevel = bsp.level;
  podium.battVoltage = bsp.voltage;
  podium.isCharging = bsp.charging;
  SendToUI(serialSendCommands::BattStat, NO_TOKEN, podium.position, bsp.level, bsp.voltage, bsp.charging);
}
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, const int len)
{
  std::array<uint8_t, 6> macAddr;
  std::copy(mac, mac + 6, macAddr.begin());
  clientPacketType type = networking.getPacketType<clientPacketType>(incomingData);
  // catch up if packetType is other than Initialization
  bool catchUpRequired = type != clientPacketType::Initialization;
  addPeer(macAddr, catchUpRequired);
  switch (type)
  {
  case clientPacketType::ButtonPress:
  {
    buttonPressed(macAddr);
    break;
  }
  case clientPacketType::Initialization:
  {
    Serial.println("DEVICE NEEDS INIT");
    initializeClient(macAddr);
    break;
  }
  case clientPacketType::BattStat:
  {
    BatteryStatPacket bsp = networking.receivePacket<BatteryStatPacket>(mac, incomingData, len);
    updatePodiumBattery(macAddr, bsp);
    break;
  }
  case clientPacketType::Heartbeat:
  {
    DeviceResponded(macAddr);
    /*     HeartbeatPacket hbp = networking.receivePacket<HeartbeatPacket>(mac, incomingData, len);
        Serial.print("Strength: ");
        Serial.print((int)hbp.signalStrength);
        Serial.println("dBm"); */

    // create DNR(Device not responding) system
    break;
  }
  default:
    break;
  }
}

void OnDataSent(const uint8_t *mac_addr, const esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success: " : "Delivery Fail: ");
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
}
void resetGameState(uint32_t responseToken)
{
  podiumAnsweredFirst = -1;
  buttonPlacement = 0;
  spottedPodium = NO_PIN;
  sendToAll(ResetStatePacket());
  /* for (std::map<std::array<uint8_t, 6>, PodiumData>::iterator it = podiumsMap.begin(); it != podiumsMap.end(); ++it)
  {
    it->second.isButtonPressed = false;
    networking.sendPacket(it->first.data(), );
  } */
  SendToUI(serialSendCommands::Acknowledge, responseToken);
}
std::array<uint8_t, 6> parseCharToArray(const char *input)
{
  std::array<uint8_t, 6> arr = {0}; // Initialize with zeros
  int index = 0;

  char buffer[strlen(input) + 1]; // Copy to avoid modifying original
  strcpy(buffer, input);

  char *token = strtok(buffer, ":"); // Split by ':' or ','
  while (token != NULL && index < 6)
  {
    arr[index++] = static_cast<uint8_t>(strtol(token, NULL, 16)); // Convert hex to uint8_t
    token = strtok(NULL, ":");                                    // Get next token
  }

  return arr;
}
void initPodiums(String storedValue)
{
  Serial.print("SAVED: ");
  Serial.println(storedValue);

  int start = 0;
  int index = 0;

  while (start < storedValue.length())
  {
    int end = storedValue.indexOf(",", start);
    if (end == -1)
    {
      end = storedValue.length();
    }

    String token = storedValue.substring(start, end);
    const std::array<uint8_t, 6U> macArray = parseCharToArray(token.c_str());
    Serial.print("init mac: ");
    Serial.println(token);
    addPeer(macArray, false, false);

    index++;
    start = end + 1;
  }
}

void setup()
{
  // Initialize Serial Monitor
  Serial.begin(115200);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  if (!preferences.begin("podiums", false))
  {
    Serial.println("Failed to initialize Preferences");
    return;
  }
  podiumBrightnessFce = preferences.getUInt("brightnessFce", 175);
  podiumBrightnessBtn = preferences.getUInt("brightnessBtn", 175);
  String storedPodiums = preferences.getString("order");
  Serial.printf("Mac Addresses: %d\n",
                storedPodiums);
  initPodiums(storedPodiums);
  nvs_stats_t nvs_stats;
  nvs_get_stats(NULL, &nvs_stats);
  Serial.printf("Used Entries: %d, Free Entries: %d, Total Entries: %d\n",
                nvs_stats.used_entries, nvs_stats.free_entries, nvs_stats.total_entries);
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  while (!Serial)
  {
  }

  SendToUI(serialSendCommands::Ready, NO_TOKEN);
  resetGameState(NO_TOKEN);
}

void DnrTick()
{
  static unsigned long lastTime = 0;
  static int sendAllTick = 0;
  if (!Clock::TimePassed(lastTime, dnrTickDuration, true))
    return;
  RunDnr();
}

void showSummary(uint32_t responseToken)
{
  SendToUI(serialSendCommands::Acknowledge, responseToken);
  paired = true;
  for (const auto &[macAddr, podium] : podiumsMap)
  {
    SendToUI(serialSendCommands::PodiumAdded, NO_TOKEN, podium.position, podium.dnrCount, arrayToString(macAddr));
    SendToUI(serialSendCommands::Dnr, NO_TOKEN, podium.position, static_cast<int>(podium.dnrSeverity));
    // SendToUI(serialSendCommands::BattStat, NO_TOKEN, podium.position, podium.battLevel, podium.battVoltage, podium.isCharging);
  }
  SendToUI(serialSendCommands::PodiumBrightness, NO_TOKEN, podiumBrightnessFce, podiumBrightnessBtn);
  if (currentGameState == GameState::QuizAnswered)
  {
    for (const auto &[macAddr, podium] : podiumsMap)
    {
      if (podium.position == -1 || podium.buttonPlacement == -1)
        continue;
      SendToUI(serialSendCommands::PodiumPlacement, NO_TOKEN, podium.position, podium.buttonPlacement);
    }
  }
  else if (currentGameState == GameState::Spotlight)
  {
    SendToUI(serialSendCommands::SpotLightPodium, NO_TOKEN, spottedPodium);
  }
  else
  {
    SendToUI(serialSendCommands::GameState, NO_TOKEN, static_cast<int>(currentGameState));
  }
}
void pairToUI(uint32_t responseToken)
{
  Serial.println("Pairing");
  SendToUI(serialSendCommands::Pair, responseToken, pairKey);
  Serial.println(WiFi.macAddress());
}
void setPodiumBrightness(uint8_t face, uint8_t btn, std::array<uint8_t, 6> macAddr = {})
{

  face = constrain(face, 0, 255);
  btn = constrain(btn, 0, 255);
  podiumBrightnessFce = face;
  podiumBrightnessBtn = btn;
  preferences.putUInt("brightnessFce", face);
  preferences.putUInt("brightnessBtn", btn);
  sendToAll(LedBrightnessPacket(face, btn));
  /* if (macAddr.empty())
  {
    // handle default case

    LedBrightnessPacket wns = LedBrightnessPacket(face, btn);
    sendToAll(&wns);
    return;
  } */
  /* for (const auto &[macAddr, podium] : podiumsMap)
  {
    SendToUI(serialSendCommands::PodiumAdded, NO_TOKEN, podium.position, podium.dnrCount,arrayToString(macAddr));
    SendToUI(serialSendCommands::Dnr, NO_TOKEN, podium.position, static_cast<int>(podium.dnrSeverity));
  } */
}
void ClearPodium()
{
  preferences.putString("order", "");
  podiumsMap.clear();
  ESP.restart();
}
void parseBrightness(String payload)
{
  int firstComma = payload.indexOf(',');
  int fceBright = payload.substring(0, firstComma).toInt();
  int btnBright = payload.substring(firstComma + 1).toInt();
  setPodiumBrightness(fceBright, btnBright, {});
}
void processUICommand(Command command)
{

  switch (command.command)
  {
  case (int)serialReceiveCommands::Pair:
    pairToUI(command.idempotencyToken);
    /* code */
    break;
  case (int)serialReceiveCommands::ResetGameState:
    resetGameState(command.idempotencyToken);
    setCurrentGameState(GameState::Idle);
    break;
  case (int)serialReceiveCommands::ReadyGameState:
    resetGameState(command.idempotencyToken);
    setCurrentGameState(GameState::QuizReady);
    break;
  case (int)serialReceiveCommands::Summary:
    showSummary(command.idempotencyToken);
    break;
  case (int)serialReceiveCommands::CorrectAnswer:
    setCurrentGameState(GameState::CorrectAns);
    correctAnswer(command.idempotencyToken);
    break;
  case (int)serialReceiveCommands::WrongAnswer:
    setCurrentGameState(GameState::WrongAns);
    wrongAnswer(command.idempotencyToken);
    break;
  case (int)serialReceiveCommands::SwapPodium:
    if (!Networking::hasPayloadLength(command.payload, 2))
      return;
    movePodium(command.idempotencyToken, command.payload);
    break;
  case (int)serialReceiveCommands::SuspenseGame:
    setCurrentGameState(GameState::SuspenseAns);
    suspenseAnswer();
    SendToUI(serialSendCommands::Acknowledge, command.idempotencyToken);
    break;
  case (int)serialReceiveCommands::SpotLightPodium:
    if (!Networking::hasPayloadLength(command.payload, 1))
      return;
    SendToUI(serialSendCommands::Acknowledge, command.idempotencyToken);
    spotLightPodium(command.payload.toInt(), false);
    setCurrentGameState(GameState::Spotlight);
    break;
  case (int)serialReceiveCommands::ClearPodium:
    SendToUI(serialSendCommands::Acknowledge, command.idempotencyToken);
    ClearPodium();
    break;
  case (int)serialReceiveCommands::PodiumBrightness:
    if (!Networking::hasPayloadLength(command.payload, 2))
      return;

    SendToUI(serialSendCommands::Acknowledge, command.idempotencyToken);
    parseBrightness(command.payload);
    return;
    break;
    /* default:
      Serial.println("NOT A COMMAND");
      break; */
  }
}

void readSerial()
{
  serialManager.update();
  while (serialManager.available())
  {
    Command incomingCommand = serialManager.readCommand();

    // End of command, process it
    processUICommand(incomingCommand);
    delay(1); // Add a small delay to yield control
  }
}
// TODO: UNUSED, can be used to manually check batt percentages
void checkBattLevels()
{
  // BattPingPacket wns = ;
  sendToAll(BattPingPacket());
}
void loop()
{
  readSerial();
  DnrTick();
}