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
  int dnrCount;
  int position = -1;
  bool isButtonPressed = false;
  int buttonPlacement = 0;
  uint8_t battLevel = 0;
  uint16_t battVoltage = 0;
  bool isCharging = false;
  DnrSeverity dnrSeverity = DnrSeverity::ok;
  PodiumData() : dnrCount(0) {};
  PodiumData(int position) : dnrCount(0), position(position) {};
} PodiumData;
const unsigned int dnrMaxcount = 2;

boolean paired = false;
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
String pairKey = "c8e89336-1cbb-4028-b1fe-0079c00d6b4f";
std::map<std::array<uint8_t, 6>, PodiumData> podiumsMap;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

enum class GameState
{
  // used when nothing is happening, resets the podium interface
  Idle,
  // use to lock podium actions, podium interface can be modified
  Lock,
  // ready for podium actions
  QuizReady,
  // button as been pressed, spotlights the podium
  QuizAnswered
};

Networking networking;
uint8_t buttonPlacement = 0;
SerialQueueManager serialManager;
unsigned long dnrTickDuration = Networking::heartbeatDuration * 2;
int podiumAnsweredFirst = -1;
template <typename T>
void sendToAll(T *packet)
{
  for (std::map<std::array<uint8_t, 6>, PodiumData>::iterator it = podiumsMap.begin(); it != podiumsMap.end(); ++it)
  {
    networking.sendPacket(it->first.data(), *packet);
  }
}
GameState currentGameState = GameState::Idle;
void SendToUI(serialSendCommands command)
{
  serialManager.queueMessage({static_cast<int>(command)});
}

template <typename... Args>
void SendToUI(serialSendCommands command, Args... args)
{
  if (!paired && !(command == serialSendCommands::Pair || command == serialSendCommands::Ready))
    return;
  Serial.println("Sending");
  String result;
  bool first = true;
  ((result += (first ? "" : ","),
    result += (args),
    first = false),
   ...);
  serialManager.queueMessage(static_cast<int>(command), result);
}
bool hasPayloadLength(Command command, int targetLen)
{
  bool valid = command.payload.length() == targetLen;
  if (!valid)
  {
    Serial.print("Invalid Command Size: actiual size");
    Serial.println(command.payload.length());
  }
  return valid;
}
// not good?
void addPeer(const std::array<uint8_t, 6> mac_addr, bool catchUpServer);
void initializeClient(const std::array<uint8_t, 6> mac_addr)
{
  // get and find what position is this client
  // TODO: send button game state to catch up
  Serial.println("Initializing Client");
  auto it = podiumsMap.find(mac_addr);
  if (it == podiumsMap.end())
  {
    Serial.println("CLIENT INITIALIZING NOT ADDED");
    addPeer(mac_addr, false);
    initializeClient(mac_addr);
    return;
  }
  Serial.print("Client Initialized with podium position:");
  Serial.println(it->second.position);

  networking.sendPacket<InitPacket>(mac_addr.data(), InitPacket(it->second.position, it->second.isButtonPressed));
}
void RunDnr()
{
  for (auto &[macAddr, podium] : podiumsMap)
  {
    DnrSeverity dnrSeverity = getPodiumDnrSeverity(podium);
    if (dnrSeverity != podium.dnrSeverity)
    {
      podium.dnrSeverity = dnrSeverity;
      SendToUI(serialSendCommands::Dnr, podium.position, dnrSeverity);
    }

    if (podium.dnrCount <= dnrMaxcount)
    {
      podium.dnrCount += 1;
    }
  }
}
void addPeer(const std::array<uint8_t, 6> mac_addr, bool catchUpServer)
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
  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  // catch up when Server rebooted
  if (catchUpServer)
    initializeClient(mac_addr);
  SendToUI(serialSendCommands::Dnr, podiumsMap[mac_addr].position, (int)DnrSeverity::ok);
  Serial.print("Sucessfully added peer with index: ");
  Serial.println(index);
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
    SendToUI(serialSendCommands::Dnr, podiumsMap[mac_addr].position, (int)DnrSeverity::ok);
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
  SpotlightPacket spm = SpotlightPacket(index, flash);
  sendToAll(&spm);
}
// callback function that will be executed when data is received

void buttonPressed(std::array<uint8_t, 6> mac_addr)
{
  if (!(currentGameState == GameState::QuizReady || currentGameState == GameState::QuizAnswered))
    return;

  int btnIndex = podiumsMap[mac_addr].position;
  if (podiumsMap[mac_addr].isButtonPressed)
    return;
  SendToUI(serialSendCommands::PodiumPlacement, btnIndex, buttonPlacement);
  if (buttonPlacement == 0 && currentGameState == GameState::QuizReady)
  {
    spotLightPodium(btnIndex, true);
    podiumAnsweredFirst = btnIndex;
  }
  podiumsMap[mac_addr].isButtonPressed = true;
  podiumsMap[mac_addr].buttonPlacement = buttonPlacement;
  // record button order
  currentGameState = GameState::QuizAnswered;
  buttonPlacement++;
}
void correctAnswer()
{
  CorrectAnsPacket cns = CorrectAnsPacket();
  sendToAll(&cns);
}
void swapPodium(int podAindex, int podBindex)
{
  if (podAindex < 0 || podBindex < 0 || podAindex >= podiumsMap.size() || podBindex >= podiumsMap.size())
    return;

  std::array<uint8_t, 6> macA, macB;
  for (auto &[macAddr, podium] : podiumsMap)
  {
    if (podium.position == podAindex)
      macA = macAddr;
    else if (podium.position == podBindex)
      macB = macAddr;
  }

  if (macA == macB)
    return;

  std::swap(podiumsMap[macA].position, podiumsMap[macB].position);

  // Notify the clients about the swap
  networking.sendPacket(macA.data(), InitPacket(podiumsMap[macA].position, podiumsMap[macA].isButtonPressed));
  networking.sendPacket(macB.data(), InitPacket(podiumsMap[macB].position, podiumsMap[macB].isButtonPressed));

  // Notify the UI about the swap
  SendToUI(serialSendCommands::PodiumAdded, podiumsMap[macA].position, podiumsMap[macA].dnrCount);
  SendToUI(serialSendCommands::PodiumAdded, podiumsMap[macB].position, podiumsMap[macB].dnrCount);
}
void suspenseAnswer()
{
  SuspenseAnsPacket sns = SuspenseAnsPacket();
  sendToAll(&sns);
}
void wrongAnswer()
{
  WrongAnsPacket wns = WrongAnsPacket();
  sendToAll(&wns);
}
void updatePodiumBattery(std::array<uint8_t, 6> &mac_addr, BatteryStatPacket &bsp)
{
  PodiumData &podium = podiumsMap[mac_addr];
  podium.battLevel = bsp.level;
  podium.battVoltage = bsp.voltage;
  podium.isCharging = bsp.charging;
  SendToUI(serialSendCommands::BattStat, podium.position, bsp.level, bsp.voltage, bsp.charging);
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
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
void resetGameState()
{
  currentGameState = GameState::Idle;
  podiumAnsweredFirst = -1;
  buttonPlacement = 0;
  for (std::map<std::array<uint8_t, 6>, PodiumData>::iterator it = podiumsMap.begin(); it != podiumsMap.end(); ++it)
  {
    it->second.isButtonPressed = false;
    networking.sendPacket(it->first.data(), ResetStatePacket());
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

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  while (!Serial)
  {
  }
  SendToUI(serialSendCommands::Ready);
  // resetGameState();
}

void DnrTick()
{
  static unsigned long lastTime = 0;
  static int sendAllTick = 0;
  if (!Clock::TimePassed(lastTime, dnrTickDuration, true))
    return;
  RunDnr();
}

void showSummary()
{
  paired = true;
  for (const auto &[macAddr, podium] : podiumsMap)
  {
    SendToUI(serialSendCommands::PodiumAdded, podium.position, podium.dnrCount);
    SendToUI(serialSendCommands::Dnr, podium.position, static_cast<int>(podium.dnrSeverity));
  }
}
void pairToUI()
{
  Serial.println("Pairing");
  SendToUI(serialSendCommands::Pair, pairKey);
  paired = true;
}
void setPodiumBrightness(uint8_t face, uint8_t btn, std::array<uint8_t, 6> macAddr = {})
{
  if (macAddr.empty())
  {
    // handle default case

    LedBrightnessPacket wns = LedBrightnessPacket(face, btn);
    sendToAll(&wns);
    return;
  }
  for (const auto &[macAddr, podium] : podiumsMap)
  {
    SendToUI(serialSendCommands::PodiumAdded, podium.position, podium.dnrCount);
    SendToUI(serialSendCommands::Dnr, podium.position, static_cast<int>(podium.dnrSeverity));
  }
}
void processUICommand(Command command)
{
  switch (command.command)
  {
  case (int)serialReceiveCommands::Pair:
    pairToUI();
    /* code */
    break;
  case (int)serialReceiveCommands::ResetGameState:
    resetGameState();
    break;
  case (int)serialReceiveCommands::ReadyGameState:
    resetGameState();
    currentGameState = GameState::QuizReady;
    break;
  case (int)serialReceiveCommands::Summary:
    showSummary();
    break;
  case (int)serialReceiveCommands::CorrectAnswer:
    currentGameState = GameState::Lock;
    correctAnswer();
    break;
  case (int)serialReceiveCommands::WrongAnswer:
    currentGameState = GameState::Lock;
    wrongAnswer();
    break;
  case (int)serialReceiveCommands::SwapPodium:
    if (!hasPayloadLength(command, 3))
      return;
    swapPodium(command.payload.charAt(0) - '0', command.payload.charAt(2) - '0');
    break;
  case (int)serialReceiveCommands::SuspenseGame:
    currentGameState = GameState::Lock;
    suspenseAnswer();
    break;
  case (int)serialReceiveCommands::SpotLightPodium:
    if (!hasPayloadLength(command, 1))
      return;
    spotLightPodium(command.payload.charAt(0) - '0', false);
    break;
  case (int)serialReceiveCommands::PodiumBrightness:
    if (!hasPayloadLength(command, 1))
      return;
    setPodiumBrightness(command.payload.charAt(0) - '0', command.payload.charAt(0) - '0', {});
    break;
  default:
    Serial.println("NOT A COMMAND");
    break;
  }
}

void readSerial()
{
  serialManager.update();
  while (serialManager.available())
  {
    Serial.println("Command Avail");
    Command incomingCommand = serialManager.readCommand();

    // End of command, process it
    processUICommand(incomingCommand);
    delay(1); // Add a small delay to yield control
  }
}
// TODO: UNUSED, can be used to manually check batt percentages
void checkBattLevels()
{
  BattPingPacket wns = BattPingPacket();
  sendToAll(&wns);
}
void loop()
{
  DnrTick();
  readSerial();
}