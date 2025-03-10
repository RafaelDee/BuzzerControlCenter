#include <Arduino.h>
#include <map>
#include <array>
#include <queue>
static const uint32_t NO_TOKEN = UINT32_MAX;
enum class serialReceiveCommands
{
    Acknowledge,
    ResetGameState,
    Pair,
    Summary,
    ReadyGameState,
    CorrectAnswer,
    WrongAnswer,
    SuspenseGame,
    SpotLightPodium,
    SwapPodium,
    PodiumBrightness,
    PodiumColor,
    ClearPodium,
    VerboseMode,
    SendDeviceInfo
};

enum class serialSendCommands
{
    Acknowledge,
    Dnr,
    Pair,
    Ready,
    HeartbeatCmd,
    PodiumAdded,
    PodiumPlacement,
    BattStat,
    GameState,
    SpotLightPodium,
    PodiumBrightness,
    DeviceInfo
};
struct Command
{
    uint32_t idempotencyToken;
    int command;
    String payload;
    Command() : payload("") {}
    Command(uint32_t idempotencyToken, int command) : idempotencyToken(idempotencyToken), command(command) {}
    Command(uint32_t idempotencyToken, int command, String payload) : idempotencyToken(idempotencyToken), command(command), payload(payload) {}
};
class SerialQueueManager
{

    struct QueueItem
    {
        unsigned long firstAttempt = 0;
        unsigned long lastAttempt = 0;
        int attempts = 0;
        Command command;
        QueueItem() : command() {}
        QueueItem(Command command) : command(command) {}
    };
    uint32_t nextRequestId = 1;
    std::map<uint32_t, QueueItem> messageQueue;
    std::map<uint32_t, Command> messageRequestQueue;
    const unsigned long RETRY_INTERVAL = 200;
    const int MAX_ATTEMPTS = 5;

public:
    uint32_t queueMessage(int command, uint32_t responseToken = NO_TOKEN)
    {
        if (responseToken == NO_TOKEN)
        {
            responseToken = nextRequestId++;
            messageQueue[responseToken] = QueueItem(Command(responseToken, command));
        }
        else
        {
            Command cmd(responseToken, command);
            messageRequestQueue[responseToken] = cmd;
            sendQueueCommand(cmd);
        }

        return responseToken;
    }
    uint32_t queueMessage(int command, uint32_t responseToken,
                          String &payload)
    {
        if (responseToken == NO_TOKEN)
        {
            responseToken = nextRequestId++;
            messageQueue[responseToken] = QueueItem(Command(responseToken, command, payload));
        }
        else
        {
            Command cmd(responseToken, command, payload);
            messageRequestQueue[responseToken] = cmd;
            sendQueueCommand(cmd);
        }
        return responseToken;
    }
    void sendQueueCommand(Command command)
    {
        Serial.print('/');
        Serial.print(command.idempotencyToken);
        Serial.print(':');
        Serial.print(command.command);
        Serial.print(' ');
        Serial.println(command.payload);
    }
    void update()
    {

        readSerial(); // Add this to process incoming commands
        if (messageQueue.empty())
            return;

        auto currentItem = messageQueue.begin();
        auto &itemId = currentItem->first;
        auto &item = currentItem->second;
        unsigned long currentTime = millis();

        if (item.attempts == 0 ||
            (currentTime - item.lastAttempt >= RETRY_INTERVAL))
        {
            sendQueueCommand(item.command);
            // SendToUI(itemId, item.command.command, item.command.payload);

            item.lastAttempt = currentTime;
            item.attempts++;

            if (item.attempts >= MAX_ATTEMPTS)
            {
                messageQueue.erase(itemId);
            }
            delay(1); // Add a small delay to yield control
        }
    }

    template <typename... Args>
    /* void SendToUI(uint32_t idempotencyToken, Args... args)
    {

        Serial.print('/');
        Serial.print(idempotencyToken);
        Serial.print(':');

        bool first = true;
        ((Serial.print(first ? "" : ","),
          Serial.print(args),
          first = false),
         ...);

        Serial.println();
    } */
    void sendACK(uint32_t idempotencyToken)
    {
        Serial.print('/');
        Serial.print(idempotencyToken);
        Serial.print(':');
        Serial.println(static_cast<int>(serialSendCommands::Acknowledge));
    }
    /**ACK received message, stop sending it again, do not broadcast ACK packet */
    void acknowledgeReceivedMessage(uint32_t idempotencyToken)
    {
        Serial.print("ACK:");
        Serial.println(idempotencyToken);
        messageQueue.erase(idempotencyToken);
        /*  auto it = messageQueue.find(idempotencyToken);
         if (it != messageQueue.end())
         {
             Serial.println("ACK:");
             messageQueue.erase(idempotencyToken);
             Serial.print("TOKEN DATRA:");
             Serial.println(messageQueue[idempotencyToken].payload);
         } */
    }
    int available()
    {
        return readQueue.size();
    }

    Command readCommand()
    {
        ReceiveCommandQueue queue = readQueue.front();
        readQueue.pop();
        // sendACK(queue.idempotencyToken);
        // does not need to be checked if index is null, already checked before inserting
        int colonPos = queue.payload.indexOf(' ');
        String idStr = queue.payload.substring(0, colonPos);
        int command = idStr.toInt();
        return {queue.idempotencyToken, command, queue.payload.substring(colonPos + 1)};
    }

private:
    struct ReceiveCommandQueue
    {
        uint32_t idempotencyToken;
        String payload;
    };
    std::queue<ReceiveCommandQueue> readQueue;
    /**command structure:  `idempotencyToken:command payload`*/
    void readSerial()
    {
        static String buffer;
        while (Serial.available() > 0)
        {
            char incomingByte = Serial.read(); // Read one byte

            if (incomingByte == '\n')
            {
                // End of command, process it
                processCommand(buffer);
                buffer = ""; // Clear the buffer for the next command
            }
            else
            {
                buffer += incomingByte; // Append byte to the command buffer
            }
        }
    }
    const static unsigned int maxCommandTokens = 10;
    std::vector<uint32_t> receivedCommandTokens;
    void processCommand(String buffer)
    {
        int colonPos = buffer.indexOf(':');
        if (colonPos == -1 || colonPos + 1 >= buffer.length())
        {
            Serial.println("Invalid ACK format");
            return;
        }
        String idStr = buffer.substring(0, colonPos);
        uint32_t idempotencyToken = idStr.toInt();
        /**checks if request is already received, if so, ignore*/

        int command = buffer[colonPos + 1] - '0';
        if (command == (int)serialReceiveCommands::Acknowledge)
        {
            acknowledgeReceivedMessage(idempotencyToken);
            return;
        }
        addToQueue(idempotencyToken, buffer.substring(colonPos + 1));
    }
    void addToQueue(uint32_t idempotencyToken, String buffer)
    {
        /**checks if request is already received, if so, resend response or none if pending*/
        if (std::find(receivedCommandTokens.begin(), receivedCommandTokens.end(), idempotencyToken) != receivedCommandTokens.end())
        {
            // send previous response

            auto it = messageRequestQueue.find(idempotencyToken);
            if (it != messageRequestQueue.end())
            {
                sendQueueCommand(it->second);
            }
            return;
        }
        readQueue.push({idempotencyToken, buffer});
        receivedCommandTokens.push_back(idempotencyToken);
        // check receivedCommandTokens length, trim excess
        if (receivedCommandTokens.size() > maxCommandTokens)
        {
            receivedCommandTokens.erase(receivedCommandTokens.begin());
        }
        if (messageRequestQueue.size() > maxCommandTokens)
        {
            messageRequestQueue.erase(messageRequestQueue.begin());
        }
    }
};