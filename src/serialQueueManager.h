#include <Arduino.h>
#include <map>
#include <array>
#include <queue>
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
    PodiumColor
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
    BattStat
};
struct Command
{
    uint32_t idempotencyToken;
    int command;
    String payload;
};
class SerialQueueManager
{

    struct QueueItem
    {
        int command;
        unsigned long firstAttempt = 0;
        unsigned long lastAttempt = 0;
        int attempts = 0;
        bool acknowledged = false;
        String payload;
        QueueItem() {}
        QueueItem(int command) : command(command) {}
        QueueItem(int command,
                  String &payload) : command(command), payload(payload) {}
    };
    uint32_t nextRequestId = 1;
    std::map<uint32_t, QueueItem> messageQueue;
    const unsigned long RETRY_INTERVAL = 200;
    const int MAX_ATTEMPTS = 5;

public:
    uint32_t queueMessage(int command)
    {
        Serial.print("QUEUEING: ");
        Serial.print(command);
        Serial.println();
        uint32_t idempotencyToken = nextRequestId++;
        messageQueue[idempotencyToken] = QueueItem(command);
        return idempotencyToken;
    }
    uint32_t queueMessage(int command,
                          String &payload)
    {
        Serial.print("QUEUEING: ");
        Serial.print(command);
        Serial.print(" : ");
        Serial.print(payload);
        Serial.println();
        uint32_t idempotencyToken = nextRequestId++;
        messageQueue[idempotencyToken] = QueueItem(command, payload);
        Serial.print("Queued payload check: ");
        Serial.println(messageQueue[idempotencyToken].payload);
        return idempotencyToken;
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
            Serial.print('/');
            Serial.print(itemId);
            Serial.print(':');
            Serial.print(item.command);
            Serial.print(' ');
            Serial.println(item.payload);
            // SendToUI(itemId, item.command.command, item.command.payload);

            item.lastAttempt = currentTime;
            item.attempts++;

            if (item.attempts >= MAX_ATTEMPTS)
            {
                messageQueue.erase(itemId);
            }
        }
        delay(1); // Add a small delay to yield control
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
    /**ACK received message, do not broadcast ACK packet */
    void acknowledgeReceivedMessage(uint32_t idempotencyToken)
    {
        Serial.println("ACK:");
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
        sendACK(queue.idempotencyToken);
        Serial.print("READING:");
        Serial.println(queue.payload);
        // does not need to be checked if index is null, already checked before inserting
        int command = queue.payload[0] - '0';
        return {queue.idempotencyToken, command, queue.payload.substring(2)};
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
        if (buffer.length() < 2)
        {
            Serial.println("Invalid Command");
            return;
        }
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
        Serial.print("COMMAND:");
        Serial.println(buffer[colonPos + 1]);
        if (command == (int)serialReceiveCommands::Acknowledge)
        {
            Serial.println("ACKNOWLEDGED");
            acknowledgeReceivedMessage(idempotencyToken);
            return;
        }
        addToQueue(idempotencyToken, buffer.substring(colonPos + 1));
    }
    void addToQueue(uint32_t idempotencyToken, String buffer)
    {
        /**checks if request is already received, if so, send ACK*/
        if (std::find(receivedCommandTokens.begin(), receivedCommandTokens.end(), idempotencyToken) != receivedCommandTokens.end())
        {
            return;
        }

        if (buffer.length() < 1 || buffer.length() == 2)
        {
            Serial.println("Invalid Command");
            return;
        }
        readQueue.push({idempotencyToken, buffer});
        receivedCommandTokens.push_back(idempotencyToken);
        // check receivedCommandTokens length, trim excess
        if (receivedCommandTokens.size() > maxCommandTokens)
        {
            receivedCommandTokens.erase(receivedCommandTokens.begin());
        }
    }
};