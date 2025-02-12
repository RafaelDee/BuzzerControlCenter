/* #include <Arduino.h>

class CommandUI
{
    enum class ccUiSendCommands
    {
        Dnr,
        Pair,
        Ready,
        HeartbeatCmd,
        PodiumAdded,
        PodiumPlacement,
        BattStat
    };
    enum class ccUiReceiveCommands
    {
        ResetGameState,
        Pair,
        Summary,
        ReadyGameState,
        CorrectAnswer,
        WrongAnswer,
        SuspenseGame,
        SpotLightPodium,
    };
    typedef struct CommandBuffer
    {
        unsigned long lastSend;
        size_t size;
        const uint8_t *buffer;
    };
    void SendToUI(ccUiSendCommands command)
    {
        Serial.print('/');
        Serial.print(static_cast<int>(command));
        Serial.println();
    }
    template <typename... Args>
    void SendToUI(ccUiSendCommands command, Args... args)
    {
        Serial.print('/');
        Serial.print(static_cast<int>(command));
        Serial.print(' ');

        bool first = true;
        ((Serial.print(first ? "" : ","),
          Serial.print(args),
          first = false),
         ...);

        Serial.println();
    }
    void addToSendBuffer(const uint8_t *buffer, size_t size){
        
    }
}; */