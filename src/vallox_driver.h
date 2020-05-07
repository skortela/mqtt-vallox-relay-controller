
class Adafruit_MCP23017;

/**
 * ValloxDriver
 * Vallox relay controller
 **/
class ValloxDriver
{
    public:
        ValloxDriver();
        ~ValloxDriver();
        //void begin();
        void begin(Adafruit_MCP23017* pMcp); //initializes ValloxDriver
        

        void setPower(bool power);
        void setSpeed(int speed); // set speed, range 1 to 4
        void setTakkatehostus(bool takkatehostus); // enable or disable takkatehostus

        bool hasPower() const;
        int speed() const;
        bool hasTakkatehostus() const;

        //char* getJsonState();
    
        // For advanced use: control each motor separately
        void setSpeedIncoming(int speed);
        void setSpeedExhaust(int speed);
        int speedIncoming() const;
        int speedExhaust() const;
    private:
        void setRelay(uint8_t relayIndex, bool state);
    private:
        Adafruit_MCP23017* m_pMcp;
        bool m_power;
        int m_speedIncoming; // current speed
        int m_speedExhaust;
        int m_lastUsedSpeed;
        bool m_takkatehostus;
};