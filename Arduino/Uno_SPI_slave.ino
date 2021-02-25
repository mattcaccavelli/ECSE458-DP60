uint8_t received[4];
volatile int i = 0;
void setup()
{
    Serial.begin(115200);
    pinMode(SS, INPUT);
    pinMode(MOSI, OUTPUT);
    pinMode(MOSI, INPUT);
    pinMode(MISO, OUTPUT);
    pinMode(SCK, INPUT);

    // turn on SPI
    SPCR |= _BV(SPE);

    // turn on interrupts
    SPCR |= _BV(SPIE);
}

void loop(void)
{   
    if(i == 4){
        i = 0;
        Serial.print("Data received:");
        for(int j = 0; j < 4; ++j){
            Serial.printf(" %d", received[j]);
        Serial.println();
    }
}

ISR(SPI_STC_vect)
{
    int temp = SPDR;
    if(i < 4)
        received[i++] = temp;
}