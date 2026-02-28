
#include  <util/parity.h>
#include <stdarg.h>

//WATCH OUT!!
//For receiving we are processing the INVERTED Manchester signal
//For sending we generate the NON-INVERTED Manchester signal, direct and delayed 250ms
//  so the half-bit values are inverted between them

//Start sequence is 16 half bits: 0101010101000111
//  then each frame is 32 half bits, starting in 0b01, next 22 data half-bits in the middle and finally 2 half-bits even parity plus 6 half bits fill 0b101010
const word sequenceStart = 0b0101010101000111;

//End of multi-frame reception, sequence and mask
const word sequenceEnd = 0x8070;
const word maskEnd = 0x8077;

//Frame alignment, sequence and mask
const word sequenceFrameOdd = 0x8000;
const word sequenceFrameEven = 0x0007;
const word maskFrame = 0x8007;

//For detection of incorrect intrabit transitions
const word checkTransitions = 0xFFFF;


//TESTING VALUES for testing plugging RX-DAT-INV to constant data or external fuction generators
//const word sequenceEnd = 0x0;
//const word sequenceFrame = 0x0;
//const word sequenceStart = 0b1111111111111111;
//const word sequenceStart = 0b0000000000000000;
//const word sequenceStart = 0b0101010101010101;
//const word checkTransitions = 0x0;

//Max numebr of consecutive frames that can be received
const int MAX_FRAMES_RX = 256;

//All cycles values are calculated for a 600Mhz core clock
const int WAIT_CYCLES_RX = 30000; //Time we wait for a response frame after transmission
const int WAIT_CYCLES_RX_PENDING_TX = 5000; //Wait time for a response frame if not reception expected
const int WAIT_CYCLES_RX_SAMPLE = 85; //Cycles between signal samples, approx 8Mhz for 75 cycles (125ns) but can be increased slightly to reduce the probability of incorrect sampling due to clock drift
const int WAIT_CYCLES_TX = 300; //Half-bit duration for transmission
const int WAIT_CYCLES_TX_DLY = 150;  //Delay for TX-DATA-DLY, approx 250ns

//The Teensy LED is the speed problem indicator, if it lights on something is being processed very slowly and delaying signal sampling
const int PIN_OVERFLOW = 13;

//Output pins to twinax drivers
const int PIN_TX_ACT = 4;
const int PIN_OUT = 5;
const int PIN_OUT_DLY = 6;

//Input from twinax receiver
const int PIN_IN = 7;

//Enable dverbose debug over serial connection, not recommended for stable operation
const int ENABLEDEBUG = 0;
char msg_debug[1000] = {0};

#define RESIDUAL_CYCLES_LOG 1

//Initialize things
void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  //Serial.println("[DEBUG] Starting...");

  //Shit to initialize cycle count readings
  ARM_DEMCR    |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
  ARM_DWT_CYCCNT = 0;

  //Set pin modes
  pinMode(PIN_OVERFLOW, OUTPUT);
  pinMode(PIN_IN, INPUT);
  pinMode(PIN_TX_ACT, OUTPUT);
  pinMode(PIN_OUT, OUTPUT);
  pinMode(PIN_OUT_DLY, OUTPUT);

  digitalWrite(PIN_OVERFLOW, LOW);
  digitalWrite(PIN_OUT, HIGH);
  digitalWrite(PIN_OUT_DLY, HIGH);

}

//Function utilities
//
//
//

//Transmits a half-bit value
int transmit(boolean value, unsigned long lastCycles)
{
  //Serial.println(value, DEC);
  //We have to put the signal in the pin PIN_OUT, activate transmission PIN_TX_ACT and put with a 250ns delay transmitted bit in PIN_OUT_DLY
  //Remember that this is the NON-INVERTED signal so the values are the opposite as those from reception branch
  unsigned long  cyclesCurrent = ARM_DWT_CYCCNT;

  while (cyclesCurrent - lastCycles < WAIT_CYCLES_TX)
  {
    cyclesCurrent = ARM_DWT_CYCCNT;
  }

  unsigned long  cyclesTx = ARM_DWT_CYCCNT;
  //Direct signal
  digitalWriteFast(PIN_OUT, !value);
  //Activate transmission
  digitalWriteFast(PIN_TX_ACT, HIGH);
  cyclesCurrent = cyclesTx;
  while (cyclesCurrent - cyclesTx < WAIT_CYCLES_TX_DLY)
  {
    cyclesCurrent = ARM_DWT_CYCCNT;
  }
  //Delayed signal
  digitalWriteFast(PIN_OUT_DLY, !value);

  return cyclesTx;
}

//End a transmission and leave the bus in steady state
void endTx(unsigned long lastCycles)
{

  unsigned long  cyclesCurrent = ARM_DWT_CYCCNT;
  while (cyclesCurrent - lastCycles < WAIT_CYCLES_TX)
  {
    cyclesCurrent = ARM_DWT_CYCCNT;
  }
  unsigned long  cyclesTx = ARM_DWT_CYCCNT;
  cyclesCurrent = cyclesTx;
  //Direct signal
  digitalWriteFast(PIN_OUT, LOW);

  while (cyclesCurrent - cyclesTx < WAIT_CYCLES_TX_DLY)
  {
    cyclesCurrent = ARM_DWT_CYCCNT;
  }
  cyclesTx = ARM_DWT_CYCCNT;
  cyclesCurrent = ARM_DWT_CYCCNT;
  //Delayed signal
  digitalWriteFast(PIN_OUT_DLY, LOW);

  while (cyclesCurrent - cyclesTx < WAIT_CYCLES_TX * 25)
  {
    cyclesCurrent = ARM_DWT_CYCCNT;
  }
  //Disable transmission
  digitalWriteFast(PIN_TX_ACT, LOW);

}

static inline void read_from_serial(word *bufferRX, bool &waitForResponse, int &index)
{
  if (!Serial.available())
    return;

  const int length = MAX_FRAMES_RX * 2;
  char buffer [length + 10];
  char termChar = '\n';

  index = 0;

  //eotxPending = true;
  int numCharsRecv = Serial.readBytesUntil(termChar, buffer, length);
  if (!numCharsRecv) {
    digitalWriteFast(PIN_OVERFLOW, HIGH);
    return;

  }

  buffer[numCharsRecv] = '\0';

  if (ENABLEDEBUG) Serial.print("[DEBUG] RECEIVED : ");
  if (ENABLEDEBUG) Serial.print(numCharsRecv, DEC);
  if (ENABLEDEBUG) Serial.print(" CHARS ");

  //Serial.print(mystring);
  waitForResponse = true;

  //Process buffer to decode and split into frames
  int i = 0;
  index = 0;

  // TBD: what happens if numCharsRecv is odd ?
  while (i + 1 < numCharsRecv)
  {
    bufferRX[index] = (buffer[i] & 0x3f) | ((buffer[i + 1] & 0x1F) << 6);
    if (ENABLEDEBUG) Serial.print(" DECODED : ");
    if (ENABLEDEBUG) Serial.print(bufferRX[index], BIN );

    i += 2;
    index++;
  }
  if (ENABLEDEBUG) Serial.println("");
  digitalWriteFast(PIN_OVERFLOW, LOW);

}

static unsigned int inline checkFrameParity(word w)
{
    const auto check1 = parity_even_bit(w);
    const auto check2 = parity_even_bit(w >> 8);

    return check1 != check2;
}

static inline void write_to_5250(word *bufferRX, int index)
{
   if (index <= 0)
    return;

  //Transmitting start sequence
  //0b0101010101000111 in sequenceStart
  int lastClock = 0;

  for (int i = 0; i < 16; i++)
  {
    lastClock = transmit(bitRead(sequenceStart, 15 - i), lastClock);
  }

  //Data transmission
  for (int i = 0; i < index; i++)
  {
    //Conditioning buffer with sync, fill and parity
    //sync
    bufferRX[i] <<= 1;
    bitWrite(bufferRX[i], 0, 1);

    //fill
    bitWrite(bufferRX[i], 12, 0);
    bitWrite(bufferRX[i], 13, 0);
    bitWrite(bufferRX[i], 14, 0);
    bitWrite(bufferRX[i], 15, 0);

    bitWrite(bufferRX[i], 12, checkFrameParity(bufferRX[i]));

    //Transmission for each bit
    for (int j = 0; j < 16; j++)
    {
      //First half bit, reversed
      lastClock = transmit(!bitRead(bufferRX[i], j), lastClock);

      //Second half bit
      lastClock = transmit(bitRead(bufferRX[i], j), lastClock);
    }
  }

  endTx(lastClock);
}

static inline void write_to_serial(unsigned int *halfBitsDataTx, int indexTx)
{

  if (ENABLEDEBUG) Serial.print("[DEBUG] SENDING : ");
  if (ENABLEDEBUG) Serial.println(indexTx, DEC);

  for (int i = 0; i < indexTx; i++) {
    //halfBitsDataTx[i] = 0b1011100001111000;
    //0b1011100001111000

    byte firstByte = 0x40 | ((halfBitsDataTx[i] >> 9) & 0x3F ) ;
    byte secondByte = 0x40 | (( halfBitsDataTx[i] >> 4) & 0x1F ) ;

    if (ENABLEDEBUG) {
      Serial.print("[DEBUG] EVEN ");
      for (int j = 0; j < 16; j++)
      {
        if (halfBitsDataTx[i] < (1U << j))
          Serial.print("0");
      }

      Serial.println(halfBitsDataTx[i], BIN);

    }

    Serial.print((char)firstByte);
    Serial.print((char)secondByte);
    Serial.println("");

  }

}

//#define RESIDUAL_CYCLES_LOG
#define TIMEOUT_CHECK_RESIDUAL_CYCLES 10000

static inline void debug_message(const char *msg, ...)
{

  unsigned int l = strlen(msg_debug);
  /* to avoid sign problem with unsigned integer */
  if (l > sizeof msg_debug - 10)
    return;
  if (l > 0)
  {
    snprintf(msg_debug + l, sizeof(msg_debug) - 1 - l, "  ");
    l += 2;
  }

  va_list ap;
  va_start(ap, msg);
  vsnprintf(msg_debug + l , sizeof(msg_debug) - 1 - l ,
        msg, ap);
  va_end(ap);
}

enum class Errors{
  NOERROR = 0,
  ERRORTOOMANYFRAME,
  ERRORPROCESSINGTOOSLOW,
  ERRORTRANSITION,
  ERRORPARITY,
  ERRORSYNC,

  WARNRESIDUALCYCLES,

  ERRORCOUNT
};
struct Error5250 {
  uint32_t  errors;
  unsigned long cycles;
  int indexTX;
  int residualCycles;
  int receptionIsActive;
  int consecutiveSamples;

  void setError(Errors e) { errors |= 1u<<(int)e;}
  bool checkError(Errors e) { return !! (errors & 1u<<(int)e);}
  void clearError() { *this = {0}; }
  bool checkError() { return errors != 0; }
};

static inline void checkParitySyncError(unsigned int *halfBitsDataTx, int indexTx,
                                        struct Error5250 &error)
{
  for (int i = 0 ; i < indexTx ; i++)
  {

    //Now some error checking
    if (bitRead(halfBitsDataTx[i], 3) != checkFrameParity(halfBitsDataTx[i] <<1 >>5))
    {
      //Parity error, light LED
      digitalWriteFast(PIN_OVERFLOW, HIGH);
      error.setError(Errors::ERRORPARITY);

      return;
    }

    //Detection of incorrect frame alignment, light LED
    if ((halfBitsDataTx[i] & maskFrame) != sequenceFrameOdd)
    {
      digitalWriteFast(PIN_OVERFLOW, HIGH);
      error.setError(Errors::ERRORSYNC);

      return;
    }

  }
}

/*
 * The parsing of a frame is composed by several steps, each one with an internal
 * state. So there is dignity to make a class for parsing a frame
 */
struct Parse5250Frame {
    int consecutiveSamples = 0;
    uint8_t sampleActive = HIGH;
    uint8_t oddSampleActive = HIGH;
    int halfBitsDataReceived = 0;
    unsigned int halfBitsDataEven = 0;
    word sequenceStartReceived = 0;
    boolean receptionIsActive = false;

    inline bool parse(const uint8_t sampleRead, int &indexTx,
                      unsigned int *halfBitsDataTx, struct Error5250 &error);

};

inline bool Parse5250Frame::parse(const uint8_t sampleRead,
                                  int &indexTx, unsigned int *halfBitsDataTx,
                                  struct Error5250 &error)
{

    //Manage samples. If we get three or four consecutive samples at the same level we have a new half-bit
    //Otherwise we have a sync error
    switch (consecutiveSamples)
    {

        case 0:
            //New half-bit
            sampleActive = sampleRead;
            consecutiveSamples++;
            break;

        case 1:
            //second sample
            if (sampleActive != sampleRead)
            {
                //Out of sync!
                sampleActive = sampleRead;
                consecutiveSamples = 1;
            }
            else
            {
                consecutiveSamples++;
            }
            break;
        case 2:

            //third sample
            if (sampleActive != sampleRead)
            {
                //Out of sync!
                sampleActive = sampleRead;
                consecutiveSamples = 1;
            }
            else
            {

                consecutiveSamples++;
                //New half-bit!
                if (!receptionIsActive)
                {
                    //Add half-bit to start sequence detection
                    sequenceStartReceived <<= 1;
                    sequenceStartReceived += sampleActive;

                }
                else
                {

                    //Already got start sequence, add to received data, odd or even half bits
                    halfBitsDataReceived++;

                    //If we are starting a frame wait till the first even half bit is 0
                    if (halfBitsDataReceived == 2 && sampleActive == 0) {
                        halfBitsDataReceived = 0;
                        halfBitsDataEven = 0;
                        break;
                    }

                    if ((halfBitsDataReceived % 2) == 0)
                    {

                        // Detect incorrect intrabit transitions
                        if (oddSampleActive == sampleActive )
                        {
                            digitalWriteFast(PIN_OVERFLOW, HIGH);
                            error.setError(Errors::ERRORTRANSITION);
                            error.indexTX = indexTx;

                            return true;
                        }

                        halfBitsDataEven <<= 1;
                        halfBitsDataEven += sampleActive;
                        if (halfBitsDataReceived == 32)
                        {
                            //We have received a full frame
                            halfBitsDataTx[indexTx] = halfBitsDataEven;
                            indexTx++;
                            halfBitsDataReceived = 0;
                            halfBitsDataEven = 0;
                        }
                    }
                    else
                    {
                        oddSampleActive = sampleActive;
                    }
                }
            }
            break;

        case 3:
            //fourth and last value of half-bit
            if (sampleActive != sampleRead)
            {
                sampleActive = sampleRead;
                consecutiveSamples = 1;
            }
            else
            {
                consecutiveSamples = 1;
            }
            break;
    }

    //Check if we have received start sequence
    if (!receptionIsActive)
    {

        if ((sequenceStartReceived & 0xFFFF) == sequenceStart)
        {
            //Signal that we are now sampling the data frame
            receptionIsActive = true;
            //Reset holder
            sequenceStartReceived = 0;
        }
    }
    else
    {
        //Check if we have stop sequence (Address 7)
        if (indexTx > 0 && (halfBitsDataTx[indexTx - 1] & maskEnd) == sequenceEnd)
        {
            //No more reception
            return true;
        }

        //Detect too many frames (unlikely)
        if (indexTx > MAX_FRAMES_RX)
        {
            digitalWriteFast(PIN_OVERFLOW, HIGH);
            error.setError(Errors::ERRORTOOMANYFRAME);
            return true;
        }
    }

    return false;
}

static inline void read_from_5250(unsigned int *halfBitsDataTx, int &indexTx, struct Error5250 &error)
{

    Parse5250Frame  parse5250Frame;

    unsigned long  cyclesBeginReception = ARM_DWT_CYCCNT;
    unsigned long  cyclesCurrent = cyclesBeginReception;

    unsigned long residual_cycles = 0;

    error.clearError();

#ifdef RESIDUAL_CYCLES_LOG
    static int check_count = TIMEOUT_CHECK_RESIDUAL_CYCLES;
#endif

    while (true)
    {
        //We wait max WAIT_CYCLES_RX for a response, unless rx is already active
        const unsigned long delta = cyclesCurrent -  cyclesBeginReception;
        if (!(parse5250Frame.receptionIsActive || (delta < WAIT_CYCLES_RX))) // WAIT_CYCLES_RX = 30000
        {
            break;
        }
        //End earlier if no response expected
        if (! parse5250Frame.receptionIsActive &&
            Serial.available() &&
            (delta >= WAIT_CYCLES_RX_PENDING_TX)) // WAIT_CYCLES_RX_PENDING_TX = 5000
        {
            break;
        }

        residual_cycles = ARM_DWT_CYCCNT - cyclesCurrent;
#ifdef RESIDUAL_CYCLES_LOG
        if (--check_count < 0)
        {
            check_count = TIMEOUT_CHECK_RESIDUAL_CYCLES;
        }
#endif

        if (residual_cycles >= WAIT_CYCLES_RX_SAMPLE)  // WAIT_CYCLES_RX_SAMPLE = 85
        {
            //Processing has been too slow, light LED and inform
            digitalWriteFast(PIN_OVERFLOW, HIGH);
            error.setError(Errors::ERRORPROCESSINGTOOSLOW);
            error.cycles = residual_cycles;
            error.receptionIsActive = parse5250Frame.receptionIsActive;
            error.consecutiveSamples = parse5250Frame.consecutiveSamples;

            break;
        }

        //Wait till it's time to get another sample from RX-DAT-INV
        const auto target = cyclesCurrent + WAIT_CYCLES_RX_SAMPLE;
        while ((int32_t)(ARM_DWT_CYCCNT - target) < 0);
        cyclesCurrent = ARM_DWT_CYCCNT;

        //Sample RX-DAT-INV
        const uint8_t sampleRead = digitalReadFast(PIN_IN);

        if (parse5250Frame.parse(sampleRead, indexTx, halfBitsDataTx, error))
            break;
    }

    // check parity and sync error, then set 'error' properly
    if (!error.checkError())
        checkParitySyncError(halfBitsDataTx, indexTx, error);

#ifdef RESIDUAL_CYCLES_LOG
    if (check_count == TIMEOUT_CHECK_RESIDUAL_CYCLES || residual_cycles > 70)
    {
        error.setError(Errors::WARNRESIDUALCYCLES);
        error.residualCycles = residual_cycles;
        residual_cycles = 0;
    }
#endif

    if (error.checkError()) {
        cyclesCurrent = ARM_DWT_CYCCNT;
        indexTx = 0;
        while (cyclesCurrent -  cyclesBeginReception < WAIT_CYCLES_RX) // WAIT_CYCLES_RX = 30000
        {
            cyclesCurrent = ARM_DWT_CYCCNT;
        }
    }
}

//MAIN LOOP
void loop() // run over and over
{

    boolean waitForResponse = false;
    boolean signalEndTx = false;
    {
        word bufferRX[MAX_FRAMES_RX+10];
        int lenRx = 0;

        //Enable interrupts for serial port
        interrupts();

        //Reception from serial port
        read_from_serial(bufferRX, waitForResponse, lenRx);

        //Start of timing-critical stuff, so we disable interruptions
        noInterrupts();

        //Transmit data to the 5250 if pending data
        //Variable to set if we need to tx [EOF] at the end of this processing cycle
        if (lenRx > 0) {
          write_to_5250(bufferRX, lenRx);
          //We have transmitted something, so we need to generate later [EOTX] over the serial port
          signalEndTx = true;
        }
    }

    struct Error5250 error5250;
    error5250.clearError();

    if (waitForResponse) {

      unsigned int halfBitsDataTx[MAX_FRAMES_RX+10];
      int lenTx = 0;

      read_from_5250(halfBitsDataTx, lenTx, error5250);

      //Transmission to serial
      if (lenTx > 0)
      {
        interrupts();
        write_to_serial(halfBitsDataTx, lenTx);
      }
    }

    if (error5250.checkError(Errors::ERRORTOOMANYFRAME))
    {
      debug_message("MAX FRAMES ERROR");
    }
    if (error5250.checkError(Errors::ERRORPROCESSINGTOOSLOW))
    {
      debug_message("ERRORPROCESSINGTOOSLOW: cycles=%d, receptionIsActive=%d,"
                      " consecutiveSamples=%d",
                    error5250.cycles, error5250.receptionIsActive,
                      error5250.consecutiveSamples);
    }
    if (error5250.checkError(Errors::ERRORTRANSITION))
    {
      debug_message("ERRORTRANSITION: indexTx=%d", error5250.indexTX);
    }
    if (error5250.checkError(Errors::ERRORPARITY))
    {
      debug_message("ERRORPARITY");
    }
    if (error5250.checkError(Errors::ERRORSYNC))
    {
      debug_message("ERRORSYNC");
    }
    if (error5250.checkError(Errors::WARNRESIDUALCYCLES))
    {
      debug_message("WARNRESIDUALCYCLES: residualCycles=%d", error5250.residualCycles);
    }
    if (msg_debug[0])
    {
      interrupts();
      Serial.print("[DEBUG] ");
      Serial.println(msg_debug);
      msg_debug[0]=0;
    }

    if (signalEndTx)
    {
      //Signal end of trnsmission
      Serial.println("[EOTX]");
    }

    //And start of another processing cycle till the end of the universe


}
