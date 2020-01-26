#include "BluetoothSerial.h"

#define DEBUG_S

#ifdef DEBUG_S
#define BUG_UART Serial
#define Debug_print(...) BUG_UART.print( __VA_ARGS__ )
#define Debug_printf(...) BUG_UART.printf( __VA_ARGS__ )
#define Debug_println(...) BUG_UART.println( __VA_ARGS__ )
#define DEBUG
#endif


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define PIN_CMD 21
#define SIO_UART Serial2
#define STANDARD_BAUDRATE 19200
#define COMMAND_FRAME_SPEED_CHANGE_THRESHOLD 2
#define HISPEED_INDEX 0x08
#define HISPEED_BAUDRATE 57600
#define SIO_UART_TIMEOUT 10
#define BT_TIMEOUT 1000 // 1 sec
#define BT_BUFFER_SIZE 2048 // 2kB
#define SD_SECTOR_SIZE 128
#define DD_SECTOR_SIZE 256
#define DELAY_T5 250

bool hispeed = false;
int command_frame_counter = 0;

union {
  struct {
    unsigned char devic;
    unsigned char comnd;
    unsigned char aux1;
    unsigned char aux2;
    unsigned char cksum;
  };
  byte cmdFrameData[5];
} cmdFrame;

byte bt_data_buffer[BT_BUFFER_SIZE];
bool SingleDensity[4] = {false,false,false,false};

BluetoothSerial SIO2BT;

void setup() {
  pinMode(PIN_CMD, INPUT_PULLUP);
  Serial.begin(115200);
  SIO_UART.begin(STANDARD_BAUDRATE);
  SIO_UART.setTimeout(SIO_UART_TIMEOUT);
  SIO2BT.begin("ATARI FUJINET");
  SIO2BT.setTimeout(BT_TIMEOUT);
}

byte sio_checksum(byte* chunk, int length) {
  int chkSum = 0;
  for (int i = 0; i < length; i++) {
    chkSum = ((chkSum + chunk[i]) >> 8) + ((chkSum + chunk[i]) & 0xff);
  }
  return (byte)chkSum;
}

void sio_ack()
{
  SIO_UART.write('A');
  SIO_UART.flush();
#ifdef DEBUG
    Debug_printf("A\n");
#endif
}

void sio_complete()
{
  delayMicroseconds(DELAY_T5);
  SIO_UART.write('C');
}

void sio_error()
{
  delayMicroseconds(DELAY_T5);
  SIO_UART.write('E');
}

void sio_to_computer(byte* b, int len)
{
    byte ck = sio_checksum(b, len);
    SIO_UART.write(b, len);
    SIO_UART.write(ck);
    SIO_UART.flush();
}

bool isDisk() {
  if (cmdFrame.devic >= 0x31 && cmdFrame.devic <= 0x34) {
    return true;
  }
  return false;
}

bool isSupportedSIO2BTDevice() {
  if (isDisk() || cmdFrame.devic == 0x45 || cmdFrame.devic == 0x4E) {
    return true;
  }
  return false;
}

int estimateSIO2BTDataSize()
{
    int estimated_size = BT_BUFFER_SIZE;
    if(isDisk())
    {
        switch(cmdFrame.comnd)
        {
            case 0x53: // get status
                estimated_size = 7; // ACK+COMPLETE+4+CHECKSUM
                break;
            case 0x52: // read
                if((cmdFrame.aux2==0) && ((cmdFrame.aux1==1) || (cmdFrame.aux1==2) || (cmdFrame.aux1==3)) )
                {
                  estimated_size = SD_SECTOR_SIZE + 3;
                }
                else
                {
                  if(SingleDensity[cmdFrame.devic-0x31])
                  {
                    estimated_size = SD_SECTOR_SIZE + 3;
                  }
                  else
                  {
                    estimated_size = DD_SECTOR_SIZE + 3;
                  }
                }
                break;
            case 0x50: // write
            case 0x57:
                estimated_size = 1; // ACK
                break;
            case 0x4E: // PERCOM
                estimated_size = 12+3;
                break;
            case 0x51: // QUIT
                estimated_size = 2; // ACK + COMPLETE
                break;
            case 0x21: // FORMAT
                  if(SingleDensity[cmdFrame.devic-0x31])
                  {
                    estimated_size = SD_SECTOR_SIZE + 3;
                  }
                  else
                  {
                    estimated_size = DD_SECTOR_SIZE + 3;
                  }
                break;
            case 0x22: // FORMAT ED
                estimated_size = SD_SECTOR_SIZE + 3;
                SingleDensity[cmdFrame.devic-0x31] = true;
                break;
            default:
                break;
        }
    }
#ifdef DEBUG
    Debug_printf("BT estimated %d bytes\n",estimated_size);
#endif
    return estimated_size;
}

int estimateATARIDataSize()
{
    int estimated_size = BT_BUFFER_SIZE;
    if(isDisk())
    {
        switch(cmdFrame.comnd)
        {
            case 0x50: // write
            case 0x57:
                if((cmdFrame.aux2==0) && ((cmdFrame.aux1==1) || (cmdFrame.aux1==2) || (cmdFrame.aux1==3)) )
                {
                  estimated_size = SD_SECTOR_SIZE + 1;
                }
                else
                {
                  if(SingleDensity[cmdFrame.devic-0x31])
                  {
                    estimated_size = SD_SECTOR_SIZE + 1;
                  }
                  else
                  {
                    estimated_size = DD_SECTOR_SIZE + 1;
                  }
                }
                break;
            case 0x4F: // PERCOM
                estimated_size = 12+1;
                break;
            default:
                break;
        }
    }
#ifdef DEBUG
    Debug_printf("ATARI estimated %d bytes\n",estimated_size);
#endif
    return estimated_size;
}

void processSIO() {

  if (!isSupportedSIO2BTDevice()) return;

  sio_ack();  // always send ACK immediately (to meet SIO timing)

  if (isDisk()) { // high speed is a local business
    if (0x3F == cmdFrame.comnd) // between ATARI and ESP32
    {
      sio_complete();
      byte hsindex = HISPEED_INDEX;
      sio_to_computer(&hsindex,1);
      return;
    }
  }

  SIO2BT.write(cmdFrame.cmdFrameData, 5); // let SIO2BT process all other requests
  SIO2BT.flush();

  int expected_BT_data_size = estimateSIO2BTDataSize();

  bool more_data_available = true;
  int read_bytes = 0;
  int read_bytes_so_far = 0;
  int remaining_space = BT_BUFFER_SIZE;

  do {
    read_bytes = SIO2BT.readBytes(bt_data_buffer+read_bytes_so_far, (remaining_space<expected_BT_data_size)?remaining_space:expected_BT_data_size);
#ifdef DEBUG
    Debug_printf("BT read %d bytes\n",read_bytes);
#endif

    if(read_bytes==0)break;

    read_bytes_so_far += read_bytes;
    remaining_space -= read_bytes;

    switch (read_bytes_so_far) {

      case 1:  // Atari will send data
        if(bt_data_buffer[0]=='A') // OK, proceed with reading
        {
            int expected_ATARI_data_size = estimateATARIDataSize();
            do
            {
                read_bytes = SIO_UART.readBytes(bt_data_buffer, expected_ATARI_data_size);
#ifdef DEBUG
    Debug_printf("ATARI read %d bytes\n",read_bytes);
#endif
            }
            while (read_bytes==0);

            sio_ack();  // always send ACK immediately (to meet SIO timing)

            SIO2BT.write(bt_data_buffer, read_bytes);
            SIO2BT.flush();

            if( isDisk() && (cmdFrame.comnd == 0x4F) ) // write PERCOM
            {
              int bytes_per_sector = (bt_data_buffer[6] & 0xFF) * 0x100 + (bt_data_buffer[7] & 0xFF);
              if(bytes_per_sector == 0x80)
              {
                SingleDensity[cmdFrame.devic-0x31] = true;
              }
              else
              {
                SingleDensity[cmdFrame.devic-0x31] = false;
              }
            }

            // reset (we expect 2 more bytes to come from SIO2BT)
            expected_BT_data_size = 2;
            read_bytes_so_far = 0;
            remaining_space = BT_BUFFER_SIZE;
        }
        else if(bt_data_buffer[0]=='N')
        {
            sio_error();
            more_data_available = false;
        }
        else
        {
            more_data_available = false;
        }
        break;

      case 2:
        if( ((bt_data_buffer[0]=='A') || (bt_data_buffer[0]=='N')) && ((bt_data_buffer[1]=='C') || (bt_data_buffer[1]=='E')) )
        {
          SIO_UART.write(bt_data_buffer[1]);  // we have already acknowledged, so we send now complete or error
#ifdef DEBUG
    Debug_printf("%c\n",bt_data_buffer[1]);
#endif
          more_data_available = false;
        }
        break;

      default:  // Atari will receive data
        byte ck = sio_checksum(bt_data_buffer+2, read_bytes_so_far-3); // skip ACK, COMPLETE and CHECKSUM
        if(ck == bt_data_buffer[read_bytes_so_far-1])
        {
            SIO_UART.write(bt_data_buffer + 1, read_bytes_so_far - 1);
            int sector = cmdFrame.aux1 + (cmdFrame.aux2 << 8);
            if( isDisk() && (cmdFrame.comnd == 0x53) ) // get status
            {
#ifdef DEBUG
    Debug_printf("STATUS %d %d\n",bt_data_buffer[2], (bt_data_buffer[2]&0x20));
#endif

              if(bt_data_buffer[2] & 0x20)
              {
                SingleDensity[cmdFrame.devic-0x31] = false;
              }
              else
              {
                SingleDensity[cmdFrame.devic-0x31] = true;
              }
            }
            more_data_available = false;
        }
        else
        {
            expected_BT_data_size = BT_BUFFER_SIZE;
        }
        break;
    }
    if(remaining_space==0)
    {
        while (SIO2BT.available()) SIO2BT.read();
        more_data_available = false;
    }
  } while (more_data_available);
}

void loop() {
  if (digitalRead(PIN_CMD) == LOW) {
    memset(cmdFrame.cmdFrameData, 0, 5);
    int read_bytes = SIO_UART.readBytes(cmdFrame.cmdFrameData, 5);
#ifdef DEBUG
    Debug_printf("CF: %02x %02x %02x %02x %02x\n", cmdFrame.devic, cmdFrame.comnd, cmdFrame.aux1, cmdFrame.aux2, cmdFrame.cksum);
#endif
    byte ck = sio_checksum(cmdFrame.cmdFrameData, 4);
    if (ck == cmdFrame.cksum) {
      while (digitalRead(PIN_CMD) == LOW) yield();
      processSIO();
    } else {
      command_frame_counter++;
      if (COMMAND_FRAME_SPEED_CHANGE_THRESHOLD == command_frame_counter) {
        command_frame_counter = 0;
        if (hispeed) {
          SIO_UART.updateBaudRate(STANDARD_BAUDRATE);
          hispeed = false;
        } else {
          SIO_UART.updateBaudRate(HISPEED_BAUDRATE);
          hispeed = true;
        }
      }
    }
  } else {
    while (SIO_UART.available()) SIO_UART.read();
  }
}
