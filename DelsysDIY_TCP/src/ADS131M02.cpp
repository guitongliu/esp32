#include "ADS131M02.h"

ADS131M02::ADS131M02(int clk,uint8_t clk_pin, uint8_t miso_pin, uint8_t mosi_pin, uint8_t cs_pin, uint8_t drdy_pin)
{
    SpiClk = clk;
  ADS131M02_CS_PIN = cs_pin;
  ADS131M02_DRDY_PIN = drdy_pin;
  ADS131M02_CLK_PIN = clk_pin;
  ADS131M02_MISO_PIN = miso_pin;
  ADS131M02_MOSI_PIN = mosi_pin;

}
void ADS131M02::spiclk_change(int clk)
{
    SpiClk = clk;
    spi->endTransaction();
    spi->beginTransaction(SPISettings(SpiClk, MSBFIRST, SPI_MODE1));
}

void ADS131M02::init(int clkin) {

    Serial.println("Setting pin configuration");
        
    pinMode(ADS131M02_CS_PIN, OUTPUT); 
    digitalWrite(ADS131M02_CS_PIN, HIGH);
    pinMode(ADS131M02_DRDY_PIN, INPUT_PULLUP);
    spi->begin(ADS131M02_CLK_PIN,ADS131M02_MISO_PIN,ADS131M02_MOSI_PIN,ADS131M02_CS_PIN);
    spi->beginTransaction(SPISettings(SpiClk, MSBFIRST, SPI_MODE1));

    Serial.println("SPI Ready...");

}


bool ADS131M02::writeRegister(uint8_t address, uint16_t value)
{

    uint16_t commandWord =(CMD_WRITE_REG) | (address << 7);
digitalWrite(ADS131M02_CS_PIN, LOW);
    spiTransferWord(commandWord);
    spiTransferWord(value);
        // Send 4 empty words
 
    for (uint8_t i=0; i<4; i++) {
        spiTransferWord();
    }

    digitalWrite(ADS131M02_CS_PIN, HIGH);

    // Get response
    uint32_t responseArr[10];
    spiCommFrame(&responseArr[0]);

    if ( ( (0x04<<12) + (address<<7) ) == responseArr[0]) {
        return true;
    } else {
        return false;
    }
}

void ADS131M02::writeRegisterMasked(uint8_t address, uint16_t value, uint16_t mask)
{
  // Escribe un valor en el registro, aplicando la mascara para tocar unicamente los bits necesarios.
  // No realiza el corrimiento de bits (shift), hay que pasarle ya el valor corrido a la posicion correcta

  // Leo el contenido actual del registro
  uint16_t register_contents = readReg(address);

  // Cambio bit aa bit la mascara (queda 1 en los bits que no hay que tocar y 0 en los bits a modificar)
  // Se realiza un AND co el contenido actual del registro.  Quedan "0" en la parte a modificar
  register_contents = register_contents & ~mask;

  // se realiza un OR con el valor a cargar en el registro.  Ojo, valor debe estar en el posicion (shitf) correcta
  register_contents = register_contents | value;

  // Escribo nuevamente el registro
  writeRegister(address, register_contents);
}

uint16_t ADS131M02::readReg(uint8_t address) {
    /* Reads the content of single register found at address reg
        Returns register value
    */
    
    // Make command word using syntax found in data sheet
    uint16_t commandWord = CMD_READ_REG | (address << 7);

    uint32_t responseArr[10];
    // Use first frame to send command
    spiCommFrame(&responseArr[0], commandWord);
    // Read response
    spiCommFrame(&responseArr[0]);
    return responseArr[0] >> 16;
}

uint32_t ADS131M02::spiTransferWord(uint16_t inputData) {
    /* Transfer a 24 bit word
        Data returned is MSB aligned
    */ 

    uint32_t data = spi->transfer(inputData >> 8);
    data <<= 8;
    data |= spi->transfer((inputData<<8) >> 8);
    data <<= 8;
    data |= spi->transfer(0x00);

    return data << 8;
}

void ADS131M02::spiCommFrame(uint32_t * outPtr, uint16_t command) {
    // Saves all the data of a communication frame to an array with pointer outPtr

    digitalWrite(ADS131M02_CS_PIN, LOW);

    // spi->beginTransaction(SPISettings(SpiClk, MSBFIRST, SPI_MODE1));

    // Send the command in the first word
    *outPtr = spiTransferWord(command);

    // For the next 8 words, just read the data
    for (uint8_t i=1; i < 9; i++) {
        outPtr++;
        *outPtr = spiTransferWord() >> 8;
    }

    // Save CRC bits
    outPtr++;
    *outPtr = spiTransferWord();

    // spi->endTransaction();

    digitalWrite(ADS131M02_CS_PIN, HIGH);
}

void ADS131M02::read_ADC(uint8_t * buffer)
{
//原本的缓冲区长度为30，现在我改一下改成12
uint8_t Tx_buffer[9];
digitalWrite(ADS131M02_CS_PIN, LOW);
// spi->beginTransaction(SPISettings(SpiClk, MSBFIRST, SPI_MODE1));
spi->transfer(Tx_buffer,9);

  digitalWrite(ADS131M02_CS_PIN, HIGH);
  for (uint i=3;i<3*1+3;i++)                                        //不知道这里是在干嘛，但是我姑且先改成2通道，原：i<3*8+3
{* buffer = Tx_buffer[i]; // 将读取的字节存入 buffer 指向的内存位置

  buffer++;
  }

// memcpy(buffer, &Tx_buffer[3], 3); // 复制第一个通道的数据

// Serial.printf("Raw SPI Data: ");
//     for (int i=0; i<9; i++) Serial.printf("%02X ", Tx_buffer[i]);
//     Serial.println();
}

bool ADS131M02::setOsr(uint16_t osr)
{
if (osr > 7)
  {
    return false;
  }
  else
  {
    writeRegisterMasked(REG_CLOCK, osr << 2 , REGMASK_CLOCK_OSR);
    return true;
  }

}

bool ADS131M02::setGain(uint16_t gain)
{
if (gain > 7)
  {
    return false;
  }
    else
  {
    uint16_t reg = gain<<12|gain<<8|gain<<4|gain;
    writeRegister(REG_GAIN1, reg);
    writeRegister(REG_GAIN2, reg);
    
    return true;
  }
}