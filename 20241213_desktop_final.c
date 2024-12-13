//Red Wire = 3V3
//Black Wire = Ground
//Orange Wire = SCLK
//Yellow Wire = MOSI
//Green Wire = MISO
//Brown Wire = CS

#include <chrono>
#include <vector>
#include <random>
#include <climits>
#include <algorithm>
#include <functional>

#include <conio.h> // For getch() and kbhit()
#include <io.h>     // For _setmode()
#include <fcntl.h>  // For _O_BINARY

#include <stdint.h>
#include <iostream>
#include <stdio.h>
#include <Windows.h>
#include <time.h>
#include <math.h>
#include "ftd2xx.h"
#include "libmpsse_spi.h"
#include "STM32.h"

// For UART
HANDLE hComm;

// For printing out error message 
void print_and_quit(const char cstring[]) {
    printf("%s\n", cstring);
    getc(stdin);
    exit(1);
}

// Initialize UART 
void Init_UART() {
    BOOL status;
    DCB dcbSerialParams = { 0 };
    COMMTIMEOUTS timeouts = { 0 };

    // Check the Port that is connected to STM32 
    hComm = CreateFile(L"\\\\.\\COM6", GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    // Check if the serial port can be open
    if (hComm == INVALID_HANDLE_VALUE) {
        printf("Error in opening serial port\n");
    }
    else {
        printf("Opening serial port successful\n");
    }

    // Check if purging comm is complete
    status = PurgeComm(hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);
    if (!status) {
        printf("Error in purging comm\n");
        CloseHandle(hComm);
    }

    // Get the comm State
    status = GetCommState(hComm, &dcbSerialParams);
    if (!status) {
        printf("Error in GetCommState\n");
        CloseHandle(hComm);
    }

    // We will be using BaudRate of 9600 for UART communication, the configuration of UART is same with that of STM32
    dcbSerialParams.BaudRate = CBR_9600;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    status = SetCommState(hComm, &dcbSerialParams);

    if (!status) {
        printf("Error in setting DCB structure\n");
        CloseHandle(hComm);
    }

    // How long can UART wait until it completes reception and transmission (It is set to be very long)
    timeouts.ReadIntervalTimeout = 200000;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 200000;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 200000;
    if (!SetCommTimeouts(hComm, &timeouts)) {
        printf("Error in setting timeouts\n");
        CloseHandle(hComm);
    }
}

//Initialize SPI
FT_HANDLE Init_SPI() {
    FT_STATUS status;
    FT_DEVICE_LIST_INFO_NODE channelInfo;
    FT_HANDLE handle;

    DWORD channelCount = 0;

    // Check the number of SPI channels
    status = SPI_GetNumChannels(&channelCount);

    // Check if there is at least one channel
    if (status != FT_OK)
        print_and_quit("Error while checking the number of available MPSSE channels.");
    else if (channelCount < 1)
        print_and_quit("Error: No MPSSE channels are available.");

    printf("There are %d channels available. \n\n", channelCount);

    // Display the information of the channel
    for (unsigned int i = 0; i < channelCount; i++) {
        status = SPI_GetChannelInfo(i, &channelInfo);
        if (status != FT_OK)
            print_and_quit("Error while getting details for an MPSSE channel.");
        printf("Channel number : %d\n", i);
        printf("Description: %s\n", channelInfo.Description);
        printf("Serial Number : %s\n", channelInfo.SerialNumber);
    }

    // Ask user to use which channel he will use
    uint32_t channel = 0;
    printf("\n Enter a channel number to use: ");
    scanf_s("%d", &channel);

    // Open that channel and check if it can be open
    status = SPI_OpenChannel(channel, &handle);
    if (status != FT_OK)
        print_and_quit("Error while opening the MPSSE channel.");

    // Configure the channel. ClockRate and LatencyTimer can be configured differrently but this combination worked optimal for me
    ChannelConfig channelConfig;
    channelConfig.ClockRate = 15000000;
    channelConfig.configOptions = SPI_CONFIG_OPTION_MODE2 | SPI_CONFIG_OPTION_CS_DBUS3 | SPI_CONFIG_OPTION_CS_ACTIVELOW;
    channelConfig.LatencyTimer = 20;

    //   Initialize the channel according to the configuration
    status = SPI_InitChannel(handle, &channelConfig);
    if (status != FT_OK)
        print_and_quit("Error while initializing the MPSSE channel.");
    return handle;
}

//This function is written considering endianness mismatch between SPI and SAI
void reversePacketOrder(UCHAR* buffer, size_t bufferSize) {
    for (size_t i = 0; i < bufferSize; i += 4) {
        for (size_t j = 0; j < 2; ++j) {
            UCHAR temp = buffer[i + j];
            buffer[i + j] = buffer[i + 3 - j];
            buffer[i + 3 - j] = temp;
        }
    }
}

int test(int argc, char** argv)
{
    FT_HANDLE handle;
    FILE* fp;
    FILE* fp2;
    FILE* fr;
    FILE* fr2;

    //Initialize the peripherals
    Init_UART();
    Init_libMPSSE();
    handle = Init_SPI();

    // Open the csv file to record position feedback
    fopen_s(&fp, "data.csv", "w+");
    fopen_s(&fp2, "renew_data.csv", "w+");
    // Read txt data
    fopen_s(&fr, "EnginePattern_raster_amp1_res512_hexadecimal_downsample.txt", "r");

    FT_STATUS status;
    bool status2;
    DWORD bytesRead;
    DWORD bytesWrite;
    //StartByte is used for FSM
    UCHAR StartByte = 0;
    //Receive a byte from STM32 every time it completes SAI transmission of chunk
    UCHAR RxByte;

    DWORD transferCount = 0;
    LPDWORD ptransferCount = &transferCount;
    DWORD options = SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES | SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE | SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE;

    //Match this count with STM32
    int COUNT = 32 * 4;
    const int CHUNK_NUM = 32;
    //extern const int CHUNK_NUM;
    int CHUNK_NUM_CURRENT = 0;
    int COMPARISON_CHUNK_NUM_CURRENT = 0;
    //Write the number of positions per chunk
    //const int NUM_OF_POSITIONS_PER_CHUNK = 3132;
    const int NUM_OF_POSITIONS_PER_CHUNK = 2620; //83840 / 32 for Raster Amplitude 1 
    const int MULTIPLIER = 4;

    //The number of positions inside full scan pattern0

    DWORD input_data[CHUNK_NUM * NUM_OF_POSITIONS_PER_CHUNK];

    //The number of bytes inside full scan pattern
    UCHAR tx_buffer[4 * CHUNK_NUM * NUM_OF_POSITIONS_PER_CHUNK];

    //The number of bytes per chunk
    int NUM_OF_BYTES_PER_CHUNK = 4 * NUM_OF_POSITIONS_PER_CHUNK;
    UCHAR tx_message[4 * NUM_OF_POSITIONS_PER_CHUNK + 4];
    UCHAR rx_buffer[MULTIPLIER * NUM_OF_POSITIONS_PER_CHUNK];

    int x;
    int y;
    int original_x;
    int original_y;
    long position;
    int16_t state = 0;
    int16_t payload = NUM_OF_POSITIONS_PER_CHUNK;
    int idle_sent = 0;
    int i = 2;
    int32_t msg = ((uint32_t)state << 16) | (uint16_t)payload;

    for (int k = 0; k < CHUNK_NUM * NUM_OF_POSITIONS_PER_CHUNK; ++k) {
        fscanf_s(fr, "%x", &input_data[k]);
    }

    for (int k = 0; k < CHUNK_NUM * NUM_OF_POSITIONS_PER_CHUNK; ++k) {
        if (k < CHUNK_NUM * NUM_OF_POSITIONS_PER_CHUNK - 5) {
            uint32_t data = input_data[k + 5];
            tx_buffer[4 * k + 3] = (uint8_t)((data >> 24) & 0xFF);
            tx_buffer[4 * k + 2] = (uint8_t)((data >> 16) & 0xFF);
            tx_buffer[4 * k + 1] = (uint8_t)((data >> 8) & 0xFF);
            tx_buffer[4 * k] = (uint8_t)(data & 0xFF);
        }
        else {
            uint32_t data = input_data[(k + 5) % (CHUNK_NUM * NUM_OF_POSITIONS_PER_CHUNK)];
            tx_buffer[4 * k + 3] = (uint8_t)((data >> 24) & 0xFF);
            tx_buffer[4 * k + 2] = (uint8_t)((data >> 16) & 0xFF);
            tx_buffer[4 * k + 1] = (uint8_t)((data >> 8) & 0xFF);
            tx_buffer[4 * k] = (uint8_t)(data & 0xFF);
        }
    }

    while (1) {
        // Check if a key is pressed
        if (_kbhit()) {
            // Get the character without blocking
            char ch = _getch();

            // Print the character
            std::cout << "Key pressed: " << ch << std::endl;
            std::cout << "msg: " << msg << std::endl;

            // If the ESC key is pressed, break out of the loop
            if (state == 0 && ch == 49) {
                state = 1;
                idle_sent = 0;
                msg = ((uint32_t)state << 16) | (uint16_t)payload;
                // status = WriteFile(hComm, &StartByte, 1, &bytesWrite, NULL);
                status = WriteFile(hComm, &msg, 8, &bytesWrite, NULL);
                printf("sent");
                Sleep(5);
                status = SPI_Write(handle, &tx_buffer[0], NUM_OF_BYTES_PER_CHUNK * 2, ptransferCount, options);
            }

            if (state == 1 && ch == 50) {
                state = 2;
                msg = ((uint32_t)state << 16) | (uint16_t)payload;
                // status = WriteFile(hComm, &StartByte, 1, &bytesWrite, NULL);
                status = WriteFile(hComm, &msg, 8, &bytesWrite, NULL);
                i = 2;
                printf("break the loop \n");
                // break;
            }

            if (state == 2 && ch == 51) {
                state = 3;
            }

        }

        if (state == 2 || state == 3) {

            Sleep(1);
            status = SPI_Read(handle, &rx_buffer[0], NUM_OF_BYTES_PER_CHUNK, ptransferCount, options);
            Sleep(1);

            CHUNK_NUM_CURRENT = i % CHUNK_NUM;

            if (i % 2 == 1) {
                status = SPI_Write(handle, &tx_buffer[CHUNK_NUM_CURRENT * NUM_OF_BYTES_PER_CHUNK], NUM_OF_BYTES_PER_CHUNK, ptransferCount, options);
            }
            else {
                if (state == 2 && idle_sent == 0) {
                    memcpy(tx_message, &tx_buffer[CHUNK_NUM_CURRENT * NUM_OF_BYTES_PER_CHUNK], NUM_OF_BYTES_PER_CHUNK);
                    UCHAR extra_bytes[4] = { 0x03, 0x00, 0x00, 0x00 };
                    memcpy(&tx_message[NUM_OF_BYTES_PER_CHUNK], extra_bytes, 4);
                    status = SPI_Write(handle, tx_message, NUM_OF_BYTES_PER_CHUNK + 4, ptransferCount, options);
                    // status = SPI_Write(handle, &tx_buffer[CHUNK_NUM_CURRENT * NUM_OF_BYTES_PER_CHUNK], NUM_OF_BYTES_PER_CHUNK, ptransferCount, options);

                }
                else if (state == 3 && idle_sent == 0) {
                    memcpy(tx_message, &tx_buffer[CHUNK_NUM_CURRENT * NUM_OF_BYTES_PER_CHUNK], NUM_OF_BYTES_PER_CHUNK);
                    UCHAR extra_bytes[4] = { 0x04, 0x00, 0x00, 0x00 };
                    memcpy(&tx_message[NUM_OF_BYTES_PER_CHUNK], extra_bytes, 4);
                    status = SPI_Write(handle, tx_message, NUM_OF_BYTES_PER_CHUNK + 4, ptransferCount, options);
                    idle_sent = 1;
                }
                else if (idle_sent == 1) {
                    state = 0;
                }
            }

            if (state == 2 || state == 3) {
                if (i < COUNT) {
                    status2 = ReadFile(hComm, &RxByte, 1, &bytesWrite, NULL);
                }

                fprintf(fp, "\n");
                fprintf(fp2, "\n");
                printf("%d \n", RxByte);
                i = i + 1;
                if (i == COUNT) {
                    break;
                }
            }
        }
    }


     // status = SPI_Write(handle, &tx_buffer[0], NUM_OF_BYTES_PER_CHUNK * 2, ptransferCount, options);

    Cleanup_libMPSSE();
    return 0;
}

int main(int argc, char** argv) {
    test(argc, argv);
}
