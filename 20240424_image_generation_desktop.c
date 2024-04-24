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

#include <stdint.h>
#include <iostream>
#include <stdio.h>
#include <Windows.h>
#include <time.h>
#include <math.h>
#include "ftd2xx.h"
#include "libmpsse_spi.h"

#include <conio.h> // For getch() and kbhit()
#include <io.h>     // For _setmode()
#include <fcntl.h>  // For _O_BINARY

HANDLE hComm;

void print_and_quit(const char cstring[]) {
    printf("%s\n", cstring);
    getc(stdin);
    exit(1);
}
void Init_UART() {
    BOOL status;
    DCB dcbSerialParams = { 0 };
    COMMTIMEOUTS timeouts = { 0 };

    hComm = CreateFile(L"\\\\.\\COM7", GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (hComm == INVALID_HANDLE_VALUE) {
        printf("Error in opening serial port\n");
    }
    else {
        printf("Opening serial port successful\n");
    }
    status = PurgeComm(hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);
    if (!status) {
        printf("Error in purging comm\n");
        CloseHandle(hComm);
    }
    status = GetCommState(hComm, &dcbSerialParams);
    if (!status) {
        printf("Error in GetCommState\n");
        CloseHandle(hComm);
    }

    dcbSerialParams.BaudRate = CBR_9600;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    status = SetCommState(hComm, &dcbSerialParams);

    if (!status) {
        printf("Error in setting DCB structure\n");
        CloseHandle(hComm);
    }

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
FT_HANDLE Init_SPI() {
    FT_STATUS status;
    FT_DEVICE_LIST_INFO_NODE channelInfo;
    FT_HANDLE handle;

    DWORD channelCount = 0;
    status = SPI_GetNumChannels(&channelCount);

    if (status != FT_OK)
        print_and_quit("Error while checking the number of available MPSSE channels.");
    else if (channelCount < 1)
        print_and_quit("Error: No MPSSE channels are available.");

    printf("There are %d channels available. \n\n", channelCount);

    for (int i = 0; i < channelCount; i++) {
        status = SPI_GetChannelInfo(i, &channelInfo);
        if (status != FT_OK)
            print_and_quit("Error while getting details for an MPSSE channel.");
        printf("Channel number : %d\n", i);
        printf("Description: %s\n", channelInfo.Description);
        printf("Serial Number : %d\n", channelInfo.SerialNumber);
    }

    uint32_t channel = 0;
    printf("\n Enter a channel number to use: ");
    scanf_s("%d", &channel);

    status = SPI_OpenChannel(channel, &handle);
    if (status != FT_OK)
        print_and_quit("Error while opening the MPSSE channel.");

    ChannelConfig channelConfig;
    channelConfig.ClockRate = 15000000;
    channelConfig.configOptions = SPI_CONFIG_OPTION_MODE2 | SPI_CONFIG_OPTION_CS_DBUS3 | SPI_CONFIG_OPTION_CS_ACTIVELOW;
    channelConfig.LatencyTimer = 100;

    status = SPI_InitChannel(handle, &channelConfig);
    if (status != FT_OK)
        print_and_quit("Error while initializing the MPSSE channel.");
    return handle;
}
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
    Init_UART();
    Init_libMPSSE();
    handle = Init_SPI();
    fopen_s(&fp, "data.csv", "w+");
    fopen_s(&fp2, "data_2.csv", "w+");
    fopen_s(&fr, "EnginePattern_hexadecimal_Downsample_new_long2.txt", "r");
    fopen_s(&fr2, "radial_hexadecimal.txt", "r");

    FT_STATUS status;
    bool status2;
    DWORD bytesRead;
    DWORD bytesWrite;
    UCHAR StartByte = 0;
    UCHAR RxByte;

    DWORD transferCount = 0;
    LPDWORD ptransferCount = &transferCount;
    DWORD options = SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES | SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE | SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE;
    //Write how many trials user want
    int COUNT = 200;
    //Write how many positions user want inside a buffer
    //int NUM_OF_POSITIONS = 8112;
    int NUM_OF_POSITIONS;
    int NUM_OF_POSITIONS_RASTER = 9696;
    int NUM_OF_POSITIONS_RADIAL = 8112;
    //DWORD input_data[8112];
    DWORD input_data[10000];
    //Write how many bytes user want inside a buffer (4 * NUM_OF_POSITIONS)
    //UCHAR tx_buffer[32448];
    UCHAR tx_buffer[40000];
    UCHAR tx_idle[20000];
    //Write how many bytes user want to transmit & receive a time (2 * NUM_OF_POSITIONS)
    //int NUM_OF_BYTES_PER_CHUNK = 16224;
    int NUM_OF_BYTES_PER_CHUNK;
    int NUM_OF_BYTES_PER_CHUNK_RASTER = NUM_OF_POSITIONS_RASTER * 2;
    int NUM_OF_BYTES_PER_CHUNK_RADIAL = NUM_OF_POSITIONS_RADIAL * 2;
    int switch_occur = 0;
    //UCHAR rx_buffer[16224];
    UCHAR rx_buffer[20000];

    int x;
    int y;
    long position;
    int state = 0;
    int count = 0;
    int idle_sent = 0;

    while (1) {
        // Check if a key is pressed
        if (_kbhit()) {
            // Get the character without blocking
            char ch = _getch();

            // Print the character
            std::cout << "Key pressed: " << ch << std::endl;

            // If the ESC key is pressed, break out of the loop
            if (state == 0 && ch == 49) {
                state = 1;
                StartByte = 1;
                NUM_OF_POSITIONS = NUM_OF_POSITIONS_RASTER;
                NUM_OF_BYTES_PER_CHUNK = NUM_OF_BYTES_PER_CHUNK_RASTER;
                printf("I'm here \n");
                status = WriteFile(hComm, &StartByte, 1, &bytesWrite, NULL);
                for (int k = 0; k < NUM_OF_POSITIONS; ++k) {
                    fscanf_s(fr, "%x", &input_data[k]);
                }
                fseek(fr, 0, SEEK_SET);

                for (int k = 0; k < NUM_OF_POSITIONS; ++k) {
                    uint32_t data = input_data[k];
                    tx_buffer[4 * k + 3] = (uint8_t)((data >> 24) & 0xFF);
                    tx_buffer[4 * k + 2] = (uint8_t)((data >> 16) & 0xFF);
                    tx_buffer[4 * k + 1] = (uint8_t)((data >> 8) & 0xFF);
                    tx_buffer[4 * k] = (uint8_t)(data & 0xFF);
                }
                Sleep(5);
                status = SPI_Write(handle, &tx_buffer[0], 2 * NUM_OF_BYTES_PER_CHUNK, ptransferCount, options);
                status = ReadFile(hComm, &RxByte, 1, &bytesRead, NULL);
                count = count + 1;
                printf("%d \n", RxByte);
            }

            if (state == 0 && ch == 50) {
                state = 2;
                StartByte = 2;
                NUM_OF_POSITIONS = NUM_OF_POSITIONS_RADIAL;
                NUM_OF_BYTES_PER_CHUNK = NUM_OF_BYTES_PER_CHUNK_RADIAL;
                printf("I'm here2 \n");
                status = WriteFile(hComm, &StartByte, 1, &bytesWrite, NULL);
                for (int k = 0; k < NUM_OF_POSITIONS; ++k) {
                    fscanf_s(fr2, "%x", &input_data[k]);
                }
                fseek(fr2, 0, SEEK_SET);

                for (int k = 0; k < NUM_OF_POSITIONS; ++k) {
                    uint32_t data = input_data[k];
                    tx_buffer[4 * k + 3] = (uint8_t)((data >> 24) & 0xFF);
                    tx_buffer[4 * k + 2] = (uint8_t)((data >> 16) & 0xFF);
                    tx_buffer[4 * k + 1] = (uint8_t)((data >> 8) & 0xFF);
                    tx_buffer[4 * k] = (uint8_t)(data & 0xFF);
                }
                Sleep(5);
                status = SPI_Write(handle, &tx_buffer[0], 2 * NUM_OF_BYTES_PER_CHUNK, ptransferCount, options);
                status = ReadFile(hComm, &RxByte, 1, &bytesRead, NULL);
                count = count + 1;
                printf("%d \n", RxByte);
            }

            if (state == 1 && ch == 48) {
                state = 3;
            }
            if (state == 2 && ch == 48) {
                state = 4;
            }
            if (ch == 27) {
                state = 5;
            }
            if (state == 1 && ch == 50) {
                state = 3;
                switch_occur = 1;
            }
            if (state == 2 && ch == 49) {
                state = 4;
                switch_occur = 1;
            }
        }

        if (state == 1 || state == 2 || state == 3 || state == 4 || state == 5) {
            Sleep(1);
            status = SPI_Read(handle, &rx_buffer[0], NUM_OF_BYTES_PER_CHUNK, ptransferCount, options);
            Sleep(1);
            if (state == 1 || state == 3) {
                for (int k = 0; k < NUM_OF_POSITIONS; ++k) {
                    fscanf_s(fr, "%x", &input_data[k]);
                }
                fseek(fr, 0, SEEK_SET);
            }
            else if (state == 2 || state == 4) {
                for (int k = 0; k < NUM_OF_POSITIONS; ++k) {
                    fscanf_s(fr2, "%x", &input_data[k]);
                }
                fseek(fr2, 0, SEEK_SET);
            }

            for (int k = 0; k < NUM_OF_POSITIONS; ++k) {
                uint32_t data = input_data[k];
                tx_buffer[4 * k + 3] = (uint8_t)((data >> 24) & 0xFF);
                tx_buffer[4 * k + 2] = (uint8_t)((data >> 16) & 0xFF);
                tx_buffer[4 * k + 1] = (uint8_t)((data >> 8) & 0xFF);
                tx_buffer[4 * k] = (uint8_t)(data & 0xFF);
            }
            //printf("%d \n", tx_buffer[5020]);
            if (RxByte == 0) {
                status = SPI_Write(handle, &tx_buffer[0], NUM_OF_BYTES_PER_CHUNK, ptransferCount, options);
            }
            else if (RxByte == 1) {
                if (state == 3 || state == 4 || state == 5) {
                    tx_buffer[2 * NUM_OF_BYTES_PER_CHUNK] = 3;
                    idle_sent = 1;
                }
                else {
                    tx_buffer[2 * NUM_OF_BYTES_PER_CHUNK] = 1;
                }
                status = SPI_Write(handle, &tx_buffer[NUM_OF_BYTES_PER_CHUNK], NUM_OF_BYTES_PER_CHUNK + 1, ptransferCount, options);
            }
            else if (RxByte == 2) {
                status = SPI_Write(handle, &tx_buffer[0], NUM_OF_BYTES_PER_CHUNK, ptransferCount, options);
            }
            else if (RxByte == 3) {
                if (state == 3 || state == 4 || state == 5) {
                    tx_buffer[2 * NUM_OF_BYTES_PER_CHUNK] = 3;
                    idle_sent = 1;
                }
                else {
                    tx_buffer[2 * NUM_OF_BYTES_PER_CHUNK] = 1;
                }
                status = SPI_Write(handle, &tx_buffer[NUM_OF_BYTES_PER_CHUNK], NUM_OF_BYTES_PER_CHUNK + 1, ptransferCount, options);
            }
            if (count == COUNT + 1)
                break;
            if ((count < COUNT + 1) && idle_sent == 0) {
                status2 = ReadFile(hComm, &RxByte, 1, &bytesWrite, NULL);
                count = count + 1;
            }
            for (int j = 0; j < NUM_OF_BYTES_PER_CHUNK; j += 4) {
                x = rx_buffer[j + 3] * 256 + rx_buffer[j + 2];
                y = rx_buffer[j + 1] * 256 + rx_buffer[j];
                //fprintf(fp, "y:%d x:%d ,", y, x);
                fprintf(fp, "%d/%d,", x, y);
                fprintf(fp2, "%d,", j);
            }
            if (count % 2 == 1) {
                fprintf(fp, "\n");
                fprintf(fp2, "\n");
            }
            printf("%d \n", RxByte);
        }
        if ((state == 3 || state == 4) && (idle_sent == 1)) {
            if (switch_occur == 0) {
                state = 0;
                idle_sent = 0;
                tx_buffer[2 * NUM_OF_BYTES_PER_CHUNK] = 0;
            }
            else if (switch_occur == 1) {
                if (state == 3) {
                    state = 2;
                    StartByte = 2;
                    NUM_OF_POSITIONS = NUM_OF_POSITIONS_RADIAL;
                    NUM_OF_BYTES_PER_CHUNK = NUM_OF_BYTES_PER_CHUNK_RADIAL;
                    Sleep(200);
                    status = WriteFile(hComm, &StartByte, 1, &bytesWrite, NULL);
                }
                else if (state == 4) {
                    state = 1;
                    StartByte = 1;
                    NUM_OF_POSITIONS = NUM_OF_POSITIONS_RASTER;
                    NUM_OF_BYTES_PER_CHUNK = NUM_OF_BYTES_PER_CHUNK_RASTER;
                    Sleep(200);
                    status = WriteFile(hComm, &StartByte, 1, &bytesWrite, NULL);
                }
                switch_occur = 0;
                idle_sent = 0;
                tx_buffer[2 * NUM_OF_BYTES_PER_CHUNK] = 0;
            }
        }
        if (state == 5 && idle_sent == 1) {
            break;
        }
    }
    Cleanup_libMPSSE();
    return 0;
}

int main(int argc, char** argv) {
    test(argc, argv);
}
