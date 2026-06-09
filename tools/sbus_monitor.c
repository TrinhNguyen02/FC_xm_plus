#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <windows.h>

static bool parse_channel_line(const char *line, unsigned int *out_channel, unsigned int *out_value)
{
    char lower[256];
    size_t len = strlen(line);
    if (len >= sizeof(lower)) {
        return false;
    }
    for (size_t i = 0; i < len; i++) {
        lower[i] = (char)tolower((unsigned char)line[i]);
    }
    lower[len] = '\0';

    char *p = strstr(lower, "ch");
    if (p == NULL) {
        return false;
    }

    unsigned int channel = 0;
    unsigned int value = 0;
    if (sscanf(p, "ch%u : %u", &channel, &value) == 2 ||
        sscanf(p, "ch%u: %u", &channel, &value) == 2 ||
        sscanf(p, "ch%u:%u", &channel, &value) == 2 ||
        sscanf(p, "ch%u %u", &channel, &value) == 2) {
        if (channel >= 1 && channel <= 16) {
            *out_channel = channel;
            *out_value = value;
            return true;
        }
    }
    return false;
}

// Set cursor position
void set_cursor_position(int x, int y)
{
    HANDLE handle = GetStdHandle(STD_OUTPUT_HANDLE);
    COORD coord = { x, y };
    SetConsoleCursorPosition(handle, coord);
}

// Get cursor position
void get_cursor_position(int *x, int *y)
{
    HANDLE handle = GetStdHandle(STD_OUTPUT_HANDLE);
    CONSOLE_SCREEN_BUFFER_INFO info;
    GetConsoleScreenBufferInfo(handle, &info);
    *x = info.dwCursorPosition.X;
    *y = info.dwCursorPosition.Y;
}

// Clear line
void clear_line()
{
    HANDLE handle = GetStdHandle(STD_OUTPUT_HANDLE);
    CONSOLE_SCREEN_BUFFER_INFO info;
    GetConsoleScreenBufferInfo(handle, &info);
    
    COORD coord = { 0, info.dwCursorPosition.Y };
    DWORD written;
    FillConsoleOutputCharacter(handle, ' ', 120, coord, &written);
    SetConsoleCursorPosition(handle, coord);
}

int main(int argc, char *argv[])
{
    HANDLE com_port;
    char com_port_name[32] = "COM6";  // Default COM port
    
    if (argc > 1) {
        snprintf(com_port_name, sizeof(com_port_name), "\\\\.\\%s", argv[1]);
    } else {
        snprintf(com_port_name, sizeof(com_port_name), "\\\\.\\COM6");
    }

    printf("Opening port: %s\n", com_port_name + 4);  // Skip \\.\

    // Open COM port
    com_port = CreateFileA(
        com_port_name,
        GENERIC_READ,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL
    );

    if (com_port == INVALID_HANDLE_VALUE) {
        printf("Failed to open COM port\n");
        return 1;
    }

    // Configure COM port
    DCB dcb = { 0 };
    dcb.DCBlength = sizeof(dcb);
    
    if (!GetCommState(com_port, &dcb)) {
        printf("Failed to get COM state\n");
        CloseHandle(com_port);
        return 1;
    }

    dcb.BaudRate = 100000;
    dcb.ByteSize = 8;
    dcb.StopBits = TWOSTOPBITS;
    dcb.Parity = EVENPARITY;

    if (!SetCommState(com_port, &dcb)) {
        printf("Failed to set COM state\n");
        CloseHandle(com_port);
        return 1;
    }

    // Set timeouts
    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 500;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    SetCommTimeouts(com_port, &timeouts);

    // Purge any existing data in buffer
    PurgeComm(com_port, PURGE_RXCLEAR | PURGE_TXCLEAR);

    printf("Port opened successfully\n");
    printf("Waiting for channel logs from COM port...\n");
    printf("(Expect lines like: ch1: 123\nch2: 567\n...)\n\n");

    uint8_t buffer[1024];
    DWORD bytes_read;
    char linebuf[256];
    size_t linepos = 0;
    uint16_t channels[16] = {0};
    uint16_t prev_channels[16] = {0};
    bool have_channel[16] = {0};
    int have_count = 0;
    bool first_display = true;
    int start_y = -1;

    while (1) {
        if (!ReadFile(com_port, buffer, sizeof(buffer), &bytes_read, NULL)) {
            printf("Error reading from COM port\n");
            break;
        }

        if (bytes_read == 0) {
            continue;
        }

        for (DWORD i = 0; i < bytes_read; i++) {
            uint8_t b = buffer[i];
            if (b == '\r') {
                continue;
            }

            if (b == '\n' || linepos >= sizeof(linebuf) - 1) {
                linebuf[linepos] = '\0';
                linepos = 0;

                char *line = linebuf;
                while (*line == ' ' || *line == '\t') {
                    line++;
                }

                unsigned int channel = 0;
                unsigned int value = 0;
                if (parse_channel_line(line, &channel, &value)) {
                    int idx = channel - 1;
                    bool changed = !have_channel[idx] || channels[idx] != value;
                    channels[idx] = (uint16_t)value;
                    if (!have_channel[idx]) {
                        have_channel[idx] = true;
                        have_count++;
                    }

                    if (have_count == 16 && (first_display || changed)) {
                        if (first_display) {
                            int x, y;
                            get_cursor_position(&x, &y);
                            start_y = y;
                            first_display = false;
                        } else {
                            set_cursor_position(0, start_y);
                        }

                        for (int row = 0; row < 4; row++) {
                            for (int col = 0; col < 4; col++) {
                                int idx = row * 4 + col;
                                printf("CH%2d: %4u   ", idx + 1, channels[idx]);
                            }
                            printf("\n");
                        }
                        fflush(stdout);
                        memcpy(prev_channels, channels, sizeof(prev_channels));
                    }
                }
                continue;
            }

            linebuf[linepos++] = (char)b;
        }
    }

    CloseHandle(com_port);
    return 0;
}
