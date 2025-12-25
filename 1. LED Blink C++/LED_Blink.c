/* * MPU SIDE CODE (Linux C) - ROBUST VERSION
 * Port: /dev/ttyHS1
 * Fixes: Flushes buffers to prevent hanging
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <errno.h>

#define RPMSG_DEV "/dev/ttyHS1" 

// BINARY COMMANDS
const unsigned char CMD_TURN_LED_ON[] = {
    0x94, 0x00, 0x02, 0xAB, 'c','m','d','_','l','e','d','_','o','f','f', 0x90 
};

const unsigned char CMD_TURN_LED_OFF[] = {
    0x94, 0x00, 0x01, 0xAA, 'c','m','d','_','l','e','d','_','o','n', 0x90 
};

int configure_serial(int fd) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) return -1;

    // Set Speed to 115200
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag &= ~PARENB; // No Parity
    tty.c_cflag &= ~CSTOPB; // 1 Stop Bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;     // 8 Bits

    tty.c_cflag &= ~CRTSCTS; // No Hardware Flow Control
    tty.c_cflag |= (CREAD | CLOCAL); // Enable Read

    // RAW Mode (Disable all formatting)
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    // TIMEOUT SETTINGS
    // VMIN=0, VTIME=10 means: "Wait up to 1.0 second. If no data, return 0."
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10; 

    if (tcsetattr(fd, TCSANOW, &tty) != 0) return -1;
    return 0;
}

int main() {
    int fd;
    unsigned char buffer[128];

    printf("--- RPC Client (Robust Mode) ---\n");

    fd = open(RPMSG_DEV, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("Error opening %s: %s\n", RPMSG_DEV, strerror(errno));
        return -1;
    }

    configure_serial(fd);

    printf("Starting Loop...\n");

    while(1) {
        // --- STEP 1: CLEAN THE PIPES ---
        // This is the fix! We delete any old data before talking.
        tcflush(fd, TCIOFLUSH);

        // --- STEP 2: TURN ON ---
        printf("Sending ON... ");
        fflush(stdout); // Force print immediately
        write(fd, CMD_TURN_LED_ON, sizeof(CMD_TURN_LED_ON));
        
        // Read Reply
        int n = read(fd, buffer, sizeof(buffer));
        if (n > 0) printf("[OK] MCU Replied\n");
        else       printf("[TIMEOUT] No Reply\n");

        sleep(1); 

        // --- STEP 1: CLEAN THE PIPES ---
        tcflush(fd, TCIOFLUSH);

        // --- STEP 3: TURN OFF ---
        printf("Sending OFF... ");
        fflush(stdout); 
        write(fd, CMD_TURN_LED_OFF, sizeof(CMD_TURN_LED_OFF));
        
        n = read(fd, buffer, sizeof(buffer));
        if (n > 0) printf("[OK] MCU Replied\n");
        else       printf("[TIMEOUT] No Reply\n");

        sleep(1); 
    }

    close(fd);
    return 0;
}
