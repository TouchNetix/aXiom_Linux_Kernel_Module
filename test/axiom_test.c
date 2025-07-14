// axiom_test.c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

// Set the firmware and configuration file names
#define FIRMWARE_FILENAME "axiom_firmware.alc"
#define CONFIG_FILENAME "axiom_config.bin"

// IOCTL commands for configuration and firmware updates
#define AXIOM_CFG_UPDATE _IOW('a', 'a', char*)
#define AXIOM_CFG_CHECKSUM _IOR('a', 'b', int32_t*)
#define AXIOM_FW_UPDATE _IOW('a', 'c', char*)
#define AXIOM_FW_CHECKSUM _IOR('a', 'd', char*)

int main(int argc, char *argv[])
{
    int fd;
    int ret;
    int checksum;
    char fw_checksum[64];

    fd = open("/dev/axiom_device", O_RDWR);
    if (fd < 0) {
        perror("open");
        return 1;
    }

    // Test firmware update
    printf("Testing firmware update...\n");
    ret = ioctl(fd, AXIOM_FW_UPDATE, FIRMWARE_FILENAME);
    if (ret < 0) {
        perror("AXIOM_FW_UPDATE");
    } else {
        printf("Firmware update successful.\n");
    }

    // Test configuration update
    printf("Testing configuration update...\n");
    ret = ioctl(fd, AXIOM_CFG_UPDATE, CONFIG_FILENAME);
    if (ret < 0) {
        perror("AXIOM_CFG_UPDATE");
    } else {
        printf("Configuration update successful.\n");
    }

    // Get configuration checksum
    printf("Testing configuration checksum...\n");
    ret = ioctl(fd, AXIOM_CFG_CHECKSUM, &checksum);
    if (ret < 0) {
        perror("AXIOM_CFG_CHECKSUM");
    } else {
        printf("Configuration checksum: %d\n", checksum);
    }

    // Get firmware checksum
    printf("Testing firmware checksum...\n");
    memset(fw_checksum, 0, sizeof(fw_checksum));
    ret = ioctl(fd, AXIOM_FW_CHECKSUM, fw_checksum);
    if (ret < 0) {
        perror("AXIOM_FW_CHECKSUM");
    } else {
        printf("Firmware checksum: %s\n", fw_checksum);
    }

    close(fd);
    return 0;
}