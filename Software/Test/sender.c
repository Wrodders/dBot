#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

typedef struct {
    int id;
    float value;
    char name[20];
} Data;

int main() {
    const char *fifo_path = "/tmp/myfifo";

    // Create the FIFO if it does not exist
    if (mkfifo(fifo_path, 0666) == -1 && errno != EEXIST) {
        perror("mkfifo");
        return 1;
    }

    // Open the FIFO for writing
    int fd = open(fifo_path, O_WRONLY);

    if (fd == -1) {
        perror("open");
        return 1;
    }

    // Create and populate the struct
    Data data = {42, 3.14, "example"};

    // Write the struct to the pipe
    if (write(fd, &data, sizeof(data)) == -1) {
        perror("write");
    }

    close(fd);
    return 0;
}
