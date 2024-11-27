#include <stdio.h>
#include <stdlib.h>
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

    // Open the FIFO for reading
    int fd = open(fifo_path, O_RDONLY);

    if (fd == -1) {
        perror("open");
        return 1;
    }

    // Read the struct from the pipe
    Data data;
    if (read(fd, &data, sizeof(data)) == -1) {
        perror("read");
    } else {
        printf("Received: id=%d, value=%.2f, name=%s\n", data.id, data.value, data.name);
    }

    close(fd);
    return 0;
}

