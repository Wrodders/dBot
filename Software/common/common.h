#ifndef COMMON_H
#define COMMON_H


#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include <sys/fcntl.h>
#include <termios.h>
#include <errno.h>
#include <signal.h>
#include <sys/time.h>
#include <signal.h>
#include <sys/select.h>

volatile sig_atomic_t stop = 0;

#define SERIAL_PORT "/dev/cu.usbserial-130"
#endif // COMMON_H