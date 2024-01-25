// test_utils_assert.c
#include <stdint.h> 
#include "../common/utils.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>


void test_clamp(void) {
    assert(_clamp(5.0, 0.0, 10.0) == 5.0);
    assert(_clamp(-5.0, 0.0, 10.0) == 0.0);
    assert(_clamp(15.0, 0.0, 10.0) == 10.0);
}

void test_fabs(void) {
    assert(_fabs(5.0) == 5.0);
    assert(_fabs(-5.0) == 5.0);
    assert(_fabs(0.0) == 0.0);
}

void test_round(void) {
    assert(_round(5.2, 0) == 5.0);
    assert(_round(5.246, 2) == 5.25);
    assert(_round(-5.246, 2) == -5.25);
}

void test_uClen(void) {
    assert(uClen("Hello, World!") == 13);
    assert(uClen("") == 0);
}

void test_uCpy(void) {
    char buf[20];

    assert(uCpy(buf, "Hello", sizeof(buf)) == 5);
    assert(strcmp(buf, "Hello") == 0);

    assert(uCpy(buf, "This is too long", 5) == 0);
}

void test_ftoa(void) {
    char buf[20];
    assert(ftoa(buf, 123.456, 2) == 6);
    assert(strcmp(buf, "123.45") == 0);

    assert(ftoa(buf, -789.01234, 4) == 9);
    assert(strcmp(buf, "-789.0123") == 0);
    
    assert(ftoa(buf, -0.01234, 4) == 7);
    assert(strcmp(buf, "-0.0123") == 0);
}

void test_itoa(void) {
    char buf[20];
    
    assert(itoa(buf, 123, 10) == 3);
    assert(strcmp(buf, "123") == 0);

    assert(itoa(buf, -456, 10) == 4);
    assert(strcmp(buf, "-456") == 0);

    assert(itoa(buf, 0, 10) == 1);
    assert(strcmp(buf, "0") == 0);
}

void test_mysprintf(void) {
    char buf[50];

    assert(mysprintf(buf, 2, "Value: %d", 42) == 9);
    assert(strcmp(buf, "Value: 42") == 0);

    assert(mysprintf(buf, 3, "Float: %f", 3.14159) == 12);
    assert(strcmp(buf, "Float: 3.141") == 0);

    assert(mysprintf(buf, 4,"%f:%f:%f", -0.12345,-0.98765,-1.456789) ==  23);
    assert(strcmp(buf, "-0.1235:-0.9876:-1.4567")== 0);

    assert(mysprintf(buf, 3, "Value: %d, Float: %f, Text: %s", 4,3.14159,"hello" ) == 35);
    assert(strcmp(buf, "Value: 4, Float: 3.141, Text: hello") == 0);
}

int main(void) {
    printf("Running tests on utils.h ...\n");
    test_clamp();
    test_fabs();
    test_round();
    test_uClen();
    test_uCpy();
    test_ftoa();
    test_itoa();
    test_mysprintf();

    printf("All tests passed!\n");

    return 0;
}
