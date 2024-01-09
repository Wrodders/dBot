#ifndef UTILS_H
#define UTILS_H

#include <stdarg.h>


// *** // MATH // **** // 

typedef struct vector_t{
    float x,y,z;
}vector_t;

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

static float invSqrt(float x) {
    union {
    float    f;
    uint32_t i;
    } conv;

    float       x2;
    const float threehalfs = 1.5F;

    x2     = x * 0.5F;
    conv.f = x;
    conv.i = 0x5f3759df - (conv.i >> 1); // ????
    conv.f = conv.f * (threehalfs - (x2 * conv.f * conv.f));
    return conv.f;
}



// *** // **** // Float Maniplation // **** // *** //
static float _clamp(float val, float min, float max){
    const float clamp = val < min ? min : val;
    return clamp > max ? max : clamp;
}

static float _fabs(float f){
    return ( f > 0 ? f : -f);
}

static float _round(float number, int decimalPlaces) {
    int sign = (number < 0) ? -1 : 1; // Handle the sign 
    number = _fabs(number); // Work with the absolute value
    
    float multiplier = 1.0;
    for (int i = 0; i < decimalPlaces; i++) {
        multiplier *= 10.0;
    }

    float roundedAbsolute = (int)(number * multiplier + 0.5) / multiplier;
    
    // Restore the sign
    return sign * roundedAbsolute;
}


// *** // **** // String Manipulation // **** // *** //
static unsigned int uClen(const char *str){
    //@brief: Return the len of a string 
    //@Note:  Excludes the null terminator
    int len = 0;
    while(*str++ != '\0'){
        len++;
    }
    return len;
}


static unsigned int uCpy(char *buf, char *src, int len){
    //@Breif: Copys null-terminated string to buffer up to len
    //@Note: setting len to 0 will copy up to null null char
    //@Return: num bytes copied, 0 for failure
    int i = 0;
    while(*src != '\0'){
        if(i > len && len != 0) {return 0;}
        buf[i++] = *src++;
    }
    buf[i] = '\0';
    return i;
}


static int ftoa(char *buf, float val, int precision){
    //@Breif: Converts floating point value to ASCII null-terminated string with specifed precision truncation
    //@Returns: Len of ASCII string

    float fval = _round(val,precision);
    int intPart = (int)fval; // truncate at decimal point
    float fractionPart =  (fval - intPart); // get remainder
    int start = 0;
    int i = 0;
    if(fval < 0){
        buf[i++] = '-';
        intPart = -intPart;
        fractionPart = -fractionPart;
    }

    // Convert the whole part to string
    if (intPart == 0) {
        buf[i++] = '0'; // pad with 0
    } else {
        start = i; // Save the start index of the whole part
        while (intPart > 0) {
            buf[i++] = '0' + (intPart % 10); // pop LSB digit
            intPart /= 10; //  shift decimal place right
        }
    }
    // reverse whole part
    int end = i - 1;
    while (start < end) {
        char temp = buf[start];
        buf[start] = buf[end];
        buf[end] = temp;
        start++;
        end--;
    }

    buf[i++] = '.';
    for(int j  = 0; j < precision; j++){
        fractionPart *= 10; // shift decimal place right
        intPart = (int)fractionPart; // extract int
        buf[i++] = '0' + intPart; // convert to char
        fractionPart -= intPart; // remove int part
    }
    buf[i] = '\0'; // null terminate

    return i;
}

static int itoa(char *buf, int num, int base){
    //@Brief: Converts integer to ASCII null-terminated String 
    //@Returns: Len of string

    int i = 0;
    int isNegative = 0;
    if(num == 0){
        buf[i++] = '0';
        buf[i] = '\0';
        return i;
    }
    if (num < 0 && base == 10){
        isNegative = 1;
        num = -num;
    }

    while(num !=0){
        int rem = num % base;
        buf[i++] = (rem > 9) ? (rem - 10) + 'a' : rem + '0';
        num = num /base;
    }
    if (isNegative) {
        buf[i++] = '-';
    }

    

    // Reverse the string
    int left = 0;
    int right = i - 1;
    while (left < right) {
        char temp = buf[left];
        buf[left] = buf[right];
        buf[right] = temp;
        left++;
        right--;
    }
    buf[i] = '\0';
    return i;
}

enum formatState{
    TEXT,
    FLOAT, // f
    INT, // d
    STR, //s
};

static int mysprintf(char *buf, uint8_t dp, char *format, ...){
    va_list args;
    va_start(args, format);
    enum formatState state = TEXT;
    int len = uClen(format);

    int fIdx = 0; // format index
    int bIdx = 0; // buffer index
  
    while(format[fIdx]){ 
        switch (state){
            case TEXT:
                if(format[fIdx] == '%' && format[fIdx + 1] == 'f'){
                    state = FLOAT;
                    break;
                }
                else if(format[fIdx] == '%' && format[fIdx + 1] == 'd'){
                    state = INT;
                    break;
                }
                else if(format[fIdx] == '%' && format[fIdx + 1] == 's'){
                    state = STR;
                    break;
                }
                else{
                    buf[bIdx++] = format[fIdx++]; // add text
                }
                break;
            case FLOAT:{
                bIdx +=  ftoa(&buf[bIdx], va_arg(args, double), dp);
                state = TEXT;
                fIdx += 2; // skip value format
                break;
            }
            case INT: {
                bIdx += itoa(&buf[bIdx],va_arg(args,int), 10);
                state = TEXT;
                fIdx += 2;
                break;  
            }       
            case STR: {
                // must be null terminated
                bIdx += uCpy(&buf[bIdx], va_arg(args, char *), 0); // copy unknown num chars
                state = TEXT;
                fIdx +=2;
            }       
        }
    }
    va_end(args);
    return bIdx;
}


#endif // UTILS_H