#ifndef UTILS_H
#define UTILS_H

#include <stdarg.h>

static float _clamp(float val, float min, float max){
    const float clamp = val < min ? min : val;
    return clamp > max ? max : clamp;
}

static float _fabs(float f){
    return ( f > 0 ? f : -f);
}

static float _round(float number, int decimalPlaces) {
    // Handle the sign of the number
    int sign = (number < 0) ? -1 : 1;
    number = _fabs(number); // Work with the absolute value
    
    float multiplier = 1.0;
    for (int i = 0; i < decimalPlaces; i++) {
        multiplier *= 10.0;
    }

    float roundedAbsolute = (int)(number * multiplier + 0.5) / multiplier;
    
    // Restore the sign
    return sign * roundedAbsolute;
}
static unsigned int uClen(const char *str){
    //@brief: Return the len of a string 
    //@Note:  Excludes the null terminator
    int len = 0;
    while(*str++ != '\0'){
        len++;
    }
    return len;
}

static int ftoa(char *buf, float val, int precision){
    int intPart = (int)val;
    float fractionPart =  (val - intPart);
    if(val < 0){
        buf[0] = '-';
        buf++;
        intPart = -intPart;
        fractionPart = -fractionPart;
    }

    int i = 0;
    // Convert the whole part to string
    if (intPart == 0) {
        buf[i++] = '0';
    } else {
        while (intPart > 0) {
            buf[i++] = '0' + (intPart % 10);
            intPart /= 10;
        }
    }
    buf[i++] = '.';
    for(int j  = 0; j  < precision; j++){
        fractionPart *= 10; // shift decimal place
        intPart = (int)fractionPart; // extract int
        buf[i++] = '0' + intPart; // convert to char
        fractionPart -= intPart; // remove int part
    }
    buf[i++] = '\0'; // null terminate

    return i;
}

static int itoa(char *buf, int num, int base){

    int i = 0;
    int isNegative = 0;
    if(num == 0){
        buf[i++] = '0';
        buf[i] = '\0';
        return i;
    }
    if (num < 0 && base !=10){
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

    buf[i] = '\0';

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
    return i;
}

enum formatState{
    TEXT,
    FLOAT, // f
    INT, // d

};


static int mysprintf(char *buf, uint8_t dp, char *format, ...){
    va_list args;
    va_start(args, format);
    enum formatState state = TEXT;
    int prescision = 0;
    int len = uClen(format);

    int fIdx = 0;
    int bIdx = 0;
  
    while(format[fIdx]){ 
        switch (state){
            case TEXT:
               
                if(format[fIdx] == '%' && format[fIdx + 1] == 'f'){
                    state = FLOAT;
                    break;
                }
                else if(format[fIdx] == '%' && format[fIdx + 1]== 'd'){
                    state = INT;
                    break;
                }
                else{
                    buf[bIdx++] = format[fIdx++]; // add text
                    break;
                }
            case FLOAT:
                float val = va_arg(args, double);
                bIdx +=  ftoa(&buf[bIdx], val, dp);
                state = TEXT;
                fIdx += 2; // skip value format
                break;
            case INT:
                uint32_t val2 = va_arg(args,uint32_t);
                bIdx += itoa(&buf[bIdx],val2, 10);
                state = TEXT;
                fIdx += 2;
                break;                
        }
    }
    va_end(args);
    return bIdx;
}



#endif // UTILS_H