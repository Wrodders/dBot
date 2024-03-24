#ifndef CALIB_H
#define CALIB_H


// Run Calibration Routines Via Lookup Configurabled Lookup Tables




typedef struct ChirpTest{
    float *buf;
    size_t size;
    int index;
    int rampDir; 
}ChirpTest;


static ChirpTest chirpInit(float * const buf,  size_t size){
    //@Brief:
    ChirpTest chirp = {
        .buf = buf,
        .size = size,
        .rampDir = 1,  // 1 or -1
        .index = 0
    };
    return chirp;
}

static float chirpGetFreq(float t, float min_freq, float max_freq, float period) {
    // Calculate the chirp frequency based on time
    float slope = (max_freq - min_freq) / period;
    return min_freq + slope * t;
}


static void chirpComputeLUT(ChirpTest *chirp, float min_freq, float max_freq, 
                            float period, float w1, float w2, float M) {
    //@Brief: Computes a polulatea LUT
    float t_increment = period / chirp->size;
    float t = 0.0;

    for (int i = 0; i < CHIRP_BUF_SIZE; i++) {
        float frequency = chirpGetFreq(t, min_freq, max_freq, period);
        float amplitude = 1.0; // Adjust amplitude as needed

        // Calculate the value of the chirp waveform (sine wave)
       float value = amplitude * cos(w1*t + (w2 - w1)*t*t/(2*M));

        // Store the value in the array
        chirp->buf[i] = value;

        // Increment time
        t += t_increment;
    }
}

static float chirpNext(ChirpTest *chirp) {
    //@Brief: Gets the next value of the chirp LUT, pingPongs left to right
    if (chirp->rampDir > 0) {
        if (chirp->index++ >= CHIRP_BUF_SIZE - 1) {chirp->rampDir = -1;}
    } else {
        if (chirp->index-- <= 0) {chirp->rampDir = 1;}
    }
    return chirp->buf[chirp->index];
}


#endif // CALIBRATION_H