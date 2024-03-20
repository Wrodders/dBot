#ifndef CALIB_H
#define CALIB_H


// Run Calibration Routines Via Lookup Configurabled Lookup Tables

#define ARRAY_SIZE 100 // Adjust array size as needed
#define POPULATION_INTERVAL 5 // Population interval in seconds


typedef struct ChirpTest{
    float *buf;
    size_t size;
    size_t index;
    int rampDir; 
}ChirpTest;


static ChirpTest chirpInit(float const *buf,  size_t size, float minFreq, float  maxFreq, float period){
    //@Brief:
    ChirpTest chirp = {
        .buf = buf,
        .size = size,
        .rampDir = 1,  // 1 or -1
        .index = 0
    };

    chirpCompute(chirp->buf, minFreq, maxFreq, period);
    return chirp;
}

double chirpGetFreq(double t, double min_freq, double max_freq, double period) {
    // Calculate the chirp frequency based on time
    double slope = (max_freq - min_freq) / period;
    return min_freq + slope * t;
}

// Function to populate the array with a chirp waveform
void chirpCompute(float *array, double min_freq, double max_freq, double period) {
    double t_increment = period / CHIRP_BUF_SIZE;
    double t = 0.0;

    for (int i = 0; i < CHIRP_BUF_SIZE; i++) {
        double frequency = chirpGetFreq(t, min_freq, max_freq, period);
        double amplitude = 1.0; // Adjust amplitude as needed

        // Calculate the value of the chirp waveform (sine wave)
       double value = amplitude * sin(0.1*t + (0.0f - 0.1f)*t*t*1/(2*0.1));

        // Store the value in the array
        array[i] = (float)value;

        // Increment time
        t += t_increment;
    }
}

float chirpNext(ChirpTest *chirp) {
    if (chirp->rampDir >= 0) {
        if (chirp->index++ == CHIRP_BUF_SIZE - 1) {chirp->rampDir = -1;}
    } else {
        if (chirp->index-- <= 0) {chirp->rampDir = 1;}
    }
    return chirp->buf[chirp->index];
}


#endif // CALIBRATION_H