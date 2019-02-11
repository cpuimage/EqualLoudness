
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define DR_WAV_IMPLEMENTATION

#include "dr_wav.h"

#define DR_MP3_IMPLEMENTATION


#include "dr_mp3.h"

#include "timing.h"
#ifndef MAX
#define MAX(a, b)                    (((a) > (b)) ? (a) : (b))
#endif

void wavWrite_f32(char *filename, float *buffer, int sampleRate, uint32_t totalSampleCount, uint32_t channels) {
    drwav_data_format format;
    format.container = drwav_container_riff;
    format.format = DR_WAVE_FORMAT_IEEE_FLOAT;
    format.channels = channels;
    format.sampleRate = (drwav_uint32) sampleRate;
    format.bitsPerSample = 32;
    drwav *pWav = drwav_open_file_write(filename, &format);
    if (pWav) {
        drwav_uint64 samplesWritten = drwav_write(pWav, totalSampleCount, buffer);
        drwav_uninit(pWav);
        if (samplesWritten != totalSampleCount) {
            fprintf(stderr, "write file [%s] error.\n", filename);
            exit(1);
        }
    }
}

float *wavRead_f32(const char *filename, uint32_t *sampleRate, uint64_t *sampleCount, uint32_t *channels) {
    drwav_uint64 totalSampleCount = 0;
    float *input = drwav_open_file_and_read_pcm_frames_f32(filename, channels, sampleRate, &totalSampleCount);
    if (input == NULL) {
        drmp3_config pConfig;
        input = drmp3_open_file_and_read_f32(filename, &pConfig, &totalSampleCount);
        if (input != NULL) {
            *channels = pConfig.outputChannels;
            *sampleRate = pConfig.outputSampleRate;
        }
    }
    if (input == NULL) {
        fprintf(stderr, "read file [%s] error.\n", filename);
        exit(1);
    }
    *sampleCount = totalSampleCount * (*channels);
    return input;
}


void splitpath(const char *path, char *drv, char *dir, char *name, char *ext) {
    const char *end;
    const char *p;
    const char *s;
    if (path[0] && path[1] == ':') {
        if (drv) {
            *drv++ = *path++;
            *drv++ = *path++;
            *drv = '\0';
        }
    } else if (drv)
        *drv = '\0';
    for (end = path; *end && *end != ':';)
        end++;
    for (p = end; p > path && *--p != '\\' && *p != '/';)
        if (*p == '.') {
            end = p;
            break;
        }
    if (ext)
        for (s = end; (*ext = *s++);)
            ext++;
    for (p = end; p > path;)
        if (*--p == '\\' || *p == '/') {
            p++;
            break;
        }
    if (name) {
        for (s = p; s < end;)
            *name++ = *s++;
        *name = '\0';
    }
    if (dir) {
        for (s = path; s < p;)
            *dir++ = *s++;
        *dir = '\0';
    }
}

void printUsage() {
    printf("usage:\n");
    printf("./EqualLoudness input.wav\n");
    printf("./EqualLoudness input.mp3\n");
    printf("or\n");
    printf("./EqualLoudness input.wav output.wav\n");
    printf("./EqualLoudness input.mp3 output.wav\n");
    printf("press any key to exit.\n");
    getchar();
}

void renormalize(float *x) {
    union {
        float f;
        uint32_t i;
    } in;
    in.f = *x;
    *x *= (!(((in.i >> 23) & 0xFF) == 0xFF && (in.i & 0x007FFFFF) != 0));
}

void updateStateLine(float *state, size_t size,
                     const float *denominator, const float *numerator,
                     const float *in, const float *out) {
    for (size_t k = 1; k < size; ++k) {
        state[k - 1] = (numerator[k] * *in - denominator[k] * *out) + state[k];
        renormalize(state + (k - 1));
    }
}

void IIR_filter(const float *input, float *output, size_t sampleCount, float *denominator,
                size_t denominator_size,
                float *numerator, size_t numerator_size
) {
    size_t wantedSize = MAX(numerator_size, denominator_size);
    float *state = (float *) calloc(wantedSize, sizeof(float));
    if (numerator == NULL || denominator == NULL || state == NULL || output == NULL || input == NULL ||
        denominator[0] == 0.0f) {
        return;
    } 
    float norm = 1.0f / denominator[0];
    if (norm != 1.f) {
        for (size_t i = 1; i < denominator_size; ++i) {
            denominator[i] *= norm;
        }
        if (denominator_size != 1) {
            for (size_t i = 0; i < numerator_size; ++i) {
                numerator[i] *= norm;
            }
        }
    }
    denominator[0] = 1.0f;
    float out = 0;
    if (numerator_size == denominator_size) {
        for (size_t n = 0; n < sampleCount; ++n) {
            out = numerator[0] * input[n] + state[0];
            updateStateLine(state, wantedSize, denominator, numerator, input + n, &out);
            output[n] = out;
        }
    } else if (numerator_size > denominator_size) {
        for (size_t n = 0; n < sampleCount; ++n) {
            out = numerator[0] * input[n] + state[0];
            updateStateLine(state, denominator_size, denominator, numerator, input + n, &out);
            for (size_t k = denominator_size; k < wantedSize; ++k) {
                state[k - 1] = numerator[k] * input[n] + state[k];
                renormalize(state + (k - 1));
            }
            output[n] = out;
        }
    } else {  //if (numerator_size < denominator_size) {
        for (size_t n = 0; n < sampleCount; ++n) {
            out = numerator[0] * input[n] + state[0];
            updateStateLine(state, numerator_size, denominator, numerator, input + n, &out);
            for (size_t k = numerator_size; k < wantedSize; ++k) {
                state[k - 1] = (-denominator[k] * out) + state[k];
                renormalize(state + (k - 1));
            }
            output[n] = out;
        }
    }
    free(state);
}

void EqualLoudness(float *input, float *output, size_t size, size_t fs) {
    if ((fs != 44100) && (fs != 48000) && (fs != 47250) && (fs != 32000)
        && (fs != 8000) && (fs != 16000) && (fs != 22050) && (fs != 11025)) {
        return;
    }
    float By[11] = {0.0};
    float Ay[11] = {0.0};
    float Bb[3] = {0.0};
    float Ab[3] = {0.0};
    if (fs == 44100) {
        // Yulewalk filter coefficients:
        By[0] = 0.05418656406430f;
        By[1] = -0.02911007808948f;
        By[2] = -0.00848709379851f;
        By[3] = -0.00851165645469f;
        By[4] = -0.00834990904936f;
        By[5] = 0.02245293253339f;
        By[6] = -0.02596338512915f;
        By[7] = 0.01624864962975f;
        By[8] = -0.00240879051584f;
        By[9] = 0.00674613682247f;
        By[10] = -0.00187763777362f;

        Ay[0] = 1.00000000000000f;
        Ay[1] = -3.47845948550071f;
        Ay[2] = 6.36317777566148f;
        Ay[3] = -8.54751527471874f;
        Ay[4] = 9.47693607801280f;
        Ay[5] = -8.81498681370155f;
        Ay[6] = 6.85401540936998f;
        Ay[7] = -4.39470996079559f;
        Ay[8] = 2.19611684890774f;
        Ay[9] = -0.75104302451432f;
        Ay[10] = 0.13149317958808f;

        // Butterworth filter coefficients:
        Bb[0] = 0.98500175787242f;
        Bb[1] = -1.97000351574484f;
        Bb[2] = 0.98500175787242f;

        Ab[0] = 1.00000000000000f;
        Ab[1] = -1.96977855582618f;
        Ab[2] = 0.97022847566350f;
    } else if (fs == 48000) {
        // Yulewalk filter coefficients:
        By[0] = 0.03857599435200f;
        By[1] = -0.02160367184185f;
        By[2] = -0.00123395316851f;
        By[3] = -0.00009291677959f;
        By[4] = -0.01655260341619f;
        By[5] = 0.02161526843274f;
        By[6] = -0.02074045215285f;
        By[7] = 0.00594298065125f;
        By[8] = 0.00306428023191f;
        By[9] = 0.00012025322027f;
        By[10] = 0.00288463683916f;

        Ay[0] = 1.00000000000000f;
        Ay[1] = -3.84664617118067f;
        Ay[2] = 7.81501653005538f;
        Ay[3] = -11.34170355132042f;
        Ay[4] = 13.05504219327545f;
        Ay[5] = -12.28759895145294f;
        Ay[6] = 9.48293806319790f;
        Ay[7] = -5.87257861775999f;
        Ay[8] = 2.75465861874613f;
        Ay[9] = -0.86984376593551f;
        Ay[10] = 0.13919314567432f;

        // Butterworth filter coefficients:
        Bb[0] = 0.98621192462708f;
        Bb[1] = -1.97242384925416f;
        Bb[2] = 0.98621192462708f;

        Ab[0] = 1.00000000000000f;
        Ab[1] = -1.97223372919527f;
        Ab[2] = 0.97261396931306f;
    } else if (fs == 47250) {
        // Yulewalk filter coefficients:
        By[0] = 0.04075143430640f;
        By[1] = -0.02061396627027f;
        By[2] = -0.00228770609680f;
        By[3] = -0.00238140312565f;
        By[4] = -0.01537780619267f;
        By[5] = 0.02133335742835f;
        By[6] = -0.02114825824762f;
        By[7] = 0.00708183842481f;
        By[8] = 0.00212853002444f;
        By[9] = 0.00152799938087f;
        By[10] = 0.00221228083567f;

        Ay[0] = 1.00000000000000f;
        Ay[1] = -3.74644140061599f;
        Ay[2] = 7.45035857658709f;
        Ay[3] = -10.67893227587523f;
        Ay[4] = 12.23998062786813f;
        Ay[5] = -11.52095216425771f;
        Ay[6] = 8.91550816515888f;
        Ay[7] = -5.55055006643942f;
        Ay[8] = 2.62104157502486f;
        Ay[9] = -0.83247558569372f;
        Ay[10] = 0.13364127665477f;

        // Butterworth filter coefficients:
        Bb[0] = 0.98599460581381f;
        Bb[1] = -1.97198921162762f;
        Bb[2] = 0.98599460581381f;

        Ab[0] = 1.00000000000000f;
        Ab[1] = -1.97179305094156f;
        Ab[2] = 0.97218537231369f;
    } else if (fs == 32000) {
        // Yulewalk filter coefficients:
        By[0] = 0.15457299681924f;
        By[1] = -0.09331049056315f;
        By[2] = -0.06247880153653f;
        By[3] = 0.02163541888798f;
        By[4] = -0.05588393329856f;
        By[5] = 0.04781476674921f;
        By[6] = 0.00222312597743f;
        By[7] = 0.03174092540049f;
        By[8] = -0.01390589421898f;
        By[9] = 0.00651420667831f;
        By[10] = -0.00881362733839f;

        Ay[0] = 1.00000000000000f;
        Ay[1] = -2.37898834973084f;
        Ay[2] = 2.84868151156327f;
        Ay[3] = -2.64577170229825f;
        Ay[4] = 2.23697657451713f;
        Ay[5] = -1.67148153367602f;
        Ay[6] = 1.00595954808547f;
        Ay[7] = -0.45953458054983f;
        Ay[8] = 0.16378164858596f;
        Ay[9] = -0.05032077717131f;
        Ay[10] = 0.02347897407020f;

        // Butterworth filter coefficients:
        Bb[0] = 0.97938932735214f;
        Bb[1] = -1.95877865470428f;
        Bb[2] = 0.97938932735214f;

        Ab[0] = 1.00000000000000f;
        Ab[1] = -1.95835380975398f;
        Ab[2] = 0.95920349965459f;
    } else if (fs == 22050) {
        // Yulewalk filter coefficients:
        By[0] = 0.33821823115580f;
        By[1] = -0.26807847188177f;
        By[2] = -0.09661820705457f;
        By[3] = 0.10379911044027f;
        By[4] = -0.07497750759619f;
        By[5] = -0.00141546938980f;
        By[6] = -0.01014630122213f;
        By[7] = 0.05970529709297f;
        By[8] = 0.00658436950009f;
        By[9] = -0.01493027897727f;
        By[10] = -0.01818092859084f;

        Ay[0] = 1.00000000000000f;
        Ay[1] = -1.52049414776012f;
        Ay[2] = 0.93688375506972f;
        Ay[3] = 0.04029217863890f;
        Ay[4] = -0.75260565996584f;
        Ay[5] = 0.47653608952590f;
        Ay[6] = -0.16105897895529f;
        Ay[7] = 0.00024249729244f;
        Ay[8] = 0.05754865070456f;
        Ay[9] = -0.03254539495824f;
        Ay[10] = 0.02762193632283f;
        // Butterworth filter coefficients:
        Bb[0] = 0.97022837669833f;
        Bb[1] = -1.94045675339666f;
        Bb[2] = 0.97022837669833f;

        Ab[0] = 1.00000000000000f;
        Ab[1] = -1.93957020735167f;
        Ab[2] = 0.94134329944165f;
    } else if (fs == 16000) {
        // Yulewalk filter coefficients:
        By[0] = 0.44915256608450f;
        By[1] = -0.14351757464547f;
        By[2] = -0.22784394429749f;
        By[3] = -0.01419140100551f;
        By[4] = 0.04078262797139f;
        By[5] = -0.12398163381748f;
        By[6] = 0.04097565135648f;
        By[7] = 0.10478503600251f;
        By[8] = -0.01863887810927f;
        By[9] = -0.03193428438915f;
        By[10] = 0.00541907748707f;

        Ay[0] = 1.00000000000000f;
        Ay[1] = -0.62820619233671f;
        Ay[2] = 0.29661783706366f;
        Ay[3] = -0.37256372942400f;
        Ay[4] = 0.00213767857124f;
        Ay[5] = -0.42029820170917f;
        Ay[6] = 0.22199650564824f;
        Ay[7] = 0.00613424350682f;
        Ay[8] = 0.06747620744683f;
        Ay[9] = 0.05784820375801f;
        Ay[10] = 0.03222754072173f;
        // Butterworth filter coefficients:
        Bb[0] = 0.95920314963834f;
        Bb[1] = -1.91840629927668f;
        Bb[2] = 0.95920314963834f;

        Ab[0] = 1.00000000000000f;
        Ab[1] = -1.91674122315762f;
        Ab[2] = 0.92007137539573f;
    } else if (fs == 11025) {
        // Yulewalk filter coefficients:
        By[0] = 0.57983338918994f;
        By[1] = -0.60372070665856f;
        By[2] = -0.02708386977708f;
        By[3] = 0.12153188507772f;
        By[4] = 0.01968162528366f;
        By[5] = 0.15845839635954f;
        By[6] = -0.27059509296227f;
        By[7] = 0.06047050641721f;
        By[8] = 0.04476881112971f;
        By[9] = -0.03608428784987f;
        By[10] = -0.00382749608177f;

        Ay[0] = 1.00000000000000f;
        Ay[1] = -0.64025329972521f;
        Ay[2] = -0.16397965568041f;
        Ay[3] = -0.25691230129275f;
        Ay[4] = 0.18528659764006f;
        Ay[5] = 0.33986554016836f;
        Ay[6] = -0.24963773504109f;
        Ay[7] = -0.00317434205112f;
        Ay[8] = -0.03528170263515f;
        Ay[9] = 0.02971468398012f;
        Ay[10] = 0.00880007392185f;
        // Butterworth filter coefficients:
        Bb[0] = 0.94134179599236f;
        Bb[1] = -1.88268359198471f;
        Bb[2] = 0.94134179599236f;

        Ab[0] = 1.00000000000000f;
        Ab[1] = -1.87923984223423f;
        Ab[2] = 0.88612734173520f;
    } else  // (fs == 8000)
    {
        // Yulewalk filter coefficients:
        By[0] = 0.536487892551045f;
        By[1] = -0.421630343506963f;
        By[2] = -0.002759536119290f;
        By[3] = 0.042678422194153f;
        By[4] = -0.102148641796756f;
        By[5] = 0.145907722893880f;
        By[6] = -0.024598648593454f;
        By[7] = -0.112023151953880f;
        By[8] = -0.040600341270002f;
        By[9] = 0.047886655481804f;
        By[10] = -0.02217936801134f;

        Ay[0] = 1.f;
        Ay[1] = -0.250498719560207f;
        Ay[2] = -0.431939423111139f;
        Ay[3] = -0.034246810176745f;
        Ay[4] = -0.046783287842416f;
        Ay[5] = 0.264083002009554f;
        Ay[6] = 0.151131305332161f;
        Ay[7] = -0.175564933664496f;
        Ay[8] = -0.188230092621155f;
        Ay[9] = 0.054777204286738f;
        Ay[10] = 0.04704409688120f;

        // Butterworth filter coefficients:
        Bb[0] = 0.92006615842917f;
        Bb[1] = -1.84013231685834f;
        Bb[2] = 0.92006615842917f;

        Ab[0] = 1.f;
        Ab[1] = -1.83373265892465f;
        Ab[2] = 0.84653197479202f;
    }
    //yulewalkFilter
    IIR_filter(input, output, size, Ay, 11, By, 11);
    //butterworthFilter
    IIR_filter(output, output, size, Ab, 3, Bb, 3);
}

void EqualLoudnessProcess(char *in_file, char *out_file) {
    uint32_t sampleRate = 0;
    uint64_t sampleCount = 0;
    uint32_t channels = 0;
    float *input = wavRead_f32(in_file, &sampleRate, &sampleCount, &channels);
    if (input) {
        float *output = (float *) malloc(sampleCount * sizeof(float));
        if (output) {
            double startTime = now();
            EqualLoudness(input, output, sampleCount, sampleRate);
            double time_interval = calcElapsed(startTime, now());
            printf("time interval: %f ms\n ", (time_interval * 1000));
            wavWrite_f32(out_file, output, sampleRate, (uint32_t) sampleCount, channels);
            free(output);
        }
        free(input);
    }
}


int main(int argc, char *argv[]) {
    printf("Audio Processing\n");
    printf("blog:http://cpuimage.cnblogs.com/\n");
    printf("Audio Equal Loudness\n");
    if (argc < 2) {
        printUsage();
        return -1;
    }
    char *in_file = argv[1];
    if (argc > 2) {
        char *out_file = argv[2];
        EqualLoudnessProcess(in_file, out_file);
    } else {
        char drive[3];
        char dir[256];
        char fname[256];
        char ext[256];
        char out_file[1024];
        splitpath(in_file, drive, dir, fname, ext);
        sprintf(out_file, "%s%s%s_out.wav", drive, dir, fname);
        EqualLoudnessProcess(in_file, out_file);
    }

    return 0;
}

#ifdef __cplusplus
}
#endif