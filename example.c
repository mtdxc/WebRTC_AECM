#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "vad.h"
#include "agc.h"
#include "aecm.h"
#include "noise_suppression.h"

#ifndef nullptr
#define nullptr 0
#endif

#ifndef MIN
#define  MIN(A, B)        ((A) < (B) ? (A) : (B))
#endif

#ifndef MAX
#define  MAX(A, B)        ((A) > (B) ? (A) : (B))
#endif

// https://www.cnblogs.com/cpuimage/p/8976346.html
static float fast_sqrt(float x) {
    float s;
#if defined(__x86_64__)
    __asm__ __volatile__ ("sqrtss %1, %0" : "=x"(s) : "x"(x));
#elif defined(__i386__)
    s = x;
    __asm__ __volatile__ ("fsqrt" : "+t"(s));
#elif defined(__arm__) && defined(__VFP_FP__)
    __asm__ __volatile__ ("vsqrt.f32 %0, %1" : "=w"(s) : "w"(x));
#else
    s = sqrtf(x);
#endif
    return s;
}

// A Simple and Efficient Audio Resampler Implementation in C
// 简洁的重采样算法，用在对采样音质要求不高的情况下，也是够用了
uint64_t Resample_f32(const float *input, float *output, int inSampleRate, int outSampleRate, 
    uint64_t inputSize, uint32_t channels) {
    if (input == NULL)
        return 0;
    uint64_t outputSize = inputSize * outSampleRate / inSampleRate;
    if (output == NULL)
        return outputSize;
    double stepDist = ((double) inSampleRate / (double) outSampleRate);
    const uint64_t fixedFraction = (1LL << 32);
    const double normFixed = (1.0 / (1LL << 32));
    uint64_t step = ((uint64_t) (stepDist * fixedFraction + 0.5));
    uint64_t curOffset = 0;
    for (uint32_t i = 0; i < outputSize; i += 1) {
        for (uint32_t c = 0; c < channels; c += 1) {
            *output++ = (float) (input[c] + (input[c + channels] - input[c]) * (
                    (double) (curOffset >> 32) + ((curOffset & (fixedFraction - 1)) * normFixed)
            )
            );
        }
        curOffset += step;
        input += (curOffset >> 32) * channels;
        curOffset &= (fixedFraction - 1);
    }
    return outputSize;
}

uint64_t Resample_s16(const int16_t *input, int16_t *output, int inSampleRate, int outSampleRate, 
    uint64_t inputSize, uint32_t channels) {
    if (input == NULL)
        return 0;
    uint64_t outputSize = inputSize * outSampleRate / inSampleRate;
    if (output == NULL)
        return outputSize;
    double stepDist = ((double) inSampleRate / (double) outSampleRate);
    const uint64_t fixedFraction = (1LL << 32);
    const double normFixed = (1.0 / (1LL << 32));
    uint64_t step = ((uint64_t) (stepDist * fixedFraction + 0.5));
    uint64_t curOffset = 0;
    for (uint32_t i = 0; i < outputSize; i += 1) {
        for (uint32_t c = 0; c < channels; c += 1) {
            *output++ = (int16_t) (input[c] + (input[c + channels] - input[c]) * (
                    (double) (curOffset >> 32) + ((curOffset & (fixedFraction - 1)) * normFixed)
            )
            );
        }
        curOffset += step;
        input += (curOffset >> 32) * channels;
        curOffset &= (fixedFraction - 1);
    }
    return outputSize;
}

int agcProcess(int16_t *buffer, uint32_t sampleRate, size_t samplesCount, int16_t agcMode) {
    if (buffer == nullptr) return -1;
    if (samplesCount == 0) return -1;
    WebRtcAgcConfig agcConfig;
    agcConfig.compressionGaindB = 9; // default 9 dB
    agcConfig.limiterEnable = 1; // default kAgcTrue (on)
    agcConfig.targetLevelDbfs = 3; // default 3 (-3 dBOv)
    int minLevel = 0;
    int maxLevel = 255;
    size_t samples = MIN(160, sampleRate / 100);
    if (samples == 0) return -1;
    const int maxSamples = 320;
    int16_t *input = buffer;
    size_t nTotal = (samplesCount / samples);
    void *agcInst = WebRtcAgc_Create();
    if (agcInst == NULL) return -1;
    int status = WebRtcAgc_Init(agcInst, minLevel, maxLevel, agcMode, sampleRate);
    if (status != 0) {
        printf("WebRtcAgc_Init fail\n");
        WebRtcAgc_Free(agcInst);
        return -1;
    }
    status = WebRtcAgc_set_config(agcInst, agcConfig);
    if (status != 0) {
        printf("WebRtcAgc_set_config fail\n");
        WebRtcAgc_Free(agcInst);
        return -1;
    }
    size_t num_bands = 1;
    int inMicLevel, outMicLevel = -1;
    int16_t out_buffer[maxSamples];
    int16_t *out16 = out_buffer;
    uint8_t saturationWarning = 1;                 //是否有溢出发生，增益放大以后的最大值超过了65536
    int16_t echo = 0;                                 //增益放大是否考虑回声影响
    for (int i = 0; i < nTotal; i++) {
        inMicLevel = 0;
        int nAgcRet = WebRtcAgc_Process(agcInst, (const int16_t *const *) &input, num_bands, samples,
                                        (int16_t *const *) &out16, inMicLevel, &outMicLevel, echo,
                                        &saturationWarning);

        if (nAgcRet != 0) {
            printf("failed in WebRtcAgc_Process\n");
            WebRtcAgc_Free(agcInst);
            return -1;
        }
        memcpy(input, out_buffer, samples * sizeof(int16_t));
        input += samples;
    }

    const size_t remainedSamples = samplesCount - nTotal * samples;
    if (remainedSamples > 0) {
        if (nTotal > 0) {
            input = input - samples + remainedSamples;
        }

        inMicLevel = 0;
        int nAgcRet = WebRtcAgc_Process(agcInst, (const int16_t *const *) &input, num_bands, samples,
                                        (int16_t *const *) &out16, inMicLevel, &outMicLevel, echo,
                                        &saturationWarning);

        if (nAgcRet != 0) {
            printf("failed in WebRtcAgc_Process during filtering the last chunk\n");
            WebRtcAgc_Free(agcInst);
            return -1;
        }
        memcpy(&input[samples-remainedSamples], &out_buffer[samples-remainedSamples], remainedSamples * sizeof(int16_t));
        input += samples;
    }

    WebRtcAgc_Free(agcInst);
    return 1;
}

enum nsLevel {
    kLow,
    kModerate,
    kHigh,
    kVeryHigh
};


int nsProcess(int16_t *buffer, uint32_t sampleRate, uint64_t samplesCount, uint32_t channels, enum nsLevel level) {
    if (buffer == nullptr) return -1;
    if (samplesCount == 0) return -1;
    size_t samples = MIN(160, sampleRate / 100);
    if (samples == 0) return -1;
    uint32_t num_bands = 1;
    int16_t *input = buffer;
    size_t frames = (samplesCount / (samples * channels));
    int16_t *frameBuffer = (int16_t *) malloc(sizeof(*frameBuffer) * channels * samples);
    NsHandle **NsHandles = (NsHandle **) malloc(channels * sizeof(NsHandle *));
    if (NsHandles == NULL || frameBuffer == NULL) {
        if (NsHandles)
            free(NsHandles);
        if (frameBuffer)
            free(frameBuffer);
        fprintf(stderr, "malloc error.\n");
        return -1;
    }
    for (int i = 0; i < channels; i++) {
        NsHandles[i] = WebRtcNs_Create();
        if (NsHandles[i] != NULL) {
            int status = WebRtcNs_Init(NsHandles[i], sampleRate);
            if (status != 0) {
                fprintf(stderr, "WebRtcNs_Init fail\n");
                WebRtcNs_Free(NsHandles[i]);
                NsHandles[i] = NULL;
            } else {
                status = WebRtcNs_set_policy(NsHandles[i], level);
                if (status != 0) {
                    fprintf(stderr, "WebRtcNs_set_policy fail\n");
                    WebRtcNs_Free(NsHandles[i]);
                    NsHandles[i] = NULL;
                }
            }
        }
        if (NsHandles[i] == NULL) {
            for (int x = 0; x < i; x++) {
                if (NsHandles[x]) {
                    WebRtcNs_Free(NsHandles[x]);
                }
            }
            free(NsHandles);
            free(frameBuffer);
            return -1;
        }
    }
    for (int i = 0; i < frames; i++) {
        for (int c = 0; c < channels; c++) {
            for (int k = 0; k < samples; k++)
                frameBuffer[k] = input[k * channels + c];

            int16_t *nsIn[1] = {frameBuffer};   //ns input[band][data]
            int16_t *nsOut[1] = {frameBuffer};  //ns output[band][data]
            WebRtcNs_Analyze(NsHandles[c], nsIn[0]);
            WebRtcNs_Process(NsHandles[c], (const int16_t *const *) nsIn, num_bands, nsOut);
            for (int k = 0; k < samples; k++)
                input[k * channels + c] = frameBuffer[k];
        }
        input += samples * channels;
    }

    for (int i = 0; i < channels; i++) {
        if (NsHandles[i]) {
            WebRtcNs_Free(NsHandles[i]);
        }
    }
    free(NsHandles);
    free(frameBuffer);
    return 1;
}


int vadProcess(int16_t *buffer, uint32_t sampleRate, size_t samplesCount, int16_t vad_mode, int per_ms_frames) {
    if (buffer == nullptr) return -1;
    if (samplesCount == 0) return -1;
    // kValidRates : 8000, 16000, 32000, 48000
    // 10, 20 or 30 ms frames
    per_ms_frames = MAX(MIN(30, per_ms_frames), 10);
    size_t samples = sampleRate * per_ms_frames / 1000;
    if (samples == 0) return -1;
    int16_t *input = buffer;
    size_t nTotal = (samplesCount / samples);

    void *vadInst = WebRtcVad_Create();
    if (vadInst == NULL) return -1;
    int status = WebRtcVad_Init(vadInst);
    if (status != 0) {
        printf("WebRtcVad_Init fail\n");
        WebRtcVad_Free(vadInst);
        return -1;
    }
    status = WebRtcVad_set_mode(vadInst, vad_mode);
    if (status != 0) {
        printf("WebRtcVad_set_mode fail\n");
        WebRtcVad_Free(vadInst);
        return -1;
    }
    printf("Activity ： \n");
    for (int i = 0; i < nTotal; i++) {
        int keep_weight = 0;
        int nVadRet = WebRtcVad_Process(vadInst, sampleRate, input, samples, keep_weight);
        if (nVadRet == -1) {
            printf("failed in WebRtcVad_Process\n");
            WebRtcVad_Free(vadInst);
            return -1;
        } else {
            // output result
            printf(" %d \t", nVadRet);
        }
        input += samples;
    }
    printf("\n");
    WebRtcVad_Free(vadInst);
    return 1;
}