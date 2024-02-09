// Display the audio frequency spectrum gathered from the ADC on a
// RP2040 board.

// Uses the ApproxFFT functions from here(see sketch tab):
// https://www.instructables.com/ApproxFFT-Fastest-FFT-Function-for-Arduino/

// ADC sampling DMA code from here:
// https://github.com/raspberrypi/pico-examples/tree/master/adc/dma_capture

// #include <TFT_eSPI.h>                 // Include the graphics library (this includes the sprite functions)

#include "hardware/adc.h"
#include "hardware/dma.h"

#include <Adafruit_IS31FL3741.h>

TwoWire *i2c = &Wire;

Adafruit_IS31FL3741_QT_buffered matrix;
Adafruit_IS31FL3741_QT_buffered matrix2;

// Swtich settings
int sample_range = 0;
#define NARROW 0  // 1k
#define MID 1 // 3k
#define LONG 2 // 10k
#define CLOSEST 3 // 500

// Ideally the signal should be bandwidth limited to sample_frequency/2
// Sampling frequency in Hz 14000 default. 44000 max. 40000 max range. 25000 - 22000 for good audible range. 12000 best for narrow 512 samples
// #define SAMPLING_FREQUENCY 18000 // 22000  // 16000 // 24000  

// ADC channel 0
#define CAPTURE_CHANNEL 0
#define SAMPLES 512
#define TOLERANCE 0
#define MAX_AMP 10000 // Not used but ideally it's 10000

int16_t sampleBuffer[SAMPLES];  // ADC sample DMA buffer
int16_t streamBuffer[SAMPLES];  // Scaled ADC sample working buffer
int approxBuffer[SAMPLES];      // ApproxFFT sample buffer
int peakBuffer[SAMPLES];        // Amplitude peak buffer


uint16_t counter = 0;         // Frame counter
long startMillis = millis();  // Timer for FPS
uint16_t interval = 100;      // FPS calculated over 100 frames
String fps = "0 fps";

int mic;
int maximum = 0;
int amp_pot = 0;
int max_amplitude = 10000;
int sampling_frequency = 18000;
int offset = 0;
int micsamples[26];
int peaks[26];
int color_count = 0;
bool color_direction = 0;
int peak_count = 0;
int divider = 2;
int bias = 0;
int slice_mid = 184;
int sliceA = 8;
int sliceB = 18;

int heatmap[26] = {
  0x99d9cf, 0x99d2cb, 0x99cbc8, 0x98c4c4, 0x98bdc0,
  0x98b6bd, 0x98afb9, 0x98a7b6, 0x97a0b2, 0x9799ae,
  0x9792ab, 0x978ba7, 0x9784a3, 0x967da0, 0x96769c,
  0x966f98, 0x966895, 0x966191, 0x955a8d, 0x95528a,
  0x954b86, 0x954483, 0x953d7f, 0x94367b, 0x942f78,
  0x942874
};
int peakColor[9] = {0x00FF0000, 0x00004400, 0x00FF0000, 0x00004400, 0x00000088, 0x00FF0000, 0x00004400, 0x00FF0000, 0x00004400};
int peakSelected[26];
int peakState[26];

int peakRange = matrix.color565(0x11AA11);
// Sprite width and height
// The width should be an integer multiple of samples/2 where that integer is a minimum of 1.
#define WIDTH 26  // 256
#define HEIGHT 9  //160

dma_channel_config cfg;
int dma_chan;


//-----------------------------FFT Function----------------------------------------------//
/*
  Code to perform High speed and Accurate FFT on arduino,
  setup:

  1. in[]     : Data array,
  2. N        : Number of sample (recommended sample size 2,4,8,16,32,64,128,256,512...)
  3. Frequency: sampling frequency required as input (Hz)

  It will by default return frequency with max aplitude,
  if you need complex output or magnitudes uncomment required sections

  If sample size is not in power of 2 it will be clipped to lower side of number.
  i.e, for 150 number of samples, code will consider first 128 sample, remaining sample  will be omitted.
  For Arduino nano, FFT of more than 256 sample not possible due to mamory limitation
  Code by ABHILASH
  Contact: abhilashpatel121@gmail.com
  Documentation & details: https://www.instructables.com/member/abhilash_patel/instructables/

  Update(06/05/21): Correction made for support on Arduino Due
*/

//---------------------------------lookup data------------------------------------//
byte isin_data[128] = { 0, 1, 3, 4, 5, 6, 8, 9, 10, 11, 13, 14, 15, 17, 18, 19, 20,
                        22, 23, 24, 26, 27, 28, 29, 31, 32, 33, 35, 36, 37, 39, 40, 41, 42,
                        44, 45, 46, 48, 49, 50, 52, 53, 54, 56, 57, 59, 60, 61, 63, 64, 65,
                        67, 68, 70, 71, 72, 74, 75, 77, 78, 80, 81, 82, 84, 85, 87, 88, 90,
                        91, 93, 94, 96, 97, 99, 100, 102, 104, 105, 107, 108, 110, 112, 113, 115, 117,
                        118, 120, 122, 124, 125, 127, 129, 131, 133, 134, 136, 138, 140, 142, 144, 146, 148,
                        150, 152, 155, 157, 159, 161, 164, 166, 169, 171, 174, 176, 179, 182, 185, 188, 191,
                        195, 198, 202, 206, 210, 215, 221, 227, 236 };
unsigned int Pow2[14] = { 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096 };
byte RSSdata[20] = { 7, 6, 6, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2 };
//---------------------------------------------------------------------------------//

float Approx_FFT(int in[], unsigned int N, float Frequency) {
  int a, c1, f, o = 0, x, data_max, data_min = 0;
  long data_avg, data_mag, temp11;
  byte scale, check = 0;

  data_max = 0;
  data_avg = 0;
  data_min = 0;

  for (int i = 0; i < 12; i++)  //calculating the levels
  {
    if (Pow2[i] <= N) {
      o = i;
    }
  }
  a = Pow2[o];
  int out_r[a];   //real part of transform
  int out_im[a];  //imaginory part of transform

  for (int i = 0; i < a; i++)  //getting min max and average for scalling
  {
    out_r[i] = 0;
    out_im[i] = 0;
    data_avg = data_avg + in[i];
    if (in[i] > data_max) {
      data_max = in[i];
    }
    if (in[i] < data_min) {
      data_min = in[i];
    }
  }

  data_avg = data_avg >> o;
  scale = 0;
  data_mag = data_max - data_min;
  temp11 = data_mag;

  //scalling data  from +512 to -512

  if (data_mag > 1024) {
    while (temp11 > 1024) {
      temp11 = temp11 >> 1;
      scale = scale + 1;
    }
  }

  if (data_mag < 1024) {
    while (temp11 < 1024) {
      temp11 = temp11 << 1;
      scale = scale + 1;
    }
  }


  if (data_mag > 1024) {
    for (int i = 0; i < a; i++) {
      in[i] = in[i] - data_avg;
      in[i] = in[i] >> scale;
    }
    scale = 128 - scale;
  }

  if (data_mag < 1024) {
    scale = scale - 1;
    for (int i = 0; i < a; i++) {
      in[i] = in[i] - data_avg;
      in[i] = in[i] << scale;
    }

    scale = 128 + scale;
  }


  x = 0;
  for (int b = 0; b < o; b++)  // bit reversal order stored in im_out array
  {
    c1 = Pow2[b];
    f = Pow2[o] / (c1 + c1);
    for (int j = 0; j < c1; j++) {
      x = x + 1;
      out_im[x] = out_im[j] + f;
    }
  }

  for (int i = 0; i < a; i++)  // update input array as per bit reverse order
  {
    out_r[i] = in[out_im[i]];
    out_im[i] = 0;
  }


  int i10, i11, n1, tr, ti;
  float e;
  int c, temp4;
  for (int i = 0; i < o; i++)  //fft
  {
    i10 = Pow2[i];                // overall values of sine/cosine
    i11 = Pow2[o] / Pow2[i + 1];  // loop with similar sine cosine
    e = 1024 / Pow2[i + 1];       //1024 is equivalent to 360 deg
    e = 0 - e;
    n1 = 0;

    for (int j = 0; j < i10; j++) {
      c = e * j;  //c is angle as where 1024 unit is 360 deg
      while (c < 0) {
        c = c + 1024;
      }
      while (c > 1024) {
        c = c - 1024;
      }

      n1 = j;

      for (int k = 0; k < i11; k++) {
        temp4 = i10 + n1;
        if (c == 0) {
          tr = out_r[temp4];
          ti = out_im[temp4];
        } else if (c == 256) {
          tr = -out_im[temp4];
          ti = out_r[temp4];
        } else if (c == 512) {
          tr = -out_r[temp4];
          ti = -out_im[temp4];
        } else if (c == 768) {
          tr = out_im[temp4];
          ti = -out_r[temp4];
        } else if (c == 1024) {
          tr = out_r[temp4];
          ti = out_im[temp4];
        } else {
          tr = fast_cosine(out_r[temp4], c) - fast_sine(out_im[temp4], c);  //the fast sine/cosine function gives direct (approx) output for A*sinx
          ti = fast_sine(out_r[temp4], c) + fast_cosine(out_im[temp4], c);
        }

        out_r[n1 + i10] = out_r[n1] - tr;
        out_r[n1] = out_r[n1] + tr;
        if (out_r[n1] > 15000 || out_r[n1] < -15000) {
          check = 1;  //check for int size, it can handle only +31000 to -31000,
        }

        out_im[n1 + i10] = out_im[n1] - ti;
        out_im[n1] = out_im[n1] + ti;
        if (out_im[n1] > 15000 || out_im[n1] < -15000) {
          check = 1;
        }

        n1 = n1 + i10 + i10;
      }
    }

    if (check == 1) {  // scalling the matrics if value higher than 15000 to prevent varible from overflowing
      for (int i = 0; i < a; i++) {
        out_r[i] = out_r[i] >> 1;
        out_im[i] = out_im[i] >> 1;
      }
      check = 0;
      scale = scale - 1;  // tracking overall scalling of input data
    }
  }


  if (scale > 128) {
    scale = scale - 128;
    for (int i = 0; i < a; i++) {
      out_r[i] = out_r[i] >> scale;
      out_im[i] = out_im[i] >> scale;
    }
    scale = 0;
  }  // revers all scalling we done till here,
  else {
    scale = 128 - scale;  // in case of nnumber getting higher than 32000, we will represent in as multiple of 2^scale
  }


  // for(int i=0;i<a;i++)
  // {
  // Serial.print(out_r[i]);Serial.print("\t");                    // un comment to print RAW o/p
  // Serial.print(out_im[i]);
  // Serial.print("i");Serial.print("\t");
  // Serial.print("*2^");Serial.println(scale);
  // }
  // Serial.println(sizeof(out_im)/sizeof(*out_im));
  // Serial.println("=======");
  // delay(10000);


  //---> here onward out_r contains amplitude and our_in conntains frequency (Hz)
  int fout, fm, fstp;
  float fstep;
  fstep = Frequency / N;
  fstp = fstep;
  fout = 0;
  fm = 0;

  for (unsigned int i = 1; i < Pow2[o - 1]; i++)  // getting amplitude from compex number
  {
    out_r[i] = fastRSS(out_r[i], out_im[i]);
    // Approx RSS function used to calculated magnitude quickly

    out_im[i] = out_im[i] + fstp;
    if (fout < out_r[i]) {
      fm = i;
      fout = out_r[i];
    }

    // un comment to print Amplitudes (1st value (offset) is not printed)
    // Serial.print(out_r[i]); Serial.print("\t");
    // Serial.print("*2^");Serial.println(scale);

    in[i - 1] = out_r[i];
  }
  // Serial.println(sizeof(out_r)/sizeof(*out_r));
  // Serial.println("------AMPS--------");
  // delay(10000);


  float fa, fb, fc;
  fa = out_r[fm - 1];
  fb = out_r[fm];
  fc = out_r[fm + 1];
  fstep = (fa * (fm - 1) + fb * fm + fc * (fm + 1)) / (fa + fb + fc);

  return (fstep * Frequency / N);
}

//---------------------------------fast sine/cosine---------------------------------------//

int fast_sine(int Amp, int th) {
  int temp3, m1, m2;
  byte temp1, temp2, test, quad, accuracy;
  accuracy = 5;  // set it value from 1 to 7, where 7 being most accurate but slowest
  // accuracy value of 5 recommended for typical applicaiton
  while (th > 1024) {
    th = th - 1024;  // here 1024 = 2*pi or 360 deg
  }
  while (th < 0) {
    th = th + 1024;
  }
  quad = th >> 8;

  if (quad == 1) {
    th = 512 - th;
  } else if (quad == 2) {
    th = th - 512;
  } else if (quad == 3) {
    th = 1024 - th;
  }

  temp1 = 0;
  temp2 = 128;  //2 multiple
  m1 = 0;
  m2 = Amp;

  temp3 = (m1 + m2) >> 1;
  Amp = temp3;
  for (int i = 0; i < accuracy; i++) {
    test = (temp1 + temp2) >> 1;
    temp3 = temp3 >> 1;
    if (th > isin_data[test]) {
      temp1 = test;
      Amp = Amp + temp3;
      m1 = Amp;
    } else if (th < isin_data[test]) {
      temp2 = test;
      Amp = Amp - temp3;
      m2 = Amp;
    }
  }

  if (quad == 2) {
    Amp = 0 - Amp;
  } else if (quad == 3) {
    Amp = 0 - Amp;
  }
  return (Amp);
}

int fast_cosine(int Amp, int th) {
  th = 256 - th;  //cos th = sin (90-th) formula
  return (fast_sine(Amp, th));
}

//--------------------------------Fast RSS----------------------------------------//
int fastRSS(int a, int b) {
  if (a == 0 && b == 0) {
    return (0);
  }
  int min, max, temp1, temp2;
  byte clevel;
  if (a < 0) {
    a = -a;
  }
  if (b < 0) {
    b = -b;
  }
  clevel = 0;
  if (a > b) {
    max = a;
    min = b;
  } else {
    max = b;
    min = a;
  }

  if (max > (min + min + min)) {
    return max;
  } else {
    temp1 = min >> 3;
    if (temp1 == 0) {
      temp1 = 1;
    }
    temp2 = min;
    while (temp2 < max) {
      temp2 = temp2 + temp1;
      clevel = clevel + 1;
    }
    temp2 = RSSdata[clevel];
    temp1 = temp1 >> 1;
    for (int i = 0; i < temp2; i++) {
      max = max + temp1;
    }
    return (max);
  }
}
//--------------------------------------------------------------------------------//

void color_next() {
  int color_bin[26];
  color_bin[0] = heatmap[25];

  for (int i; i < 25; i++) {
    color_bin[i+1] = heatmap[i];
  }

  for (int i; i < 26; i++) {
    heatmap[i] = color_bin[i];
  }
}

void color_reverse() {
  int color_bin[26];
  color_bin[25] = heatmap[0];

  for (int i; i < 25; i++) {
    color_bin[i] = heatmap[i+1];
  }

  for (int i; i < 26; i++) {
    heatmap[i] = color_bin[i];
  }
}


void setup() {
  // Serial.begin();

  // Initialize switches
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);

  // Set sampling frequency
  if (!digitalRead(3) && digitalRead(4)) {
    sampling_frequency = 18000;
  }
  else if (!digitalRead(4) && digitalRead(3)) {
    sampling_frequency = 22000;
  }
  else if (!digitalRead(3) && !digitalRead(4)) {
    sampling_frequency = 26000;
  }
  else {
    sampling_frequency = 14000;
  }

  // Set max amplitude. Can only be set here before ADC is DMA-ed.
  // Reset must be done after changing value.
  amp_pot = analogRead(27);
  max_amplitude = map(amp_pot, 0, 1024, 500, 19500);
  // if (amp_pot > 490 && amp_pot < 525) {
  //   max_amplitude = 10000;
  // }
  // else {
  //   max_amplitude = map(amp_pot, 0, 1024, 500, 19500);
  // }

  if (max_amplitude < 9000) {
    bias = 2;
  }
  

  // Serial.println(max_amplitude);
  // Initialize the Matrix library
  if (!matrix.begin(0x31, i2c)) {
    // Serial.println("IS41 not found");
    while (1)
      ;
  }

  if (!matrix2.begin(IS3741_ADDR_DEFAULT, i2c)) {
    // Serial.println("IS41 not found");
    while (1)
      ;
  }

  i2c->setClock(1000000);

  int LEDscaling = 10;
  int GlobalCurrent = 10;
  matrix.setLEDscaling(LEDscaling);
  matrix.setGlobalCurrent(GlobalCurrent);
  matrix.enable(true);  // bring out of shutdown
  matrix.setRotation(0);

  matrix2.setLEDscaling(LEDscaling);
  matrix2.setGlobalCurrent(GlobalCurrent);
  matrix2.enable(true);  // bring out of shutdown
  matrix2.setRotation(0);

  
  // Init GPIO for analogue use: hi-Z, no pulls, disable digital input buffer.
  adc_gpio_init(26 + CAPTURE_CHANNEL);

  adc_init();
  adc_select_input(CAPTURE_CHANNEL);
  adc_fifo_setup(
    true,   // Write each completed conversion to the sample FIFO
    true,   // Enable DMA data request (DREQ)
    1,      // DREQ (and IRQ) asserted when at least 1 sample present
    false,  // We won't see the ERR bit because of 8 bit reads; disable.
    false   // Shift each sample to 8 bits when pushing to FIFO
  );

  // Divisor of 0 -> 500kHz. Free-running capture with the divider is
  // equivalent to pressing the ADC_CS_START_ONCE button once per `div + 1`
  // cycles (div not necessarily an integer). Each conversion takes 96
  // cycles, so in general you want a divider of 0 (hold down the button
  // continuously) or > 95 (take samples less frequently than 96 cycle
  // intervals). This is all timed by the 48 MHz ADC clock.
  adc_set_clkdiv((48000000 / sampling_frequency) - 1);

  // Set up the DMA to start transferring data as soon as it appears in FIFO
  dma_chan = dma_claim_unused_channel(true);
  cfg = dma_channel_get_default_config(dma_chan);

  // Reading from constant address, writing to incrementing byte addresses
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
  channel_config_set_read_increment(&cfg, false);
  channel_config_set_write_increment(&cfg, true);

  // Pace transfers based on availability of ADC samples
  channel_config_set_dreq(&cfg, DREQ_ADC);

  dma_channel_configure(dma_chan, &cfg,
                        (uint16_t *)sampleBuffer,  // dst
                        &adc_hw->fifo,             // src
                        SAMPLES,                   // transfer count
                        true                       // start immediately
  );
  // Start capture
  adc_run(true);
  
  // Reset FPS timer
  // startMillis = millis();

  
}

void loop() {
  // if (!digitalRead(3) && digitalRead(4)) {
  //   sample_range = MID;
  // }
  // else if (!digitalRead(4) && digitalRead(3)) {
  //   sample_range = LONG;
  // }
  // else if (!digitalRead(3) && !digitalRead(4)) {
  //   sample_range = CLOSEST;
  // }
  // else {
  //   sample_range = NARROW;
  // }

  // if (sample_range == NARROW) {
  //   divider = 8;
  //   slice_mid = 52;
  //   sliceA = 2;
  //   sliceB = 3;
  // }
  // else if (sample_range == CLOSEST) {
  //   divider = 16;
  //   slice_mid = 22;
  //   sliceA = 1;
  //   sliceB = 2;
  // }
  // else if (sample_range == MID) {
  //   divider = 4;
  //   slice_mid = 92;
  //   sliceA = 4;
  //   sliceB = 12;
  // }
  // else if (sample_range == LONG) {
  //   divider = 2;
  //   slice_mid = 184;
  //   sliceA = 8;
  //   sliceB = 18;
  // }

  divider = 16;
  slice_mid = 22;
  sliceA = 1;
  sliceB = 2;
 
  matrix.fill(0);
  matrix2.fill(0);
  // Wait for DMA to finish (may have already)
  dma_channel_wait_for_finish_blocking(dma_chan);

  // Stop and clean out FIFO
  adc_run(false);
  adc_fifo_drain();

  // Copy samples into buffer for approxFFT
  for (int i = 0; i < SAMPLES; i++) {
    streamBuffer[i] = sampleBuffer[i] / 4 - 512 + 49;  // The +49 compensated for a slight DC offset from VCC/2
    // Serial.println(streamBuffer[i]); delay(100);
    approxBuffer[i] = streamBuffer[i];
  }

  // Now we have a copy of the samples we can start capture again
  dma_channel_configure(dma_chan, &cfg,
                        (uint16_t *)sampleBuffer,  // dst
                        &adc_hw->fifo,             // src
                        SAMPLES,                   // transfer count
                        true                       // start immediately
  );

  // Restart the ADC capture
  adc_run(true);

  // Do the FFT number crunching
  // approxBuffer contains samples, but will be updated with amplitudes
  // Approx_FFT(approxBuffer, SAMPLES, SAMPLING_FREQUENCY);

  int peak_freq = int(Approx_FFT(approxBuffer, SAMPLES, sampling_frequency));

  unsigned int sample_bin[20];
  int sample_counter = 0;
  int peak_counter = 0;

  // float freq_slices = (SAMPLING_FREQUENCY / SAMPLES) * 5; // frequency slices per peak_range.
  // int peak_range = map(peak_freq, 0, SAMPLING_FREQUENCY, 0, WIDTH -1);
  // Serial.println(slices);

  for (int i = 0; i < SAMPLES / divider; i++) {
    int maxVal = 0;
    if (i < slice_mid) {
      sample_bin[sample_counter] = abs(approxBuffer[i+bias]);
      if (sample_counter < sliceA - 1) {
        sample_counter++;
      } else {
        for (int t = 0; t < sliceA; t++) {
          maxVal = max(sample_bin[t], maxVal);
        }
        peakBuffer[peak_counter] = maxVal;
        sample_counter = 0;
        peak_counter++;
      }
    } else {
      sample_bin[sample_counter] = abs(approxBuffer[i+bias]);
      if (sample_counter < sliceB - 1) {
        sample_counter++;
      } else {
        for (int t = 0; t < sliceB; t++) {
          maxVal = max(sample_bin[t], maxVal);
        }
        peakBuffer[peak_counter] = maxVal;
        sample_counter = 0;
        peak_counter++;
      }
    }
  }

  for (int i = 0; i < 26; i++) {
    int col = matrix.color565(heatmap[i]);
    // if (peakBuffer[i] > maximum) {
    //   maximum = peakBuffer[i];
    // }
    int y;
    if (peakBuffer[i] > TOLERANCE) {
      y = map(peakBuffer[i], 0, max_amplitude, 0, 8);
    }
    else {
      y = 0;
    }
    // Serial.print(peakBuffer[i]);
    // Serial.print(" / ");
    // Serial.print(y);
    // Serial.print(" / ");
    // Serial.println(maximum);
    if (y > peaks[i] && y > 2) {
      peaks[i] = y;
      
    }
    if (i < 13) {
      // int col = i == peak_range ? matrix.color565(0x00FFFF00) : matrix.color565(heatmap[peakBuffer[i] % 40]);
      if (y >= 7) {
        matrix.drawLine(12 - i, 0, 12 - i, y, peakRange);
      }
      else {
        matrix.drawLine(12 - i, 0, 12 - i, y, col);
      }
      if (peaks[i] > 2 && peakState[i] == 0) {
        peakSelected[i] = peakColor[peakBuffer[i] % 9];
        matrix.drawPixel(12 - i, peaks[i], matrix.color565(peakSelected[i]));
        peakState[i] = 1;
      }
      else if (peaks[i] > 2) {
        matrix.drawPixel(12 - i, peaks[i], matrix.color565(peakSelected[i]));
      }
    } else {
      // int col = i == peak_range ? matrix2.color565(0x00FF0000) : matrix2.color565(heatmap[peakBuffer[i] % 40]);
      if (y >= 7) {
        matrix2.drawLine(12 - (i - 13), 0, 12 - (i - 13), y, peakRange);
      }
      else {
        matrix2.drawLine(12 - (i - 13), 0, 12 - (i - 13), y, col);
      }
      if (peaks[i] > 2 && peakState[i] == 0) {
        peakSelected[i] = peakColor[peakBuffer[i] % 9];
        matrix2.drawPixel(12 - (i - 13), peaks[i], matrix2.color565(peakSelected[i]));
        peakState[i] = 1;
      }
      else if (peaks[i] > 2) {
        matrix2.drawPixel(12 - (i - 13), peaks[i], matrix2.color565(peakSelected[i]));
      }
    }
  }

  if (peak_count > 4) {
    peak_count = 0;
    for (int i = 0; i < 26; i++) {
      peaks[i] = min(peaks[i] - 1, 8);
      if (peaks[i] <= 2) {
        peakState[i] = 0;
      }
    }
  }

  matrix.show();
  matrix2.show();

  if (peak_count % 2 == 0) {
    if (color_count > 25) {
        color_direction = !color_direction;
        color_count = 0;
        // maximum = 0;
      }

    if (color_direction == 0) {
      color_next();
    }
    else {
      color_reverse();
    }
    color_count++;
  }
  
  // Serial.println(max_amplitude);
 
  peak_count++;
  // delay(500);
  //   counter++;
  //   // only calculate the fps every <interval> iterations.
  //   if (counter % interval == 0) {
  //     long millisNow = millis();
  //     fps = String((interval * 1000.0 / (millisNow - startMillis))) + " fps";
  //     startMillis = millisNow;
  //     Serial.println(fps);
  //   }
  // }
}