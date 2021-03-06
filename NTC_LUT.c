/*
 * NTC_LUT.c
 *
 *  Created on: Apr 15, 2021
 *      Author: Zoltan Gere
 */

int TempLUT[] = {
               -39.44, -38.33, -37.22, -36.11, -35.00, -33.89, -32.78, -31.67, -30.56, -29.44, -28.33, -27.22, -26.11, -25.00, -23.89, -22.78, -21.67, -20.56, -19.44,
               -18.33, -17.22, -16.11, -15.00, -13.89, -12.78, -11.67, -10.56,  -9.44,  -8.33,  -7.22,  -6.11,  -5.00,  -3.89,  -2.78,  -1.67,  -0.56,   0.56,   1.67,
                 2.78,   3.89,   5.00,   6.11,   7.22,   8.33,   9.44,  10.56,  11.67,  12.78,  13.89,  15.00,  16.11,  17.22,  18.33,  19.44,  20.56,  21.67,  22.78,
                23.89,  25.00,  26.11,  27.22,  28.33,  29.44,  30.56,  31.67,  32.78,  33.89,  35.00,  36.11,  37.22,  38.33,  39.44,  40.56,  41.67,  42.78,  43.89,
                45.00,  46.11,  47.22,  48.33,  49.44,  50.56,  51.67,  52.78,  53.89,  55.00,  56.11,  57.22,  58.33,  59.44,  60.56,  61.67,  62.78,  63.89,  65.00,
                66.11,  67.22,  68.33,  69.44,  70.56,  71.67,  72.78,  73.89,  75.00,  76.11,  77.22,  78.33,  79.44,  80.56,  81.67,  82.78,  83.89,  85.00,  86.11};

long int ResLUT[] = {
                   3916295, 3627711, 3362274, 3117987, 2893035, 2685770, 2494694, 2318444, 2155781, 2004274, 1865595, 1737397, 1618827, 1509102, 1407512, 1313405, 1226184, 1145306, 1069620,
                   1000019,  935383,  875329,  819505,  767589,  719284,  674319,  632442,  593086,  556739,  522842,  491217,  461699,  434134,  408383,  384316,  361813,  340581,  320895,
                    302466,  285206,  269035,  253877,  239664,  226331,  213819,  201971,  190946,  180588,  170853,  161700,  153092,  144992,  137367,  130189,  123368,  117000,  110998,
                    105338,  100000,   94963,   90208,   85719,   81479,   77438,   73654,   70076,   66692,   63491,   60461,   57594,   54878,   52306,   49847,   47538,   45349,   43273,
                     41303,   39434,   37660,   35976,   34376,   32843,   31399,   30027,   28722,   27481,   26300,   25177,   24107,   23089,   22111,   21188,   20308,   19469,   18670,
                     17907,   17180,   16486,   15824,   15187,   14584,   14008,   13458,   12932,   12430,   11949,   11490,   11051,   10627,   10225,    9841,    9473,    9121,    8783};

int LookUpTEMP(long int res) {
    unsigned int i = 0;
    const int tableLength = sizeof(ResLUT) / sizeof(long int);
    float Y;
    while ((ResLUT[i] > res) && (i < tableLength - 1)) {
        i++;
    }
    // i points to lower bound
    if (i > 0) {
        // Calculate linear interpolation between higher bound (i-1) and lower bound (i)
        Y = ((float)(res - ResLUT[i]) / (ResLUT[i-1] - ResLUT[i]));
        Y *= (TempLUT[i-1] - TempLUT[i]);
        Y += TempLUT[i];
    } else {
        // Resistance is out of table, return the closest value (temperature associated to the largest resistance)
        Y = (float)TempLUT[0];
    }
    return (int)(Y * 100);
}





//  -39.44 3916295
//  -38.33 3627711
//  -37.22 3362274
//  -36.11 3117987
//  -35.00 2893035
//  -33.89 2685770
//  -32.78 2494694
//  -31.67 2318444
//  -30.56 2155781
//  -29.44 2004274
//  -28.33 1865595
//  -27.22 1737397
//  -26.11 1618827
//  -25.00 1509102
//  -23.89 1407512
//  -22.78 1313405
//  -21.67 1226184
//  -20.56 1145306
//  -19.44 1069620
//  -18.33 1000019
//  -17.22 935383
//  -16.11 875329
//  -15.00 819505
//  -13.89 767589
//  -12.78 719284
//  -11.67 674319
//  -10.56 632442
//  -9.44 593086
//  -8.33 556739
//  -7.22 522842
//  -6.11 491217
//  -5.00 461699
//  -3.89 434134
//  -2.78 408383
//  -1.67 384316
//  -0.56 361813
//  0.56 340581
//  1.67 320895
//
//  2.78 302466
//  3.89 285206
//  5.00 269035
//  6.11 253877
//  7.22 239664
//  8.33 226331
//  9.44 213819
//  10.56 201971
//  11.67 190946
//  12.78 180588
//  13.89 170853
//  15.00 161700
//  16.11 153092
//  17.22 144992
//  18.33 137367
//  19.44 130189
//  20.56 123368
//  21.67 117000
//  22.78 110998
//  23.89 105338
//  25.00 100000
//  26.11 94963
//  27.22 90208
//  28.33 85719
//  29.44 81479
//  30.56 77438
//  31.67 73654
//  32.78 70076
//  33.89 66692
//  35.00 63491
//  36.11 60461
//  37.22 57594
//  38.33 54878
//  39.44 52306
//  40.56 49847
//  41.67 47538
//  42.78 45349
//  43.89 43273
//
//  45.00 41303
//  46.11 39434
//  47.22 37660
//  48.33 35976
//  49.44 34376
//  50.56 32843
//  51.67 31399
//  52.78 30027
//  53.89 28722
//  55.00 27481
//  56.11 26300
//  57.22 25177
//  58.33 24107
//  59.44 23089
//  60.56 22111
//  61.67 21188
//  62.78 20308
//  63.89 19469
//  65.00 18670
//  66.11 17907
//  67.22 17180
//  68.33 16486
//  69.44 15824
//  70.56 15187
//  71.67 14584
//  72.78 14008
//  73.89 13458
//  75.00 12932
//  76.11 12430
//  77.22 11949
//  78.33 11490
//  79.44 11051
//  80.56 10627
//  81.67 10225
//  82.78 9841
//  83.89 9473
//  85.00 9121
//  86.11 8783
