import numpy as np
import math

import matplotlib.pyplot as plt

# int array instead of float array
# 4x200 points per 360 deg
# 2x storage save (int 2Byte float 4 Byte )
# sin*10000
sine_array = np.array([0,79,158,237,316,395,473,552,631,710,789,867,946,1024,1103,1181,1260,1338,1416,1494,1572,1650,1728,1806,1883,1961,2038,2115,2192,2269,2346,2423,2499,2575,2652,2728,2804,2879,2955,3030,3105,3180,3255,3329,3404,3478,3552,3625,3699,3772,3845,3918,3990,4063,4135,4206,4278,4349,4420,4491,4561,4631,4701,4770,4840,4909,4977,5046,5113,5181,5249,5316,5382,5449,5515,5580,5646,5711,5775,5839,5903,5967,6030,6093,6155,6217,6279,6340,6401,6461,6521,6581,6640,6699,6758,6815,6873,6930,6987,7043,7099,7154,7209,7264,7318,7371,7424,7477,7529,7581,7632,7683,7733,7783,7832,7881,7930,7977,8025,8072,8118,8164,8209,8254,8298,8342,8385,8428,8470,8512,8553,8594,8634,8673,8712,8751,8789,8826,8863,8899,8935,8970,9005,9039,9072,9105,9138,9169,9201,9231,9261,9291,9320,9348,9376,9403,9429,9455,9481,9506,9530,9554,9577,9599,9621,9642,9663,9683,9702,9721,9739,9757,9774,9790,9806,9821,9836,9850,9863,9876,9888,9899,9910,9920,9930,9939,9947,9955,9962,9969,9975,9980,9985,9989,9992,9995,9997,9999,10000,10000], dtype='uint16')

values = np.array([3377,3401,3425,3426,3426,3443,3470,3494,3519,3521,3521,3538,3560,3581,3610,3612,3613,3633,3662,3684,3708,3712,3713,3727,3755,3778,3805,3810,3811,3822,3851,3876,3903,3910,3913,3920,3954,3974,3999,4007,4009,4013,4045,4067,4091,10,11,14,45,69,94,112,114,114,145,167,190,209,208,209,233,256,281,307,308,308,330,356,379,401,406,406,428,449,469,491,496,497,510,543,569,594,598,599,616,638,662,693,698,700,713,739,755,782,788,789,797,826,849,875,886,886,892,921,943,968,984,983,987,1020,1042,1066,1081,1081,1083,1107,1131,1157,1172,1176,1177,1203,1224,1247,1274,1274,1274,1299,1319,1344,1364,1368,1367,1389,1417,1434,1462,1463,1463,1482,1510,1532,1556,1558,1559,1577,1605,1626,1649,1652,1653,1667,1695,1720,1747,1753,1755,1766,1793,1815,1840,1846,1848,1859,1891,1913,1938,1948,1951,1956,1988,2011,2036,2050,2051,2053,2083,2108,2133,2152,2153,2154,2183,2201,2227,2244,2244,2244,2268,2293,2315,2338,2338,2338,2360,2386,2411,2440,2440,2440,2462,2493,2516,2539,2540,2541,2560,2591,2616,2642,2645,2646,2662,2690,2713,2740,2745,2746,2762,2789,2809,2832,2837,2838,2845,2874,2900,2928,2938,2941,2947,2978,3001,3024,3037,3038,3041,3070,3092,3114,3125,3128,3131,3155,3178,3202,3222,3223,3224,3247,3274,3299,3322,3324,3324,3347,3374])

values2 = np.array([255., 238., 221., 207., 193., 181., 169., 159., 150., 142., 135., 128., 123., 118.,
 114., 111., 108., 106., 105., 104., 104., 104., 104., 105., 106., 107., 108., 110.,
 111., 113., 115., 117., 119., 120., 122., 124., 125., 126., 127., 127., 128., 128.,
 127., 126., 125., 124., 122., 119., 116., 113., 108., 104.,  99.,  93.,  86.,  79.,
  72.,  63.,  55.,  45.,  35.,  24.,  12.,   0.,])

#  function approximating the sine calculation by using fixed size array
#  ~40us (float array)
#  ~50us (int array)
#  precision +-0.005
#  it has to receive an angle in between 0 and 2PI

def _sin(a) :
  if a < math.pi/2:
    return 0.0001*sine_array[np.array((round(126.6873 * a))).astype(int)]      # int array optimized
  elif a < math.pi:
    return 0.0001*sine_array[np.array(398 - round(126.6873*a)).astype(int)]     # int array optimized
  elif a < 3 * math.pi / 2:
    return -0.0001*sine_array[np.array(-398 + round(126.6873*a)).astype(int)]      # int array optimized
  else:
    return -0.0001*sine_array[np.array(796 - round(126.6873*a)).astype(int)];      # int array optimized
 

#x: 0 - 255
def sin_lin_approx(x):
  return (-512 + (147 * x) - (int)(x * x / 4))

def _sin2(a) : 
  #if a < math.pi/2:
    return (values2[(int)((a/256.0)*64.0) ] / 3439 - 3.71 / 77 + (sin_lin_approx(a)) / 19712)     # int array optimized
  # elif a < math.pi:
  #   return 0.0001*values2[np.array(398 - round(126.6873*a)).astype(int)]     # int array optimized
  # elif a < 3 * math.pi / 2:
  #   return -0.0001*values2[np.array(-398 + round(126.6873*a)).astype(int)]      # int array optimized
  # else:
  #   return -0.0001*values2[np.array(796 - round(126.6873*a)).astype(int)];      # int array optimized

xhat = np.linspace(0, math.pi/2, 2000)
yhat = np.vectorize(_sin)(xhat)
ysin = np.vectorize(math.sin)(xhat)

xhathat = np.linspace(0, 256.0, 2000, endpoint=False)
#yhat = np.vectorize(_sin)(xhat)
ysinsin = np.vectorize(math.sin)(math.pi/2 * (xhathat/256))
yhathat = np.vectorize(_sin2)(xhathat)

min_v = min(77.0 * ysinsin-yhathat)
max_v = max(77.0 * ysinsin-yhathat-min_v)
#print(min_v)
#print(max_v)
#print(np.round_(255 * ((77 * ysinsin - yhathat - min_v)/max_v)))

#print(list(xhathat))
#print(list(77 * np.sin(math.pi/2 * (xhathat / 255))))
#zipped = zip(xhathat, ysinsin)
#print(list(zipped))

#plt.plot(xhat, ysin - yhat)
#plt.show()

#values = np.sort(values)

#xp = range(len(values))

#plt.plot(xp, (values - values[0]) - np.array(xp) * 16)
#plt.plot(xp, np.array(xp) * 16, 'o')
plt.plot(xhat, ysin - yhat)
#plt.plot(xhathat, yhathat)
plt.plot(math.pi / 2 * (xhathat/256), yhathat - ysinsin)
#plt.plot(xhat, ysin)
plt.show()