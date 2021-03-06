#include<iostream>
#include<string>
#include<sstream>
#include<algorithm>
#include<fstream>
#include<vector>
#include<chrono>
#include<iomanip>
#include<opencv2/core.hpp>

#include<misc.h>

cv::Matx<double, 4, 4> rs[8] = {
cv::Matx<double, 4, 4>(
  1, 0, 0, 0,
  0, 1, 0, 0,
  0, 0, 1, 0,
  0, 0, 0, 1

),
cv::Matx<double, 4, 4>(
  -0.5875666,    -0.7775254,     0.2240977,      51.54014,
 0.5122082,    -0.1429838,     0.8468756,      184.6689,
 -0.626425,     0.6123805,     0.4822674,     -127.1372,
0.0, 0.0, 0.0, 1.0
),
cv::Matx<double, 4, 4>(
  0.7920728,     0.5675062,    -0.2248498,     -51.73229,
0.1441103,    -0.5317821,    -0.8345298,     -174.9063,
-0.5931719,     0.6286052,    -0.5029936,     -323.9452,
0.0, 0.0, 0.0, 1.0
),
cv::Matx<double, 4, 4>(
  -0.03350636,      -0.99939,   0.009841846,       5.34964,
 -0.9992037,    0.03328344,   -0.02200208,     -1.782479,
 0.02166109,   -0.01057122,    -0.9997095,     -440.1455,
0.0, 0.0, 0.0, 1.0
),
cv::Matx<double, 4, 4>(
-0.1189993856, 0.407829603511, 0.905270214205, 0.0390169,
0.971631732915, -0.139844138768, 0.190723340064, -0.0608652,
0.204379357617, 0.902285227271, -0.3796188178, 0.0232081,
0.0, 0.0, 0.0, 1.0
),
cv::Matx<double, 4, 4>(
0.490534487042, -0.60222280151, -0.629844119099, -0.0365087,
-0.331667216146, 0.53935518744, -0.774010878164, -0.0621305,
0.805836692362, 0.588577674672, 0.0648347601281, 0.0218305,
0.0, 0.0, 0.0, 1.0
),
cv::Matx<double, 4, 4>(
-0.481306690282, -0.792598029877, 0.374342400652, 0.037505,
0.273959218692, -0.541688916933, -0.794681989078, 0.062768,
0.832640508493, -0.279931206384, 0.477858036774, 0.0209629,
0.0, 0.0, 0.0, 1.0
),
cv::Matx<double, 4, 4>(
0.131860969255, 0.982046353526, -0.134898644594, -0.0380713,
-0.961836880762, 0.159670583371, 0.22220512958, 0.0618759,
0.239755082517, 0.100450307776, 0.965622719324, 0.0210265,
0.0, 0.0, 0.0, 1.0
)
};



// unwarp cameraMatrix output
//
/*
cv::Matx<double, 4, 4> rs[8] = {
cv::Matx<double, 4, 4>(
  -0.0112594,  0.725651,  0.687971,0.000197691,
  0.0101927, -0.687895,  0.725738,-0.000408114,
   0.999885, 0.0151837,0.000349045, 0.0754296,
   0 ,         0,         0,         1
),
cv::Matx<double, 4, 4>(
  -0.224101, -0.963521,  0.146301,-9.74564e-05,
   0.836199, -0.267208, -0.478925, 0.0002825 ,
   0.500548,  0.015009,  0.865579, 0.0748124 ,
0.0, 0.0, 0.0, 1.0
),
cv::Matx<double, 4, 4>(
  0.224144,  0.965926,   0.12941,-0.000102305         ,
  -0.836516,  0.258819, -0.482963,-0.000566356         ,
       -0.5,-4.47035e-08,  0.866026, 0.0745087        ,
 0.0, 0.0, 0.0, 1.0
),
cv::Matx<double, 4, 4>(
  0.0111808, -0.712315,   0.70177,0.000368726,
  -0.00978908,  0.701703,  0.712402,-0.000267715,
   -0.999889,-0.0148347,0.000872731, 0.0754296,
 0.0, 0.0, 0.0, 1.0
),
cv::Matx<double, 4, 4>(
  0.558149,  0.574535,  0.598648,0.000701563,
  -0.653717, -0.139844,  0.743705,0.000265621,
   0.511002, -0.806445,   0.29753, 0.0759272,
  0.0, 0.0, 0.0, 1.0
),
cv::Matx<double, 4, 4>(
  0.722973,  -0.18962,  0.664345,-0.000110676         ,
  -0.488694,  0.539355,  0.685765,-0.000698227         ,
  -0.488353, -0.820451,  0.297273, 0.0752938 ,
0.0, 0.0, 0.0, 1.0
),
cv::Matx<double, 4, 4>(
  -0.67729,  0.154304,  0.719352,-0.00063673         ,
  0.550903, -0.541689,  0.634885,-3.01115e-05         ,
  0.487631,  0.826295,  0.281874, 0.0760624 ,
0.0, 0.0, 0.0, 1.0
),
cv::Matx<double, 4, 4>(
  -0.552549, -0.561271 , 0.616168,-0.000736984         ,
   0.658216,  0.159671,  0.735701,0.000289802         ,
  -0.511312,  0.812083,  0.281213, 0.0756276  ,
0.0, 0.0, 0.0, 1.0
)

};
*/
cv::Matx<double, 6, 1> c;

using namespace std;
//using MultiColSLAM

int main(int argc, char **argv)
{
  cout << endl << "Convert..." << endl;
  // cout << endl << rs << endl;
  //cout << endl << c << endl;
  //
  int n = 1;
  for (auto& rotation : rs) {
    //cout << endl << rotation << endl;
    c = MultiColSLAM::hom2cayley<double>(rotation);
    //CameraSystem.cam1_1
    //cout << rotation(0,3) << rotation(1,3) << rotation(2,3) << endl;
    for (unsigned int j=0; j<6; j++){
      cout << "CameraSystem.cam" << n << "_" << (j+1) << ": ";
      cout << c(j) << endl;
    }
    //cout << c << endl;
    n ++;

  }

   /*

  const cv::Matx<double, 6, 1> c1 = {-0.0754145, -0.0015695, 0.000133857, 89.98, -0.869996, 133.47};
  cv::Matx<double, 4, 4>  ht1 = MultiColSLAM::cayley2hom<double>(c1);
  cv::Matx<double, 6, 1> c11 = MultiColSLAM::hom2cayley<double>(ht1);

  cout << "caylay c1: " << c1 << endl;
  cout << "caylay c11: " << c11 << endl;
  cout << " HT1: " <<  ht1 << endl;


  cout << MultiColSLAM::cayley2hom<double>(c1) << endl;
  const cv::Matx<double, 6, 1> c2 = {-0.0103943566650926, 1.12505249943085, -0.402901183146028, 0.108753418890423 ,0.0636197216520153,0.0657911760105832};
  cout << MultiColSLAM::cayley2hom<double>(c2) << endl;
  const cv::Matx<double, 6, 1> c3 = {0, 0, 0, -0.00157612288268783,0.103615531247527,0.201416323496156};
  cout << MultiColSLAM::cayley2hom<double>(c3) << endl;

  */
}
