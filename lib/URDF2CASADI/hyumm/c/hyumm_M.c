/* This file was automatically generated by CasADi 3.6.3+.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) hyumm_M_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[13] = {9, 1, 0, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8};
static const casadi_int casadi_s1[93] = {9, 9, 0, 9, 18, 27, 36, 45, 54, 63, 72, 81, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8};

/* M:(i0[9])->(o0[9x9]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a11, a110, a111, a112, a113, a114, a115, a116, a117, a118, a119, a12, a120, a121, a122, a123, a124, a125, a126, a127, a128, a129, a13, a130, a131, a132, a133, a134, a135, a136, a137, a138, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a8, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a9, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99;
  a0=7.9617501299999986e+01;
  if (res[0]!=0) res[0][0]=a0;
  a0=0.;
  if (res[0]!=0) res[0][1]=a0;
  a1=arg[0]? arg[0][2] : 0;
  a2=cos(a1);
  a3=-7.9617499299999992e+01;
  a4=-4.3799416342633164e-04;
  a5=3.1036792761965087e-01;
  a6=arg[0]? arg[0][3] : 0;
  a7=sin(a6);
  a8=-1.0999038140348657e-04;
  a9=5.3686310411601923e-01;
  a10=2.2204460492503131e-16;
  a11=arg[0]? arg[0][4] : 0;
  a12=cos(a11);
  a13=(a10*a12);
  a11=sin(a11);
  a13=(a13+a11);
  a14=-1.2993458852078907e-01;
  a15=5.5920810843623758e-01;
  a16=-4.5000000000000001e-01;
  a17=arg[0]? arg[0][5] : 0;
  a18=cos(a17);
  a19=-6.0751819848336858e-02;
  a20=6.3846844796900415e-01;
  a21=-2.6700000000000002e-01;
  a22=arg[0]? arg[0][6] : 0;
  a23=cos(a22);
  a24=(a10*a23);
  a25=-2.2204460492503131e-16;
  a22=sin(a22);
  a26=(a25*a22);
  a24=(a24+a26);
  a26=-1.2072237989225459e-04;
  a27=5.5033195555460734e-01;
  a28=arg[0]? arg[0][7] : 0;
  a29=cos(a28);
  a30=(a10*a29);
  a28=sin(a28);
  a30=(a30+a28);
  a31=-8.3585558345339220e-02;
  a32=1.4675807652024542e-01;
  a33=-1.6800000000000001e-01;
  a34=8.1469787034552752e-05;
  a35=arg[0]? arg[0][8] : 0;
  a36=cos(a35);
  a37=(a10*a36);
  a35=sin(a35);
  a38=(a25*a35);
  a37=(a37+a38);
  a38=(a34*a37);
  a39=-4.6555878300977510e-04;
  a40=(a25*a36);
  a41=(a10*a35);
  a40=(a40-a41);
  a41=(a39*a40);
  a42=-3.0791490739415412e-02;
  a41=(a41+a42);
  a38=(a38+a41);
  a38=(a33+a38);
  a41=(a32*a38);
  a31=(a31+a41);
  a41=(a30*a31);
  a42=(a10*a28);
  a42=(a29-a42);
  a43=-1.9721833819311043e-04;
  a44=(a34*a36);
  a45=6.8370843962862553e-18;
  a46=(a39*a35);
  a46=(a45-a46);
  a44=(a44+a46);
  a46=(a32*a44);
  a43=(a43+a46);
  a46=(a42*a43);
  a47=5.4999052886227598e-02;
  a48=6.9000000000000006e-02;
  a39=(a39*a36);
  a45=(a45-a39);
  a34=(a34*a35);
  a45=(a45-a34);
  a45=(a48+a45);
  a32=(a32*a45);
  a47=(a47+a32);
  a32=(a10*a47);
  a46=(a46+a32);
  a41=(a41+a46);
  a46=(a27*a41);
  a26=(a26+a46);
  a46=(a24*a26);
  a25=(a25*a23);
  a32=(a10*a22);
  a25=(a25-a32);
  a32=-3.1919416334129543e-02;
  a34=-1.1400000000000000e-01;
  a39=(a10*a28);
  a49=(a39*a31);
  a50=(a10*a29);
  a51=(a50*a43);
  a51=(a51-a47);
  a49=(a49+a51);
  a49=(a34+a49);
  a51=(a27*a49);
  a32=(a32+a51);
  a51=(a25*a32);
  a52=3.4395684294724983e-02;
  a53=8.3000000000000004e-02;
  a54=(a10*a28);
  a54=(a54-a29);
  a55=(a54*a31);
  a29=(a10*a29);
  a28=(a28+a29);
  a29=(a28*a43);
  a55=(a55+a29);
  a55=(a53+a55);
  a27=(a27*a55);
  a52=(a52+a27);
  a51=(a51-a52);
  a46=(a46+a51);
  a46=(a21+a46);
  a51=(a20*a46);
  a19=(a19+a51);
  a51=(a18*a19);
  a17=sin(a17);
  a27=7.7443673760559616e-05;
  a29=(a23*a26);
  a56=(a10*a52);
  a57=(a22*a32);
  a56=(a56-a57);
  a29=(a29+a56);
  a56=(a20*a29);
  a27=(a27+a56);
  a56=(a17*a27);
  a51=(a51-a56);
  a51=(a16+a51);
  a56=(a15*a51);
  a14=(a14+a56);
  a56=(a13*a14);
  a57=(a10*a11);
  a57=(a12-a57);
  a58=-1.0407096559820434e-05;
  a59=(a17*a19);
  a60=(a18*a27);
  a59=(a59+a60);
  a60=(a15*a59);
  a58=(a58+a60);
  a60=(a57*a58);
  a61=2.0992951863345637e-02;
  a62=-3.0499999999999999e-02;
  a63=-2.5308593308013983e-02;
  a64=-7.4999999999999997e-02;
  a65=(a10*a52);
  a66=(a23*a32);
  a65=(a65-a66);
  a66=(a22*a26);
  a65=(a65-a66);
  a65=(a64+a65);
  a20=(a20*a65);
  a63=(a63+a20);
  a20=(a62+a63);
  a15=(a15*a20);
  a61=(a61+a15);
  a15=(a10*a61);
  a60=(a60+a15);
  a56=(a56+a60);
  a60=(a9*a56);
  a8=(a8+a60);
  a60=(a7*a8);
  a6=cos(a6);
  a15=-1.9962649831083683e-02;
  a66=-1.0900000000000000e-01;
  a67=(a10*a11);
  a68=(a67*a14);
  a69=(a10*a12);
  a70=(a69*a58);
  a70=(a70-a61);
  a68=(a68+a70);
  a68=(a66+a68);
  a9=(a9*a68);
  a15=(a15+a9);
  a9=(a6*a15);
  a60=(a60+a9);
  a9=(a5*a60);
  a4=(a4+a9);
  a3=(a3*a4);
  a9=(a2*a3);
  a1=sin(a1);
  a70=7.9617499299999992e+01;
  a71=-9.1754676253268871e-03;
  a72=2.9999999999999999e-01;
  a73=(a6*a8);
  a74=(a7*a15);
  a73=(a73-a74);
  a73=(a72+a73);
  a5=(a5*a73);
  a71=(a71+a5);
  a5=(a70*a71);
  a74=(a1*a5);
  a9=(a9-a74);
  if (res[0]!=0) res[0][2]=a9;
  a74=-2.4710718260000000e+01;
  a74=(a74*a15);
  a75=(a6*a74);
  a76=2.4710718260000000e+01;
  a77=(a76*a8);
  a78=(a7*a77);
  a75=(a75-a78);
  a78=(a2*a75);
  a74=(a7*a74);
  a77=(a6*a77);
  a74=(a74+a77);
  a77=(a1*a74);
  a78=(a78-a77);
  if (res[0]!=0) res[0][3]=a78;
  a77=-1.3266272909999998e+01;
  a77=(a77*a58);
  a79=(a13*a77);
  a80=1.3266272909999998e+01;
  a81=(a80*a14);
  a82=(a57*a81);
  a79=(a79+a82);
  a82=(a6*a79);
  a77=(a67*a77);
  a81=(a69*a81);
  a77=(a77+a81);
  a81=(a7*a77);
  a82=(a82-a81);
  a81=(a2*a82);
  a83=(a7*a79);
  a77=(a6*a77);
  a83=(a83+a77);
  a77=(a1*a83);
  a81=(a81-a77);
  if (res[0]!=0) res[0][4]=a81;
  a77=-7.4186073799999992e+00;
  a77=(a77*a27);
  a84=(a18*a77);
  a85=7.4186073799999992e+00;
  a86=(a85*a19);
  a87=(a17*a86);
  a84=(a84-a87);
  a87=(a13*a84);
  a77=(a17*a77);
  a86=(a18*a86);
  a77=(a77+a86);
  a86=(a57*a77);
  a87=(a87+a86);
  a86=(a6*a87);
  a88=(a67*a84);
  a89=(a69*a77);
  a88=(a88+a89);
  a89=(a7*a88);
  a86=(a86-a89);
  a89=(a2*a86);
  a90=(a7*a87);
  a88=(a6*a88);
  a90=(a90+a88);
  a88=(a1*a90);
  a89=(a89-a88);
  if (res[0]!=0) res[0][5]=a89;
  a88=-4.7365467399999996e+00;
  a88=(a88*a32);
  a91=(a24*a88);
  a92=4.7365467399999996e+00;
  a93=(a92*a26);
  a94=(a25*a93);
  a91=(a91+a94);
  a94=(a18*a91);
  a95=(a23*a88);
  a96=(a22*a93);
  a95=(a95-a96);
  a96=(a17*a95);
  a94=(a94-a96);
  a96=(a13*a94);
  a97=(a17*a91);
  a98=(a18*a95);
  a97=(a97+a98);
  a98=(a57*a97);
  a88=(a22*a88);
  a93=(a23*a93);
  a88=(a88+a93);
  a93=(a10*a88);
  a98=(a98-a93);
  a96=(a96+a98);
  a98=(a6*a96);
  a93=(a67*a94);
  a99=(a69*a97);
  a99=(a99+a88);
  a93=(a93+a99);
  a99=(a7*a93);
  a98=(a98-a99);
  a99=(a2*a98);
  a100=(a7*a96);
  a93=(a6*a93);
  a100=(a100+a93);
  a93=(a1*a100);
  a99=(a99-a93);
  if (res[0]!=0) res[0][6]=a99;
  a93=-2.6066730300000001e+00;
  a93=(a93*a43);
  a101=(a30*a93);
  a102=2.6066730300000001e+00;
  a103=(a102*a31);
  a104=(a42*a103);
  a101=(a101+a104);
  a104=(a24*a101);
  a105=(a39*a93);
  a106=(a50*a103);
  a105=(a105+a106);
  a106=(a25*a105);
  a93=(a54*a93);
  a103=(a28*a103);
  a93=(a93+a103);
  a106=(a106-a93);
  a104=(a104+a106);
  a106=(a18*a104);
  a103=(a23*a101);
  a107=(a10*a93);
  a108=(a22*a105);
  a107=(a107-a108);
  a103=(a103+a107);
  a107=(a17*a103);
  a106=(a106-a107);
  a107=(a13*a106);
  a108=(a17*a104);
  a109=(a18*a103);
  a108=(a108+a109);
  a109=(a57*a108);
  a110=(a10*a93);
  a111=(a23*a105);
  a110=(a110-a111);
  a111=(a22*a101);
  a110=(a110-a111);
  a111=(a10*a110);
  a109=(a109+a111);
  a107=(a107+a109);
  a109=(a6*a107);
  a111=(a67*a106);
  a112=(a69*a108);
  a112=(a112-a110);
  a111=(a111+a112);
  a112=(a7*a111);
  a109=(a109-a112);
  a112=(a2*a109);
  a113=(a7*a107);
  a111=(a6*a111);
  a113=(a113+a111);
  a111=(a1*a113);
  a112=(a112-a111);
  if (res[0]!=0) res[0][7]=a112;
  a111=1.7809966141920004e-04;
  a114=(a111*a37);
  a115=3.1166293100400009e-05;
  a116=(a115*a40);
  a114=(a114+a116);
  a116=(a30*a114);
  a117=(a111*a36);
  a118=(a115*a35);
  a117=(a117-a118);
  a118=(a42*a117);
  a111=(a111*a35);
  a115=(a115*a36);
  a111=(a111+a115);
  a115=(a10*a111);
  a118=(a118-a115);
  a116=(a116+a118);
  a118=(a24*a116);
  a115=(a39*a114);
  a119=(a50*a117);
  a119=(a119+a111);
  a115=(a115+a119);
  a119=(a25*a115);
  a120=(a54*a114);
  a121=(a28*a117);
  a120=(a120+a121);
  a119=(a119-a120);
  a118=(a118+a119);
  a119=(a18*a118);
  a121=(a23*a116);
  a122=(a10*a120);
  a123=(a22*a115);
  a122=(a122-a123);
  a121=(a121+a122);
  a122=(a17*a121);
  a119=(a119-a122);
  a13=(a13*a119);
  a122=(a17*a118);
  a123=(a18*a121);
  a122=(a122+a123);
  a57=(a57*a122);
  a123=(a10*a120);
  a124=(a23*a115);
  a123=(a123-a124);
  a124=(a22*a116);
  a123=(a123-a124);
  a124=(a10*a123);
  a57=(a57+a124);
  a13=(a13+a57);
  a57=(a6*a13);
  a67=(a67*a119);
  a69=(a69*a122);
  a69=(a69-a123);
  a67=(a67+a69);
  a69=(a7*a67);
  a57=(a57-a69);
  a69=(a2*a57);
  a7=(a7*a13);
  a6=(a6*a67);
  a7=(a7+a6);
  a6=(a1*a7);
  a69=(a69-a6);
  if (res[0]!=0) res[0][8]=a69;
  if (res[0]!=0) res[0][9]=a0;
  a0=7.9617500299999989e+01;
  if (res[0]!=0) res[0][10]=a0;
  a3=(a1*a3);
  a5=(a2*a5);
  a3=(a3+a5);
  if (res[0]!=0) res[0][11]=a3;
  a75=(a1*a75);
  a5=(a2*a74);
  a75=(a75+a5);
  if (res[0]!=0) res[0][12]=a75;
  a82=(a1*a82);
  a5=(a2*a83);
  a82=(a82+a5);
  if (res[0]!=0) res[0][13]=a82;
  a86=(a1*a86);
  a5=(a2*a90);
  a86=(a86+a5);
  if (res[0]!=0) res[0][14]=a86;
  a98=(a1*a98);
  a5=(a2*a100);
  a98=(a98+a5);
  if (res[0]!=0) res[0][15]=a98;
  a109=(a1*a109);
  a5=(a2*a113);
  a109=(a109+a5);
  if (res[0]!=0) res[0][16]=a109;
  a1=(a1*a57);
  a2=(a2*a7);
  a1=(a1+a2);
  if (res[0]!=0) res[0][17]=a1;
  if (res[0]!=0) res[0][18]=a9;
  if (res[0]!=0) res[0][19]=a3;
  a3=6.6014850698111980e+00;
  a9=6.0031470000000003e-02;
  a2=(a10*a11);
  a2=(a2-a12);
  a57=3.5997429999999997e-02;
  a5=1.6172099999999995e-02;
  a0=2.7988909999999999e-02;
  a6=1.1052970000000002e-02;
  a67=4.2115936804441195e-04;
  a124=2.0666968412726009e-04;
  a125=(a124*a36);
  a126=3.6860125152904353e-22;
  a127=-3.3999996207092593e-07;
  a128=(a127*a35);
  a128=(a126-a128);
  a125=(a125+a128);
  a128=(a125*a36);
  a129=(a127*a36);
  a130=-2.2560143724319785e-21;
  a131=2.1448968391715186e-04;
  a132=(a131*a35);
  a132=(a130-a132);
  a129=(a129+a132);
  a132=(a129*a35);
  a128=(a128-a132);
  a67=(a67-a128);
  a132=(a127*a36);
  a126=(a126-a132);
  a124=(a124*a35);
  a126=(a126-a124);
  a124=(a126*a35);
  a131=(a131*a36);
  a130=(a130-a131);
  a127=(a127*a35);
  a130=(a130-a127);
  a127=(a130*a36);
  a124=(a124+a127);
  a67=(a67+a124);
  a127=5.8419000022338300e-04;
  a67=(a67+a127);
  a6=(a6+a67);
  a67=3.2640797086459566e-01;
  a131=-2.3113999999999996e-04;
  a131=(a131-a44);
  a44=casadi_sq(a131);
  a132=6.4458919999999975e-02;
  a132=(a132-a45);
  a45=casadi_sq(a132);
  a44=(a44+a45);
  a44=(a67*a44);
  a6=(a6+a44);
  a44=2.7547950000000002e-02;
  a124=(a127-a124);
  a44=(a44+a124);
  a124=-9.7962319999999978e-02;
  a124=(a124-a38);
  a38=casadi_sq(a124);
  a45=casadi_sq(a131);
  a38=(a38+a45);
  a38=(a67*a38);
  a44=(a44+a38);
  a6=(a6-a44);
  a38=3.6982910000000001e-02;
  a128=(a128+a127);
  a38=(a38+a128);
  a128=casadi_sq(a124);
  a127=casadi_sq(a132);
  a128=(a128+a127);
  a128=(a67*a128);
  a38=(a38+a128);
  a38=(a38-a44);
  a128=(a6+a38);
  a127=(a39*a6);
  a45=5.5170000000000002e-05;
  a125=(a125*a37);
  a129=(a129*a40);
  a125=(a125+a129);
  a129=-5.0800927435135708e-06;
  a133=(a129*a35);
  a134=8.3001622951724945e-07;
  a135=(a134*a36);
  a133=(a133-a135);
  a125=(a125+a133);
  a45=(a45+a125);
  a125=(a67*a124);
  a125=(a125*a131);
  a45=(a45-a125);
  a125=(a50*a45);
  a133=2.;
  a135=-1.4819769999999999e-02;
  a136=(a126*a37);
  a137=(a130*a40);
  a136=(a136+a137);
  a137=(a129*a36);
  a138=(a134*a35);
  a137=(a137+a138);
  a136=(a136+a137);
  a135=(a135+a136);
  a124=(a67*a124);
  a124=(a124*a132);
  a135=(a135-a124);
  a124=(a133*a135);
  a125=(a125-a124);
  a127=(a127+a125);
  a125=(a127*a39);
  a124=(a39*a45);
  a136=(a50*a38);
  a137=-3.7400000000000001e-05;
  a126=(a126*a36);
  a130=(a130*a35);
  a126=(a126-a130);
  a134=(a134*a40);
  a129=(a129*a37);
  a134=(a134-a129);
  a126=(a126+a134);
  a137=(a137+a126);
  a67=(a67*a131);
  a67=(a67*a132);
  a137=(a137-a67);
  a67=(a137+a137);
  a136=(a136-a67);
  a124=(a124+a136);
  a136=(a124*a50);
  a125=(a125+a136);
  a128=(a128-a125);
  a6=(a54*a6);
  a136=(a28*a45);
  a6=(a6+a136);
  a136=(a6*a54);
  a45=(a54*a45);
  a38=(a28*a38);
  a45=(a45+a38);
  a38=(a45*a28);
  a136=(a136+a38);
  a128=(a128-a136);
  a128=(a128+a44);
  a0=(a0+a128);
  a128=1.1721375639086467e+00;
  a38=-7.0984400000000003e-02;
  a38=(a38-a49);
  a49=casadi_sq(a38);
  a67=7.6491279999999995e-02;
  a67=(a67-a55);
  a55=casadi_sq(a67);
  a49=(a49+a55);
  a49=(a128*a49);
  a0=(a0+a49);
  a49=1.4962110000000001e-02;
  a136=(a136+a44);
  a49=(a49+a136);
  a136=-2.6846999999999998e-04;
  a136=(a136-a41);
  a41=casadi_sq(a136);
  a55=casadi_sq(a38);
  a41=(a41+a55);
  a41=(a128*a41);
  a49=(a49+a41);
  a0=(a0-a49);
  a41=1.4430760000000001e-02;
  a125=(a125+a44);
  a41=(a41+a125);
  a125=casadi_sq(a136);
  a55=casadi_sq(a67);
  a125=(a125+a55);
  a125=(a128*a125);
  a41=(a41+a125);
  a41=(a41-a49);
  a125=(a0+a41);
  a55=(a23*a0);
  a132=-4.7679999999999998e-05;
  a131=(a6*a30);
  a126=(a45*a42);
  a131=(a131+a126);
  a126=(a50*a135);
  a134=(a39*a137);
  a126=(a126-a134);
  a131=(a131-a126);
  a132=(a132+a131);
  a131=(a128*a136);
  a131=(a131*a67);
  a132=(a132-a131);
  a131=(a133*a132);
  a126=(a10*a131);
  a134=3.8930000000000002e-05;
  a127=(a127*a30);
  a124=(a124*a42);
  a127=(a127+a124);
  a124=(a28*a135);
  a129=(a54*a137);
  a124=(a124-a129);
  a127=(a127+a124);
  a134=(a134+a127);
  a136=(a128*a136);
  a136=(a136*a38);
  a134=(a134-a136);
  a136=(a22*a134);
  a126=(a126-a136);
  a55=(a55+a126);
  a126=(a55*a23);
  a136=(a23*a134);
  a127=-1.2662959999999999e-02;
  a6=(a6*a39);
  a45=(a45*a50);
  a6=(a6+a45);
  a45=(a42*a135);
  a124=(a30*a137);
  a45=(a45-a124);
  a6=(a6+a45);
  a127=(a127+a6);
  a128=(a128*a38);
  a128=(a128*a67);
  a127=(a127-a128);
  a128=(a127+a127);
  a67=(a10*a128);
  a38=(a22*a41);
  a67=(a67-a38);
  a136=(a136+a67);
  a67=(a136*a22);
  a126=(a126-a67);
  a125=(a125-a126);
  a131=(a10*a131);
  a67=(a23*a134);
  a131=(a131-a67);
  a0=(a22*a0);
  a131=(a131-a0);
  a0=(a131*a22);
  a128=(a10*a128);
  a41=(a23*a41);
  a128=(a128-a41);
  a134=(a22*a134);
  a128=(a128-a134);
  a134=(a128*a23);
  a0=(a0+a134);
  a125=(a125+a0);
  a125=(a125+a49);
  a5=(a5+a125);
  a125=1.7124110941795541e+00;
  a134=2.1420999999999994e-04;
  a134=(a134-a29);
  a29=casadi_sq(a134);
  a41=-7.0003829999999975e-02;
  a41=(a41-a65);
  a65=casadi_sq(a41);
  a29=(a29+a65);
  a29=(a125*a29);
  a5=(a5+a29);
  a29=1.0022522000000000e-01;
  a0=(a49-a0);
  a29=(a29+a0);
  a0=-1.6804015999999994e-01;
  a0=(a0-a46);
  a46=casadi_sq(a0);
  a65=casadi_sq(a134);
  a46=(a46+a65);
  a46=(a125*a46);
  a29=(a29+a46);
  a5=(a5-a29);
  a46=1.1364055000000001e-01;
  a126=(a126+a49);
  a46=(a46+a126);
  a126=casadi_sq(a0);
  a65=casadi_sq(a41);
  a126=(a126+a65);
  a126=(a125*a126);
  a46=(a46+a126);
  a46=(a46-a29);
  a126=(a5+a46);
  a5=(a17*a5);
  a65=-1.1817000000000000e-04;
  a55=(a55*a24);
  a136=(a136*a25);
  a55=(a55+a136);
  a136=(a22*a127);
  a67=(a23*a132);
  a136=(a136-a67);
  a55=(a55+a136);
  a65=(a65+a55);
  a55=(a125*a0);
  a55=(a55*a134);
  a65=(a65-a55);
  a55=(a18*a65);
  a5=(a5+a55);
  a55=(a5*a17);
  a65=(a17*a65);
  a46=(a18*a46);
  a65=(a65+a46);
  a46=(a65*a18);
  a55=(a55+a46);
  a126=(a126-a55);
  a126=(a126+a29);
  a57=(a57+a126);
  a126=3.2700619797990886e+00;
  a46=-2.3609999999999999e-05;
  a46=(a46-a59);
  a59=casadi_sq(a46);
  a136=4.7625540000000001e-02;
  a136=(a136-a20);
  a20=casadi_sq(a136);
  a59=(a59+a20);
  a59=(a126*a59);
  a57=(a57+a59);
  a59=7.0024118999999996e-01;
  a59=(a59+a29);
  a20=-2.9477535999999999e-01;
  a20=(a20-a51);
  a51=casadi_sq(a20);
  a67=casadi_sq(a46);
  a51=(a51+a67);
  a51=(a126*a51);
  a59=(a59+a51);
  a57=(a57-a59);
  a57=(a2*a57);
  a12=(a10*a12);
  a11=(a11+a12);
  a12=-4.6930000000000000e-05;
  a5=(a5*a18);
  a65=(a65*a17);
  a5=(a5-a65);
  a12=(a12+a5);
  a5=(a126*a20);
  a5=(a5*a46);
  a12=(a12-a5);
  a5=(a11*a12);
  a57=(a57+a5);
  a57=(a57*a2);
  a12=(a2*a12);
  a5=7.2293306000000002e-01;
  a55=(a55+a29);
  a5=(a5+a55);
  a55=casadi_sq(a20);
  a65=casadi_sq(a136);
  a55=(a55+a65);
  a55=(a126*a55);
  a5=(a5+a55);
  a5=(a5-a59);
  a5=(a11*a5);
  a12=(a12+a5);
  a12=(a12*a11);
  a57=(a57+a12);
  a57=(a57+a59);
  a9=(a9+a57);
  a57=6.1441004554871421e+00;
  a12=-2.3749000000000000e-04;
  a12=(a12-a56);
  a12=casadi_sq(a12);
  a56=-4.3103130000000003e-02;
  a56=(a56-a68);
  a56=casadi_sq(a56);
  a12=(a12+a56);
  a57=(a57*a12);
  a9=(a9+a57);
  a3=(a3+a9);
  a57=1.7041303843650734e+01;
  a12=-1.3304873704113908e-02;
  a12=(a12-a73);
  a12=casadi_sq(a12);
  a73=-6.3511281010255423e-04;
  a73=(a73-a60);
  a73=casadi_sq(a73);
  a12=(a12+a73);
  a57=(a57*a12);
  a3=(a3+a57);
  a71=casadi_sq(a71);
  a4=casadi_sq(a4);
  a71=(a71+a4);
  a70=(a70*a71);
  a3=(a3+a70);
  if (res[0]!=0) res[0][20]=a3;
  a8=casadi_sq(a8);
  a15=casadi_sq(a15);
  a8=(a8+a15);
  a76=(a76*a8);
  a9=(a9+a76);
  a74=(a72*a74);
  a74=(a9+a74);
  if (res[0]!=0) res[0][21]=a74;
  a76=-5.2403459999999999e-02;
  a8=3.3418820000000002e-02;
  a15=(a131*a24);
  a3=(a128*a25);
  a15=(a15+a3);
  a3=(a23*a127);
  a70=(a22*a132);
  a3=(a3+a70);
  a15=(a15+a3);
  a8=(a8+a15);
  a0=(a125*a0);
  a0=(a0*a41);
  a8=(a8-a0);
  a133=(a133*a8);
  a0=(a133*a18);
  a15=-4.3710000000000000e-05;
  a131=(a131*a23);
  a128=(a128*a22);
  a131=(a131-a128);
  a128=(a25*a132);
  a3=(a24*a127);
  a128=(a128-a3);
  a131=(a131+a128);
  a15=(a15+a131);
  a125=(a125*a134);
  a125=(a125*a41);
  a15=(a15-a125);
  a125=(a15+a15);
  a41=(a125*a17);
  a0=(a0-a41);
  a41=(a18*a8);
  a134=(a17*a15);
  a41=(a41-a134);
  a0=(a0-a41);
  a76=(a76+a0);
  a20=(a126*a20);
  a20=(a20*a136);
  a76=(a76-a20);
  a20=(a80*a14);
  a20=(a20*a61);
  a76=(a76-a20);
  a76=(a2*a76);
  a20=1.7600000000000001e-06;
  a133=(a133*a17);
  a125=(a125*a18);
  a133=(a133+a125);
  a125=(a18*a15);
  a0=(a17*a8);
  a125=(a125+a0);
  a133=(a133-a125);
  a20=(a20+a133);
  a126=(a126*a46);
  a126=(a126*a136);
  a20=(a20-a126);
  a126=(a80*a58);
  a126=(a126*a61);
  a20=(a20-a126);
  a20=(a11*a20);
  a76=(a76+a20);
  a79=(a66*a79);
  a76=(a76-a79);
  a83=(a72*a83);
  a83=(a76+a83);
  if (res[0]!=0) res[0][22]=a83;
  a79=(a85*a19);
  a79=(a79*a63);
  a8=(a8-a79);
  a79=(a18*a8);
  a20=(a85*a27);
  a20=(a20*a63);
  a15=(a15-a20);
  a20=(a17*a15);
  a79=(a79-a20);
  a20=(a62*a77);
  a79=(a79-a20);
  a79=(a2*a79);
  a8=(a17*a8);
  a15=(a18*a15);
  a8=(a8+a15);
  a84=(a62*a84);
  a8=(a8+a84);
  a8=(a11*a8);
  a79=(a79+a8);
  a87=(a66*a87);
  a79=(a79-a87);
  a90=(a72*a90);
  a90=(a79+a90);
  if (res[0]!=0) res[0][23]=a90;
  a87=(a92*a26);
  a87=(a87*a52);
  a132=(a132-a87);
  a87=(a24*a132);
  a8=(a92*a32);
  a8=(a8*a52);
  a127=(a127-a8);
  a8=(a25*a127);
  a26=casadi_sq(a26);
  a32=casadi_sq(a32);
  a26=(a26+a32);
  a92=(a92*a26);
  a49=(a49+a92);
  a8=(a8-a49);
  a87=(a87+a8);
  a8=(a64*a95);
  a87=(a87-a8);
  a8=(a18*a87);
  a92=(a23*a132);
  a26=(a10*a49);
  a32=(a22*a127);
  a26=(a26-a32);
  a92=(a92+a26);
  a91=(a64*a91);
  a26=(a21*a88);
  a91=(a91+a26);
  a92=(a92+a91);
  a91=(a17*a92);
  a8=(a8-a91);
  a91=(a62*a97);
  a8=(a8-a91);
  a8=(a2*a8);
  a87=(a17*a87);
  a92=(a18*a92);
  a87=(a87+a92);
  a94=(a62*a94);
  a88=(a16*a88);
  a94=(a94+a88);
  a87=(a87+a94);
  a87=(a11*a87);
  a8=(a8+a87);
  a96=(a66*a96);
  a8=(a8-a96);
  a100=(a72*a100);
  a100=(a8+a100);
  if (res[0]!=0) res[0][24]=a100;
  a96=(a102*a31);
  a96=(a96*a47);
  a135=(a135-a96);
  a96=(a30*a135);
  a87=(a102*a43);
  a87=(a87*a47);
  a137=(a137-a87);
  a87=(a42*a137);
  a31=casadi_sq(a31);
  a43=casadi_sq(a43);
  a31=(a31+a43);
  a102=(a102*a31);
  a44=(a44+a102);
  a102=(a10*a44);
  a87=(a87+a102);
  a96=(a96+a87);
  a93=(a34*a93);
  a105=(a53*a105);
  a93=(a93-a105);
  a96=(a96+a93);
  a93=(a24*a96);
  a105=(a39*a135);
  a87=(a50*a137);
  a87=(a87-a44);
  a105=(a105+a87);
  a87=(a53*a101);
  a105=(a105+a87);
  a87=(a25*a105);
  a135=(a54*a135);
  a137=(a28*a137);
  a135=(a135+a137);
  a101=(a34*a101);
  a135=(a135-a101);
  a87=(a87-a135);
  a93=(a93+a87);
  a87=(a64*a103);
  a93=(a93-a87);
  a87=(a18*a93);
  a101=(a23*a96);
  a137=(a10*a135);
  a102=(a22*a105);
  a137=(a137-a102);
  a101=(a101+a137);
  a104=(a64*a104);
  a137=(a21*a110);
  a104=(a104-a137);
  a101=(a101+a104);
  a104=(a17*a101);
  a87=(a87-a104);
  a104=(a62*a108);
  a87=(a87-a104);
  a87=(a2*a87);
  a93=(a17*a93);
  a101=(a18*a101);
  a93=(a93+a101);
  a106=(a62*a106);
  a110=(a16*a110);
  a106=(a106-a110);
  a93=(a93+a106);
  a93=(a11*a93);
  a87=(a87+a93);
  a107=(a66*a107);
  a87=(a87-a107);
  a113=(a72*a113);
  a113=(a87+a113);
  if (res[0]!=0) res[0][25]=a113;
  a107=-1.2964039586562383e-07;
  a37=(a107*a37);
  a93=4.0386133176874770e-07;
  a40=(a93*a40);
  a106=-5.8427545519626933e-04;
  a40=(a40+a106);
  a37=(a37+a40);
  a40=(a48*a117);
  a37=(a37-a40);
  a30=(a30*a37);
  a40=(a107*a36);
  a106=1.2973521261644846e-19;
  a110=(a93*a35);
  a110=(a106-a110);
  a40=(a40+a110);
  a48=(a48*a114);
  a111=(a33*a111);
  a48=(a48+a111);
  a40=(a40+a48);
  a42=(a42*a40);
  a93=(a93*a36);
  a106=(a106-a93);
  a107=(a107*a35);
  a106=(a106-a107);
  a33=(a33*a117);
  a106=(a106+a33);
  a33=(a10*a106);
  a42=(a42+a33);
  a30=(a30+a42);
  a120=(a34*a120);
  a115=(a53*a115);
  a120=(a120-a115);
  a30=(a30+a120);
  a24=(a24*a30);
  a39=(a39*a37);
  a50=(a50*a40);
  a50=(a50-a106);
  a39=(a39+a50);
  a53=(a53*a116);
  a39=(a39+a53);
  a25=(a25*a39);
  a54=(a54*a37);
  a28=(a28*a40);
  a54=(a54+a28);
  a34=(a34*a116);
  a54=(a54-a34);
  a25=(a25-a54);
  a24=(a24+a25);
  a25=(a64*a121);
  a24=(a24-a25);
  a25=(a18*a24);
  a34=(a23*a30);
  a116=(a10*a54);
  a28=(a22*a39);
  a116=(a116-a28);
  a34=(a34+a116);
  a64=(a64*a118);
  a118=(a21*a123);
  a64=(a64-a118);
  a34=(a34+a64);
  a64=(a17*a34);
  a25=(a25-a64);
  a64=(a62*a122);
  a25=(a25-a64);
  a2=(a2*a25);
  a17=(a17*a24);
  a18=(a18*a34);
  a17=(a17+a18);
  a62=(a62*a119);
  a123=(a16*a123);
  a62=(a62-a123);
  a17=(a17+a62);
  a11=(a11*a17);
  a2=(a2+a11);
  a66=(a66*a13);
  a2=(a2-a66);
  a72=(a72*a7);
  a72=(a2+a72);
  if (res[0]!=0) res[0][26]=a72;
  if (res[0]!=0) res[0][27]=a78;
  if (res[0]!=0) res[0][28]=a75;
  if (res[0]!=0) res[0][29]=a74;
  if (res[0]!=0) res[0][30]=a9;
  if (res[0]!=0) res[0][31]=a76;
  if (res[0]!=0) res[0][32]=a79;
  if (res[0]!=0) res[0][33]=a8;
  if (res[0]!=0) res[0][34]=a87;
  if (res[0]!=0) res[0][35]=a2;
  if (res[0]!=0) res[0][36]=a81;
  if (res[0]!=0) res[0][37]=a82;
  if (res[0]!=0) res[0][38]=a83;
  if (res[0]!=0) res[0][39]=a76;
  a14=casadi_sq(a14);
  a58=casadi_sq(a58);
  a14=(a14+a58);
  a80=(a80*a14);
  a59=(a59+a80);
  if (res[0]!=0) res[0][40]=a59;
  a19=casadi_sq(a19);
  a27=casadi_sq(a27);
  a19=(a19+a27);
  a85=(a85*a19);
  a29=(a29+a85);
  a77=(a16*a77);
  a77=(a29+a77);
  if (res[0]!=0) res[0][41]=a77;
  a85=(a10*a49);
  a127=(a23*a127);
  a85=(a85-a127);
  a132=(a22*a132);
  a85=(a85-a132);
  a95=(a21*a95);
  a85=(a85+a95);
  a97=(a16*a97);
  a97=(a85+a97);
  if (res[0]!=0) res[0][42]=a97;
  a95=(a10*a135);
  a105=(a23*a105);
  a95=(a95-a105);
  a96=(a22*a96);
  a95=(a95-a96);
  a103=(a21*a103);
  a95=(a95+a103);
  a108=(a16*a108);
  a108=(a95+a108);
  if (res[0]!=0) res[0][43]=a108;
  a10=(a10*a54);
  a23=(a23*a39);
  a10=(a10-a23);
  a22=(a22*a30);
  a10=(a10-a22);
  a21=(a21*a121);
  a10=(a10+a21);
  a16=(a16*a122);
  a16=(a10+a16);
  if (res[0]!=0) res[0][44]=a16;
  if (res[0]!=0) res[0][45]=a89;
  if (res[0]!=0) res[0][46]=a86;
  if (res[0]!=0) res[0][47]=a90;
  if (res[0]!=0) res[0][48]=a79;
  if (res[0]!=0) res[0][49]=a77;
  if (res[0]!=0) res[0][50]=a29;
  if (res[0]!=0) res[0][51]=a85;
  if (res[0]!=0) res[0][52]=a95;
  if (res[0]!=0) res[0][53]=a10;
  if (res[0]!=0) res[0][54]=a99;
  if (res[0]!=0) res[0][55]=a98;
  if (res[0]!=0) res[0][56]=a100;
  if (res[0]!=0) res[0][57]=a8;
  if (res[0]!=0) res[0][58]=a97;
  if (res[0]!=0) res[0][59]=a85;
  if (res[0]!=0) res[0][60]=a49;
  if (res[0]!=0) res[0][61]=a135;
  if (res[0]!=0) res[0][62]=a54;
  if (res[0]!=0) res[0][63]=a112;
  if (res[0]!=0) res[0][64]=a109;
  if (res[0]!=0) res[0][65]=a113;
  if (res[0]!=0) res[0][66]=a87;
  if (res[0]!=0) res[0][67]=a108;
  if (res[0]!=0) res[0][68]=a95;
  if (res[0]!=0) res[0][69]=a135;
  if (res[0]!=0) res[0][70]=a44;
  if (res[0]!=0) res[0][71]=a106;
  if (res[0]!=0) res[0][72]=a69;
  if (res[0]!=0) res[0][73]=a1;
  if (res[0]!=0) res[0][74]=a72;
  if (res[0]!=0) res[0][75]=a2;
  if (res[0]!=0) res[0][76]=a16;
  if (res[0]!=0) res[0][77]=a10;
  if (res[0]!=0) res[0][78]=a54;
  if (res[0]!=0) res[0][79]=a106;
  a106=5.8427545519626933e-04;
  if (res[0]!=0) res[0][80]=a106;
  return 0;
}

CASADI_SYMBOL_EXPORT int M(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int M_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int M_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void M_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int M_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void M_release(int mem) {
}

CASADI_SYMBOL_EXPORT void M_incref(void) {
}

CASADI_SYMBOL_EXPORT void M_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int M_n_in(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_int M_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real M_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* M_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* M_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* M_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* M_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int M_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
