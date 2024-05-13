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
  #define CASADI_PREFIX(ID) indy7_id_ ## ID
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

static const casadi_int casadi_s0[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};

/* rnea:(q[6],dq[6],ddq[6])->(tau[6]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a11, a110, a111, a112, a113, a114, a115, a116, a117, a118, a119, a12, a120, a121, a122, a123, a124, a125, a126, a127, a128, a129, a13, a130, a131, a132, a133, a134, a135, a136, a137, a138, a139, a14, a140, a141, a142, a143, a144, a145, a146, a147, a148, a149, a15, a150, a151, a152, a153, a154, a155, a156, a157, a158, a159, a16, a160, a161, a162, a163, a164, a165, a166, a167, a168, a169, a17, a170, a171, a172, a173, a174, a175, a176, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a8, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a9, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99;
  a0=6.0031470000000003e-02;
  a1=arg[2]? arg[2][0] : 0;
  a0=(a0*a1);
  a2=-2.3749000000000000e-04;
  a3=1.1444445350000001e+01;
  a4=(a2*a1);
  a4=(a3*a4);
  a2=(a2*a4);
  a4=-4.3103130000000003e-02;
  a5=(a4*a1);
  a3=(a3*a5);
  a4=(a4*a3);
  a2=(a2+a4);
  a0=(a0+a2);
  a2=2.2204460492503131e-16;
  a4=arg[0]? arg[0][1] : 0;
  a3=sin(a4);
  a5=(a2*a3);
  a4=cos(a4);
  a5=(a5-a4);
  a6=3.5997429999999997e-02;
  a7=arg[1]? arg[1][1] : 0;
  a8=(a2*a4);
  a8=(a3+a8);
  a9=arg[1]? arg[1][0] : 0;
  a10=(a8*a9);
  a11=(a7*a10);
  a12=(a5*a1);
  a11=(a11+a12);
  a12=(a6*a11);
  a13=-4.6930000000000000e-05;
  a14=(a8*a1);
  a15=(a5*a9);
  a16=(a7*a15);
  a14=(a14-a16);
  a16=(a13*a14);
  a12=(a12+a16);
  a16=-5.2403459999999999e-02;
  a17=arg[2]? arg[2][1] : 0;
  a18=(a16*a17);
  a12=(a12+a18);
  a18=-2.3609999999999999e-05;
  a19=5.8476655299999996e+00;
  a20=-1.0900000000000000e-01;
  a1=(a20*a1);
  a21=(a2*a1);
  a22=-2.9477535999999999e-01;
  a23=(a22*a14);
  a24=(a18*a11);
  a23=(a23-a24);
  a23=(a21+a23);
  a23=(a19*a23);
  a24=(a18*a23);
  a25=4.7625540000000001e-02;
  a26=(a2*a4);
  a26=(a26+a3);
  a9=(a20*a9);
  a27=(a26*a9);
  a28=(a7*a27);
  a29=9.8100000000000005e+00;
  a30=(a29*a8);
  a3=(a2*a3);
  a4=(a4-a3);
  a3=(a4*a1);
  a30=(a30-a3);
  a28=(a28+a30);
  a30=(a25*a11);
  a3=(a22*a17);
  a30=(a30-a3);
  a30=(a28-a30);
  a30=(a19*a30);
  a3=(a25*a30);
  a24=(a24+a3);
  a12=(a12-a24);
  a24=(a16*a15);
  a3=1.7600000000000001e-06;
  a31=(a3*a10);
  a24=(a24+a31);
  a31=7.0024118999999996e-01;
  a32=(a31*a7);
  a24=(a24+a32);
  a32=(a18*a7);
  a33=(a25*a10);
  a32=(a32-a33);
  a32=(a27+a32);
  a32=(a19*a32);
  a33=(a18*a32);
  a34=(a4*a9);
  a35=(a25*a15);
  a36=(a22*a7);
  a35=(a35-a36);
  a35=(a34+a35);
  a35=(a19*a35);
  a36=(a22*a35);
  a33=(a33-a36);
  a24=(a24+a33);
  a33=(a10*a24);
  a36=(a13*a15);
  a37=7.2293306000000002e-01;
  a38=(a37*a10);
  a36=(a36+a38);
  a38=(a3*a7);
  a36=(a36+a38);
  a9=(a2*a9);
  a38=(a22*a10);
  a39=(a18*a15);
  a38=(a38-a39);
  a38=(a9+a38);
  a38=(a19*a38);
  a39=(a22*a38);
  a40=(a25*a32);
  a39=(a39-a40);
  a36=(a36+a39);
  a39=(a7*a36);
  a33=(a33-a39);
  a39=(a34*a38);
  a40=(a9*a35);
  a39=(a39-a40);
  a33=(a33+a39);
  a12=(a12+a33);
  a33=arg[0]? arg[0][2] : 0;
  a39=cos(a33);
  a40=1.6172099999999995e-02;
  a41=arg[1]? arg[1][2] : 0;
  a42=(a39*a10);
  a33=sin(a33);
  a43=(a33*a15);
  a42=(a42-a43);
  a43=(a41*a42);
  a44=(a39*a11);
  a45=(a33*a14);
  a44=(a44+a45);
  a43=(a43+a44);
  a44=(a40*a43);
  a45=-1.1817000000000000e-04;
  a46=(a39*a14);
  a47=(a33*a11);
  a46=(a46-a47);
  a47=(a39*a15);
  a48=(a33*a10);
  a47=(a47+a48);
  a48=(a41*a47);
  a46=(a46-a48);
  a48=(a45*a46);
  a44=(a44+a48);
  a48=3.3418820000000002e-02;
  a49=arg[2]? arg[2][2] : 0;
  a49=(a49+a17);
  a50=(a48*a49);
  a44=(a44+a50);
  a50=2.1420999999999994e-04;
  a51=2.6820606400000000e+00;
  a52=-4.5000000000000001e-01;
  a53=(a52*a14);
  a21=(a21+a53);
  a53=-1.6804015999999994e-01;
  a54=(a53*a46);
  a55=(a50*a43);
  a54=(a54-a55);
  a54=(a21+a54);
  a54=(a51*a54);
  a55=(a50*a54);
  a56=-7.0003829999999975e-02;
  a57=-3.0499999999999999e-02;
  a58=(a57*a11);
  a59=(a52*a17);
  a58=(a58-a59);
  a28=(a28-a58);
  a58=(a39*a28);
  a29=(a29*a5);
  a1=(a26*a1);
  a29=(a29-a1);
  a1=(a7*a34);
  a29=(a29-a1);
  a1=(a57*a14);
  a1=(a29+a1);
  a59=(a33*a1);
  a58=(a58-a59);
  a59=(a57*a10);
  a59=(a59-a27);
  a60=(a39*a59);
  a61=(a57*a15);
  a62=(a52*a7);
  a61=(a61-a62);
  a61=(a34+a61);
  a62=(a33*a61);
  a60=(a60-a62);
  a62=(a41*a60);
  a58=(a58-a62);
  a62=(a56*a43);
  a63=(a53*a49);
  a62=(a62-a63);
  a62=(a58-a62);
  a62=(a51*a62);
  a63=(a56*a62);
  a55=(a55+a63);
  a44=(a44-a55);
  a55=(a48*a47);
  a63=-4.3710000000000000e-05;
  a64=(a63*a42);
  a55=(a55+a64);
  a64=1.0022522000000000e-01;
  a65=(a41+a7);
  a66=(a64*a65);
  a55=(a55+a66);
  a59=(a33*a59);
  a61=(a39*a61);
  a59=(a59+a61);
  a61=(a56*a47);
  a66=(a53*a65);
  a61=(a61-a66);
  a61=(a59+a61);
  a61=(a51*a61);
  a66=(a53*a61);
  a67=(a50*a65);
  a68=(a56*a42);
  a67=(a67-a68);
  a67=(a60-a67);
  a67=(a51*a67);
  a68=(a50*a67);
  a66=(a66+a68);
  a55=(a55-a66);
  a66=(a42*a55);
  a68=(a45*a47);
  a69=1.1364055000000001e-01;
  a70=(a69*a42);
  a68=(a68+a70);
  a70=(a63*a65);
  a68=(a68+a70);
  a70=(a56*a67);
  a71=(a52*a10);
  a71=(a9+a71);
  a72=(a53*a42);
  a73=(a50*a47);
  a72=(a72-a73);
  a72=(a71+a72);
  a72=(a51*a72);
  a73=(a53*a72);
  a70=(a70+a73);
  a68=(a68+a70);
  a70=(a65*a68);
  a66=(a66-a70);
  a70=(a59*a72);
  a73=(a71*a61);
  a70=(a70-a73);
  a66=(a66+a70);
  a44=(a44+a66);
  a66=arg[0]? arg[0][3] : 0;
  a70=cos(a66);
  a73=(a2*a70);
  a74=-2.2204460492503131e-16;
  a66=sin(a66);
  a75=(a74*a66);
  a73=(a73+a75);
  a75=2.7988909999999999e-02;
  a76=arg[1]? arg[1][3] : 0;
  a77=(a74*a70);
  a78=(a2*a66);
  a77=(a77-a78);
  a78=(a77*a47);
  a79=(a66*a42);
  a80=(a70*a65);
  a79=(a79+a80);
  a78=(a78-a79);
  a79=(a76*a78);
  a80=(a73*a43);
  a81=(a70*a46);
  a82=(a66*a49);
  a81=(a81-a82);
  a80=(a80+a81);
  a79=(a79+a80);
  a80=(a75*a79);
  a81=3.8930000000000002e-05;
  a82=(a77*a43);
  a83=(a66*a46);
  a84=(a70*a49);
  a83=(a83+a84);
  a82=(a82-a83);
  a83=(a73*a47);
  a84=(a70*a42);
  a85=(a66*a65);
  a84=(a84-a85);
  a83=(a83+a84);
  a84=(a76*a83);
  a82=(a82-a84);
  a84=(a81*a82);
  a80=(a80+a84);
  a84=-4.7679999999999998e-05;
  a85=arg[2]? arg[2][3] : 0;
  a86=(a2*a46);
  a87=(a2*a49);
  a86=(a86+a87);
  a86=(a86-a43);
  a85=(a85+a86);
  a86=(a84*a85);
  a80=(a80+a86);
  a86=-7.0984400000000003e-02;
  a87=2.1298737100000000e+00;
  a88=-7.4999999999999997e-02;
  a89=(a88*a43);
  a90=-2.6700000000000002e-01;
  a91=(a90*a49);
  a89=(a89-a91);
  a58=(a58-a89);
  a89=(a2*a58);
  a91=(a90*a46);
  a21=(a21+a91);
  a91=(a2*a21);
  a89=(a89-a91);
  a1=(a39*a1);
  a28=(a33*a28);
  a1=(a1+a28);
  a41=(a41*a59);
  a1=(a1-a41);
  a41=(a88*a46);
  a41=(a1+a41);
  a89=(a89-a41);
  a28=-2.6846999999999998e-04;
  a91=(a28*a82);
  a92=(a86*a79);
  a91=(a91-a92);
  a91=(a89-a91);
  a91=(a87*a91);
  a92=(a86*a91);
  a93=7.6491279999999995e-02;
  a94=(a77*a41);
  a95=(a70*a21);
  a96=(a66*a58);
  a95=(a95-a96);
  a94=(a94+a95);
  a95=(a88*a42);
  a95=(a60+a95);
  a96=(a73*a95);
  a97=(a90*a42);
  a97=(a71+a97);
  a98=(a66*a97);
  a99=(a88*a47);
  a100=(a90*a65);
  a99=(a99-a100);
  a99=(a59+a99);
  a100=(a70*a99);
  a98=(a98-a100);
  a96=(a96+a98);
  a98=(a76*a96);
  a94=(a94-a98);
  a98=(a93*a79);
  a100=(a28*a85);
  a98=(a98-a100);
  a98=(a94-a98);
  a98=(a87*a98);
  a100=(a93*a98);
  a92=(a92-a100);
  a80=(a80+a92);
  a92=(a84*a83);
  a100=-1.2662959999999999e-02;
  a101=(a100*a78);
  a92=(a92+a101);
  a101=1.4962110000000001e-02;
  a102=(a2*a42);
  a103=(a2*a65);
  a102=(a102+a103);
  a102=(a102-a47);
  a102=(a76+a102);
  a103=(a101*a102);
  a92=(a92+a103);
  a103=(a77*a95);
  a104=(a66*a99);
  a105=(a70*a97);
  a104=(a104+a105);
  a103=(a103+a104);
  a104=(a93*a83);
  a105=(a28*a102);
  a104=(a104-a105);
  a104=(a103-a104);
  a104=(a87*a104);
  a105=(a28*a104);
  a106=(a86*a102);
  a107=(a93*a78);
  a106=(a106-a107);
  a106=(a96-a106);
  a106=(a87*a106);
  a107=(a86*a106);
  a105=(a105-a107);
  a92=(a92+a105);
  a105=(a78*a92);
  a107=(a81*a83);
  a108=1.4430760000000001e-02;
  a109=(a108*a78);
  a107=(a107+a109);
  a109=(a100*a102);
  a107=(a107+a109);
  a109=(a93*a106);
  a99=(a2*a99);
  a97=(a2*a97);
  a99=(a99+a97);
  a95=(a95+a99);
  a99=(a28*a78);
  a97=(a86*a83);
  a99=(a99-a97);
  a99=(a95+a99);
  a99=(a87*a99);
  a97=(a28*a99);
  a109=(a109+a97);
  a107=(a107+a109);
  a109=(a102*a107);
  a105=(a105-a109);
  a109=(a95*a104);
  a97=(a103*a99);
  a109=(a109-a97);
  a105=(a105+a109);
  a80=(a80+a105);
  a105=arg[0]? arg[0][4] : 0;
  a109=cos(a105);
  a97=(a2*a109);
  a105=sin(a105);
  a97=(a97+a105);
  a110=1.1052970000000002e-02;
  a111=arg[1]? arg[1][4] : 0;
  a112=(a2*a105);
  a112=(a109-a112);
  a113=(a112*a83);
  a114=(a2*a109);
  a115=(a114*a78);
  a116=(a2*a109);
  a116=(a105+a116);
  a117=(a116*a102);
  a115=(a115+a117);
  a113=(a113+a115);
  a115=(a111*a113);
  a117=(a97*a79);
  a118=(a2*a105);
  a119=(a118*a82);
  a105=(a2*a105);
  a105=(a105-a109);
  a109=(a105*a85);
  a119=(a119+a109);
  a117=(a117+a119);
  a115=(a115+a117);
  a117=(a110*a115);
  a119=5.5170000000000002e-05;
  a109=(a112*a79);
  a120=(a114*a82);
  a121=(a116*a85);
  a120=(a120+a121);
  a109=(a109+a120);
  a120=(a97*a83);
  a121=(a118*a78);
  a122=(a105*a102);
  a121=(a121+a122);
  a120=(a120+a121);
  a121=(a111*a120);
  a109=(a109-a121);
  a121=(a119*a109);
  a117=(a117+a121);
  a121=-1.4819769999999999e-02;
  a122=arg[2]? arg[2][4] : 0;
  a123=(a2*a79);
  a123=(a123-a82);
  a122=(a122+a123);
  a123=(a121*a122);
  a117=(a117+a123);
  a123=-2.3113999999999996e-04;
  a124=2.2241227100000001e+00;
  a76=(a76*a103);
  a41=(a73*a41);
  a58=(a70*a58);
  a21=(a66*a21);
  a58=(a58+a21);
  a41=(a41+a58);
  a76=(a76+a41);
  a41=-1.1400000000000000e-01;
  a58=(a41*a85);
  a21=8.3000000000000004e-02;
  a125=(a21*a82);
  a58=(a58-a125);
  a58=(a76-a58);
  a125=(a2*a58);
  a126=(a21*a79);
  a94=(a94-a126);
  a125=(a125-a94);
  a126=-9.7962319999999978e-02;
  a127=(a126*a109);
  a128=(a123*a115);
  a127=(a127-a128);
  a127=(a125-a127);
  a127=(a124*a127);
  a128=(a123*a127);
  a129=6.4458919999999975e-02;
  a130=(a112*a58);
  a131=(a114*a94);
  a132=(a41*a79);
  a89=(a89+a132);
  a132=(a116*a89);
  a131=(a131+a132);
  a130=(a130+a131);
  a131=(a41*a102);
  a132=(a21*a78);
  a131=(a131-a132);
  a131=(a96-a131);
  a132=(a97*a131);
  a133=(a21*a83);
  a133=(a103-a133);
  a134=(a118*a133);
  a135=(a41*a83);
  a135=(a135-a95);
  a136=(a105*a135);
  a134=(a134+a136);
  a132=(a132+a134);
  a134=(a111*a132);
  a130=(a130-a134);
  a134=(a129*a115);
  a136=(a126*a122);
  a134=(a134-a136);
  a134=(a130-a134);
  a134=(a124*a134);
  a136=(a129*a134);
  a128=(a128-a136);
  a117=(a117+a128);
  a128=(a121*a120);
  a136=-3.7400000000000001e-05;
  a137=(a136*a113);
  a128=(a128+a137);
  a137=2.7547950000000002e-02;
  a138=(a2*a83);
  a138=(a138-a78);
  a138=(a111+a138);
  a139=(a137*a138);
  a128=(a128+a139);
  a139=(a112*a131);
  a140=(a114*a133);
  a135=(a116*a135);
  a140=(a140+a135);
  a139=(a139+a140);
  a140=(a129*a120);
  a135=(a126*a138);
  a140=(a140-a135);
  a140=(a139-a140);
  a140=(a124*a140);
  a135=(a126*a140);
  a141=(a123*a138);
  a142=(a129*a113);
  a141=(a141-a142);
  a141=(a132-a141);
  a141=(a124*a141);
  a142=(a123*a141);
  a135=(a135-a142);
  a128=(a128+a135);
  a135=(a113*a128);
  a142=(a119*a120);
  a143=3.6982910000000001e-02;
  a144=(a143*a113);
  a142=(a142+a144);
  a144=(a136*a138);
  a142=(a142+a144);
  a144=(a129*a141);
  a131=(a2*a131);
  a131=(a131-a133);
  a133=(a126*a113);
  a145=(a123*a120);
  a133=(a133-a145);
  a133=(a131-a133);
  a133=(a124*a133);
  a145=(a126*a133);
  a144=(a144-a145);
  a142=(a142+a144);
  a144=(a138*a142);
  a135=(a135-a144);
  a144=(a139*a133);
  a145=(a131*a140);
  a144=(a144-a145);
  a135=(a135+a144);
  a117=(a117+a135);
  a135=arg[0]? arg[0][5] : 0;
  a144=cos(a135);
  a145=(a2*a144);
  a135=sin(a135);
  a146=(a74*a135);
  a145=(a145+a146);
  a146=7.9085968435064309e-04;
  a147=arg[1]? arg[1][5] : 0;
  a74=(a74*a144);
  a148=(a2*a135);
  a74=(a74-a148);
  a148=(a74*a120);
  a149=(a135*a113);
  a150=(a144*a138);
  a149=(a149+a150);
  a148=(a148-a149);
  a149=(a147*a148);
  a150=(a145*a115);
  a151=(a144*a109);
  a152=(a135*a122);
  a151=(a151-a152);
  a150=(a150+a151);
  a149=(a149+a150);
  a150=(a146*a149);
  a151=-3.3999996207092593e-07;
  a152=(a74*a115);
  a153=(a135*a109);
  a154=(a144*a122);
  a153=(a153+a154);
  a152=(a152-a153);
  a153=(a145*a120);
  a154=(a144*a113);
  a155=(a135*a138);
  a154=(a154-a155);
  a153=(a153+a154);
  a154=(a147*a153);
  a152=(a152-a154);
  a154=(a151*a152);
  a150=(a150+a154);
  a154=8.3001622951724945e-07;
  a155=arg[2]? arg[2][5] : 0;
  a156=(a2*a109);
  a157=(a2*a122);
  a156=(a156+a157);
  a156=(a156-a115);
  a155=(a155+a156);
  a156=(a154*a155);
  a150=(a150+a156);
  a156=-4.6555878300977510e-04;
  a157=3.8255032000000000e-01;
  a158=6.9000000000000006e-02;
  a159=(a158*a115);
  a160=-1.6800000000000001e-01;
  a161=(a160*a122);
  a159=(a159-a161);
  a130=(a130-a159);
  a159=(a2*a130);
  a161=(a160*a109);
  a125=(a125-a161);
  a161=(a2*a125);
  a159=(a159+a161);
  a111=(a111*a139);
  a58=(a97*a58);
  a94=(a118*a94);
  a89=(a105*a89);
  a94=(a94+a89);
  a58=(a58+a94);
  a111=(a111+a58);
  a58=(a158*a109);
  a58=(a111+a58);
  a159=(a159-a58);
  a94=8.1469787034552752e-05;
  a89=(a94*a152);
  a161=(a156*a149);
  a89=(a89-a161);
  a159=(a159-a89);
  a159=(a157*a159);
  a89=(a156*a159);
  a161=3.0791490739415412e-02;
  a162=(a74*a58);
  a163=(a135*a130);
  a164=(a144*a125);
  a163=(a163+a164);
  a162=(a162-a163);
  a163=(a158*a113);
  a163=(a132+a163);
  a164=(a145*a163);
  a165=(a158*a120);
  a166=(a160*a138);
  a165=(a165-a166);
  a165=(a139-a165);
  a166=(a144*a165);
  a167=(a160*a113);
  a167=(a131-a167);
  a168=(a135*a167);
  a166=(a166-a168);
  a164=(a164+a166);
  a166=(a147*a164);
  a162=(a162-a166);
  a166=(a161*a149);
  a168=(a94*a155);
  a166=(a166-a168);
  a162=(a162-a166);
  a162=(a157*a162);
  a166=(a161*a162);
  a89=(a89-a166);
  a150=(a150+a89);
  a89=(a154*a153);
  a166=-5.0800927435135708e-06;
  a168=(a166*a148);
  a89=(a89+a168);
  a168=5.8419000022338300e-04;
  a169=(a2*a113);
  a170=(a2*a138);
  a169=(a169+a170);
  a169=(a169-a120);
  a169=(a147+a169);
  a170=(a168*a169);
  a89=(a89+a170);
  a170=(a74*a163);
  a171=(a135*a165);
  a172=(a144*a167);
  a171=(a171+a172);
  a170=(a170-a171);
  a171=(a161*a153);
  a172=(a94*a169);
  a171=(a171-a172);
  a171=(a170-a171);
  a171=(a157*a171);
  a172=(a94*a171);
  a173=(a156*a169);
  a174=(a161*a148);
  a173=(a173-a174);
  a173=(a164-a173);
  a173=(a157*a173);
  a174=(a156*a173);
  a172=(a172-a174);
  a89=(a89+a172);
  a172=(a148*a89);
  a174=(a151*a153);
  a175=7.9867968414053486e-04;
  a176=(a175*a148);
  a174=(a174+a176);
  a176=(a166*a169);
  a174=(a174+a176);
  a176=(a161*a173);
  a165=(a2*a165);
  a167=(a2*a167);
  a165=(a165+a167);
  a165=(a165-a163);
  a163=(a94*a148);
  a167=(a156*a153);
  a163=(a163-a167);
  a163=(a165-a163);
  a163=(a157*a163);
  a167=(a94*a163);
  a176=(a176-a167);
  a174=(a174+a176);
  a176=(a169*a174);
  a172=(a172-a176);
  a176=(a170*a163);
  a167=(a165*a171);
  a176=(a176-a167);
  a172=(a172+a176);
  a150=(a150+a172);
  a172=(a145*a150);
  a176=(a151*a149);
  a175=(a175*a152);
  a176=(a176+a175);
  a175=(a166*a155);
  a176=(a176+a175);
  a147=(a147*a170);
  a58=(a145*a58);
  a130=(a144*a130);
  a125=(a135*a125);
  a130=(a130-a125);
  a58=(a58+a130);
  a147=(a147+a58);
  a58=(a156*a155);
  a130=(a161*a152);
  a58=(a58-a130);
  a147=(a147-a58);
  a157=(a157*a147);
  a147=(a161*a157);
  a58=(a94*a159);
  a147=(a147-a58);
  a176=(a176+a147);
  a146=(a146*a153);
  a151=(a151*a148);
  a146=(a146+a151);
  a151=(a154*a169);
  a146=(a146+a151);
  a151=(a156*a163);
  a161=(a161*a171);
  a151=(a151-a161);
  a146=(a146+a151);
  a151=(a169*a146);
  a89=(a153*a89);
  a151=(a151-a89);
  a165=(a165*a173);
  a89=(a164*a163);
  a165=(a165-a89);
  a151=(a151+a165);
  a176=(a176+a151);
  a151=(a74*a176);
  a154=(a154*a149);
  a166=(a166*a152);
  a154=(a154+a166);
  a168=(a168*a155);
  a154=(a154+a168);
  a94=(a94*a162);
  a156=(a156*a157);
  a94=(a94-a156);
  a154=(a154+a94);
  a174=(a153*a174);
  a146=(a148*a146);
  a174=(a174-a146);
  a164=(a164*a171);
  a170=(a170*a173);
  a164=(a164-a170);
  a174=(a174+a164);
  a154=(a154+a174);
  a151=(a151-a154);
  a172=(a172+a151);
  a151=(a148*a163);
  a174=(a169*a171);
  a151=(a151-a174);
  a157=(a157+a151);
  a151=(a144*a157);
  a171=(a153*a171);
  a148=(a148*a173);
  a171=(a171-a148);
  a159=(a159+a171);
  a171=(a2*a159);
  a169=(a169*a173);
  a153=(a153*a163);
  a169=(a169-a153);
  a162=(a162+a169);
  a169=(a135*a162);
  a171=(a171-a169);
  a151=(a151+a171);
  a171=(a158*a151);
  a172=(a172-a171);
  a117=(a117+a172);
  a172=(a97*a117);
  a171=(a119*a115);
  a143=(a143*a109);
  a171=(a171+a143);
  a143=(a136*a122);
  a171=(a171+a143);
  a143=(a123*a122);
  a169=(a129*a109);
  a143=(a143-a169);
  a111=(a111-a143);
  a124=(a124*a111);
  a111=(a129*a124);
  a143=(a126*a127);
  a111=(a111-a143);
  a171=(a171+a111);
  a110=(a110*a120);
  a119=(a119*a113);
  a110=(a110+a119);
  a119=(a121*a138);
  a110=(a110+a119);
  a119=(a123*a133);
  a129=(a129*a140);
  a119=(a119-a129);
  a110=(a110+a119);
  a119=(a138*a110);
  a128=(a120*a128);
  a119=(a119-a128);
  a131=(a131*a141);
  a128=(a132*a133);
  a131=(a131-a128);
  a119=(a119+a131);
  a171=(a171+a119);
  a119=(a144*a150);
  a131=(a2*a154);
  a128=(a135*a176);
  a131=(a131-a128);
  a119=(a119+a131);
  a145=(a145*a157);
  a74=(a74*a162);
  a74=(a74-a159);
  a145=(a145+a74);
  a158=(a158*a145);
  a159=(a2*a159);
  a162=(a144*a162);
  a159=(a159-a162);
  a157=(a135*a157);
  a159=(a159-a157);
  a157=(a160*a159);
  a158=(a158-a157);
  a119=(a119+a158);
  a171=(a171+a119);
  a119=(a112*a171);
  a121=(a121*a115);
  a136=(a136*a109);
  a121=(a121+a136);
  a137=(a137*a122);
  a121=(a121+a137);
  a126=(a126*a134);
  a123=(a123*a124);
  a126=(a126-a123);
  a121=(a121+a126);
  a142=(a120*a142);
  a110=(a113*a110);
  a142=(a142-a110);
  a132=(a132*a140);
  a139=(a139*a141);
  a132=(a132-a139);
  a142=(a142+a132);
  a121=(a121+a142);
  a142=(a2*a154);
  a144=(a144*a176);
  a142=(a142-a144);
  a135=(a135*a150);
  a142=(a142-a135);
  a160=(a160*a151);
  a142=(a142+a160);
  a121=(a121+a142);
  a142=(a2*a121);
  a119=(a119+a142);
  a172=(a172+a119);
  a119=(a113*a133);
  a142=(a138*a140);
  a119=(a119-a142);
  a124=(a124+a119);
  a124=(a124+a145);
  a145=(a105*a124);
  a138=(a138*a141);
  a133=(a120*a133);
  a138=(a138-a133);
  a134=(a134+a138);
  a134=(a134+a151);
  a151=(a116*a134);
  a145=(a145+a151);
  a151=(a41*a145);
  a138=(a118*a124);
  a133=(a114*a134);
  a120=(a120*a140);
  a113=(a113*a141);
  a120=(a120-a113);
  a127=(a127+a120);
  a127=(a127+a159);
  a133=(a133-a127);
  a138=(a138+a133);
  a133=(a21*a138);
  a151=(a151-a133);
  a172=(a172+a151);
  a80=(a80+a172);
  a172=(a73*a80);
  a151=(a81*a79);
  a108=(a108*a82);
  a151=(a151+a108);
  a108=(a100*a85);
  a151=(a151+a108);
  a108=(a86*a85);
  a133=(a93*a82);
  a108=(a108-a133);
  a76=(a76-a108);
  a87=(a87*a76);
  a76=(a93*a87);
  a108=(a28*a91);
  a76=(a76-a108);
  a151=(a151+a76);
  a75=(a75*a83);
  a81=(a81*a78);
  a75=(a75+a81);
  a81=(a84*a102);
  a75=(a75+a81);
  a81=(a86*a99);
  a93=(a93*a104);
  a81=(a81+a93);
  a75=(a75-a81);
  a81=(a102*a75);
  a92=(a83*a92);
  a81=(a81-a92);
  a92=(a96*a99);
  a95=(a95*a106);
  a92=(a92-a95);
  a81=(a81+a92);
  a151=(a151+a81);
  a118=(a118*a117);
  a114=(a114*a171);
  a114=(a114-a121);
  a118=(a118+a114);
  a97=(a97*a124);
  a112=(a112*a134);
  a127=(a2*a127);
  a112=(a112+a127);
  a97=(a97+a112);
  a21=(a21*a97);
  a118=(a118+a21);
  a151=(a151+a118);
  a118=(a77*a151);
  a84=(a84*a79);
  a100=(a100*a82);
  a84=(a84+a100);
  a101=(a101*a85);
  a84=(a84+a101);
  a28=(a28*a98);
  a86=(a86*a87);
  a28=(a28-a86);
  a84=(a84+a28);
  a107=(a83*a107);
  a75=(a78*a75);
  a107=(a107-a75);
  a96=(a96*a104);
  a103=(a103*a106);
  a96=(a96-a103);
  a107=(a107+a96);
  a84=(a84+a107);
  a105=(a105*a117);
  a116=(a116*a171);
  a105=(a105+a116);
  a41=(a41*a97);
  a105=(a105-a41);
  a84=(a84+a105);
  a118=(a118-a84);
  a172=(a172+a118);
  a118=(a78*a99);
  a105=(a102*a104);
  a118=(a118+a105);
  a87=(a87-a118);
  a87=(a87+a97);
  a97=(a70*a87);
  a104=(a83*a104);
  a78=(a78*a106);
  a104=(a104-a78);
  a91=(a91+a104);
  a91=(a91+a145);
  a145=(a2*a91);
  a102=(a102*a106);
  a83=(a83*a99);
  a102=(a102+a83);
  a98=(a98+a102);
  a98=(a98+a138);
  a138=(a66*a98);
  a145=(a145-a138);
  a97=(a97+a145);
  a145=(a88*a97);
  a172=(a172-a145);
  a44=(a44+a172);
  a172=(a39*a44);
  a145=(a45*a43);
  a69=(a69*a46);
  a145=(a145+a69);
  a69=(a63*a49);
  a145=(a145+a69);
  a69=(a50*a49);
  a138=(a56*a46);
  a69=(a69-a138);
  a1=(a1-a69);
  a51=(a51*a1);
  a1=(a56*a51);
  a69=(a53*a54);
  a1=(a1+a69);
  a145=(a145+a1);
  a40=(a40*a47);
  a45=(a45*a42);
  a40=(a40+a45);
  a45=(a48*a65);
  a40=(a40+a45);
  a56=(a56*a61);
  a45=(a50*a72);
  a56=(a56-a45);
  a40=(a40+a56);
  a56=(a65*a40);
  a55=(a47*a55);
  a56=(a56-a55);
  a55=(a60*a72);
  a71=(a71*a67);
  a55=(a55-a71);
  a56=(a56+a55);
  a145=(a145+a56);
  a56=(a70*a80);
  a55=(a2*a84);
  a71=(a66*a151);
  a55=(a55-a71);
  a56=(a56+a55);
  a73=(a73*a87);
  a77=(a77*a98);
  a77=(a77-a91);
  a73=(a73+a77);
  a88=(a88*a73);
  a91=(a2*a91);
  a98=(a70*a98);
  a91=(a91-a98);
  a87=(a66*a87);
  a91=(a91-a87);
  a87=(a90*a91);
  a88=(a88-a87);
  a56=(a56+a88);
  a145=(a145+a56);
  a56=(a33*a145);
  a172=(a172-a56);
  a56=(a65*a61);
  a88=(a42*a72);
  a56=(a56-a88);
  a56=(a51+a56);
  a56=(a56+a73);
  a73=(a33*a56);
  a65=(a65*a67);
  a72=(a47*a72);
  a65=(a65+a72);
  a65=(a62+a65);
  a65=(a65+a97);
  a72=(a39*a65);
  a73=(a73+a72);
  a72=(a57*a73);
  a172=(a172-a72);
  a12=(a12+a172);
  a5=(a5*a12);
  a12=(a13*a11);
  a37=(a37*a14);
  a12=(a12+a37);
  a37=(a3*a17);
  a12=(a12+a37);
  a37=(a18*a17);
  a172=(a25*a14);
  a37=(a37-a172);
  a29=(a29-a37);
  a19=(a19*a29);
  a29=(a25*a19);
  a37=(a22*a23);
  a29=(a29+a37);
  a12=(a12+a29);
  a6=(a6*a15);
  a13=(a13*a10);
  a6=(a6+a13);
  a13=(a16*a7);
  a6=(a6+a13);
  a25=(a25*a35);
  a13=(a18*a38);
  a25=(a25-a13);
  a6=(a6+a25);
  a25=(a7*a6);
  a24=(a15*a24);
  a25=(a25-a24);
  a9=(a9*a32);
  a24=(a27*a38);
  a9=(a9-a24);
  a25=(a25+a9);
  a12=(a12+a25);
  a44=(a33*a44);
  a145=(a39*a145);
  a44=(a44+a145);
  a39=(a39*a56);
  a33=(a33*a65);
  a39=(a39-a33);
  a57=(a57*a39);
  a33=(a47*a61);
  a65=(a42*a67);
  a33=(a33+a65);
  a54=(a54+a33);
  a91=(a91-a54);
  a54=(a52*a91);
  a57=(a57-a54);
  a44=(a44+a57);
  a12=(a12+a44);
  a8=(a8*a12);
  a5=(a5+a8);
  a8=(a7*a35);
  a12=(a10*a38);
  a8=(a8-a12);
  a8=(a19+a8);
  a8=(a8+a39);
  a26=(a26*a8);
  a38=(a15*a38);
  a7=(a7*a32);
  a38=(a38-a7);
  a38=(a30+a38);
  a38=(a38+a73);
  a4=(a4*a38);
  a38=(a10*a32);
  a7=(a15*a35);
  a38=(a38-a7);
  a38=(a38-a23);
  a38=(a38+a91);
  a38=(a2*a38);
  a4=(a4+a38);
  a26=(a26+a4);
  a20=(a20*a26);
  a5=(a5-a20);
  a0=(a0+a5);
  if (res[0]!=0) res[0][0]=a0;
  a16=(a16*a11);
  a3=(a3*a14);
  a16=(a16+a3);
  a31=(a31*a17);
  a16=(a16+a31);
  a22=(a22*a30);
  a18=(a18*a19);
  a22=(a22-a18);
  a16=(a16+a22);
  a15=(a15*a36);
  a10=(a10*a6);
  a15=(a15-a10);
  a27=(a27*a35);
  a34=(a34*a32);
  a27=(a27-a34);
  a15=(a15+a27);
  a16=(a16+a15);
  a48=(a48*a43);
  a63=(a63*a46);
  a48=(a48+a63);
  a64=(a64*a49);
  a48=(a48+a64);
  a53=(a53*a62);
  a50=(a50*a51);
  a53=(a53-a50);
  a48=(a48+a53);
  a47=(a47*a68);
  a42=(a42*a40);
  a47=(a47-a42);
  a59=(a59*a67);
  a60=(a60*a61);
  a59=(a59-a60);
  a47=(a47+a59);
  a48=(a48+a47);
  a2=(a2*a84);
  a70=(a70*a151);
  a2=(a2-a70);
  a66=(a66*a80);
  a2=(a2-a66);
  a90=(a90*a97);
  a2=(a2+a90);
  a48=(a48+a2);
  a52=(a52*a73);
  a52=(a48+a52);
  a16=(a16+a52);
  if (res[0]!=0) res[0][1]=a16;
  if (res[0]!=0) res[0][2]=a48;
  if (res[0]!=0) res[0][3]=a84;
  if (res[0]!=0) res[0][4]=a121;
  if (res[0]!=0) res[0][5]=a154;
  return 0;
}

CASADI_SYMBOL_EXPORT int rnea(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int rnea_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int rnea_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void rnea_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int rnea_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void rnea_release(int mem) {
}

CASADI_SYMBOL_EXPORT void rnea_incref(void) {
}

CASADI_SYMBOL_EXPORT void rnea_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int rnea_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int rnea_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real rnea_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* rnea_name_in(casadi_int i) {
  switch (i) {
    case 0: return "q";
    case 1: return "dq";
    case 2: return "ddq";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* rnea_name_out(casadi_int i) {
  switch (i) {
    case 0: return "tau";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* rnea_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* rnea_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int rnea_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
