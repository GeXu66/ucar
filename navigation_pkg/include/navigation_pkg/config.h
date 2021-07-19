#include "common_include.h"
#include "GridMap.h"
class Config
{
public:
    Config(){}
    void set(const ros::NodeHandle &nh_priv){
        nh_priv.param("inflation_r", inflation_r, 0.2);
        nh_priv.param("inner_r", inner_r, 1.0);
        nh_priv.param("extern_r", extern_r, 1.0);
        nh_priv.param("timeWeight", timeWeight, 7000.0);
        nh_priv.param("accWeight", accWeight, 1.0);
        nh_priv.param("jerkWeight", jerkWeight, 3.0);
        nh_priv.param("maxVel", maxVel, 0.2);
        nh_priv.param("maxAcc", maxAcc, 0.2);
        nh_priv.param("maxIts", maxIts, 23);
        nh_priv.param("eps", eps, 0.02);
        nh_priv.param("loopHz", loopHz, 50.0);
        nh_priv.param("loopT", loopT, 1.0/loopHz);
        nh_priv.param("globalDstX", globalDstX, 0.589); //need to adjust
        nh_priv.param("globalDstY", globalDstY, -1.734); //need to adjust
        nh_priv.param("globalOriX", globalOriX, -1.0);
        nh_priv.param("globalOriY", globalOriY, 0.0);
        nh_priv.param("gaussianKernelSize", gaussianKernelSize, 5);
        nh_priv.param("gaussianKernelMiu", gaussianKernelMiu, 0.0);
        nh_priv.param("gaussianKernelSigma", gaussianKernelSigma, 3.0);
        nh_priv.param("linearKPP", linearKPP, 0.2);
        nh_priv.param("linearKP", linearKP, 1.0);
        nh_priv.param("linearTI", linearTI, 0.0);
        nh_priv.param("linearTD", linearTD, 0.0);
        nh_priv.param("angularKPP", angularKPP, 1.0);
        nh_priv.param("angularKP", angularKP, 1.0);
        nh_priv.param("angularTI", angularTI, 1.0);
        nh_priv.param("angularTD", angularTD, 0.0);
        nh_priv.param("angularIDecay", angularIDecay, 0.95);
        nh_priv.param("globalSttX", globalSttX, 0.0);
        nh_priv.param("globalSttY", globalSttY, 0.0);
        nh_priv.param("globalSttOri", globalSttOri, 0.0);
        nh_priv.param("aheadTime", aheadTime, 0.5);
    }
    double inner_r;
    double extern_r;
    double inflation_r;
    double timeWeight;
    double accWeight;
    double jerkWeight;
    double maxVel;
    double maxAcc;
    int maxIts;
    double eps;
    double loopHz;
    double velRange;
    double velAccRange;
    double velKP;
    double velKD;
    double velKI;
    double globalDstX;
    double globalDstY;
    double globalOriX;
    double globalOriY;
    int gaussianKernelSize;
    double gaussianKernelMiu;
    double gaussianKernelSigma;
    double linearKPP;
    double linearKP;
    double linearTI;
    double linearTD;
    double angularKP;
    double angularTI;
    double angularTD;
    double angularIDecay;
    double globalSttX;
    double globalSttY;
    double globalSttOri;
    double aheadTime;
    double loopT;
    double angularKPP;
};
