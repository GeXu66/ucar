//
// Created by stdcat on 3/14/21.
//

#ifndef SRC_PID_H
#define SRC_PID_H

class PID_t
{
public:
    double KP;
    double KI;
    double KD;
    double fdb;
    double ref;
    double cur_error;
    double error[2];
    double output;
    double outputMax;
    PID_t(){};
    PID_t(double KP, double KT, double KD, double outputMax){
        this->KP = KP;
        this->KD = KD;
        this->KI = KI;
        this->outputMax = outputMax;
    }
    void PID_Calc(){
        cur_error = ref - fdb;
        output += KP * (cur_error - error[1])  + KI * cur_error + KD * (cur_error - 2 * error[1] + error[0]);
        error[0] = error[1];
        error[1] = ref - fdb;
        /*set ouput limit*/
        if(output > outputMax) output = outputMax;
        if(output < -outputMax) output = -outputMax;
}
};



#endif //SRC_PID_H
