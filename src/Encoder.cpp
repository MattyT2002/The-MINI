#include "Encoder.h"
#include "mbed.h"
using namespace mbed;



void Encoder::countPulseA(){
    _EncCountA++;
    if(_EncCountA%6 == 0){
        if(_EncCountA%100 == 0){
            _ShaftRevA++;
        }
    }
}

void Encoder::countPulseB(){
    _EncCountB++;
    if(_EncCountB%6 == 0){
        if(_EncCountB%100 == 0){
            _ShaftRevB++;
        }
    }
}

void Encoder::reset(){
    _EncCountA = 0;
    _EncCountB = 0;
    _ShaftRevA = 0;
    _ShaftRevB = 0;
}
