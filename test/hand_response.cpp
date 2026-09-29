#include <HandResponse.h>
#include <fstream>
#include <iostream>
#include <limits>

void require(bool ok,const char *message) {if(!ok) throw std::runtime_error(message);}
int main(int argc,char **argv) try {
    auto settings=defaultHandResponse(); validateHandResponse(settings);
    std::array<double,28> a{};
    for(int i=0;i<28;++i) a[i]=-110.125+i*9.73;
    require(applyHandResponse(a,settings)==a,"Defaults changed the pose/IMU");
    // Official negative-range balance reversal, both segments, and extrapolation.
    HandResponseCurve c{-60,0,.25,0};
    require(evaluateHandResponse(-60,-60,0,c)==-60,"Low endpoint");
    require(evaluateHandResponse(-30,-60,0,c)==-15,"Negative balance reversal");
    require(evaluateHandResponse(-45,-60,0,c)==-37.5,"First segment");
    require(evaluateHandResponse(-15,-60,0,c)==-7.5,"Second segment");
    require(evaluateHandResponse(-90,-60,0,c)==-105,"Extrapolation was clamped");
    require(evaluateHandResponse(30,-60,0,c)==15,"High extrapolation");
    c={-5,35,.25,3};
    require(evaluateHandResponse(10,0,20,c)==8,"Positive-range balance/offset");
    settings[11].offset=7; settings[0].offset=-4;
    const auto result=applyHandResponse(a,settings);
    require(result[11]==a[11]+7 && result[0]==a[0]-4 && result[24]==a[24],"Offsets/IMU");
    for(int kind=0;kind<5;++kind) {
        auto bad=defaultHandResponse();
        if(kind==0) bad[1].balance=std::numeric_limits<double>::quiet_NaN();
        if(kind==1) bad[1].balance=1.1;
        if(kind==2) bad[1].outputMin=20;
        if(kind==3) bad[1].offset=46;
        if(kind==4) bad[11].balance=.2;
        bool rejected=false;try{validateHandResponse(bad);}catch(const std::invalid_argument &){rejected=true;}
        require(rejected,"Invalid curve accepted");
    }
    if(argc>1) {
        std::ifstream in(argv[1]);require(bool(in),"Missing official curve fixtures");
        double lo,hi,omin,omax,balance,input,expected;std::size_t count=0;
        while(in>>lo>>hi>>omin>>omax>>balance>>input>>expected) {
            const auto actual=evaluateHandResponse(input,lo,hi,{omin,omax,balance,0});
            require(std::abs(actual-expected)<.0002,"Official response curve differs");++count;
        }
        require(count>100,"Too few official fixtures");std::cout<<count<<" official response samples matched\n";
    }
    std::cout<<"Response curve endpoints, midpoint, extrapolation, offsets and validation passed\n";
} catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
