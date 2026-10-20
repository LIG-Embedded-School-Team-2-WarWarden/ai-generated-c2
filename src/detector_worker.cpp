// Private stdin/stdout process bridge, never a device/network wire protocol.
#include "target_detector.h"
#include <cstdint>
#include <bit>
#include <iostream>
#include <iomanip>
#include <limits>
#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif
bool read32(std::uint32_t& out){unsigned char b[4];if(!std::cin.read(reinterpret_cast<char*>(b),4))return false;out=0;for(int i=0;i<4;++i)out|=std::uint32_t(b[i])<<(8*i);return true;}
int main(){
#ifdef _WIN32
    _setmode(_fileno(stdin),_O_BINARY);
#endif
    Watch::Detector detector;
    std::uint32_t width,height;
    while(read32(width)){
        if(!read32(height)||!width||!height||std::uint64_t(width)*height>500000)return 2;
        std::vector<Watch::Pixel> pixels(std::size_t(width)*height);
        for(auto& p:pixels){std::uint32_t depth,valid,x,y,z;if(!read32(depth)||!read32(x)||!read32(y)||!read32(z)||!read32(valid))return 2;
            p.depth=valid?depth:0;p.x=valid?std::bit_cast<float>(x):0;p.y=valid?std::bit_cast<float>(y):0;p.z=valid?std::bit_cast<float>(z):0;}
        try{const auto result=detector.process(width,height,pixels);
            std::cout<<std::setprecision(9)<<'[';bool first=true;
            for(const auto& t:result.targets){if(!first)std::cout<<',';first=false;
                std::cout<<"{\"x\":"<<t.x<<",\"y\":"<<t.y<<",\"z\":"<<t.z<<",\"count\":"<<t.count<<'}';}
            std::cout<<"]\n"<<std::flush;
        }catch(...){std::cout<<"null\n"<<std::flush;}
    }
}
