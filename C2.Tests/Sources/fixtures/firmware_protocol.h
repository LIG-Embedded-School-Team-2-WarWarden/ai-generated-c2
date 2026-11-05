#ifndef ICD_PROTOCOL_H
#define ICD_PROTOCOL_H
#include <cstdint>
#include <string>
#include <vector>
namespace icd {
// uint16: source asset (4 bits), destination asset (4 bits), message number (8 bits).
// RDR keeps its name; only its wire asset code is D.
enum AssetCode { ECS=0xE, FCS=0xF, RDR=0xD };
inline unsigned sourceAsset(unsigned id) { return (id>>12)&0xF; }
inline unsigned destinationAsset(unsigned id) { return (id>>8)&0xF; }
inline unsigned messageNumber(unsigned id) { return id&0xFF; }
enum MessageId : uint16_t { EF01=0xEF01, FE01=0xFE01, EF02=0xEF02, FE02=0xFE02,
    EF03=0xEF03, FE03=0xFE03, EF04=0xEF04, FE04=0xFE04, EF05=0xEF05, FE05=0xFE05 };
inline int responseId(int id) {
    if(id<0 || id>65535 || sourceAsset(id)!=ECS || destinationAsset(id)!=FCS) return 0;
    switch(id) {
    case EF01: return FE01; case EF02: return FE02; case EF03: return FE03;
    case EF04: return FE04; case EF05: return FE05; default: return 0;
    }
}
enum Result { ACCEPTED=0, BUSY=1, INVALID=2, DEVICE_ERROR=3 };
enum Flag { CONNECTED=1, MOVING=2, LASER=4, DEMO=8, FAULT=16 };
inline unsigned word(const std::string& s, size_t n) {
    return (static_cast<unsigned char>(s.at(n))<<8) | static_cast<unsigned char>(s.at(n+1));
}
inline int signedWord(const std::string& s, size_t n) {
    unsigned v=word(s,n); return v>=32768 ? int(v)-65536 : int(v);
}
inline void append(std::string& s, unsigned v) { s+=char((v>>8)&255); s+=char(v&255); }
inline std::string packet(int id, unsigned tx, unsigned rx, const std::vector<int>& data={}) {
    std::string s; append(s,id); append(s,tx); append(s,rx);
    for (int v:data) append(s,static_cast<unsigned>(v));
    while(s.size()%4) s+='\0';
    return s;
}
inline size_t length(int id) {
    switch(id) {
    case EF01: case FE01: case EF02: case EF04: case FE03: case EF05: case FE05: return 8;
    case FE02: case EF03: case FE04: return 12;
    default: return 0;
    }
}
inline bool validRequest(const std::string& s) {
    if(s.size()<6) return false;
    unsigned id=word(s,0);
    if(responseId(id)==0 || s.size()!=length(id) || word(s,4)!=0) return false;
    size_t pad=id==EF03?10:id==EF04?8:6;
    for(size_t i=pad;i<s.size();++i) if(s[i]!=0) return false;
    return true;
}
}
#endif
