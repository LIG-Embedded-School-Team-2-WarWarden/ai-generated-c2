#pragma once
#include "rdr_session.hpp"
#include "udp_transport.hpp"
#include <chrono>
#include <mutex>
#include <random>
#include <stdexcept>
#include <utility>
namespace c2::rdr {
inline double seconds(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
class Client {
    Endpoint peer_;std::mutex mutex_;Session session_;UdpTransport transport_;
    void flush(){for(const auto&p:session_.take()){
        std::vector<std::byte> bytes;for(auto v:p)bytes.push_back(std::byte(v));transport_.send(bytes,peer_);
    }}
public:
    Client(UdpConfig local,Endpoint peer):peer_(std::move(peer)),session_(std::random_device{}()),
        transport_(std::move(local),[this](auto bytes,auto from){
            if(from.address!=peer_.address||from.port!=peer_.port||bytes.size()!=24)return;
            Packet p{};for(size_t i=0;i<24;++i)p[i]=std::to_integer<uint8_t>(bytes[i]);
            std::lock_guard lock(mutex_);session_.receive(p,seconds());flush();
        }){
            if(peer_.address.empty() || !peer_.port)throw std::invalid_argument("invalid RDR endpoint");
            transport_.start();
        }
    ~Client(){transport_.stop();}
    Endpoint local_endpoint() const{return transport_.local_endpoint();}
    UdpTransportStats transport_stats() const noexcept{return transport_.stats();}
    uint16_t connect(){std::lock_guard lock(mutex_);auto result=session_.connect(seconds());flush();return result;}
    uint16_t command(uint16_t msg,uint16_t a=0,uint16_t b=0){std::lock_guard lock(mutex_);auto result=session_.command(msg,a,b,seconds());flush();return result;}
    void tick(){std::lock_guard lock(mutex_);session_.tick(seconds());flush();}
    State snapshot(){std::lock_guard lock(mutex_);return session_.state();}
};
}
