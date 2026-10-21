#include "c2mvp/domain.hpp"
#include <stdexcept>
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <unistd.h>
#include <cerrno>
#endif
namespace c2mvp {
struct UdpTransport::Impl {
#ifdef _WIN32
    SOCKET socket=INVALID_SOCKET;
    Impl(){WSADATA data;require(WSAStartup(MAKEWORD(2,2),&data)==0,"WSAStartup failed");}
    ~Impl(){if(socket!=INVALID_SOCKET)closesocket(socket);WSACleanup();}
#else
    int socket=-1;
    ~Impl(){if(socket>=0)::close(socket);}
#endif
    Endpoint bound;
};
static sockaddr_in native(const Endpoint& ep){
    sockaddr_in a{};a.sin_family=AF_INET;a.sin_port=htons(static_cast<uint16_t>(ep.port));
    require(ep.port>=0&&ep.port<=65535&&inet_pton(AF_INET,ep.ip.c_str(),&a.sin_addr)==1,"invalid IPv4 endpoint");return a;
}
UdpTransport::UdpTransport(const std::string& ip,int port):impl(std::make_unique<Impl>()){
    impl->socket=::socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
#ifdef _WIN32
    require(impl->socket!=INVALID_SOCKET,"socket failed");
#else
    require(impl->socket>=0,"socket failed");
#endif
    auto a=native({ip,port});require(::bind(impl->socket,reinterpret_cast<sockaddr*>(&a),sizeof a)==0,"UDP bind failed");
#ifdef _WIN32
    int n=sizeof a;
#else
    socklen_t n=sizeof a;
#endif
    require(getsockname(impl->socket,reinterpret_cast<sockaddr*>(&a),&n)==0,"getsockname failed");
    impl->bound={ip,ntohs(a.sin_port)};
}
UdpTransport::~UdpTransport()=default;
Endpoint UdpTransport::address()const{return impl->bound;}
void UdpTransport::send(const std::string& bytes,const Endpoint& ep){
    auto a=native(ep);auto n=sendto(impl->socket,bytes.data(),static_cast<int>(bytes.size()),0,reinterpret_cast<sockaddr*>(&a),sizeof a);
    if(n!=static_cast<int>(bytes.size()))throw std::runtime_error("UDP send failed");
}
std::optional<Datagram> UdpTransport::receive(){
    fd_set read;FD_ZERO(&read);FD_SET(impl->socket,&read);timeval wait{0,20000};
    int rc=select(static_cast<int>(impl->socket)+1,&read,nullptr,nullptr,&wait);
    if(rc==0)return {};if(rc<0)throw std::runtime_error("UDP select failed");
    char buf[65536];sockaddr_in from{};
#ifdef _WIN32
    int size=sizeof from;
#else
    socklen_t size=sizeof from;
#endif
    auto n=recvfrom(impl->socket,buf,sizeof buf,0,reinterpret_cast<sockaddr*>(&from),&size);
    if(n<0)throw std::runtime_error("UDP receive failed");
    char ip[INET_ADDRSTRLEN];inet_ntop(AF_INET,&from.sin_addr,ip,sizeof ip);
    return Datagram{std::string(buf,n),{ip,ntohs(from.sin_port)}};
}
}
