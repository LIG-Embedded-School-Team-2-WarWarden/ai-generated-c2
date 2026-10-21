#pragma once
#include "mfs/icd/v1/mfs.pb.h"
#include <nlohmann/json.hpp>
#include <array>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace c2mvp {
namespace pb = mfs::icd::v1;
using Json=nlohmann::json;
std::string uid();
double stamp(const std::string&);
std::string utc(double);
void require(bool,const std::string&);
bool finite(double);
struct Clock {
    virtual ~Clock()=default;
    virtual double time() const=0;
    virtual double monotonic() const=0;
    std::string utc(double after=0) const { return c2mvp::utc(time()+after); }
};
struct SystemClock final:Clock { double time()const override; double monotonic()const override; };
struct Endpoint { std::string ip; int port=0; bool operator==(const Endpoint&)const=default; };
struct Datagram { std::string bytes; Endpoint source; };
struct DatagramTransport {
    virtual ~DatagramTransport()=default;
    virtual void send(const std::string&,const Endpoint&)=0;
    virtual std::optional<Datagram> receive()=0;
    virtual Endpoint address()const=0;
};
class UdpTransport final:public DatagramTransport {
    struct Impl; std::unique_ptr<Impl> impl;
public:
    UdpTransport(const std::string&,int); ~UdpTransport() override;
    void send(const std::string&,const Endpoint&)override;
    std::optional<Datagram> receive()override;
    Endpoint address()const override;
};
pb::Envelope envelope(pb::Envelope,const std::string&,const std::string&,const Clock&);
std::string encode(const pb::Envelope&);
pb::Envelope decode(const std::string&);
const pb::Result* outcome(const pb::Envelope&);
std::string request_id(const pb::Envelope&);
struct Installation {
    std::string role,calibration,profile; unsigned revision=0;
    std::array<double,3> position{},offset{};
    std::array<double,9> rotation{};
};
struct Config {
    bool simulation=false; std::string observer; double uncertainty=0;
    std::map<std::string,Installation> devices;
    static Config parse(const Json&);
};
struct Peer {
    std::string id,boot,session; Endpoint address;
    uint64_t hb=0,sample=0,epoch=0,obs_epoch=1,obs_sequence=0;
    double alive=0,status_received=0,mode_applied=0;
    std::optional<pb::DeviceStatus> status;
    std::optional<pb::CapabilityReport> cap;
    bool installed=false,install_result=false,install_sent=false,mode_sent=false,mode=false,collecting=false;
    unsigned mode_revision=1;
    std::string obs_session,collection,measurement_result="PENDING";
    std::map<std::string,pb::ObsPose> poses;
    std::deque<std::string> pose_order;
    Json mapping=nullptr;
};
class RequestTracker {
public:
    using Callback=std::function<void(const pb::Envelope*)>;
private:
    struct Pending { Peer peer; pb::Envelope command; pb::Envelope::BodyCase kind;
        Callback callback; std::string packet; double started=0,sent=0,timeout=2;
        unsigned attempts=1; bool accepted=false; };
    Clock& clock; DatagramTransport& transport;
    std::map<std::string,Pending> pending;
public:
    RequestTracker(Clock& c,DatagramTransport& t):clock(c),transport(t){}
    std::string send(const Peer&,const pb::Envelope&);
    void request(const Peer&,const pb::Envelope&,pb::Envelope::BodyCase,Callback,double timeout=2);
    void result(const Peer&,const pb::Envelope&);
    void cancel(const std::string&,std::set<pb::Envelope::BodyCase> kinds={});
    bool has(const std::string&,pb::Envelope::BodyCase kind=pb::Envelope::BODY_NOT_SET)const;
    void tick(const std::map<std::string,Peer>&);
    size_t size()const{return pending.size();}
};
struct Object { std::string id; double x=0,y=0,z=0; unsigned count=0; };
struct PointDetector {
    virtual ~PointDetector()=default;
    virtual std::vector<Object> detect(unsigned,unsigned,const std::vector<pb::Point>&)=0;
};
class LidarDetector final:public PointDetector {
public: std::vector<Object> detect(unsigned,unsigned,const std::vector<pb::Point>&)override;
};
struct PointingSolver {
    virtual ~PointingSolver()=default;
    virtual std::pair<double,double> solve(const Object&,const Installation&,const Installation&)const=0;
};
struct RelativePointingSolver final:PointingSolver {
    std::pair<double,double> solve(const Object&,const Installation&,const Installation&)const override;
};
enum class ObserverPhase { WaitEffStop,WaitCollectionStop,Opening,Moving,Stopping,Collecting,Completed,Failed };
std::string name(ObserverPhase);
struct ObserverOperation {
    ObserverPhase phase=ObserverPhase::WaitEffStop;
    std::string session,reason,operation="POINT";
    double started=0,pan=0,tilt=0; uint64_t sample=0;
    bool ack=false,failed=false;
    bool terminal()const;
    void transition(ObserverPhase);
};
enum class EffectorPhase { Opening,Running,Manual,Stopping,Stopped };
std::string name(EffectorPhase);
struct EffectorSession {
    std::string session,object,reason;
    uint64_t epoch=0,sequence=0,last_frame=0,stop_sample=0;
    EffectorPhase state=EffectorPhase::Opening;
    double last_send=0,last_observation=0;
    bool stop_ack=false;
    std::optional<std::pair<double,double>> manual;
};
struct Observation {
    pb::PointCloudChunk meta; std::vector<Object> objects; bool verified=false; double received=0;
};
class Cloud {
    struct Assembly { pb::PointCloudChunk meta; std::vector<pb::Point> slots;
        std::vector<bool> occupied; std::map<unsigned,std::string> chunks; size_t bytes=0; bool verified=false; };
    struct Job { uint64_t generation; std::string session; pb::PointCloudChunk meta;
        std::vector<pb::Point> points; bool verified; };
    struct Completion { Job job; std::vector<Object> objects; bool error=false; };
    Clock& clock; PointDetector& detector;
    std::optional<Assembly> assembly;
    std::map<unsigned,std::string> sealed;
    std::mutex mutex; std::condition_variable cv; bool closing=false,threaded;
    std::optional<Job> queued; std::optional<Completion> done;
    std::thread worker;
    void work(); void execute(Job);
public:
    std::string session,state="NO_DATA"; uint64_t high_frame=0,generation=0; double started=0;
    std::optional<Observation> result;
    Cloud(Clock&,PointDetector&,bool threaded); ~Cloud();
    void reset(); void fail(const std::string&);
    bool ingest(const std::string&,const pb::PointCloudChunk&,bool);
    void tick();
    std::optional<Observation> poll();
};
class Engine;
class DeviceSessions {
    Engine& host;
    struct Registration { std::string raw; double received; };
    std::map<std::pair<std::string,std::string>,Registration> registrations;
    std::map<std::string,std::set<std::string>> retired;
public:
    std::map<std::string,Peer> peers;
    explicit DeviceSessions(Engine& e):host(e){}
    void registration(const pb::Envelope&,const Endpoint&);
    void heartbeat(Peer&,const pb::Heartbeat&);
    void status(Peer&,const pb::DeviceStatus&);
    void capability(Peer&,const pb::CapabilityReport&);
    bool valid(const Peer&,bool stopped=false)const;
    void configure(Peer&); void tick();
};
class ObservationService {
    Engine& host; std::optional<std::pair<Object,double>> previous;
public:
    Cloud cloud;
    ObservationService(Engine&,PointDetector&,bool);
    void invalidate(const std::string&,bool reset=false);
    void trust_clock(const std::string&,const Json&);
    void trust_profile(const std::string&,const Json&);
    bool verify(const Peer&,const pb::PointCloudChunk&)const;
    const Observation& latest()const;
    void tick();
};
class EffectorController {
    Engine& host; const PointingSolver& solver;
    void send_manual(Peer&,const std::shared_ptr<EffectorSession>&);
    void update(const std::string&,const std::shared_ptr<EffectorSession>&,double);
public:
    std::map<std::string,std::shared_ptr<EffectorSession>> sessions;
    EffectorController(Engine& h,const PointingSolver& s):host(h),solver(s){}
    void begin(const std::string&,const std::string& object={},std::optional<std::pair<double,double>> manual={});
    void stop(const std::string&,const std::string& reason="OPERATOR_STOP");
    void stop_all(const std::string&);
    void observed(const std::vector<Object>&,double);
    bool stopped()const; void tick(double); Json snapshot()const;
};
class ObserverController {
    Engine& host;
    void motion(Peer&,const std::shared_ptr<ObserverOperation>&,bool point);
public:
    std::shared_ptr<ObserverOperation> action;
    explicit ObserverController(Engine& e):host(e){}
    bool busy()const{return action && !action->terminal();}
    void point(double,double); void stop(); void collect(bool,bool internal=false);
    void tick(double); Json snapshot()const;
};
// Composition root and CLI facade. Engine serializes all service calls.
class Engine {
    double last_hb=0,last_wall,last_mono; uint64_t hb_sequence=0;
public:
    Config config; Clock& clock; DatagramTransport& transport; std::string boot=uid();
    RequestTracker requests; DeviceSessions devices; ObservationService observation;
    EffectorController effector; ObserverController observer;
    std::deque<Json> events;
    Engine(Config,Clock&,DatagramTransport&,PointDetector&,const PointingSolver&,bool worker=true);
    void receive(const Datagram&); void tick();
    void log(const std::string&,const std::string& reason={});
    Json snapshot()const;
    Json command(const std::string&); // CLI parser, also driven by integration tests
};
}
