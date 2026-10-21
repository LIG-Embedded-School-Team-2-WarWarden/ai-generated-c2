#include "c2mvp/domain.hpp"
namespace c2mvp {
std::string name(ObserverPhase p) {
    static const char *names[] = {"WAIT_EFF_STOP", "WAIT_COLLECTION_STOP", "OPENING",   "MOVING",
                                  "STOPPING",      "COLLECTING",           "COMPLETED", "FAILED"};
    return names[static_cast<int>(p)];
}
std::string name(EffectorPhase p) {
    static const char *names[] = {"OPENING", "RUNNING", "MANUAL", "STOPPING", "STOPPED"};
    return names[static_cast<int>(p)];
}
bool ObserverOperation::terminal() const {
    return phase == ObserverPhase::Completed || phase == ObserverPhase::Failed;
}
void ObserverOperation::transition(ObserverPhase next) {
    if (next == phase)
        return;
    static const std::map<ObserverPhase, std::set<ObserverPhase>> allowed = {
        {ObserverPhase::WaitEffStop, {ObserverPhase::WaitCollectionStop, ObserverPhase::Failed}},
        {ObserverPhase::WaitCollectionStop,
         {ObserverPhase::Opening, ObserverPhase::Completed, ObserverPhase::Failed}},
        {ObserverPhase::Opening, {ObserverPhase::Moving, ObserverPhase::Failed}},
        {ObserverPhase::Moving,
         {ObserverPhase::Collecting, ObserverPhase::Stopping, ObserverPhase::Failed}},
        {ObserverPhase::Stopping, {ObserverPhase::WaitCollectionStop, ObserverPhase::Failed}},
        {ObserverPhase::Collecting, {ObserverPhase::Completed, ObserverPhase::Failed}},
        {ObserverPhase::Completed, {}},
        {ObserverPhase::Failed, {}}};
    require(allowed.at(phase).contains(next),
            "illegal OBS transition: " + name(phase) + " -> " + name(next));
    phase = next;
}
} // namespace c2mvp
