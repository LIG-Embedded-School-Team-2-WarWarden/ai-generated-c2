#pragma once
#include "batch_sequence.hpp"
#include <cstdint>
#include <vector>

// Automates the existing operator-paced batch workflow. Each registered target
// receives the normal EF03 point command followed by the normal EF05 capture
// command. BatchSequence remains the authority for alignment and completion.
class SequentialPhotoSequence {
public:
    using Result=BatchSequence::Result;
    enum class Phase : std::uint32_t { idle, running, completed, stopped, failed };

    Phase phase() const {return phase_;}
    Result reason() const {return reason_;}
    std::uint32_t total() const {return static_cast<std::uint32_t>(targets_.size());}
    std::uint32_t confirmed() const {return confirmed_;}
    std::uint32_t current_target() const {
        return index_<targets_.size()?targets_[index_]:0;
    }
    bool running() const {return phase_==Phase::running;}
    bool holds_batch() const {return running();}

    Result start(const std::vector<BatchSequence::PhotoTarget>& targets,
                 const BatchSequence& batch,const BatchSequence::Context& context) {
        if(running())return Result::device_busy;
        reset();
        if(targets.empty()||!batch.photo_pause_valid(context))return Result::invalid_request;
        for(const auto& target:targets)
            if(!target.id||batch.state(target.id)!=102)return Result::invalid_request;
        targets_.reserve(targets.size());
        for(const auto& target:targets)targets_.push_back(target.id);
        phase_=Phase::running;
        return Result::success;
    }

    void tick(BatchSequence& batch,const BatchSequence::Context& context,
              const BatchSequence::Actions& actions) {
        if(!running())return;
        if(index_>=targets_.size()){phase_=Phase::completed;return;}
        const auto id=targets_[index_];
        const auto state=batch.state(id);
        if(state==3){fail(batch.reason(id));return;}
        if(stopping_) {
            if(state==106){phase_=Phase::stopped;return;}
            if(state!=105&&state!=112){phase_=Phase::stopped;return;}
            return;
        }
        if(state==102) {
            const auto result=batch.command(1203,id,context,actions);
            if(result!=Result::success)fail(result);
            return;
        }
        if(state==110)return;
        if(state==104) {
            const auto result=batch.command(1205,id,context,actions);
            if(result!=Result::success)fail(result);
            return;
        }
        if(state==105||state==112)return;
        if(state==106) {
            ++confirmed_;++index_;
            if(index_==targets_.size())phase_=Phase::completed;
            return;
        }
        fail(Result::invalid_request);
    }

    Result stop(BatchSequence& batch,const BatchSequence::Context& context,
                const BatchSequence::Actions& actions) {
        if(!running())return phase_==Phase::failed?reason_:Result::success;
        const auto id=current_target();
        const auto state=batch.state(id);
        if(state==105||state==112) {
            stopping_=true;
            const auto result=batch.command(uint32_t(c2::ipc::Command::stop_batch_capture),id,context,actions);
            if(result!=Result::success)fail(result);
            return result;
        }
        phase_=Phase::stopped;
        return Result::success;
    }

private:
    Phase phase_{Phase::idle};
    Result reason_{Result::success};
    std::vector<std::uint32_t> targets_;
    std::size_t index_{};
    std::uint32_t confirmed_{};
    bool stopping_{};

    void fail(Result reason) {phase_=Phase::failed;reason_=reason;}
    void reset() {
        phase_=Phase::idle;reason_=Result::success;targets_.clear();index_=0;confirmed_=0;stopping_=false;
    }
};
