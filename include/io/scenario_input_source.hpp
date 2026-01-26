#pragma once
#include "input_source.hpp"

// Scenario는 const로 받아도 apply가 in만 바꾸니까 const 유지 가능
template <typename ScenarioT>
class ScenarioInputSource final : public IInputSource {
public:
    ScenarioInputSource(const ScenarioT& sc, double dt_s)
        : sc_(sc), dt_s_(dt_s) {
        sc_.init(frame_.in);
    }

    bool read(InputFrame& out) override {
        if (tick_ >= sc_.end_tick()) return false;
        sc_.apply(tick_, frame_.in);

        // test에서는 굳이 실제 시간(us) 안 써도 되므로 "tick 기반"으로도 충분
        frame_.t_us = static_cast<std::uint64_t>(tick_ * dt_s_ * 1e6);
        frame_.valid = true;

        out = frame_;
        ++tick_;
        return true;
    }

    int tick() const { return tick_; }
    const ScenarioT& scenario() const { return sc_; }

private:
    const ScenarioT& sc_;
    double dt_s_ = 0.01;
    int tick_ = 0;
    InputFrame frame_{};
};
