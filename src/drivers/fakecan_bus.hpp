#pragma once
#include <deque>
#include <optional>
#include "can_frame.hpp"

class FakeCanBus
{
public:
    void push_tx(const CanFrame& f) {
        tx_.push_back(f);
    }

    void push_rx(const CanFrame& f) {
        rx_.push_back(f);
    }

    std::optional<CanFrame> pop_tx() {
        if (tx_.empty()) return std::nullopt;
        auto f = tx_.front();
        tx_.pop_front();
        return f;        
    }

    std::optional<CanFrame> pop_rx() {
        if (rx_.empty()) return std::nullopt;
        auto f = rx_.front();
        rx_.pop_front();
        return f;        
    }
  private:
  std::deque<CanFrame> tx_;
  std::deque<CanFrame> rx_;
};