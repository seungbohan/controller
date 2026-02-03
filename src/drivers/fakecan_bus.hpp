#pragma once
#include <deque>
#include <optional>
#include <cstdint>
#include <random>
#include <algorithm>
#include "can_frame.hpp"

class FakeCanBus {
public:
  struct Config {
    uint64_t delay_us  = 0;       // 기본 지연
    uint64_t jitter_us = 0;       // 0~jitter_us 추가
    double   drop_rate = 0.0;     // 0.0~1.0
  };

  explicit FakeCanBus(Config cfg = {}) : cfg_(cfg), rng_(std::random_device{}()) {}

  void set_config(const Config& cfg) { cfg_ = cfg; }

  // TX는 즉시 큐잉(원하면 TX도 pending 처리 가능)
  void push_tx(const CanFrame& f) { tx_.push_back(f); }

  // RX는 "도착 예정"으로 pending에 넣음 (지연/지터/드롭 적용)
  void push_rx(const CanFrame& f) {
    if (should_drop_()) return;

    CanFrame g = f;
    const uint64_t extra = (cfg_.jitter_us > 0) ? (rand_u64_(0, cfg_.jitter_us)) : 0;
    g.t_us = f.t_us; // 원본 timestamp 유지(원하면 now로 overwrite 가능)

    const uint64_t deliver_us = now_us_ + cfg_.delay_us + extra;
    pending_rx_.push_back({deliver_us, g});
  }

  // 시간을 진행시키고, 도착 시간이 된 pending을 rx_로 이동
  void poll(uint64_t now_us) {
    now_us_ = now_us;

    // pending 중 deliver_us <= now_us 인 것들을 rx로 이동
    // pending_rx_가 작으니 단순 순회(나중에 정렬/힙 가능)
    auto it = pending_rx_.begin();
    while (it != pending_rx_.end()) {
      if (it->deliver_us <= now_us_) {
        rx_.push_back(it->frame);
        it = pending_rx_.erase(it);
      } else {
        ++it;
      }
    }
  }

  std::optional<CanFrame> pop_tx() {
    if (tx_.empty()) return std::nullopt;
    CanFrame f = tx_.front();
    tx_.pop_front();
    return f;
  }

  std::optional<CanFrame> pop_rx() {
    if (rx_.empty()) return std::nullopt;
    CanFrame f = rx_.front();
    rx_.pop_front();
    return f;
  }

private:
  struct Pending {
    uint64_t deliver_us;
    CanFrame frame;
  };

  bool should_drop_() {
    if (cfg_.drop_rate <= 0.0) return false;
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(rng_) < cfg_.drop_rate;
  }

  uint64_t rand_u64_(uint64_t lo, uint64_t hi) {
    std::uniform_int_distribution<uint64_t> dist(lo, hi);
    return dist(rng_);
  }

  Config cfg_;
  uint64_t now_us_ = 0;

  std::deque<CanFrame> tx_;
  std::deque<CanFrame> rx_;
  std::deque<Pending>  pending_rx_;

  std::mt19937 rng_;
};
