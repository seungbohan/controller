#pragma once
#include "../../include/io/output_sink.hpp"
#include "../../sim/plant.hpp"

class PlantOutputSink final : public IOutputSink {
public:
    PlantOutputSink(Plant& plant, Inputs& shared_in, double dt_s)
        : plant_(plant), in_(shared_in), dt_s_(dt_s) {}

    void write(const OutputFrame& frame) override {
        // ✅ 여기서 plant가 in_.velocity 를 업데이트한다 (네 Plant 설계와 정확히 일치)
        plant_.step(frame.out, in_, dt_s_);
    }

private:
    Plant& plant_;
    Inputs& in_;      // ✅ runner가 쓰는 Inputs와 같은 객체(공유)
    double dt_s_;
};
