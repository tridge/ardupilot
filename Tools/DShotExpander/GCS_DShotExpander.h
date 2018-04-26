#pragma once

#include <GCS_MAVLink/GCS.h>
#include "GCS_Mavlink.h"

class GCS_DShotExpander : public GCS
{
    friend class DShotExpander;

public:

    // return the number of valid GCS objects
    uint8_t num_gcs() const override { return ARRAY_SIZE(_chan); };

    // return GCS link at offset ofs
    GCS_MAVLINK_DShotExpander &chan(const uint8_t ofs) override {
        return _chan[ofs];
    };
    const GCS_MAVLINK_DShotExpander &chan(const uint8_t ofs) const override {
        return _chan[ofs];
    };

private:

    GCS_MAVLINK_DShotExpander _chan[MAVLINK_COMM_NUM_BUFFERS];
};
