#include "AP_UAVCAN_Driver.h"
#include "AP_UAVCAN_Clock.h"
#include <AP_HAL/AP_HAL.h>
using namespace uavcan;
extern const AP_HAL::HAL& hal;


/*****************************************************
 *                                                   *
 *                    CAN Iface                      *
 *                                                   *
 * ***************************************************/

/**
 * Non-blocking transmission.
 *
 * If the frame wasn't transmitted upon TX deadline, the driver should discard it.
 *
 * Note that it is LIKELY that the library will want to send the frames that were passed into the select()
 * method as the next ones to transmit, but it is NOT guaranteed. The library can replace those with new
 * frames between the calls.
 *
 * @return 1 = one frame transmitted, 0 = TX buffer full, negative for error.
 */
int16_t CanIface::send(const CanFrame& frame, MonotonicTime tx_deadline, CanIOFlags flags) {
    return can_drv_->send(AP_HAL::CanFrame(frame.id, frame.data, frame.dlc), tx_deadline.toUSec(), flags);
}

/**
 * Non-blocking reception.
 *
 * Timestamps should be provided by the CAN driver, ideally by the hardware CAN controller.
 *
 * Monotonic timestamp is required and can be not precise since it is needed only for
 * protocol timing validation (transfer timeouts and inter-transfer intervals).
 *
 * UTC timestamp is optional, if available it will be used for precise time synchronization;
 * must be set to zero if not available.
 *
 * Refer to @ref ISystemClock to learn more about timestamps.
 *
 * @param [out] out_ts_monotonic Monotonic timestamp, mandatory.
 * @param [out] out_ts_utc       UTC timestamp, optional, zero if unknown.
 * @return 1 = one frame received, 0 = RX buffer empty, negative for error.
 */
int16_t CanIface::receive(CanFrame& out_frame, MonotonicTime& out_ts_monotonic, UtcTime& out_ts_utc,
                        CanIOFlags& out_flags) {
    AP_HAL::CanFrame frame;
    uint64_t rx_timestamp;
    uint16_t flags;
    out_ts_monotonic = uavcan::MonotonicTime::fromUSec(AP_HAL::micros());
    int16_t ret = can_drv_->receive(frame, rx_timestamp, flags);
    if (ret < 0) {
        return ret;
    }
    out_frame = CanFrame(frame.id, (const uint8_t*)frame.data, frame.dlc);
    out_flags = flags;
    if (rx_timestamp != 0) {
        out_ts_utc = uavcan::UtcTime::fromUSec(SystemClock::instance().getAdjustUsec() + rx_timestamp);
    } else {
        out_ts_utc = uavcan::UtcTime::fromUSec(0);
    }
    return ret;
}

/**
 * Configure the hardware CAN filters. @ref CanFilterConfig.
 *
 * @return 0 = success, negative for error.
 */
int16_t CanIface::configureFilters(const CanFilterConfig* filter_configs, uint16_t num_configs) {
    AP_HAL::CANDriver::CanFilterConfig* hal_filter_configs = new AP_HAL::CANDriver::CanFilterConfig[num_configs];
    if (hal_filter_configs == nullptr) {
        return -1;
    }
    for (uint16_t i = 0; i < num_configs; i++) {
        hal_filter_configs[i].id = filter_configs[i].id;
        hal_filter_configs[i].mask = filter_configs[i].mask;
    }
    return can_drv_->configureFilters(hal_filter_configs, num_configs);
}

/**
 * Number of available hardware filters.
 */
uint16_t CanIface::getNumFilters() const {
    return can_drv_->getNumFilters();
}

/**
 * Continuously incrementing counter of hardware errors.
 * Arbitration lost should not be treated as a hardware error.
 */
uint64_t CanIface::getErrorCount() const {
    return can_drv_->getErrorCount();
}

/*****************************************************
 *                                                   *
 *                    CAN Driver                     *
 *                                                   *
 * ***************************************************/

bool CanDriver::add_interface(AP_HAL::CANDriver *can_drv) {
    if (num_ifaces > MAX_NUMBER_OF_CAN_INTERFACES) {
        return false;
    }
    if (ifaces[num_ifaces] != nullptr) {
        return false;
    }
    ifaces[num_ifaces] = new CanIface(can_drv);
    if (ifaces[num_ifaces] == nullptr) {
        return false;
    }
    if (!ifaces[num_ifaces]->can_drv_->set_event_handle(&_event_handle)) {
        return false;
    }
    hal.console->printf("UAVCANDriver: Successfully added interface %d\n", num_ifaces);
    num_ifaces++;
    return true;
}
/**
 * Returns an interface by index, or null pointer if the index is out of range.
 */
ICanIface* CanDriver::getIface(uint8_t iface_index) {
    if (iface_index > num_ifaces) {
        return UAVCAN_NULLPTR;
    }
    return ifaces[iface_index];
}

/**
 * Total number of available CAN interfaces.
 * This value shall not change after initialization.
 */
uint8_t CanDriver::getNumIfaces() const { 
    return num_ifaces; 
}

CanSelectMasks CanDriver::makeSelectMasks(const CanFrame* (& pending_tx)[MaxCanIfaces]) const
{
    CanSelectMasks msk;
    for (uint8_t i = 0; i < num_ifaces; i++) {
        bool read = true;
        bool write = true;
        CanIface* iface = ifaces[i];
        if (pending_tx[i] == UAVCAN_NULLPTR) {
            if (iface->can_drv_->select(read, write, nullptr, 0) > 0) {
                msk.read  |= (read ? 1 : 0) << i;
                msk.write |= (write ? 1 : 0) << i;
            }
        } else {
            AP_HAL::CanFrame frame {pending_tx[i]->id, pending_tx[i]->data, pending_tx[i]->dlc};
            if (iface->can_drv_->select(read, write, &frame, 0) > 0) {
                msk.read  |= (read ? 1 : 0) << i;
                msk.write |= (write ? 1 : 0) << i;
            }
        }
    }

    return msk;
}

/**
 * Block until the deadline, or one of the specified interfaces becomes available for read or write.
 *
 * Iface masks will be modified by the driver to indicate which exactly interfaces are available for IO.
 *
 * Bit position in the masks defines interface index.
 *
 * Note that it is allowed to return from this method even if no requested events actually happened, or if
 * there are events that were not requested by the library.
 *
 * The pending TX argument contains an array of pointers to CAN frames that the library wants to transmit
 * next, per interface. This is intended to allow the driver to properly prioritize transmissions; many
 * drivers will not need to use it. If a write flag for the given interface is set to one in the select mask
 * structure, then the corresponding pointer is guaranteed to be valid (not UAVCAN_NULLPTR).
 *
 * @param [in,out] inout_masks        Masks indicating which interfaces are needed/available for IO.
 * @param [in]     pending_tx         Array of frames, per interface, that are likely to be transmitted next.
 * @param [in]     blocking_deadline  Zero means non-blocking operation.
 * @return Positive number of ready interfaces or negative error code.
 */
int16_t CanDriver::select(CanSelectMasks& inout_masks,
                                const CanFrame* (& pending_tx)[MaxCanIfaces],
                                const MonotonicTime blocking_deadline)
{
    const CanSelectMasks in_masks = inout_masks;
    const uint64_t time = AP_HAL::micros();

    inout_masks = makeSelectMasks(pending_tx);          // Check if we already have some of the requested events
    if ((inout_masks.read  & in_masks.read)  != 0 ||
        (inout_masks.write & in_masks.write) != 0) {
        return 1;
    }

    _event_handle.wait(blocking_deadline.toUSec() - time); // Block until timeout expires or any iface updates

    inout_masks = makeSelectMasks(pending_tx);  // Return what we got even if none of the requested events are set
    return 1;                                   // Return value doesn't matter as long as it is non-negative
}

