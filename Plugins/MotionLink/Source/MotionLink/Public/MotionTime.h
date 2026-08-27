#pragma once

// One process-wide monotonic nanosecond clock, used by every thread in the
// plugin (physics tick, worker, game thread) so their timestamps share an
// epoch and differences are meaningful across threads.
//
// FPlatformTime::Seconds() is QPC-backed and consistent process-wide. ns as
// uint64 keeps the wire/observer math in integers; the double->ns conversion
// stays exact well past any realistic uptime (2^53 ns ~= 104 days).

#include "HAL/PlatformTime.h"
#include <cstdint>

FORCEINLINE uint64_t MotionNowNs()
{
	return (uint64_t)(FPlatformTime::Seconds() * 1.0e9);
}
