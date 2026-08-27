#pragma once

// Hand-written single-producer / single-consumer lock-free ring.
//
// Deliberately NOT TQueue: the point of the prototype is to write the memory
// ordering ourselves and understand it (CLAUDE.md Stage 3).
//
//   Producer: the physics thread (UTelemetrySourceComponent async tick).
//   Consumer: the MotionWorker thread (const-accel observer).
//
// Exactly one of each. Two indices, each on its own cache line (alignas(64))
// so the producer's writes to WriteIdx never invalidate the consumer's
// ReadIdx line and vice versa (false-sharing avoidance).

#include <atomic>
#include <cstdint>
#include <cstring>

// One physics-tick snapshot. POD, trivially copyable — only copies of this
// cross the thread boundary, never a UObject.
struct FTelemetrySample
{
	uint64_t t_phys_ns = 0;      // solver SimTime, ns — for the observer's accel slope
	uint64_t t_wall_ns = 0;      // MotionNowNs() at the physics tick — for latency
	float    pose[6] = {0};      // x,y,z (m), roll,pitch,yaw (rad)
	float    vel[6]  = {0};      // linear (m/s), angular (rad/s)
};

class FTelemetryRing
{
public:
	// Capacity must be a power of two so index wrap is a mask. One slot is
	// always left empty to disambiguate full from empty, so usable depth is
	// kCapacity - 1.
	static constexpr uint32_t kCapacity = 1024;
	static constexpr uint32_t kMask = kCapacity - 1;
	static_assert((kCapacity & kMask) == 0, "kCapacity must be a power of two");

	FTelemetryRing() = default;

	// --- producer side (physics thread only) ---------------------------------
	// Returns false and drops the sample if the ring is full (consumer stalled).
	bool Push(const FTelemetrySample& s)
	{
		const uint32_t w = WriteIdx.load(std::memory_order_relaxed); // we are the only writer
		const uint32_t next = (w + 1) & kMask;
		// acquire: don't let the emptiness check float above the slot write on
		// the consumer's view; pairs with the consumer's release store of ReadIdx.
		if (next == ReadIdx.load(std::memory_order_acquire))
		{
			return false; // full — consumer stalled; caller drops the sample
		}
		Buffer[w] = s;
		// release: publish the slot contents before the index the consumer reads.
		WriteIdx.store(next, std::memory_order_release);
		return true;
	}

	// --- consumer side (worker thread only) ----------------------------------
	// Returns false if empty.
	bool Pop(FTelemetrySample& out)
	{
		const uint32_t r = ReadIdx.load(std::memory_order_relaxed); // we are the only reader
		// acquire: ensure the slot read below sees the producer's release store.
		if (r == WriteIdx.load(std::memory_order_acquire))
		{
			return false; // empty
		}
		out = Buffer[r];
		// release: free the slot only after we've copied it out.
		ReadIdx.store((r + 1) & kMask, std::memory_order_release);
		return true;
	}

	// Approximate occupancy for the debug overlay (either thread; racy by nature).
	uint32_t ApproxDepth() const
	{
		const uint32_t w = WriteIdx.load(std::memory_order_acquire);
		const uint32_t r = ReadIdx.load(std::memory_order_acquire);
		return (w - r) & kMask;
	}

private:
	FTelemetrySample Buffer[kCapacity];

	// Each index owns a cache line to avoid false sharing between the two
	// threads. 64 bytes is the common x86-64 line size.
	alignas(64) std::atomic<uint32_t> WriteIdx{0};
	alignas(64) std::atomic<uint32_t> ReadIdx{0};
};
