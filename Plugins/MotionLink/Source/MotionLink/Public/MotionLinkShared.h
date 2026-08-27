#pragma once

// Cross-thread state shared between FMotionWorker (worker thread) and the game
// thread (subsystem tick + debug overlay). POD only, std::atomic only, no
// locks. This header is the entire contract between the two threads.
//
// Memory ordering convention (see CLAUDE.md "Порядок памяти"):
//   * Worker publishes telemetry with memory_order_release.
//   * Game thread reads it with memory_order_acquire.
//   * Game thread publishes controls with release; worker reads with acquire.
// Every field is an independent scalar snapshot for a read-only overlay, so an
// occasional torn *set* across fields is harmless — we never need multiple
// fields to be mutually consistent. release/acquire is used anyway to keep the
// ordering intent explicit and greppable.

#include <atomic>
#include <cstdint>

// Worker -> game thread. Observed state for the debug overlay.
struct FMotionLinkTelemetry
{
	std::atomic<uint32_t> Seq{0};          // last setpoint seq sent by worker
	std::atomic<uint32_t> TickHz{0};       // measured worker rate over last 1 s
	std::atomic<uint32_t> JitterP99Us{0};  // 99th pct tick interval, last 1 s
	std::atomic<uint32_t> JitterMaxUs{0};  // max tick interval, last 1 s
	std::atomic<uint32_t> RttUs{0};        // last feedback round trip, us
	std::atomic<uint32_t> State{0};        // last MotionState from feedback
	std::atomic<uint32_t> FeedbackSeq{0};  // last feedback seq observed
	std::atomic<uint32_t> SendErrors{0};   // cumulative sendto failures
	std::atomic<uint32_t> RingDepth{0};    // telemetry ring occupancy (approx)
	std::atomic<uint32_t> SampleAgeUs{0};  // age of last consumed sample, us
};

// Game thread -> worker. Live-tunable inputs sourced from motion.* CVars and
// pushed each frame. Fixed-point integers so the atomics are unambiguously
// lock-free and there is no float tearing to reason about.
struct FMotionLinkControls
{
	std::atomic<uint32_t> Enabled{1};            // 0/1: emit setpoints
	std::atomic<uint32_t> SineFreqMilliHz{500};  // sine frequency, mHz (0.5 Hz)
	std::atomic<uint32_t> SineAmpMicroM{200000}; // sine amplitude, um (0.2 m)
	std::atomic<uint32_t> SourceMode{1};         // 0: synthetic sine, 1: telemetry observer
	std::atomic<uint32_t> CorrectMs{8};          // residual correction smear, ms
};
