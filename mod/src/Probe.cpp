// Windows.h first, matching main.cpp's proven include order. winsock2.h must
// never enter this translation unit - that isolation lives in Socket.cpp.
// WIN32_LEAN_AND_MEAN is defined project-wide in CMakeLists.txt.
#include <Windows.h>

#include "Probe.hpp"

#include <Geode/Geode.hpp>

#include <algorithm>
#include <cstdio>
#include <cmath>

using namespace geode::prelude;

namespace probe {

// ---------------------------------------------------------------------------
// Trace
// ---------------------------------------------------------------------------

void Trace::reserve(size_t rows) {
	m_rows.reserve(rows);
	m_rows.clear();
	m_overflow = 0;
}

void Trace::clear() {
	m_rows.clear();
	m_overflow = 0;
}

void Trace::push(TraceRow const& row) {
	// Never grow here: a realloc in the stepping path is an unbounded stall.
	if (m_rows.size() >= m_rows.capacity()) {
		m_overflow++;
		return;
	}
	m_rows.push_back(row);
}

uint64_t Trace::hash() const {
	if (m_rows.empty()) return kFnvOffset;
	return fnv1a(m_rows.data(), m_rows.size() * sizeof(TraceRow));
}

size_t Trace::firstDivergence(Trace const& other) const {
	const size_t n = std::min(m_rows.size(), other.m_rows.size());
	for (size_t i = 0; i < n; i++) {
		if (std::memcmp(&m_rows[i], &other.m_rows[i], sizeof(TraceRow)) != 0) {
			return i;
		}
	}
	if (m_rows.size() != other.m_rows.size()) return n;
	return SIZE_MAX;
}

uint64_t Trace::hashFrom(size_t skip) const {
	if (m_rows.size() <= skip) return kFnvOffset;
	return fnv1a(m_rows.data() + skip, (m_rows.size() - skip) * sizeof(TraceRow));
}

size_t Trace::firstDivergenceMasked(Trace const& other, uint32_t ignoreFlags) const {
	const size_t n = std::min(m_rows.size(), other.m_rows.size());
	const uint32_t keep = ~ignoreFlags;

	for (size_t i = 0; i < n; i++) {
		TraceRow a = m_rows[i];
		TraceRow b = other.m_rows[i];
		a.flags &= keep;
		b.flags &= keep;
		if (std::memcmp(&a, &b, sizeof(TraceRow)) != 0) return i;
	}
	if (m_rows.size() != other.m_rows.size()) return n;
	return SIZE_MAX;
}

float Trace::lastPercent() const {
	if (m_rows.empty()) return 0.f;
	return asFloat(m_rows.back().percent);
}

bool Trace::writeCsv(std::string const& path) const {
	std::FILE* f = std::fopen(path.c_str(), "wb");
	if (!f) {
		log::error("Probe: could not open {} for writing", path);
		return false;
	}

	std::fprintf(f,
		"step,engine_step,x,y,rotation,percent,player_speed,y_velocity,gravity,"
		"x_bits,y_bits,rotation_bits,percent_bits,player_speed_bits,y_velocity_bits,gravity_bits,"
		"flags,dead,on_ground,on_ground2,on_ground3,on_ground4,ring_count,"
		"jump_buffered,upside_down,ship,bird_ufo,ball,dart_wave,robot,spider,swing,sideways,pressed,"
		"src_handle_button,src_push_button,src_holding_map,src_async_key\n");

	for (auto const& r : m_rows) {
		std::fprintf(f,
			"%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.17g,%.17g,"
			"%08X,%08X,%08X,%08X,%08X,%016llX,%016llX,"
			"%08X,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			r.step,
			r.engineStep,
			static_cast<double>(asFloat(r.x)),
			static_cast<double>(asFloat(r.y)),
			static_cast<double>(asFloat(r.rotation)),
			static_cast<double>(asFloat(r.percent)),
			static_cast<double>(asFloat(r.playerSpeed)),
			asDouble(r.yVelocity),
			asDouble(r.gravity),
			r.x, r.y, r.rotation, r.percent, r.playerSpeed,
			static_cast<unsigned long long>(r.yVelocity),
			static_cast<unsigned long long>(r.gravity),
			r.flags,
			(r.flags & FlagDead)         ? 1 : 0,
			(r.flags & FlagOnGround)     ? 1 : 0,
			(r.flags & FlagOnGround2)    ? 1 : 0,
			(r.flags & FlagOnGround3)    ? 1 : 0,
			(r.flags & FlagOnGround4)    ? 1 : 0,
			unpackRingCount(r.flags),
			(r.flags & FlagJumpBuffered) ? 1 : 0,
			(r.flags & FlagUpsideDown)   ? 1 : 0,
			(r.flags & FlagShip)         ? 1 : 0,
			(r.flags & FlagBird)         ? 1 : 0,
			(r.flags & FlagBall)         ? 1 : 0,
			(r.flags & FlagDart)         ? 1 : 0,
			(r.flags & FlagRobot)        ? 1 : 0,
			(r.flags & FlagSpider)       ? 1 : 0,
			(r.flags & FlagSwing)        ? 1 : 0,
			(r.flags & FlagSideways)     ? 1 : 0,
			(r.flags & FlagPressed)      ? 1 : 0,
			(r.flags & FlagSrcHandleButton) ? 1 : 0,
			(r.flags & FlagSrcPushButton)   ? 1 : 0,
			(r.flags & FlagSrcHoldingMap)   ? 1 : 0,
			(r.flags & FlagSrcAsyncKey)     ? 1 : 0);
	}

	std::fclose(f);

	if (m_overflow > 0) {
		log::error("Probe: trace OVERFLOWED by {} rows - this run is INVALID", m_overflow);
	}
	return true;
}

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------

static double queryFreq() {
	LARGE_INTEGER f;
	QueryPerformanceFrequency(&f);
	return static_cast<double>(f.QuadPart);
}

uint64_t nowTicks() {
	LARGE_INTEGER t;
	QueryPerformanceCounter(&t);
	return static_cast<uint64_t>(t.QuadPart);
}

double ticksToMicros(uint64_t ticks) {
	static const double freq = queryFreq();
	return (static_cast<double>(ticks) * 1'000'000.0) / freq;
}

Stats summarize(std::vector<double>& samples) {
	Stats s;
	if (samples.empty()) return s;

	std::sort(samples.begin(), samples.end());
	s.count  = samples.size();
	s.min    = samples.front();
	s.max    = samples.back();
	s.median = samples[samples.size() / 2];

	// Nearest-rank p99, clamped so small sample counts stay in bounds.
	size_t idx = static_cast<size_t>(std::ceil(0.99 * samples.size()));
	if (idx > 0) idx--;
	if (idx >= samples.size()) idx = samples.size() - 1;
	s.p99 = samples[idx];

	double sum = 0.0;
	for (double v : samples) sum += v;
	s.mean = sum / static_cast<double>(samples.size());

	return s;
}

// ---------------------------------------------------------------------------
// Paths
// ---------------------------------------------------------------------------

std::string outputPath(std::string const& name) {
	auto dir = Mod::get()->getSaveDir();
	return (dir / name).string();
}

} // namespace probe
