// The on-board radar mapper's inner loop in C++ (docs/survey_mapping_plan.md, phase 5): the same as the
// Python of scripts/onboard_mapper.py (VoxelMap) and scripts/radar_array.py (doppler_consistent), which
// stay the reference; tests/test_mapper_core.py holds them to each other. Built by cpp/build.sh into
// scripts/mapper_core*.so; onboard_mapper.py uses it when it is there.

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace py = pybind11;
using Key = std::array<int64_t, 3>;

namespace {

int64_t pack(const Key &k)
{
	// 21 bits each, offset: +-1e6 voxels, ample for a site.
	const int64_t o = 1 << 20;
	return ((k[0] + o) << 42) | ((k[1] + o) << 21) | (k[2] + o);
}

struct Cell {
	Key key;
	int64_t hits = 0, frames = 0, last_frame = -1, hits_sent = 0;
	double sum[3] = {0, 0, 0};
	double smin[3], smax[3];
	bool confirmed = false;
	double weight = 0.0;
};

class VoxelMap {
public:
	VoxelMap(double voxel, double ground_z, int min_frames, double min_baseline, double near_range, double ground_band)
		: voxel_(voxel), ground_z_(ground_z), min_frames_(min_frames), min_baseline_(min_baseline),
		  near_range_(near_range), ground_band_(ground_band) {}

	// points (n, 3) seen from sensor (3): the keys to report, as onboard_mapper.VoxelMap.add.
	std::vector<Key> add(int64_t frame, py::array_t<double, py::array::c_style | py::array::forcecast> points,
			     py::array_t<double, py::array::c_style | py::array::forcecast> sensor)
	{
		auto p = points.unchecked<2>();
		auto s = sensor.unchecked<1>();
		std::vector<Key> report;

		for (py::ssize_t i = 0; i < p.shape(0); ++i) {
			const double x = p(i, 0), y = p(i, 1), z = p(i, 2);
			const Key key{(int64_t)std::floor(x / voxel_), (int64_t)std::floor(y / voxel_), (int64_t)std::floor(z / voxel_)};
			const int64_t id = pack(key);
			auto it = index_.find(id);

			if (it == index_.end()) {
				Cell c;
				c.key = key;
				for (int k = 0; k < 3; ++k) { c.smin[k] = c.smax[k] = s(k); }
				it = index_.emplace(id, cells_.size()).first;
				cells_.push_back(c);
			}

			Cell &c = cells_[it->second];
			const double r = std::sqrt((x - s(0)) * (x - s(0)) + (y - s(1)) * (y - s(1)) + (z - s(2)) * (z - s(2)));
			const double w = near_range_ / std::max(r, near_range_);
			c.weight += w * w;
			c.hits += 1;
			if (c.last_frame != frame) { c.frames += 1; c.last_frame = frame; }
			c.sum[0] += x; c.sum[1] += y; c.sum[2] += z;
			double spread = 0.0;
			for (int k = 0; k < 3; ++k) {
				c.smin[k] = std::min(c.smin[k], s(k));
				c.smax[k] = std::max(c.smax[k], s(k));
				spread = std::max(spread, c.smax[k] - c.smin[k]);
			}

			if (!c.confirmed) {
				if (c.frames >= min_frames_ && spread >= min_baseline_) {
					c.confirmed = true;
					c.hits_sent = c.hits;
					report.push_back(key);
				}
			} else if (c.hits >= 2 * c.hits_sent) {
				c.hits_sent = c.hits;
				report.push_back(key);
			}
		}

		return report;
	}

	// (hits, weight, mean z) of the cells with these keys, for radar_link's voxel records.
	py::tuple records(const std::vector<Key> &keys) const
	{
		py::array_t<int64_t> hits(keys.size());
		py::array_t<double> weight(keys.size()), mean_z(keys.size());
		auto h = hits.mutable_unchecked<1>();
		auto w = weight.mutable_unchecked<1>();
		auto m = mean_z.mutable_unchecked<1>();
		for (size_t i = 0; i < keys.size(); ++i) {
			const Cell &c = cells_.at(index_.at(pack(keys[i])));
			h(i) = c.hits; w(i) = c.weight; m(i) = c.sum[2] / c.hits;
		}
		return py::make_tuple(hits, weight, mean_z);
	}

	// (keys (n, 3), centroids (n, 3), hits (n,), ground (n,)) of the confirmed voxels.
	py::tuple confirmed() const
	{
		std::vector<const Cell *> rows;
		for (const Cell &c : cells_) { if (c.confirmed) { rows.push_back(&c); } }
		py::array_t<int64_t> keys({(py::ssize_t)rows.size(), (py::ssize_t)3});
		py::array_t<double> cen({(py::ssize_t)rows.size(), (py::ssize_t)3});
		py::array_t<int64_t> hits(rows.size());
		py::array_t<bool> ground(rows.size());
		auto k = keys.mutable_unchecked<2>();
		auto c = cen.mutable_unchecked<2>();
		auto h = hits.mutable_unchecked<1>();
		auto g = ground.mutable_unchecked<1>();
		for (size_t i = 0; i < rows.size(); ++i) {
			for (int j = 0; j < 3; ++j) { k(i, j) = rows[i]->key[j]; c(i, j) = rows[i]->sum[j] / rows[i]->hits; }
			h(i) = rows[i]->hits;
			g(i) = c(i, 2) < ground_z_ + ground_band_;
		}
		return py::make_tuple(keys, cen, hits, ground);
	}

	size_t size() const { return cells_.size(); }
	double ground_z() const { return ground_z_; }

private:
	double voxel_, ground_z_;
	int min_frames_;
	double min_baseline_, near_range_, ground_band_;
	std::vector<Cell> cells_;
	std::unordered_map<int64_t, size_t> index_;
};

// One radar frame: the Doppler gate (radar_array.doppler_consistent) and the detections in the world.
// det (n, 5) in the radar frame; rotation radar -> world, position, velocity in the radar frame.
py::tuple place_frame(py::array_t<double, py::array::c_style | py::array::forcecast> det,
		      py::array_t<double, py::array::c_style | py::array::forcecast> rotation,
		      py::array_t<double, py::array::c_style | py::array::forcecast> position,
		      py::array_t<double, py::array::c_style | py::array::forcecast> velocity,
		      double sigma_v, double sigma_angle, double gate)
{
	auto d = det.unchecked<2>();
	auto R = rotation.unchecked<2>();
	auto P = position.unchecked<1>();
	auto V = velocity.unchecked<1>();
	const double vv = V(0) * V(0) + V(1) * V(1) + V(2) * V(2);
	std::vector<py::ssize_t> keep;

	for (py::ssize_t i = 0; i < d.shape(0); ++i) {
		if (gate > 0.0) {
			const double n = std::max(std::sqrt(d(i, 0) * d(i, 0) + d(i, 1) * d(i, 1) + d(i, 2) * d(i, 2)), 1e-6);
			const double expected = -(d(i, 0) * V(0) + d(i, 1) * V(1) + d(i, 2) * V(2)) / n;
			const double v_perp = std::sqrt(std::max(vv - expected * expected, 0.0));
			const double spread = std::sqrt(sigma_v * sigma_v + (v_perp * sigma_angle) * (v_perp * sigma_angle));
			if (std::abs(d(i, 3) - expected) > gate * std::max(spread, 0.05)) { continue; }
		}
		keep.push_back(i);
	}

	py::array_t<double> out({(py::ssize_t)keep.size(), (py::ssize_t)5});
	auto o = out.mutable_unchecked<2>();
	for (size_t j = 0; j < keep.size(); ++j) {
		const py::ssize_t i = keep[j];
		for (int r = 0; r < 3; ++r) {
			o(j, r) = R(r, 0) * d(i, 0) + R(r, 1) * d(i, 1) + R(r, 2) * d(i, 2) + P(r);
		}
		o(j, 3) = d(i, 3);
		o(j, 4) = d(i, 4);
	}
	return py::make_tuple(out, (int64_t)(d.shape(0) - (py::ssize_t)keep.size()));
}

// PX4's estimated pose as the companion computer has it (onboard_mapper.CausalPose over
// survey_process.EstimatedPose): the samples taken up to now - delay; interpolated at the scan time t,
// or the newest carried forward to t with its velocity. World ENU, base_link FLU.
class PoseTrack {
public:
	using Arr = py::array_t<double, py::array::c_style | py::array::forcecast>;

	PoseTrack(Arr tp, Arr p_ned, Arr v_ned, Arr tq, Arr q, Arr tw, Arr w_frd, Arr ts, Arr heading_sigma, Arr origin,
		  double delay)
		: delay_(delay)
	{
		copy(tp, tp_); copy(p_ned, p_); copy(v_ned, v_); copy(tq, tq_); copy(q, q_); copy(tw, tw_);
		copy(w_frd, w_); copy(ts, ts_); copy(heading_sigma, hs_); copy(origin, origin_);
	}

	py::tuple at(double t, double now) const
	{
		const double known = now - delay_;
		const double last = std::min({tp_[last_index(tp_, known)], tq_[last_index(tq_, known)], tw_[last_index(tw_, known)]});
		const double sigma = hs_[last_index(ts_, known)];
		const double tc = t <= last ? t : last;
		double p[3], v[3], w[3], q[4];
		lerp(tp_, p_, 3, tc, p);
		lerp(tp_, v_, 3, tc, v);
		lerp(tw_, w_, 3, tc, w);
		nlerp(tc, q);

		if (t > last) {
			for (int k = 0; k < 3; ++k) { p[k] += v[k] * (t - last); }
		}

		// NED -> ENU: (x, y, z) -> (y, x, -z); FRD -> FLU: (x, y, z) -> (x, -y, -z).
		py::array_t<double> P(3), V(3), W(3), R({3, 3});
		auto Pm = P.mutable_unchecked<1>(); auto Vm = V.mutable_unchecked<1>(); auto Wm = W.mutable_unchecked<1>();
		Pm(0) = p[1] + origin_[0]; Pm(1) = p[0] + origin_[1]; Pm(2) = -p[2] + origin_[2];
		Vm(0) = v[1]; Vm(1) = v[0]; Vm(2) = -v[2];
		Wm(0) = w[0]; Wm(1) = -w[1]; Wm(2) = -w[2];
		const double qw = q[0], qx = q[1], qy = q[2], qz = q[3];
		const double M[3][3] = {{1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)},
					{2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)},
					{2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)}};
		// rot = NED_TO_ENU @ M @ FLU_TO_FRD: rows (1, 0, -2) of M, columns scaled by (1, -1, -1).
		const int rows[3] = {1, 0, 2};
		const double rs[3] = {1, 1, -1}, cs[3] = {1, -1, -1};
		auto Rm = R.mutable_unchecked<2>();
		for (int i = 0; i < 3; ++i) { for (int j = 0; j < 3; ++j) { Rm(i, j) = rs[i] * M[rows[i]][j] * cs[j]; } }
		return py::make_tuple(P, R, V, W, sigma);
	}

private:
	static void copy(const Arr &a, std::vector<double> &out)
	{
		out.assign(a.data(), a.data() + a.size());
	}

	// The newest index with t[i] <= x (0 if none).
	static size_t last_index(const std::vector<double> &t, double x)
	{
		auto it = std::upper_bound(t.begin(), t.end(), x);
		return it == t.begin() ? 0 : (size_t)(it - t.begin() - 1);
	}

	// np.interp: linear, clamped to the ends.
	static void lerp(const std::vector<double> &t, const std::vector<double> &v, int dim, double x, double *out)
	{
		const size_t n = t.size();
		if (x <= t.front()) { for (int k = 0; k < dim; ++k) { out[k] = v[k]; } return; }
		if (x >= t.back()) { for (int k = 0; k < dim; ++k) { out[k] = v[(n - 1) * dim + k]; } return; }
		const size_t j = std::upper_bound(t.begin(), t.end(), x) - t.begin();
		const size_t i = j - 1;
		const double a = (x - t[i]) / (t[j] - t[i]);
		for (int k = 0; k < dim; ++k) { out[k] = (1 - a) * v[i * dim + k] + a * v[j * dim + k]; }
	}

	// survey_process.interp_quat (q already in one hemisphere throughout).
	void nlerp(double x, double *out) const
	{
		const size_t n = tq_.size();
		size_t i = std::lower_bound(tq_.begin(), tq_.end(), x) - tq_.begin();
		i = i == 0 ? 0 : i - 1;
		i = std::min(i, n - 2);
		double a = (x - tq_[i]) / std::max(tq_[i + 1] - tq_[i], 1e-9);
		a = std::min(std::max(a, 0.0), 1.0);
		double norm = 0.0;
		for (int k = 0; k < 4; ++k) { out[k] = (1 - a) * q_[i * 4 + k] + a * q_[(i + 1) * 4 + k]; norm += out[k] * out[k]; }
		norm = std::sqrt(norm);
		for (int k = 0; k < 4; ++k) { out[k] /= norm; }
	}

	double delay_;
	std::vector<double> tp_, p_, v_, tq_, q_, tw_, w_, ts_, hs_, origin_;
};

} // namespace

PYBIND11_MODULE(mapper_core, m)
{
	m.doc() = "On-board radar mapper core (see cpp/mapper_core.cpp)";
	py::class_<VoxelMap>(m, "VoxelMap")
		.def(py::init<double, double, int, double, double, double>(), py::arg("voxel"), py::arg("ground_z"),
		     py::arg("min_frames"), py::arg("min_baseline"), py::arg("near_range"), py::arg("ground_band"))
		.def("add", &VoxelMap::add)
		.def("records", &VoxelMap::records)
		.def("confirmed", &VoxelMap::confirmed)
		.def("__len__", &VoxelMap::size)
		.def_property_readonly("ground_z", &VoxelMap::ground_z);
	m.def("place_frame", &place_frame);
	py::class_<PoseTrack>(m, "PoseTrack")
		.def(py::init<PoseTrack::Arr, PoseTrack::Arr, PoseTrack::Arr, PoseTrack::Arr, PoseTrack::Arr, PoseTrack::Arr,
			      PoseTrack::Arr, PoseTrack::Arr, PoseTrack::Arr, PoseTrack::Arr, double>())
		.def("at", &PoseTrack::at);
}
