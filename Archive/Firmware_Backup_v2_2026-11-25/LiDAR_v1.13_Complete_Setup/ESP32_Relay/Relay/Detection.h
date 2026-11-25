#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <vector>
#include <utility>

namespace lidar {
struct Point {
  float x;
  float y;
  float angle;
  uint16_t index;
};

struct Result {
  bool close = false;
  bool newClose = false;
  bool incoming = false;
  bool renewWarning = false;
  float speed = 0;
  std::vector<uint16_t> closePoints;
  std::vector<uint16_t> approachPoints;
};

class Detector {
 public:
  void reset() {
    *this = Detector{};
  }

  Result processScan(const std::vector<Point> &points, double now) {
    if (lastScan >= 0 && now <= lastScan) reset();
    if (lastScan >= 0 && now - lastScan > 0.65) {
      closeRuns = 0;
      lastClose = -1;
      wasClose = false;
    }
    lastScan = now;
    Result result;
    detectClosePass(points, now, result);
    detectApproach(points, now, result);
    return result;
  }

 private:
  struct Shape {
    float left;
    float right;
    float y;
    unsigned count;
    float width() const { return right - left; }
    float center() const { return (left + right) / 2; }
    float weight() const {
      const float dx = left <= 0 && right >= 0 ? 0 : std::min(std::abs(left), std::abs(right));
      return 1.35f - 0.65f * std::min(dx / 5, 1.0f);
    }
  };

  struct Candidate {
    Shape shape;
    std::vector<uint16_t> indices;
  };

  struct ScanShapes {
    double time;
    std::vector<Shape> shapes;
  };

  struct Evidence {
    double time;
    float speed;
    float weight;
  };

  double lastScan = -1;
  double lastClose = -1;
  double warningUntil = -1;
  unsigned closeRuns = 0;
  bool wasClose = false;
  float speed = 0;
  std::deque<ScanShapes> history;
  std::deque<Evidence> evidence;

  void detectClosePass(const std::vector<Point> &points, double now, Result &result) {
    std::vector<Point> nearby;
    for (const Point &p : points) {
      const float radius = std::hypot(p.x, p.y);
      if (p.angle >= 55 && p.angle < 180 && radius >= 0.15f && radius <= 1.5f)
        nearby.push_back(p);
    }
    std::sort(nearby.begin(), nearby.end(), [](const Point &a, const Point &b) {
      return a.y < b.y;
    });
    float bestSpan = 0;
    for (size_t start = 0; start < nearby.size();) {
      size_t end = start + 1;
      while (end < nearby.size() && nearby[end].y - nearby[end - 1].y <= 0.55f) ++end;
      const float span = nearby[end - 1].y - nearby[start].y;
      if (end - start >= 12 && span >= 0.75f && span > bestSpan) {
        bestSpan = span;
        result.closePoints.clear();
        for (size_t i = start; i < end; ++i) result.closePoints.push_back(nearby[i].index);
      }
      start = end;
    }
    closeRuns = bestSpan ? std::min(closeRuns + 1, 2u) : 0;
    if (closeRuns == 2) lastClose = now;
    result.close = lastClose >= 0 && now - lastClose <= 2;
    result.newClose = result.close && !wasClose;
    wasClose = result.close;
  }

  static bool inApproachZone(const Point &p) {
    const float radius = std::hypot(p.x, p.y);
    const float angle = std::atan2(p.y, p.x) * 57.2957795f;
    return radius > 1.5f && radius <= 12 && angle >= -125 && angle < 0 && p.x <= 5;
  }

  static void collectGroups(const std::vector<Point> &band, std::vector<Candidate> &candidates) {
    for (size_t start = 0; start < band.size();) {
      size_t end = start + 1;
      while (end < band.size() && band[end].x - band[end - 1].x <= 0.85f) ++end;
      const float width = band[end - 1].x - band[start].x;
      if (end - start >= 3 && width >= 0.25f) {
        std::vector<float> ys;
        for (size_t i = start; i < end; ++i) ys.push_back(band[i].y);
        std::sort(ys.begin(), ys.end());
        if (width >= 0.85f * (ys.back() - ys.front())) {
          Shape shape{band[start].x, band[end - 1].x, ys[ys.size() / 2],
                      static_cast<unsigned>(ys.size())};
          Candidate candidate{shape, {}};
          for (size_t i = start; i < end; ++i) candidate.indices.push_back(band[i].index);
          candidates.push_back(std::move(candidate));
        }
      }
      start = end;
    }
  }

  static std::vector<Candidate> approachShapes(const std::vector<Point> &points) {
    std::vector<Point> filtered;
    float low = 0;
    float high = -12;
    for (const Point &p : points) {
      if (!inApproachZone(p)) continue;
      filtered.push_back(p);
      low = std::min(low, p.y);
      high = std::max(high, p.y);
    }
    if (filtered.size() < 3) return {};
    std::stable_sort(filtered.begin(), filtered.end(), [](const Point &a, const Point &b) {
      return a.x < b.x;
    });
    std::vector<Candidate> candidates;
    for (int bandIndex = static_cast<int>(std::floor(low * 5)); bandIndex / 5.0 <= high; ++bandIndex) {
      const double bottom = bandIndex / 5.0;
      std::vector<Point> band;
      for (const Point &p : filtered) {
        if (p.y >= bottom && p.y < bottom + 0.75) band.push_back(p);
      }
      collectGroups(band, candidates);
    }
    std::stable_sort(candidates.begin(), candidates.end(), [](const Candidate &a, const Candidate &b) {
      return a.shape.width() > b.shape.width();
    });
    std::vector<Candidate> unique;
    for (auto &candidate : candidates) {
      const Shape &shape = candidate.shape;
      const bool duplicate = std::any_of(unique.begin(), unique.end(), [&](const Candidate &prior) {
        return std::abs(shape.y - prior.shape.y) <= 0.35f &&
               std::abs(shape.center() - prior.shape.center()) <= 0.65f;
      });
      if (!duplicate) unique.push_back(std::move(candidate));
      if (unique.size() == 30) break;
    }
    return unique;
  }

  void detectApproach(const std::vector<Point> &points, double now, Result &result) {
    const auto candidates = approachShapes(points);
    while (!history.empty() && now - history.front().time > 0.85) history.pop_front();
    const Candidate *best = nullptr;
    float bestSpeed = 0;
    float bestScore = 0;
    for (const Candidate &candidate : candidates) {
      const Shape &shape = candidate.shape;
      if (shape.left > 1.5f || shape.right < -1.5f || shape.y >= 0) continue;
      for (const ScanShapes &scan : history) {
        const double elapsed = now - scan.time;
        if (elapsed < 0.04) continue;
        for (const Shape &prior : scan.shapes) {
          if (shape.left > prior.right + 0.7f || shape.right < prior.left - 0.7f) continue;
          const float velocity = (shape.y - prior.y) / elapsed;
          if (velocity < 0.6f || velocity > 35) continue;
          const float eta = (-1.5f - shape.y) / velocity;
          if (eta < -0.2f || eta > 8) continue;
          const float score = shape.weight() * shape.width();
          if (!best || score > bestScore || (score == bestScore && shape.count > best->shape.count)) {
            best = &candidate;
            bestScore = score;
            bestSpeed = velocity;
          }
        }
      }
    }
    ScanShapes current{now, {}};
    for (const Candidate &candidate : candidates) current.shapes.push_back(candidate.shape);
    history.push_back(std::move(current));
    if (history.size() > 48) history.pop_front();

    if (best) evidence.push_back({now, bestSpeed, best->shape.weight()});
    while (!evidence.empty() && now - evidence.front().time > 0.65) evidence.pop_front();
    if (evidence.size() > 48) evidence.pop_front();
    float weight = 0;
    for (const Evidence &item : evidence) weight += item.weight;
    if (evidence.size() >= 2 && evidence.back().time - evidence.front().time >= 0.08 && weight >= 2) {
      std::vector<float> speeds;
      for (const Evidence &item : evidence) speeds.push_back(item.speed);
      std::sort(speeds.begin(), speeds.end());
      const float median = speeds[speeds.size() / 2];
      speed = speed ? 0.65f * speed + 0.35f * median : median;
      warningUntil = now + 2;
      result.renewWarning = true;
      if (best) result.approachPoints = best->indices;
    }
    result.incoming = now < warningUntil;
    if (!result.incoming) speed = 0;
    result.speed = speed;
  }
};
}
