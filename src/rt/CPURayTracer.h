#pragma once

#include <atomic>
#include <cassert>
#include <deque>
#include <future>
#include <thread>
#include <core/SharedObject.h>
#include <utils/Stopwatch.h>
#include "RayTracer.h"

namespace cg::rt
{

class CPURayTracer : public SharedObject
{
public:
    struct Options
    {
        float       eps = 1e-4f;
        bool        flipYAxis = false;
        int         recursionDepth = 6;
        int         threads = std::thread::hardware_concurrency();
        uint16_t    tileSize = 32;
    };

    struct Status
    {
        std::atomic<uint32_t> rays = 0;
        std::atomic<uint32_t> hits = 0;
        std::atomic_flag running;
        std::atomic<uint32_t> workDone = 0;
        volatile uint32_t totalWork = 0;
    };

    CPURayTracer() = default;
    CPURayTracer(const Options& op) { setOptions(op); }

    void render(Frame* frame, const Camera* camera, const Scene* scene);

    const auto& options() const { return _options; }
    void setOptions(const Options& op);

    const auto status() const { return &_status; }

private:
    using Key = Scene::Key;

    vec3f trace(const Ray& ray, vec3f attenuation, int depth) const;

    vec3f miss() const;

    vec3f anyHit(const Intersection& hit, const Ray& ray,
        uint32_t object, int depth) const;

    vec3f closestHit(const Intersection& hit, const Ray& ray,
        vec3f attenuation, uint32_t object, int depth) const;

    int intersect(Intersection& hit, const Ray& ray) const;

    bool intersect(const Ray& ray) const;

    void renderTile(
        uint16_t X0, uint16_t X1,
        uint16_t Y0, uint16_t Y1
    ) const;

    void work();

    void progress(int done, int total) const;

    vec3f pixelRayDirection(int x, int y) const
    {
        vec3f P
        {
            _topLeftCorner.x + _half_dx * (2*x),
            _topLeftCorner.y - _half_dy * (2*y),
            -1.0f
        };
        P = _cameraToWorld.transform3x4(P);
        return (P - _cameraPosition).versor();
    }

    struct Tile { uint16_t x, y; };

    mutable Status _status;
    std::vector<std::thread> _workers;
    std::deque<Tile> _work;
    std::mutex _workMutex;

    Options _options {};
    Frame* _frame = nullptr;
    const Scene* _scene = nullptr;
    uint16_t _countX;
    uint16_t _countY;

    vec2f _topLeftCorner;
    float _half_dx;
    float _half_dy;

    mat4f _cameraToWorld;
    mat3f _cameraNormalToWorld;
    vec3f _cameraPosition;
    Camera::ProjectionType _cameraProjection;

    Stopwatch clock;
};

inline void CPURayTracer::setOptions(const Options& op)
{
    _options = op;
    _options.threads = std::clamp(_options.threads, 1, 0x1000);
    _options.tileSize = std::max<uint16_t>(_options.tileSize, 8);
}

} // namespace cg::rt
