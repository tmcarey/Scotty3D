
#include "../geometry/spline.h"
#include "debug.h"

template<typename T>
T Spline<T>::cubic_unit_spline(float time, const T& position0, const T& position1,
                               const T& tangent0, const T& tangent1) {

    // TODO (Animation): Task 1a
    // Given time in [0,1] compute the cubic spline coefficients and use them to compute
    // the interpolated value at time 'time' based on the positions & tangents
    float time2 = time * time;
    float time3 = time * time2;

    // Note that Spline is parameterized on type T, which allows us to create splines over
    // any type that supports the * and + operators.
    float h00 = (2 * time3) - (3 * time2) + 1;
    float h10 = (time3) - (2 * time2) + time;
    float h01 = (-2 * time3) + (3 * time2);
    float h11 = time3 - time2;

    return (h00 * position0) + (h10 * tangent0) + (h01 * position1) + (h11 * tangent1);
}

template<typename T> T Spline<T>::at(float time) const {
    // TODO (Animation): Task 1b

    // Given a time, find the nearest positions & tangent values
    // defined by the control point map.

    // Transform them for use with cubic_unit_spline

    // Be wary of edge cases! What if time is before the first knot,
    // before the second knot, etc...
    int keyCount = control_points.size();
    if(keyCount == 0) {
        return T();
    } else if(keyCount == 1) {
        return control_points.begin()->second;
    }
    auto t1it = control_points.upper_bound(time);
    int keyIdx = std::distance(control_points.begin(), t1it) - 1;
    std::pair<float, T> t0 = *(std::prev(std::prev(t1it)));
    std::pair<float, T> t1 = *(std::prev(t1it));
    std::pair<float, T> t2 = *(t1it);
    std::pair<float, T> t3 = *(std::next(t1it));
    if(keyIdx == -1) {
        return t2.second;
    } else if(t1it == control_points.end()) {
        return t1.second;
    } else {
        if(keyIdx == 0) {
            t0 = std::pair<float, T>(t1.first - (t2.first - t1.first),
                                     t1.second - (t2.second - t1.second));
        }
        if(keyIdx == (keyCount - 2)) {
            t3 = std::pair<float, T>(t2.first + (t2.first - t1.first),
                                     t2.second + (t2.second - t1.second));
        }
    }
    T m0 = (t2.second - t0.second) / (t2.first - t0.first);
    T m1 = (t3.second - t1.second) / (t3.first - t1.first);

    return cubic_unit_spline((time - t1.first) / (t2.first - t1.first), t1.second, t2.second, m0,
                             m1);
}
