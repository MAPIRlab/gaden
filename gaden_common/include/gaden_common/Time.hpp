#pragma once
#include <chrono>
#include <cmath>

namespace gaden::Utils::Time
{
    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::_V2::system_clock::duration Duration;
    typedef std::chrono::_V2::system_clock::time_point TimePoint;
    
    inline double toSeconds(Duration duration)
    {
        constexpr double nanoToSec = 1e-9;
        return duration.count() * nanoToSec ;
    }

    struct Stopwatch
    {
        Clock clock;
        TimePoint start;

        Stopwatch() : clock(), start(clock.now())
        {
        }

        double ellapsed()
        {
            return toSeconds(clock.now() - start);
        }

        void restart()
        {
            start = clock.now();
        }
    };

    struct Countdown
    {
    public:

        Countdown(): clock(), start(clock.now()), length(0)
        {}

        Countdown(double _length) : clock(), start(clock.now()), length(_length)
        {}

        void Restart()
        {
            start = clock.now();
        }

        void Restart(double _length)
        {
            start = clock.now();
            length = _length;
        }

        bool isDone()
        {
            return toSeconds(clock.now() - start) >= length;
        }

        float timeRemaining()
        {
            return toSeconds(clock.now() - start);
        }

        float proportionComplete()
        {
            if (length <= 0)
                return 1;
            else
                return timeRemaining() / length;
        }

    private:
        Clock clock;
        TimePoint start;
        double length;
    };
} // namespace GSL::Utils::Time