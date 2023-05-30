#include "rummy_simulation.hpp"

#include <tools/random/random_generator.hpp>
#include <numeric>

#ifdef USE_TBB
#include <tbb/tbb.h>
#include <oneapi/tbb/parallel_for.h>
#endif

namespace simu {
    namespace simulation {
        RummySimulation::RummySimulation(defaults::RummySimuParams params)
            : Simulation(params), _params(params)
        {
            reinit();
        }

        void RummySimulation::reinit()
        {
            _iteration = 0;
            _is_kicking = false;

            _fish.clear();
            if (_fish.size() != _params.num_fish) {
                _fish.resize(static_cast<size_t>(_params.num_fish));
                for (size_t i = 0; i < _fish.size(); ++i) {
                    _fish[i] = std::make_shared<RummyIndividual>(i);
                }
            }
            else {
                for (size_t i = 0; i < _fish.size(); ++i) {
                    _fish[i]->reinit();
                }
            }

            // log the initial position
            _update_stats(std::make_shared<RummySimulation>(*this));
        }

        void RummySimulation::spin_once()
        {
            _t_next_kick = std::numeric_limits<float>::max();
            for (size_t i = 0; i < _fish.size(); ++i) {
                if ((_fish[i]->t0() + _fish[i]->kick_duration()) < _t_next_kick) {
                    _t_next_kick = _fish[i]->t0() + _fish[i]->kick_duration();
                    _next_kicker_idx = i;
                }
            }

            _is_kicking = false;

            // no fish kicking yet
            if ((_iteration + 1) * _sim_settings.timestep <= _t_next_kick) {
                for (size_t i = 0; i < _fish.size(); ++i) {
                    _fish[i]->coast(std::make_shared<RummySimulation>(*this));
                }
            }
            // compute new kick
            else {
                _is_kicking = true;
                for (size_t i = 0; i < _fish.size(); ++i) {
                    _fish[i]->prepare_kick(std::make_shared<RummySimulation>(*this));
                }
                _fish[_next_kicker_idx]->kick(std::make_shared<RummySimulation>(*this));
            }

            // update statistics
            _update_stats(std::make_shared<RummySimulation>(*this));
            _update_descriptors(std::make_shared<RummySimulation>(*this));

            if (!_is_kicking) {
                // if ((_iteration % 10000) == 0) {
                //     std::cout << "Iteration: " << _iteration << std::endl;
                // }

                Simulation::spin_once();
            }
        }

        std::vector<RummyIndividualPtr> RummySimulation::fish() const { return _fish; }
        std::vector<RummyIndividualPtr>& RummySimulation::fish() { return _fish; }

        float RummySimulation::t_next_kick() const
        {
            return _t_next_kick;
        }

        const defaults::RummySimuParams RummySimulation::params() const
        {
            return _params;
        }

        defaults::RummySimuParams& RummySimulation::params()
        {
            return _params;
        }

        int RummySimulation::next_kicker_idx() const
        {
            return _next_kicker_idx;
        }

        bool RummySimulation::is_kicking() const
        {
            return _is_kicking;
        }

    } // namespace simulation
} // namespace simu
