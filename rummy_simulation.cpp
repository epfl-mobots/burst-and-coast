#include "rummy_simulation.hpp"

#include <tools/random/random_generator.hpp>

namespace simu {
    namespace simulation {

        RummySimulation::RummySimulation(bool stats_enable) : Simulation(stats_enable) { _init(); }

        void RummySimulation::_init()
        {
            reinit();

            Eigen::VectorXd kick_len_ref(_rummy_fish_settings.reference_sample_size);
            Eigen::VectorXd peak_vel_ref(_rummy_fish_settings.reference_sample_size);
            Eigen::VectorXd tau_ref(_rummy_fish_settings.reference_sample_size);

            RummyIndividual reference_fish;
            for (int i = 0; i < _rummy_fish_settings.reference_sample_size; ++i) {
                reference_fish.stepper(std::make_shared<RummySimulation>(*this));
                kick_len_ref(i) = reference_fish.kick_length();
                peak_vel_ref(i) = reference_fish.peak_velocity();
                tau_ref(i) = reference_fish.kick_duration();
            }

            std::cout << "ln: " << kick_len_ref.minCoeff()
                      << " < " << kick_len_ref.sum() / _rummy_fish_settings.reference_sample_size
                      << " < " << kick_len_ref.maxCoeff() << std::endl;
            std::cout << "vn: " << peak_vel_ref.minCoeff()
                      << " < " << peak_vel_ref.sum() / _rummy_fish_settings.reference_sample_size
                      << " < " << peak_vel_ref.maxCoeff() << std::endl;
            std::cout << "taun: " << tau_ref.minCoeff()
                      << " < " << tau_ref.sum() / _rummy_fish_settings.reference_sample_size
                      << " < " << tau_ref.maxCoeff() << std::endl;
            std::cout << std::endl;
        }

        void RummySimulation::reinit()
        {
            // init model parameters
            _num_timesteps = _rummy_fish_settings.num_timesteps;
            _num_fish = _rummy_fish_settings.num_fish;
            _radius = _rummy_fish_settings.radius;

            _perceived_agents = _rummy_fish_settings.perceived_agents;
            _gamma_rand = _rummy_fish_settings.gamma_rand;
            _wall_interaction_range = _rummy_fish_settings.wall_interaction_range;
            _gamma_wall = _rummy_fish_settings.gamma_wall * std::exp(-std::pow(std::max(.0, _radius) / _wall_interaction_range, 2));
            _body_length = _rummy_fish_settings.body_length;

            _alpha = _rummy_fish_settings.alpha;
            _tau0 = _rummy_fish_settings.tau0;

            _velocity_coef = _rummy_fish_settings.velocity_coef;
            _length_coef = _rummy_fish_settings.length_coef;
            _time_coef = _rummy_fish_settings.time_coef;

            // init members
            _num_kicks = Eigen::VectorXd::Zero(_num_fish);

            // update sim related vars
            _sim_settings.sim_time = _num_timesteps;

            if (_perceived_agents >= _num_fish) {
                std::cout << "Correcting number of perceived agents to N-1" << std::endl;
                _perceived_agents = _num_fish - 1;
            }

            _fish.clear();
            _fish.resize(static_cast<size_t>(_num_fish));
            for (size_t i = 0; i < _fish.size(); ++i) {
                _fish[i] = std::make_shared<RummyIndividual>();
                _fish[i]->id() = static_cast<int>(i);

                // _fish[i]->position().x = .5 * tools::random_in_range(-.5, .5) * _radius;
                // _fish[i]->position().y = -.5 * tools::random_in_range(-.5, .5) * _radius;
                // _fish[i]->phi() = 2. * tools::random_in_range(-.5, .5) * M_PI;
                // assert(_fish[i]->position().x < _radius);

                _fish[i]->stepper(std::make_shared<RummySimulation>(*this));
                _fish[i]->position().x = -(i - 1. - _num_fish / 2) * _body_length;
                _fish[i]->angular_direction() = i * 2. * M_PI / (_num_fish + 1);
                _fish[i]->position().y = -0.1;
            }

            // log the initial position
            _update_stats(std::make_shared<RummySimulation>(*this));
        }

        void RummySimulation::spin_once()
        {
            // reset kickers
            for (uint i = 0; i < _fish.size(); ++i)
                _fish[i]->is_kicking() = false;

            // decide which individual is the next one to kick
            _kicking_idx = _fish[0]->id();
            double tkicker = _fish[0]->time_kicker();
            for (uint i = 1; i < _fish.size(); ++i) {
                if (_fish[i]->time_kicker() < tkicker) {
                    _kicking_idx = i;
                    tkicker = _fish[i]->time_kicker();
                }
            }
            _fish[_kicking_idx]->is_kicking() = true;
            ++_num_kicks(_kicking_idx);

            // the individuals decide on the desired position
            _fish[_kicking_idx]->stimulate(std::make_shared<RummySimulation>(*this)); // kicking individual goes first
            for (size_t i = 0; i < _fish.size(); ++i) {
                if (static_cast<int>(i) == _kicking_idx)
                    continue;
                _fish[i]->stimulate(std::make_shared<RummySimulation>(*this));
            }

            // apply attractors/repulsors and update the fish intuitions
            // (for the kicker, the rest are in their gliding phase)
            _fish[_kicking_idx]->interact(std::make_shared<RummySimulation>(*this));

            // update position and velocity information -- actual move step
            for (uint i = 0; i < _fish.size(); ++i)
                _fish[i]->move(std::make_shared<RummySimulation>(*this));

            // update statistics
            _update_stats(std::make_shared<RummySimulation>(*this));
            _update_descriptors(std::make_shared<RummySimulation>(*this));

            Simulation::spin_once();
        }

        std::vector<RummyIndividualPtr> RummySimulation::fish() const { return _fish; }
        std::vector<RummyIndividualPtr>& RummySimulation::fish() { return _fish; }

        RummyFishSettings& RummySimulation::rummy_fish_settings() { return _rummy_fish_settings; }
        RummyFishSettings RummySimulation::rummy_fish_settings() const { return _rummy_fish_settings; }

        int RummySimulation::num_timesteps() const { return _num_timesteps; }
        int RummySimulation::num_fish() const { return _num_fish; }
        double RummySimulation::radius() const { return _radius; }

        int RummySimulation::perceived_agents() const { return _perceived_agents; }
        double RummySimulation::gamma_rand() const { return _gamma_rand; }
        double RummySimulation::gamma_wall() const { return _gamma_wall; }
        double RummySimulation::wall_interaction_range() const { return _wall_interaction_range; }
        double RummySimulation::body_length() const { return _body_length; }

        double RummySimulation::alpha() const { return _alpha; }
        double RummySimulation::tau0() const { return _tau0; }
        double RummySimulation::velocity_coef() const { return _velocity_coef; }
        double RummySimulation::length_coef() const { return _length_coef; }
        double RummySimulation::time_coef() const { return _time_coef; }

        int RummySimulation::kicking_idx() const { return _kicking_idx; }

    } // namespace simulation
} // namespace simu
