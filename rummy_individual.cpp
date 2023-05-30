#include "rummy_individual.hpp"
#include "rummy_simulation.hpp"

#include <algorithm>
#include <cmath>
#include <tools/random/random_generator.hpp>

namespace simu {
    namespace simulation {

        float ran3()
        {
            return tools::random_in_range(0., 1.);
        }

        RummyIndividual::RummyIndividual(int id, defaults::RummyIndividualParams params) : _params(params)
        {
            _id = id;
            reinit();
        }

        void RummyIndividual::reinit()
        {
            _t0 = 0;
            _speed = _params.vmean;
            float dtau = _params.taumean - _params.taumin;

            float x, y, r, rn, phi;
            do {
                _tau = _params.taumin - dtau * 0.5 * log(ran3() * ran3());
                r = _params.radius * std::pow(ran3(), 1. / 4.);
                float theta = M_PI * (_id + 1) / 16. * ran3();
                x = r * cos(theta);
                y = r * sin(theta);
                phi = angle_to_pipi(theta + M_PI_2);

                float dli = _speed * _params.tau0 * (1. - std::exp(-_tau / _params.tau0));
                float xn = x + dli * cos(phi);
                float yn = y + dli * sin(phi);
                rn = std::sqrt(xn * xn + yn * yn);
            } while (rn > _params.radius);

            _num_jumps = 0;
            _num_kicks = 0;
            _num_uturn = 0;

            _pose.x = x;
            _pose.y = y;
            _pose.yaw = phi;
            _traj_pose = _pose;
            _speed = _params.vmean;
            _traj_speed = _speed;
        }

        RummyIndividual::~RummyIndividual() {}

        void RummyIndividual::coast(const std::shared_ptr<Simulation> sim)
        {
            auto rsim = std::static_pointer_cast<RummySimulation>(sim);

            int it0 = rsim->iteration() + 1;
            float tt0 = it0 * rsim->sim_settings().timestep;
            float dt = tt0 - _t0;
            float expt = std::exp(-dt / _params.tau0);
            float dl = _speed * _params.tau0 * (1. - expt);
            _traj_pose.x = _pose.x + dl * cos(_pose.yaw);
            _traj_pose.y = _pose.y + dl * sin(_pose.yaw);
            _traj_pose.yaw = _pose.yaw;
            _traj_speed = _speed * expt;
        }

        void RummyIndividual::prepare_kick(const std::shared_ptr<Simulation> sim)
        {
            auto rsim = std::static_pointer_cast<RummySimulation>(sim);

            float dt = rsim->t_next_kick() - _t0;
            float dl = _speed * _params.tau0 * (1. - std::exp(-dt / _params.tau0));
            _kick_pose.x = _pose.x + dl * cos(_pose.yaw);
            _kick_pose.y = _pose.y + dl * sin(_pose.yaw);
            _kick_pose.yaw = _pose.yaw;
        }

        bool RummyIndividual::kick(const std::shared_ptr<Simulation> sim)
        {
            auto rsim = std::static_pointer_cast<RummySimulation>(sim);

            ++_num_kicks;

            float v0old = _speed;
            // float vold = _speed * std::exp(-_tau / _params.tau0);
            _t0 = rsim->t_next_kick();
            _pose.x = _kick_pose.x;
            _pose.y = _kick_pose.y;
            _pose.yaw = _kick_pose.yaw;

            auto state = compute_state(rsim->fish());
            auto neighs = sort_neighbours(std::get<0>(state), _id, Order::INCREASING); // by distance

            float dphi_int, fw;
            std::tie(dphi_int, fw) = compute_interactions(state, neighs, sim);

            /**
             *
             * Step the model, this is the actual "movement" of the fish
             *
             **/

            int iter = 0;
            float phi_new, r_new;
            do {
                float theta = std::atan2(_kick_pose.y, _kick_pose.x);
                float theta_w = angle_to_pipi(_kick_pose.yaw - theta);

                do {
                    float prob = ran3();
                    if (prob < _params.vmem) {
                        if (prob < _params.vmem12) {
                            // auto most_inf_neigh = rsim->fish()[neighs[0]];
                            // _speed = most_inf_neigh->speed() * _params.coeff_peak_v;
                            _speed = 0.;
                            for (size_t idx : neighs) {
                                _speed += rsim->fish()[idx]->speed();
                            }
                            _speed /= neighs.size();
                        }
                        else {
                            _speed = v0old;
                        }
                    }
                    else {
                        _speed = _params.vmin + _params.vmean * (-log(ran3() * ran3() * ran3())) / 3.;
                    }
                } while (_speed > _params.vcut); // speed
                _speed = std::max(_speed, _params.vmin);

                float dtau = _params.taumean - _params.taumin;
                _tau = _params.taumin - dtau * 0.5 * log(ran3() * ran3());

                // cognitive noise
                // float gauss = std::sqrt(-2. * log(ran3())) * cos(2 * M_PI * ran3());
                float gauss = tools::normal_random(0., 1.);
                phi_new = _pose.yaw + dphi_int + _params.gamma_rand * gauss * (1. - _params.alpha_w * fw);
                float dl = _speed * _params.tau0 * (1. - std::exp(-_tau / _params.tau0)) + _params.dc;
                // float dl = _speed * _tau + _params.dc;
                float x_new = _pose.x + dl * std::cos(phi_new);
                float y_new = _pose.y + dl * std::sin(phi_new);
                r_new = std::sqrt(x_new * x_new + y_new * y_new);

                if (++iter > _params.itermax) {
                    iter = 0;
                    float dphiplus = 0.1 * (-log(ran3()));
                    if (theta_w > 0) {
                        _kick_pose.yaw = (theta + M_PI_2 + dphiplus);
                    }
                    else {
                        _kick_pose.yaw = (theta - M_PI_2 - dphiplus);
                    }
                    std::tie(dphi_int, fw) = compute_interactions(state, neighs, sim);
                }

            } while (r_new > _params.radius); // radius

            _pose.yaw = angle_to_pipi(phi_new);

            return true;
        }

        void RummyIndividual::stimulate(const std::shared_ptr<Simulation> sim) {}
        void RummyIndividual::move(const std::shared_ptr<Simulation> sim) {}

        std::tuple<float, float> RummyIndividual::compute_interactions(const state_t& state, std::vector<int> neighs, const std::shared_ptr<Simulation> sim)
        {
            auto rsim = std::static_pointer_cast<RummySimulation>(sim);

            /**
             *
             * Compute interactions, i.e., attraction/repulsion between the current individual and its neighbours.
             *
             **/

            float theta = std::atan2(_kick_pose.y, _kick_pose.x);
            float rw = rsim->params().radius - std::sqrt(_kick_pose.x * _kick_pose.x + _kick_pose.y * _kick_pose.y);

            float dphiw = 0.;
            float fw;

            // wall interaction
            float theta_w = angle_to_pipi(_kick_pose.yaw - theta);
            fw = std::exp(-std::pow(rw / _params.dw, 2));
            float ow = std::sin(theta_w) * (1. + 0.7 * std::cos(2. * theta_w));
            dphiw = _params.gamma_wall * fw * ow;

            float dphiwsym = _params.gamma_sym * std::sin(2. * theta_w) * std::exp(-rw / _params.dw) * rw / _params.dw;
            float dphiwasym = _params.gamma_asym * std::cos(theta_w) * std::exp(-rw / _params.dw) * rw / _params.dw;

            dphiw += dphiwsym + dphiwasym;

            // fish interaction
            std::vector<float> dphi_fish;
            for (int j : neighs) {
                float dij = std::get<0>(state)[j];
                float psi_ij = std::get<1>(state)[j];
                float dphi_ij = std::get<3>(state)[j];

                float fatt = (dij - 3.) / 3. / (1. + std::pow(dij / 20., 2));
                float oatt = std::sin(psi_ij) * (1. - 0.33 * std::cos(psi_ij));
                // float eatt = 1. - 0.48 * std::cos(dphi_ij) - 0.31 * std::cos(2. * dphi_ij);
                // float oatt = std::sin(psi_ij) * (1. + std::cos(psi_ij));
                float eatt = 1.;
                float dphiatt = _params.gamma_attraction * fatt * oatt * eatt;

                // float fali = (dij + 3. ) / 6. * std::exp(-std::pow(dij / 20., 2));
                float fali = std::exp(-std::pow(dij / 20., 2));
                float oali = std::sin(dphi_ij) * (1. + 0.3 * std::cos(2. * dphi_ij));
                float eali = 1. + 0.6 * std::cos(psi_ij) - 0.32 * std::cos(2. * psi_ij);
                float dphiali = _params.gamma_alignment * fali * oali * eali;

                dphi_fish.push_back(dphiatt + dphiali);
            }

            _most_inf_idcs.clear();
            _most_inf_idcs = neighs;
            if (!_params.use_closest_individual) {
                std::sort(_most_inf_idcs.begin(), _most_inf_idcs.end(),
                    [&](size_t lidx, size_t ridx) -> bool {
                        return std::abs(dphi_fish[lidx]) < std::abs(dphi_fish[ridx]);
                    });
                std::sort(dphi_fish.begin(), dphi_fish.end(), [](const float& lv, const float& rv) {
                    return std::abs(lv) > std::abs(rv);
                });
            }

            size_t offset = std::min(dphi_fish.size(), static_cast<size_t>(_params.perceived_agents));
            float dphi_f = std::accumulate(dphi_fish.begin(), dphi_fish.begin() + offset, 0.);
            float dphi_int = dphiw + dphi_f;

            return {dphi_int, fw};
        }

        state_t RummyIndividual::compute_state(const std::vector<RummyIndividualPtr>& fish) const
        {
            size_t num_fish = fish.size();

            Eigen::VectorXd distances(num_fish);
            Eigen::VectorXd psis_ij(num_fish);
            Eigen::VectorXd psis_ji(num_fish);
            Eigen::VectorXd dphis(num_fish);

            for (uint i = 0; i < num_fish; ++i) {
                distances(i) = std::sqrt(
                    std::pow(_kick_pose.x - fish[i]->kick_pose().x, 2)
                    + std::pow(_kick_pose.y - fish[i]->kick_pose().y, 2));

                psis_ij(i) = std::atan2(fish[i]->kick_pose().y - _kick_pose.y,
                                 fish[i]->kick_pose().x - _kick_pose.x)
                    - _kick_pose.yaw;

                psis_ji(i) = std::atan2(_kick_pose.y - fish[i]->kick_pose().y,
                                 _kick_pose.x - fish[i]->kick_pose().x)
                    - fish[i]->kick_pose().yaw;

                dphis(i) = fish[i]->kick_pose().yaw - _kick_pose.yaw;
            }

            return {distances, psis_ij, psis_ji, dphis};
        }

        std::vector<int> RummyIndividual::sort_neighbours(
            const Eigen::VectorXd& values, const int kicker_idx, Order order) const
        {
            std::vector<int> neigh_idcs;
            for (int i = 0; i < values.rows(); ++i) {
                if (i == kicker_idx)
                    continue;
                neigh_idcs.push_back(i);
            }

            std::sort(std::begin(neigh_idcs), std::end(neigh_idcs), [&](int lhs, int rhs) {
                return (order == Order::INCREASING) ? (values(lhs) < values(rhs))
                                                    : (values(lhs) > values(rhs));
            });

            return neigh_idcs;
        }

        double RummyIndividual::angle_to_pipi(double difference) const
        {
            do {
                if (difference < -M_PI)
                    difference += 2. * M_PI;
                if (difference > M_PI)
                    difference -= 2. * M_PI;
            } while (std::abs(difference) > M_PI);
            return difference;
        }

        float RummyIndividual::kick_duration() const
        {
            return _tau;
        }

        float RummyIndividual::t0() const
        {
            return _t0;
        }

        float& RummyIndividual::t0()
        {
            return _t0;
        }

        float RummyIndividual::tau() const
        {
            return _tau;
        }

        float& RummyIndividual::tau()
        {
            return _tau;
        }

        Pose2d<float> RummyIndividual::kick_pose() const
        {
            return _kick_pose;
        }

        float RummyIndividual::speed() const
        {
            return _speed;
        }

        Pose2d<float> RummyIndividual::pose() const
        {
            return _pose;
        }

        Pose2d<float>& RummyIndividual::pose()
        {
            return _pose;
        }

        Pose2d<float> RummyIndividual::traj_pose() const
        {
            return _traj_pose;
        }

        float RummyIndividual::traj_speed() const
        {
            return _traj_speed;
        }

        const defaults::RummyIndividualParams& RummyIndividual::params() const
        {
            return _params;
        }

        defaults::RummyIndividualParams& RummyIndividual::params()
        {
            return _params;
        }

    } // namespace simulation
} // namespace simu
