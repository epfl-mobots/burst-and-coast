#include "rummy_individual.hpp"
#include "rummy_simulation.hpp"

#include <cmath>

namespace simu {
    namespace simulation {

        RummyIndividual::RummyIndividual()
            : _is_kicking(false), _time(0)
        {
        }

        RummyIndividual::~RummyIndividual() {}

        void RummyIndividual::stimulate(const std::shared_ptr<Simulation> sim)
        {
            auto rsim = std::static_pointer_cast<RummySimulation>(sim);

            if (_is_kicking) {
                _time += _kick_duration;
                _desired_position.x = _position.x + _kick_length * std::cos(_angular_direction);
                _desired_position.y = _position.y + _kick_length * std::sin(_angular_direction);

                _desired_speed.vx = (_desired_position.x - _position.x) / _kick_duration;
                _desired_speed.vy = (_desired_position.y - _position.y) / _kick_duration;
            }
            else {
                double time_diff = rsim->fish()[rsim->kicking_idx()]->time() - _time;
                double beta = (1. - std::exp(-time_diff / rsim->tau0()))
                    / (1. - std::exp(-_kick_duration / rsim->tau0()));
                _desired_position.x = _position.x + _kick_length * beta * std::cos(_angular_direction);
                _desired_position.y = _position.y + _kick_length * beta * std::sin(_angular_direction);

                _desired_speed.vx = (_desired_position.x - _position.x) / rsim->fish()[rsim->kicking_idx()]->kick_duration();
                _desired_speed.vy = (_desired_position.y - _position.y) / rsim->fish()[rsim->kicking_idx()]->kick_duration();
            }
        }

        void RummyIndividual::interact(const std::shared_ptr<Simulation> sim)
        {
            auto rsim = std::static_pointer_cast<RummySimulation>(sim);
            int num_fish = rsim->fish().size();

            // kicker advancing to the new position
            _position = _desired_position;
            _speed = _desired_speed;

            // computing the state for the focal individual
            // distances -> distances to neighbours
            // perception -> angle of focal individual compared to neighbours
            // thetas -> angles to center
            // phis -> relative bearing difference
            Eigen::VectorXd distances, perception, thetas, phis;
            std::tie(distances, perception, thetas, phis) = compute_state(rsim->fish());

            // indices to nearest neighbours
            std::vector<int> nn_idcs = sort_neighbours(distances, rsim->kicking_idx(), Order::INCREASING);

            // compute influence from the environment to the focal fish
            Eigen::VectorXd influence = Eigen::VectorXd::Zero(num_fish);
            for (int i = 0; i < num_fish; ++i) {
                if (i == rsim->kicking_idx())
                    continue;

                double attraction = wall_distance_attractor(distances(i), rsim->radius())
                    * wall_perception_attractor(perception(i))
                    * wall_angle_attractor(phis(i));

                double alignment = alignment_distance_attractor(distances(i), rsim->radius())
                    * alignment_perception_attractor(perception(i))
                    * alignment_angle_attractor(phis(i));

                influence(i) = std::abs(attraction + alignment);
            }

            // indices to highly influential individuals
            std::vector<int> inf_idcs = sort_neighbours(influence, rsim->kicking_idx(), Order::DECREASING);

            // in case the influence from neighbouring fish is insignificant,
            // then use the nearest neighbours
            double inf_sum = std::accumulate(influence.data(), influence.data() + influence.size(), 0.);
            std::vector<int> idcs = inf_idcs;
            if (inf_sum < 1.0e-6)
                idcs = nn_idcs;

            // step using the model
            double r_w, theta_w;
            std::tie(r_w, theta_w) = model_stepper(rsim->radius());

            double qx, qy;
            do {
                stepper(sim); // decide on the next kick length, duration, peak velocity
                free_will(sim, {distances, perception, thetas, phis}, {r_w, theta_w}, idcs); // throw in some free will

                // rejection test -- don't want to hit the wall
                qx = _desired_position.x + (_kick_length + rsim->body_length()) * std::cos(_angular_direction);
                qy = _desired_position.y + (_kick_length + rsim->body_length()) * std::sin(_angular_direction);
            } while (std::sqrt(qx * qx + qy * qy) > rsim->radius());
        }

        void RummyIndividual::move(const std::shared_ptr<Simulation> sim)
        {
            _speed = _desired_speed;
        }

        state_t RummyIndividual::compute_state(const std::vector<RummyIndividualPtr>& fish) const
        {
            size_t num_fish = fish.size();

            Eigen::VectorXd distances(num_fish);
            Eigen::VectorXd perception(num_fish);
            Eigen::VectorXd thetas(num_fish);
            Eigen::VectorXd phis(num_fish);

            for (uint i = 0; i < num_fish; ++i) {
                distances(i) = std::sqrt(
                    std::pow(_desired_position.x - fish[i]->desired_position().x, 2)
                    + std::pow(_desired_position.y - fish[i]->desired_position().y, 2));

                thetas(i) = std::atan2(
                    fish[i]->desired_position().y - _desired_position.y,
                    fish[i]->desired_position().x - _desired_position.x);

                perception(i) = angle_to_pipi(thetas(i) - _angular_direction);

                phis(i) = angle_to_pipi(fish[i]->angular_direction() - _angular_direction);
            }

            return {distances, perception, thetas, phis};
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

        void RummyIndividual::stepper(const std::shared_ptr<Simulation> sim)
        {
            auto rsim = std::static_pointer_cast<RummySimulation>(sim);
            double bb;

            bb = std::sqrt(-2. * std::log(tools::random_in_range(.0, 1.) + 1.0e-16));
            _peak_velocity = rsim->velocity_coef() * std::sqrt(2. / M_PI) * bb;

            bb = std::sqrt(-2. * std::log(tools::random_in_range(.0, 1.) + 1.0e-16));
            _kick_length = rsim->length_coef() * std::sqrt(2. / M_PI) * bb;

            bb = std::sqrt(-2. * std::log(tools::random_in_range(.0, 1.) + 1.0e-16));
            _kick_duration = rsim->time_coef() * std::sqrt(2. / M_PI) * bb;

            _kick_length = _peak_velocity * rsim->tau0() * (1. - std::exp(-_kick_duration / rsim->tau0()));
        }

        void RummyIndividual::free_will(
            const std::shared_ptr<Simulation> sim,
            const_state_t state, const std::tuple<double, double>& model_out, const std::vector<int>& idcs)
        {
            auto rsim = std::static_pointer_cast<RummySimulation>(sim);
            double r_w, theta_w;
            Eigen::VectorXd distances, perception, thetas, phis;
            std::tie(r_w, theta_w) = model_out;
            std::tie(distances, perception, thetas, phis) = state;

            double g = std::sqrt(-2. * std::log(tools::random_in_range(0., 1.) + 1.0e-16))
                * std::sin(2. * M_PI * tools::random_in_range(0., 1.));

            double q = 1. * rsim->alpha()
                * wall_distance_interaction(rsim->gamma_wall(), rsim->wall_interaction_range(), r_w, rsim->radius()) / rsim->gamma_wall();

            double dphi_rand = rsim->gamma_rand() * (1. - q) * g;
            double dphi_wall = wall_distance_interaction(rsim->gamma_wall(), rsim->wall_interaction_range(), r_w, rsim->radius())
                * wall_angle_interaction(theta_w);

            double dphi_attraction = 0;
            double dphi_ali = 0;
            for (int j = 0; j < rsim->perceived_agents(); ++j) {
                int fidx = idcs[j];
                dphi_attraction += wall_distance_attractor(distances(fidx), rsim->radius())
                    * wall_perception_attractor(perception(fidx))
                    * wall_angle_attractor(phis(fidx));
                dphi_ali += alignment_distance_attractor(distances(fidx), rsim->radius())
                    * alignment_perception_attractor(perception(fidx))
                    * alignment_angle_attractor(phis(fidx));
            }

            double dphi = dphi_rand + dphi_wall + dphi_attraction + dphi_ali;
            _angular_direction = angle_to_pipi(_angular_direction + dphi);
        }

        std::tuple<double, double> RummyIndividual::model_stepper(double radius) const
        {
            double r = std::sqrt(std::pow(_desired_position.x, 2) + std::pow(_desired_position.y, 2));
            double rw = radius - r;
            double theta = std::atan2(_desired_position.y, _desired_position.x);
            double thetaW = angle_to_pipi(_angular_direction - theta);
            return {rw, thetaW};
        }

        double RummyIndividual::wall_distance_interaction(
            double gamma_wall,
            double wall_interaction_range,
            double ag_radius,
            double radius) const
        {
            double x = std::max(0., ag_radius);
            return gamma_wall * std::exp(-std::pow(x / wall_interaction_range, 2));
        }

        double RummyIndividual::wall_angle_interaction(double theta) const
        {
            double a0 = 1.915651;
            return a0 * std::sin(theta) * (1. + .7 * std::cos(2. * theta));
        }

        double RummyIndividual::wall_distance_attractor(double distance, double radius) const
        {
            double a0 = 4.;
            double a1 = .03;
            double a2 = .2;
            return a0 * (distance - a1) / (1. + std::pow(distance / a2, 2));
        }

        double RummyIndividual::wall_perception_attractor(double perception) const
        {
            return 1.395 * std::sin(perception) * (1. - 0.33 * std::cos(perception));
        }

        double RummyIndividual::wall_angle_attractor(double phi) const
        {
            return 0.93263 * (1. - 0.48 * std::cos(phi) - 0.31 * std::cos(2. * phi));
        }

        double RummyIndividual::alignment_distance_attractor(double distance, double radius) const
        {
            double a0 = 1.5;
            double a1 = .03;
            double a2 = .2;
            return a0 * (distance + a1) * std::exp(-std::pow(distance / a2, 2));
        }

        double RummyIndividual::alignment_perception_attractor(double perception) const
        {
            return 0.90123 * (1. + .6 * std::cos(perception) - .32 * std::cos(2. * perception));
        }

        double RummyIndividual::alignment_angle_attractor(double phi) const
        {
            return 1.6385 * std::sin(phi) * (1. + .3 * std::cos(2. * phi));
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

        double RummyIndividual::time_kicker() const { return _time + _kick_duration; }

        double RummyIndividual::time() const { return _time; }
        double& RummyIndividual::time() { return _time; }

        double RummyIndividual::angular_direction() const { return _angular_direction; }
        double& RummyIndividual::angular_direction() { return _angular_direction; }

        double RummyIndividual::peak_velocity() const { return _peak_velocity; }
        double& RummyIndividual::peak_velocity() { return _peak_velocity; }

        double RummyIndividual::kick_length() const { return _kick_length; }
        double& RummyIndividual::kick_length() { return _kick_length; }

        double& RummyIndividual::kick_duration() { return _kick_duration; }
        double RummyIndividual::kick_duration() const { return _kick_duration; }

        bool& RummyIndividual::is_kicking() { return _is_kicking; }
        bool RummyIndividual::is_kicking() const { return _is_kicking; }

    } // namespace simulation
} // namespace simu
