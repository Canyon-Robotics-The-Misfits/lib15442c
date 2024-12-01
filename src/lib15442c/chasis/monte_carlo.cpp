#include "lib15442c/chasis/odometry.hpp"

#define FIELD_WIDTH 144

lib15442c::MCLOdom::MCLOdom(MCLConfig config, std::shared_ptr<TrackerOdom> tracker_odom, std::vector<MCLSensorParams> sensor_params)
    : tracker_odom(tracker_odom),
      uniform_random_percent(config.uniform_random_percent), tracker_odom_sd(config.tracker_odom_sd)
{
    // random seed stolen from vexcode
    int systemTime = pros::micros();
    double batteryCurrent = pros::c::battery_get_current();
    double batteryVoltage = pros::c::battery_get_voltage();

    int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

    rng.seed(seed);
    
    for (MCLSensorParams params : sensor_params)
    {
        sensors.push_back(MCLSensor {
            sensor : std::make_shared<pros::Distance>(params.port),
            x_offset : params.x_offset,
            y_offset : params.y_offset,
            theta_offset : params.theta_offset
        });
    }
}

lib15442c::MCLOdom::~MCLOdom()
{
    stop_task();
}

void lib15442c::MCLOdom::initialize(double initial_x, double initial_y, Angle initial_theta)
{
    start_task(initial_x, initial_y, initial_theta);
}

double lib15442c::MCLOdom::sensor_sd(double distance)
{
    double variance = std::max(distance * 0.05, 0.590551);

    return variance / 3;
}

double lib15442c::MCLOdom::get_particle_chance(double x, double y, MCLSensor sensor, double theta)
{
    double distance = sensor.sensor->get_distance() * 0.0393701; // convert mm to in
    double x_offset = sensor.y_offset * cos(theta) + sensor.x_offset * cos(theta - M_PI / 2.0);
    double y_offset = sensor.y_offset * sin(theta) + sensor.x_offset * sin(theta - M_PI / 2.0);

    double x_predict = (cos(theta) > 0 ? FIELD_WIDTH - distance * cos(theta) : distance * abs(cos(theta))) + x_offset;
    double y_predict = (sin(theta) > 0 ? FIELD_WIDTH - distance * sin(theta) : distance * abs(sin(theta))) + y_offset;

    return std::max(gaussian_distribution(x, x_predict, sensor_sd(distance)),
                    gaussian_distribution(y, y_predict, sensor_sd(distance)));
}

double lib15442c::MCLOdom::random()
{
    return (double)rng() / (double)rng.max();
}

// https://stackoverflow.com/questions/25582882/javascript-math-random-normal-distribution-gaussian-bell-curve
double lib15442c::MCLOdom::gaussian_random(double mean, double sd)
{
    double u = 1 - random();
    double v = random();
    double z = sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);

    return z * sd + mean;
}

double lib15442c::MCLOdom::gaussian_distribution(double x, double mean, double sd)
{
    double EPSILON = 0.0000000000000000001;

    return std::max(
        pow(M_E, -1.0 / 2.0 * pow((x - mean) / sd, 2)) / (sd * sqrt(2 * M_PI)),
        EPSILON);
}

void lib15442c::MCLOdom::motion_update()
{
    position_mutex.lock();
    double delta_x = tracker_odom->get_x() - last_x;
    double delta_y = tracker_odom->get_y() - last_y;

    for (MCLParticle particle : particles)
    {
        particle.x += gaussian_random(delta_x, 0.1);
        particle.y += gaussian_random(delta_y, 0.1);
    }

    last_x = tracker_odom->get_x();
    last_y = tracker_odom->get_y();
    position_mutex.unlock();
}

void lib15442c::MCLOdom::resample()
{
    position_mutex.lock();

    double max_weight = 0;
    for (MCLParticle particle : particles)
    {
        if (particle.weight > max_weight)
        {
            max_weight = particle.weight;
        }
    }

    double UNIFORM_RANDOM_PARTICLES = uniform_random_percent * particle_count;

    std::vector<MCLParticle> new_particles;

    int index = floor(random() * particles.size());
    double beta = 0;
    for (int i = 0; i < particles.size() - UNIFORM_RANDOM_PARTICLES; i++)
    {
        beta += random() * 2 * max_weight;
        while (beta > particles[index].weight)
        {
            beta -= particles[index].weight;
            index = (index + 1) % particles.size();
        }

        new_particles.push_back(MCLParticle{
            x : particles[index].x,
            y : particles[index].y,
            weight : 1
        });
    }

    for (int i = 0; i < UNIFORM_RANDOM_PARTICLES; i++)
    {
        new_particles.push_back(MCLParticle{
            x : random() * FIELD_WIDTH,
            y : random() * FIELD_WIDTH,
            weight : 1
        });
    }

    particles = new_particles;

    position_mutex.unlock();
}

void lib15442c::MCLOdom::sensor_update()
{
    position_mutex.lock();

    for (MCLParticle particle : particles)
    {
        particle.weight = 1;
        for (MCLSensor sensor : sensors)
        {
            double theta = -tracker_odom->get_rotation().rad() + sensor.theta_offset;

            particle.weight *= get_particle_chance(particle.x, particle.y, sensor, theta);
        }
    }

    position_mutex.unlock();
}

void lib15442c::MCLOdom::set_mirrored(bool mirrored)
{
    this->mirrored = mirrored;
}

bool lib15442c::MCLOdom::get_mirrored()
{
    return mirrored;
}

double lib15442c::MCLOdom::get_x()
{
    position_mutex.lock();
    double temp = predicted_x;
    position_mutex.unlock();

    return (mirrored ? FIELD_WIDTH - temp : temp);
}

double lib15442c::MCLOdom::get_y()
{
    position_mutex.lock();
    double temp = predicted_y;
    position_mutex.unlock();

    return temp;
}

lib15442c::Vec lib15442c::MCLOdom::get_position()
{
    position_mutex.lock();
    Vec temp = Vec(predicted_x, predicted_y);
    position_mutex.unlock();

    return temp;
}
lib15442c::Pose lib15442c::MCLOdom::get_pose()
{
    position_mutex.lock();
    double x = predicted_x;
    double y = predicted_y;
    position_mutex.unlock();
    Angle rotation = get_rotation();

    return pose(x, y, rotation);
}

lib15442c::Angle lib15442c::MCLOdom::get_rotation()
{
    return tracker_odom->get_rotation() * (mirrored ? -1 : 1);
}

void lib15442c::MCLOdom::set_rotation(Angle rotation)
{
    tracker_odom->set_rotation(rotation);
}

void lib15442c::MCLOdom::start_task(double initial_x, double initial_y, Angle initial_theta)
{
    position_mutex.lock();
    predicted_x = initial_x;
    predicted_y = initial_y;

    last_x = tracker_odom->get_x();
    last_y = tracker_odom->get_y();
    position_mutex.unlock();

    tracker_odom->set_rotation(initial_theta);

    task = pros::Task(
        [initial_x, initial_y, this]
        {
            particles.push_back(MCLParticle{
                x : initial_x,
                y : initial_y,
                weight : 1
            });
            for (int i = 1; i < particle_count; i++)
            {
                particles.push_back(MCLParticle{
                    x : random() * FIELD_WIDTH,
                    y : random() * FIELD_WIDTH,
                    weight : 1
                });
            }

            auto initial_comp_state = pros::c::competition_get_status();
            while (initial_comp_state == pros::c::competition_get_status())
            {
                motion_update();
                resample();
                sensor_update();

                position_mutex.lock();
                predicted_x = 0;
                predicted_y = 0;
                double total_weight = 0;

                for (MCLParticle particle : particles)
                {
                    predicted_x += particle.x * particle.weight;
                    predicted_y += particle.y * particle.weight;
                    total_weight += particle.weight;
                }

                predicted_x /= total_weight;
                predicted_y /= total_weight;
                position_mutex.unlock();

                if (pros::Task::notify_take(true, 10) > 0)
                {
                    break;
                }
            }
        }
    );
}

void lib15442c::MCLOdom::stop_task()
{
    task.notify();
    task.join();
}