#include "Perception.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>

Perception::Perception()
{
    receivedMap_=false;
    startedMCL_=false;

    numParticles_=10000;
    maxRange_ = 10.0;

    // construct a trivial random generator engine from a time-based seed:
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator_ = new std::default_random_engine(seed); 
}

///////////////////////////////////////
/// Funcoes do filtro de particulas ///
///////////////////////////////////////
void Perception::MCL_sampling(const Action &u)
{
    // Modelo de movimento baseado em odometria
    std::normal_distribution<double> rot1Dist(0.0, u.rot1 * 0.1); // Variância proporcional à magnitude da rotação
    std::normal_distribution<double> transDist(0.0, u.trans * 0.1); // Variância proporcional à magnitude da translação
    std::normal_distribution<double> rot2Dist(0.0, u.rot2 * 0.1);

    for (int i = 0; i < numParticles_; i++)
    {
        double rot1Sample = rot1Dist(*generator_) + u.rot1;
        double transSample = transDist(*generator_) + u.trans;
        double rot2Sample = rot2Dist(*generator_) + u.rot2;

        particles_[i].p.x += transSample * cos(particles_[i].p.theta + rot1Sample);
        particles_[i].p.y += transSample * sin(particles_[i].p.theta + rot1Sample);
        particles_[i].p.theta += rot1Sample + rot2Sample;

        // Garantir que theta fique entre -PI e PI
        if (particles_[i].p.theta > M_PI)
            particles_[i].p.theta -= 2.0 * M_PI;
        else if (particles_[i].p.theta < -M_PI)
            particles_[i].p.theta += 2.0 * M_PI;
    }
}

void Perception::MCL_weighting(const std::vector<float> &z)
{
    double weightSum = 0.0;
    double variance = 0.8; // Variância associada às medições, ajustável conforme necessário

    for (int i = 0; i < numParticles_; i++)
    {
        unsigned int ix = particles_[i].p.x * scale_;
        unsigned int iy = particles_[i].p.y * scale_;
        unsigned int indice = ix + iy * numCellsX_;

        // Partículas fora do espaço livre recebem peso 0
        if (gridMap_.data[indice] != 0)
        {
            particles_[i].w = 0.0;
        }
        else
        {
            // Inicializa o peso da partícula com 1
            particles_[i].w = 1.0;

            // Atualiza o peso com base nas medições do laser
            for (size_t k = 0; k < z.size(); k += 10) // Salta de 10 em 10 medições para acelerar o cálculo
            {
                double expectedZ = computeExpectedMeasurement(k, particles_[i].p);
                
                // Cálculo da probabilidade usando o modelo simplificado (distribuição Gaussiana)
                double prob = (1.0 / sqrt(2.0 * M_PI * variance)) *
                              exp(-0.5 * pow((z[k] - expectedZ), 2) / variance);

                // Multiplica a probabilidade atual pelo peso da partícula
                particles_[i].w *= prob;
            }
        }

        weightSum += particles_[i].w;
    }

    // Normaliza os pesos das partículas
    if (weightSum > 0.0)
    {
        for (int i = 0; i < numParticles_; i++)
        {
            particles_[i].w /= weightSum;
        }
    }
    else
    {
        // Se todos os pesos são zero, distribua uniformemente entre as partículas
        for (int i = 0; i < numParticles_; i++)
        {
            particles_[i].w = 1.0 / numParticles_;
        }
    }
}


void Perception::MCL_resampling()
{
    std::vector<Particle> nextGeneration;
    nextGeneration.resize(numParticles_);

    // Amostragem de baixa variância
    std::uniform_real_distribution<double> samplerU(0.0, 1.0 / numParticles_);
    double r = samplerU(*generator_);
    double c = particles_[0].w;
    int i = 0;

    for (int m = 0; m < numParticles_; m++)
    {
        double U = r + m * (1.0 / numParticles_);
        while (U > c)
        {
            i++;
            c += particles_[i].w;
        }
        nextGeneration[m] = particles_[i]; // Seleciona a partícula
    }

    particles_ = nextGeneration; // Substitui o conjunto atual pelo reamostrado
}


/////////////////////////////////////////////////////////////////////////////
/// Funcoes de inicializacao e funcoes auxiliares do filtro de particulas ///
/////////////////////////////////////////////////////////////////////////////

void Perception::MCL_initialize()
{

    int minKnownX_, minKnownY_, maxKnownX_, maxKnownY_;
    minKnownX_ = numCellsX_-1;
    minKnownY_ = numCellsY_-1;
    maxKnownX_ = maxKnownY_ = 0;

    // Update known limits
    for(int x=0; x<numCellsX_; x++){
        for(int y=0; y<numCellsY_; y++){
            unsigned int i = x + y*numCellsX_;
            if(gridMap_.data[i]!=-1)
            {
                if(x<minKnownX_) minKnownX_ = x;
                if(y<minKnownY_) minKnownY_ = y;
                if(x>maxKnownX_) maxKnownX_ = x;
                if(y>maxKnownY_) maxKnownY_ = y;

                if(gridMap_.data[i]>-1 && gridMap_.data[i]<90)
                    gridMap_.data[i]=0;
                else
                    gridMap_.data[i]=100;
            }
        }
    }

    particles_.resize(numParticles_);

    std::uniform_real_distribution<double> randomX(minKnownX_/scale_,maxKnownX_/scale_);
    std::uniform_real_distribution<double> randomY(minKnownY_/scale_,maxKnownY_/scale_);
    std::uniform_real_distribution<double> randomTh(-M_PI,M_PI);

    // generate initial particles set
    for(int i=0; i<numParticles_; i++){

        bool valid = false;
        do{
            // sample particle pose
            particles_[i].p.x = randomX(*generator_);
            particles_[i].p.y = randomY(*generator_);
            particles_[i].p.theta = randomTh(*generator_);

            // check if particle is valid (known and not obstacle)
            unsigned int ix = particles_[i].p.x*scale_;
            unsigned int iy = particles_[i].p.y*scale_;
            if(gridMap_.data[ix + iy*numCellsX_] == 0)
                valid = true;

        }while(!valid);

        std::cout << "Particle (" << i << "): "
                  << particles_[i].p.x << ' '
                  << particles_[i].p.y << ' '
                  << RAD2DEG(particles_[i].p.theta) << std::endl;
    }


    startedMCL_=true;
}

float Perception::computeExpectedMeasurement(int rangeIndex, Pose2D &pose)
{
    double angle = pose.theta + double(90-rangeIndex)*M_PI/180.0;

    // Ray-casting using DDA
    double dist;
    double difX=cos(angle);
    double difY=sin(angle);
    double deltaX, deltaY;

    if(tan(angle)==1 || tan(angle)==-1){
        deltaX=deltaY=1.0;
        dist = difX*maxRange_;
    }else if(difX*difX > difY*difY){
        deltaX=1.0;
        deltaY=difY/difX;
        dist = difX*maxRange_;
    }else{
        deltaX=difX/difY;
        deltaY=1.0;
        dist = difY*maxRange_;
    }

    if(deltaX*difX < 0.0)
        deltaX = -deltaX;
    if(deltaY*difY < 0.0)
        deltaY = -deltaY;
    if(dist < 0.0)
        dist = -dist;

    dist *= scale_;

    double i=pose.x*scale_;
    double j=pose.y*scale_;
    for(int k=0;k<(int)(dist);k++){

        if(gridMap_.data[(int)i + (int)j*numCellsX_] == 100){
            // the real obstacle is one step ahead due to wall thickening
            return sqrt(pow(pose.x*scale_-(i+deltaX),2)+pow(pose.y*scale_-(j+deltaY),2))/scale_;
        }

        i+=deltaX;
        j+=deltaY;
    }

    return maxRange_;
}

void Perception::MCL_updateParticles(geometry_msgs::msg::PoseArray& msg_particles, rclcpp::Time now)
{
    msg_particles.header.frame_id="map";
    msg_particles.header.stamp = now;

    msg_particles.poses.resize(numParticles_);
    for(int i=0; i<numParticles_; i++){
        msg_particles.poses[i].position.x = particles_[i].p.x + mapOrigin_.x;
        msg_particles.poses[i].position.y = particles_[i].p.y + mapOrigin_.y;

        tf2::Quaternion quat_tf;
        quat_tf.setRPY( 0, 0, particles_[i].p.theta );
        msg_particles.poses[i].orientation = tf2::toMsg(quat_tf);
    }
}

/////////////////////////////////////////////////
/// Callbacks dos topicos do LASER e do SONAR ///
/////////////////////////////////////////////////

void Perception::receiveLaser(const sensor_msgs::msg::LaserScan::ConstSharedPtr &value)
{
    //  STRUCTURE OF sensor_msgs::msg::LaserScan

    // Single scan from a planar laser range-finder
    // 
    // If you have another ranging device with different behavior (e.g. a sonar
    // array), please find or create a different message, since applications
    // will make fairly laser-specific assumptions about this data

    // Header header
    //     # Standard metadata for higher-level stamped data types.
    //     # This is generally used to communicate timestamped data
    //     # in a particular coordinate frame.
    //     #
    //     # sequence ID: consecutively increasing ID
    //     uint32 seq
    //     #Two-integer timestamp that is expressed as:
    //     # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    //     # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    //     # time-handling sugar is provided by the client library
    //     time stamp
    //     #Frame this data is associated with
    //     # 0: no frame
    //     # 1: global frame
    //     string frame_id
    //              # timestamp in the header is the acquisition time of
    //              # the first ray in the scan.
    //              #
    //              # in frame frame_id, angles are measured around
    //              # the positive Z axis (counterclockwise, if Z is up)
    //              # with zero angle being forward along the x axis
    laserROS_.header = value->header;

    // float32 angle_min        # start angle of the scan [rad]
    // float32 angle_max        # end angle of the scan [rad]
    // float32 angle_increment  # angular distance between measurements [rad]
    laserROS_.angle_min = value->angle_min;
    laserROS_.angle_max = value->angle_max;
    laserROS_.angle_increment = value->angle_increment;

    // float32 time_increment   # time between measurements [seconds] - if your scanner
    //                          # is moving, this will be used in interpolating position
    //                          # of 3d points
    // float32 scan_time        # time between scans [seconds]
    laserROS_.time_increment = value->time_increment;
    laserROS_.scan_time = value->scan_time;

    // float32 range_min        # minimum range value [m]
    // float32 range_max        # maximum range value [m]
    laserROS_.range_min = value->range_min;
    laserROS_.range_max = value->range_max;

    // float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
    // float32[] intensities    # intensity data [device-specific units].  If your
    //                          # device does not provide intensities, please leave
    //                          # the array empty.
    laserROS_.ranges = value->ranges;
    laserROS_.intensities = value->intensities;
}

std::vector<float> Perception::getLatestLaserRanges()
{
    int numLasers = laserROS_.ranges.size();

    std::vector<float> lasers(numLasers);

    //    std::cout << "LASER: " << numLasers << std::endl;
    for (int i = 0; i < numLasers; i++)
    {
        lasers[i] = laserROS_.ranges[numLasers - i - 1];
        if (lasers[i] < 0)
            lasers[i] = 32.0; // max range from rosaria
    }

    return lasers;
}

void Perception::receiveGridmap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &value)
{
    // STRUCTURE OF nav_msgs::OccupancyGrid
    // # This represents a 2-D grid map, in which each cell represents the probability of occupancy.
    // Header header 
    // 
    // # MetaData for the map
    //   # The time at which the map was loaded
    //   time map_load_time
    //   # The map resolution [m/cell]
    //   float32 resolution
    //   # Map width [cells]
    //   uint32 width
    //   # Map height [cells]
    //   uint32 height
    //   # The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map.
    //   geometry_msgs/Pose origin
    // MapMetaData info
    //
    // # The map data, in row-major order, starting with (0,0).  
    // # Occupancy probabilities are in the range [0,100].  Unknown is -1.
    // # OBS: implemented in c++ with std::vector<u_int8>
    // int8[] data

    if(receivedMap_==false){
        gridMap_.header = value->header;
        gridMap_.info = value->info;
        gridMap_.data = value->data;

        numCellsX_ = value->info.width;
        numCellsY_ = value->info.height;

        float cellSize = value->info.resolution;
        scale_ = 1.0/cellSize;

        mapWidth_ = numCellsX_*cellSize;
        mapHeight_ = numCellsY_*cellSize;

        mapOrigin_.x = value->info.origin.position.x;
        mapOrigin_.y = value->info.origin.position.y;

        receivedMap_=true;
    }

}

bool Perception::hasReceivedMap()
{
    return receivedMap_;
}

bool Perception::hasStartedMCL()
{
    return startedMCL_;
}