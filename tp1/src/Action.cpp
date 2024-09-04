//Eduardo André Leite 00287684
#include "Action.h"

#include "Utils.h"

#include <algorithm>

Action::Action()
{
    linVel = 0.0;
    angVel = 0.0;
}

void Action::avoidObstacles(std::vector<float> lasers, std::vector<float> sonars)
{
    // Parâmetro de controle
    float minDistThreshold = 1.0; // Distância mínima permitida para evitar obstáculos

    // Encontra a menor distância medida pelos lasers à esquerda
    float minDistLeft = *std::min_element(lasers.begin(), lasers.begin() + lasers.size() / 2);

    // Encontra a menor distância medida pelos lasers à direita
    float minDistRight = *std::min_element(lasers.begin() + lasers.size() / 2, lasers.end());

    
    // Se a distância mínima estiver abaixo do limiar
    if (minDistLeft < minDistThreshold || minDistRight < minDistThreshold) {
        float leftDist = sonars[0]; // Supondo que o primeiro sonar representa a esquerda
        float rightDist = sonars[7]; // Supondo que o segundo sonar representa a direita

        // Se houver a distância da direita for maior que da esquerda
        if (rightDist > leftDist) {
            linVel = 0.2; // Velocidade linear para frente
            angVel = -0.5; // Velocidade angular para girar à direita
        // Se houver a distância da esquerda for maior que da direita
        } else{
            linVel = 0.2; // Velocidade linear para frente
            angVel = 0.5; // Velocidade angular para girar à esquerda
        }
    } else {
        // Se não houver obstáculos próximos, continuar em linha reta
        linVel = 0.5; // Velocidade linear para frente
        angVel = 0.0; // Sem rotação angular
    }
}

void Action::wallFollowing(std::vector<float> lasers, std::vector<float> sonars)
{
    // Parâmetros de controle PID
    float desiredDistance = 0.5; // Distância desejada da parede à esquerda
    float kp = 0.32; // Ganho proporcional
    float ki = 0.000000012; // Ganho integral
    float kd = 24.0; // Ganho derivativo
    float maxCTE = 1.2; // Maximum cross-track error (adjusted)

    // Encontra a menor distância medida pelos sonares à esquerda
    float minLeftDist = *std::min_element(sonars.begin(), sonars.end());

    // Calcula o erro cruzado CTE
    float crossTrackError = desiredDistance - minLeftDist;

    // Limita o valor máximo do CTE
    crossTrackError = std::max(-maxCTE, std::min(maxCTE, crossTrackError));

    // Atualiza os termos PID
    static float integral = 0;
    static float previousError = 0;
    float derivative = crossTrackError - previousError;
    integral += crossTrackError;

    // Calcula a velocidade angular usando controle PID
    angVel = (-kp) * crossTrackError - kd * derivative - (ki * integral );

    // Define a velocidade linear para manter o robô se movendo para frente
    linVel = 0.5; // Velocidade linear constante

    // Atualiza o erro anterior para o próximo ciclo
    previousError = crossTrackError;
}

void Action::manualRobotMotion(MovingDirection direction)
{
    if(direction == FRONT){
        linVel= 0.5; angVel= 0.0;
    }else if(direction == BACK){
        linVel= -0.5; angVel= 0.0;
    }else if(direction == LEFT){
        linVel= 0.0; angVel= 0.5;
    }else if(direction == RIGHT){
        linVel= 0.0; angVel=-0.5;
    }else if(direction == STOP){
        linVel= 0.0; angVel= 0.0;
    }
}

void Action::correctVelocitiesIfInvalid()
{
    float b=0.38;

    float leftVel  = linVel - angVel*b/(2.0);
    float rightVel = linVel + angVel*b/(2.0);

    float VELMAX = 0.5;

    float absLeft = fabs(leftVel);
    float absRight = fabs(rightVel);

    if(absLeft>absRight){
        if(absLeft > VELMAX){
            leftVel *= VELMAX/absLeft;
            rightVel *= VELMAX/absLeft;
        }
    }else{
        if(absRight > VELMAX){
            leftVel *= VELMAX/absRight;
            rightVel *= VELMAX/absRight;
        }
    }
    
    linVel = (leftVel + rightVel)/2.0;
    angVel = (rightVel - leftVel)/b;
}

float Action::getLinearVelocity()
{
    return linVel;
}

float Action::getAngularVelocity()
{
    return angVel;
}

MotionControl Action::handlePressedKey(char key)
{
    MotionControl mc;
    mc.mode=MANUAL;
    mc.direction=STOP;

    if(key=='1'){
        mc.mode=MANUAL;
        mc.direction=STOP;
    }else if(key=='2'){
        mc.mode=WANDER;
        mc.direction=AUTO;
    }else if(key=='3'){
        mc.mode=WALLFOLLOW;
        mc.direction=AUTO;
    }else if(key=='w' or key=='W'){
        mc.mode=MANUAL;
        mc.direction = FRONT;
    }else if(key=='s' or key=='S'){
        mc.mode=MANUAL;
        mc.direction = BACK;
    }else if(key=='a' or key=='A'){
        mc.mode=MANUAL;
        mc.direction = LEFT;
    }else if(key=='d' or key=='D'){
        mc.mode=MANUAL;
        mc.direction = RIGHT;
    }else if(key==' '){
        mc.mode=MANUAL;
        mc.direction = STOP;
    }
    
    return mc;
}

