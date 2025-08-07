#include "swerveClass.h"
#include "kinematicsVectors.h"
#include <cmath>

void Swerve::normalize(){
    m_power /= m_power.maxMagnitude();
}

//if a max vector is over 1, don't do that
void Swerve::capPower(){
    double maxMagnitude {m_power.maxMagnitude()};
    if(maxMagnitude > 1){
        m_power / maxMagnitude;
    }
}

// please not this uses an = sign
// There is only one way to rotate around a point, so the = sign is used
KVector Swerve::rotateAroundCOM(double power = 1){
    std::vector<std::vector<float>> temp (m_power.getRows(), std::vector<float>(2));
    for (int i = 0; i < m_power.getRows(); ++i){
        temp[i][0] = -(m_swervePos[i][1]-m_COM[1]) * power;
        temp[i][1] = (m_swervePos[i][0]-m_COM[0]) * power;
    }
    m_power += temp;
    return m_power;
}

//sets the rotation to clear any current power info, this is cause it's what you gotta do.
KVector Swerve::rotateAroundPoint(std::vector<float> pos2d){
    m_power.setConstant(0);
    rotateAroundCOM;
    double scale {m_power.avgMagnitude()};
    std::vector<std::vector<float>> temp (m_power.getRows(), std::vector<float>(2));
    for (int i = 0; i < m_power.getRows(); ++i){
        temp[i][0] = m_swervePos[i][1]-pos2d[1];
        temp[i][1] = -(m_swervePos[i][0]-pos2d[0]);
    }
    KVector tempV {temp};
    m_power *= scale / tempV.avgMagnitude();
    m_power += tempV;
    return m_power;
}


KVector Swerve::translatePower(std::vector<float> input){
    KVector temp(m_power.getColumns(), m_power.getRows());
    m_power += temp.setConstant(1) * input;
    return m_power;
}

void Swerve::kinematics(float x, float y, float r) {
  Swerve result;

  result.translatePower({x, y});
  result.rotateAroundCOM(r);
  result.capPower();
}
