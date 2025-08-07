#ifndef SWERVE_CLASS_H
#define SWERVE_CLASS_H

#include"kinematicsVectors.h"

class Swerve{
public:
    // left is # of modules, 2 is the x/y vector for each
    Swerve(int rows = 4) : m_power{m_swervePos.size(), 2}{}
    Swerve(KVector swerve) : m_power{swerve}{}
    
    void normalize();
    void capPower();

    KVector rotateAroundCOM(double power);
    KVector rotateAroundPoint(std::vector<float> pos2d);
    KVector translatePower(std::vector<float> input);

    void kinematics(float x, float y, float r);

    std::vector<float> getModule(size_t module){return m_power.getRow(module);};
    static void inline setCOM(float x, float y){
        m_COM[0] = x;
        m_COM[1] = y;
    }

private:
    static inline std::vector<float> m_COM {0.5f, 0.5f};
    static inline std::vector<std::vector<float>> m_swervePos {
    {0.0f, 0.0f},
    {0.0f, 1.0f},
    {1.0f, 1.0f},
    {1.0f, 0.0f}
    };
    KVector m_power {4,2};

};
#endif
