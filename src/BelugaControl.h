#ifndef BELUGA_CONTROL_H
#define BELUGA_CONTROL_H

#include "BelugaConstants.h"
#include "Controller.h"

class BelugaWaypointControlLaw : public mt_ControlLaw
{
public:
    BelugaWaypointControlLaw();

    mt_dVector_t doControl(const mt_dVector_t& state,
                           const mt_dVector_t& u_in);
	
    bool doActivate(bool value = true){m_bActive = value;  return m_bActive;};
    
	double m_dTiming;
	double m_dDistThreshold;
    double m_dTurningGain;
    
protected:
    bool m_bActive;
    
    static std::string s_sName;
};

class BelugaLowLevelControlLaw : public mt_ControlLaw
{
public:
    BelugaLowLevelControlLaw();

    mt_dVector_t doControl(const mt_dVector_t& state,
                           const mt_dVector_t& u_in);

    bool doActivate(bool value = true){m_bActive = value;  return m_bActive;};

protected:
    bool m_bActive;
    
    static std::string s_sName;
};

class BelugaBoundaryControlLaw : public mt_ControlLaw
{
public:
    BelugaBoundaryControlLaw();
	
    mt_dVector_t doControl(const mt_dVector_t& state,
                           const mt_dVector_t& u_in);
	
    bool doActivate(bool value = true){m_bActive = value;  return m_bActive;};
	
	const double m_dFcr;
    
protected:
    bool m_bActive;
    
	const double step(0.05);
	const unsigned int size_R((DEFAULT_TANK_RADIUS/step) + 1);
	const unsigned int size_TH((PI/step) + 1);
	const double R[size_R];
	const double TH[size_TH];
	const double C[size_R][size_TH];
	
    static std::string s_sName;
};

BelugaWaypointControlLaw* belugaWaypointControlLawFactory(unsigned int bot_num,
                                                          unsigned int law_num);
BelugaLowLevelControlLaw* belugaLowLevelControlLawFactory(unsigned int bot_num,
                                                          unsigned int law_num);
BelugaBoundaryControlLaw* belugaBoundaryControlLawFactory(unsigned int bot_num,
                                                          unsigned int law_num);


#endif // BELUGA_CONTROL_H
