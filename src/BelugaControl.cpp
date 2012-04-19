#include "BelugaControl.h"

#include <math.h>

std::string BelugaWaypointControlLaw::s_sName("Beluga Waypoint Controller\n");
std::string BelugaLowLevelControlLaw::s_sName("Beluga Low Level Controller\n");
std::string BelugaBoundaryControlLaw::s_sName("Beluga Boundary Controller\n");

BelugaWaypointControlLaw* belugaWaypointControlLawFactory(unsigned int bot_num,
                                                          unsigned int law_num)
{
    return new BelugaWaypointControlLaw();
}

BelugaLowLevelControlLaw* belugaLowLevelControlLawFactory(unsigned int bot_num,
                                                          unsigned int law_num)
{
    return new BelugaLowLevelControlLaw();
}

BelugaBoundaryControlLaw* belugaBoundaryControlLawFactory(unsigned int bot_num,
                                                          unsigned int law_num)
{
    return new BelugaBoundaryControlLaw();
}


BelugaWaypointControlLaw::BelugaWaypointControlLaw()  // includes HITL timing control (instead of having a separate ControlLaw)
    : mt_ControlLaw(3 /* # control inputs */,
                    4 /* # parameters */),
      m_bActive(false),
      m_dTiming(7500),	      // time for robot to travel between waypoints (msec), set in js and updated during IPC exchange
      m_dMaxSpeed(1.28),      // speed robot travels when more than 'm_dDistThreshold' away from waypoint (m/s), updated during IPC exchange
	  m_dDistThreshold(0.5),  // distance from waypoint at which robot starts to decrease speed from max (m)
      m_dTurningGain(10.0)
{
}

mt_dVector_t BelugaWaypointControlLaw::doControl(const mt_dVector_t& state,
                                                 const mt_dVector_t& u_in)
{
    if (!m_bActive || state.size() < BELUGA_NUM_STATES || u_in.size() < BELUGA_WAYPOINT_SIZE)
    {
        return u_in;
    }

    mt_dVector_t u(BELUGA_CONTROL_SIZE, 0.0);

    double x = state[BELUGA_STATE_X];
    double y = state[BELUGA_STATE_Y];
    double z = state[BELUGA_STATE_Z];
    double th = state[BELUGA_STATE_THETA];
    double omega = state[BELUGA_STATE_OMEGA];

    double to_x = u_in[BELUGA_WAYPOINT_X];
    double to_y = u_in[BELUGA_WAYPOINT_Y];
    double to_z = u_in[BELUGA_WAYPOINT_Z];

	if (to_x == BELUGA_WAYPOINT_NONE || to_y == BELUGA_WAYPOINT_NONE)
	{
		return u;
	}

    if (to_z BELUGA_MAINTAIN_Z)
    {
        to_z = z;
    }

    double dx = to_x - x;
    double dy = to_y - y;
    double dz = to_z - z;

    double dth = atan2(dy, dx) - th;
	dth = atan2(sin(dth), cos(dth));

    double d = sqrt(dx*dx + dy*dy);
    double u_speed = 0;
    double u_vert = 0;
    double u_turn = 0;
	
	if (0 /*d < m_dDist*/)
    {
        m_bActive = false;
    }
    else
    {
		if (d > m_dDistThreshold)
        {
            u_speed = m_dMaxSpeed;
        }
        else
        {
            u_speed = m_dMaxSpeed*(d/m_dDistThreshold)*fabs(cos(dth));
        }
        u_turn = -m_dTurningGain*sin(dth);
		if (dth > 1.57)
		{
			u_turn = -m_dTurningGain;
		}
		if (dth < -1.57)
		{
			u_turn = m_dTurningGain;
		}
    }

    u[BELUGA_CONTROL_FWD_SPEED] = u_speed;
    u[BELUGA_CONTROL_VERT_SPEED] = u_vert;
    u[BELUGA_CONTROL_STEERING] = u_turn;
    
	//printf("dx = %f, dy = %f, dth = %f, dz = %f\n", dx, dy, dth, dz);
	//printf("Control out: speed %f, vert %f, steer %f\n", u[BELUGA_CONTROL_FWD_SPEED], u[BELUGA_CONTROL_VERT_SPEED], u[BELUGA_CONTROL_STEERING]);

    return u;
}

BelugaLowLevelControlLaw::BelugaLowLevelControlLaw()
    : mt_ControlLaw(3 /* # control inputs */,
                    0 /* # parameters */),
      m_bActive(true)
{
}

mt_dVector_t BelugaLowLevelControlLaw::doControl(const mt_dVector_t& state,
                                                 const mt_dVector_t& u_in)
{
    if (!m_bActive || state.size() < 4 || u_in.size() < BELUGA_CONTROL_SIZE)
    {
        return u_in;
    }

    mt_dVector_t u(BELUGA_CONTROL_SIZE, 0.0);
    
    double u_speed = u_in[BELUGA_CONTROL_FWD_SPEED];
    double u_vert = u_in[BELUGA_CONTROL_VERT_SPEED];
    double u_turn = u_in[BELUGA_CONTROL_STEERING];

    double u_thrust = (K_d1/K_t)*u_speed;
	double speed_normalization_factor = 0.2;
	if (fabs(u_speed) > BELUGA_MIN_TURNING_SPEED)
	{
		speed_normalization_factor = u_speed;
	}
	else
	{
		if (u_speed < 0)
		{
			speed_normalization_factor *= -1.0;
		}
	}
    double u_steer = (K_omega/(K_d1*speed_normalization_factor*r_1))*u_turn;
    double u_vthrust = 0;

    if (u_vert != 0.0)
    {
        /* need z in the vertical thrust controller */
        double z = state[BELUGA_STATE_Z];
    
        /* calculating the vertical thrust requires finding the roots of
         * the polynomial u^2 + k_vp*u - (S/eta_x)*g, g defined here,
         * eta_x either eta_up or eta_down, and S either +1 (eta_up) or
         * -1 (eta_down).  One of the two combinations should give a positive
         * result - the corresponding thrust has this magnitude and is
         * negative if the eta_down (S = -1) solution is taken.  */
        double g = (k_d*u_vert*fabs(u_vert) - K_t*(z_off - z))*(fabs(u_vert) + v_off);

        /* determinants for polynomial */
        double D_up = k_vp*k_vp + 4*g/eta_up;
        double D_down = k_vp*k_vp - 4*g/eta_down;

        /* roots - note we'll throw out the -'ve root (i.e. (-b - sqrt(D))/2a */
        double r_up = -0.5*k_vp;  // just -b part
        double r_down = r_up;     // same

        /* can only calculate the determinant part if D > 0 */
        if (D_up > 0)
        {
            r_up += 0.25*sqrt(D_up);
        }
        if (D_down > 0)
        {
            r_down += 0.25*sqrt(D_down);
        }

        /* note we favor up control if both have solutions */
        if (D_down > 0)
        {
            u_vthrust = -r_down;
        }
        if (D_up > 0)
        {
            u_vthrust = r_up;
        }
    }

    u[BELUGA_CONTROL_FWD_SPEED] = u_thrust;
    u[BELUGA_CONTROL_VERT_SPEED] = u_vthrust;
    u[BELUGA_CONTROL_STEERING] = u_steer;

	//printf("Control out: thrust %f, v_thrust %f, turn %f\n", u[BELUGA_CONTROL_FWD_SPEED], u[BELUGA_CONTROL_VERT_SPEED], u[BELUGA_CONTROL_STEERING]);

	return u;
}

BelugaBoundaryControlLaw::BelugaBoundaryControlLaw()
	: mt_ControlLaw(3 /* # control inputs */,
					7 /* # parameters */),
	  m_bActive(true)
{
	/* map tank at depth z */
	for (unsigned int n = 0; n < size_R; n++)
	{
		R[n] = n*step;
	}
	for (unsigned int n = 0; n < size_TH; n++)
	{
		TH[n] = n*step;
	}
	
	/* create certainty matrix for tank */
	double boundary_length = DEFAULT_TANK_RADIUS/sqrt((double) 2.0);
	for (unsigned int n = 0; n < size_R; n++)
	{
		for (unsigned int m = 0; m < size_TH; m++)
		{
			/* calculate (x,y) from (R,TH) */
			double cx = R[n]*cos(TH[m]);
			double cy = R[n]*sin(TH[m]);
			/* is (x,y) outside of inscribed-square boundaries? */
			if (fabs(cx) > boundary_length || fabs(cy) > boundary_length)
			{
				/* soften boundaries */
				if (fabs(cx) < boundary_length)
					C[n][m] = Fcr*(fabs(cy)/DEFAULT_TANK_RADIUS);
				else
					C[n][m] = Fcr*(fabs(cx)/DEFAULT_TANK_RADIUS);
			}
			else
				C[n][m] = 0;
		}
	}
}

mt_dVector_t BelugaBoundaryControlLaw::doControl(const mt_dVector_t& state,
                                                 const mt_dVector_t& u_in)
{
    if (!m_bActive || state.size() < BELUGA_NUM_STATES || u_in.size() < BELUGA_CONTROL_SIZE)
    {
        return u_in;
    }
	
    mt_dVector_t u(BELUGA_CONTROL_SIZE, 0.0);
	
	double x = state[BELUGA_STATE_X];
    double y = state[BELUGA_STATE_Y];
	double th = state[BELUGA_STATE_THETA];
	
	double u_speed = u_in[BELUGA_CONTROL_FWD_SPEED];
    double u_vert = u_in[BELUGA_CONTROL_VERT_SPEED];
    double u_turn = u_in[BELUGA_CONTROL_STEERING];
	
	/* initialize net repulsive force (components in x and y) on the robot */
	double fx = 0;
	double fy = 0;
	
	/* calculate net repulsive force on the robot */
	for (unsigned int n = 0; n < size_R; n++)
	{
		for (unsigned int m = 0; m < size_TH; m++)
		{
			/* (x,y) of "active" cell in tank map */
			double cx = R[n]*cos(TH[m]);
			double cy = R[n]*sin(TH[m]);
			/* calculate Euclidean distance between "active" cell and robot */
			double d = sqrt((cx - x)*(cx - x) + (cy - y)*(cy - y));
			/* add to fx and fy */
			if (d != 0)
			{
				fx += C[n][m]*(cx - x)/(d*d);
				fy += C[n][m]*(cy - y)/(d*d);
			}
		}
	}
	
	/* calculate control parameters */
	double ax = fx/m_eff;
	double ay = fy/m_eff;
	double dt = BELUGA_MIN_COMMAND_PERIOD_MSEC;
	double dvx = ax*dt;
	double dvy = ay*dt;
	
	/* convert control parameters (dvx & dvy) to the robot-body coordinates (dvr & dvth) */
	double dvr = dvx*cos(th) - dvy*sin(th);
	double dvth = dvx*sin(th) + dvy*cos(th);
	
	/* add in control parameters */
	u_speed = u_speed + dvr;
	u_turn = u_turn + dvth;
	
	u[BELUGA_CONTROL_FWD_SPEED] = u_speed;
    u[BELUGA_CONTROL_VERT_SPEED] = u_vert;
    u[BELUGA_CONTROL_STEERING] = u_turn;
    
	//printf("Control out: speed %f, vert %f, steer %f\n", u[BELUGA_CONTROL_FWD_SPEED], u[BELUGA_CONTROL_VERT_SPEED], u[BELUGA_CONTROL_STEERING]);
	
    return u;
}
