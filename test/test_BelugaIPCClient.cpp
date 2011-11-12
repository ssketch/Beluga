#include "BelugaIPCClient.h"

const int ERROR = -1;
const int OK = 0;

#define START_TEST(x) std::cout << x << "... ";
#define RETURN_ERROR(x) { std::cerr << x << std::endl; return ERROR; }
#define RETURN_ERROR_ELSE_OK(x) RETURN_ERROR(x) else { std::cout << "OK" << std::endl; }

#define DO_TEST(desc, cond, err_msg) \
    START_TEST(desc); \
    if(cond) RETURN_ERROR_ELSE_OK(err_msg);


enum OP
{
    GET_POSITION = 0,
    SET_POSITION
};

double randomfloat()
{

    double f;
#ifdef _WIN32
    f = ((rand() % 1000) / 1000.0);
#else  
    f = ((rand() % 1000000) / 1000000.0);
#endif
    return 3*(f - 0.5);
}

std::vector<double> randomVector(unsigned int size)
{
    std::vector<double> r(size);

    for(unsigned int i = 0; i < size; i++)
    {
        r[i] = randomfloat();
    }
    return r;
}

bool eq_wf(double a, double b)
{
    char a_s[16];
    char b_s[16];

    sprintf(a_s, "%4.3f", a);
    sprintf(b_s, "%4.3f", b);

    return (strncmp(a_s, b_s, 16) == 0);
}

bool check_doubles_match(double a, double a_exp,
                         double b, double b_exp,
                         double c, double c_exp,
                         const char* a_label,
                         const char* b_label,
                         const char* c_label,
                         std::string* err_msg)
{
    std::ostringstream ss(*err_msg);
    if(!eq_wf(a, a_exp) || !eq_wf(b, b_exp) || !eq_wf(c, c_exp))
    {
        ss << std::endl << "Mismatch error: "
           << "\tExpected " << a_label << "to be " << a_exp << ", got " << a << std::endl
           << "\tExpected " << b_label << "to be " << b_exp << ", got " << b << std::endl
           << "\tExpected " << c_label << "to be " << c_exp << ", got " << c << std::endl;
        *err_msg = ss.str();
        return false;
    }
    return true;
}

bool check_vector_match(const std::vector<double>& a,
                        const std::vector<double>& a_exp,
                        const std::vector<double>& b,
                        const std::vector<double>& b_exp,
                        const std::vector<double>& c,
                        const std::vector<double>& c_exp,
                        const char* a_label,
                        const char* b_label,
                        const char* c_label,
                        std::string* err_msg)
{
    bool r = true;
    for(unsigned int i = 0; i < a.size(); i++)
    {
        r &= check_doubles_match(a[i], a_exp[i], b[i], b_exp[i], c[i], c_exp[i],
                                 a_label, b_label, c_label, err_msg);
    }
    return r;
}

bool check_single_position(belugaIPCClient* client,
                           OP op,
                           unsigned int robot,
                           double x_exp,
                           double y_exp,
                           double z_exp,
                           std::string* err_msg)
{
    double x = 42.0; double y = -1.0; double z = 1e3;

    bool r = true;
    switch(op)
    {
    case GET_POSITION:
        r = client->getPosition(robot, &x, &y, &z);
        break;
    case SET_POSITION:
        x = x_exp; y = y_exp; z = z_exp;        
        r = client->setPosition(robot, &x, &y, &z);
        break;
    default:
        std::cerr << "Unknown operation" << std::endl;
        return false;
    }
    
    if(!r)
    {
        *err_msg += "\nError getting/setting position from server.";
    }

    r &= check_doubles_match(x, x_exp, y, y_exp, z, z_exp,
                             "x position", "y position", "z position",
                             err_msg);

    return r;
}

bool check_multiple_positions(belugaIPCClient* client,
                              OP op,
                              std::vector<unsigned int> robots,
                              std::vector<double> x_exp,
                              std::vector<double> y_exp,
                              std::vector<double> z_exp,
                              std::string *err_msg)
{
    unsigned int num_bots = robots.size();
    std::vector<double> x = randomVector(num_bots);
    std::vector<double> y = randomVector(num_bots);
    std::vector<double> z = randomVector(num_bots);    

    bool r = true;
    switch(op)
    {
    case GET_POSITION:
        r = client->getPositions(robots, &x, &y, &z);
        break;
    case SET_POSITION:
        x = x_exp; y = y_exp; z = z_exp;
        r = client->setPositions(robots, &x, &y, &z);
        break;
    default:
        std::cerr << "Unknown operation" << std::endl;
        return false;
    }
        
    if(!r)
    {
        *err_msg += "\nError getting positions from server.";
    }

    check_vector_match(x, x_exp, y, y_exp, z, z_exp,
                       "x position", "y position", "z position",
                       err_msg);
    
    return r;
    
}

bool check_all_positions(belugaIPCClient* client,
                         OP op,
                         std::vector<double> x_exp,
                         std::vector<double> y_exp,
                         std::vector<double> z_exp,
                         std::string *err_msg)
{
    unsigned int num_bots = 4;

    std::vector<double> x = randomVector(4);
    std::vector<double> y = randomVector(4);
    std::vector<double> z = randomVector(4);   

    bool r = true;
    switch(op)
    {
    case GET_POSITION:
        r = client->getAllPositions(&x, &y, &z);
        break;
    case SET_POSITION:
        x = x_exp; y = y_exp; z = z_exp;
        r = client->setAllPositions(&x, &y, &z);
        break;
    default:
        std::cerr << "Unknown operation" << std::endl;
        return false;
    }
    
    if(!r)
    {
        *err_msg += "\nError getting positions from server.";
    }

    check_vector_match(x, x_exp, y, y_exp, z, z_exp,
                       "x position", "y position", "z position",
                       err_msg);
    
    return r;
    
}

int main(int argc, char** argv)
{
    belugaIPCClient client("127.0.0.1", 1234);

    std::string motd("");

    DO_TEST("Check connection", !client.doConnect(&motd), "Unable to connect to server.");
    
    std::string expected_motd("Welcome to BelugaServer, client ");

    DO_TEST("Check MOTD",
            expected_motd.compare(0, expected_motd.size(), motd, 0, expected_motd.size()) != 0,
            "Obtained motd \"" << motd << "\" does not match expected.");

    std::string expected_ping_response("PONG!");
    std::string ping_response = client.doMessage("ping");
    DO_TEST("Check ping response",
            expected_ping_response.compare(ping_response),
            "Unexpected ping response \"" << ping_response << "\".");

    std::string err_msg;
    DO_TEST("Checking get position 0",
            !check_single_position(&client, GET_POSITION, 0, 0.0, 0.0, 0.0, &err_msg),
            err_msg);

    DO_TEST("Checking get position 1",
            !check_single_position(&client, GET_POSITION, 1, 0.0, 0.0, 0.0, &err_msg),
            err_msg);

    DO_TEST("Checking get position 2",
            !check_single_position(&client, GET_POSITION, 2, 0.0, 0.0, 0.0, &err_msg),
            err_msg);

    DO_TEST("Checking get position 3",
            !check_single_position(&client, GET_POSITION, 3, 0.0, 0.0, 0.0, &err_msg),
            err_msg);


    std::vector<unsigned int> robots(4);
    std::vector<double> x, y, z;

    unsigned int bots3[] = {3};
    unsigned int bots201[] = {2, 0, 1};
    unsigned int bots12[] = {1, 2};
    unsigned int bots0123[] = {0, 1, 2, 3};

    double zeros1[] = {0};    
    double zeros2[] = {0, 0};
    double zeros3[] = {0, 0, 0};
    double zeros4[] = {0, 0, 0, 0};    

    robots.assign(bots12, bots12+2);
    x.assign(zeros2, zeros2+2);
    y.assign(zeros2, zeros2+2);
    z.assign(zeros2, zeros2+2);    
    
    DO_TEST("Checking get positions [1 2]",
            !check_multiple_positions(&client, GET_POSITION, robots, x, y, z, &err_msg),
            err_msg);

    robots.assign(bots3, bots3+1);
    x.assign(zeros1, zeros1 + 1);
    y.assign(zeros1, zeros1 + 1);
    z.assign(zeros1, zeros1 + 1);    
    
    DO_TEST("Checking get positions [3]",
            !check_multiple_positions(&client, GET_POSITION, robots, x, y, z, &err_msg),
            err_msg);

    robots.assign(bots201, bots201+3);
    x.assign(zeros3, zeros3 + 3);
    y.assign(zeros3, zeros3 + 3);
    z.assign(zeros3, zeros3 + 3);    
    
    DO_TEST("Checking get positions [2 0 1]",
            !check_multiple_positions(&client, GET_POSITION, robots, x, y, z, &err_msg),
            err_msg);

    robots.assign(bots0123, bots0123+4);
    x.assign(zeros4, zeros4 + 4);
    y.assign(zeros4, zeros4 + 4);
    z.assign(zeros4, zeros4 + 4);    
    
    DO_TEST("Checking get positions [0 1 2 3]",
            !check_multiple_positions(&client, GET_POSITION, robots, x, y, z, &err_msg),
            err_msg);

    x.assign(zeros4, zeros4 + 3);
    y.assign(zeros4, zeros4 + 3);
    z.assign(zeros4, zeros4 + 3);    
    
    DO_TEST("Checking get all positions",
            !check_all_positions(&client, GET_POSITION, x, y, z, &err_msg),
            err_msg);

    DO_TEST("Checking set position 0",
            !check_single_position(&client, SET_POSITION, 0, randomfloat(), randomfloat(), randomfloat(), &err_msg),
            err_msg);
    
    DO_TEST("Checking set position 1",
            !check_single_position(&client, SET_POSITION, 1, randomfloat(), randomfloat(), randomfloat(), &err_msg),
            err_msg);
    
    DO_TEST("Checking set position 2",
            !check_single_position(&client, SET_POSITION, 2, randomfloat(), randomfloat(), randomfloat(), &err_msg),
            err_msg);
    
    DO_TEST("Checking set position 3",
            !check_single_position(&client, SET_POSITION, 3, randomfloat(), randomfloat(), randomfloat(), &err_msg),
            err_msg);

    unsigned int bots03[] = {0, 3};
    unsigned int bots1[] = {1};
    unsigned int bots312[] = {3, 1, 2};

    robots.assign(bots03, bots03+2);
    x = randomVector(2);
    y = randomVector(2);
    z = randomVector(2);

    DO_TEST("Checking set positions [0 3]",
            !check_multiple_positions(&client, SET_POSITION, robots, x, y, z, &err_msg),
            err_msg);

    robots.assign(bots312, bots312+3);
    x = randomVector(3);
    y = randomVector(3);
    z = randomVector(3);

    DO_TEST("Checking set positions [3 1 2]",
            !check_multiple_positions(&client, SET_POSITION, robots, x, y, z, &err_msg),
            err_msg);

    robots.assign(bots1, bots1+1);
    x = randomVector(1);
    y = randomVector(1);
    z = randomVector(1);

    DO_TEST("Checking set positions [1]",
            !check_multiple_positions(&client, SET_POSITION, robots, x, y, z, &err_msg),
            err_msg);

    robots.assign(bots0123, bots0123+4);
    x = randomVector(4);
    y = randomVector(4);
    z = randomVector(4);

    DO_TEST("Checking set positions [0 1 2 3]",
            !check_multiple_positions(&client, SET_POSITION, robots, x, y, z, &err_msg),
            err_msg);

    x = randomVector(4);
    y = randomVector(4);
    z = randomVector(4);
    
    DO_TEST("Checking set all positions",
            !check_all_positions(&client, SET_POSITION, x, y, z, &err_msg),
            err_msg);
    
    
        std::cout << std::endl << "\tAll tests pass!" << std::endl;
    return OK;
}
